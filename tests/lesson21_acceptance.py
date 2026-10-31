#!/usr/bin/env python3
"""Check real clock domains, original events, isolated restarts and cancellation."""
import argparse
from bisect import bisect_left
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import actual_pose
from tests.lesson06_acceptance import actual_stats

TYPES = {"camera": "ImageStamped", "imu": "IMU", "world_stats": "WorldStatistics",
         "performance_metrics": "PerformanceMetrics", "metadata": "Any"}


def require(value, message):
    if not value:
        raise AssertionError(message)


def close(actual, expected, message):
    require(math.isfinite(actual) and math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9), message)


def ns(value):
    return int(value["sec"])*1000000000+int(value["nsec"])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hashes(folder):
    return {p.name: sha(p) for p in sorted(folder.iterdir()) if p.is_file()}


def configure(lesson, workspace, camera_hz=None, step_ms=None):
    spec = importlib.util.spec_from_file_location("l21_independent_config", lesson.folder/"config.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    previous = Path.cwd()
    try:
        os.chdir(str(workspace))
        module.edit(camera_hz=camera_hz, step_ms=step_ms)
    finally:
        os.chdir(str(previous))


class Assets:
    """Read original bytes before temp deletion, then retain short portable paths."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.cache, self.references = {}, {}

    def image(self, row):
        image, storage = row["native"]["image"], row["binary_storage"]
        require("data" not in image and storage["removed_field"] == "native.image.data",
                "Camera data was silently replaced instead of explicitly externalized")
        require((image["width"], image["height"], image["pixel_format"], image["step"]) == (128, 96, 3, 384),
                "Actual native image layout is not RGB_INT8 128x96")
        original = storage["path"]
        if original not in self.cache:
            data = Path(original).read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            require(len(data) == 36864, "Actual camera bytes do not match step*height")
            target = self.directory/(digest+".rgb")
            if not target.exists(): target.write_bytes(data)
            require(sha(target) == digest, "Retained original image bytes differ")
            center = list(data[48*384+64*3:48*384+64*3+3])
            self.cache[original] = {"sha256": digest, "byte_count": len(data), "center_rgb": center,
                                    "distinct_values": len(set(data)), "retained_path": str(target.resolve())}
        evidence = self.cache[original]
        require(storage["sha256"] == evidence["sha256"] and storage["byte_count"] == evidence["byte_count"],
                "Original image asset reference hash/length is wrong")
        self.references[original] = evidence
        return evidence

    def save(self, target):
        Path(target).write_text(json.dumps(self.references, indent=2), encoding="utf-8")


def inspect_producer(rows, window, camera_hz, mode):
    snapshots = []
    for row in rows:
        if row["source"] != "metadata": continue
        value = row["server_state"]["camera"].get("producer_update", {})
        if value.get("hook_attached") and int(value.get("event_count", 0)) > 0:
            snapshots.append((row, value))
    require(len(snapshots) >= 2, "No successful producer update snapshots")
    by_count = {}
    for row, value in snapshots:
        count = int(value["event_count"])
        if count in by_count:
            require(by_count[count][1] == value, "Same producer count has inconsistent non-coherent snapshot fields")
        else:
            by_count[count] = (row, value)
        require(int(value["producer_last_callback_monotonic_ns"]) <= row["observer_receive_monotonic_ns"],
                "Producer callback occurred after receipt of its snapshot")
    counts = sorted(by_count)
    require(len(counts) >= 2, "Producer counter never advanced")
    a_row, a = by_count[counts[0]]
    b_row, b = by_count[counts[-1]]
    elapsed = (int(b["last_measurement_time_ns"])-int(a["last_measurement_time_ns"]))/1e9
    delta = counts[-1]-counts[0]
    require(elapsed >= 1 and delta > 0, "Producer measurement interval is too short")
    rate = delta/elapsed  # Counters are interval endpoints, so there is no N-1.
    same = int(b["same_stamp_count"])-int(a["same_stamp_count"])
    regressed = int(b["regressed_stamp_count"])-int(a["regressed_stamp_count"])
    require(same >= 0 and regressed >= 0, "Producer diagnostics counter moved backward")
    if mode:
        require(abs(rate/camera_hz-1) <= .02, "Actual successful camera update rate differs under lockstep")
        require(same == 0 and regressed == 0, "Strict producer updates repeated/regressed their measurement timestamp")
    declared = window["producer_updates"]["camera"]
    close(declared["successful_update_rate_hz"], rate, "Producer rate uses a wrong interval/count")
    close(declared["measurement_span_s"], elapsed, "Producer measurement duration differs")
    require(declared["event_count_delta"] == delta and declared["same_stamp_delta"] == same
            and declared["regressed_stamp_delta"] == regressed, "Producer counter deltas differ from actual metadata")
    require(declared["snapshot_count"] == len(snapshots) and declared["unique_counter_count"] == len(counts),
            "Producer raw/unique snapshot count differs")
    for key, count in (("first", counts[0]), ("last", counts[-1])):
        endpoint = declared[key]
        candidates = [(row, value) for row, value in snapshots if row["observer_sequence"] == endpoint["observer_sequence"]]
        require(len(candidates) == 1 and int(candidates[0][1]["event_count"]) == count
                and endpoint["producer_update"] == candidates[0][1], "Producer endpoint does not identify an original metadata event")
    return {"actual_event_delta": delta, "measurement_span_s": elapsed,
            "successful_update_rate_hz": rate, "same_stamp_delta": same,
            "regressed_stamp_delta": regressed, "snapshot_count": len(snapshots)}


def inspect_window(report, phase_name, window, camera_hz, step_s, mode, ros, assets):
    rows = [row for row in report["trace"] if row["phase"] == phase_name]
    require(rows, "No original native events in "+phase_name)
    sequences = [row["observer_sequence"] for row in rows]
    require(all(b > a for a, b in zip(sequences, sequences[1:])), "Original observer output order was altered")
    stream_ids = set()
    for row in rows:
        source, native = row["source"], row["native"]
        require(row["native_type"] == "gazebo.msgs."+TYPES[source], "Wrong actual native message class")
        require(row["python_receive_monotonic_ns"] >= row["observer_receive_monotonic_ns"], "Observer/Python host clocks disagree")
        if source in ("camera", "imu", "world_stats"):
            field = "stamp" if source == "imu" else "sim_time" if source == "world_stats" else "time"
            require(row["source_stamp_ns"] == ns(native[field]), "Original native timestamp was changed")
            expected_basis = "world_statistics_sim_time" if source == "world_stats" else "native_sensor_stamp"
            require(row["time_basis"] == expected_basis, "A native clock domain is mislabeled")
        else:
            require(row["source_stamp_ns"] is None and row["time_basis"] == "no_native_stamp",
                    "Diagnostic message was assigned a fabricated sample timestamp")
        if source == "camera": assets.image(row)
        if source == "metadata":
            actual = json.loads(native["string_value"])
            require(actual == row["server_state"], "Actual server metadata was rewritten")
            require(actual["rendering_lockstep_enabled"] is mode and actual["world"] == "lesson21", "Actual startup mode differs")
            close(actual["max_step_size_s"], step_s, "Actual physics step differs")
            close(actual["real_time_update_rate_hz"], 1/step_s if ros else 0, "Actual rate cap differs")
            for sensor, rate in (("camera", camera_hz), ("imu", 100)):
                require(actual[sensor]["strict_rate"] is mode, "Actual sensor strict mode differs")
                close(actual[sensor]["update_rate_hz"], rate, "Actual sensor configured rate differs")
        if ros:
            stream_ids.add(row["stream_id"])
            require(row["observer_receive_monotonic_ns"] <= row["python_receive_monotonic_ns"]
                    <= row["ros_publish_monotonic_ns"] <= row["ros_receive_monotonic_ns"],
                    "Actual ROS loopback timing is inconsistent")
    if ros: require(len(stream_ids) == 1, "More than one ROS stream was mixed into a measurement")
    channels, representatives = {}, {}
    for source in ("camera", "imu"):
        received = [row for row in rows if row["source"] == source]
        stamps = [row["source_stamp_ns"] for row in received]
        first = {}
        for row in received: first.setdefault(row["source_stamp_ns"], row)
        unique = sorted(first)
        require(len(unique) >= (8 if source == "camera" else 20), "Too few actual samples")
        span = (unique[-1]-unique[0])/1e9
        require(span >= (2 if ros else 1), "Actual native tag coverage is too short")
        density = (len(unique)-1)/span
        if ros:
            accepted = [r["source_stamp_ns"] for r in received if r["disposition"] == "accepted"]
            require(all(b > a for a, b in zip(accepted, accepted[1:])), "Main accepted samples are not monotonic")
            target = camera_hz if source == "camera" else 100
            require(abs(density/target-1) <= (.02 if source == "camera" else .05), "Main actual sample rate differs")
        tick_ns = round(step_s*1e9)
        require(all((b-a) % tick_ns == 0 for a, b in zip(unique, unique[1:])), "Native label gaps are outside the physics grid")
        regression = [a-b for a, b in zip(stamps, stamps[1:]) if b < a]
        summary = window["channels"][source]
        require(summary["stamps_ns"] == unique, "Statistical label index differs from original observed unique labels")
        require(summary["received_count"] == len(received) and summary["unique_stamp_count"] == len(unique)
                and summary["duplicate_stamp_count"] == len(received)-len(unique), "Raw/unique/duplicate counts differ")
        require(summary["arrival_stamp_regressions"]["count"] == len(regression)
                and summary["arrival_stamp_regressions"]["max_backstep_ns"] == max(regression, default=0),
                "Arrival timestamp regressions were hidden or miscounted")
        close(summary["stamp_span_s"], span, "Native coverage span differs")
        close(summary["sim_rate_hz"], density, "Observed unique-label coverage density differs")
        require(summary["sim_rate_interpretation"] == "observed_unique_time_label_coverage_density", "Coverage density is overstated as producer rate")
        receipt = [row["observer_receive_monotonic_ns"] for row in received]
        receipt_span = (max(receipt)-min(receipt))/1e9
        unique_receipt = [row["observer_receive_monotonic_ns"] for row in first.values()]
        unique_span = (max(unique_receipt)-min(unique_receipt))/1e9
        close(summary["observer_wall_rate_hz"], (len(received)-1)/receipt_span, "Raw observer wall frequency differs")
        close(summary["unique_stamp_observer_wall_rate_hz"], (len(unique)-1)/unique_span, "Unique observer wall frequency differs")
        if ros:
            arrivals = [row["ros_receive_monotonic_ns"] for row in received]
            close(summary["ros_wall_rate_hz"], (len(received)-1)/((max(arrivals)-min(arrivals))/1e9), "ROS arrival frequency differs")
        representatives[source] = first
        channels[source] = {"received": len(received), "unique": len(unique), "duplicate": len(received)-len(unique),
                            "arrival_regressions": len(regression), "max_backstep_ns": max(regression, default=0),
                            "label_span_s": span, "unique_label_density_hz": density,
                            "observer_wall_hz": (len(received)-1)/receipt_span}
    imu = sorted(representatives["imu"])
    pairs = window["nearest_neighbors"]["pairs"]
    require(pairs, "No actual common native timestamp interval")
    for pair in pairs:
        camera_stamp, imu_stamp = pair["camera_stamp_ns"], pair["imu_stamp_ns"]
        require(camera_stamp in representatives["camera"] and imu_stamp in representatives["imu"], "Nearest pair invented a timestamp")
        require(pair["camera_observer_sequence"] == representatives["camera"][camera_stamp]["observer_sequence"]
                and pair["imu_observer_sequence"] == representatives["imu"][imu_stamp]["observer_sequence"], "Pair does not identify the actual first event")
        position = bisect_left(imu, camera_stamp)
        candidates = imu[max(0, position-1):position+1]
        require(abs(imu_stamp-camera_stamp) == min(abs(v-camera_stamp) for v in candidates), "Pair is not actually nearest")
        require(pair["signed_skew_ns"] == imu_stamp-camera_stamp, "Pair skew changed original timestamps")
        if ros: require(abs(pair["signed_skew_ns"]) <= 10000000+2*round(step_s*1e9), "Main nearest timing is outside its bound")
    stats = window["world_statistics"]
    require(len(stats) >= 2, "No independent native world statistics window")
    sim = ns(stats[-1]["native"]["sim_time"])-ns(stats[0]["native"]["sim_time"])
    real = ns(stats[-1]["native"]["real_time"])-ns(stats[0]["native"]["real_time"])
    require(real > 0 and sim > 0, "Continuous world statistics did not advance")
    close(window["world_statistics_rtf"], sim/real, "World RTF uses inconsistent time intervals")
    sample = assets.image(next(row for row in reversed(rows) if row["source"] == "camera"))
    red, green, blue = sample["center_rgb"]
    require(red > 150 and green < 60 and blue < 60 and sample["distinct_values"] > 3,
            "Actual camera image does not show the rendered red target")
    return {"channels": channels, "nearest_pairs": len(pairs), "world_statistics_rtf": sim/real,
            "actual_image": sample, "producer_updates": inspect_producer(rows, window, camera_hz, mode)}


def clocks(snapshot):
    value = snapshot["world_statistics"]["native"]
    require(snapshot["ros_service_paused"] is value["paused"], "ROS service/native pause disagreement")
    require(snapshot["ros_service_world_sim_time_ns"] == ns(value["sim_time"]), "ROS service/native simulation time disagreement")
    return ns(value["sim_time"]), int(value["iterations"]), ns(value["real_time"]), value["paused"]


def inspect_raw(report, camera_hz, step_s, assets, artifacts, label, workspace):
    require(set(report["phases"]) == {"continuous", "pause_hold", "single_step", "multi_step"}, "Missing actual clock phase")
    checked = {"continuous": inspect_window(report, "continuous", report["phases"]["continuous"], camera_hz, step_s, True, True, assets)}
    hold = report["phases"]["pause_hold"]
    before, after = clocks(hold["before"]), clocks(hold["after"])
    require(before == after and before[3] is True, "Paused native simulation/iterations/real-time advanced")
    mono_delta = (hold["after"]["world_statistics"]["observer_receive_monotonic_ns"]-hold["before"]["world_statistics"]["observer_receive_monotonic_ns"])/1e9
    require(hold["wall_span_s"] >= .35 and mono_delta >= .35, "Observer wall clock did not include the pause")
    checked["pause_hold"] = {"observer_wall_span_s": mono_delta, "native_clock_snapshot": before}
    for name, count in (("single_step", 1), ("multi_step", round(.1/step_s))):
        phase = report["phases"][name]
        a, b = clocks(phase["before"]), clocks(phase["after"])
        require(phase["step_count"] == count and b[1]-a[1] == count and b[0]-a[0] == round(count*step_s*1e9)
                and a[3] is True and b[3] is True, "Actual single/multi step does not match exact physics ticks")
        checked[name] = {"actual_steps": b[1]-a[1], "actual_sim_delta_ns": b[0]-a[0]}
    children = report["children"]
    require([c["name"] for c in children] == ["lockstep_off", "lockstep_on"], "Missing isolated complete restart pair")
    require(children[0]["benchmark_world_sha256"] == children[1]["benchmark_world_sha256"], "Restart comparison used different worlds")
    require(children[0]["gazebo_master_uri"] != children[1]["gazebo_master_uri"]
            and children[0]["ign_partition"] != children[1]["ign_partition"], "Child native transports were not isolated")
    require([a for a in children[1]["argv"] if a != "--lockstep"] == children[0]["argv"], "More than the lockstep flag differs between child starts")
    for child in children:
        name = child["name"]
        require(child["server_pgid"] == child["observer_pgid"] == child["caller_pgid"]
                and child["inherits_session_token"] is True, "Child escaped caller ownership")
        require(not child["cleanup_errors"] and child["server_cleanup"]["exited"] is True
                and child["observer_cleanup"]["exited"] is True, "Owned child was not reaped")
        require(not Path("/proc", str(child["pid"])).exists() and not Path("/proc", str(child["observer_pid"])).exists(), "Exited child PID still exists")
        benchmark = Path(child["benchmark_world"])
        require(sha(benchmark) == child["benchmark_world_sha256"], "Actual benchmark input fingerprint differs")
        original, copied = ET.parse(str(workspace/"world.sdf")).getroot(), ET.parse(str(benchmark)).getroot()
        copied.find("world/physics/real_time_update_rate").text = original.findtext("world/physics/real_time_update_rate")
        def canonical(node):
            return node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(), tuple(canonical(c) for c in node)
        require(canonical(original) == canonical(copied), "Benchmark changed more than explicit shared rate cap")
        checked[name] = inspect_window(report, "comparison_"+name, child["window"], camera_hz, step_s, child["lockstep"], False, assets)
        for filename in ("gzserver.log", "observer.stderr.log"):
            source = benchmark.parent/name/filename
            if source.exists(): shutil.copy2(str(source), str(artifacts/(label+"-"+name+"-"+filename)))
    return checked


def verify_case(session, artifacts, assets, label, camera_hz, step_s, expected=True, diagnosis=None):
    before, pose_before = actual_stats(session), actual_pose(session, "sync_rig")
    world, report_path = session.workspace/"world.sdf", artifacts/(label+".json")
    original = world.read_bytes()
    completed = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(report_path)],
                               cwd=str(session.workspace), env=session.env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=200,
                               start_new_session=True)
    report_path.with_suffix(".stdout.log").write_text(completed.stdout, encoding="utf-8")
    observation = json.loads(report_path.read_text(encoding="utf-8"))
    # Preserve evidence before any status assertion can abort this fresh fixture.
    observer_folders = set()
    for row in observation.get("trace", []):
        if row.get("source") == "camera":
            assets.image(row)
            observer_folders.add(Path(row["binary_storage"]["path"]).parent)
    for index, folder in enumerate(sorted(observer_folders)):
        source_log = folder/"observer.stderr.log"
        if source_log.is_file(): shutil.copy2(str(source_log), str(artifacts/(label+"-observer-"+str(index)+".stderr.log")))
    for child in observation.get("children", []):
        benchmark = Path(child["benchmark_world"])
        if benchmark.is_file(): shutil.copy2(str(benchmark), str(artifacts/(label+"-benchmark.sdf")))
        for filename in ("gzserver.log", "observer.stderr.log"):
            source_log = benchmark.parent/child["name"]/filename
            if source_log.is_file(): shutil.copy2(str(source_log), str(artifacts/(label+"-"+child["name"]+"-"+filename)))
    require((completed.returncode == 0) is expected, "Unexpected verifier result "+label+": "+completed.stdout[-3000:])
    require(observation.get("passed") is expected, "Process/report status differs")
    if diagnosis:
        require(diagnosis.lower() in (completed.stdout+json.dumps(observation)).lower(), "Missing specific diagnosis: "+diagnosis)
    checked = inspect_raw(observation, camera_hz, step_s, assets, artifacts, label, session.workspace) if expected else None
    after, pose_after = actual_stats(session), actual_pose(session, "sync_rig")
    require(before["paused"] == after["paused"], "Original actual pause state was not restored")
    require(max(abs(a-b) for a, b in zip(pose_before, pose_after)) < 1e-6, "Timing experiment moved the fixed observation rig")
    require(world.read_bytes() == original, "Verifier overwrote learner inputs")
    if expected:
        require(observation["restore"]["pause_before"] == before["paused"] == observation["restore"]["pause_after"], "Declared pause restoration differs")
        require(observation["restore"]["observer_cleanup"]["exited"] is True, "Main native observer was not reaped")
    elif before["paused"]:
        require(before["iterations"] == after["iterations"], "Rejected input advanced actual physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected, "raw_report": str(report_path),
            "independent_raw_checks": checked, "stats_before": before, "stats_after": after,
            "pose_before": pose_before, "pose_after": pose_after}


def owned_processes(session):
    """Inspect only exact current-token PIDs; never enumerate external commands."""
    found = []
    for pid in live_token_processes(session.token):
        path = Path("/proc", str(pid))
        try:
            argv = (path/"cmdline").read_bytes().split(b"\0")
            argv = [part.decode(errors="replace") for part in argv if part]
            env = dict(part.split(b"=", 1) for part in (path/"environ").read_bytes().split(b"\0") if b"=" in part)
            start_time = (path/"stat").read_text().rsplit(") ", 1)[1].split()[19]
            found.append({"pid": pid, "argv": argv,
                          "start_time": start_time,
                          "master": env.get(b"GAZEBO_MASTER_URI", b"").decode(),
                          "partition": env.get(b"IGN_PARTITION", b"").decode()})
        except (OSError, ValueError):
            pass
    return found


def signal_exact(session, observed, number, missing_ok=False):
    current = next((p for p in owned_processes(session) if p["pid"] == observed["pid"]), None)
    if current is None and missing_ok: return
    require(current is not None and current["start_time"] == observed["start_time"]
            and current["master"] == observed["master"] and current["argv"] == observed["argv"],
            "Owned child identity changed before signal")
    os.kill(observed["pid"], number)


def wait_for_child_observer(session, caller, timeout=30):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        require(caller.poll() is None, "Comparison caller exited before its child became active")
        processes = owned_processes(session)
        servers = [p for p in processes if any(arg.endswith("benchmark.sdf") for arg in p["argv"])
                   and p["argv"] and Path(p["argv"][0]).name == "gzserver"]
        for server in servers:
            observers = [p for p in processes if p["argv"] and Path(p["argv"][0]).name == "native_observer"
                         and p["master"] == server["master"]]
            if observers:
                return server, observers[0]
        time.sleep(.02)
    raise AssertionError("No exact-token isolated child observer became active")


def stop_caller(process):
    if process.poll() is None:
        process.terminate()
        try: process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def interrupt_cleanup(session, artifacts):
    """Signal during actual owned-observer cleanup, then verify no on restart."""
    initial = set(live_token_processes(session.token))
    before = actual_stats(session)
    output = artifacts/"cancel_during_cleanup.json"
    caller, suspended, observer = None, None, None
    log_path = artifacts/"cancel_during_cleanup.stdout.log"
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8") as log:
        try:
            caller = subprocess.Popen([sys.executable, str(session.lesson.folder/"lab.py"), "compare", "--report", str(output)],
                                      cwd=str(session.workspace), env=session.env, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            server, observer = wait_for_child_observer(session, caller)
            # Wait until this exact observer has actually written camera-sized
            # data, then let five-stream readiness settle before suspending it.
            # The comparison's two-wall-second minimum is still pending.
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                require(caller.poll() is None, "Comparison finished before interruption setup")
                values = Path("/proc", str(observer["pid"]), "io").read_text().splitlines()
                written = next(int(line.split()[1]) for line in values if line.startswith("wchar:"))
                if written >= 1000000: break
                time.sleep(.01)
            else:
                raise AssertionError("Owned observer did not write actual camera data before cancellation")
            time.sleep(.45)
            require(caller.poll() is None, "Comparison finished before interruption setup")
            suspended = observer["pid"]
            signal_exact(session, observer, signal.SIGSTOP)
            deadline, pending = time.monotonic()+8, False
            while time.monotonic() < deadline:
                require(caller.poll() is None, "Comparison exited before cleanup signal was pending")
                status = Path("/proc", str(suspended), "status").read_text()
                masks = [int(line.split()[1], 16) for line in status.splitlines() if line.startswith(("SigPnd:", "ShdPnd:"))]
                if any(mask & (1 << (signal.SIGINT-1)) for mask in masks):
                    pending = True
                    break
                time.sleep(.01)
            require(pending, "Did not observe actual stop_owned SIGINT pending during suspended observer cleanup")
            caller.send_signal(signal.SIGTERM)
            time.sleep(.03)
            signal_exact(session, observer, signal.SIGCONT)
            suspended = None
            caller.wait(timeout=20)
            require(caller.returncode != 0 and output.is_file(), "Cancellation did not produce a failed finite report")
            observation = json.loads(output.read_text(encoding="utf-8"))
            require(not observation["passed"] and "cancel" in observation.get("error", "").lower(), "Cancellation was not diagnosed")
            require(len(observation["children"]) == 1 and observation["children"][0]["name"] == "lockstep_off",
                    "Cancellation during off cleanup still launched on experiment")
            child = observation["children"][0]
            require(child["server_cleanup"]["exited"] is True and child["observer_cleanup"]["exited"] is True
                    and not child["cleanup_errors"], "Cancelled child cleanup did not reap both owned PIDs")
            after = actual_stats(session)
            require(before["paused"] == after["paused"] == observation["pause_after"], "Cancellation did not restore parent pause")
            require(set(live_token_processes(session.token)) == initial, "Cancellation left extra token processes")
            return {"name": "signal_during_owned_cleanup", "passed": True, "caller_pid": caller.pid,
                    "observed_server": server, "observed_observer": observer, "actual_pending_sigint": True,
                    "raw_report": str(output), "elapsed_seconds": time.monotonic()-started}
        except Exception as error:
            diagnostic = {"error": repr(error), "elapsed_seconds": time.monotonic()-started,
                          "caller_pid": caller.pid if caller is not None else None,
                          "caller_returncode": caller.poll() if caller is not None else None,
                          "suspended_pid": suspended, "owned_processes": owned_processes(session)}
            for process in diagnostic["owned_processes"]:
                try:
                    lines = Path("/proc", str(process["pid"]), "status").read_text().splitlines()
                    process["status"] = [line for line in lines if line.startswith(("State:", "PPid:", "SigPnd:", "ShdPnd:"))]
                except OSError:
                    pass
            (artifacts/"cancel_during_cleanup-diagnostic.json").write_text(json.dumps(diagnostic, indent=2), encoding="utf-8")
            raise
        finally:
            if suspended is not None:
                try: signal_exact(session, observer, signal.SIGCONT, missing_ok=True)
                except ProcessLookupError: pass
            if caller is not None: stop_caller(caller)


def stop_session_during_child(session, artifacts):
    output = artifacts/"stop_session_during_child.json"
    log_path = artifacts/"stop_session_during_child.stdout.log"
    caller = None
    started = time.monotonic()
    with log_path.open("w", encoding="utf-8") as log:
        try:
            caller = subprocess.Popen([sys.executable, str(session.lesson.folder/"lab.py"), "compare", "--report", str(output)],
                                      cwd=str(session.workspace), env=session.env, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            server, observer = wait_for_child_observer(session, caller)
            observed = live_token_processes(session.token)
            session.stop(reason="L21 real cancellation while isolated child is active")
            caller.wait(timeout=12)
            require(not live_token_processes(session.token), "Application shutdown left lesson/child processes")
            return {"name": "application_stop_during_isolated_child", "passed": True,
                    "caller_pid": caller.pid, "caller_returncode": caller.returncode,
                    "observed_server": server, "observed_observer": observer,
                    "owned_before_stop": observed, "leftovers": [], "elapsed_seconds": time.monotonic()-started}
        finally:
            if caller is not None: stop_caller(caller)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    assets = Assets(artifacts/"assets")
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == "classic11-noetic:21"]
    require(len(found) == 1, "Expected L21: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source,
              "scenarios": [], "cancellation_checks": []}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l21-") as temporary:
        try:
            for label, camera_hz, step_ms, paused in (("baseline", 20, 1, True), ("camera_rate_only", 10, 1, False), ("physics_step_only", 20, 2, True)):
                storage = Storage(Path(temporary)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline": configure(lesson, workspace, camera_hz, step_ms)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, assets, label, camera_hz, step_ms/1000))
                if label == "physics_step_only":
                    world, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        configure(lesson, workspace, camera_hz=10)
                        report["scenarios"].append(verify_case(session, artifacts, assets, "saved_without_reload", camera_hz, step_ms/1000, False, "reload"))
                    finally: world.write_bytes(saved)
                    for label_bad, xpath, value, diagnosis in (
                        ("wrong_imu_rate", "./world/model[@name='sync_rig']/link/sensor[@name='imu']/update_rate", "50", "IMU"),
                        ("wrong_camera_format", "./world/model[@name='sync_rig']/link/sensor[@name='camera']/camera/image/format", "L8", "format"),
                        ("wrong_main_rate_cap", "./world/physics/real_time_update_rate", "0", "real_time_update_rate")):
                        try:
                            tree = ET.fromstring(saved)
                            node = tree.find(xpath)
                            require(node is not None, "Missing fault injection field")
                            node.text = value
                            world.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, assets, label_bad, camera_hz, step_ms/1000, False, diagnosis))
                        finally: world.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, assets, "exact_source_repaired", camera_hz, step_ms/1000))
                    report["cancellation_checks"].append(interrupt_cleanup(session, artifacts))
                    report["cancellation_checks"].append(stop_session_during_child(session, artifacts))
                session.stop(reason="L21 physical scenario completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L21 final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error: report["cleanup_errors"].append(label+": "+repr(error))
            assets.save(artifacts/"asset-reference-map.json")
            report["retained_asset_map"] = str(artifacts/"asset-reference-map.json")
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("scenarios", "cancellation_checks")}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__": sys.exit(main())
