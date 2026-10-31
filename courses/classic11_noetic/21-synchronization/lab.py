#!/usr/bin/env python3
"""Read native sensors, optionally forward actual rows through ROS, own only our children."""
import argparse
import base64
from bisect import bisect_left
from collections import deque
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import uuid
from config import prepared, edit, digest, replace_scalar

SOURCES = ("camera", "imu", "world_stats", "performance_metrics", "metadata")
_cleanup_depth = 0
_cancelled = threading.Event()


def check_cancelled():
    require(not _cancelled.is_set(), "Experiment cancelled; no further child may be started")


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def ns(value):
    return int(value["sec"])*1000000000+int(value["nsec"])


@contextmanager
def cancellation():
    original, interrupted = {}, [False]
    _cancelled.clear()
    def stop(number, frame):
        if not interrupted[0]:
            interrupted[0] = True
            _cancelled.set()
            if not _cleanup_depth:
                raise RuntimeError("Experiment cancelled by signal "+str(number))
    try:
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            original[number] = signal.signal(number, stop)
        yield
        require(not interrupted[0], "Experiment cancelled during owned-process cleanup")
    finally:
        for number, handler in original.items():
            signal.signal(number, handler)


def stop_owned(process):
    """No process group kill: this is the exact Popen object we created."""
    if process is None:
        return None
    global _cleanup_depth
    _cleanup_depth += 1
    try:
        attempted = []
        for name, action, timeout in (("SIGINT", lambda: process.send_signal(signal.SIGINT), 2),
                                      ("SIGTERM", process.terminate, 2), ("SIGKILL", process.kill, 2)):
            if process.poll() is not None:
                break
            attempted.append(name)
            try:
                action()
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
            except ProcessLookupError:
                pass
        require(process.poll() is not None, "Owned child did not exit: "+str(process.pid))
        return {"pid": process.pid, "returncode": process.returncode, "signals": attempted, "exited": True}
    finally:
        _cleanup_depth -= 1


class Observer:
    def __init__(self, record, directory, env=None, callback=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.env = os.environ.copy() if env is None else env.copy()
        self.lock = threading.Lock()
        self.rows, self.latest, self.errors = deque(), {}, []
        self.callback = callback
        self.image_cache = None
        self.log = (self.directory/"observer.stderr.log").open("w", encoding="utf-8")
        self.process = subprocess.Popen([record["observer"]], env=self.env, cwd=str(Path.cwd()),
            stdout=subprocess.PIPE, stderr=self.log, text=True, encoding="utf-8", bufsize=1)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        self.cleanup = None

    def read(self):
        try:
            for line in self.process.stdout:
                received = time.monotonic_ns()
                try:
                    row = json.loads(line)
                except ValueError:
                    self.log.write("NON_JSON_STDOUT "+line)
                    self.log.flush()
                    continue
                require(row.get("source") in SOURCES, "Unknown native observer source")
                for key in ("observer_sequence", "observer_receive_monotonic_ns", "source_stamp_ns"):
                    row[key] = None if row[key] is None else int(row[key])
                row["python_receive_monotonic_ns"] = received
                require(received >= row["observer_receive_monotonic_ns"], "Host CLOCK_MONOTONIC clocks disagree")
                source = row["source"]
                native = row["native"]
                field = "stamp" if source == "imu" else "sim_time" if source == "world_stats" else "time"
                if source in ("camera", "imu", "world_stats"):
                    require(row["source_stamp_ns"] == ns(native[field]), "Native time field was altered")
                else:
                    require(row["source_stamp_ns"] is None, "Unstamped diagnostic must not acquire a native sample stamp")
                if source == "camera":
                    encoded = native["image"].pop("data")
                    if self.image_cache is None or self.image_cache[0] != encoded:
                        raw = base64.b64decode(encoded, validate=True)
                        sha = hashlib.sha256(raw).hexdigest()
                        path = self.directory/("image-"+sha+".rgb")
                        if not path.exists():
                            path.write_bytes(raw)
                        self.image_cache = (encoded, {"path": str(path.resolve()), "sha256": sha,
                            "byte_count": len(raw), "removed_field": "native.image.data"})
                    row["binary_storage"] = self.image_cache[1].copy()
                if source == "metadata":
                    row["server_state"] = json.loads(native["string_value"])
                with self.lock:
                    require(len(self.rows) < 60000, "Native observation buffer capacity exceeded; no silent dropping")
                    self.rows.append(row)
                    self.latest[source] = row
                if self.callback:
                    self.callback(row)
        except Exception as error:
            with self.lock:
                self.errors.append(type(error).__name__+": "+str(error))

    def drain(self):
        with self.lock:
            require(not self.errors, "; ".join(self.errors))
            result = list(self.rows)
            self.rows.clear()
            return result

    def close(self):
        try:
            self.cleanup = stop_owned(self.process)
        finally:
            self.thread.join(timeout=2)
            self.log.close()
        return self.cleanup


class Client:
    def __init__(self, record, directory):
        import rospy
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        if not rospy.core.is_initialized():
            rospy.init_node("lesson21_timing", anonymous=True, disable_signals=True)
        self.stream_id = uuid.uuid4().hex
        self.lock, self.rows, self.latest = threading.Lock(), deque(), {}
        self.errors = []
        self.publisher = rospy.Publisher("/lesson21/events", String, queue_size=1000)
        self.subscription = rospy.Subscriber("/lesson21/events", String, self.receive, queue_size=1000, buff_size=16777216)
        self.observer = Observer(record, directory, callback=self.forward)

    def forward(self, native_row):
        row = dict(native_row)
        row["stream_id"] = self.stream_id
        row["ros_publish_monotonic_ns"] = time.monotonic_ns()
        self.publisher.publish(self.String(data=json.dumps(row, separators=(",", ":"), allow_nan=False)))

    def receive(self, message):
        try:
            row = json.loads(message.data)
            if row.get("stream_id") != self.stream_id:
                return
            row["ros_receive_monotonic_ns"] = time.monotonic_ns()
            require(row["ros_receive_monotonic_ns"] >= row["ros_publish_monotonic_ns"], "ROS receipt precedes host publish")
            with self.lock:
                require(len(self.rows) < 60000, "ROS observation buffer capacity exceeded; no silent dropping")
                self.rows.append(row)
                self.latest[row["source"]] = row
        except Exception as error:
            with self.lock:
                self.errors.append(type(error).__name__+": "+str(error))

    def drain(self):
        self.observer.drain()  # Bound the native queue; the independent ROS queue is the measured stream.
        with self.lock:
            require(not self.errors, "; ".join(self.errors))
            result = list(self.rows)
            self.rows.clear()
            return result

    def call(self, name, service_type, *args, **kwargs):
        self.rospy.wait_for_service(name, timeout=4)
        done, answer = threading.Event(), {}
        def invoke():
            proxy = self.rospy.ServiceProxy(name, service_type)
            try:
                answer["value"] = proxy(*args, **kwargs)
            except Exception as error:
                answer["error"] = error
            finally:
                proxy.close()
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        require(done.wait(4), "Finite ROS service timeout: "+name)
        if "error" in answer:
            raise RuntimeError(str(answer["error"]))
        result = answer["value"]
        require(not hasattr(result, "success") or result.success, "ROS service refused: "+getattr(result, "status_message", name))
        return result

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, value):
        from std_srvs.srv import Empty
        self.call("/gazebo/pause_physics" if value else "/gazebo/unpause_physics", Empty)

    def sim_time_ns(self):
        from gazebo_msgs.srv import GetWorldProperties
        return round(self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time*1e9)

    def snapshot(self, paused=None):
        after = time.monotonic_ns()
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            self.drain()
            value = self.latest.get("world_stats")
            if value and value["observer_receive_monotonic_ns"] >= after and (paused is None or value["native"]["paused"] == paused):
                return {"world_statistics": value, "ros_service_world_sim_time_ns": self.sim_time_ns(),
                        "ros_service_paused": self.physics().pause}
            time.sleep(.01)
        raise RuntimeError("Fresh native WorldStatistics acknowledgement timed out")

    def close(self):
        try:
            return self.observer.close()
        finally:
            self.subscription.unregister()
            self.publisher.unregister()


def check_metadata(row, config, lockstep=True, benchmark=False):
    data = row["server_state"]
    require(data["rendering_lockstep_enabled"] is lockstep, "Actual server lockstep mode differs from requested argv")
    require(data["world"] == "lesson21", "Wrong native world")
    require(abs(data["max_step_size_s"]-config["physics_step_s"]) < 1e-12, "Actual physics step differs")
    require(abs(data["real_time_update_rate_hz"]-(0 if benchmark else config["real_time_update_rate_hz"])) < 1e-9, "Actual physics rate cap differs")
    for source in ("camera", "imu"):
        require(data[source]["strict_rate"] is lockstep, "Actual sensor StrictRate differs from startup mode")
        require(data[source]["update_rate_hz"] == config[source+"_rate_hz"], "Actual sensor rate differs from configuration")
    return data


def collect(reader, config, phase, trace, min_sim=2, min_wall=0, timeout=20, nominal=True, ros=True, producer_min_sim=0):
    deadline = time.monotonic()+timeout
    ready = set()
    while time.monotonic() < deadline:
        values = reader.drain()
        ready.update(row["source"] for row in values)
        if len(ready) == len(SOURCES):
            break
        time.sleep(.01)
    require(len(ready) == len(SOURCES), "Five native streams did not become observable before sampling")
    start_wall = time.monotonic_ns()
    rows = []
    stamps = {source: set() for source in ("camera", "imu")}
    while time.monotonic() < deadline:
        for row in reader.drain():
            if row["observer_receive_monotonic_ns"] < start_wall:
                continue
            row.update(phase=phase, disposition="diagnostic")
            if row["source"] in stamps:
                stamp = row["source_stamp_ns"]
                if stamp in stamps[row["source"]]:
                    row["disposition"] = "duplicate_native_stamp"
                else:
                    stamps[row["source"]].add(stamp)
                    row["disposition"] = "accepted"
            rows.append(row)
            trace.append(row)
        elapsed = (time.monotonic_ns()-start_wall)/1e9
        if elapsed >= min_wall and all(len(stamps[s]) >= (8 if s == "camera" else 20) and
             (max(stamps[s])-min(stamps[s]))/1e9 >= min_sim for s in stamps):
            if producer_min_sim:
                updates = producer_statistics([row for row in rows if row["source"] == "metadata"])
                camera = updates.get("camera", {})
                if camera.get("measurement_span_s", 0) < producer_min_sim:
                    time.sleep(.01)
                    continue
            break
        time.sleep(.01)
    else:
        raise RuntimeError("Finite native sampling window timed out during "+phase)
    result = summarize(rows, config, nominal=nominal, ros=ros)
    result["start_monotonic_ns"] = start_wall
    result["wall_seconds"] = (time.monotonic_ns()-start_wall)/1e9
    return result


def describe(values):
    values = sorted(values)
    require(values, "Empty numeric summary")
    return {"min": values[0], "mean": sum(values)/len(values), "median": (values[(len(values)-1)//2]+values[len(values)//2])/2, "max": values[-1]}


def producer_statistics(metadata_rows):
    """Index coherent producer snapshots by event counter, never arrival order."""
    snapshots, index = [], {}
    for row in metadata_rows:
        update = row.get("server_state", {}).get("camera", {}).get("producer_update")
        if not update or not update.get("hook_attached") or int(update["event_count"]) <= 0:
            continue
        count = int(update["event_count"])
        require(int(update["producer_last_callback_monotonic_ns"]) >= int(update["hook_requested_monotonic_ns"]), "Producer callback predates its hook request")
        snapshots.append(row)
        if count in index:
            previous = index[count]["server_state"]["camera"]["producer_update"]
            require(update == previous, "Same producer event counter has incoherent snapshot fields")
        else:
            index[count] = row
    if not index:
        return {}
    low, high = index[min(index)], index[max(index)]
    first = low["server_state"]["camera"]["producer_update"]
    last = high["server_state"]["camera"]["producer_update"]
    delta = int(last["event_count"])-int(first["event_count"])
    span = (int(last["last_measurement_time_ns"])-int(first["last_measurement_time_ns"]))/1e9
    return {"camera": {"first": {"observer_sequence": low["observer_sequence"], "producer_update": first},
        "last": {"observer_sequence": high["observer_sequence"], "producer_update": last},
        "event_count_delta": delta, "measurement_span_s": span,
        "successful_update_rate_hz": delta/span if span > 0 else None,
        "same_stamp_delta": int(last["same_stamp_count"])-int(first["same_stamp_count"]),
        "regressed_stamp_delta": int(last["regressed_stamp_count"])-int(first["regressed_stamp_count"]),
        "snapshot_count": len(snapshots), "unique_counter_count": len(index),
        "note": "Successful native sensor Update events after hook attachment. Counter delta divided by actual LastMeasurementTime delta; not hardware exposures or delivered Image count."}}


def summarize(rows, config, nominal=True, ros=True):
    result = {"channels": {}}
    channels = {}
    for source in ("camera", "imu"):
        received = [row for row in rows if row["source"] == source]
        values = [row for row in received if row["disposition"] == "accepted"]
        arrival = [row["source_stamp_ns"] for row in received]
        accepted_arrival = [row["source_stamp_ns"] for row in values]
        if nominal:
            require(all(b > a for a, b in zip(accepted_arrival, accepted_arrival[1:])), "Main fresh native stamps must strictly increase")
        # Keep trace/arrival rows untouched. This index measures observed unique
        # label coverage; it is not proof of every producer update or exposure.
        times = accepted_arrival if nominal else sorted(set(arrival))
        require(len(times) >= 2, "Need two distinct native time labels")
        regressions = [{"previous_stamp_ns": a, "current_stamp_ns": b,
                        "backstep_ns": a-b, "previous_observer_sequence": received[i]["observer_sequence"],
                        "observer_sequence": received[i+1]["observer_sequence"]}
                       for i, (a, b) in enumerate(zip(arrival, arrival[1:])) if b < a]
        duration = (times[-1]-times[0])/1e9
        rate = (len(times)-1)/duration
        if nominal:
            tolerance = .02 if source == "camera" else .05
            require(abs(rate/config[source+"_rate_hz"]-1) <= tolerance, "Measured main "+source+" sim-time rate differs from configured rate")
        gaps = [(b-a)/1e9 for a, b in zip(times, times[1:])]
        for gap in gaps:
            ticks = gap/config["physics_step_s"]
            require(abs(ticks-round(ticks)) < 1e-5, "Native stamp gap is not quantized to the actual physics tick")
        wall = [row["observer_receive_monotonic_ns"] for row in received]
        wall_span = (max(wall)-min(wall))/1e9
        unique_wall = [row["observer_receive_monotonic_ns"] for row in values]
        unique_wall_span = (max(unique_wall)-min(unique_wall))/1e9
        channel = {"count": len(times), "stamps_ns": times, "stamp_span_s": duration, "sim_rate_hz": rate,
                   "stamps_order": "strict_unique_arrival" if nominal else "ascending_unique_native_label",
                   "sim_rate_interpretation": "observed_unique_time_label_coverage_density",
                   "received_count": len(received), "unique_stamp_count": len(times),
                   "duplicate_stamp_count": len(received)-len(set(arrival)),
                   "arrival_stamp_regressions": {"count": len(regressions),
                       "max_backstep_ns": max((item["backstep_ns"] for item in regressions), default=0),
                       "examples": regressions[:5]},
                   "stamp_gaps_s": describe(gaps), "observer_receive_span_s": wall_span,
                   "observer_wall_rate_hz": (len(received)-1)/wall_span,
                   "unique_stamp_observer_receive_span_s": unique_wall_span,
                   "unique_stamp_observer_wall_rate_hz": (len(times)-1)/unique_wall_span}
        if ros:
            receive = [row["ros_receive_monotonic_ns"] for row in received]
            channel["ros_wall_rate_hz"] = (len(received)-1)/((max(receive)-min(receive))/1e9)
            channel["ros_send_to_receive_ms"] = describe([(row["ros_receive_monotonic_ns"]-row["ros_publish_monotonic_ns"])/1e6 for row in received])
            channel["observer_to_ros_receive_ms"] = describe([(row["ros_receive_monotonic_ns"]-row["observer_receive_monotonic_ns"])/1e6 for row in received])
        result["channels"][source] = channel
        channels[source] = values
    indexes = {source: {row["source_stamp_ns"]: row for row in channels[source]} for source in channels}
    imu = sorted(indexes["imu"])
    pairs = []
    for stamp in sorted(indexes["camera"]):
        if imu[0] <= stamp <= imu[-1]:
            position = bisect_left(imu, stamp)
            candidates = imu[max(0, position-1):position+1]
            nearest = min(candidates, key=lambda actual: (abs(actual-stamp), actual))
            pairs.append({"camera_stamp_ns": stamp, "imu_stamp_ns": nearest, "signed_skew_ns": nearest-stamp,
                          "camera_observer_sequence": indexes["camera"][stamp]["observer_sequence"],
                          "imu_observer_sequence": indexes["imu"][nearest]["observer_sequence"]})
    require(pairs, "No common native timestamp interval for nearest-neighbor observation")
    if nominal:
        require(max(abs(pair["signed_skew_ns"]) for pair in pairs)/1e9 <= .01+2*config["physics_step_s"], "Main nearest IMU stamp is unexpectedly distant")
    result["nearest_neighbors"] = {"pairs": pairs, "abs_skew_ms": describe([abs(pair["signed_skew_ns"])/1e6 for pair in pairs]),
        "note": "Sorted unique native-label lookup only; trace arrival order remains intact. Pair IDs select the first observed event at each label. No resampling, timestamp rewriting, fusion or network-layer reorder attribution."}
    result["world_statistics"] = [row for row in rows if row["source"] == "world_stats"]
    result["performance_metrics"] = [row for row in rows if row["source"] == "performance_metrics"]
    result["server_metadata"] = [row for row in rows if row["source"] == "metadata"]
    result["producer_updates"] = producer_statistics(result["server_metadata"])
    stats = result["world_statistics"]
    if len(stats) >= 2:
        sim = ns(stats[-1]["native"]["sim_time"])-ns(stats[0]["native"]["sim_time"])
        real = ns(stats[-1]["native"]["real_time"])-ns(stats[0]["native"]["real_time"])
        result["world_statistics_rtf"] = sim/real if real > 0 else None
    # Decode one actual frame as evidence that this is a rendered camera stream.
    sample = channels["camera"][-1]
    image = sample["native"]["image"]
    raw = Path(sample["binary_storage"]["path"]).read_bytes()
    require(image["width"] == 128 and image["height"] == 96 and image["step"] == 384 and len(raw) == 36864, "Actual camera byte layout differs")
    require(hashlib.sha256(raw).hexdigest() == sample["binary_storage"]["sha256"], "Raw camera artifact hash differs")
    center = 48*384+64*3
    result["actual_image_evidence"] = {"binary_storage": sample["binary_storage"], "width": image["width"], "height": image["height"], "step": image["step"], "pixel_format": image["pixel_format"], "center_rgb": list(raw[center:center+3])}
    return result


def native_command(env, *arguments):
    completed = subprocess.run(["gz", "world", "-w", "lesson21"]+list(arguments), env=env,
        capture_output=True, text=True, timeout=8)
    require(completed.returncode == 0, "Native world command failed: "+completed.stderr.strip())
    return {"argv": completed.args, "returncode": completed.returncode,
            "stdout": completed.stdout, "stderr": completed.stderr}


def isolated_port():
    with socket.socket() as reserved:
        reserved.bind(("127.0.0.1", 0))
        return reserved.getsockname()[1]


def compare_children(config, record, directory, trace, children):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    text = (Path.cwd()/"world.sdf").read_bytes().decode("utf-8")
    benchmark = directory/"benchmark.sdf"
    benchmark.write_bytes(replace_scalar(text, "real_time_update_rate", 0).encode("utf-8"))
    for mode in (False, True):
        check_cancelled()
        name = "lockstep_on" if mode else "lockstep_off"
        folder = directory/name
        folder.mkdir()
        env = os.environ.copy()  # Preserve the caller's session token and process group.
        env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:"+str(isolated_port())
        env["IGN_PARTITION"] = "gazeboteach-sync-"+uuid.uuid4().hex
        env["GAZEBO_LOG_PATH"] = str(folder/"gazebo-log")
        Path(env["GAZEBO_LOG_PATH"]).mkdir()
        argv = ["gzserver", "--verbose", "--pause"]+(["--lockstep"] if mode else [])+[str(benchmark)]
        evidence = {"name": name, "lockstep": mode, "argv": argv, "benchmark_world_sha256": digest(benchmark),
                    "benchmark_world": str(benchmark), "gazebo_master_uri": env["GAZEBO_MASTER_URI"],
                    "ign_partition": env["IGN_PARTITION"], "caller_pgid": os.getpgrp(),
                    "inherits_session_token": env.get("GAZEBOTEACH_SESSION_TOKEN", "") == os.environ.get("GAZEBOTEACH_SESSION_TOKEN", "")}
        children.append(evidence)
        server, reader = None, None
        with (folder/"gzserver.log").open("w", encoding="utf-8") as log:
            try:
                server = subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT)
                evidence.update(pid=server.pid, server_pgid=os.getpgid(server.pid))
                reader = Observer(record, folder, env=env)
                evidence.update(observer_pid=reader.process.pid, observer_pgid=os.getpgid(reader.process.pid))
                deadline = time.monotonic()+25
                while time.monotonic() < deadline:
                    require(server.poll() is None, "Isolated child server exited before its world became ready")
                    reader.drain()
                    if "world_stats" in reader.latest:
                        break
                    time.sleep(.02)
                require("world_stats" in reader.latest, "Isolated child WorldStatistics readiness timed out")
                evidence["unpause_command"] = native_command(env, "-p", "0")
                evidence["window"] = collect(reader, config, "comparison_"+name, trace,
                    min_sim=1, min_wall=2, timeout=20, nominal=False, ros=False, producer_min_sim=1)
                require(evidence["window"]["server_metadata"], "Need actual child server mode metadata")
                for row in evidence["window"]["server_metadata"]:
                    check_metadata(row, config, lockstep=mode, benchmark=True)
                if mode:
                    producer = evidence["window"]["producer_updates"]["camera"]
                    actual = producer["successful_update_rate_hz"]
                    require(producer["measurement_span_s"] >= 1, "Need one simulation second of actual producer Update evidence")
                    require(producer["same_stamp_delta"] == 0 and producer["regressed_stamp_delta"] == 0, "Lockstep producer measurement labels repeated or regressed during the selected event interval")
                    require(abs(actual/config["camera_rate_hz"]-1) <= .02, "Lockstep child successful native camera Update-event rate differs from configured rate")
            finally:
                # Attempt cleanup for both owned PIDs even if one fails.
                errors = {}
                for label, obj in (("observer", reader), ("server", server)):
                    if obj is None:
                        continue
                    try:
                        evidence[label+"_cleanup"] = obj.close() if label == "observer" else stop_owned(obj)
                    except Exception as error:
                        errors[label] = str(error)
                evidence["cleanup_errors"] = errors
                require(not errors, "Owned child cleanup failures: "+str(errors))
                check_cancelled()


def save(path, result):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ready")
    for command in ("observe", "probe"):
        p = sub.add_parser(command)
        p.add_argument("--seconds", type=float, default=2)
        p.add_argument("--report", type=Path)
    sub.add_parser("compare").add_argument("--report", type=Path)
    p = sub.add_parser("config")
    p.add_argument("--camera-hz", type=int, choices=(10, 20))
    p.add_argument("--step-ms", type=int, choices=(1, 2))
    args = parser.parse_args()
    result = {"passed": False, "trace": [], "children": []}
    started = time.monotonic()
    path = None
    try:
        with cancellation():
            if hasattr(args, "seconds"):
                require(math.isfinite(args.seconds) and .5 <= args.seconds <= 5, "Observation seconds must be finite within[0.5,5]")
            if args.command == "config":
                result.update(edit(args.camera_hz, args.step_ms))
            else:
                config, record = prepared()
                result["config"] = config
                session = Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))
                directory = session/("timing-"+uuid.uuid4().hex)
                path = getattr(args, "report", None) or directory/"report.json"
                result["trace_path"] = str(path.resolve())
                if args.command in ("probe", "ready"):
                    reader = Observer(record, directory)
                    try:
                        result["window"] = collect(reader, config, args.command, result["trace"], min_sim=1 if args.command == "ready" else args.seconds, nominal=args.command == "ready", ros=False)
                        if args.command == "ready":
                            for row in result["window"]["server_metadata"]:
                                check_metadata(row, config)
                    finally:
                        result["observer_cleanup"] = reader.close()
                else:
                    client, original = Client(record, directory), None
                    try:
                        original = client.physics().pause
                        result["pause_before"] = original
                        if args.command == "compare":
                            client.pause(True)
                            compare_children(config, record, directory/"comparison", result["trace"], result["children"])
                        else:
                            client.pause(False)
                            result["window"] = collect(client, config, args.command, result["trace"], min_sim=1 if args.command == "ready" else args.seconds)
                            for row in result["window"]["server_metadata"]:
                                check_metadata(row, config)
                    finally:
                        try:
                            if original is not None:
                                client.pause(original)
                                result["pause_after"] = client.physics().pause
                        finally:
                            result["observer_cleanup"] = client.close()
            result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-started
    if path:
        save(path, result)
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
