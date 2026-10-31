#!/usr/bin/env python3
"""Actual CPU/GPU scans: geometry, missing echoes, and single-variable cases.

GPU acceptance requires ikaros' real graphical session. Each case starts a new
private ROS/Gazebo pair with its own actual gzclient. Exact wall poses and pause
state are measured before/after the canonical verifier; raw scans survive cleanup.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
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


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def mutate(path, samples=None, distance=None):
    tree = ET.parse(str(path))
    if samples is not None:
        nodes = tree.findall("./world/model[@name='scanner']/link/sensor/ray/scan/horizontal/samples")
        require(len(nodes) == 2, "Expected two sensor sample counts")
        for node in nodes:
            node.text = str(samples)
    if distance is not None:
        node = tree.find("./world/model[@name='target_wall']/pose")
        values = node.text.split()
        values[0] = str(distance)
        node.text = " ".join(values)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def inspect_raw(report, samples):
    counts, observed = {}, {}
    for phase_name, phase in report["phases"].items():
        front = phase["wall_front_x_m"]
        counts[phase_name], observed[phase_name] = {}, {}
        for source in ("cpu", "gpu"):
            rows = [row for row in report["trace"] if row["phase"] == phase_name
                    and row["source"] == source and row["disposition"] == "accepted"]
            stamps = [row["stamp_ns"] for row in rows]
            require(len(rows) >= 12 and len(set(stamps)) == len(rows),
                    "Expected twelve distinct actual scan headers per source/phase")
            require(all(b > a for a, b in zip(stamps, stamps[1:])) and (stamps[-1]-stamps[0])/1e9 >= .5,
                    "Actual scan window lacks monotonic simulation duration")
            for row in rows:
                require(len(row["ranges"]) == samples, "Raw beam count differs from current personal input")
                require(row["scan_time"] == 0 and row["time_increment"] == 0,
                        "Current stock plugin unexpectedly reports rolling scan timing")
                for degrees in (-90, -60, -45, -30, 0, 30, 45, 60, 90):
                    angle = math.radians(degrees)
                    index = int(round((angle-row["angle_min"])/row["angle_increment"]))
                    actual = row["ranges"][index]
                    near = (source == "cpu" and degrees == -45) or (source == "gpu" and degrees == 45)
                    if near:
                        expected = math.sqrt(2)*1.3  # near face of fixed .4 m boxes at (1.5,+/-1.5)
                    elif abs(degrees) >= 60 or (abs(degrees) == 45 and front > 4):
                        expected = None  # outside finite wall width, no other target on this ray
                    else:
                        expected = front/math.cos(angle)
                    if expected is None:
                        require(actual == "+inf", "A documented missing echo is not actual +inf")
                    else:
                        require(isinstance(actual, (int, float)) and math.isfinite(actual)
                                and abs(actual-expected) < .08,
                                "Actual {} {} angle{} differs from independent geometry".format(phase_name, source, degrees))
            counts[phase_name][source] = {"messages": len(rows), "span_s": (stamps[-1]-stamps[0])/1e9,
                                         "rate_hz": (len(rows)-1)/((stamps[-1]-stamps[0])/1e9)}
            center = int(round(-rows[-1]["angle_min"]/rows[-1]["angle_increment"]))
            observed[phase_name][source] = rows[-1]["ranges"][center]
    require(abs(observed["moved"]["cpu"]-observed["baseline"]["cpu"]-1) < .08,
            "Actual wall displacement did not produce expected CPU range change")
    require(abs(observed["moved"]["gpu"]-observed["baseline"]["gpu"]-1) < .08,
            "Actual wall displacement did not produce expected GPU range change")
    return {"windows": counts, "observed_center_m": observed}


def verify_case(session, artifacts, label, samples, expected=True, diagnosis=None):
    before, pose_before = actual_stats(session), actual_pose(session, "target_wall")
    world = session.workspace/"world.sdf"
    world_bytes = world.read_bytes()
    path = artifacts/(label+".json")
    result = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                            cwd=str(session.workspace), env=session.env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=75)
    path.with_suffix(".stdout.log").write_text(result.stdout, encoding="utf-8")
    require((result.returncode == 0) is expected,
            "Unexpected verifier result {}: {}".format(label, result.stdout[-3500:]))
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "JSON and process result disagree")
    if diagnosis:
        require(diagnosis.lower() in (result.stdout+json.dumps(observation)).lower(), "Missing diagnosis "+diagnosis)
    raw = inspect_raw(observation, samples) if expected else None
    after, pose_after = actual_stats(session), actual_pose(session, "target_wall")
    require(before["paused"] == after["paused"], "Verifier did not restore actual pause state")
    require(all(abs(a-b) < 1e-6 for a, b in zip(pose_before, pose_after)), "Verifier did not restore actual target pose")
    require(world.read_bytes() == world_bytes, "Verifier rewrote personal input")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Rejected input still advanced paused physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected,
            "independent_raw_checks": raw, "wall_before": pose_before, "wall_after": pose_after,
            "stats_before": before, "stats_after": after, "raw_report": str(path),
            "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    candidates = [l for l in lessons if l.key == "classic11-noetic:15"]
    require(len(candidates) == 1, "Expected L15: "+repr(errors))
    lesson = candidates[0]
    original_source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": original_source,
              "real_gui_each_case": True, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l15-") as directory:
        try:
            for label, samples, distance, paused in (("baseline", 181, 3., True),
                                                     ("samples_only", 361, 3., False),
                                                     ("distance_only", 181, 3.5, True)):
                storage = Storage(Path(directory)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    mutate(workspace/"world.sdf", samples=samples, distance=distance)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=True, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label, samples))
                if label == "distance_only":
                    path, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        mutate(path, distance=3.6)
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", samples, False, "reload"))
                    finally:
                        path.write_bytes(saved)
                    for name, xpath, value, diagnostic in (
                        ("wrong_frame", "./world/model[@name='scanner']/link/sensor/plugin/frameName", "fake_frame", "frame"),
                        ("wrong_noise", "./world/model[@name='scanner']/link/sensor/ray/noise/stddev", "0.2", "noise")):
                        try:
                            tree = ET.fromstring(saved)
                            node = tree.find(xpath)
                            require(node is not None, "Injection target missing: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, name, samples, False, diagnostic))
                        finally:
                            path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired", samples))
                session.stop(reason="L15 private graphical scenario completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L15 acceptance final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = hashes(lesson.folder) == original_source
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
