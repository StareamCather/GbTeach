#!/usr/bin/env python3
"""Real IMU data: upright/tilted support, signed rotation, and free fall.

All changes are confined to fresh private exercises. Saved source, pause state,
and actual floating-body poses are checked independently. Raw Imu messages and
queried world-state brackets survive cleanup, rather than replacing observations
with the ideal equations used to evaluate them.
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


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def inspect_raw(report):
    parameters = report["parameters"]
    result = {}
    for name, phase in report["phases"].items():
        rows = [row for row in report["trace"] if row["phase"] == name
                and row["source"] == phase["source"] and row["disposition"] == "accepted"]
        stamps = [row["stamp_ns"] for row in rows]
        fall = name == "freefall"
        require(len(rows) >= (30 if fall else 50), "Insufficient distinct actual IMU messages")
        require(all(b > a for a, b in zip(stamps, stamps[1:])), "IMU timestamp regression or duplication")
        span = (stamps[-1]-stamps[0])/1e9
        require(span >= (.3 if fall else .6), "IMU window lacks actual simulation duration")
        rate = (len(rows)-1)/span
        require(90 <= rate <= 110, "Measured unique IMU frequency differs from native 100 Hz")
        omega = parameters["speed_rad_s"] * (1 if name == "positive_yaw" else -1 if name == "negative_yaw" else 0)
        angle = math.radians(parameters["tilt_deg"]) if name == "tilted_static" else 0
        expected_force = [0, 9.81*math.sin(angle), 9.81*math.cos(angle)] if not fall else [0, 0, 0]
        gyro_mean, force_mean = [], []
        for row in rows:
            q = row["orientation"]
            require(abs(sum(value*value for value in q)-1) < 1e-6, "Actual orientation quaternion is not unit length")
            require(max(abs(a-b) for a, b in zip(row["angular_velocity"], [0, 0, omega])) < .025,
                    "Actual gyro differs from controlled principal-axis motion")
            require(max(abs(a-b) for a, b in zip(row["linear_acceleration"], expected_force)) < .25,
                    "Actual specific force differs from supported/tilted/freefall prediction")
            for field in ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"):
                require(len(row[field]) == 9 and all(value == 0 for value in row[field]),
                        "Stock noise-zero covariance differs from documented unknown convention")
        gyro_mean = [sum(row["angular_velocity"][axis] for row in rows)/len(rows) for axis in range(3)]
        force_mean = [sum(row["linear_acceleration"][axis] for row in rows)/len(rows) for axis in range(3)]
        measured_yaw = sum(math.atan2(math.sin(yaw(b["orientation"])-yaw(a["orientation"])),
                                     math.cos(yaw(b["orientation"])-yaw(a["orientation"])))
                           for a, b in zip(rows, rows[1:]))
        require(abs(measured_yaw-omega*span) < .025, "Real quaternion progression disagrees with gyro/time")
        if name in ("upright_static", "tilted_static"):
            expected_q = [math.sin(angle/2), 0, 0, math.cos(angle/2)]
            require(all(abs(abs(sum(a*b for a, b in zip(row["orientation"], expected_q)))-1) < 1e-5 for row in rows),
                    "Stationary IMU orientation does not reflect actual tilt")
        result[name] = {"messages": len(rows), "span_s": span, "unique_rate_hz": rate,
                        "mean_gyro": gyro_mean, "mean_specific_force": force_mean,
                        "yaw_delta_rad": measured_yaw, "expected_yaw_delta_rad": omega*span}
        if fall:
            truth = phase["truth"]
            require(len(truth) >= 2, "Free fall has no independent world-state sequence")
            first, last = truth[0], truth[-1]
            dt = ((last["clock_before_ns"]+last["clock_after_ns"])-
                  (first["clock_before_ns"]+first["clock_after_ns"]))/2e9
            dz = last["state"]["position"][2]-first["state"]["position"][2]
            dv = last["state"]["linear"][2]-first["state"]["linear"][2]
            require(dt > .2 and dz < -.25 and last["state"]["linear"][2] < -1,
                    "Near-zero accelerometer data is not accompanied by actual falling motion")
            require(abs(dv/dt+9.81) < .4, "Queried actual freefall acceleration does not match gravity")
            result[name].update(actual_world_dz_m=dz, actual_world_dv_m_s=dv,
                                actual_world_acceleration_estimate_m_s2=dv/dt)
    require(set(result) == {"upright_static", "tilted_static", "positive_yaw", "negative_yaw", "freefall"},
            "One of the five physical contrasts is missing")
    return result


def verify_case(session, artifacts, label, expected=True, diagnosis=None):
    before = actual_stats(session)
    body_before = actual_pose(session, "turntable")
    inputs = {name: (session.workspace/name).read_bytes() for name in ("world.sdf", "parameters.json")}
    path = artifacts/(label+".json")
    answer = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                            cwd=str(session.workspace), env=session.env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=85)
    path.with_suffix(".stdout.log").write_text(answer.stdout, encoding="utf-8")
    require((answer.returncode == 0) is expected,
            "Unexpected verifier result {}: {}".format(label, answer.stdout[-4000:]))
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "JSON and process results disagree")
    if diagnosis:
        require(diagnosis.lower() in (answer.stdout+json.dumps(observation)).lower(), "Missing specific diagnosis: "+diagnosis)
    independent = inspect_raw(observation) if expected else None
    after, body_after = actual_stats(session), actual_pose(session, "turntable")
    require(before["paused"] == after["paused"], "Verifier failed to restore pause state")
    require(all(abs(a-b) < 1e-6 for a, b in zip(body_before, body_after)), "Actual floating body pose was not restored")
    require(all((session.workspace/name).read_bytes() == data for name, data in inputs.items()), "Verifier modified personal inputs")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Rejected input advanced paused physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected,
            "independent_raw_checks": independent, "stats_before": before, "stats_after": after,
            "floating_pose_before": body_before, "floating_pose_after": body_after,
            "raw_report": str(path), "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    candidates = [lesson for lesson in lessons if lesson.key == "classic11-noetic:16"]
    require(len(candidates) == 1, "Expected L16: "+repr(errors))
    lesson = candidates[0]
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l16-") as folder:
        try:
            for label, speed, tilt, paused in (("baseline", .6, 30, True),
                                               ("speed_only", .9, 30, False),
                                               ("tilt_only", .6, 45, True)):
                storage = Storage(Path(folder)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    (workspace/"parameters.json").write_text(json.dumps({"speed_rad_s": speed, "tilt_deg": tilt})+"\n", encoding="utf-8")
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label))
                if label == "tilt_only":
                    path = workspace/"parameters.json"
                    saved = path.read_bytes()
                    try:
                        path.write_text(json.dumps({"speed_rad_s": .8, "tilt_deg": tilt}), encoding="utf-8")
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", False, "reload"))
                    finally:
                        path.write_bytes(saved)
                    path = workspace/"world.sdf"
                    saved = path.read_bytes()
                    for label_bad, xpath, value, diagnosis in (
                        ("wrong_frame", "./world/model[@name='turntable']/link/sensor/plugin/frameName", "fake_frame", "frame"),
                        ("wrong_noise", "./world/model[@name='turntable']/link/sensor/plugin/gaussianNoise", ".1", "noise"),
                        ("wrong_reference", "./world/model[@name='turntable']/link/sensor/plugin/initialOrientationAsReference", "true", "reference")):
                        try:
                            tree = ET.fromstring(saved)
                            node = tree.find(xpath)
                            require(node is not None, "Fault injection target missing: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, label_bad, False, diagnosis))
                        finally:
                            path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired"))
                session.stop(reason="L16 private physical scenario finished")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L16 acceptance final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = hashes(lesson.folder) == source
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
