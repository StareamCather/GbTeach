"""L03: real reference-frame composition, rotation wrap, and invalid reference.

The independent oracle composes 3x3 rotation matrices and translated origins.
Actual Gazebo Euler poses are converted to quaternions only for a sign-invariant
rotation-distance comparison. Each case uses an actual private-world gzserver.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
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


def command(session, *arguments):
    result = subprocess.run(list(arguments), cwd=str(session.workspace), env=session.env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=7)
    if result.returncode:
        raise AssertionError("Command failed: {}\n{}".format(" ".join(arguments), result.stderr[-2000:]))
    return result.stdout


def actual_paused(session):
    output = command(session, "gz", "topic", "-e", "/gazebo/{}/world_stats".format(session.world_name), "-d", "1")
    samples = re.findall(r"\bpaused:\s*(true|false)\b", output)
    if not samples:
        raise AssertionError("No measured pause state for this world")
    return samples[-1] == "true"


def actual_pose(session, model):
    output = command(session, "gz", "model", "-w", session.world_name, "-m", model, "-p")
    for line in reversed(output.splitlines()):
        if len(line.split()) == 6:
            try:
                values = [float(value) for value in line.split()]
                if all(math.isfinite(value) for value in values):
                    return values
            except ValueError:
                pass
    raise AssertionError("No real six-component model pose: " + output)


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def compose(parent, local):
    point, orientation = parent
    offset, relative = local
    position = [point[row] + sum(orientation[row][k] * offset[k] for k in range(3)) for row in range(3)]
    matrix = [[sum(orientation[row][k] * relative[k][column] for k in range(3))
               for column in range(3)] for row in range(3)]
    return position, matrix


def transform(values):
    return list(values[:3]), rotation(values[3:])


def quaternion(matrix):
    """Return xyzw; use the dominant diagonal branch for angles near pi."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        q = [(m[2][1] - m[1][2]) / scale, (m[0][2] - m[2][0]) / scale,
             (m[1][0] - m[0][1]) / scale, scale / 4]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = [scale / 4, (m[0][1] + m[1][0]) / scale,
             (m[0][2] + m[2][0]) / scale, (m[2][1] - m[1][2]) / scale]
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = [(m[0][1] + m[1][0]) / scale, scale / 4,
             (m[1][2] + m[2][1]) / scale, (m[0][2] - m[2][0]) / scale]
    else:
        scale = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = [(m[0][2] + m[2][0]) / scale, (m[1][2] + m[2][1]) / scale,
             scale / 4, (m[1][0] - m[0][1]) / scale]
    length = math.sqrt(sum(value * value for value in q))
    return [value / length for value in q]


def rotation_distance(first, second):
    return 2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(first, second)))))


def pose_values(element):
    if element is None or not element.text:
        return [0.0] * 6
    values = [float(value) for value in element.text.split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise AssertionError("Acceptance oracle needs a finite six-component pose")
    return values


def expected_transforms(path):
    world = ET.parse(str(path)).getroot().find("world")
    rig = world.find("model[@name='rig']")
    frame = world.find("frame[@name='mount_frame']")
    marker = world.find("model[@name='sensor_marker']")
    if rig is None or frame is None or marker is None:
        raise AssertionError("L03 rig/mount_frame/sensor_marker chain is missing")
    if frame.find("pose").get("relative_to") != "rig":
        raise AssertionError("Acceptance expected mount_frame relative to rig")
    base = transform(pose_values(rig.find("pose")))
    mount = compose(base, transform(pose_values(frame.find("pose"))))
    reference = marker.find("pose").get("relative_to", "world")
    local = transform(pose_values(marker.find("pose")))
    if reference == "mount_frame":
        sensor = compose(mount, local)
    elif reference in ("", "world", "__world__"):
        sensor = local
    else:
        raise AssertionError("Unexpected reference in acceptance oracle: " + reference)
    return {"rig": base, "sensor_marker": sensor}


def challenge_world(path):
    tree = ET.parse(str(path))
    world = tree.getroot().find("world")
    world.find("model[@name='rig']/pose").text = "-1.2 0.7 0.4 0 0 3.4"
    world.find("frame[@name='mount_frame']/pose").text = "0.2 -0.1 0.7 0 0 -0.45"
    world.find("model[@name='sensor_marker']/pose").text = "0.6 -0.35 0.2 0.2 -0.1 1.1"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def invalid_reference(path):
    tree = ET.parse(str(path))
    tree.getroot().find("world/model[@name='sensor_marker']/pose").set("relative_to", "missing_frame")
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def validate(session, label, should_pass):
    path = session.workspace / session.lesson.data.get("world", "world.sdf")
    initial_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    paused_before = actual_paused(session)
    offset = session.log_path.stat().st_size
    item = {"name": label, "expected_verifier_pass": should_pass, "original_paused": paused_before,
            "world_sha256_before": initial_hash}
    try:
        expected = expected_transforms(path) if should_pass else None
        if label == "baseline_rig_frame_marker_chain":
            if any(abs(value - target) > 1e-7 for value, target in zip(expected["sensor_marker"][0], (2.0, 2.0, 0.8))):
                raise AssertionError("Baseline source no longer represents the documented (2, 2, 0.8) example")
        if label == "omitted_relative_to_uses_world":
            if any(abs(value - target) > 1e-7 for value, target in zip(expected["sensor_marker"][0], (1.0, 0.0, 0.0))):
                raise AssertionError("Omitted reference must use world coordinates for this world-level model")
        result = session.verify()
        item["verifier_result"] = result
        item["evidence"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        if result.get("passed") is not should_pass:
            raise AssertionError("Unexpected source verifier result for " + label)
        paused_after = actual_paused(session)
        item.update(actual_paused_after=paused_after, pause_restored=paused_after == paused_before)
        if paused_after != paused_before:
            raise AssertionError("Verifier changed pause state")
        final_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        item.update(world_sha256_after=final_hash, configuration_unchanged=final_hash == initial_hash)
        if final_hash != initial_hash:
            raise AssertionError("Verifier changed the student's configuration")
        if should_pass:
            measurements = {}
            for model, (position, matrix) in expected.items():
                measured = actual_pose(session, model)
                actual_q = quaternion(rotation(measured[3:]))
                expected_q = quaternion(matrix)
                error_xyz = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, measured[:3])))
                error_angle = rotation_distance(expected_q, actual_q)
                if error_xyz > 0.025 or error_angle > 0.025:
                    raise AssertionError("{} real transform mismatch: position error {}, rotation error {}".format(
                        model, error_xyz, error_angle))
                measurements[model] = {"expected_position": position, "actual_pose_xyz_rpy": measured,
                                       "expected_quaternion_xyzw": expected_q, "actual_quaternion_xyzw": actual_q,
                                       "position_error_m": error_xyz, "rotation_error_rad": error_angle,
                                       "negated_quaternion_error_rad": rotation_distance(expected_q, [-x for x in actual_q])}
            item["measurements"] = measurements
            if "sensor_marker" not in item["evidence"]:
                raise AssertionError("Canonical verifier log did not include actual marker evidence")
        elif "未知参考系" not in item["evidence"] or "missing_frame" not in item["evidence"]:
            raise AssertionError("Missing frame did not produce a clear reference diagnosis")
        item["passed"] = True
        return item
    finally:
        if session.running and actual_paused(session) != paused_before:
            session.pause(paused_before)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    arguments = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    candidates = [lesson for lesson in lessons if lesson.key == "classic11-noetic:03"]
    if len(candidates) != 1:
        raise RuntimeError("Expected one L03 course; catalog errors: " + repr(errors))
    lesson = candidates[0]
    report = {"passed": False, "lesson": lesson.key, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l03-") as directory:
        root = Path(directory)
        try:
            baseline = LessonSession(ROOT, lesson, storage=Storage(root / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(True)
            report["scenarios"].append(validate(baseline, "baseline_rig_frame_marker_chain", True))
            baseline.stop(reason="L03 baseline accepted")

            storage = Storage(root / "challenge")
            workspace = storage.prepare(lesson)
            path = workspace / lesson.data.get("world", "world.sdf")
            challenge_world(path)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            if actual_paused(challenge):
                raise AssertionError("Challenge must begin in running state")
            report["scenarios"].append(validate(challenge, "challenge_translations_and_wrapped_rotation", True))

            valid = path.read_bytes()
            try:
                invalid_reference(path)
                item = validate(challenge, "unknown_reference_rejected", False)
                item["configuration_changed_after_server_start"] = True
                report["scenarios"].append(item)
            finally:
                path.write_bytes(valid)
            report["scenarios"].append(validate(challenge, "restored_reference_reaccepted", True))
            challenge.stop(reason="L03 explicit-reference challenges accepted")

            storage = Storage(root / "default-world-reference")
            workspace = storage.prepare(lesson)
            path = workspace / lesson.data.get("world", "world.sdf")
            tree = ET.parse(str(path))
            marker_pose = tree.getroot().find("world/model[@name='sensor_marker']/pose")
            marker_pose.attrib.pop("relative_to", None)
            marker_pose.text = "1 0 0 0 0 0"
            tree.write(str(path), encoding="utf-8", xml_declaration=True)
            implicit = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(implicit)
            implicit.start(gui=False, terminals=False)
            implicit.pause(True)
            report["scenarios"].append(validate(implicit, "omitted_relative_to_uses_world", True))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-24000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L03 scenario test finished")
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(0.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report["leftovers"] = leftovers
            report["elapsed_seconds"] = round(time.monotonic() - started, 3)
            if any(leftovers.values()):
                report["passed"] = False
            if arguments.report:
                path = Path(arguments.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
