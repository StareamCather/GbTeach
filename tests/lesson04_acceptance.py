"""L04 real model-package reuse, renamed instances, and resource-entry repair.

Every positive case starts actual Gazebo in a fresh private practice directory.
Live Model messages must contain the changed box geometry for every instance;
the canonical verifier additionally performs physical fall and independence
checks. All pre-existing instance poses, pause state, and resource files must be
preserved by that verifier. Only private copies are mutated by these scenarios.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes

NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def retry_regression(verifier_path):
    """Fault injection for the observed CLI timeout, separate from real physics."""
    spec = importlib.util.spec_from_file_location("gazeboteach_l04_timeout_regression", str(verifier_path))
    verifier = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(verifier)
    cases = []
    for flag, output in (("-p", "1.8 0 0.25 0 0 0.4\n"), ("-i", 'name: "crate_right"\n')):
        argv = ("gz", "model", "-w", "default", "-m", "crate_right", flag)
        verifier.DEADLINE = time.monotonic() + 20
        success = subprocess.CompletedProcess(argv, 0, stdout=output, stderr="")
        with mock.patch.object(verifier.subprocess, "run", side_effect=[subprocess.TimeoutExpired(argv, 2.1), success]) as execute:
            actual = verifier.read_model("default", "crate_right", flag)
            if actual != output or execute.call_count != 2:
                raise AssertionError("One timed-out read did not recover after exactly one retry")
            if any(tuple(call.args[0]) != argv for call in execute.call_args_list):
                raise AssertionError("Read retry changed its target or command")
            cases.append({"name": "timed_out_{}_read_recovers".format(flag[1:]), "passed": True,
                          "attempts": execute.call_count, "result": actual.strip()})

    argv = ("gz", "model", "-w", "default", "-m", "crate_right", "-p")
    verifier.DEADLINE = time.monotonic() + 20
    with mock.patch.object(verifier.subprocess, "run", side_effect=subprocess.TimeoutExpired(argv, 2.1)) as execute:
        try:
            verifier.read_model("default", "crate_right", "-p")
        except RuntimeError as error:
            if "有限重试已用完" not in str(error):
                raise AssertionError("Repeated timeout did not report exhausted finite retry")
            message = str(error)
        else:
            raise AssertionError("Continuous timeout unexpectedly succeeded")
        if execute.call_count != 2:
            raise AssertionError("Continuous timeout exceeded the two-attempt bound")
        cases.append({"name": "continuous_timeout_is_bounded", "passed": True,
                      "attempts": execute.call_count, "diagnostic": message})

    clock = [100.0]
    verifier.DEADLINE = 100.5
    def budget_timeout(*arguments, **options):
        clock[0] = 100.4
        raise subprocess.TimeoutExpired(arguments[0], options["timeout"])
    with mock.patch.object(verifier.time, "monotonic", side_effect=lambda: clock[0]), \
            mock.patch.object(verifier.subprocess, "run", side_effect=budget_timeout) as execute:
        try:
            verifier.read_model("default", "crate_right", "-p")
        except RuntimeError as error:
            if "预算不足" not in str(error):
                raise AssertionError("Exhausted total deadline lacks a budget diagnostic")
            message = str(error)
        else:
            raise AssertionError("Query retried beyond the remaining overall deadline")
        if execute.call_count != 1:
            raise AssertionError("Near-deadline query retried despite insufficient budget")
        cases.append({"name": "total_deadline_limits_retry", "passed": True,
                      "attempts": execute.call_count, "diagnostic": message})

    verifier.DEADLINE = time.monotonic() + 20
    failure = subprocess.CompletedProcess(argv, 1, stdout="", stderr="explicit query failure")
    with mock.patch.object(verifier.subprocess, "run", return_value=failure) as execute:
        try:
            verifier.read_model("default", "crate_right", "-p")
        except RuntimeError as error:
            message = str(error)
        else:
            raise AssertionError("Nonzero read exit unexpectedly succeeded")
        if execute.call_count != 1:
            raise AssertionError("An explicit command failure was retried as a transport timeout")
        cases.append({"name": "explicit_failure_not_retried", "passed": True,
                      "attempts": execute.call_count, "diagnostic": message})

    for mutation in (("gz", "model", "-w", "default", "-m", "crate_right", "-x", "1.8"),
                     ("gz", "topic", "-p", "/gazebo/default/world_control", "-m", "pause: true")):
        verifier.DEADLINE = time.monotonic() + 20
        with mock.patch.object(verifier.subprocess, "run", side_effect=subprocess.TimeoutExpired(mutation, 2.1)) as execute:
            try:
                verifier.run(*mutation)
            except subprocess.TimeoutExpired:
                pass
            else:
                raise AssertionError("Timed-out mutation did not propagate its uncertain outcome")
            if execute.call_count != 1:
                raise AssertionError("State-changing command was retried")
            cases.append({"name": "mutation_not_retried", "passed": True,
                          "attempts": execute.call_count, "command": list(mutation)})
    return {"passed": True, "kind": "mocked_transport_fault_regression_not_physics_acceptance", "cases": cases}


def command(session, *arguments):
    result = subprocess.run(list(arguments), cwd=str(session.workspace), env=session.env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=7)
    if result.returncode:
        raise AssertionError("Command failed: {}\n{}".format(" ".join(arguments), result.stderr[-1800:]))
    return result.stdout


def actual_paused(session):
    text = command(session, "gz", "topic", "-e", "/gazebo/{}/world_stats".format(session.world_name), "-d", "1")
    values = re.findall(r"\bpaused:\s*(true|false)\b", text)
    if not values:
        raise AssertionError("No actual pause state from the current world")
    return values[-1] == "true"


def actual_pose(session, name):
    text = command(session, "gz", "model", "-w", session.world_name, "-m", name, "-p")
    for line in reversed(text.splitlines()):
        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            continue
        if len(values) == 6 and all(math.isfinite(value) for value in values):
            return values
    raise AssertionError("No live pose for {}: {}".format(name, text))


def quaternion(rpy):
    r, p, y = (value / 2 for value in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def pose_errors(first, second):
    position = math.sqrt(sum((a - b) ** 2 for a, b in zip(first[:3], second[:3])))
    dot = abs(sum(a * b for a, b in zip(quaternion(first[3:]), quaternion(second[3:]))))
    return position, 2 * math.acos(min(1.0, dot))


def asset_hashes(folder):
    files = [folder / "world.sdf"] + sorted((folder / "models").rglob("*"))
    return {path.relative_to(folder).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.is_file()}


def includes(path):
    world = ET.parse(str(path)).getroot().find("world")
    result = []
    for item in world.findall("include"):
        result.append({"name": item.findtext("name"), "uri": item.findtext("uri"),
                       "pose": [float(value) for value in item.findtext("pose", "0 0 0 0 0 0").split()]})
    if len(result) < 2 or len({item["name"] for item in result}) != len(result):
        raise AssertionError("L04 needs at least two uniquely named instances")
    return result


def live_sizes(session, name):
    text = command(session, "gz", "model", "-w", session.world_name, "-m", name, "-i")
    pattern = (r"\bbox\s*\{\s*size\s*\{\s*x:\s*(" + NUMBER + r")\s*y:\s*(" + NUMBER +
               r")\s*z:\s*(" + NUMBER + r")")
    sizes = [[float(value) for value in match] for match in re.findall(pattern, text, flags=re.S)]
    if len(sizes) < 2:
        raise AssertionError("Live Model message lacks collision and visual box sizes for {}: {}".format(name, text[-2500:]))
    return sizes


def wait_settled(session, names, height, timeout=12):
    deadline = time.monotonic() + timeout
    previous = None
    while time.monotonic() < deadline:
        poses = {name: actual_pose(session, name) for name in names}
        centered = all(abs(pose[2] - height) < 0.025 and max(abs(pose[3]), abs(pose[4])) < 0.03
                       for pose in poses.values())
        stable = previous is not None and all(abs(poses[name][2] - previous[name][2]) < 0.003 for name in names)
        if centered and stable:
            return poses
        previous = poses
    raise AssertionError("Instances did not settle to the expected box half-height: " + repr(previous))


def challenge_files(workspace):
    model_path = workspace / "models/teach_crate/model.sdf"
    tree = ET.parse(str(model_path))
    body = tree.getroot().find("model/link")
    dimensions = (0.8, 0.6, 0.7)
    for kind in ("collision", "visual"):
        body.find(kind + "/geometry/box/size").text = " ".join(str(value) for value in dimensions)
    mass = float(body.findtext("inertial/mass"))
    x, y, z = dimensions
    for axis, value in (("ixx", mass * (y * y + z * z) / 12),
                        ("iyy", mass * (x * x + z * z) / 12),
                        ("izz", mass * (x * x + y * y) / 12)):
        body.find("inertial/inertia/" + axis).text = str(value)
    tree.write(str(model_path), encoding="utf-8", xml_declaration=True)

    world_path = workspace / "world.sdf"
    tree = ET.parse(str(world_path))
    world = tree.getroot().find("world")
    for item in list(world.findall("include")):
        world.remove(item)
    for name, values in (("alpha_box", (-1.8, -0.6, 0.35, 0, 0, 0.2)),
                         ("beta_box", (0.3, 1.3, 0.35, 0, 0, -1.1)),
                         ("gamma_box", (1.7, -0.8, 0.35, 0, 0, 2.7))):
        item = ET.SubElement(world, "include")
        ET.SubElement(item, "uri").text = "model://teach_crate"
        ET.SubElement(item, "name").text = name
        ET.SubElement(item, "pose").text = " ".join(str(value) for value in values)
    tree.write(str(world_path), encoding="utf-8", xml_declaration=True)
    return dimensions


def verify_case(session, name, should_pass, expected_dimensions):
    folder = session.workspace
    entries = includes(folder / "world.sdf")
    names = [entry["name"] for entry in entries]
    before_assets = asset_hashes(folder)
    before_paused = actual_paused(session)
    before_poses = {instance: actual_pose(session, instance) for instance in names}
    offset = session.log_path.stat().st_size
    item = {"name": name, "expected_verifier_pass": should_pass, "instances": entries,
            "original_paused": before_paused, "poses_before": before_poses}
    try:
        result = session.verify()
        item["verifier_result"] = result
        item["evidence"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        if result.get("passed") is not should_pass:
            raise AssertionError("Unexpected verification result for " + name)
        after_paused = actual_paused(session)
        item.update(actual_paused_after=after_paused, pause_restored=after_paused == before_paused)
        if after_paused != before_paused:
            raise AssertionError("Model package verifier changed original pause state")
        after_assets = asset_hashes(folder)
        item["asset_hashes"] = after_assets
        item["assets_unchanged"] = before_assets == after_assets
        if before_assets != after_assets:
            raise AssertionError("Verifier changed world or model resources")
        after_poses = {instance: actual_pose(session, instance) for instance in names}
        item["poses_after"] = after_poses
        errors = {instance: pose_errors(before_poses[instance], after_poses[instance]) for instance in names}
        item["restoration_errors_position_rotation"] = errors
        if any(position > 0.025 or angle > 0.03 for position, angle in errors.values()):
            raise AssertionError("Verifier did not restore original runtime instance poses: " + repr(errors))
        if should_pass:
            item["live_box_sizes"] = {instance: live_sizes(session, instance) for instance in names}
            for instance, sizes in item["live_box_sizes"].items():
                if any(any(abs(actual - expected) > 1e-6 for actual, expected in zip(size, expected_dimensions))
                       for size in sizes):
                    raise AssertionError("Changed shared geometry is not loaded in " + instance)
                if instance not in item["evidence"]:
                    raise AssertionError("Canonical verifier lacks evidence for instance " + instance)
        elif "模型入口文件不存在" not in item["evidence"] or "model_missing.sdf" not in item["evidence"]:
            raise AssertionError("Missing model entry did not produce a clear resource diagnosis")
        item["passed"] = True
        return item
    finally:
        if session.running and actual_paused(session) != before_paused:
            session.pause(before_paused)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:04"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L04 course; catalog errors: " + repr(errors))
    lesson = matches[0]
    source_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l04-") as directory:
        root = Path(directory)
        try:
            report["retry_regression"] = retry_regression(lesson.file(lesson.data.get("verify", "verify.py")))
            baseline = LessonSession(ROOT, lesson, storage=Storage(root / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline_names = [entry["name"] for entry in includes(baseline.workspace / "world.sdf")]
            if set(baseline_names) != {"crate_left", "crate_right", "crate_back", "crate_front"}:
                raise AssertionError("Published baseline must contain the four documented crate instances")
            wait_settled(baseline, baseline_names, 0.25)
            baseline.pause(True)
            report["scenarios"].append(verify_case(baseline, "baseline_four_shared_model_instances", True, (0.5, 0.5, 0.5)))
            baseline.stop(reason="L04 baseline accepted")

            storage = Storage(root / "challenge")
            workspace = storage.prepare(lesson)
            dimensions = challenge_files(workspace)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            challenge_names = [entry["name"] for entry in includes(workspace / "world.sdf")]
            wait_settled(challenge, challenge_names, dimensions[2] / 2)
            if actual_paused(challenge):
                raise AssertionError("Challenge should begin running")
            report["scenarios"].append(verify_case(challenge, "changed_shared_dimensions_and_renamed_three_instances", True, dimensions))

            challenge.pause(True)
            config_path = workspace / "models/teach_crate/model.config"
            valid = config_path.read_bytes()
            try:
                config = ET.parse(str(config_path))
                config.getroot().find("sdf").text = "model_missing.sdf"
                config.write(str(config_path), encoding="utf-8", xml_declaration=True)
                item = verify_case(challenge, "missing_model_entry_rejected", False, dimensions)
                item["resource_changed_after_server_start"] = True
                report["scenarios"].append(item)
            finally:
                config_path.write_bytes(valid)
            report["scenarios"].append(verify_case(challenge, "repaired_model_entry_reaccepted", True, dimensions))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-28000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L04 scenario test finished")
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(0.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report.update(leftovers=leftovers, source_assets_unchanged=asset_hashes(lesson.folder) == source_assets,
                          elapsed_seconds=round(time.monotonic() - started, 3))
            if any(leftovers.values()) or not report["source_assets_unchanged"]:
                report["passed"] = False
            if args.report:
                path = Path(args.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
