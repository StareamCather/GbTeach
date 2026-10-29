"""L02: real visual/collision scenarios in private workspaces.

The baseline and changed-world cases each start an actual gzserver. The invalid
case edits only the private configuration after a valid world is running, so it
checks a clear verifier diagnostic without asking Gazebo to load bad geometry.
Every verifier call must preserve the actual original pause state.
"""
import argparse
import copy
import json
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


def command(session, *arguments, timeout=7):
    result = subprocess.run(list(arguments), cwd=str(session.workspace), env=session.env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
    if result.returncode:
        raise AssertionError("Command failed: {}\n{}".format(" ".join(arguments), result.stderr[-1500:]))
    return result.stdout


def actual_paused(session):
    output = command(session, "gz", "topic", "-e", "/gazebo/{}/world_stats".format(session.world_name), "-d", "1")
    values = re.findall(r"\bpaused:\s*(true|false)\b", output)
    if not values:
        raise AssertionError("No actual pause state received from this world's statistics")
    return values[-1] == "true"


def model_pose(session, name):
    output = command(session, "gz", "model", "-w", session.world_name, "-m", name, "-p")
    for line in reversed(output.splitlines()):
        values = line.split()
        if len(values) == 6:
            try:
                return [float(value) for value in values]
            except ValueError:
                pass
    raise AssertionError("No measured pose for {}: {}".format(name, output))


def challenge_world(path):
    tree = ET.parse(str(path))
    world = tree.getroot().find("world")
    solid = world.find("model[@name='solid_platform']")
    ghost = world.find("model[@name='ghost_platform']/link")
    if solid is None or ghost is None:
        raise AssertionError("L02 expected solid_platform and ghost_platform")
    if ghost.find("collision") is not None:
        raise AssertionError("Baseline ghost platform unexpectedly already has collision")
    visual = ghost.find("visual")
    geometry = visual.find("geometry") if visual is not None else None
    if geometry is None:
        raise AssertionError("Ghost visual geometry is missing")
    collision = ET.SubElement(ghost, "collision", {"name": "challenge_collision"})
    if visual.find("pose") is not None:
        collision.append(copy.deepcopy(visual.find("pose")))
    collision.append(copy.deepcopy(geometry))
    pose = solid.find("pose")
    values = pose.text.split()
    values[2] = "1.4"
    pose.text = " ".join(values)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def invalid_world(path):
    tree = ET.parse(str(path))
    size = tree.getroot().find("world/model[@name='solid_platform']/link/collision/geometry/box/size")
    if size is None:
        raise AssertionError("Solid platform collision box is missing")
    size.text = "1 1 -0.2"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def verifier_evidence(session, name, should_pass, expected_z=None):
    original = actual_paused(session)
    offset = session.log_path.stat().st_size
    item = {"name": name, "expected_verifier_pass": should_pass, "original_paused": original}
    try:
        result = session.verify()
        item["verifier_result"] = result
        item["evidence"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        restored = actual_paused(session)
        item["actual_paused_after"] = restored
        item["pause_restored"] = restored == original
        if result.get("passed") is not should_pass:
            raise AssertionError("{} returned unexpected verification result".format(name))
        if restored != original:
            raise AssertionError("{} changed the original pause state".format(name))
        if should_pass:
            measured = {model: model_pose(session, model) for model in expected_z}
            item["measured_poses"] = measured
            item["expected_center_z"] = expected_z
            for model, height in expected_z.items():
                if abs(measured[model][2] - height) > 0.045:
                    raise AssertionError("{} measured z={} instead of {}".format(model, measured[model][2], height))
                if model not in item["evidence"] or "测量" not in item["evidence"]:
                    raise AssertionError("{} has no actual measurement in the canonical verifier log".format(model))
        else:
            if "box尺寸必须为正" not in item["evidence"]:
                raise AssertionError("Invalid geometry was not rejected with the expected clear size diagnostic")
        item["passed"] = True
        return item
    finally:
        # Preserve user-visible state even when a regression caused the verifier
        # itself to fail to restore it. Such a regression still fails this case.
        if session.running:
            now = actual_paused(session)
            if now != original:
                session.pause(original)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    arguments = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    candidates = [lesson for lesson in lessons if lesson.key == "classic11-noetic:02"]
    if len(candidates) != 1:
        raise RuntimeError("Expected one L02 course; catalog errors: " + repr(errors))
    lesson = candidates[0]
    report = {"passed": False, "lesson": lesson.key, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l02-") as directory:
        base = Path(directory)
        try:
            baseline = LessonSession(ROOT, lesson, storage=Storage(base / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(True)
            report["scenarios"].append(verifier_evidence(
                baseline, "baseline_visual_is_not_collision", True,
                {"solid_probe": 1.25, "ghost_probe": 0.15}))
            baseline.stop(reason="L02 baseline accepted")

            storage = Storage(base / "challenge")
            workspace = storage.prepare(lesson)
            path = workspace / lesson.data.get("world", "world.sdf")
            challenge_world(path)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            if actual_paused(challenge):
                raise AssertionError("Fresh challenge world unexpectedly starts paused")
            report["scenarios"].append(verifier_evidence(
                challenge, "challenge_added_collision_and_raised_platform", True,
                {"solid_probe": 1.65, "ghost_probe": 1.25}))

            challenge.pause(True)
            valid_bytes = path.read_bytes()
            try:
                invalid_world(path)
                item = verifier_evidence(challenge, "negative_collision_size_rejected", False)
                item["configuration_changed_after_server_start"] = True
                report["scenarios"].append(item)
            finally:
                path.write_bytes(valid_bytes)
            report["scenarios"].append(verifier_evidence(
                challenge, "repaired_configuration_reaccepted", True,
                {"solid_probe": 1.65, "ghost_probe": 1.25}))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-24000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L02 scenario test finished")
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(0.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report["leftovers"] = leftovers
            if any(leftovers.values()):
                report["passed"] = False
            report["elapsed_seconds"] = round(time.monotonic() - started, 3)
            if arguments.report:
                path = Path(arguments.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
