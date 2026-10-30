"""L06: real mass, center-of-mass, and inertia experiments in private worlds.

Each positive case explicitly resets this experiment's models and independently
measures a fixed-step free fall. It then runs the canonical three-station
verifier and retains its measured result. Resetting and stepping intentionally
change the experiment state; only pause state and resource bytes are restored,
never an alleged reconstruction of the previous linear/angular velocities.
The negative case changes private XML after a valid server is running and must
be rejected before either stepping or resetting that paused simulation.
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
from tests.lesson05_acceptance import actual_pose, command, first, parsed_model, short_name

FALL_NAMES = ("fall_light", "fall_heavy")
STATION_NAMES = FALL_NAMES + ("tower_low", "tower_high", "flywheel_solid", "flywheel_ring")


def asset_hashes(folder):
    paths = [folder / "world.sdf", folder / "parameters.json"] + sorted((folder / "models").rglob("*"))
    return {path.relative_to(folder).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths if path.is_file()}


def actual_stats(session):
    output = command(session, "gz", "topic", "-e", "/gazebo/{}/world_stats".format(session.world_name),
                     "-d", "1", read_only=True)
    paused = re.findall(r"\bpaused:\s*(true|false)\b", output)
    iterations = re.findall(r"\biterations:\s*([0-9]+)\b", output)
    clocks = re.findall(r"\bsim_time\s*\{\s*sec:\s*([0-9]+)\s*nsec:\s*([0-9]+)\s*\}", output)
    if not paused or not iterations or not clocks:
        raise AssertionError("Actual world statistics lack pause, iterations, or simulation time: " + output[-1800:])
    sec, nsec = (int(value) for value in clocks[-1])
    return {"paused": paused[-1] == "true", "iterations": int(iterations[-1]),
            "sim_time": sec + nsec / 1e9, "sim_time_sec_nsec": [sec, nsec]}


def world_data(path):
    world = ET.parse(str(path)).getroot().find("world")
    if world is None:
        raise AssertionError("Current exercise has no world")
    return world


def named_model(world, name):
    matches = world.findall("model[@name='{}']".format(name))
    if len(matches) != 1:
        raise AssertionError("Expected one model " + name)
    return matches[0]


def loaded_inertials(session):
    """Compare actual Model inertials with the current workspace configuration."""
    world = world_data(session.workspace / "world.sdf")
    evidence = {}
    for name in STATION_NAMES:
        source_model = named_model(world, name)
        message = parsed_model(command(session, "gz", "model", "-w", session.world_name,
                                       "-m", name, "-i", read_only=True))
        links = {short_name(first(link, "name")): link for link in message.get("link", [])}
        observed = []
        for source_link in source_model.findall("link"):
            inertial = source_link.find("inertial")
            if inertial is None:
                continue
            link_name = source_link.get("name")
            if link_name not in links:
                raise AssertionError("Actual {} lacks source link {}".format(name, link_name))
            actual = first(links[link_name], "inertial", {})
            values = {}
            for label, relative in (("mass", "mass"), ("ixx", "inertia/ixx"),
                                    ("iyy", "inertia/iyy"), ("izz", "inertia/izz")):
                expected = float(inertial.findtext(relative))
                measured = float(first(actual, label, "nan"))
                if not math.isfinite(measured) or abs(measured - expected) > max(abs(expected) * 1e-4, 1e-7):
                    raise AssertionError("Loaded {} {} {}={} differs from exercise {}".format(
                        name, link_name, label, measured, expected))
                values[label] = {"actual": measured, "source": expected}
            raw_pose = [float(x) for x in inertial.findtext("pose", "0 0 0 0 0 0").split()]
            center = first(first(actual, "pose", {}), "position", {})
            actual_center = [float(first(center, axis, "nan")) for axis in ("x", "y", "z")]
            if any(not math.isfinite(a) or abs(a - b) > 1e-5 for a, b in zip(actual_center, raw_pose[:3])):
                raise AssertionError("Loaded {} {} center of mass differs: {} != {}".format(
                    name, link_name, actual_center, raw_pose[:3]))
            observed.append({"link": link_name, "actual_center_of_mass": actual_center,
                             "source_center_of_mass": raw_pose[:3], "properties": values})
        if not observed:
            raise AssertionError("No actual inertial evidence for " + name)
        evidence[name] = observed
    return evidence


def independent_freefall(session, steps=200):
    """Reset deliberately, step actual physics, and estimate acceleration."""
    world = world_data(session.workspace / "world.sdf")
    gravity = [float(x) for x in world.findtext("gravity", "0 0 -9.81").split()]
    if len(gravity) != 3 or max(abs(x) for x in gravity[:2]) > 1e-9 or gravity[2] >= 0:
        raise AssertionError("Free-fall experiment requires vertical downward gravity")
    step_size = float(world.findtext("physics/max_step_size"))
    expected_start = {name: [float(x) for x in named_model(world, name).findtext("pose").split()]
                      for name in FALL_NAMES}
    original = actual_stats(session)["paused"]
    try:
        session.pause(True)
        session.reset_models()
        deadline = time.monotonic() + 8
        while True:
            initial = {name: actual_pose(session, name) for name in FALL_NAMES}
            if all(max(abs(a - b) for a, b in zip(initial[name][:3], expected_start[name][:3])) < 0.005
                   for name in FALL_NAMES):
                break
            if time.monotonic() >= deadline:
                raise AssertionError("Explicit reset did not put both free-fall models at their initial positions")
        before = actual_stats(session)
        session.step(steps)
        deadline = time.monotonic() + 8
        while True:
            after = actual_stats(session)
            if after["iterations"] >= before["iterations"] + steps:
                break
            if time.monotonic() >= deadline:
                raise AssertionError("The requested fixed physics steps did not complete")
        actual_steps = after["iterations"] - before["iterations"]
        if actual_steps != steps or not after["paused"]:
            raise AssertionError("Fixed-step experiment advanced an unexpected number of iterations")
        elapsed = after["sim_time"] - before["sim_time"]
        if abs(elapsed - steps * step_size) > max(step_size * 0.1, 1e-6):
            raise AssertionError("Simulation time is inconsistent with actual fixed steps")
        final = {name: actual_pose(session, name) for name in FALL_NAMES}
        drops = {name: initial[name][2] - final[name][2] for name in FALL_NAMES}
        expected_drop = -0.5 * gravity[2] * elapsed * elapsed
        # ODE uses a discrete integrator. This bound admits its O(dt) endpoint
        # difference, but excludes no fall, floor contact, or mass-dependent g.
        tolerance = max(0.006, abs(gravity[2]) * elapsed * step_size * 2)
        if expected_drop < 0.03 or any(abs(drop - expected_drop) > tolerance for drop in drops.values()):
            raise AssertionError("Actual free-fall displacement disagrees with g: {} expected {} +/- {}".format(
                drops, expected_drop, tolerance))
        if abs(drops[FALL_NAMES[0]] - drops[FALL_NAMES[1]]) > 0.003:
            raise AssertionError("Equal-shape unequal-mass models acquired different free-fall accelerations")
        return {"explicit_reset_of_all_lesson_models": True, "previous_twist_not_restored": True,
                "stats_before": before, "stats_after": after, "initial_poses": initial, "final_poses": final,
                "gravity": gravity, "expected_drop": expected_drop, "tolerance": tolerance,
                "actual_drops": drops, "estimated_accelerations": {
                    name: 2 * drop / (elapsed * elapsed) for name, drop in drops.items()}}
    finally:
        if session.running:
            session.pause(original)


def queue_native_wrench(session):
    """Queue root-confirmed Classic Link input while physics stays paused."""
    before = actual_stats(session)
    if not before["paused"]:
        raise AssertionError("Historical-wrench regression requires a paused world")
    model = "flywheel_solid"
    world = world_data(session.workspace / "world.sdf")
    link = named_model(world, model).find("link").get("name").replace("::", "/")
    topic = "/gazebo/{}/{}/{}/wrench".format(session.world_name, model, link)
    wrench = "force: {x: 0 y: 0 z: 0} torque: {x: 0 y: 0 z: 50}"
    pose_before = actual_pose(session, model)
    command(session, "gz", "topic", "-p", topic, "-m", wrench)
    after = actual_stats(session)
    if not after["paused"] or after["iterations"] != before["iterations"] or after["sim_time_sec_nsec"] != before["sim_time_sec_nsec"]:
        raise AssertionError("Queuing a historical wrench unexpectedly advanced physics")
    return {"model": model, "topic": topic, "message_type": "gazebo.msgs.Wrench", "message": wrench,
            "torque_z_Nm": 50.0, "no_manual_step_after_publication": True,
            "stats_before": before, "stats_after": after, "pose_before": pose_before,
            "pose_after": actual_pose(session, model)}


def verify_case(session, name, should_pass=True, queued_wrench=False):
    before_assets = asset_hashes(session.workspace)
    before = actual_stats(session)
    item = {"name": name, "expected_verifier_pass": should_pass, "original_stats": before,
            "restoration_contract": "restore pause and resource bytes; reset experiment results remain"}
    try:
        if should_pass:
            item["actual_loaded_inertials"] = loaded_inertials(session)
            item["independent_fixed_step_freefall"] = independent_freefall(session)
            if queued_wrench:
                item["queued_wrench_before_verifier"] = queue_native_wrench(session)
        else:
            item["invalid_case_model_poses_before"] = {model: actual_pose(session, model) for model in STATION_NAMES}
        offset = session.log_path.stat().st_size
        result = session.verify()
        item["verifier_result"] = result
        item["evidence"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        if result.get("passed") is not should_pass:
            raise AssertionError("Unexpected canonical verifier result for " + name)
        after = actual_stats(session)
        item.update(actual_stats_after=after, pause_restored=after["paused"] == before["paused"])
        if not item["pause_restored"]:
            raise AssertionError("Mass/inertia verifier did not restore the original pause state")
        item["assets_unchanged"] = before_assets == asset_hashes(session.workspace)
        item["asset_hashes"] = before_assets
        if not item["assets_unchanged"]:
            raise AssertionError("Verifier rewrote the world's physical parameters")
        if should_pass:
            missing = [model for model in STATION_NAMES if model not in item["evidence"]]
            if missing:
                raise AssertionError("Three-station verifier lacks named actual observations: " + repr(missing))
            measurement_lines = re.findall(r"^MEASUREMENTS_JSON(?:[=:])?\s+(\{.*\})\s*$", item["evidence"], re.M)
            if not measurement_lines:
                raise AssertionError("Three-station verifier omitted its raw MEASUREMENTS_JSON record")
            item["canonical_measurements"] = json.loads(measurement_lines[-1])
            if queued_wrench:
                drain = item["canonical_measurements"].get("historical_input_drain", {})
                if drain.get("requested_steps") != 1:
                    raise AssertionError("Historical-wrench regression lacks an acknowledged drain step before reset")
                rotor = item["canonical_measurements"]["flywheels"]["models"]["flywheel_solid"]
                actual_omega = rotor["measured_omega_rad_s"]
                predicted_omega = rotor["predicted_omega_rad_s"]
                if abs(actual_omega - predicted_omega) > max(.002, abs(predicted_omega) * .04):
                    raise AssertionError("Historical queued wrench contaminated the new measured angular velocity")
                item["queued_wrench_did_not_contaminate_formal_measurement"] = True
        else:
            if "惯性" not in item["evidence"] or "FAIL" not in item["evidence"]:
                raise AssertionError("Invalid inertia was rejected without a clear inertia diagnosis")
            if not before["paused"]:
                raise AssertionError("The no-mutation negative case must start paused")
            item["invalid_input_did_not_advance_or_reset_world"] = (
                before["sim_time_sec_nsec"] == after["sim_time_sec_nsec"] and before["iterations"] == after["iterations"])
            if not item["invalid_input_did_not_advance_or_reset_world"]:
                raise AssertionError("Verifier controlled physics before rejecting invalid inertia")
            after_poses = {model: actual_pose(session, model) for model in STATION_NAMES}
            item["invalid_case_model_poses_after"] = after_poses
            errors = {}
            for model, original_pose in item["invalid_case_model_poses_before"].items():
                measured = after_poses[model]
                position_error = math.sqrt(sum((a-b)**2 for a, b in zip(measured[:3], original_pose[:3])))
                angle_error = max(abs((a-b+math.pi) % (2*math.pi)-math.pi)
                                  for a, b in zip(measured[3:], original_pose[3:]))
                errors[model] = {"position_error": position_error, "angle_error": angle_error}
            item["invalid_case_pose_errors"] = errors
            if any(value["position_error"] > .002 or value["angle_error"] > .002 for value in errors.values()):
                raise AssertionError("Invalid-input verifier reset or moved models despite not advancing the clock")
        item["passed"] = True
        return item
    finally:
        if session.running and actual_stats(session)["paused"] != before["paused"]:
            session.pause(before["paused"])


def challenge_files(workspace):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    world = tree.getroot().find("world")
    world.find("gravity").text = "0 0 -6"
    masses = {"fall_light": 0.25, "fall_heavy": 8.0}
    for name, mass in masses.items():
        model = named_model(world, name)
        values = [float(x) for x in model.findtext("pose").split()]
        values[2] = 2.5
        model.find("pose").text = " ".join(str(x) for x in values)
        link = model.find("link")
        dimensions = [float(x) for x in link.findtext("collision/geometry/box/size").split()]
        x, y, z = dimensions
        link.find("inertial/mass").text = str(mass)
        for axis, value in (("ixx", mass * (y*y + z*z) / 12), ("iyy", mass * (x*x + z*z) / 12),
                            ("izz", mass * (x*x + y*y) / 12)):
            link.find("inertial/inertia/" + axis).text = str(value)
    flywheel_masses = {}
    for name in ("flywheel_solid", "flywheel_ring"):
        link = named_model(world, name).find("link")
        old_mass = float(link.findtext("inertial/mass"))
        new_mass = old_mass * 1.5
        link.find("inertial/mass").text = str(new_mass)
        for field in link.findall("inertial/inertia/*"):
            field.text = str(float(field.text) * 1.5)
        flywheel_masses[name] = new_mass
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    return {"vertical_gravity": -6.0, "fall_masses": masses, "fall_initial_z": 2.5,
            "flywheel_masses": flywheel_masses, "flywheel_tensor_scale": 1.5}


def angle_aliasing_regression(workspace):
    """Validate an instrument's sampling guard; this is not physics evidence."""
    path = workspace / "parameters.json"
    original = path.read_bytes()
    parameters = json.loads(original.decode("utf-8"))
    modified = dict(parameters, torque_z_Nm=200.0, flywheel_coast_steps=5000)
    world = world_data(workspace / "world.sdf")
    dt = float(world.findtext("physics/max_step_size"))
    izz = float(named_model(world, "flywheel_solid").findtext("link/inertial/inertia/izz"))
    predicted_angle = modified["torque_z_Nm"] * dt * dt * modified["flywheel_coast_steps"] / izz
    if predicted_angle < 2.8:
        raise AssertionError("Aliasing negative setup does not exceed the sampling bound")
    original_assets = asset_hashes(workspace)
    try:
        path.write_text(json.dumps(modified, ensure_ascii=False, indent=2), encoding="utf-8")
        before_rejection = asset_hashes(workspace)
        rejected = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"],
                                  cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
        diagnostic = rejected.stdout + rejected.stderr
        if rejected.returncode == 0 or "2.8" not in diagnostic or "混叠" not in diagnostic:
            raise AssertionError("High predicted coast angle lacks a clear sampling-alias diagnosis: " + diagnostic)
        if before_rejection != asset_hashes(workspace):
            raise AssertionError("Rejected aliasing parameters were silently modified")
    finally:
        path.write_bytes(original)
    repaired = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"],
                              cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
    if repaired.returncode:
        raise AssertionError("Restored sampling parameters remain invalid: " + repaired.stderr)
    json.loads(repaired.stdout)
    if original_assets != asset_hashes(workspace):
        raise AssertionError("Configuration-only alias regression did not restore resource bytes")
    return {"passed": True, "kind": "configuration_only_not_physics_acceptance",
            "parameters": {"torque_z_Nm": 200.0, "flywheel_coast_steps": 5000},
            "independently_predicted_coast_angle_rad": predicted_angle, "sampling_bound_rad": 2.8,
            "rejected_returncode": rejected.returncode, "diagnostic": diagnostic,
            "repair_configuration_accepted": True, "resources_restored": True}


def predict_configuration(workspace):
    result = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"],
                            cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
    if result.returncode:
        raise AssertionError("Valid sampling configuration was rejected: " + result.stdout + result.stderr)
    return json.loads(result.stdout)


def sampling_window_regressions(workspace):
    """Reject unobservable short windows without starting or controlling Gazebo."""
    path = workspace / "parameters.json"
    original = path.read_bytes()
    parameters = json.loads(original.decode("utf-8"))
    original_assets = asset_hashes(workspace)
    default_predictions = predict_configuration(workspace)
    cases = []
    for key, overrides, concepts in (
            ("freefall_steps", {"freefall_steps": 20}, ("自由落体采样落差不足",)),
            ("tower_steps", {"tower_steps": 20}, ("倾覆观察窗口不足",)),
            ("flywheel_coast_steps", {"flywheel_coast_steps": 20}, ("coast采样转角不足",)),
            ("weak_angular_speed", {"torque_z_Nm": 1.0, "flywheel_coast_steps": 5000}, ("角速度采样信号不足",))):
        try:
            modified = dict(parameters, **overrides)
            path.write_text(json.dumps(modified, ensure_ascii=False, indent=2), encoding="utf-8")
            before_rejection = asset_hashes(workspace)
            result = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"],
                                    cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
            diagnostic = result.stdout + result.stderr
            if result.returncode == 0 or "FAIL" not in diagnostic or not any(concept in diagnostic for concept in concepts):
                raise AssertionError("Unobservable {}={} lacks its sampling-range diagnosis: {}".format(key, overrides, diagnostic))
            if before_rejection != asset_hashes(workspace):
                raise AssertionError("Sampling-window rejection changed resource bytes")
            item = {"name": key, "parameter_overrides": overrides, "rejected_returncode": result.returncode,
                    "diagnostic": diagnostic, "rejection_did_not_modify_parameters": True}
        finally:
            path.write_bytes(original)
        predict_configuration(workspace)
        item.update(repair_configuration_accepted=True, passed=True)
        cases.append(item)
    if original_assets != asset_hashes(workspace):
        raise AssertionError("Sampling-window regressions did not restore the original resources")
    return {"passed": True, "kind": "configuration_only_not_physics_acceptance",
            "default_predictions_accepted": default_predictions, "short_window_cases": cases[:3],
            "long_coast_but_weak_angular_speed_case": cases[3],
            "resources_restored": True}


def configuration_only_main(lesson, args):
    """Keep a focused maintenance run distinct from the four physics cases."""
    started = time.monotonic()
    original_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "kind": "configuration_only_not_physics_acceptance",
              "gazebo_started": False, "physical_scenarios_executed": 0}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l06-configuration-") as directory:
        try:
            root = Path(directory)
            baseline = Storage(root / "baseline").prepare(lesson)
            report["angle_aliasing_regression"] = angle_aliasing_regression(baseline)
            report["sampling_window_regressions"] = sampling_window_regressions(baseline)
            variant = Storage(root / "original-g6-challenge").prepare(lesson)
            report["challenge_parameters"] = challenge_files(variant)
            report["original_g6_challenge_predictions_accepted"] = predict_configuration(variant)
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report.update(source_assets_unchanged=original_assets == asset_hashes(lesson.folder),
                          elapsed_seconds=round(time.monotonic() - started, 3))
            if not report["source_assets_unchanged"]:
                report["passed"] = False
            if args.report:
                path = Path(args.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    parser.add_argument("--configuration-only", action="store_true",
                        help="Only configuration observability/aliasing checks; no Gazebo or physics acceptance")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:06"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L06 course; catalog errors: " + repr(errors))
    lesson = matches[0]
    if args.configuration_only:
        return configuration_only_main(lesson, args)
    source_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l06-") as directory:
        private = Path(directory)
        try:
            guard_workspace = Storage(private / "aliasing-guard").prepare(lesson)
            report["angle_aliasing_regression"] = angle_aliasing_regression(guard_workspace)
            report["sampling_window_regressions"] = sampling_window_regressions(guard_workspace)
            baseline = LessonSession(ROOT, lesson, storage=Storage(private / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(False)
            report["scenarios"].append(verify_case(baseline, "baseline_three_actual_physics_stations"))
            baseline.stop(reason="L06 baseline physics accepted")

            storage = Storage(private / "challenge")
            workspace = storage.prepare(lesson)
            parameters = challenge_files(workspace)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            challenge.pause(True)
            item = verify_case(challenge, "changed_gravity_fall_masses_height_and_flywheel_inertias", queued_wrench=True)
            item["challenge"] = parameters
            report["scenarios"].append(item)

            path = workspace / "world.sdf"
            valid = path.read_bytes()
            try:
                tree = ET.parse(str(path))
                bad = named_model(tree.getroot().find("world"), "flywheel_solid").find("link/inertial/inertia/izz")
                if bad is None:
                    raise AssertionError("The lesson's solid flywheel lacks a configurable inertia tensor")
                bad.text = "-0.01"
                tree.write(str(path), encoding="utf-8", xml_declaration=True)
                report["scenarios"].append(verify_case(challenge, "negative_inertia_rejected_before_physics_control", False))
            finally:
                path.write_bytes(valid)
            report["scenarios"].append(verify_case(challenge, "repaired_inertia_reaccepted_with_actual_measurements"))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-40000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L06 private acceptance finished")
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
