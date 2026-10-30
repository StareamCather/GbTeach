"""L07 real friction directions and first-rebound observations.

Five private-session scenarios exercise the canonical verifier: the baseline,
changed friction/restitution/gravity, common Y friction directions, invalid
surfaces rejected before any physics control, and repaired surfaces. Actual
pose samples independently substantiate sliding and rebound peaks. Prediction
checks are explicitly separate from physics acceptance. Resetting is part of
the instrument; only pause and resource bytes are preserved, not old twist.
"""
import argparse
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
from tests.lesson05_acceptance import actual_pose
from tests.lesson06_acceptance import actual_stats, asset_hashes, named_model, world_data

DYNAMIC_NAMES = ("slider_low", "slider_high", "ball_dead", "ball_bouncy")
X_STATES = {"slider_low": "sliding", "slider_high": "held"}
Y_STATES = {"slider_low": "held", "slider_high": "held"}


def pose(node):
    values = [float(x) for x in node.findtext("pose", "0 0 0 0 0 0").split()]
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise AssertionError("An observation frame has an invalid pose")
    return values


def contact_geometry(world, ball_name):
    """Independent contact height from the exercise's horizontal pad and sphere."""
    pad_name = "pad_dead" if ball_name == "ball_dead" else "pad_bouncy"
    pad = named_model(world, pad_name)
    link = pad.find("link")
    collision = link.find("collision")
    frames = [pose(node) for node in (pad, link, collision)]
    if any(abs(angle) > 1e-8 for frame in frames for angle in frame[3:]):
        raise AssertionError("Rebound observation supports horizontal, unrotated pads")
    size = [float(x) for x in collision.findtext("geometry/box/size").split()]
    top = sum(frame[2] for frame in frames) + size[2] / 2
    ball = named_model(world, ball_name)
    ball_link = ball.find("link")
    ball_collision = ball_link.find("collision")
    radius = float(ball_collision.findtext("geometry/sphere/radius"))
    initial_z = sum(pose(node)[2] for node in (ball, ball_link, ball_collision))
    return {"pad_top_z": top, "radius": radius, "rest_center_z": top + radius,
            "initial_center_z": initial_z, "drop_height": initial_z - top - radius}


def independent_observations(workspace, measurements, expected_states, expected_restitution):
    """Recompute observations from measured poses, never from predicted peaks."""
    world = world_data(workspace / "world.sdf")
    ramp = pose(named_model(world, "friction_ramp"))
    if abs(ramp[3]) + abs(ramp[5]) > 1e-8:
        raise AssertionError("This course observes an incline rotated around Y only")
    down = [-math.cos(ramp[4]), 0, math.sin(ramp[4])]
    sliding = measurements["sliding"]
    projected = {}
    for name, state in expected_states.items():
        before, after = sliding["models"][name]["before"], sliding["models"][name]["after"]
        actual = sum((b-a)*axis for a, b, axis in zip(before[:3], after[:3], down))
        reported = sliding["models"][name]["measured_displacement_m"]
        if abs(actual - reported) > .0005:
            raise AssertionError("Reported sliding distance is not supported by actual poses for " + name)
        if sliding["models"][name]["predicted_state"] != state:
            raise AssertionError("Friction-axis case has an unexpected declared state for " + name)
        if (state == "sliding" and actual < .10) or (state == "held" and abs(actual) > .035):
            raise AssertionError("Actual pose change does not demonstrate {} {}: {}m".format(name, state, actual))
        projected[name] = {"actual_downslope_displacement_m": actual, "before": before, "after": after,
                           "expected_observed_state": state}

    gravity = abs(float(world.findtext("gravity").split()[2]))
    baseline_time = measurements["reset_baseline"]["sim_time_s"]
    bounce = measurements["bounce"]
    observed = {}
    for name in ("ball_dead", "ball_bouncy"):
        geometry = contact_geometry(world, name)
        impact_time = math.sqrt(2 * geometry["drop_height"] / gravity)
        samples = []
        for sample in bounce["samples"]:
            elapsed = sample["stats"]["sim_time_s"] - baseline_time
            measured = sample["poses"][name]
            if elapsed > impact_time + .015:
                samples.append({"time_since_reset_s": elapsed, "pose": measured})
        samples.sort(key=lambda sample: sample["time_since_reset_s"])
        if len(samples) < 3:
            raise AssertionError("Actual rebound samples do not cover the post-impact flight of " + name)
        maximum = max(range(len(samples)), key=lambda index: samples[index]["pose"][2])
        peak_z = samples[maximum]["pose"][2]
        rise = max(0.0, peak_z - geometry["rest_center_z"])
        source = bounce["models"][name]
        if abs(peak_z - source["sample_peak_z"]) > .003:
            raise AssertionError("Canonical sample peak differs from its actual pose samples for " + name)
        item = {"geometry": geometry, "actual_samples": samples, "sample_peak_z": peak_z,
                "sample_peak_rise_m": rise,
                "restitution_inferred_from_measured_sample_peak": math.sqrt(rise / geometry["drop_height"])}
        if name == "ball_bouncy":
            if maximum == 0 or maximum == len(samples)-1:
                raise AssertionError("Actual rebound peak is not bracketed on both sides")
            if peak_z - min(sample["pose"][2] for sample in samples[:maximum]) < .002 or \
                    peak_z - min(sample["pose"][2] for sample in samples[maximum+1:]) < .002:
                raise AssertionError("Actual rebound has no observable rise and fall around its peak")
            fitted_z = source["fitted_peak_z"]
            if not math.isfinite(fitted_z) or fitted_z < peak_z - .002 or fitted_z > peak_z + .025:
                raise AssertionError("Fitted rebound peak is unsupported by measured sample heights")
            if abs(item["restitution_inferred_from_measured_sample_peak"] - expected_restitution) > .10:
                raise AssertionError("Observed first rebound does not reflect the changed restitution")
            bracket = source["peak_bracket"]
            for sample_time, sample_z in bracket:
                if not any(abs(point["time_since_reset_s"]-sample_time) < 1e-7 and
                           abs(point["pose"][2]-sample_z) < 1e-7 for point in samples):
                    raise AssertionError("Canonical peak bracket is not made of actual recorded samples")
            fitted_time = source["fit"]["time_s"]
            if len(bracket) != 3 or not bracket[0][0] < fitted_time < bracket[2][0]:
                raise AssertionError("Fitted peak time is outside the actual sampled bracket")
            item.update(fitted_peak_z=fitted_z, fitted_peak_time_s=fitted_time, canonical_peak_bracket=bracket)
        elif rise > .035:
            raise AssertionError("The non-bouncing ball acquired a substantial rebound")
        observed[name] = item
    return {"sliding_from_actual_poses": projected, "bounce_from_actual_samples": observed}


def verify_case(session, name, expected_states=None, expected_restitution=.8, expected_failure=None):
    original = actual_stats(session)
    original_assets = asset_hashes(session.workspace)
    item = {"name": name, "original_stats": original,
            "state_contract": "instrument resets and keeps experiment results; restore pause and resource bytes only"}
    if expected_failure:
        if not original["paused"]:
            raise AssertionError("Invalid surface check must start in a paused valid world")
        item["poses_before_invalid_input"] = {model: actual_pose(session, model) for model in DYNAMIC_NAMES}
    try:
        offset = session.log_path.stat().st_size
        result = session.verify()
        item["verifier_result"] = result
        item["raw_verifier_output"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        if result.get("passed") is not (expected_failure is None):
            raise AssertionError("Unexpected canonical verifier result for " + name)
        after = actual_stats(session)
        item.update(stats_after=after, pause_restored=after["paused"] == original["paused"],
                    resource_bytes_unchanged=original_assets == asset_hashes(session.workspace))
        if not item["pause_restored"] or not item["resource_bytes_unchanged"]:
            raise AssertionError("Friction verifier changed original pause or resource bytes")
        if expected_failure:
            if "FAIL" not in item["raw_verifier_output"] or expected_failure not in item["raw_verifier_output"]:
                raise AssertionError("Invalid surface lacks a clear relevant diagnosis")
            if after["iterations"] != original["iterations"] or after["sim_time_sec_nsec"] != original["sim_time_sec_nsec"]:
                raise AssertionError("Invalid surface was rejected only after controlling physics")
            current = {model: actual_pose(session, model) for model in DYNAMIC_NAMES}
            item["poses_after_invalid_input"] = current
            for model, before in item["poses_before_invalid_input"].items():
                position_error = math.sqrt(sum((a-b)**2 for a, b in zip(before[:3], current[model][:3])))
                angle_error = max(abs((a-b+math.pi) % (2*math.pi)-math.pi)
                                  for a, b in zip(before[3:], current[model][3:]))
                if position_error > .002 or angle_error > .002:
                    raise AssertionError("Invalid surface handling reset or moved " + model)
            item["invalid_configuration_rejected_before_control"] = True
        else:
            lines = re.findall(r"^MEASUREMENTS_JSON(?:[=:])?\s+(\{.*\})\s*$", item["raw_verifier_output"], re.M)
            if not lines:
                raise AssertionError("Canonical verifier omitted its actual measurement JSON")
            measurements = json.loads(lines[-1])
            item["canonical_measurements"] = measurements
            item["independent_observations"] = independent_observations(
                session.workspace, measurements, expected_states, expected_restitution)
        item["passed"] = True
        return item
    finally:
        if session.running and actual_stats(session)["paused"] != original["paused"]:
            session.pause(original["paused"])


def challenge_files(workspace):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    world = tree.getroot().find("world")
    world.find("gravity").text = "0 0 -8"
    named_model(world, "slider_low").find("link/collision/surface/friction/ode/mu").text = ".2"
    named_model(world, "ball_bouncy").find("link/collision/surface/bounce/restitution_coefficient").text = ".6"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    return {"gravity": [0, 0, -8], "slider_low_mu": .2, "ball_bouncy_restitution": .6}


def direction_files(workspace):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    world = tree.getroot().find("world")
    for name in ("friction_ramp", "slider_low", "slider_high"):
        named_model(world, name).find("link/collision/surface/friction/ode/fdir1").text = "0 1 0"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    return {"common_fdir1": [0, 1, 0], "downslope_coefficient": "mu2", "observed_states": dict(Y_STATES)}


def compare_actual_cases(baseline, changed, directed):
    cases = [item["independent_observations"] for item in (baseline, changed, directed)]
    slides = [item["sliding_from_actual_poses"]["slider_low"]["actual_downslope_displacement_m"] for item in cases]
    peaks = [item["bounce_from_actual_samples"]["ball_bouncy"]["sample_peak_rise_m"] for item in cases]
    if not .1 < slides[1] < slides[0]*.85:
        raise AssertionError("Changed friction/gravity did not visibly reduce the real low-slider displacement")
    if abs(slides[2]) > .035:
        raise AssertionError("Selecting high mu2 via common Y direction did not prevent real sliding")
    if not peaks[0]*.25 < peaks[1] < peaks[0]*.85:
        raise AssertionError("Reduced restitution did not reduce the observed first rebound height")
    if abs(peaks[2]-peaks[0]) > max(.04, peaks[0]*.12):
        raise AssertionError("Friction-direction change unexpectedly altered the isolated ball rebound")
    return {"passed": True, "basis": "actual poses and actual rebound samples, not predicted distances/heights",
            "actual_low_slider_displacements_baseline_changed_direction_m": slides,
            "actual_first_rebound_rises_baseline_changed_direction_m": peaks}


def predict_configuration(workspace, expected_success=True, diagnostic=None):
    result = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"], cwd=str(workspace),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
    output = result.stdout + result.stderr
    if (result.returncode == 0) is not expected_success:
        raise AssertionError("Unexpected configuration prediction result: " + output)
    if diagnostic and diagnostic not in output:
        raise AssertionError("Configuration rejection lacks its range diagnosis: " + output)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr,
            "prediction": json.loads(result.stdout) if expected_success else None}


def edit_world(workspace, change):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    change(tree.getroot().find("world"))
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def set_surface(workspace, model, field, text):
    edit_world(workspace, lambda world: setattr(named_model(world, model).find(
        "link/collision/surface/" + field), "text", text))


def set_parameter(workspace, key, value):
    path = workspace / "parameters.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    data[key] = value
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def high_threshold(workspace):
    for name in ("ball_bouncy", "pad_bouncy"):
        set_surface(workspace, name, "bounce/threshold", "10")


def slanted_direction(workspace):
    for name in ("friction_ramp", "slider_low", "slider_high"):
        set_surface(workspace, name, "friction/ode/fdir1", ".7071067812 .7071067812 0")


def late_runoff(workspace):
    def change(world):
        ramp = pose(named_model(world, "friction_ramp"))
        model = named_model(world, "slider_low")
        current = pose(model)
        # Move two metres downhill. The slider still fits at slide_steps, but
        # would leave the usable ramp before the later rebound sample finishes.
        current[0] -= 2 * math.cos(ramp[4])
        current[2] += 2 * math.sin(ramp[4])
        model.find("pose").text = " ".join(str(x) for x in current)
    edit_world(workspace, change)


def initially_above_ramp_end(workspace):
    def change(world):
        ramp = pose(named_model(world, "friction_ramp"))
        model = named_model(world, "slider_high")
        current = pose(model)
        # Preserve normal gap and orientation while moving local u=1 to u=3.
        # High friction cannot hold an object with no initial ramp support.
        current[0] += 2 * math.cos(ramp[4])
        current[2] -= 2 * math.sin(ramp[4])
        model.find("pose").text = " ".join(str(x) for x in current)
    edit_world(workspace, change)


def low_global_correction_cap(workspace):
    edit_world(workspace, lambda world: setattr(world.find(
        "physics/ode/constraints/contact_max_correcting_vel"), "text", ".1"))


def low_paired_ball_correction_caps(workspace):
    for name in ("ball_dead", "ball_bouncy"):
        set_surface(workspace, name, "contact/ode/max_vel", ".1")


def configuration_regressions(lesson, private):
    """Configuration observability tests are not substitutes for real samples."""
    workspace = Storage(private / "invalid-inputs").prepare(lesson)
    original_world = (workspace / "world.sdf").read_bytes()
    original_parameters = (workspace / "parameters.json").read_bytes()
    original_assets = asset_hashes(workspace)
    report = {"kind": "configuration_only_not_physics_acceptance", "gazebo_started": False,
              "default_accepted": predict_configuration(workspace), "rejections": []}
    cases = (
        ("negative_mu", lambda p: set_surface(p, "slider_low", "friction/ode/mu", "-.1"), "摩擦系数"),
        ("non_finite_mu", lambda p: set_surface(p, "slider_low", "friction/ode/mu", "nan"), "有限数值"),
        ("restitution_greater_than_one", lambda p: set_surface(p, "ball_bouncy", "bounce/restitution_coefficient", "1.2"), "恢复系数"),
        ("conflicting_friction_axes", lambda p: set_surface(p, "slider_low", "friction/ode/fdir1", "0 1 0"), "fdir1须同"),
        ("unsupported_oblique_friction_axis", slanted_direction, "fdir1须同"),
        ("near_stick_slip_boundary", lambda p: set_surface(p, "slider_low", "friction/ode/mu", ".36"), "静止/滑移边界"),
        ("unobservable_short_slide", lambda p: set_parameter(p, "slide_steps", 50), "预测滑移不足"),
        ("unobservable_small_rebound", lambda p: set_surface(p, "ball_bouncy", "bounce/restitution_coefficient", ".05"), "反弹峰信号不足"),
        ("threshold_suppresses_instrumented_rebound", high_threshold, "有效threshold关闭或接近反弹窗口"),
        ("insufficient_peak_sampling_window", lambda p: set_parameter(p, "peak_half_window_s", .01), "peak_half_window_s"),
        ("ramp_exit_after_slide_before_final_peak_sample", late_runoff, "全部反弹采样结束前可能滑出坡面"),
        ("high_slider_initially_fully_beyond_upslope_end", initially_above_ramp_end, "初始坡向支撑超出坡面"),
        ("global_cap_truncates_rebound_velocity", low_global_correction_cap,
         "QuickStep反弹目标限幅：global contact_max_correcting_vel"),
        ("paired_dynamic_ball_caps_truncate_rebound_velocity", low_paired_ball_correction_caps,
         "QuickStep反弹目标限幅：ball_dead contact/max_vel"),
    )
    for name, change, diagnostic in cases:
        try:
            change(workspace)
            before = asset_hashes(workspace)
            item = predict_configuration(workspace, expected_success=False, diagnostic=diagnostic)
            if before != asset_hashes(workspace):
                raise AssertionError("Rejected configuration silently modified the exercise: " + name)
            item.update(name=name, resources_unchanged_on_rejection=True)
        finally:
            (workspace / "world.sdf").write_bytes(original_world)
            (workspace / "parameters.json").write_bytes(original_parameters)
        predict_configuration(workspace)
        item.update(repair_accepted=True, passed=True)
        report["rejections"].append(item)
    if original_assets != asset_hashes(workspace):
        raise AssertionError("Configuration regression did not restore original exercise bytes")
    changed = Storage(private / "changed-accepted").prepare(lesson)
    report["changed_parameters"] = challenge_files(changed)
    report["changed_accepted"] = predict_configuration(changed)
    directed = Storage(private / "Y-accepted").prepare(lesson)
    report["Y_parameters"] = direction_files(directed)
    report["Y_accepted"] = predict_configuration(directed)
    report.update(passed=True, all_invalid_resource_bytes_restored=True)
    return report


def configuration_only_main(lesson, args):
    source = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "kind": "configuration_only_not_physics_acceptance",
              "gazebo_started": False, "physical_scenarios_executed": 0}
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l07-configuration-") as directory:
        try:
            report["configuration_regressions"] = configuration_regressions(lesson, Path(directory))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report.update(source_assets_unchanged=source == asset_hashes(lesson.folder),
                          elapsed_seconds=round(time.monotonic()-started, 3))
            if not report["source_assets_unchanged"]:
                report["passed"] = False
            if args.report:
                path = Path(args.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


def invalid_surface_group(session):
    path = session.workspace / "world.sdf"
    valid = path.read_bytes()
    items = []
    invalid = (("negative_friction", "slider_low", "friction/ode/mu", "-.1", "摩擦"),
               ("out_of_range_restitution", "ball_bouncy", "bounce/restitution_coefficient", "1.2", "恢复"))
    for name, model, field, value, diagnostic in invalid:
        try:
            tree = ET.parse(str(path))
            named_model(tree.getroot().find("world"), model).find("link/collision/surface/" + field).text = value
            tree.write(str(path), encoding="utf-8", xml_declaration=True)
            items.append(verify_case(session, name, expected_failure=diagnostic))
        finally:
            path.write_bytes(valid)
        predict_configuration(session.workspace)
    return {"name": "invalid_surfaces_rejected_before_any_control", "passed": True,
            "kind": "configuration_rejection_inside_existing_real_session", "rejections": items,
            "valid_resource_bytes_restored_after_each_rejection": path.read_bytes() == valid}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    parser.add_argument("--configuration-only", action="store_true",
                        help="Only configuration/range guards; no Gazebo or physics acceptance")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:07"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L07 lesson; catalog errors: " + repr(errors))
    lesson = matches[0]
    if args.configuration_only:
        return configuration_only_main(lesson, args)
    original_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scenarios": [],
              "scope": "Classic 11 ODE, aligned X/Y friction axes and observable first vertical rebound; pause-only state restoration"}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l07-") as directory:
        private = Path(directory)
        try:
            baseline = LessonSession(ROOT, lesson, storage=Storage(private / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(False)
            baseline_case = verify_case(baseline, "baseline_real_sliding_and_first_rebound", X_STATES)
            report["scenarios"].append(baseline_case)
            baseline.stop(reason="L07 baseline accepted")

            storage = Storage(private / "changed")
            workspace = storage.prepare(lesson)
            changes = challenge_files(workspace)
            changed = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(changed)
            changed.start(gui=False, terminals=False)
            changed.pause(True)
            changed_case = verify_case(changed, "changed_friction_restitution_and_gravity", X_STATES, .6)
            changed_case["parameters"] = changes
            report["scenarios"].append(changed_case)
            changed.stop(reason="L07 parameter variant accepted")

            storage = Storage(private / "directions")
            workspace = storage.prepare(lesson)
            directions = direction_files(workspace)
            directed = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(directed)
            directed.start(gui=False, terminals=False)
            directed.pause(True)
            direction_case = verify_case(directed, "common_Y_friction_direction_prevents_both_sliders", Y_STATES)
            direction_case["parameters"] = directions
            report["scenarios"].append(direction_case)
            report["actual_cross_case_effects"] = compare_actual_cases(baseline_case, changed_case, direction_case)
            report["scenarios"].append(invalid_surface_group(directed))
            report["scenarios"].append(verify_case(directed, "repaired_surfaces_reaccepted", Y_STATES))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-45000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L07 private acceptance finished")
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report.update(leftovers=leftovers, source_assets_unchanged=original_assets == asset_hashes(lesson.folder),
                          elapsed_seconds=round(time.monotonic()-started, 3))
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
