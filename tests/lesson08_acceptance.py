"""L08: real sampled contact overlap, settlement, and solver configurations.

Every positive scenario runs the canonical instrument in a private world. The
tests independently recompute pose-derived overlap, mean/range/std, and the
equilibrium-reference difference from its actual samples. Those observations
are not ODE's internal LCP residual or a complete vibration spectrum. Solver
variants do not have to be less accurate, noisier, or faster than the baseline.
Explicit resets retain experiment results; only pause/resource bytes restore.
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

PROBES = ("probe_soft", "probe_firm")


def source_pose(node):
    values = [float(x) for x in node.findtext("pose", "0 0 0 0 0 0").split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise AssertionError("Contact geometry has an invalid pose")
    return values


def support_geometry(world, name):
    probe = named_model(world, name)
    support = named_model(world, "support_" + name.split("_")[-1])
    support_link = support.find("link")
    support_collision = support_link.find("collision")
    frames = [source_pose(node) for node in (support, support_link, support_collision)]
    if any(abs(angle) > 1e-8 for frame in frames for angle in frame[3:]):
        raise AssertionError("This course measures contact with horizontal, unrotated supports")
    size = [float(x) for x in support_collision.findtext("geometry/box/size").split()]
    top = sum(frame[2] for frame in frames) + size[2]/2
    radius = float(probe.findtext("link/collision/geometry/sphere/radius"))
    return {"support_top_z": top, "radius_m": radius, "unpenetrated_center_z": top + radius,
            "initial_probe_pose": source_pose(probe)}


def close_metric(actual, reported, label, tolerance=1e-8):
    if not math.isfinite(actual) or not math.isfinite(reported) or abs(actual-reported) > tolerance:
        raise AssertionError("{} is not supported by raw samples: {} != {}".format(label, reported, actual))


def independent_metrics(workspace, measurements):
    world = world_data(workspace / "world.sdf")
    parameters = json.loads((workspace / "parameters.json").read_text(encoding="utf-8"))
    repeats = measurements["repeats"]
    if len(repeats) != parameters["repeats"] or len(repeats) < 2:
        raise AssertionError("Repeatability demonstration needs two actual experimental repetitions")
    output = []
    for index, repeat in enumerate(repeats):
        samples = repeat["samples"]
        if len(samples) != parameters["tail_samples"] or len(samples) < 3:
            raise AssertionError("Too few real settled samples to describe dispersion")
        times = [sample["stats"]["sim_time_s"] for sample in samples]
        if not all(a < b for a, b in zip(times, times[1:])):
            raise AssertionError("Contact samples have not advanced actual simulation time")
        if any(not sample["stats"]["paused"] for sample in samples):
            raise AssertionError("Contact instrument sampled while physics was uncontrolled")
        close_metric(times[0]-repeat["reset_baseline"]["sim_time_s"], parameters["warmup_time_s"],
                     "actual warmup simulation time")
        for first, second in zip(times, times[1:]):
            close_metric(second-first, parameters["tail_interval_s"], "actual tail sample interval")
        item = {"repeat": index, "actual_sample_times_s": times, "models": {}}
        for name in PROBES:
            if any(sample.get("enabled", {}).get(name) is not True for sample in samples):
                raise AssertionError("Sampled contact stability cannot be substituted by a sleeping/disabled link: " + name)
            geometry = support_geometry(world, name)
            poses = [sample["poses"][name] for sample in samples]
            if any(len(pose) != 6 or not all(math.isfinite(value) for value in pose) for pose in poses):
                raise AssertionError("Contact observations contain non-finite or missing poses")
            z = [pose[2] for pose in poses]
            overlaps = [geometry["unpenetrated_center_z"]-height for height in z]
            mean_overlap = sum(overlaps)/len(overlaps)
            mean_z = sum(z)/len(z)
            range_z = max(z)-min(z)
            std_z = math.sqrt(sum((height-mean_z)**2 for height in z)/len(z))
            declared = repeat["metrics"][name]
            if len(declared["overlap_samples_m"]) != len(overlaps):
                raise AssertionError("Reported contact overlaps omit actual samples")
            for actual, reported in zip(overlaps, declared["overlap_samples_m"]):
                close_metric(actual, reported, name + " sampled overlap")
            close_metric(mean_overlap, declared["mean_overlap_m"], name + " mean overlap")
            close_metric(range_z, declared["range_z_m"], name + " sampled range")
            close_metric(std_z, declared["std_z_m"], name + " sampled population standard deviation")
            reference_difference = mean_overlap-declared["predicted_overlap_m"]
            close_metric(reference_difference, declared["mean_error_m"], name + " geometric reference difference")
            if mean_overlap < -.005 or max(overlaps) > geometry["radius_m"]*.45:
                raise AssertionError("The measured sphere is detached or excessively penetrating: " + name)
            if any(math.hypot(pose[0]-geometry["initial_probe_pose"][0],
                              pose[1]-geometry["initial_probe_pose"][1]) > .02 for pose in poses):
                raise AssertionError("The measured sphere left its intended support centre: " + name)
            item["models"][name] = {"geometry": geometry, "actual_poses": poses,
                                     "overlap_samples_m": overlaps, "mean_overlap_m": mean_overlap,
                                     "sampled_range_z_m": range_z, "sampled_population_std_z_m": std_z,
                                     "settlement_from_initial_center_m": geometry["initial_probe_pose"][2]-mean_z,
                                     "difference_from_declared_equilibrium_reference_m": reference_difference}
        if item["models"]["probe_soft"]["mean_overlap_m"] <= item["models"]["probe_firm"]["mean_overlap_m"]+.005:
            raise AssertionError("Actual soft/firm contact observations lack the intended compliance contrast")
        output.append(item)
    differences = {}
    for name in PROBES:
        actual = abs(output[1]["models"][name]["mean_overlap_m"]-output[0]["models"][name]["mean_overlap_m"])
        close_metric(actual, measurements["repeatability"][name]["mean_overlap_difference_m"],
                     name + " same-session repeat difference")
        differences[name] = actual
    return {"definition": "pose-derived contact geometry and sampled dispersion; not internal LCP residuals",
            "quality_ranking_between_solver_variants_required": False, "repeats": output,
            "actual_repeat_mean_overlap_differences_m": differences,
            "repeatability_is_not_a_cross_hardware_bitwise_guarantee": True}


def requested_physics(workspace):
    physics = world_data(workspace / "world.sdf").find("physics")
    return {"solver": physics.findtext("ode/solver/type"), "iterations": int(physics.findtext("ode/solver/iters")),
            "step_size_s": float(physics.findtext("max_step_size")),
            "iterations_control_quick_solver_only": physics.findtext("ode/solver/type") == "quick"}


def check_loaded_physics(workspace, measurements):
    requested = requested_physics(workspace)
    actual = measurements["live_physics"]
    if actual["type"] != "ODE" or actual["solver_type"] != requested["solver"]:
        raise AssertionError("Native physics_info did not confirm the selected ODE solver")
    close_metric(actual["max_step_size"], requested["step_size_s"], "native loaded physics timestep", 1e-11)
    if requested["solver"] == "quick" and actual["iters"] != requested["iterations"]:
        raise AssertionError("Native physics_info did not confirm the selected quick iteration count")
    return {"actual_native_physics_info": actual, "requested": requested,
            "world_solver_does_not_use_quick_iteration_count": requested["solver"] == "world"}


def verify_case(session, name, expected_failure=None):
    before = actual_stats(session)
    original_assets = asset_hashes(session.workspace)
    item = {"name": name, "original_stats": before,
            "requested_physics": requested_physics(session.workspace),
            "state_contract": "explicit reset keeps experiment results; only pause and resource bytes restore"}
    if expected_failure:
        if not before["paused"]:
            raise AssertionError("Invalid contact input test requires a paused valid simulation")
        item["poses_before_invalid_input"] = {probe: actual_pose(session, probe) for probe in PROBES}
    try:
        offset = session.log_path.stat().st_size
        result = session.verify()
        evidence = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        item.update(verifier_result=result, raw_verifier_output=evidence)
        if result.get("passed") is not (expected_failure is None):
            raise AssertionError("Unexpected canonical verifier result for " + name)
        after = actual_stats(session)
        item.update(stats_after=after, pause_restored=after["paused"] == before["paused"],
                    resources_unchanged=original_assets == asset_hashes(session.workspace))
        if not item["pause_restored"] or not item["resources_unchanged"]:
            raise AssertionError("Stability instrument changed original pause or exercise resources")
        if expected_failure:
            if "FAIL" not in evidence or expected_failure not in evidence:
                raise AssertionError("Invalid contact configuration lacks a clear diagnostic")
            if before["iterations"] != after["iterations"] or before["sim_time_sec_nsec"] != after["sim_time_sec_nsec"]:
                raise AssertionError("Invalid contact configuration was rejected after physics control")
            current = {probe: actual_pose(session, probe) for probe in PROBES}
            item["poses_after_invalid_input"] = current
            for probe, original in item["poses_before_invalid_input"].items():
                if max(abs(a-b) for a, b in zip(original, current[probe])) > .002:
                    raise AssertionError("Invalid contact configuration moved/reset " + probe)
            item["configuration_rejected_before_control"] = True
        else:
            lines = re.findall(r"^MEASUREMENTS_JSON(?:[=:])?\s+(\{.*\})\s*$", evidence, re.M)
            if not lines:
                raise AssertionError("Stability instrument omitted actual measurement JSON")
            measurements = json.loads(lines[-1])
            item["canonical_measurements"] = measurements
            item["actual_physics_info"] = measurements["live_physics"]
            item["independent_loaded_physics_check"] = check_loaded_physics(session.workspace, measurements)
            item["independent_pose_metrics"] = independent_metrics(session.workspace, measurements)
        item["passed"] = True
        return item
    finally:
        if session.running and actual_stats(session)["paused"] != before["paused"]:
            session.pause(before["paused"])


def change_physics(workspace, field, value):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    node = tree.getroot().find("world/physics/" + field)
    if node is None:
        raise AssertionError("The declared physics field is absent: " + field)
    previous = node.text
    node.text = str(value)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    return {"changed_field": "physics/" + field, "from": previous, "to": str(value)}


def predict_configuration(workspace, expected_success=True, diagnostic=None):
    result = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"], cwd=str(workspace),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
    output = result.stdout+result.stderr
    if (result.returncode == 0) is not expected_success or (diagnostic and diagnostic not in output):
        raise AssertionError("Unexpected stability configuration result: " + output)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr,
            "prediction": json.loads(result.stdout) if expected_success else None}


def edit_world(workspace, change):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    change(tree.getroot().find("world"))
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def set_contact(workspace, model, field, value):
    edit_world(workspace, lambda world: setattr(named_model(world, model).find(
        "link/collision/surface/contact/ode/" + field), "text", str(value)))


def change_paired_soft_stiffness(workspace, value=4000):
    world = world_data(workspace / "world.sdf")
    previous = {model: float(named_model(world, model).findtext("link/collision/surface/contact/ode/kp"))
                for model in ("support_soft", "probe_soft")}
    for model in previous:
        set_contact(workspace, model, "kp", value)
    return {"independent_variable": "paired soft-contact stiffness",
            "declared_kp_before_N_m": previous,
            "declared_kp_after_N_m": {model: value for model in previous},
            "pair_kp_before_N_m": 1/sum(1/kp for kp in previous.values()),
            "pair_kp_after_N_m": value/2}


def compare_actual_stiffness_effect(baseline, changed):
    before = baseline["independent_pose_metrics"]["repeats"]
    after = changed["independent_pose_metrics"]["repeats"]
    old_soft = sum(item["models"]["probe_soft"]["mean_overlap_m"] for item in before)/len(before)
    new_soft = sum(item["models"]["probe_soft"]["mean_overlap_m"] for item in after)/len(after)
    new_firm = sum(item["models"]["probe_firm"]["mean_overlap_m"] for item in after)/len(after)
    reduction = old_soft-new_soft
    if reduction < .005 or new_soft-new_firm < .005:
        raise AssertionError("Increased paired stiffness did not produce the intended real compression contrast")
    return {"passed": True, "basis": "independently recomputed actual pose samples, not predicted compression",
            "actual_baseline_soft_mean_overlap_m": old_soft, "actual_changed_soft_mean_overlap_m": new_soft,
            "actual_changed_firm_mean_overlap_m": new_firm, "actual_soft_overlap_reduction_m": reduction,
            "error_noise_or_performance_ranking_required": False}


def set_parameter(workspace, field, value):
    path = workspace / "parameters.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    data[field] = value
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def weak_damping_window(workspace):
    for model in ("probe_soft", "support_soft"):
        set_contact(workspace, model, "kd", 5)


def deep_overlap(workspace):
    for model in ("probe_soft", "support_soft"):
        set_contact(workspace, model, "kp", 500)


def unsupported_sleep(workspace):
    edit_world(workspace, lambda world: setattr(named_model(world, "probe_soft").find(
        "allow_auto_disable"), "text", "true"))


def off_support(workspace):
    def change(world):
        model = named_model(world, "probe_soft")
        location = source_pose(model)
        location[0] += 2
        model.find("pose").text = " ".join(str(x) for x in location)
    edit_world(workspace, change)


def configuration_regressions(lesson, private):
    workspace = Storage(private / "invalid-inputs").prepare(lesson)
    original_world = (workspace / "world.sdf").read_bytes()
    original_parameters = (workspace / "parameters.json").read_bytes()
    original_assets = asset_hashes(workspace)
    report = {"kind": "configuration_only_not_physics_acceptance", "gazebo_started": False,
              "baseline_accepted": predict_configuration(workspace), "rejections": [], "variants": {}}
    cases = (
        ("negative_contact_stiffness", lambda p: set_contact(p, "probe_soft", "kp", -1), "kp/kd"),
        ("zero_physics_step", lambda p: change_physics(p, "max_step_size", 0), "步长dt"),
        ("unknown_solver_name", lambda p: change_physics(p, "ode/solver/type", "quickstep"), "solver/type"),
        ("warmup_too_short_for_current_damping", weak_damping_window, "warmup观察窗口不足"),
        ("fractional_physics_sample_interval", lambda p: set_parameter(p, "tail_interval_s", .1005), "整数物理步"),
        ("one_sample_cannot_show_dispersion", lambda p: set_parameter(p, "tail_samples", 1), "tail_samples=5"),
        ("one_repeat_cannot_show_repeatability", lambda p: set_parameter(p, "repeats", 1), "repeats=2"),
        ("sleep_must_not_fake_stability", unsupported_sleep, "不能用sleep伪装稳定"),
        ("probe_does_not_overlie_its_support", off_support, "球需位于平台中心"),
        ("dynamic_correction_cap_truncates_soft_constraint", lambda p: set_contact(p, "probe_soft", "max_vel", .1), "纠穿速度上限过低"),
        ("penetration_outside_small_deformation_rig", deep_overlap, "可分辨且小变形rig量程"),
    )
    for name, change, diagnostic in cases:
        try:
            change(workspace)
            before_rejection = asset_hashes(workspace)
            item = predict_configuration(workspace, expected_success=False, diagnostic=diagnostic)
            if before_rejection != asset_hashes(workspace):
                raise AssertionError("Rejected configuration modified exercise bytes: " + name)
            item.update(name=name, rejection_did_not_modify_resources=True)
        finally:
            (workspace / "world.sdf").write_bytes(original_world)
            (workspace / "parameters.json").write_bytes(original_parameters)
        predict_configuration(workspace)
        item.update(repair_accepted=True, passed=True)
        report["rejections"].append(item)
    if original_assets != asset_hashes(workspace):
        raise AssertionError("Configuration checks did not restore original resource bytes")
    for name, field, value in (("iters20", "ode/solver/iters", 20), ("dt002", "max_step_size", .002),
                                ("world", "ode/solver/type", "world")):
        variant = Storage(private / name).prepare(lesson)
        change = change_physics(variant, field, value)
        report["variants"][name] = {"single_variable_change": change, "result": predict_configuration(variant)}
    variant = Storage(private / "paired-soft-stiffness4000").prepare(lesson)
    change = change_paired_soft_stiffness(variant)
    report["variants"]["paired_soft_stiffness4000"] = {
        "single_variable_change": change, "result": predict_configuration(variant)}
    report.update(passed=True, original_resource_bytes_restored=True)
    return report


def configuration_only_main(lesson, args):
    original = asset_hashes(lesson.folder)
    started = time.monotonic()
    report = {"passed": False, "lesson": lesson.key, "kind": "configuration_only_not_physics_acceptance",
              "gazebo_started": False, "physical_scenarios_executed": 0}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l08-configuration-") as directory:
        try:
            report["configuration_regressions"] = configuration_regressions(lesson, Path(directory))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report.update(source_assets_unchanged=original == asset_hashes(lesson.folder),
                          elapsed_seconds=round(time.monotonic()-started, 3))
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
                        help="Only instrument-configuration guards; no Gazebo or physics acceptance")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:08"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L08 lesson; catalog errors: " + repr(errors))
    lesson = matches[0]
    if args.configuration_only:
        return configuration_only_main(lesson, args)
    source_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scenarios": [],
              "scope": "sampled geometric contact errors, not internal ODE residuals or a solver CPU benchmark"}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l08-") as directory:
        private = Path(directory)
        try:
            baseline = LessonSession(ROOT, lesson, storage=Storage(private / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(False)
            report["scenarios"].append(verify_case(baseline, "baseline_quick80_dt001_real_contact_metrics"))
            baseline.stop(reason="L08 baseline accepted")
            for name, field, value in (("quick20_only", "ode/solver/iters", 20),
                                        ("dt002_only", "max_step_size", .002),
                                        ("paired_soft_stiffness4000_only", "paired_soft_kp", 4000),
                                        ("world_solver_only", "ode/solver/type", "world")):
                storage = Storage(private / name)
                workspace = storage.prepare(lesson)
                changed = (change_paired_soft_stiffness(workspace, value) if field == "paired_soft_kp"
                           else change_physics(workspace, field, value))
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(session)
                session.start(gui=False, terminals=False)
                session.pause(True)
                item = verify_case(session, name)
                item["single_variable_change"] = changed
                if field == "paired_soft_kp":
                    item["actual_stiffness_effect"] = compare_actual_stiffness_effect(report["scenarios"][0], item)
                report["scenarios"].append(item)
                if name != "world_solver_only":
                    session.stop(reason="L08 single-variable case accepted")
            path = session.workspace / "world.sdf"
            valid = path.read_bytes()
            try:
                tree = ET.parse(str(path))
                node = named_model(tree.getroot().find("world"), "probe_soft").find("link/collision/surface/contact/ode/kp")
                node.text = "-1"
                tree.write(str(path), encoding="utf-8", xml_declaration=True)
                report["scenarios"].append(verify_case(session, "negative_contact_stiffness_rejected_before_control", expected_failure="kp"))
            finally:
                path.write_bytes(valid)
            report["scenarios"].append(verify_case(session, "repaired_world_solver_contact_rig_reaccepted"))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-45000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L08 private acceptance finished")
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report.update(leftovers=leftovers, source_assets_unchanged=source_assets == asset_hashes(lesson.folder),
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
