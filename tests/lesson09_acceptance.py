"""L09 base lesson: actual revolute/prismatic joint states, not model poses.

Private worlds cover baseline, changed gravity, a changed slider lower stop,
invalid axis/topology/limits before control, and byte-for-byte repair. Periods
are independently estimated from same-direction zero crossings of real joint
samples. The physical reference includes the sphere's COM inertia and the
parallel-axis term. This runner does not exercise the later optional controls.
"""
import argparse
import contextlib
import importlib.util
import io
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
from tests.lesson05_acceptance import command, first, parsed_model, short_name
from tests.lesson06_acceptance import actual_stats, asset_hashes, named_model, world_data

JOINTS = {"pendulum_rig": "swing_joint", "slider_rig": "slide_joint"}


def actual_joint_positions(session):
    observed = {}
    for model, joint_name in JOINTS.items():
        message = parsed_model(command(session, "gz", "model", "-w", session.world_name,
                                       "-m", model, "-i", read_only=True))
        joints = [joint for joint in message.get("joint", []) if short_name(first(joint, "name")) == joint_name]
        if len(joints) != 1 or len(joints[0].get("angle", [])) != 1:
            raise AssertionError("Actual Model.joint.angle lacks the expected one-axis joint " + joint_name)
        value = float(joints[0]["angle"][0])
        if not math.isfinite(value):
            raise AssertionError("Actual joint position is non-finite")
        observed[model] = {"joint": joint_name, "actual_position": value}
    return observed


def mechanics_reference(workspace):
    world = world_data(workspace / "world.sdf")
    pendulum = named_model(world, "pendulum_rig")
    bob = pendulum.find("link[@name='bob']")
    raw = [float(x) for x in bob.findtext("pose").split()]
    pivot = [float(x) for x in pendulum.findtext("joint[@name='swing_joint']/pose", "0 0 0 0 0 0").split()]
    dx, dy, dz = [a-b for a, b in zip(raw[:3], pivot[:3])]
    if abs(dy) > 1e-8:
        raise AssertionError("The base lesson expects the pendulum COM in its XZ plane")
    length = math.hypot(dx, dz)
    theta0 = math.atan2(-dx, -dz)
    mass = float(bob.findtext("inertial/mass"))
    com_inertia = float(bob.findtext("inertial/inertia/iyy"))
    pivot_inertia = com_inertia+mass*length*length
    gravity = float(world.findtext("gravity").split()[2])
    period = 2*math.pi*math.sqrt(pivot_inertia/(mass*(-gravity)*length))
    slider = named_model(world, "slider_rig")
    axis = slider.find("joint[@name='slide_joint']/axis")
    return {"initial_angle_rad": theta0, "length_m": length, "mass_kg": mass,
            "I_com_kg_m2": com_inertia, "I_pivot_kg_m2": pivot_inertia,
            "physical_small_angle_period_s": period, "gravity_z_m_s2": gravity,
            "slider_mass_kg": float(slider.findtext("link[@name='carriage']/inertial/mass")),
            "slider_damping_Ns_m": float(axis.findtext("dynamics/damping")),
            "slider_friction_N": float(axis.findtext("dynamics/friction")),
            "slider_lower_m": float(axis.findtext("limit/lower")),
            "slider_upper_m": float(axis.findtext("limit/upper"))}


def independent_period(samples, theta0, reference_period):
    crossings = {"downward": [], "upward": []}
    for left, right in zip(samples, samples[1:]):
        t0, t1 = left["t"], right["t"]
        a, b = theta0+left["pendulum_q_rad"], theta0+right["pendulum_q_rad"]
        if a > 0 >= b:
            direction = "downward"
        elif a < 0 <= b:
            direction = "upward"
        else:
            continue
        crossing = t0-a*(t1-t0)/(b-a)
        crossings[direction].append({"interpolated_time_s": crossing, "actual_time_bracket_s": [t0, t1],
                                     "actual_joint_angle_bracket_rad": [a, b]})
    periods = []
    brackets = []
    for values in crossings.values():
        for first_crossing, second_crossing in zip(values, values[1:]):
            periods.append(second_crossing["interpolated_time_s"]-first_crossing["interpolated_time_s"])
            brackets.append([second_crossing["actual_time_bracket_s"][0]-first_crossing["actual_time_bracket_s"][1],
                             second_crossing["actual_time_bracket_s"][1]-first_crossing["actual_time_bracket_s"][0]])
    if len(periods) < 2:
        raise AssertionError("Actual joint sequence lacks repeated same-direction zero crossings")
    maximum_interval = max(b["t"]-a["t"] for a, b in zip(samples, samples[1:]))
    if maximum_interval > reference_period/12:
        raise AssertionError("Joint sampling is too sparse for this period estimator's declared scope")
    measured = sum(periods)/len(periods)
    if abs(measured-reference_period) > reference_period*.025:
        raise AssertionError("Measured joint period disagrees with the physical-pendulum reference")
    return {"method": "linear interpolation of real same-direction joint-angle zero crossings",
            "crossings": crossings, "actual_period_samples_s": periods, "actual_period_brackets_s": brackets,
            "measured_period_s": measured, "maximum_actual_sample_interval_s": maximum_interval,
            "physical_small_angle_reference_s": reference_period,
            "reference_tolerance_fraction": .025,
            "sampling_scope": "small-angle damped motion with at least twelve samples per reference period"}


def independent_observations(workspace, measurements):
    reference = mechanics_reference(workspace)
    samples = measurements["samples"]
    if len(samples) < 20:
        raise AssertionError("Too few actual joint samples for the base experiment")
    origin = samples[0]["stats"]["sim_time_s"]
    initial_slider_link_z = samples[0]["slider_carriage_pose"][2]-samples[0]["slider_guide_pose"][2]
    for index, item in enumerate(samples):
        fields = [item["t"], item["pendulum_q_rad"], item["slider_q_m"], item["theta_rad"], item["theta_geometry_rad"]]
        if not all(math.isfinite(value) for value in fields) or not item["stats"]["paused"]:
            raise AssertionError("Actual joint capture contains non-finite or uncontrolled data")
        if abs(item["t"]-(item["stats"]["sim_time_s"]-origin)) > 1e-7:
            raise AssertionError("Joint sequence time is unsupported by native world statistics")
        if index and item["t"] <= samples[index-1]["t"]:
            raise AssertionError("Actual joint sample times do not advance")
        theta = reference["initial_angle_rad"]+item["pendulum_q_rad"]
        if abs(theta-item["theta_rad"]) > 1e-7 or abs(theta-item["theta_geometry_rad"]) > .01:
            raise AssertionError("Actual joint angle and independently observed bob geometry disagree")
        dx = item["pendulum_bob_pose"][0]-item["pendulum_frame_pose"][0]
        dz = item["pendulum_bob_pose"][2]-item["pendulum_frame_pose"][2]
        geometry_angle = math.atan2(-dx, -dz)
        if abs(math.atan2(math.sin(geometry_angle-theta), math.cos(geometry_angle-theta))) > .01:
            raise AssertionError("Raw child-link geometry does not corroborate the revolute joint's reference angle")
        slider_link_delta = item["slider_carriage_pose"][2]-item["slider_guide_pose"][2]-initial_slider_link_z
        if abs(slider_link_delta-(item["slider_q_m"]-samples[0]["slider_q_m"])) > .005:
            raise AssertionError("Raw link displacement does not corroborate the native prismatic joint position")
        if not reference["slider_lower_m"]-.005 <= item["slider_q_m"] <= reference["slider_upper_m"]+.005:
            raise AssertionError("Actual prismatic position exceeds the declared mechanical stops")
    period = independent_period(samples, reference["initial_angle_rad"], reference["physical_small_angle_period_s"])
    reported = measurements["pendulum_measurement"]["measured_period_s"]
    if abs(period["measured_period_s"]-reported) > max(.001, period["maximum_actual_sample_interval_s"]*.05):
        raise AssertionError("Reported period is not supported by the recorded actual joint crossings")
    small_angle = measurements["predictions"]["pendulum"]["small_angle_period_s"]
    if abs(small_angle-reference["physical_small_angle_period_s"]) > 1e-7:
        raise AssertionError("Period reference omitted the physical sphere inertia or parallel-axis term")
    early = min(samples, key=lambda item: abs(item["t"]-.2))
    if abs(early["t"]-.2) > 1e-7:
        raise AssertionError("Slider experiment omitted its actual 0.2-second observation")
    m, damping, friction, g = (reference[key] for key in (
        "slider_mass_kg", "slider_damping_Ns_m", "slider_friction_N", "gravity_z_m_s2"))
    elapsed = early["t"]
    if damping > 1e-10:
        predicted = (m*g+friction)/damping*(elapsed-m/damping*(1-math.exp(-damping*elapsed/m)))
    else:
        predicted = .5*(g+friction/m)*elapsed*elapsed
    observed = early["slider_q_m"]-samples[0]["slider_q_m"]
    if abs(observed-predicted) > .007:
        raise AssertionError("Actual early prismatic motion disagrees with its passive damping/friction model")
    final = samples[-1]["slider_q_m"]
    if abs(final-reference["slider_lower_m"]) > .005:
        raise AssertionError("Actual joint position did not reach its lower mechanical stop")
    return {"basis": "actual Model.joint.angle and native sample times; model pose is not a joint proxy",
            "mechanics_reference": reference, "period": period,
            "slider": {"early_time_s": elapsed, "actual_early_q_change_m": observed,
                       "passive_model_early_q_change_m": predicted, "actual_final_q_m": final,
                       "lower_stop_m": reference["slider_lower_m"]}}


def check_loaded_joint_configuration(workspace, measurements):
    world = world_data(workspace / "world.sdf")
    checked = {}
    for model_name, joint_name in JOINTS.items():
        source = named_model(world, model_name).find("joint[@name='{}']".format(joint_name))
        actual_model = measurements["live_models"][model_name]
        joints = [joint for joint in actual_model["joint"] if short_name(joint["name"]) == joint_name]
        if len(joints) != 1:
            raise AssertionError("SDK did not inspect the intended actual joint " + joint_name)
        joint = joints[0]
        if joint["type"].lower() != source.get("type"):
            raise AssertionError("Actual joint type differs from the exercise")
        for endpoint in ("parent", "child"):
            if short_name(joint[endpoint]) != source.findtext(endpoint):
                raise AssertionError("Actual joint topology differs for " + joint_name)
        expected_axis = [float(x) for x in source.findtext("axis/xyz").split()]
        actual_axis = [float(joint["axis1"]["xyz"][axis]) for axis in ("x", "y", "z")]
        if max(abs(a-b) for a, b in zip(expected_axis, actual_axis)) > 1e-7:
            raise AssertionError("Actual joint axis differs from the aligned base rig")
        for field, relative in (("limit_lower", "axis/limit/lower"), ("limit_upper", "axis/limit/upper"),
                                ("damping", "axis/dynamics/damping"), ("friction", "axis/dynamics/friction")):
            if abs(float(joint["axis1"][field])-float(source.findtext(relative))) > 1e-7:
                raise AssertionError("Actual joint {} differs for {}".format(field, joint_name))
        checked[model_name] = {"actual_joint": joint, "aligned_frame_axis": actual_axis}
    actual_g = measurements["live_physics"]["gravity"]
    expected_g = [float(x) for x in world.findtext("gravity").split()]
    if max(abs(float(actual_g[axis])-value) for axis, value in zip(("x", "y", "z"), expected_g)) > 1e-7:
        raise AssertionError("SDK physics_info did not confirm actual loaded gravity")
    return {"actual_joint_configuration": checked, "actual_physics": measurements["live_physics"]}


class CaseFailure(AssertionError):
    def __init__(self, message, evidence):
        super().__init__(message)
        self.evidence = evidence


def verify_case(session, name, diagnostic=None):
    original = actual_stats(session)
    original_assets = asset_hashes(session.workspace)
    item = {"name": name, "original_stats": original,
            "state_contract": "reset base experiment and retain results; restore pause/resources only"}
    if diagnostic:
        if not original["paused"]:
            raise AssertionError("Invalid joint input must start in a paused valid world")
        item["joint_positions_before_invalid_input"] = actual_joint_positions(session)
    try:
        offset = session.log_path.stat().st_size
        result = session.verify()
        output = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        item.update(verifier_result=result, raw_verifier_output=output)
        failed = re.findall(r"^FAILED_MEASUREMENTS_JSON(?:[=:])?\s+(\{.*\})\s*$", output, re.M)
        if failed:
            item["failed_actual_measurements"] = json.loads(failed[-1])
        if result.get("passed") is not (diagnostic is None):
            raise AssertionError("Unexpected canonical verifier result for " + name)
        after = actual_stats(session)
        item.update(stats_after=after, pause_restored=after["paused"] == original["paused"],
                    resource_bytes_unchanged=original_assets == asset_hashes(session.workspace))
        if not item["pause_restored"] or not item["resource_bytes_unchanged"]:
            raise AssertionError("Joint instrument changed original pause or exercise bytes")
        if diagnostic:
            if "FAIL" not in output or diagnostic not in output:
                raise AssertionError("Invalid joint input lacks a clear relevant diagnosis")
            if original["iterations"] != after["iterations"] or original["sim_time_sec_nsec"] != after["sim_time_sec_nsec"]:
                raise AssertionError("Invalid joint input was rejected after physics control")
            current = actual_joint_positions(session)
            item["joint_positions_after_invalid_input"] = current
            if any(abs(current[model]["actual_position"]-before["actual_position"]) > 1e-6
                   for model, before in item["joint_positions_before_invalid_input"].items()):
                raise AssertionError("Invalid joint input reset or moved an actual joint")
            item["invalid_configuration_rejected_before_control"] = True
        else:
            lines = re.findall(r"^MEASUREMENTS_JSON(?:[=:])?\s+(\{.*\})\s*$", output, re.M)
            if not lines:
                raise AssertionError("Base joint instrument omitted its real measurement JSON")
            measurements = json.loads(lines[-1])
            item["canonical_measurements"] = measurements
            item["independent_actual_joint_configuration"] = check_loaded_joint_configuration(session.workspace, measurements)
            item["independent_joint_observations"] = independent_observations(session.workspace, measurements)
        item["passed"] = True
        return item
    except Exception as error:
        item["error"] = "{}: {}".format(type(error).__name__, error)
        raise CaseFailure(str(error), item) from error
    finally:
        if session.running and actual_stats(session)["paused"] != original["paused"]:
            session.pause(original["paused"])


def change_world(workspace, mutation):
    path = workspace / "world.sdf"
    tree = ET.parse(str(path))
    mutation(tree.getroot().find("world"))
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def gravity_variant(workspace):
    change_world(workspace, lambda world: setattr(world.find("gravity"), "text", "0 0 -6"))


def limit_variant(workspace):
    change_world(workspace, lambda world: setattr(named_model(world, "slider_rig").find(
        "joint[@name='slide_joint']/axis/limit/lower"), "text", "-.5"))


def predict_configuration(workspace, success=True, diagnostic=None):
    result = subprocess.run([sys.executable, str(workspace / "lab.py"), "predict"], cwd=str(workspace),
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
    output = result.stdout+result.stderr
    if (result.returncode == 0) is not success or (diagnostic and diagnostic not in output):
        raise AssertionError("Unexpected base joint configuration result: " + output)
    return {"returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr,
            "prediction": json.loads(result.stdout) if success else None}


def set_model_field(workspace, model, field, value):
    def change(world):
        source = named_model(world, model)
        node = source.find(field)
        if node is None:
            if "/" in field:
                raise AssertionError("Required fixture field is absent: " + field)
            node = ET.SubElement(source, field)
        node.text = str(value)
    change_world(workspace, change)


def set_parameter(workspace, field, value):
    path = workspace / "parameters.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    data[field] = value
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def wrong_axis_reference(workspace):
    change_world(workspace, lambda world: named_model(world, "pendulum_rig").find(
        "joint[@name='swing_joint']/axis/xyz").set("expressed_in", "world"))


def desynchronized_initial_bob(workspace):
    def change(world):
        node = named_model(world, "pendulum_rig").find("link[@name='bob']/pose")
        raw = [float(value) for value in node.text.split()]
        raw[0] += .1
        node.text = " ".join(str(value) for value in raw)
    change_world(workspace, change)


def excessive_capture_horizon(workspace):
    change_world(workspace, lambda world: setattr(world.find("gravity"), "text", "0 0 -5"))
    set_parameter(workspace, "periods_to_observe", 3.2)


def configuration_regressions(lesson, root):
    workspace = Storage(root / "configuration-rejections").prepare(lesson)
    original_world = (workspace / "world.sdf").read_bytes()
    original_parameters = (workspace / "parameters.json").read_bytes()
    original_assets = asset_hashes(workspace)
    report = {"kind": "configuration_only_not_physics_acceptance", "gazebo_started": False,
              "baseline_accepted": predict_configuration(workspace), "rejections": [], "variants": {}}
    checks = (
        ("unsupported_revolute_axis", lambda p: set_model_field(p, "pendulum_rig", "joint[@name='swing_joint']/axis/xyz", "1 0 0"), "axis"),
        ("unknown_prismatic_child", lambda p: set_model_field(p, "slider_rig", "joint[@name='slide_joint']/child", "missing_carriage"), "child"),
        ("reversed_prismatic_limits", lambda p: set_model_field(p, "slider_rig", "joint[@name='slide_joint']/axis/limit/lower", ".2"), "限位"),
        ("unsupported_axis_reference_frame", wrong_axis_reference, "expressed_in"),
        ("pendulum_stop_interrupts_free_oscillation", lambda p: set_model_field(p, "pendulum_rig", "joint[@name='swing_joint']/axis/limit/lower", "-.25"), "限位远离预计振幅"),
        ("pivot_inertia_mistaken_for_com_inertia", lambda p: set_model_field(p, "pendulum_rig", "link[@name='bob']/inertial/inertia/iyy", ".644"), "均匀几何惯性"),
        ("static_model_cannot_replace_world_fixed_joint", lambda p: set_model_field(p, "pendulum_rig", "static", "true"), "需动态模型"),
        ("sampling_too_sparse_for_period", lambda p: set_parameter(p, "sample_interval_s", .2), "采样需interval"),
        ("sample_grid_omits_early_slider_observation", lambda p: set_parameter(p, "sample_interval_s", .06), "覆盖t=.2"),
        ("insufficient_cycles_for_repeated_crossings", lambda p: set_parameter(p, "periods_to_observe", 1), "观察2.8至3.2周期"),
        ("otherwise_valid_parameters_exceed_capture_budget", excessive_capture_horizon, "本机仪器预算"),
        ("initial_bob_geometry_not_synchronized_with_angle", desynchronized_initial_bob, "初始bob位置需随Ry"),
        ("displaced_hinge_anchor_invalidates_reference", lambda p: set_model_field(p, "pendulum_rig", "joint[@name='swing_joint']/pose", ".05 0 0 0 0 0"), "锚点"),
    )
    for name, change, diagnostic in checks:
        try:
            change(workspace)
            before = asset_hashes(workspace)
            item = predict_configuration(workspace, success=False, diagnostic=diagnostic)
            if before != asset_hashes(workspace):
                raise AssertionError("Rejected base joint configuration changed source bytes: " + name)
            item.update(name=name, rejection_did_not_modify_resources=True)
        finally:
            (workspace / "world.sdf").write_bytes(original_world)
            (workspace / "parameters.json").write_bytes(original_parameters)
        predict_configuration(workspace)
        item.update(repair_accepted=True, passed=True)
        report["rejections"].append(item)
    if original_assets != asset_hashes(workspace):
        raise AssertionError("Configuration tests did not restore original base exercise bytes")
    for name, change in (("gravity6", gravity_variant), ("slider_lower_minus05", limit_variant)):
        variant = Storage(root / name).prepare(lesson)
        change(variant)
        report["variants"][name] = predict_configuration(variant)
    report.update(passed=True, all_invalid_resource_bytes_restored=True)
    return report


def configuration_only_main(lesson, args):
    source = asset_hashes(lesson.folder)
    started = time.monotonic()
    report = {"passed": False, "lesson": lesson.key, "scope": "base lesson only",
              "kind": "configuration_only_not_physics_acceptance", "gazebo_started": False,
              "physical_scenarios_executed": 0}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l09-base-configuration-") as directory:
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


def load_cleanup_instrument(lesson):
    """Import the current source in isolation without writing source bytecode."""
    absent = object()
    previous_modules = {name: sys.modules.get(name, absent) for name in ("kinematics", "transport")}
    previous_path = list(sys.path)
    previous_bytecode = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        for name in previous_modules:
            sys.modules.pop(name, None)
        sys.path.insert(0, str(lesson.folder))
        specification = importlib.util.spec_from_file_location("_gt09_cleanup_fault_probe", str(lesson.file("instrument.py")))
        instrument = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(instrument)
        return instrument
    finally:
        sys.path[:] = previous_path
        sys.dont_write_bytecode = previous_bytecode
        for name, previous in previous_modules.items():
            if previous is absent:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def cleanup_protocol_regressions(lesson):
    """Inject only transport/state-read outcomes; no simulator is started."""
    instrument = load_cleanup_instrument(lesson)
    cases = []
    world = "injected_cleanup_world"
    command_line = ["gz", "world", "-w", world, "-p", "1"]
    injected_stderr = "injected boost::thread self-join failure after publication\n"
    for name, observed_states, desired_confirmed in (
            ("already_matching_state_issues_no_command", [True], True),
            ("abnormal_exit_after_apply_is_confirmed_with_warning", [False, True], True),
            ("abnormal_exit_without_apply_fails_without_resend", [False, False], False)):
        captured = io.StringIO()
        readbacks = [{"paused": value, "iterations": 73, "sim_time_s": 1.25} for value in observed_states]
        fault = subprocess.CalledProcessError(-6, command_line, output="injected publication output\n", stderr=injected_stderr)
        instrument.gz.DEADLINE = 106.0
        with mock.patch.object(instrument.time, "monotonic", return_value=100.0), \
                mock.patch.object(instrument.gz, "stats", side_effect=readbacks) as read_state, \
                mock.patch.object(instrument.subprocess, "run", side_effect=fault) as send_command, \
                contextlib.redirect_stdout(captured):
            result = instrument.restore_pause(world, True)
        if result.get("confirmed") is not desired_confirmed:
            raise AssertionError("Pause restoration ignored its authoritative read-back in " + name)
        if len(observed_states) == 1:
            if send_command.call_count or read_state.call_count != 1 or result.get("command_sent"):
                raise AssertionError("Already-matching pause issued an unnecessary state-changing command")
            if result.get("action") != "already_matched":
                raise AssertionError("No-op restoration lacks an explicit already-matched result")
        else:
            if send_command.call_count != 1 or read_state.call_count != 2:
                raise AssertionError("Abnormal CLI exit caused blind command retry or omitted confirmation")
            invocation = send_command.call_args
            if invocation.args[0] != command_line or invocation.kwargs.get("check") is not True or \
                    not 0 < invocation.kwargs.get("timeout", 0) <= 2.5:
                raise AssertionError("Pause write did not use the single bounded checked command")
            error = result.get("command_error", {})
            if error.get("returncode") != -6 or error.get("stderr") != injected_stderr:
                raise AssertionError("Pause cleanup discarded the original abnormal-exit diagnostic")
            if desired_confirmed:
                if not result.get("transport_warning") or "WARNING" not in captured.getvalue():
                    raise AssertionError("Applied command failure was silently swallowed instead of retained as a warning")
            elif not result.get("error") or result.get("transport_warning"):
                raise AssertionError("Unapplied command failure was reported as a successful transport warning")
        cases.append({"name": name, "passed": True, "injected_paused_readbacks": observed_states,
                      "state_read_calls": read_state.call_count, "command_calls": send_command.call_count,
                      "cleanup_result": result, "captured_helper_output": captured.getvalue()})
    return {"passed": True, "kind": "fault_injection_not_physics_acceptance", "gazebo_started": False,
            "physical_scenarios_executed": 0, "cases": cases}


def cleanup_regression_only_main(lesson, args):
    before = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scope": "base cleanup protocol only",
              "kind": "fault_injection_not_physics_acceptance", "gazebo_started": False,
              "physical_scenarios_executed": 0, "configuration_risk_cases_executed": 0}
    started = time.monotonic()
    try:
        report["cleanup_protocol_regressions"] = cleanup_protocol_regressions(lesson)
        report["passed"] = True
    except Exception as error:
        report["error"] = "{}: {}".format(type(error).__name__, error)
    finally:
        report.update(source_assets_unchanged=before == asset_hashes(lesson.folder),
                      elapsed_seconds=round(time.monotonic()-started, 3))
        if not report["source_assets_unchanged"]:
            report["passed"] = False
        if args.report:
            path = Path(args.report).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


def invalid_joint_group(session):
    path = session.workspace / "world.sdf"
    valid = path.read_bytes()
    checks = (("unsupported_swing_axis", "pendulum_rig", "joint[@name='swing_joint']/axis/xyz", "1 0 0", "axis"),
              ("unknown_joint_child", "slider_rig", "joint[@name='slide_joint']/child", "missing_carriage", "child"),
              ("reversed_slider_limits", "slider_rig", "joint[@name='slide_joint']/axis/limit/lower", ".2", "限位"))
    items = []
    for name, model, field, value, diagnostic in checks:
        try:
            change_world(session.workspace, lambda world: setattr(named_model(world, model).find(field), "text", value))
            items.append(verify_case(session, name, diagnostic=diagnostic))
        finally:
            path.write_bytes(valid)
        predict_configuration(session.workspace)
    return {"name": "invalid_axis_topology_limits_rejected_before_any_control", "passed": True,
            "rejections": items, "valid_resource_bytes_restored_after_each_check": path.read_bytes() == valid}


def compare_actual_variants(baseline, changed, limited):
    old = baseline["independent_joint_observations"]
    new = changed["independent_joint_observations"]
    stop = limited["independent_joint_observations"]["slider"]
    ratio = new["period"]["measured_period_s"]/old["period"]["measured_period_s"]
    expected = math.sqrt(abs(old["mechanics_reference"]["gravity_z_m_s2"])/abs(new["mechanics_reference"]["gravity_z_m_s2"]))
    if ratio < 1.2 or abs(ratio-expected) > .04:
        raise AssertionError("Actual changed-gravity joint periods lack the expected clear physical change")
    if new["slider"]["actual_early_q_change_m"] <= old["slider"]["actual_early_q_change_m"]+.04:
        raise AssertionError("Actual reduced-gravity slider did not show its expected smaller early fall")
    if stop["actual_final_q_m"]-old["slider"]["actual_final_q_m"] < .07:
        raise AssertionError("Changing the lower joint stop did not change the actual final joint position")
    return {"passed": True, "basis": "actual native joint-angle sequences, not predicted periods or link positions",
            "actual_period_ratio_g6_to_baseline": ratio,
            "actual_slider_early_displacements_baseline_g6_m": [old["slider"]["actual_early_q_change_m"], new["slider"]["actual_early_q_change_m"]],
            "actual_slider_lower_positions_baseline_changed_m": [old["slider"]["actual_final_q_m"], stop["actual_final_q_m"]]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--configuration-only", action="store_true",
                      help="Base lesson configuration guards only; no Gazebo or optional control projects")
    mode.add_argument("--cleanup-regression-only", action="store_true",
                      help="Three transport fault injections only; no physics or configuration-risk cases")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:09"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L09 base lesson; catalog errors: " + repr(errors))
    lesson = matches[0]
    if args.configuration_only:
        return configuration_only_main(lesson, args)
    if args.cleanup_regression_only:
        return cleanup_regression_only_main(lesson, args)
    original_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scope": "base pendulum and passive slider only", "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l09-base-") as directory:
        root = Path(directory)
        try:
            baseline = LessonSession(ROOT, lesson, storage=Storage(root / "baseline"))
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(False)
            base_case = verify_case(baseline, "baseline_actual_physical_pendulum_and_prismatic_joint")
            report["scenarios"].append(base_case)
            baseline.stop(reason="L09 base reference accepted")
            storage = Storage(root / "gravity6")
            workspace = storage.prepare(lesson)
            gravity_variant(workspace)
            changed = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(changed)
            changed.start(gui=False, terminals=False)
            changed.pause(True)
            gravity_case = verify_case(changed, "gravity6_actual_period_and_slider_change")
            report["scenarios"].append(gravity_case)
            changed.stop(reason="L09 gravity variant accepted")
            storage = Storage(root / "lower_stop")
            workspace = storage.prepare(lesson)
            limit_variant(workspace)
            limited = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(limited)
            limited.start(gui=False, terminals=False)
            limited.pause(True)
            limit_case = verify_case(limited, "changed_prismatic_lower_stop")
            report["scenarios"].append(limit_case)
            report["actual_variant_effects"] = compare_actual_variants(base_case, gravity_case, limit_case)
            report["scenarios"].append(invalid_joint_group(limited))
            report["scenarios"].append(verify_case(limited, "repaired_base_joint_configuration_reaccepted"))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            if isinstance(error, CaseFailure):
                report["failed_case"] = error.evidence
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-45000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L09 base acceptance finished")
            deadline = time.monotonic()+5
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
