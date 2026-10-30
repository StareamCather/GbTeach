#!/usr/bin/env python3
"""L09 optional B: actual serial joints, two world rod axes, six-state control.

Only temporary Storage receives a completed reference or a deliberately wrong
student. --unit-only runs no simulator; --parser-only also works before the
course package is complete. Synthetic parser checks never count as physics.
The default runs demo, student, stale-code rejection and zero-control failure.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson06_acceptance import actual_stats
from tests.lesson091_acceptance import (
    TIME_EPS, await_bridge, closed_time, control_directory, digest,
    disturbance_matches, official_lab, read_json, read_rows, require,
    snapshot, source_hashes, verify_rejected,
)

LESSON_KEY = "classic11-noetic:092"
TRIALS = ((.1, .03, .03, 0.), (-.1, -.03, -.03, 0.), (.1, .03, -.03, 2.))
RMS_LIMITS = {"x_m": .025, "xdot_m_s": .04, "theta_up_rad": .01,
              "q1dot_rad_s": .05, "theta2_up_rad": .01, "omega2_abs_rad_s": .05}


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def upper_hinge_geometry(row):
    anchor = [float(row["upper_hinge_"+axis+"_m"]) for axis in "xyz"]
    lower_com = [float(row["pole_com_"+axis+"_m"]) for axis in "xyz"]
    upper_com = [float(row["pole2_com_"+axis+"_m"]) for axis in "xyz"]
    lower_axis = [float(row["pole_axis_x"]), 0., float(row["pole_axis_z"])]
    upper_axis = [float(row["pole2_axis_x"]), 0., float(row["pole2_axis_z"])]
    require(all(math.isfinite(value) for vector in (anchor, lower_com, upper_com) for value in vector),
            "Non-finite actual rod COM or upper joint anchor")
    # Different rod lengths matter: L1/2=.30 m and L2/2=.25 m.
    lower_tip = [com+.3*axis for com, axis in zip(lower_com, lower_axis)]
    upper_base = [com-.25*axis for com, axis in zip(upper_com, upper_axis)]
    lower_error = math.sqrt(sum((a-b)**2 for a, b in zip(lower_tip, anchor)))
    upper_error = math.sqrt(sum((a-b)**2 for a, b in zip(upper_base, anchor)))
    require(lower_error <= .002 and upper_error <= .002,
            "Actual rod COMs, their world axes, and the actual upper hinge do not meet")
    return {"actual_anchor_world_m": anchor, "lower_rod_tip_world_m": lower_tip,
            "upper_rod_base_world_m": upper_base, "lower_tip_error_m": lower_error,
            "upper_base_error_m": upper_error}


def state_from_native(row):
    """Convert actual relative hinge states, corroborated by two world axes."""
    names = ("x_m", "xdot_m_s", "q1_rad", "q1dot_rad_s", "q2_rad", "q2dot_rad_s",
             "theta_up_rad", "theta2_up_rad", "omega2_abs_rad_s",
             "pole_axis_x", "pole_axis_z", "pole2_axis_x", "pole2_axis_z")
    raw = {name: float(row[name]) for name in names}
    require(all(math.isfinite(value) for value in raw.values()), "Non-finite actual serial-joint or rod-axis state")
    theta1, theta2 = wrap(raw["q1_rad"]), wrap(raw["q1_rad"]+raw["q2_rad"])
    omega2 = raw["q1dot_rad_s"]+raw["q2dot_rad_s"]
    require(abs(theta1-raw["theta_up_rad"]) < 1e-7, "First upright angle differs from native q1")
    require(abs(theta2-raw["theta2_up_rad"]) < 1e-7, "Second absolute angle must use q1+q2_relative")
    require(abs(omega2-raw["omega2_abs_rad_s"]) < 1e-7, "Second absolute angular velocity must use q1dot+q2dot")
    for angle, x, z in ((theta1, raw["pole_axis_x"], raw["pole_axis_z"]),
                        (theta2, raw["pole2_axis_x"], raw["pole2_axis_z"])):
        require(abs(x-math.sin(angle)) <= .002 and abs(z-math.cos(angle)) <= .002,
                "Native joint sum does not match the corresponding rod's actual world axis")
        geometry = math.atan2(x, z)
        require(abs(wrap(geometry-angle)) <= .002,
                "Independent world-axis angle disagrees with the actual serial-joint state")
    upper_hinge_geometry(row)
    return [raw["x_m"], raw["xdot_m_s"], theta1, raw["q1dot_rad_s"], theta2, omega2]


def parser_regressions():
    """Known numerical records check conversion and rejection, not simulation."""
    raw = {"x_m": .1, "xdot_m_s": -.05, "q1_rad": .03, "q1dot_rad_s": .1,
           "q2_rad": -.06, "q2dot_rad_s": -.04, "theta_up_rad": .03,
           "theta2_up_rad": -.03, "omega2_abs_rad_s": .06,
           "pole_axis_x": math.sin(.03), "pole_axis_z": math.cos(.03),
           "pole2_axis_x": math.sin(-.03), "pole2_axis_z": math.cos(-.03)}
    for axis, com1, direction1, direction2 in zip("xyz", (.2, 0., 1.1),
                                                 (math.sin(.03), 0., math.cos(.03)),
                                                 (math.sin(-.03), 0., math.cos(-.03))):
        anchor = com1+.3*direction1
        raw["pole_com_"+axis+"_m"] = com1
        raw["upper_hinge_"+axis+"_m"] = anchor
        raw["pole2_com_"+axis+"_m"] = anchor+.25*direction2
    actual = state_from_native(raw)
    expected = [.1, -.05, .03, .1, -.03, .06]
    require(all(abs(x-y) < 1e-12 for x, y in zip(actual, expected)), "Six-state converter is incorrect")
    rejected = []
    for name, field, value in (("q2_is_not_an_absolute_angle", "theta2_up_rad", -.06),
                               ("q2dot_is_not_an_absolute_velocity", "omega2_abs_rad_s", -.04),
                               ("second_world_axis_is_not_the_first", "pole2_axis_x", math.sin(.03)),
                               ("upper_hinge_is_not_a_disconnected_anchor", "upper_hinge_x_m", raw["upper_hinge_x_m"]+.01)):
        altered = dict(raw)
        altered[field] = value
        try:
            state_from_native(altered)
        except AssertionError as error:
            rejected.append({"name": name, "diagnosis": str(error)})
        else:
            raise AssertionError("State parser accepted a deliberately inconsistent record: "+name)
    points = [10+.01*index for index in range(201)]
    points[0], points[-1] = 9.999999999999996, 12.000000000000002
    require(sum(closed_time(value, 10, 12) for value in points) == 201,
            "Closed terminal window lost a floating-point endpoint")
    require(not closed_time(9.99, 10, 12) and not closed_time(12.01, 10, 12),
            "Terminal endpoint tolerance includes a neighboring control tick")
    return {"passed": True, "kind": "pure_numerical_parser_checks_not_physics",
            "relative_joint_example": raw, "actual_conversion": actual,
            "rejected_inconsistent_records": rejected, "closed_terminal_window_s": [10, 12],
            "endpoint_tolerance_s": TIME_EPS, "terminal_grid_points": 201}


def pure_unit_regressions(lesson, private):
    reference = (lesson.folder/"reference_controller.py").read_bytes()
    wrong_angle = b"\ndef state_error(observation, target_x=0.0):\n    return np.array([observation['x']-target_x, observation['xdot'], (observation['q1']+np.pi)%(2*np.pi)-np.pi, observation['q1dot'], (observation['q2']+np.pi)%(2*np.pi)-np.pi, observation['q1dot']+observation['q2dot']])\n"
    wrong_velocity = b"\ndef state_error(observation, target_x=0.0):\n    return np.array([observation['x']-target_x, observation['xdot'], (observation['q1']+np.pi)%(2*np.pi)-np.pi, observation['q1dot'], (observation['q1']+observation['q2']+np.pi)%(2*np.pi)-np.pi, observation['q2dot']])\n"
    variants = (
        ("unfinished_TODO_rejected", (lesson.folder/"learner.py").read_bytes(), False, "NotImplementedError"),
        ("completed_private_reference", reference, True, None),
        ("relative_angle_used_as_absolute_rejected", reference+wrong_angle, False, "state_error"),
        ("relative_velocity_used_as_absolute_rejected", reference+wrong_velocity, False, "state_error"),
        ("zero_feedback_rejected", reference+b"\ndef feedback_force(error):\n    return 0.0\n", False, "feedback_force"),
        ("wrong_feedback_sign_rejected", reference+b"\ndef feedback_force(error):\n    return float(K @ error)\n", False, "feedback_force"),
        ("native_clip_cannot_mask_missing_student_clip", reference+b"\ndef clip_force(force, force_limit=20.0):\n    return float(force)\n", False, "clip_force"),
    )
    checks = []
    for name, code, accepted, diagnostic in variants:
        path = private/(name+".py")
        path.write_bytes(code)
        result = subprocess.run([sys.executable, str(lesson.folder/"unit_probe.py"), str(path)],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
        require((result.returncode == 0) is accepted,
                "Unexpected double-pendulum behavior check: "+name+"\n"+result.stdout+result.stderr)
        if diagnostic:
            require(diagnostic in result.stdout+result.stderr, "Missing relevant student-code diagnosis: "+name)
        item = {"name": name, "expected_acceptance": accepted, "returncode": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr, "learner_sha256": digest(path)}
        if accepted:
            item["behavior_checks"] = json.loads(result.stdout.splitlines()[-1])
            require(item["behavior_checks"]["passed"] is True
                    and item["behavior_checks"]["source_sha256"] == digest(path), "Behavior probes did not identify the executed code")
        checks.append(item)
    return {"kind": "pure_python_behavior_not_physics", "gazebo_started": False, "checks": checks}


def validate_native_model(status):
    require(status.get("poles") == 2, "Native bridge did not inspect two actual pendulum rods")
    model = status["model"]
    expected = {"cart_mass_kg": 1., "pole_mass_kg": .2, "pole_Iyy_kg_m2": .00601125,
                "pole2_mass_kg": .15, "pole2_Iyy_kg_m2": .0031334375,
                "cart_damping": .05, "pole_damping": .002, "pole2_damping": .002,
                "physics_step_s": .001, "gravity_z": -9.81, "cart_effort_limit_N": 25.}
    for key, value in expected.items():
        require(abs(float(model[key])-value) <= 1e-9, "Actual SDK double-rig parameter differs: "+key)
    return {"actual": model, "expected": expected, "poles": 2}


def independent_trial(rows, expected, initial, mode, sha, compare_canonical=True):
    selected = [row for row in rows if row["episode"] == expected["episode"]
                and closed_time(float(row["episode_time_s"]), 0, 12)]
    require(len(selected) >= 1190, "Insufficient actual 100 Hz double-pendulum control samples")
    numeric = ("episode_time_s", "sim_time_s", "x_m", "xdot_m_s", "q1_rad", "q1dot_rad_s",
               "q2_rad", "q2dot_rad_s", "theta_up_rad", "theta2_up_rad", "omega2_abs_rad_s",
               "pole_axis_x", "pole_axis_z", "pole2_axis_x", "pole2_axis_z", "python_force_N",
               "command_force_N", "disturbance_N", "applied_force_N", "callback_count", "episode_callback_count",
               "pole_com_x_m", "pole_com_y_m", "pole_com_z_m", "pole2_com_x_m", "pole2_com_y_m", "pole2_com_z_m",
               "upper_hinge_x_m", "upper_hinge_y_m", "upper_hinge_z_m")
    values = [{name: float(row[name]) for name in numeric} for row in selected]
    require(all(math.isfinite(value) for row in values for value in row.values()), "Non-finite actual double-rig telemetry")
    require(all(row["mode"] == mode and row["source_sha256"] == sha and row["controller_ok"] == "1"
                for row in selected), "Running student mode/SHA/control status differs from the submitted code")
    x, theta1, theta2, disturbance = initial
    first_state = state_from_native(values[0])
    require(abs(first_state[0]-x) <= .001 and abs(first_state[2]-theta1) <= .001
            and abs(first_state[4]-theta2) <= .001 and abs(values[0]["q2_rad"]-(theta2-theta1)) <= .001,
            "Actual serial-joint initial positions do not implement both requested absolute angles")
    for before, after in zip(values, values[1:]):
        require(abs(after["episode_time_s"]-before["episode_time_s"]-.01) <= .0001,
                "Actual double-rig student callbacks are not at simulated 100 Hz")
        require(after["callback_count"]-before["callback_count"] == 1
                and after["episode_callback_count"]-before["episode_callback_count"] == 1,
                "Double-rig callback accounting skipped or duplicated a Python call")
    for item in values:
        state_from_native(item)
        require(abs(item["python_force_N"]-item["command_force_N"]) <= 1e-6,
                "Native safety clipping masked a missing student clip function")
        require(abs(item["applied_force_N"]-item["command_force_N"]-item["disturbance_N"]) <= 1e-6,
                "Applied cart force does not contain the actual recorded disturbance")
        require(disturbance_matches(item["episode_time_s"], item["disturbance_N"], disturbance),
                "Actual native disturbance timing or amplitude differs from the intended trial")
    tail = [item for item in values if closed_time(item["episode_time_s"], 10, 12)]
    require(len(tail) == 201, "Actual 10–12 s closed terminal window must contain its 201 native 100 Hz samples")
    rms = {key: math.sqrt(sum(item[key]**2 for item in tail)/len(tail)) for key in RMS_LIMITS}
    peaks = {key: max(abs(item[key]) for item in values)
             for key in ("x_m", "theta_up_rad", "theta2_up_rad", "command_force_N")}
    require(peaks["x_m"] <= 1.2 and peaks["theta_up_rad"] <= .35
            and peaks["theta2_up_rad"] <= .35 and peaks["command_force_N"] <= 20.000001,
            "Actual cart position, either absolute rod angle, or control force left the declared bounds")
    require(all(rms[key] <= value for key, value in RMS_LIMITS.items()), "Actual final-two-second six-state RMS did not converge: "+repr(rms))
    if compare_canonical:
        require(expected["terminal_samples"] == len(tail), "Canonical terminal sample count differs from actual closed endpoints")
        for key, actual in rms.items():
            require(abs(actual-expected["terminal_RMS"][key]) < 1e-9, "Canonical double-rig RMS is unsupported by the raw CSV")
    return {"episode": expected["episode"], "actual_samples": len(values), "actual_tail_samples": len(tail),
            "actual_initial_six_state": first_state, "actual_initial_q2_relative_rad": values[0]["q2_rad"],
            "requested_initial_absolute_state": [x, theta1, theta2], "actual_final_six_state": state_from_native(values[-1]),
            "independent_terminal_RMS": rms, "RMS_bounds": RMS_LIMITS, "actual_peaks": peaks,
            "terminal_window_s": [10, 12], "time_endpoint_tolerance_s": TIME_EPS,
            "mode": mode, "executed_source_sha256": sha, "disturbance_N": disturbance,
            "first_actual_hinge_geometry": upper_hinge_geometry(values[0]),
            "maximum_lower_tip_anchor_error_m": max(upper_hinge_geometry(item)["lower_tip_error_m"] for item in values),
            "maximum_upper_base_anchor_error_m": max(upper_hinge_geometry(item)["upper_base_error_m"] for item in values),
            "basis": "actual serial joint q/qd plus independently measured world direction of each rod"}


def validate_three_trials(session, canonical, mode, sha):
    require(canonical["mode"] == mode and len(canonical["trials"]) == 3, "Missing double-pendulum physical trial")
    require(canonical["passed"] is (mode == "student") and canonical["demo_validated"] is (mode == "reference_demo"),
            "Reference demonstration was confused with student completion")
    rows = read_rows(session)
    measured = [independent_trial(rows, expected, initial, mode, sha)
                for expected, initial in zip(canonical["trials"], TRIALS)]
    third = measured[2]
    require(abs(third["actual_initial_q2_relative_rad"]+.06) < .001
            and abs(third["actual_initial_six_state"][4]+.03) < .001,
            "Opposed-rod trial cannot distinguish relative q2 from the second absolute angle")
    return measured


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--unit-only", action="store_true", help="Student function probes and parser checks only; no Gazebo")
    mode.add_argument("--parser-only", action="store_true", help="Numerical parser/endpoint checks only; no course dependency or Gazebo")
    args = parser.parse_args()
    report = {"passed": False, "lesson": LESSON_KEY, "physical_sessions_started": 0, "scenarios": []}
    started = time.monotonic()
    sessions = []
    lesson = None
    before = None
    report_path = Path(args.report).expanduser().resolve() if args.report else None
    artifacts = (report_path.with_name(report_path.stem+"-artifacts") if report_path else
                 Path(tempfile.mkdtemp(prefix="gazeboteach-092-evidence-")))
    with tempfile.TemporaryDirectory(prefix="gazeboteach-092-private-") as folder:
        private = Path(folder)
        try:
            report["parser_regressions"] = parser_regressions()
            if not args.parser_only:
                lessons, errors = load_catalog(ROOT)
                candidates = [candidate for candidate in lessons if candidate.key == LESSON_KEY]
                require(len(candidates) == 1, "Expected one optional-B lesson: "+repr(errors))
                lesson = candidates[0]
                before = source_hashes(lesson.folder)
                report["unit_regressions"] = pure_unit_regressions(lesson, private)
            if not args.parser_only and not args.unit_only:
                demo = LessonSession(ROOT, lesson, storage=Storage(private/"demo"))
                sessions.append(("demo", demo))
                report["physical_sessions_started"] += 1
                demo.start(gui=False, terminals=False)
                demo_sha = digest(lesson.folder/"reference_controller.py")
                initial = await_bridge(demo, "reference_demo", demo_sha)
                native = validate_native_model(initial)
                demo.pause(True)
                rejection = verify_rejected(demo, "student")
                run = official_lab(demo, "demo-check")
                canonical = read_json(control_directory(demo)/"demo-validation.json")
                measured = validate_three_trials(demo, canonical, "reference_demo", demo_sha)
                require(actual_stats(demo)["paused"], "Double-rig demo failed to restore the original pause")
                require("verified" not in demo.storage.progress().get(lesson.key, set()), "Double-rig demonstration awarded student completion")
                require(digest(demo.workspace/"learner.py") == before["learner.py"], "Double-rig demonstration overwrote student TODOs")
                report["scenarios"].append({"name": "default_double_demo_without_assignment_credit", "passed": True,
                                            "native_model_check": native, "assignment_rejection": rejection, "official_run": run,
                                            "canonical_measurements": canonical, "independent_actual_trials": measured})
                demo.stop(reason="092 demonstration accepted without student credit")

                storage = Storage(private/"student")
                workspace = storage.prepare(lesson)
                (workspace/"learner.py").write_bytes((lesson.folder/"reference_controller.py").read_bytes())
                (workspace/"runtime.json").write_text('{"mode":"student"}\n', encoding="utf-8")
                student = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("student", student))
                report["physical_sessions_started"] += 1
                student.start(gui=False, terminals=False)
                student_sha = digest(workspace/"learner.py")
                initial = await_bridge(student, "student", student_sha)
                native = validate_native_model(initial)
                student.pause(False)
                original_files = source_hashes(workspace)
                result = student.verify()
                require(result["passed"] is True, "Completed private double-pendulum student failed real verification")
                canonical = read_json(control_directory(student)/"student-verification.json")
                measured = validate_three_trials(student, canonical, "student", student_sha)
                require(not actual_stats(student)["paused"], "Double-rig student verification did not restore running state")
                require("verified" in storage.progress().get(lesson.key, set()), "Completed double-rig student verification was not recorded")
                require(original_files == source_hashes(workspace), "Double-rig verifier changed personal exercise files")
                report["scenarios"].append({"name": "private_completed_student_double_rig_three_real_trials", "passed": True,
                                            "native_model_check": native, "verifier_result": result,
                                            "canonical_measurements": canonical, "independent_actual_trials": measured})

                student.pause(True)
                learner_path = workspace/"learner.py"
                valid = learner_path.read_bytes()
                previous_episode = read_json(control_directory(student)/"status.json")["episode"]
                try:
                    learner_path.write_bytes(valid+b"\n# test-only edit after this native controller was loaded\n")
                    offset = student.log_path.stat().st_size
                    failed = student.verify()
                    log = student.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
                    require(failed["passed"] is False and "重载" in log, "Stale double-rig student code was accepted")
                    state = read_json(control_directory(student)/"status.json")
                    require(state["source_sha256"] == student_sha and state["episode"] == previous_episode,
                            "Stale double-rig code rejection silently reloaded code or started a trial")
                    require(actual_stats(student)["paused"], "Stale-code rejection changed the double-rig pause")
                    report["scenarios"].append({"name": "double_student_saved_code_requires_reload", "passed": True,
                                                "loaded_source_sha256": student_sha, "edited_source_sha256": digest(learner_path),
                                                "verifier_result": failed, "raw_verifier_output": log,
                                                "physical_episode_unchanged": True})
                finally:
                    learner_path.write_bytes(valid)
                student.stop(reason="092 student and stale-code checks finished")

                storage = Storage(private/"zero")
                workspace = storage.prepare(lesson)
                code = (lesson.folder/"reference_controller.py").read_bytes()+b"\ndef feedback_force(error):\n    return 0.0\n"
                (workspace/"learner.py").write_bytes(code)
                (workspace/"runtime.json").write_text('{"mode":"student"}\n', encoding="utf-8")
                zero = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("zero", zero))
                report["physical_sessions_started"] += 1
                zero.start(gui=False, terminals=False)
                zero_sha = digest(workspace/"learner.py")
                initial = await_bridge(zero, "student", zero_sha, allow_control_error=True)
                native = validate_native_model(initial)
                zero.pause(True)
                rejection = verify_rejected(zero, "feedback_force")
                run = official_lab(zero, "observe", "--seconds", "3", timeout=35)
                observed = read_json(control_directory(zero)/"raw-observation.json")
                rows = [row for row in read_rows(zero) if row["episode"] == observed["episode"]]
                require(rows and observed["observation_only"] is True and observed["assignment_pass"] is False,
                        "Raw zero-control double-rig observation incorrectly earned completion credit")
                states = [state_from_native(row) for row in rows]
                peak = max(max(abs(state[2]), abs(state[4])) for state in states)
                initial_angle = max(abs(states[0][2]), abs(states[0][4]))
                require(peak > .35 and peak > 4*initial_angle,
                        "Zero double-rig controller lacks actual growing-angle evidence beyond its unit rejection")
                require(all(row["mode"] == "student" and row["source_sha256"] == zero_sha
                            and abs(float(row["python_force_N"])) < 1e-10 for row in rows),
                        "Raw double-rig failure observation did not execute the intended zero student")
                require(actual_stats(zero)["paused"], "Raw double-rig failure observation did not restore pause")
                require("verified" not in storage.progress().get(lesson.key, set()), "Raw failing observation recorded student completion")
                report["scenarios"].append({"name": "zero_double_student_rejected_and_actually_unstable", "passed": True,
                                            "native_model_check": native, "unit_gate_rejection": rejection,
                                            "official_run": run, "raw_observation": observed,
                                            "actual_initial_six_state": states[0], "actual_final_six_state": states[-1],
                                            "actual_samples": len(rows), "actual_peak_absolute_rod_angle_rad": peak,
                                            "evidence": "native serial joints corroborated by both actual world rod directions"})
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"] = {}
            for name, session in reversed(sessions):
                session.stop(reason="092 private acceptance finished")
                report["evidence"][name] = snapshot(session, artifacts/name)
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers, source_templates_unchanged=(before == source_hashes(lesson.folder)) if lesson else None,
                          elapsed_seconds=round(time.monotonic()-started, 3))
            if any(leftovers.values()) or report["source_templates_unchanged"] is False:
                report["passed"] = False
            if report_path:
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
