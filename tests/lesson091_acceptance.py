#!/usr/bin/env python3
"""L09 optional A: private student code, real native joint CSV, separate demo credit.

--unit-only never starts Gazebo. The default runs three sequential, isolated
sessions: reference demonstration, completed private student copy, and a zero
controller's raw failure observation. Original TODOs and user data stay intact.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson06_acceptance import actual_stats

TIME_EPS = 1e-7  # Floating endpoint tolerance, 100000 times smaller than a control interval.


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes(folder):
    return {str(path.relative_to(folder)): digest(path) for path in folder.rglob("*")
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc"}


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def closed_time(value, lower, upper):
    return lower-TIME_EPS <= value <= upper+TIME_EPS


def disturbance_matches(time_s, actual, amplitude):
    # Boundary rows retain the force actually emitted by the native half-open
    # interval. Floating display cannot decide which side that exact tick used.
    if abs(time_s-4) <= TIME_EPS or abs(time_s-4.25) <= TIME_EPS:
        return abs(actual) < 1e-7 or abs(actual-amplitude) < 1e-7
    expected = amplitude if 4+TIME_EPS < time_s < 4.25-TIME_EPS else 0.
    return abs(actual-expected) < 1e-7


def floating_endpoint_regression():
    """Numerical boundary check only; these numbers are not physical evidence."""
    points = [8+.01*index for index in range(201)]
    points[0], points[-1] = 7.999999999999996, 10.000000000000002
    selected = [value for value in points if closed_time(value, 8, 10)]
    require(len(selected) == 201 and len([value for value in points if 8 <= value <= 10]) == 199,
            "Floating endpoint reproduction did not include both declared closed-window endpoints")
    require(not closed_time(7.99, 8, 10) and not closed_time(10.01, 8, 10),
            "Endpoint tolerance accidentally admitted a neighboring 100 Hz sample")
    require(disturbance_matches(4-1e-14, 2, 2) and disturbance_matches(4+1e-14, 0, 2)
            and disturbance_matches(4.25-1e-14, 0, 2) and disturbance_matches(4.25+1e-14, 2, 2)
            and disturbance_matches(4.01, 2, 2) and not disturbance_matches(4.01, 0, 2)
            and disturbance_matches(4.26, 0, 2) and not disturbance_matches(4.26, 2, 2),
            "Endpoint handling changed the strict interior/exterior disturbance requirements")
    return {"kind": "pure_floating_point_regression_not_physics", "terminal_window_s": [8, 10],
            "endpoint_tolerance_s": TIME_EPS, "representative_first_last": [points[0], points[-1]],
            "raw_comparison_samples": 199, "closed_endpoint_samples": len(selected),
            "neighboring_control_ticks_excluded": True, "actual_force_values_never_rewritten": True}


def pure_unit_regressions(lesson, private):
    """Exercise the trusted behavior probes with useful, deliberately wrong code."""
    reference = (lesson.folder / "reference_controller.py").read_bytes()
    variants = (
        ("unfinished_TODO_rejected", (lesson.folder / "learner.py").read_bytes(), False, "NotImplementedError"),
        ("completed_private_reference", reference, True, None),
        ("zero_feedback_rejected", reference+b"\ndef feedback_force(error):\n    return 0.0\n", False, "feedback_force"),
        ("wrong_feedback_sign_rejected", reference+b"\ndef feedback_force(error):\n    return float(K @ error)\n", False, "feedback_force"),
        ("native_clip_cannot_mask_missing_student_clip", reference+b"\ndef clip_force(force, force_limit=15.0):\n    return float(force)\n", False, "clip_force"),
        ("one_sided_clip_rejected", reference+b"\ndef clip_force(force, force_limit=15.0):\n    return float(np.clip(force, 0, force_limit))\n", False, "clip_force"),
    )
    checks = []
    for name, code, accepted, diagnostic in variants:
        path = private / (name+".py")
        path.write_bytes(code)
        result = subprocess.run([sys.executable, str(lesson.folder / "unit_probe.py"), str(path)],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
        require((result.returncode == 0) is accepted, "Unexpected behavior-check result: "+name+"\n"+result.stdout+result.stderr)
        if diagnostic:
            require(diagnostic in result.stdout+result.stderr, "Missing useful failure diagnosis: "+name)
        item = {"name": name, "expected_acceptance": accepted, "returncode": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr, "learner_sha256": digest(path)}
        if accepted:
            measured = json.loads(result.stdout.splitlines()[-1])
            require(measured["passed"] is True and measured["source_sha256"] == digest(path),
                    "Behavior probes did not identify the code actually executed")
            item["behavior_checks"] = measured
        checks.append(item)
    return {"kind": "pure_python_behavior_not_physics", "gazebo_started": False, "checks": checks}


def control_directory(session):
    return Path(session.env["GAZEBOTEACH_SESSION_DIR"]) / "control"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def await_bridge(session, mode, sha, timeout=12, allow_control_error=False):
    deadline = time.monotonic()+timeout
    last = None
    while time.monotonic() < deadline:
        require(session.running, "Gazebo server exited before the native controller became ready")
        try:
            last = read_json(control_directory(session) / "status.json")
            if last.get("mode") == mode and last.get("source_sha256") == sha:
                require(allow_control_error or not last.get("error"), "Native bridge failed: "+str(last["error"]))
                require(last.get("numpy_version"), "Embedded Python did not confirm NumPy")
                return last
        except (OSError, ValueError):
            pass
        time.sleep(.05)
    raise AssertionError("No ready native bridge with the required mode/source SHA: "+repr(last))


def official_lab(session, action, *arguments, timeout=150):
    offset = session.log_path.stat().st_size
    process = session._spawn("091 official lab acceptance", ["/usr/bin/python3", str(session.lesson.folder / "lab.py"), action, *arguments])
    try:
        result = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        terminate_owned(process)
        raise RuntimeError("Bounded official lab operation timed out: "+action)
    output = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
    require(result == 0, "Official lab operation failed: "+action+"\n"+output)
    return {"returncode": result, "raw_output": output}


def read_rows(session):
    with (control_directory(session) / "control.csv").open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def independent_trial(rows, expected, mode, sha, compare_canonical=True):
    """Recompute metrics from actual CSV, never from the verifier's summary."""
    selected = [row for row in rows if row["episode"] == expected["episode"]
                and closed_time(float(row["episode_time_s"]), 0, 10)]
    require(len(selected) >= 990, "Insufficient actual 100 Hz control samples")
    numeric = ("episode_time_s", "sim_time_s", "x_m", "xdot_m_s", "q1_rad", "q1dot_rad_s",
               "theta_up_rad", "pole_axis_x", "pole_axis_z", "python_force_N", "command_force_N",
               "disturbance_N", "applied_force_N", "callback_count", "episode_callback_count")
    values = [{key: float(row[key]) for key in numeric} for row in selected]
    require(all(math.isfinite(value) for row in values for value in row.values()), "Non-finite native state/force telemetry")
    require(all(row["mode"] == mode and row["source_sha256"] == sha and row["controller_ok"] == "1"
                for row in selected), "Actual executing code or mode differs from the submitted copy")
    require(abs(values[0]["x_m"]-expected["initial_x_m"]) <= .001
            and abs(values[0]["q1_rad"]-expected["initial_theta_rad"]) <= .001,
            "Real trial did not start at its required nonzero state")
    for before, after in zip(values, values[1:]):
        require(abs(after["episode_time_s"]-before["episode_time_s"]-.01) <= .0001,
                "Student callback did not run at the declared simulated 100 Hz")
        require(after["callback_count"]-before["callback_count"] == 1
                and after["episode_callback_count"]-before["episode_callback_count"] == 1,
                "Native callback accounting skipped or duplicated a student call")
    for item in values:
        angle = (item["q1_rad"]+math.pi) % (2*math.pi)-math.pi
        require(abs(angle-item["theta_up_rad"]) < 1e-7, "Reported upright angle differs from actual joint position")
        require(abs(math.sin(item["q1_rad"])-item["pole_axis_x"]) <= .002
                and abs(math.cos(item["q1_rad"])-item["pole_axis_z"]) <= .002,
                "Actual link direction does not corroborate the native hinge angle convention")
        require(abs(item["python_force_N"]-item["command_force_N"]) <= 1e-6,
                "Native clipping hid an incomplete student limit function")
        require(abs(item["applied_force_N"]-item["command_force_N"]-item["disturbance_N"]) <= 1e-6,
                "Actual commanded force does not contain the recorded disturbance")
        require(disturbance_matches(item["episode_time_s"], item["disturbance_N"], expected["disturbance_N"]),
                "Native disturbance timing/amplitude differs from the trial")
    tail = [item for item in values if closed_time(item["episode_time_s"], 8, 10)]
    require(len(tail) >= 195, "Missing actual final two seconds of the trial")
    limits = {"x_m": .02, "xdot_m_s": .035, "theta_up_rad": .01, "q1dot_rad_s": .04}
    rms = {key: math.sqrt(sum(item[key]**2 for item in tail)/len(tail)) for key in limits}
    peaks = {key: max(abs(item[key]) for item in values) for key in ("x_m", "theta_up_rad", "command_force_N")}
    require(peaks["x_m"] <= 1.2 and peaks["theta_up_rad"] <= .35 and peaks["command_force_N"] <= 15.000001,
            "Actual trial exceeded the declared local stability/rail/input bounds")
    require(all(rms[key] <= limit for key, limit in limits.items()), "Actual final-two-second RMS did not converge: "+repr(rms))
    if compare_canonical:
        for key, actual in rms.items():
            require(abs(actual-expected["terminal_RMS"][key]) < 1e-9, "Canonical RMS is unsupported by the saved native CSV")
        if "terminal_samples" in expected:
            require(len(tail) == expected["terminal_samples"], "Canonical RMS window uses different actual sample endpoints")
    return {"episode": expected["episode"], "actual_samples": len(values), "actual_tail_samples": len(tail),
            "initial_state": values[0], "final_state": values[-1], "actual_peaks": peaks,
            "independent_terminal_RMS": rms, "RMS_bounds": limits,
            "terminal_window_s": [8, 10], "time_endpoint_tolerance_s": TIME_EPS,
            "simulated_callback_period_s": .01, "mode": mode, "executed_source_sha256": sha}


def validate_three_trials(session, canonical, mode, sha):
    require(canonical["mode"] == mode and len(canonical["trials"]) == 3, "Canonical exercise omitted one of the three physical trials")
    require(canonical["passed"] is (mode == "student") and canonical["demo_validated"] is (mode == "reference_demo"),
            "Demonstration credit and student completion have been conflated")
    rows = read_rows(session)
    measured = [independent_trial(rows, trial, mode, sha) for trial in canonical["trials"]]
    require([trial["initial_theta_rad"] for trial in canonical["trials"]] == [.08, -.08, -.06]
            and canonical["trials"][2]["disturbance_N"] == 2., "Required signed initial conditions or disturbed trial are missing")
    return measured


def verify_rejected(session, diagnostic):
    original = actual_stats(session)
    offset = session.log_path.stat().st_size
    result = session.verify()
    output = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
    require(result["passed"] is False and diagnostic in output, "Assignment check did not clearly reject "+diagnostic+"\n"+output)
    require("verified" not in session.storage.progress().get(session.lesson.key, set()),
            "Rejected work was recorded as completed")
    after = actual_stats(session)
    require(after["paused"] == original["paused"], "Rejected assignment changed the original pause state")
    return {"verifier_result": result, "raw_verifier_output": output, "original_stats": original,
            "stats_after": after, "assignment_completed": False}


def snapshot(session, destination):
    destination.mkdir(parents=True, exist_ok=True)
    copied = {}
    for path in control_directory(session).glob("*"):
        if path.is_file() and path.suffix in (".csv", ".json"):
            target = destination/path.name
            shutil.copyfile(str(path), str(target))
            copied[path.name] = {"path": str(target), "sha256": digest(target), "bytes": target.stat().st_size}
    if session.log_path.is_file():
        target = destination/"runtime.log"
        shutil.copyfile(str(session.log_path), str(target))
        copied["runtime.log"] = {"path": str(target), "sha256": digest(target)}
    return copied


def recompute_saved_csv(lesson, args):
    original_path = Path(args.recompute_csv).expanduser().resolve()
    original = read_json(original_path)
    before = source_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "input_report": str(original_path),
              "kind": "offline_reanalysis_of_retained_real_csv_not_a_new_physics_run",
              "physical_sessions_started": 0, "floating_endpoint_regression": floating_endpoint_regression(), "sessions": []}
    try:
        for name in ("demo", "student"):
            evidence = original["evidence"][name]["control.csv"]
            path = Path(evidence["path"])
            require(digest(path) == evidence["sha256"], "Retained actual CSV bytes changed: "+str(path))
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            mode = "reference_demo" if name == "demo" else "student"
            candidates = [item["canonical_measurements"] for item in original["scenarios"]
                          if item.get("canonical_measurements", {}).get("mode") == mode]
            require(len(candidates) == 1, "Missing original canonical mode report: "+mode)
            trials = []
            for expected in candidates[0]["trials"]:
                measured = independent_trial(rows, expected, mode, expected["source_sha256"], compare_canonical=False)
                require(measured["actual_tail_samples"] == 201, "Retained 100 Hz CSV lacks one of the closed 8–10 s endpoints")
                measured["previous_terminal_RMS"] = expected["terminal_RMS"]
                trials.append(measured)
            report["sessions"].append({"name": name, "original_csv": str(path), "csv_sha256": evidence["sha256"], "trials": trials})
        report["passed"] = True
    except Exception as error:
        report["error"] = "{}: {}".format(type(error).__name__, error)
    report["source_templates_unchanged"] = before == source_hashes(lesson.folder)
    report["passed"] = report["passed"] and report["source_templates_unchanged"]
    if args.report:
        output = Path(args.report).expanduser().resolve()
        require(output != original_path, "Offline output must preserve the original run report")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", help="JSON report; raw telemetry is retained in a sibling -artifacts directory")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--unit-only", action="store_true", help="Only Python behavior probes; no Gazebo")
    mode.add_argument("--recompute-csv", metavar="OLD_REPORT", help="Reanalyze the retained real CSV files without starting Gazebo")
    args = parser.parse_args()
    lessons, errors = load_catalog(ROOT)
    candidates = [lesson for lesson in lessons if lesson.key == "classic11-noetic:091"]
    require(len(candidates) == 1, "Expected one optional-A lesson: "+repr(errors))
    lesson = candidates[0]
    if args.recompute_csv:
        return recompute_saved_csv(lesson, args)
    before = source_hashes(lesson.folder)
    report_path = Path(args.report).expanduser().resolve() if args.report else None
    artifacts = (report_path.with_name(report_path.stem+"-artifacts") if report_path else
                 Path(tempfile.mkdtemp(prefix="gazeboteach-091-evidence-")))
    report = {"passed": False, "lesson": lesson.key, "unit_only": args.unit_only,
              "scenarios": [], "physical_sessions_started": 0, "artifacts_directory": str(artifacts),
              "floating_endpoint_regression": floating_endpoint_regression()}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-091-private-") as folder:
        private = Path(folder)
        try:
            report["unit_regressions"] = pure_unit_regressions(lesson, private)
            if not args.unit_only:
                demo = LessonSession(ROOT, lesson, storage=Storage(private/"demo"))
                sessions.append(("demo", demo))
                report["physical_sessions_started"] += 1
                demo.start(gui=False, terminals=False)
                demo_sha = digest(lesson.folder/"reference_controller.py")
                initial = await_bridge(demo, "reference_demo", demo_sha)
                demo.pause(True)
                rejection = verify_rejected(demo, "student")
                run = official_lab(demo, "demo-check")
                canonical = read_json(control_directory(demo)/"demo-validation.json")
                measured = validate_three_trials(demo, canonical, "reference_demo", demo_sha)
                require(actual_stats(demo)["paused"], "Reference demonstration did not restore the original pause")
                require("verified" not in demo.storage.progress().get(lesson.key, set()), "Reference demonstration awarded student completion")
                require(digest(demo.workspace/"learner.py") == before["learner.py"], "Reference demonstration overwrote the learner's TODOs")
                report["scenarios"].append({"name": "default_demo_real_physics_without_student_credit", "passed": True,
                                            "initial_bridge": initial, "assignment_rejection": rejection, "official_run": run,
                                            "canonical_measurements": canonical, "independent_actual_trials": measured})
                demo.stop(reason="091 reference demonstration accepted without assignment credit")

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
                student.pause(False)
                original_files = source_hashes(workspace)
                result = student.verify()
                require(result["passed"] is True, "Completed private student controller failed real verification")
                canonical = read_json(control_directory(student)/"student-verification.json")
                measured = validate_three_trials(student, canonical, "student", student_sha)
                require(not actual_stats(student)["paused"], "Student verification did not restore the running state")
                require("verified" in storage.progress().get(lesson.key, set()), "Completed student's verification was not recorded")
                require(original_files == source_hashes(workspace), "Student verification changed personal exercise files")
                report["scenarios"].append({"name": "temporary_reference_as_student_three_real_trials", "passed": True,
                                            "initial_bridge": initial, "verifier_result": result,
                                            "canonical_measurements": canonical, "independent_actual_trials": measured})

                student.pause(True)
                learner_path = workspace/"learner.py"
                valid = learner_path.read_bytes()
                original_episode = read_json(control_directory(student)/"status.json")["episode"]
                try:
                    learner_path.write_bytes(valid+b"\n# test-only edit after the native module was loaded\n")
                    offset = student.log_path.stat().st_size
                    failed = student.verify()
                    log = student.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
                    require(failed["passed"] is False and "重载" in log, "Stale loaded code SHA was accepted")
                    state = read_json(control_directory(student)/"status.json")
                    require(state["source_sha256"] == student_sha and state["episode"] == original_episode,
                            "Stale-source rejection started a physical trial or silently reloaded the code")
                    require(actual_stats(student)["paused"], "Stale-source rejection changed pause")
                    report["scenarios"].append({"name": "saved_code_requires_explicit_reload", "passed": True,
                                                "actual_loaded_sha256": state["source_sha256"], "edited_learner_sha256": digest(learner_path),
                                                "verifier_result": failed, "raw_verifier_output": log,
                                                "physical_episode_unchanged": True})
                finally:
                    learner_path.write_bytes(valid)
                student.stop(reason="091 student and stale-source checks complete")

                storage = Storage(private/"zero")
                workspace = storage.prepare(lesson)
                zero_code = (lesson.folder/"reference_controller.py").read_bytes()+b"\ndef feedback_force(error):\n    return 0.0\n"
                (workspace/"learner.py").write_bytes(zero_code)
                (workspace/"runtime.json").write_text('{"mode":"student"}\n', encoding="utf-8")
                zero = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("zero", zero))
                report["physical_sessions_started"] += 1
                zero.start(gui=False, terminals=False)
                zero_sha = digest(workspace/"learner.py")
                # A deliberately unstable controller can leave its automatic
                # startup episode's local range before the headless ready check.
                # observe requests a fresh initial condition, never hides this.
                initial = await_bridge(zero, "student", zero_sha, allow_control_error=True)
                zero.pause(True)
                rejection = verify_rejected(zero, "feedback_force")
                run = official_lab(zero, "observe", "--seconds", "3", timeout=35)
                observed = read_json(control_directory(zero)/"raw-observation.json")
                rows = [row for row in read_rows(zero) if row["episode"] == observed["episode"]]
                require(rows and observed["observation_only"] is True and observed["assignment_pass"] is False,
                        "Raw failure demonstration was incorrectly treated as student credit")
                angles = [abs(float(row["theta_up_rad"])) for row in rows]
                require(max(angles) > .35 and max(angles) > 4*abs(float(rows[0]["theta_up_rad"])),
                        "Zero controller lacks actual growing-angle evidence; a unit rejection is not physical evidence")
                require(all(row["mode"] == "student" and row["source_sha256"] == zero_sha
                            and abs(float(row["python_force_N"])) < 1e-10 for row in rows),
                        "Zero-controller observation did not execute the intended student code")
                require(actual_stats(zero)["paused"], "Raw failure observation did not restore original pause")
                report["scenarios"].append({"name": "zero_student_rejected_and_actually_unstable", "passed": True,
                                            "initial_bridge": initial, "unit_gate_rejection": rejection, "official_run": run,
                                            "raw_observation": observed, "actual_samples": len(rows),
                                            "actual_initial_angle_rad": float(rows[0]["theta_up_rad"]),
                                            "actual_peak_angle_rad": max(angles),
                                            "evidence": "real native joint CSV; not inferred from a failed unit test"})
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"] = {}
            for name, session in reversed(sessions):
                session.stop(reason="091 private acceptance finished")
                report["evidence"][name] = snapshot(session, artifacts/name)
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers, source_templates_unchanged=before == source_hashes(lesson.folder),
                          elapsed_seconds=round(time.monotonic()-started, 3))
            if any(leftovers.values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            if report_path:
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
