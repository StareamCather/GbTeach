#!/usr/bin/env python3
"""L14: real contact messages, free fall, support load, and sensor filtering.

Every physical case starts a fresh private ROS/Gazebo pair. Source mutations are
limited to these temporary exercises. Raw verifier messages and session logs
survive cleanup; input hashes and independent world statistics guard persistence
and pause restoration. A successful process start alone never passes this test.
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
from tests.lesson06_acceptance import actual_stats


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def edit_puck(path, mass=None, height=None):
    tree = ET.parse(str(path))
    puck = tree.find("./world/model[@name='puck']")
    require(puck is not None, "Private world lacks puck")
    if mass is not None:
        inertial = puck.find("link/inertial")
        inertial.find("mass").text = str(mass)
        # The lesson's fixed uniform box is 0.35 x 0.35 x 0.20 m.
        for name, value in (("ixx", mass*(.35**2+.2**2)/12),
                            ("iyy", mass*(.35**2+.2**2)/12),
                            ("izz", mass*(.35**2+.35**2)/12)):
            inertial.find("inertia/"+name).text = repr(value)
    if height is not None:
        pose = puck.find("pose")
        values = pose.text.split()
        values[2] = str(height)
        pose.text = " ".join(values)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def inspect_trace(observation):
    """Inspect saved real messages independently of the verifier's exit code."""
    phases = observation["phases"]
    trace = observation["trace"]
    stamps = [row["stamp_ns"] for row in trace]
    require(stamps and all(b > a for a, b in zip(stamps, stamps[1:])),
            "Raw contact header timestamps are not strictly increasing")
    drop = [row for row in trace if row["phase"] == "drop"]
    first = next((row for row in drop if row["states"]), None)
    require(first is not None, "No real collision in raw trace")
    empties = [row for row in drop if row["stamp_ns"] < first["stamp_ns"] and not row["states"]]
    require(len(empties) >= 3, "Raw trace lacks three actual airborne empty messages")
    require(phases["off_plate"]["empty_messages"] >= 10 and phases["off_plate"]["empty_span_s"] >= .2,
            "No sustained off-plate negative observation")
    require(abs(phases["off_plate"]["settled"]["position"][2]-.1) < .01,
            "Off-plate object did not actually rest on floor")
    stable_stamps = phases["settled"]["stamp_ns"]
    require(len(stable_stamps) >= 20 and (stable_stamps[-1]-stable_stamps[0])/1e9 >= .4,
            "Support window lacks real duration or distinct messages")
    stable_set = set(stable_stamps)
    stable = [row for row in drop if row["stamp_ns"] in stable_set]
    require(len(stable) == len(stable_stamps), "Steady summary references missing raw messages")
    all_records = 0
    forces = []
    for row in stable:
        require(row["states"], "Raw stable support message is empty")
        state_forces = []
        for state in row["states"]:
            require({state["collision1"], state["collision2"]} ==
                    {"bumper::plate::bumper_collision", "puck::link::collision"},
                    "Raw support does not involve exact expected collisions")
            require(len(state["positions"]) > 0, "Raw contact has no geometric point")
            state_forces.append(math.sqrt(sum(value**2 for value in state["force"])))
            all_records += 1
        forces.append(sum(state_forces)/len(state_forces))
    measured = sum(forces)/len(forces)
    require(abs(measured-observation["observations"]["support_mean_n"]) < 1e-7,
            "Reported support differs from saved per-message averages")
    require(all_records == phases["settled"]["native_contact_states"],
            "ROS message count was confused with native contact record count")
    return {"raw_messages": len(trace), "steady_messages": len(stable),
            "steady_native_records": all_records, "airborne_empty_messages": len(empties),
            "independently_recomputed_support_n": measured}


def verify_case(session, artifacts, label, expected=True, diagnosis=None):
    before = actual_stats(session)
    world = session.workspace/"world.sdf"
    original = world.read_bytes()
    destination = artifacts/(label+".json")
    destination.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"),
                             "--report", str(destination)], cwd=str(session.workspace),
                            env=session.env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=65)
    destination.with_suffix(".stdout.log").write_text(result.stdout, encoding="utf-8")
    require((result.returncode == 0) is expected,
            "Unexpected verifier outcome for {}: {}".format(label, result.stdout[-3500:]))
    require(destination.is_file(), "Verifier did not preserve actual observation JSON")
    observation = json.loads(destination.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "Report/process outcome disagree")
    if diagnosis:
        require(diagnosis.lower() in (result.stdout+json.dumps(observation)).lower(),
                "Missing specific failure diagnosis: "+diagnosis)
    independent = inspect_trace(observation) if expected else None
    after = actual_stats(session)
    require(before["paused"] == after["paused"], "Verifier changed original pause state")
    require(world.read_bytes() == original, "Verifier overwrote the personal world")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"],
                "Rejected source changed paused physics before reporting failure")
    item = {"name": label, "passed": True, "expected_acceptance": expected,
            "world_stats_before": before, "world_stats_after": after,
            "world_sha256": hashlib.sha256(original).hexdigest(),
            "report": str(destination), "independent_raw_trace_checks": independent,
            "observation": {key: value for key, value in observation.items() if key != "trace"}}
    print(json.dumps({"scenario": label, "passed": True,
                      "verifier_accepted": expected}, ensure_ascii=False), flush=True)
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:14"]
    require(len(matches) == 1, "Expected L14 in actual catalog: "+repr(errors))
    lesson = matches[0]
    original_source = hashes(lesson.folder)
    sessions = []
    report = {"passed": False, "lesson": lesson.key, "scenarios": [],
              "source_sha256": original_source, "physical_sessions_started": 0}
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l14-") as directory:
        try:
            for label, mass, height, paused in (("baseline", 1., 1.4, True),
                                                ("mass_only", 2., 1.4, False),
                                                ("height_only", 1., 2.2, True)):
                storage = Storage(Path(directory)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    edit_puck(workspace/"world.sdf", mass=mass, height=height)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                report["physical_sessions_started"] += 1
                session.pause(paused)
                item = verify_case(session, artifacts, label)
                item["single_variable"] = {"mass_kg": mass, "release_center_z_m": height}
                report["scenarios"].append(item)
                if label == "height_only":
                    # Valid edit without server restart must fail before control.
                    path = workspace/"world.sdf"
                    saved = path.read_bytes()
                    try:
                        edit_puck(path, mass=1.5)
                        report["scenarios"].append(verify_case(
                            session, artifacts, "valid_saved_but_not_reloaded", False, "reload"))
                    finally:
                        path.write_bytes(saved)
                    for name, xpath, text, diagnosis in (
                        ("wrong_sensor_collision", "./world/model[@name='bumper']/link/sensor/contact/collision", "visual", "collision"),
                        ("misleading_frame_label", "./world/model[@name='bumper']/link/sensor/plugin/frameName", "imaginary_frame", "frame"),
                        ("wrong_uniform_inertia", "./world/model[@name='puck']/link/inertial/inertia/ixx", "0.8", "inertia")):
                        try:
                            tree = ET.fromstring(saved)
                            element = tree.find(xpath)
                            require(element is not None, "Fault-injection target missing: "+xpath)
                            element.text = text
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, name, False, diagnosis))
                        finally:
                            path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired"))
                session.stop(reason="L14 private scenario finished")
            positive = {case["name"]: case["observation"] for case in report["scenarios"] if case["expected_acceptance"]}
            baseline, heavy, high = (positive[key] for key in ("baseline", "mass_only", "height_only"))
            force0, force2, forceh = (case["observations"]["support_mean_n"] for case in (baseline, heavy, high))
            require(abs(force2/force0-2) < .15, "Doubling actual mass did not double observed support")
            require(abs(forceh/force0-1) < .10, "Release height changed sustained support unexpectedly")
            fall0, fall2, fallh = (case["phases"]["airborne"]["observed_fall_s"] for case in (baseline, heavy, high))
            require(abs(fall2-fall0) < .06 and fallh-fall0 > .10,
                    "Actual fall timing did not distinguish mass and release-height experiments")
            report["single_variable_comparisons"] = {"double_mass_support_ratio": force2/force0,
                "higher_release_support_ratio": forceh/force0, "observed_fall_s": [fall0, fall2, fallh]}
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["cleanup_errors"] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L14 acceptance final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(name+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(name+"-session.json")))
                except Exception as error:
                    report["cleanup_errors"].append(name+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = original_source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "scenarios"},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
