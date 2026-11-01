#!/usr/bin/env python3
"""Read real multi-channel joint feedback; independently infer mass and arm."""
import argparse
import hashlib
import importlib.util
import json
import math
import os
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
from tests.lesson05_acceptance import parsed_model, first
from tests.lesson06_acceptance import actual_stats


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def edit_parameters(lesson, workspace, mass=None, arm=None):
    specification = importlib.util.spec_from_file_location("l19_config_test", lesson.folder/"config.py")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    previous = Path.cwd()
    try:
        os.chdir(str(workspace))
        module.edit(mass=mass, arm=arm)
    finally:
        os.chdir(str(previous))


def actual_pose(session, name):
    # This course requires the exact SDK; use its stable existing read channel,
    # retaining CLI -i only for the full actual joint/collision metadata.
    require(session.native_control is not None, 'L19 acceptance needs the exact persistent SDK backend')
    value = session.native_control.model_pose(name)
    return [value['pose']['position'][axis] for axis in ('x','y','z')]+value['rpy']


def inspect_raw(report, mass, arm):
    load, moment = mass*9.81, mass*9.81*arm
    expected = {
        "native_child": ([0, 0, -load], [0, moment, 0], "load_link"),
        "native_parent_to_child": ([0, 0, load], [0, -moment, 0], "load_link"),
        "native_sensor_x90": ([0, -load, 0], [0, 0, -moment], "sensor_x90"),
        "stock_child": ([0, 0, -load], [0, moment, 0], "stock_load_link")}
    measurements = {}
    for source, (force, torque, frame) in expected.items():
        rows = [row for row in report["trace"] if row["source"] == source and row["disposition"] == "accepted"]
        stamps = [row["stamp_ns"] for row in rows]
        require(len(rows) >= 30 and all(b > a for a, b in zip(stamps, stamps[1:])), "Actual wrench window lacks fresh unique samples")
        span = (stamps[-1]-stamps[0])/1e9
        require(span >= .6 and 40 <= (len(rows)-1)/span <= 60, "Actual wrench sampling duration/frequency invalid")
        for row in rows:
            require(row["frame"] == frame, "Wrench axes have the wrong frame label")
            require(all(math.isfinite(v) for v in row["force"]+row["torque"]), "Actual wrench contains non-finite values")
            require(max(abs(a-b) for a, b in zip(row["force"], force)) < .06,
                    "Actual force differs from the independent gravity/frame prediction")
            require(max(abs(a-b) for a, b in zip(row["torque"], torque)) < .04,
                    "Actual joint-anchor torque differs from independent r cross F")
        measurements[source] = {"messages": len(rows), "span_s": span, "rate_hz": (len(rows)-1)/span,
                                "mean_force": [sum(row["force"][axis] for row in rows)/len(rows) for axis in range(3)],
                                "mean_torque": [sum(row["torque"][axis] for row in rows)/len(rows) for axis in range(3)]}
    a, b, c, d = (measurements[name] for name in expected)
    for field in ("mean_force", "mean_torque"):
        require(max(abs(x+y) for x, y in zip(a[field], b[field])) < .03, "Opposite measurement directions are not negatives")
        require(max(abs(x-y) for x, y in zip(a[field], d[field])) < .03, "Native and stock steady feedback disagree")
        transformed = [a[field][0], a[field][2], -a[field][1]]  # Rx(-pi/2)
        require(max(abs(x-y) for x, y in zip(transformed, c[field])) < .03, "Rotated sensor axes do not preserve the same physical wrench")
    inferred_mass = -a["mean_force"][2]/9.81
    inferred_arm = -a["mean_torque"][1]/a["mean_force"][2]
    require(abs(inferred_mass-mass) < .01 and abs(inferred_arm-arm) < .005,
            "Actual wrench does not identify the current mass and lever arm")
    actual_rigs = {}
    for rig, model, link, joint, anchor, truth_key in (
            ('native','weighing_rig','load_link','mount_joint',[0,0,1.2],'truth'),
            ('stock','stock_weighing_rig','stock_load_link','stock_mount_joint',[0,1,1.2],'stock_truth')):
        properties = report['rig_properties'][rig]
        require(abs(properties['mass_kg']-mass) < 1e-9 and properties['gravity_mode'] is True,
                'Actual '+rig+' physical mass/gravity differs from prepared input')
        require(max(abs(a-b) for a,b in zip(properties['com_position'],[arm,0,0])) < 1e-9,
                'Actual '+rig+' center of mass did not move with the arm parameter')
        require(max(abs(a-mass*.2**2/6) for a in properties['inertia_diagonal']) < 1e-10 and
                max(abs(a) for a in properties['inertia_offdiagonal']) < 1e-10,
                'Actual '+rig+' uniform cube inertia differs')
        raw = parsed_model(report['rig_geometry'][rig]['raw_model'])
        require(first(raw,'name').rsplit('::',1)[-1] == model,'Actual model identity differs')
        actual_link = next(v for v in raw['link'] if first(v,'name').rsplit('::',1)[-1] == link)
        actual_joint = next(v for v in raw['joint'] if first(v,'name').rsplit('::',1)[-1] == joint)
        require(first(actual_joint,'type') in ('FIXED','8') and first(actual_joint,'parent') == 'world' and
                first(actual_joint,'child').rsplit('::',1)[-1] == link,'Actual independent fixed joint topology differs')
        require(len(actual_joint.get('sensor',[])) == (3 if rig == 'native' else 0),'Stock/native joint readers are not isolated')
        local_anchor = first(actual_joint,'pose');position = first(local_anchor,'position');orientation = first(local_anchor,'orientation')
        xyz = [float(first(position,key)) for key in ('x','y','z')]
        quat = [float(first(orientation,key)) for key in ('x','y','z','w')]
        require(max(abs(v) for v in xyz+quat[:3]) < 1e-9 and abs(abs(quat[3])-1) < 1e-9,
                'Actual child-frame joint anchor or orientation differs')
        collision = next(v for v in actual_link['collision'] if first(v,'name').rsplit('::',1)[-1] == 'payload_collision')
        box = first(first(first(collision,'geometry'),'box'),'size')
        require(max(abs(float(first(box,key))-.2) for key in ('x','y','z')) < 1e-9,'Actual payload physical box size differs')
        collision_pose=first(collision,'pose');cp=first(collision_pose,'position')
        require(max(abs(float(first(cp,key))-wanted) for key,wanted in zip(('x','y','z'),(arm,0,0))) < 1e-9,
                'Actual collision did not follow the paired arm parameter')
        truth = report[truth_key]
        require(len(truth) >= 3,'No actual '+rig+' state observations support equilibrium')
        low=max(v['stamp_ns'][0] for v in report['channels'].values());high=min(v['stamp_ns'][-1] for v in report['channels'].values())
        common=[v for v in truth if low <= (v['clock_before_ns']+v['clock_after_ns'])/2 <= high]
        require(high-low >= 500000000 and len(common) >= 3,'Both physical rigs need three real queries in the same common window')
        for value in truth+[report['initial_truth' if rig=='native' else 'stock_initial_truth'],report['final_truth' if rig=='native' else 'stock_final_truth']]:
            require(math.sqrt(sum((a-b)**2 for a,b in zip(value['position'],anchor))) <= .005,'Actual '+rig+' joint-anchor/child world position differs')
            q=value['orientation']
            require(math.sqrt(sum(v*v for v in q[:3])) <= .005 and abs(math.sqrt(sum(v*v for v in q))-1) <= 1e-6,'Actual '+rig+' child axes are not world-aligned')
            require(math.sqrt(sum(v*v for v in value['linear'])) <= .01 and math.sqrt(sum(v*v for v in value['angular'])) <= .01,'Actual '+rig+' is not static')
        actual_rigs[rig]={'model':model,'joint':joint,'link':link,'actual_properties':properties,
                          'anchor_child_position':xyz,'anchor_child_orientation':quat,
                          'actual_world_link_positions':[v['position'] for v in common],
                          'common_window_queries':len(common)}
    return {"channels": measurements, "inferred_mass_kg": inferred_mass, "inferred_arm_m": inferred_arm,
            'independent_physical_rigs':actual_rigs}


def verify_case(session, artifacts, label, mass, arm, expected=True, diagnosis=None):
    before = actual_stats(session)
    pose_before = {model: actual_pose(session, model) for model in ("weighing_rig", "stock_weighing_rig")}
    world, path = session.workspace/"world.sdf", artifacts/(label+".json")
    original = world.read_bytes()
    (artifacts/(label+"-world.sdf")).write_bytes(original)
    answer = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                            cwd=str(session.workspace), env=session.env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=80)
    path.with_suffix(".stdout.log").write_text(answer.stdout, encoding="utf-8")
    require((answer.returncode == 0) is expected, "Unexpected verifier result {}: {}".format(label, answer.stdout[-3200:]))
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "Process/JSON result mismatch")
    if diagnosis:
        require(diagnosis.lower() in (answer.stdout+json.dumps(observation)).lower(), "Missing specific diagnosis: "+diagnosis)
    checked = inspect_raw(observation, mass, arm) if expected else None
    after = actual_stats(session)
    pose_after = {model: actual_pose(session, model) for model in ("weighing_rig", "stock_weighing_rig")}
    require(before["paused"] == after["paused"], "Verifier failed to restore actual pause state")
    require(max(abs(a-b) for model in pose_before for a, b in zip(pose_before[model], pose_after[model])) < 1e-5, "World-fixed fixture moved during observation")
    require(world.read_bytes() == original, "Verifier rewrote the learner's SDF")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Rejected input advanced paused physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected, "raw_report": str(path),
            "input_world_file": str(artifacts/(label+"-world.sdf")), "input_world_sha256": hashlib.sha256(original).hexdigest(),
            "independent_raw_checks": checked, "stats_before": before, "stats_after": after,
            "fixture_pose_before": pose_before, "fixture_pose_after": pose_after,
            "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [l for l in lessons if l.key == "classic11-noetic:19"]
    require(len(found) == 1, "Expected L19: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": []}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l19-") as folder:
        try:
            for label, mass, arm, paused in (("baseline", 1., .3, True), ("mass_only", 2., .3, False), ("arm_only", 1., .5, True)):
                storage = Storage(Path(folder)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":edit_parameters(lesson, workspace, mass=mass, arm=arm)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label, mass, arm))
                if label == "arm_only":
                    path, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        edit_parameters(lesson, workspace, mass=1.5)
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", mass, arm, False, "reload"))
                    finally:path.write_bytes(saved)
                    for bad, xpath, value, diagnosis in (
                        ("missing_feedback_contract", "./world/model[@name='weighing_rig']/joint/physics/provide_feedback", "false", "feedback"),
                        ("wrong_measure_direction", "./world/model[@name='weighing_rig']/joint/sensor/force_torque/measure_direction", "parent_to_child", "direction"),
                        ("wrong_uniform_inertia", "./world/model[@name='weighing_rig']/link[@name='load_link']/inertial/inertia/ixx", "1", "inertia"),
                        ("stock_mass_drift", "./world/model[@name='stock_weighing_rig']/link[@name='stock_load_link']/inertial/mass", "1.25", "stock"),
                        ("stock_anchor_drift", "./world/model[@name='stock_weighing_rig']/pose", "0 1.1 1.2 0 0 0", "anchor"),
                        ("stock_missing_feedback", "./world/model[@name='stock_weighing_rig']/joint/physics/provide_feedback", "false", "feedback")):
                        try:
                            tree = ET.fromstring(saved);node = tree.find(xpath)
                            require(node is not None, "Missing fault injection field: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, bad, mass, arm, False, diagnosis))
                        finally:path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired", mass, arm))
                session.stop(reason="L19 physical scenario finished")
            for channel in ("native_child", "stock_child"):
                positive = {c["name"]: c["independent_raw_checks"]["channels"][channel] for c in report["scenarios"] if c["expected_acceptance"]}
                a, b, c = (positive[name] for name in ("baseline", "mass_only", "arm_only"))
                require(abs(b["mean_force"][2]/a["mean_force"][2]-2) < .02 and abs(b["mean_torque"][1]/a["mean_torque"][1]-2) < .02,
                    "Mass-only change did not double actual force and torque")
                require(abs(c["mean_force"][2]/a["mean_force"][2]-1) < .01 and abs(c["mean_torque"][1]/a["mean_torque"][1]-5/3) < .02,
                    "Lever-only change did not preserve force and scale moment")
            report["passed"] = True
        except Exception as error:report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L19 final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error:report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":sys.exit(main())
