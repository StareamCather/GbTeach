#!/usr/bin/env python3
"""Independently check actual five-sensor payloads, geometry and inertial motion."""
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
from tests.lesson05_acceptance import actual_pose
from tests.lesson06_acceptance import actual_stats

SOURCES = {"gps": "GPS", "mag": "Magnetometer", "alt": "Altimeter",
           "sonar": "SonarStamped", "logical": "LogicalCameraImage"}
KINDS = {"gps": "gps", "mag": "magnetometer", "alt": "altimeter",
         "sonar": "sonar", "logical": "logical_camera"}
PHASES = ("baseline", "nav_up", "nav_east_north", "nav_yaw90", "nav_velocity",
          "sonar_obstacle", "sonar_out_of_range", "logical_outside", "logical_restored")


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def configure(lesson, workspace, field_scale=None, initial_height=None):
    spec = importlib.util.spec_from_file_location("l20_independent_config", lesson.folder/"config.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    previous = Path.cwd()
    try:
        os.chdir(str(workspace))
        module.edit(field_scale=field_scale, initial_height=initial_height)
    finally:
        os.chdir(str(previous))


def vector(value):
    return [float(value[axis]) for axis in ("x", "y", "z")]


def local_from_geodetic(gps):
    # Independent WGS84 ECEF -> ENU about lat/lon0, ellipsoidal height100m.
    # At this explicit Classic heading180, worldX/Y/Z are east/north/up.
    latitude, longitude = (math.radians(gps[key]) for key in ("latitude_deg", "longitude_deg"))
    a, flattening = 6378137., 1/298.257223563
    eccentricity_squared = flattening*(2-flattening)
    radius = a/math.sqrt(1-eccentricity_squared*math.sin(latitude)**2)
    x = (radius+gps["altitude"])*math.cos(latitude)*math.cos(longitude)
    y = (radius+gps["altitude"])*math.cos(latitude)*math.sin(longitude)
    z = (radius*(1-eccentricity_squared)+gps["altitude"])*math.sin(latitude)
    return [y, z, x-(a+100)]


def finite_payload(value):
    if isinstance(value, dict):
        return all(finite_payload(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_payload(item) for item in value)
    if isinstance(value, (float, int)):
        return math.isfinite(value)
    return value not in ("NaN", "Infinity", "-Infinity")


def inspect_raw(observation, scale, height):
    require(set(observation["phases"]) == set(PHASES), "Missing physical sensor phase")
    summary = {}
    for phase in PHASES:
        phase_report = observation["phases"][phase]
        channels, per_source = phase_report["channels"], {}
        epoch = phase_report["control_epoch_ns"]
        setups = phase_report["setup_truth"]
        expected_positions = {"sonar": [-4, 0, 4 if phase == "sonar_out_of_range" else 1.5],
                              "obstacle": [-4, 0 if phase == "sonar_obstacle" else 2, .3],
                              "target": [-2 if phase == "logical_outside" else 2, -4, 1]}
        for station, position in expected_positions.items():
            require(max(abs(a-b) for a, b in zip(setups[station]["state"]["position"], position)) < 1e-5,
                    "Actual fixture initialization differs from controlled geometry: "+station)
        for source, native_type in SOURCES.items():
            rows = [r for r in observation["trace"] if r["phase"] == phase and r["source"] == source
                    and r["disposition"] == "accepted"]
            require(len(rows) >= 12, "Insufficient original payloads: "+phase+"/"+source)
            sequences = [r["bridge_sequence"] for r in rows]
            require(all(b > a for a, b in zip(sequences, sequences[1:])), "Repeated native bridge events")
            times = [r["receipt_world_time_ns"] if source == "logical" else r["source_stamp_ns"] for r in rows]
            ordered = all(b >= a for a, b in zip(times, times[1:])) if source == "logical" else all(b > a for a, b in zip(times, times[1:]))
            require(ordered, "Native sample time or bridge receipt clock moved backward/repeated")
            span = (times[-1]-times[0])/1e9
            require(span >= 1 and 8 <= (len(rows)-1)/span <= 12, "Implausible actual native/receipt cadence")
            require(times[0] >= epoch+round(phase_report["guard_s"]*1e9), "Accepted data predates the phase guard")
            for row in rows:
                wire = json.loads(row["wire_json"])
                native = row["native"]
                require(wire["native"] == native, "Inspection rewrote original protobuf payload")
                require(finite_payload(native), "Native payload contains invalid numeric values")
                require(int(wire["bridge_sequence"]) == row["bridge_sequence"]
                        and int(wire["receipt_world_time_ns"]) == row["receipt_world_time_ns"]
                        and wire["native_topic"] == row["native_topic"]
                        and wire["native_type"] == row["native_type"] and wire["source_kind"] == KINDS[source],
                        "Normalized event metadata differs from the original wire envelope")
                require(row["native_type"] == "gazebo.msgs."+native_type, "Wrong actual native sensor class")
                require(row["native_topic"].startswith("/gazebo/lesson20/"), "Native routing is not resolved")
                if source == "logical":
                    require(wire["source_stamp_ns"] is None and row["source_stamp_ns"] is None and "time" not in native,
                            "LogicalCameraImage was assigned a fabricated native sample stamp")
                    require(row["time_basis"] == "bridge_receipt_world", "Logical arrival rate mislabeled as native sampling")
                else:
                    require(row["time_basis"] == "native", "Native sample rate mislabeled as arrival time")
                    stamp = native["time"]["sec"]*1000000000+native["time"]["nsec"]
                    require(stamp == row["source_stamp_ns"] == int(wire["source_stamp_ns"]), "Original native time changed")
                if source == "gps":
                    local = local_from_geodetic(native)
                    x, y = (10, 5) if phase == "nav_east_north" else (0, 0)
                    require(abs(local[0]-x) < .02 and abs(local[1]-y) < .02, "GPS geodetic position does not recover actual local XY")
                    if phase != "nav_velocity":
                        target_z = height+1 if phase == "nav_up" else height
                        require(abs(local[2]-target_z) < .003, "GPS ellipsoidal altitude differs from actual height")
                    else:
                        require(abs(local[2]-(height+.25*(row["source_stamp_ns"]-epoch)/1e9)) < .003,
                                "GPS absolute height does not follow actual initialized inertial trajectory")
                    velocity = .25 if phase == "nav_velocity" else 0
                    require(abs(native["velocity_up"]-velocity) < .005 and abs(native["velocity_east"]) < .005
                            and abs(native["velocity_north"]) < .005, "Actual GPS velocity is inconsistent with motion")
                elif source == "mag":
                    expected = [0, -2e-5*scale, -4e-5*scale] if phase == "nav_yaw90" else [2e-5*scale, 0, -4e-5*scale]
                    require(max(abs(a-b) for a, b in zip(vector(native["field_tesla"]), expected)) < 1e-9,
                            "Native body-frame magnetic field violates inverse-yaw transformation")
                elif source == "alt":
                    require(abs(native["vertical_reference"]-height) < 1e-9, "Altimeter initial reference was reset during teleport")
                    if phase != "nav_velocity":
                        require(abs(native["vertical_position"]-(1 if phase == "nav_up" else 0)) < .003,
                                "Altimeter relative height differs from the initial reference")
                    else:
                        require(abs(native["vertical_position"]-.25*(row["source_stamp_ns"]-epoch)/1e9) < .003,
                                "Altimeter absolute displacement does not follow inertial trajectory")
                    require(abs(native["vertical_velocity"]-(.25 if phase == "nav_velocity" else 0)) < .005,
                            "Altimeter velocity does not observe inertial vertical movement")
                elif source == "sonar":
                    sonar = native["sonar"]
                    require(sonar["geometry"] == "cone" and sonar["range_min"] == 0 and sonar["range_max"] == 3
                            and abs(sonar["radius"]-.2) < 1e-9, "Wrong actual sonar contact volume")
                    expected = 3 if phase == "sonar_out_of_range" else (.9 if phase == "sonar_obstacle" else 1.5)
                    require(abs(sonar["range"]-expected) < (1e-6 if expected == 3 else .06),
                            "Sonar did not react to floor, elevated obstacle or out-of-range geometry")
                    require(abs(vector(sonar["world_pose"]["position"])[2]-(4 if expected == 3 else 1.5)) < 1e-5,
                            "Reported sonar pose does not support its range")
                else:
                    require(max(abs(a-b) for a, b in zip(vector(native["pose"]["position"]), [0, -4, 1])) < 1e-5,
                            "Logical image pose is not the camera world pose")
                    models = {item["name"]: item for item in native.get("model", [])}
                    require("occluder" in models, "Actual foreground occluder disappeared")
                    require(("tracked_target" in models) is (phase != "logical_outside"), "Logical frustum failed actual leave/return behavior")
                    if "tracked_target" in models:
                        require(max(abs(a-b) for a, b in zip(vector(models["tracked_target"]["pose"]["position"]), [2, 0, 0])) < 1e-5,
                                "Logical model pose is not sensor-relative")
            if phase == "nav_velocity" and source in ("gps", "alt"):
                positions = [local_from_geodetic(r["native"])[2] if source == "gps" else r["native"]["vertical_position"] for r in rows]
                require(abs((positions[-1]-positions[0])/span-.25) < .005,
                        "Native position change does not support the reported vertical velocity")
            require(channels[source]["message_count"] == len(rows), "Summary count differs from original payload count")
            per_source[source] = {"messages": len(rows), "span_s": span, "observed_rate_hz": (len(rows)-1)/span,
                                  "time_basis": rows[0]["time_basis"], "last_native": rows[-1]["native"]}
        truth = observation["phases"][phase]["truth"]
        require(len(truth) >= 2, "Missing independent actual model-state observations")
        for row in truth:
            x, y = (10, 5) if phase == "nav_east_north" else (0, 0)
            z = height+1 if phase == "nav_up" else height
            elapsed = ((row["clock_before_ns"]+row["clock_after_ns"])/2-epoch)/1e9
            if phase == "nav_velocity": z += .25*elapsed
            state = row["state"]
            require(finite_payload(state), "Actual physical state is not finite")
            require(max(abs(a-b) for a, b in zip(state["position"], [x, y, z])) < .005,
                    "Actual physical position differs from phase initialization")
            require(max(abs(a-b) for a, b in zip(state["linear"], [0, 0, .25 if phase == "nav_velocity" else 0])) < .005
                    and max(abs(a) for a in state["angular"]) < .005, "Actual physical twist differs from the phase")
            angle = math.pi/2 if phase == "nav_yaw90" else 0
            target_q = [0, 0, math.sin(angle/2), math.cos(angle/2)]
            norm = math.sqrt(sum(a*a for a in state["orientation"]))
            dot = abs(sum(a*b for a, b in zip(state["orientation"], target_q)))/norm
            require(2*math.acos(min(1, max(0, dot))) < .005, "Actual model orientation differs from controlled yaw")
        if phase == "nav_velocity":
            for row in truth:
                require(abs(row["state"]["linear"][2]-.25) < .005, "Actual physical body is not moving upward at .25m/s")
            require(truth[-1]["state"]["position"][2] > truth[0]["state"]["position"][2]+.2,
                    "The model did not actually move while GPS and altimeter reported velocity")
        summary[phase] = per_source
    return summary


def verify_case(session, artifacts, label, scale, height, expected=True, diagnosis=None):
    before = actual_stats(session)
    poses_before = {model: actual_pose(session, model) for model in ("nav_rig", "sonar_rig", "sonar_obstacle", "tracked_target")}
    world, path = session.workspace/"world.sdf", artifacts/(label+".json")
    original = world.read_bytes()
    completed = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                               cwd=str(session.workspace), env=session.env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=100)
    path.with_suffix(".stdout.log").write_text(completed.stdout, encoding="utf-8")
    require((completed.returncode == 0) is expected, "Unexpected verifier result "+label+": "+completed.stdout[-3000:])
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "Process/JSON result mismatch")
    if diagnosis:
        require(diagnosis.lower() in (completed.stdout+json.dumps(observation)).lower(), "Missing diagnosis: "+diagnosis)
    checked = inspect_raw(observation, scale, height) if expected else None
    after = actual_stats(session)
    poses_after = {model: actual_pose(session, model) for model in poses_before}
    require(before["paused"] == after["paused"], "Verification did not restore actual pause state")
    for model in poses_before:
        require(max(abs(a-b) for a, b in zip(poses_before[model], poses_after[model])) < 1e-5, "Verification did not restore "+model)
    require(world.read_bytes() == original, "Verification rewrote learner world")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Invalid input advanced actual physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected, "raw_report": str(path),
            "independent_raw_checks": checked, "stats_before": before, "stats_after": after,
            "poses_before": poses_before, "poses_after": poses_after,
            "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [l for l in lessons if l.key == "classic11-noetic:20"]
    require(len(found) == 1, "Expected L20: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": []}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l20-") as temporary:
        try:
            for label, scale, height, paused in (("baseline", 1, 1, True), ("magnetic_only", 2, 1, False), ("initial_height_only", 1, 2, True)):
                storage = Storage(Path(temporary)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline": configure(lesson, workspace, scale, height)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label, scale, height))
                if label == "initial_height_only":
                    path, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        configure(lesson, workspace, field_scale=2)
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", scale, height, False, "reload"))
                    finally: path.write_bytes(saved)
                    for bad, xpath, value, diagnosis in (
                        ("wrong_spherical_heading", "./world/spherical_coordinates/heading_deg", "0", "heading"),
                        ("wrong_gps_topic", "./world/model[@name='nav_rig']/link/sensor[@name='gps']/topic", "wrong", "topic"),
                        ("wrong_sonar_geometry", "./world/model[@name='sonar_rig']/link/sensor/sonar/geometry", "sphere", "sonar")):
                        try:
                            tree = ET.fromstring(saved)
                            node = tree.find(xpath)
                            require(node is not None, "Missing fault field: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, bad, scale, height, False, diagnosis))
                        finally: path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired", scale, height))
                session.stop(reason="L20 physical scenario complete")
            positive = {c["name"]: c["independent_raw_checks"]["baseline"] for c in report["scenarios"] if c["expected_acceptance"]}
            a, b, c = (positive[name] for name in ("baseline", "magnetic_only", "initial_height_only"))
            for axis in ("x", "z"):
                require(abs(b["mag"]["last_native"]["field_tesla"][axis]/a["mag"]["last_native"]["field_tesla"][axis]-2) < 1e-7,
                        "Actual magnetic-only setting did not double the field")
            require(abs(c["gps"]["last_native"]["altitude"]-a["gps"]["last_native"]["altitude"]-1) < .002,
                    "GPS absolute height did not follow changed initial rig height")
            require(abs(c["alt"]["last_native"]["vertical_position"]) < .002 and c["alt"]["last_native"]["vertical_reference"] == 2,
                    "Reloaded altimeter reference does not follow initial height")
            report["passed"] = True
        except Exception as error: report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L20 final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error: report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__": sys.exit(main())
