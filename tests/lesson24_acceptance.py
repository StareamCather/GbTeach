#!/usr/bin/env python3
"""Check actual JointState -> TF data, clock behavior and persistent reset recovery."""
import argparse
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
from tests.lesson22_acceptance import rpc
from tests.lesson23_acceptance import Probe as ServiceProbe
from tests.lesson23_acceptance import (axis_rotation, flat_pose, hashes, matrix_from_pose,
                                     multiply, pose_error, ros_value, run, same_snapshot,
                                     save, sha, transform)

JOINTS = ("lesson24_shoulder", "lesson24_elbow")
LINKS = ("base_link", "arm1", "arm2", "tool_link")
RSP = "/lesson24/robot_state_publisher"


def require(value, message):
    if not value:
        raise AssertionError(message)


def frame(prefix, name):
    return prefix+"/"+name if prefix else name


def relative_transforms(q):
    return {("world", "base_link"): transform((0, 0, .7), (0, 0, .2)),
            ("base_link", "arm1"): multiply(transform((0, 0, .08)), axis_rotation((0, 0, 1), q[0])),
            ("arm1", "arm2"): multiply(transform((.6, 0, 0), (0, 0, .15)), axis_rotation((0, 0, 1), q[1])),
            ("arm2", "tool_link"): transform((.4, 0, 0))}


def expected_poses(q):
    relative = relative_transforms(q)
    base = relative[("world", "base_link")]
    arm1 = multiply(base, relative[("base_link", "arm1")])
    arm2 = multiply(arm1, relative[("arm1", "arm2")])
    return {"base_link": base, "arm1": arm1, "arm2": arm2,
            "tool_link": multiply(arm2, relative[("arm2", "tool_link")])}


def tf_matrix(value):
    return matrix_from_pose(flat_pose({"position": value["translation"], "orientation": value["rotation"]}))


def joint_angles(row):
    data = row["data"]
    return [data["position"][data["name"].index(name)] for name in JOINTS]


def inspect_rows(rows, prefix, required_tf_ids=None):
    """Check original data; repeated time labels after reset remain separate rows."""
    require(rows, "Missing original ROS graph events")
    require(all(b["event_id"] > a["event_id"] and b["receive_monotonic_ns"] > a["receive_monotonic_ns"]
                for a, b in zip(rows, rows[1:])), "Original callback arrival order was changed")
    joints_by_stamp = {}
    for row in rows:
        source = row["source"]
        require(source in ("clock", "joint_state", "tf", "tf_static"), "Unknown original graph message source")
        require(row["callerid"] == ("/gazebo" if source in ("clock", "joint_state") else RSP), "Unexpected actual ROS publisher identity")
        require(type(row["ros_time_at_receive_ns"]) is int and row["ros_time_at_receive_ns"] >= 0, "Invalid ROS-time-at-receipt readback")
        if source == "clock":
            require(row["stamp_ns"] == row["data"]["clock_ns"], "Raw Clock value was rewritten")
        elif source == "joint_state":
            data = row["data"]
            require(set(data["name"]) == set(JOINTS) and len(data["name"]) == len(data["position"]) == len(data["velocity"]) == 2,
                    "Actual JointState names/axis0 position/velocity arrays differ")
            require(data["header_frame_id"] == "" and data["effort"] == [], "Stock plugin header/absent effort data was rewritten")
            require(all(math.isfinite(v) for field in ("position", "velocity") for v in data[field]), "Nonfinite actual moving-joint state")
            joints_by_stamp.setdefault(row["stamp_ns"], []).append(row)
        else:
            require(row["stamp_ns"] is None, "TFMessage was assigned a nonexistent common header timestamp")
    matches, static_count, unverified = [], 0, []
    for row in rows:
        if row["source"] not in ("tf", "tf_static"):
            continue
        transforms = row["data"]["transforms"]
        moving = row["source"] == "tf"
        edges = (("base_link", "arm1"), ("arm1", "arm2")) if moving else (("world", "base_link"), ("arm2", "tool_link"))
        require({(item["parent"], item["child"]) for item in transforms} == {(frame(prefix, a), frame(prefix, b)) for a, b in edges}
                and len(transforms) == 2, "Actual TF topology/prefix differs from the intended tree")
        if moving:
            stamps = {item["stamp_ns"] for item in transforms}
            require(len(stamps) == 1, "A stock RSP dynamic message contains inconsistent source stamps")
            candidates = joints_by_stamp.get(next(iter(stamps)), [])
        else:
            candidates = [None]
        fitting = []
        for candidate in candidates:
            expected = relative_transforms(joint_angles(candidate) if candidate else (0, 0))
            errors = []
            for value in transforms:
                edge = next(edge for edge in edges if frame(prefix, edge[1]) == value["child"])
                error, angle = pose_error(tf_matrix(value), expected[edge])
                errors.append((error, angle))
            if all(error < 1e-8 and angle < 1e-6 for error, angle in errors):
                fitting.append(candidate["event_id"] if candidate else None)
        if not fitting and moving and required_tf_ids is not None and row["event_id"] not in required_tf_ids:
            unverified.append(row["event_id"])
            continue
        require(fitting, "Actual TF values lack a matching original source JointState stamp/angles/URDF transform")
        if moving:
            matches.append({"tf_event_id": row["event_id"], "source_joint_state_event_ids": fitting,
                            "unchanged_header_stamp_ns": transforms[0]["stamp_ns"]})
        else:
            static_count += 1
    return {"event_count": len(rows), "dynamic_matches": matches, "static_message_count": static_count,
            "unverified_boundary_tf_event_ids": unverified,
            "time_basis": "JointState header is cached ros::Time::now(), not a native joint acquisition stamp; arrival clocks are distinct."}


class Probe(ServiceProbe):
    def __init__(self):
        import rospy
        rospy.init_node("independent_l24_probe", anonymous=True, disable_signals=True)
        self.rospy = rospy

    def snapshot(self):
        from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelProperties
        from gazebo_msgs.srv import GetModelState, GetPhysicsProperties, GetWorldProperties
        host = time.monotonic_ns()
        before = self.call("/gazebo/get_world_properties", GetWorldProperties)
        model = self.call("/gazebo/get_model_properties", GetModelProperties, model_name="tf_arm")
        state = self.call("/gazebo/get_model_state", GetModelState, model_name="tf_arm", relative_entity_name="world")
        links = {name: ros_value(self.call("/gazebo/get_link_state", GetLinkState, link_name="tf_arm::"+name, reference_frame="world")) for name in LINKS}
        joints = {name: ros_value(self.call("/gazebo/get_joint_properties", GetJointProperties, joint_name="tf_arm::"+name)) for name in JOINTS}
        physics = self.call("/gazebo/get_physics_properties", GetPhysicsProperties)
        after = self.call("/gazebo/get_world_properties", GetWorldProperties)
        return {"host_monotonic_before_ns": host, "host_monotonic_after_ns": time.monotonic_ns(),
                "world_time_before_ns": round(before.sim_time*1e9), "world_time_after_ns": round(after.sim_time*1e9),
                "model_properties": ros_value(model), "model_state": ros_value(state), "link_states": links,
                "joint_properties": joints, "physics": ros_value(physics)}


def worker(operation, output):
    result = {"passed": False, "operation": operation}
    try:
        probe = Probe()
        if operation == "seed":
            from gazebo_msgs.srv import GetLinkState, SetLinkState, SetModelConfiguration
            require(probe.snapshot()["physics"]["pause"], "Entry-state fixture requires a paused private world")
            probe.call("/gazebo/set_model_configuration", SetModelConfiguration, model_name="tf_arm", urdf_param_name="",
                       joint_names=list(JOINTS), joint_positions=[.1, -.2])
            state = probe.call("/gazebo/get_link_state", GetLinkState, link_name="tf_arm::arm2", reference_frame="world").link_state
            state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = .003, -.002, .001
            state.twist.angular.z = .004
            probe.call("/gazebo/set_link_state", SetLinkState, link_state=state)
            actual = probe.call("/gazebo/get_link_state", GetLinkState, link_name="tf_arm::arm2", reference_frame="world")
            result["seed_actual_readback"] = ros_value(actual)
            require(all(math.isfinite(getattr(vector, axis)) for vector in (actual.link_state.twist.linear, actual.link_state.twist.angular) for axis in ("x", "y", "z"))
                    and max(abs(getattr(actual.link_state.twist.linear, axis)) for axis in ("x", "y", "z")) > 1e-6
                    and abs(actual.link_state.twist.angular.z) > 1e-6, "Nonzero entry twist was not actually read back")
        result["snapshot"] = probe.snapshot()
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    finally:
        save(output, result)
    return 0 if result["passed"] else 1


def probe_result(session, artifacts, name, operation="snapshot"):
    path = artifacts/(name+".json")
    status = run(session, [sys.executable, str(Path(__file__).resolve()), "--worker", operation, "--report", str(path)], artifacts/(name+".log"), timeout=40)
    result = json.loads(path.read_text())
    require(status == 0 and result["passed"], "Independent physical/TF probe failed: "+str(result.get("error")))
    return result


def original_events(values, by_id):
    require(all(value["event_id"] in by_id and by_id[value["event_id"]] == value for value in values), "Phase changed/invented an original graph event")
    require(all(b["event_id"] > a["event_id"] for a, b in zip(values, values[1:])), "Phase reordered callback arrivals")
    return values


def check_lookup(value, parent, child, expected, tolerance=2e-5):
    require(value["parent"] == parent and value["child"] == child, "Lookup resolved an unintended TF frame")
    position, angle = pose_error(matrix_from_pose(flat_pose(value)), expected)
    require(position < tolerance and angle < tolerance, "Actual TF lookup disagrees with independent robot kinematics")
    return {"position_error_m": position, "rotation_error_rad": angle}


def inspect_report(result, rate, prefix):
    trace = result["trace"]
    by_id = {row["event_id"]: row for row in trace}
    require(len(by_id) == len(trace), "Original event IDs were duplicated")
    stream = result["stream"]
    rows = [by_id[number] for number in stream["event_ids"]]
    original_events(rows, by_id)
    require(all(row["event_id"] > stream["epoch_event_id"] for row in rows), "Stream contains pre-epoch callbacks")
    joint_ids = [row["event_id"] for row in rows if row["source"] == "joint_state"]
    require(stream["joint_state_event_ids"] == joint_ids, "Stream JointState index differs from original rows")
    joints = [by_id[number] for number in joint_ids]
    stamps = [row["stamp_ns"] for row in joints]
    require(len(stamps) >= rate and all(b > a for a, b in zip(stamps, stamps[1:])), "Normal stream has too few/nonmonotonic ROS header labels")
    span = (stamps[-1]-stamps[0])/1e9
    frequency = (len(stamps)-1)/span
    require(span >= 1 and abs(frequency/rate-1) <= .05, "Observed JointState header-label cadence differs from configured update rate")
    stats = stream["statistics"]
    require(stats["count"] == len(stamps) and abs(stats["span_s"]-span) < 1e-9 and abs(stats["rate_hz"]-frequency) < 1e-9,
            "Reported cadence does not match actual original JointState headers")
    tf_ids = [row["event_id"] for row in rows if row["source"] == "tf"
              and all(stamps[0] <= item["stamp_ns"] <= stamps[-1] for item in row["data"]["transforms"])]
    require(stream["tf_event_ids"] == tf_ids and tf_ids, "Closed source-label window omits/invents an eligible TF packet")
    checked = {"stream": {"header_label_count": len(stamps), "header_span_s": span, "observed_header_rate_hz": frequency},
               "raw_graph": inspect_rows(trace, prefix, set(tf_ids)), "poses": {}}
    expected_names = {"zero", "positive_mixed", "negative_mixed"}
    require({phase["name"] for phase in result["pose_phases"]} == expected_names, "Missing paused pose/TF experiment")
    for phase in result["pose_phases"]:
        snapshot = phase["snapshot"]
        require(snapshot["paused"] is True, "Pose/TF agreement was claimed across an advancing world")
        q = [snapshot["joint_readback"][name]["position"][0] for name in JOINTS]
        require(max(abs(a-b) for a, b in zip(q, phase["requested_q"])) < 1e-4, "Actual joint state differs from the paused initialization request")
        expected = expected_poses(q)
        comparisons = {}
        for name in LINKS:
            actual = snapshot["link_readback"][name]
            require(actual["world_time_before_ns"] == actual["world_time_after_ns"] == snapshot["world_time_ns"], "Paused geometry service queries used different simulation times")
            error, angle = pose_error(matrix_from_pose(flat_pose(actual["link_state"])), expected[name])
            require(error < 2e-5 and angle < 2e-5, "Gazebo link state disagrees with independent FK")
            comparisons[name] = check_lookup(phase["lookups"][name], frame(prefix, "world"), frame(prefix, name), expected[name])
        latest = phase["latest_joint_state"]
        original_events([latest], by_id)
        require(max(abs(a-b) for a, b in zip(joint_angles(latest), q)) < 1e-5, "Paused JointState angles differ from actual Gazebo joint readback")
        relatives = relative_transforms(q)
        relative = multiply(multiply(relatives[("base_link", "arm1")], relatives[("arm1", "arm2")]), relatives[("arm2", "tool_link")])
        check_lookup(phase["relative_lookup"], frame(prefix, "base_link"), frame(prefix, "tool_link"), relative)
        actual = phase["relative_actual"]
        state = actual["link_state"]
        error, angle = pose_error(matrix_from_pose(flat_pose(state)), relative)
        require(error < 2e-5 and angle < 2e-5, "Actual scoped-base relative link state disagrees with TF/FK")
        checked["poses"][phase["name"]] = comparisons
    pause = result["pause_phase"]
    a, b = pause["before"], pause["after"]
    require(a["world_time_ns"] == b["world_time_ns"] and a["ros_time_ns"] == b["ros_time_ns"]
            and a["clock_event"]["stamp_ns"] == b["clock_event"]["stamp_ns"], "Pause failed to freeze world and ROS simulation time")
    require(b["host_monotonic_ns"]-a["host_monotonic_ns"] >= 150000000, "Pause hold lacks a measured host-time interval")
    held = original_events(pause["events"], by_id)
    require(not [row for row in held if row["source"] in ("joint_state", "tf")], "RSP/JointState continued publishing without simulation updates")
    steps = {}
    for phase in result["step_phases"]:
        count = phase["steps"]
        require(count in (1, 100) and phase["after"]["world_time_ns"]-phase["before"]["world_time_ns"] == count*1000000,
                "Actual world single/multiple stepping differs from 1ms ticks")
        require(phase["command"]["argv"] == ["gz", "world", "-w", "lesson24", "-m", str(count)]
                and phase["command"]["steps"] == count, "Step evidence does not preserve the actual native command")
        events = original_events(phase["events"], by_id)
        js_count = sum(row["source"] == "joint_state" for row in events)
        require(js_count <= math.ceil(count*rate/1000)+1, "JointState samples ignored their simulation-time publication gate")
        if count == 100:
            require(js_count >= max(1, math.floor(count*rate/1000)-1), "Multiple steps failed to generate gated actual joint messages")
        steps[count] = {"actual_simulation_delta_ns": phase["after"]["world_time_ns"]-phase["before"]["world_time_ns"], "joint_state_count": js_count}
    require(set(steps) == {1, 100}, "Missing true single/multiple-step comparison")
    checked["steps"] = steps
    late = result["late_static_phase"]
    require(late["latching"] == "1" and late["callerid"] == RSP
            and late["received_monotonic_ns"] >= late["subscribed_monotonic_ns"], "Late static subscriber did not receive an actual latched RSP message")
    original_events([late["row"]], by_id)
    require(late["row"]["source"] == "tf_static" and late["row"]["latching"] == "1", "Late static evidence is not its original latched connection message")
    reset = result["reset_phase"]
    require(reset["buffer_identity_before"] == reset["buffer_identity_after"]
            and reset["listener_identity_before"] == reset["listener_identity_after"], "Reset recovery replaced its listener/buffer")
    require(reset["after"]["world_time_ns"] < reset["before"]["world_time_ns"]
            and reset["after"]["clock_event"]["stamp_ns"] < reset["before"]["clock_event"]["stamp_ns"], "Real reset did not move simulation clock backward")
    delta = reset["clear_count_after"]-reset["clear_count_before"]
    require(delta > 0 and len(reset["clear_events"]) >= delta, "Persistent stock listener did not actually clear its dynamic cache")
    require(any(event["ros_time_ns"] < reset["before"]["ros_time_ns"] for event in reset["clear_events"]), "Cache clear lacks an actual ROS-time regression")
    actual_clears = reset["clear_events"][reset["clear_count_before"]:reset["clear_count_after"]]
    require(len(actual_clears) == delta and all(reset["before"]["host_monotonic_ns"] <= item["receive_monotonic_ns"] <= reset["after"]["host_monotonic_ns"] for item in actual_clears), "Audited clear events are outside the actual reset interval")
    reset_rows = original_events(reset["events"], by_id)
    clocks = [reset["before"]["clock_event"]]+[row for row in reset_rows if row["source"] == "clock"]
    require(any(b["stamp_ns"] < a["stamp_ns"] for a, b in zip(clocks, clocks[1:])), "Original clock messages do not show the backward jump")
    post = reset["post_ready"]
    require(post["listener_identity"] == reset["listener_identity_before"] and post["buffer_identity"] == reset["buffer_identity_before"], "Ready check used a replacement listener")
    for source in ("clock", "joint_state", "tf"):
        value = post["latest_events"][source]
        original_events([value], by_id)
        require(value["event_id"] in {row["event_id"] for row in reset_rows}, "Reset reused a pre-reset latest message as recovery evidence")
    require(post["latest_events"]["joint_state"]["stamp_ns"] < reset["before"]["clock_event"]["stamp_ns"], "Recovered stream still refers to the old timeline")
    post_stamps = {item["stamp_ns"] for row in reset_rows if row["source"] == "tf" for item in row["data"]["transforms"]}
    expected = expected_poses(joint_angles(post["latest_events"]["joint_state"]))
    for name in LINKS:
        lookup = post["lookups"][name]
        check_lookup(lookup, frame(prefix, "world"), frame(prefix, name), expected[name])
        if name != "base_link":
            require(lookup["stamp_ns"] in post_stamps and lookup["stamp_ns"] < reset["before"]["clock_event"]["stamp_ns"], "Post-reset lookup reused old dynamic cache timestamps")
    checked["reset"] = {"actual_clear_count_delta": delta, "old_world_time_ns": reset["before"]["world_time_ns"], "new_world_time_ns": reset["after"]["world_time_ns"]}
    fault = result["wrong_frame_probe"]
    require(fault["queried_frame"] == "lesson24/base_link" and "LookupException" in fault["error"], "Wrong-frame probe lacks the real namespace-as-frame lookup failure")
    restore = result["restore"]
    require(restore["passed"] is True and not restore["errors"] and restore["before"]["paused"] == restore["pause_after"], "Course state restoration failed")
    return checked


def input_hashes(workspace):
    result = hashes(workspace)
    for name in ("build/arm.urdf", "build/arm.sdf", "build/arm_lab.json"):
        result[name] = sha(workspace/name)
    return result


def inspect_inputs(session, rate, prefix):
    workspace = session.workspace
    record = json.loads((workspace/"build/arm_lab.json").read_text())
    require(record["config"]["joint_state_rate_hz"] == rate and record["config"]["tf_prefix"] == prefix, "Prepared input configuration differs")
    for name, key in (("arm_settings.json", "settings_sha256"), ("world.sdf", "world_sha256"),
                      ("build/arm.urdf", "generated_urdf_sha256"), ("build/arm.sdf", "converted_sdf_sha256")):
        require(sha(workspace/name) == record[key], "Prepared input/generated SHA mismatch: "+name)
    require(all(sha(workspace/name) == digest for name, digest in record["source_sha256"].items()), "Running package source fingerprints differ")
    plugin = ET.parse(str(workspace/"build/arm.urdf")).getroot().find("gazebo/plugin[@filename='libgazebo_ros_joint_state_publisher.so']")
    require(plugin is not None and float(plugin.findtext("updateRate")) == rate
            and plugin.findtext("robotNamespace") == "/lesson24"
            and [name.strip() for name in plugin.findtext("jointName").split(",")] == list(JOINTS), "Generated actual stock joint publisher configuration differs")
    require(rpc(session.env["ROS_MASTER_URI"], "getParam", "/use_sim_time") is True, "Private Master is not configured for simulation time")
    for key, value in (("tf_prefix", prefix), ("use_tf_static", True), ("ignore_timestamp", False), ("publish_frequency", 50.0)):
        require(rpc(session.env["ROS_MASTER_URI"], "getParam", RSP+"/"+key) == value, "Actual RSP parameter differs: "+key)
    return record


def verify_case(session, artifacts, name, rate, prefix, expected=True, diagnosis=None):
    before = probe_result(session, artifacts, name+"-entry")["snapshot"]
    fingerprints = input_hashes(session.workspace)
    for relative in ("arm_settings.json", "world.sdf", "build/arm.urdf", "build/arm.sdf", "build/arm_lab.json"):
        shutil.copy2(str(session.workspace/relative), str(artifacts/(name+"-input-"+Path(relative).name)))
    output = artifacts/(name+".json")
    status = run(session, [sys.executable, str(session.workspace/"verify.py"), "--report", str(output)], artifacts/(name+".log"), timeout=100)
    result = json.loads(output.read_text())
    require((status == 0) is expected and result["passed"] is expected, name+": unexpected course result: "+str(result.get("error") or result.get("restore_error")))
    if diagnosis:
        require(diagnosis.lower() in result.get("error", "").lower(), "Invalid configuration was rejected for another reason")
    after = probe_result(session, artifacts, name+"-exit")["snapshot"]
    same_snapshot(before, after)
    require(input_hashes(session.workspace) == fingerprints, "Verifier changed exercise inputs/generated files")
    if expected:
        inspect_inputs(session, rate, prefix)
        checked = inspect_report(result, rate, prefix)
    else:
        require(not result.get("stream") and not result["pose_phases"] and not result["step_phases"] and not result.get("reset_phase"), "Preflight fault reached clock/robot manipulation")
        if before["physics"]["pause"]:
            require(before["world_time_after_ns"] == after["world_time_before_ns"], "Preflight fault advanced/reset the paused simulation clock")
        checked = {"diagnosis": result["error"]}
    print(json.dumps({"scenario": name, "passed": True, "raw_report": str(output)}), flush=True)
    return {"name": name, "passed": True, "raw_report": str(output), "input_sha256": fingerprints, "checked": checked}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--worker", choices=("snapshot", "seed"))
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker, args.report)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == "classic11-noetic:24"]
    require(len(found) == 1, "Expected L24: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": [], "graphical_checks_performed": False}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l24-") as temporary:
        try:
            for name, rate, prefix, paused in (("baseline", 20, "", True), ("rate_only", 40, "", False), ("prefix_only", 20, "demo", True)):
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                if name != "baseline":
                    save(workspace/"arm_settings.json", {"joint_state_rate_hz": rate, "tf_prefix": prefix})
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                if name == "prefix_only":
                    probe_result(session, artifacts, "nonzero_entry_state", "seed")
                report["scenarios"].append(verify_case(session, artifacts, name, rate, prefix))
                if name == "prefix_only":
                    settings = workspace/"arm_settings.json"
                    original = settings.read_bytes()
                    try:
                        save(settings, {"joint_state_rate_hz": 40, "tf_prefix": prefix})
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", rate, prefix, False, "reload"))
                    finally:
                        settings.write_bytes(original)
                    parameter = RSP+"/tf_prefix"
                    rpc(session.env["ROS_MASTER_URI"], "setParam", parameter, "wrong_prefix")
                    try:
                        require(rpc(session.env["ROS_MASTER_URI"], "getParam", parameter) == "wrong_prefix", "Live parameter fault was not read back")
                        report["scenarios"].append(verify_case(session, artifacts, "live_rsp_parameter_mismatch", rate, prefix, False, "RSP tf_prefix"))
                    finally:
                        rpc(session.env["ROS_MASTER_URI"], "setParam", parameter, prefix)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_inputs_and_parameter_repaired", rate, prefix))
                session.stop(reason="L24 independent case completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L24 final independent cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(name+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(name+"-session.json")))
                    for index, path in enumerate(sorted((session.session_dir/"ros-home/log").rglob("*"))):
                        if path.is_file():
                            shutil.copy2(str(path), str(artifacts/(name+"-ros-"+str(index)+"-"+path.name)))
                except Exception as error:
                    report["cleanup_errors"].append(name+": "+repr(error))
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
