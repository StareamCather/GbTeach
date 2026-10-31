#!/usr/bin/env python3
"""Independent ROS graph, finite-wrench physics, cancellation and launch failure checks.

Run on ikaros. Every ROS client lives in a bounded subprocess, and only current
session-token processes can be signalled. Reports retain the unmodified service
readbacks and headerless ModelStates feedback, including both host clock bounds.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import Lesson, load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes

PACKAGE = "ros_ws/src/gazeboteach_l22"
STATE_KEYS = ("position", "orientation", "linear", "angular")


def require(value, message):
    if not value:
        raise AssertionError(message)


def near(actual, expected, tolerance=1e-9):
    return math.isfinite(actual) and abs(actual-expected) <= tolerance


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hashes(folder):
    return {path.relative_to(folder).as_posix(): sha(path) for path in sorted(Path(folder).rglob("*"))
            if path.is_file() and not any(part in ("__pycache__", "build", "devel", "install") for part in path.relative_to(folder).parts)}


def save(path, result):
    Path(path).write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def rpc(uri, method, *args):
    with xmlrpc.client.ServerProxy(uri, transport=Transport(), allow_none=True) as proxy:
        code, message, value = getattr(proxy, method)("/independent_l22_acceptance", *args)
    require(code == 1, "Actual XMLRPC request failed: "+method+": "+message)
    return value


def process_identity(pid, token):
    directory = Path("/proc", str(pid))
    require(pid in live_token_processes(token), "PID is not a live current-session process")
    return {"pid": pid, "start_time": directory.joinpath("stat").read_text().rsplit(") ", 1)[1].split()[19],
            "argv": [item.decode(errors="replace") for item in directory.joinpath("cmdline").read_bytes().split(b"\0") if item]}


def signal_owned(identity, token, number):
    require(process_identity(identity["pid"], token) == identity, "Owned PID identity changed before signal")
    os.kill(identity["pid"], number)


def mid(row):
    return (row["world_time_before_ns"]+row["world_time_after_ns"])/2


def host_mid(row):
    return (row["host_monotonic_before_ns"]+row["host_monotonic_after_ns"])/2


def fit(rows, minimum_span=.15):
    require(len(rows) >= 10, "A meaningful motion fit needs ten actual truth readbacks")
    times = [(mid(row)-mid(rows[0]))/1e9 for row in rows]
    require(times[-1]-times[0] >= minimum_span, "Truth fit has insufficient simulation-time coverage")
    values = [row["state"]["linear"][0] for row in rows]
    mean_t, mean_v = sum(times)/len(times), sum(values)/len(values)
    slope = sum((t-mean_t)*(v-mean_v) for t, v in zip(times, values))/sum((t-mean_t)**2 for t in times)
    return {"sample_count": len(rows), "span_s": times[-1]-times[0], "acceleration_m_s2": slope,
            "velocity_min_m_s": min(values), "velocity_max_m_s": max(values)}


def same_state(a, b, tolerance=1e-7):
    error = max(abs(x-y) for key in ("position", "linear", "angular") for x, y in zip(a[key], b[key]))
    norm = math.sqrt(sum(x*x for x in a["orientation"])*sum(x*x for x in b["orientation"]))
    require(norm > 0, "Invalid state quaternion")
    angle = 2*math.acos(min(1, abs(sum(x*y for x, y in zip(a["orientation"], b["orientation"])))/norm))
    require(error < tolerance and angle < 1e-6, "Actual entry pose/twist was not restored")


def inspect_truth(row):
    require(row["host_monotonic_before_ns"] <= row["host_monotonic_after_ns"]
            and row["world_time_before_ns"] <= row["world_time_after_ns"], "Truth service brackets are reversed")
    require(set(row["service_header"]) == {"seq", "stamp_ns", "frame_id"}, "Raw GetModelState header is missing")
    require(type(row["service_header"]["stamp_ns"]) is int and row["service_header"]["stamp_ns"] >= 0,
            "GetModelState API ROS-time header is invalid")
    require("not a guaranteed native acquisition stamp" in row["time_basis"], "Service ROS clock is misrepresented as native capture time")
    require(all(len(row["state"][key]) == (4 if key == "orientation" else 3) for key in STATE_KEYS), "State dimensions differ")
    require(all(math.isfinite(x) for key in STATE_KEYS for x in row["state"][key]), "Non-finite actual model state")


def inspect_feedback(rows, namespace):
    require(rows, "No actual headerless ModelStates feedback")
    for row in rows:
        require(row["source_type"] == "gazebo_msgs/ModelStates" and row["model_name"] == "launch_slider"
                and row["node"] == namespace+"/observer", "Feedback source identity differs")
        require(row["source_has_header"] is False and row["source_stamp_ns"] is None,
                "Headerless ModelStates was assigned a fabricated source timestamp")
        require(row["observer_receipt_monotonic_ns"] <= row["client_receipt_monotonic_ns"], "Same-host receipt order is reversed")
        require(type(row["observer_receipt_ros_time_ns"]) is int, "Observer ROS-time readback is missing")
        require(all(math.isfinite(x) for key in STATE_KEYS for x in row["state"][key]), "Non-finite feedback state")
    require(all(b["source_message_sequence"] > a["source_message_sequence"] and
                b["observer_receipt_monotonic_ns"] > a["observer_receipt_monotonic_ns"]
                for a, b in zip(rows, rows[1:])), "Original observer sequence/host receipt order was altered")


def inspect_graph(session, readiness, force, namespace):
    graph, statuses = readiness["graph"], readiness["statuses"]
    master = session.env["ROS_MASTER_URI"]
    master_pid = rpc(master, "getPid")
    require(graph["master_uri"] == master and graph["master_pid"] == master_pid, "Actual existing Master identity differs")
    master_identity = process_identity(master_pid, session.token)
    require(any("rosmaster" in item for item in master_identity["argv"]), "Master PID is not the independent rosmaster process")
    require(rpc(master, "getParam", "/use_sim_time") is True, "Private Master /use_sim_time is not true")
    record = json.loads((session.workspace/"build/launch_lab.json").read_text())
    driver_params = {"force_n": float(force), "duration_s": .3, "lead_s": .1, "body_name": "launch_slider::body"}
    observer_params = {"model_name": "launch_slider", "publish_wall_hz": 50.0}
    launch_parents = set()
    for role, expected in (("driver", driver_params), ("observer", observer_params)):
        node, status = graph["nodes"][role], statuses[role]
        name = namespace+"/"+role
        uri = rpc(master, "lookupNode", name)
        require(node["name"] == name and node["uri"] == uri and node["pid"] == rpc(uri, "getPid") == status["pid"], "Node URI/PID differs from XMLRPC")
        process_identity(node["pid"], session.token)
        require(status["node"] == name and status["namespace"] == namespace+"/" and status["master_uri"] == master
                and status["session_token"] == session.token and Path(status["workspace"]).resolve() == session.workspace.resolve(),
                "Latched node status belongs to another namespace/session/workspace")
        require(status["parameters"] == expected == rpc(master, "getParam", name), "Private parameters/cached startup parameters differ")
        require(status["settings_sha256"] == sha(session.workspace/"launch_settings.json") == record["settings_sha256"], "Latched settings fingerprint differs")
        require(status["source_sha256"] == record["source_sha256"], "Latched source fingerprints differ")
        require(all(sha(session.workspace/path) == digest for path, digest in status["source_sha256"].items()), "Actual private launch source bytes differ")
        require(status["ros_package_path"].split(os.pathsep)[0] == str(session.workspace/"ros_ws/src"), "Private ROS package overlay is not first")
        parent = process_identity(status["parent_pid"], session.token)
        require(any("roslaunch" in Path(item).name for item in parent["argv"]) and parent["pid"] != master_pid,
                "Teaching nodes are not children of a distinct actual roslaunch")
        launch_parents.add(parent["pid"])
    require(len(launch_parents) == 1, "Both nodes must belong to the same launch process")
    driver_names = {"pulse_service": namespace+"/start_pulse", "cancel_service": namespace+"/cancel_pulse",
                    "apply_service": "/gazebo/apply_body_wrench", "clear_service": "/gazebo/clear_body_wrenches",
                    "world_service": "/gazebo/get_world_properties", "status_topic": namespace+"/driver_status"}
    require(statuses["driver"]["resolved_names"] == driver_names, "Actual driver remap resolution differs")
    require(statuses["observer"]["resolved_names"] == {"input_topic": "/gazebo/model_states", "feedback_topic": namespace+"/state_feedback", "status_topic": namespace+"/observer_status"}, "Actual observer remap resolution differs")
    for service in ("start_pulse", "cancel_pulse"):
        require(namespace+"/driver" in graph["services"].get(namespace+"/"+service, []), "Remapped actual Trigger service is missing")
    require(namespace+"/observer" in graph["subscribers"].get("/gazebo/model_states", []), "Real ModelStates input subscription is missing")
    require(namespace+"/observer" in graph["publishers"].get(namespace+"/state_feedback", []), "Real feedback publisher is missing")
    require("/gazebo" in graph["publishers"].get("/gazebo/model_states", []), "Existing Gazebo is not the ModelStates publisher")
    require("/gazebo" in graph["publishers"].get("/clock", []), "Existing Gazebo is not the simulation clock publisher")
    fresh = readiness["fresh_feedback"]
    require(len(fresh) == readiness["fresh_feedback_count"], "Fresh feedback count differs from original rows")
    if fresh:
        inspect_feedback(fresh, namespace)
    require(len(fresh) >= 3 or readiness["paused"] and readiness["feedback_pending_resume"], "Ready accepted stale feedback in a running simulation")
    return {"master_pid": master_pid, "launch_pid": next(iter(launch_parents)), "nodes": graph["nodes"]}


def inspect_report(session, result, force, namespace):
    identities = inspect_graph(session, result["readiness"], force, namespace)
    require(set(result["phases"]) == {"baseline", "active", "coast"}, "Missing real baseline/active/coast phase")
    require(result["config"]["force_n"] == force and result["config"]["namespace"] == namespace, "Report used another selected configuration")
    require(result["world_sha256"] == sha(session.workspace/"world.sdf"), "Loaded world fingerprint differs")
    record = json.loads((session.workspace/"build/launch_lab.json").read_text())
    require(result["source_sha256"] == record["source_sha256"] and result["settings_sha256"] == record["settings_sha256"],
            "Verifier report source/settings fingerprints differ from the actual prepared package")
    require(result["world"]["mass_kg"] == 1 and result["world"]["gravity_mode"] is False, "Actual body mass/gravity differs")
    truth = [row for row in result["trace"] if row["kind"] == "model_state_truth"]
    feedback = [row for row in result["trace"] if row["kind"] == "feedback"]
    for row in truth:
        inspect_truth(row)
    inspect_feedback(feedback, namespace)
    require(all(b["host_monotonic_before_ns"] > a["host_monotonic_after_ns"] and mid(b) >= mid(a)
                for a, b in zip(truth, truth[1:])), "Sequential actual readbacks moved backward")
    baseline = result["phases"]["baseline"]["truth"]
    require(len(baseline) >= 10 and all(row in truth and row["phase"] == "baseline" for row in baseline), "Baseline rows are not original trace events")
    for row in baseline:
        require(max(abs(v) for v in row["state"]["linear"]+row["state"]["angular"]) < 1e-5, "Idle startup applies a force")
        require(max(abs(v-t) for v, t in zip(row["state"]["position"], (0, 0, 1))) < 1e-5, "Idle startup moves the body")
    command = result["command"]
    start, end = command["requested_start_time_ns"], command["requested_end_time_ns"]
    require(command["force_n"] == force and command["duration_ns"] == end-start == 300000000
            and start-command["scheduling_world_time_ns"] == 100000000, "Actual Trigger message lost future start/duration/force")
    require(command["body_name"] == "launch_slider::body" and command["reference_frame"] == "world"
            and command["apply_response_success"] is True and command["command_sequence"] > 0 and len(command["pulse_id"]) == 32,
            "Actual finite ApplyBodyWrench request identity differs")
    require(result["duplicate_pulse"]["rejected"] is True and "already scheduled" in result["duplicate_pulse"]["message"], "Concurrent pulse rejection lost actual Trigger.message")
    cycle = [row for row in truth if row["phase"] != "baseline"]
    active = [row for row in cycle if start+30000000 <= mid(row) <= end-30000000]
    coast = [row for row in cycle if mid(row) >= end+50000000]
    active_fit, coast_fit = fit(active), fit(coast, .3)
    require(active == result["phases"]["active"]["truth"] and coast == result["phases"]["coast"]["truth"], "Fit rows do not point to the original service events")
    for name, actual in (("active", active_fit), ("coast", coast_fit)):
        for key, value in actual.items():
            require(near(result["phases"][name]["fit"][key], value), "Declared regression differs from independent raw least-squares fit")
    require(abs(active_fit["acceleration_m_s2"]/force-1) <= .05 and abs(coast_fit["acceleration_m_s2"]) <= .05,
            "Real motion does not accelerate at F/m then coast after finite job expiry")
    max_error = 0
    for row in cycle:
        t = max(0, (mid(row)-start)/1e9)
        expected_x = .5*force*min(.3, t)**2+force*.3*max(0, t-.3)
        error = abs(row["state"]["position"][0]-expected_x)
        max_error = max(max_error, error)
        require(error <= .01, "Real finite-pulse displacement differs by more than 1cm")
        state = row["state"]
        require(abs(state["position"][1]) <= .0001 and abs(state["position"][2]-1) <= .0001
                and max(abs(v) for v in state["linear"][1:]+state["angular"]) <= .0001, "Off-axis body motion occurred")
        if mid(row) >= end+50000000:
            require(abs(state["linear"][0]-force*.3) <= .02, "Terminal velocity does not match the finite impulse")
    require(near(result["phases"]["active"]["maximum_position_error_m"], max_error), "Declared worst displacement error differs")
    require(result["initial_cancel"]["measurement_proof"] is False, "Empty clear response was used as physical proof")
    pairs = result["phases"]["coast"]["feedback_pairs"]
    require(len(pairs) >= 5, "Insufficient real feedback/service consistency pairs")
    for pair in pairs:
        candidates = [row for row in feedback if row["source_message_sequence"] == pair["source_message_sequence"]
                      and row["observer_receipt_monotonic_ns"] == pair["observer_receipt_monotonic_ns"]]
        require(len(candidates) == 1, "Feedback pair references an invented/ambiguous original event")
        row = candidates[0]
        nearby = min(coast, key=lambda value: abs(host_mid(value)-row["observer_receipt_monotonic_ns"]))
        gap = abs(host_mid(nearby)-row["observer_receipt_monotonic_ns"])/1e9
        require(pair["truth_host_before_ns"] == nearby["host_monotonic_before_ns"] and pair["truth_host_after_ns"] == nearby["host_monotonic_after_ns"]
                and near(pair["host_mid_gap_s"], gap) and gap <= .05, "Pair is not a bounded same-host receipt neighbor")
        for key, field, tolerance in (("position", "position_error_m", .05), ("linear", "velocity_error_m_s", .02)):
            error = max(abs(a-b) for a, b in zip(row["state"][key], nearby["state"][key]))
            require(near(pair[field], error) and error <= tolerance, "Nearby actual feedback differs from independent service readback")
    require("no capture synchronization is claimed" in result["phases"]["coast"]["feedback_time_note"], "Receipt-neighbor consistency is misrepresented as capture synchronization")
    restore = result["restore"]
    require(not restore["cleanup_errors"] and restore["pause_before"] == restore["pause_after"], "Verifier cleanup failed")
    same_state(restore["state_before"], restore["state_after"])
    return {"identities": identities, "active_fit": active_fit, "coast_fit": coast_fit,
            "maximum_position_error_m": max_error, "feedback_count": len(feedback), "nearby_pairs": len(pairs)}


def run_process(session, argv, log_path, timeout=120):
    with Path(log_path).open("w", encoding="utf-8") as log:
        completed = subprocess.run(argv, cwd=str(session.workspace), env=session.env, stdout=log,
                                   stderr=subprocess.STDOUT, timeout=timeout, start_new_session=True)
    return completed.returncode


def worker_call(session, artifacts, name, operation):
    path = artifacts/(name+".json")
    status = run_process(session, [sys.executable, str(Path(__file__).resolve()), "--worker", operation, "--report", str(path)],
                         artifacts/(name+".stdout.log"), timeout=100)
    result = json.loads(path.read_text())
    require(status == 0 and result["passed"], "Actual ROS worker failed: "+str(result.get("error")))
    return result


def verify_case(session, artifacts, label, force, namespace, expected=True, diagnosis=None):
    before = worker_call(session, artifacts, label+"-entry", "snapshot")
    require(before["physics"]["time_step"] == .001 and before["body"]["mass_kg"] == 1
            and before["body"]["gravity_mode"] is False, "Independent actual physics/body properties differ")
    inputs = hashes(session.workspace)
    path = artifacts/(label+".json")
    status = run_process(session, [sys.executable, str(session.workspace/"verify.py"), "--report", str(path)], artifacts/(label+".stdout.log"))
    result = json.loads(path.read_text())
    require((status == 0) is expected and result["passed"] is expected, label+": unexpected verify result: "+str(result.get("error")))
    if diagnosis:
        require(diagnosis.lower() in result.get("error", "").lower(), "Fault rejected for an unrelated reason")
    after = worker_call(session, artifacts, label+"-exit", "snapshot")
    require(before["paused"] == after["paused"], "Verifier changed external pause state")
    same_state(before["truth"]["state"], after["truth"]["state"], tolerance=1e-6)
    require(hashes(session.workspace) == inputs, "Verifier changed personal source inputs")
    if not expected:
        require(not result["trace"] and not result.get("command"), "Invalid source/launch state reached experimental control")
        if before["paused"]:
            require(before["truth"]["world_time_after_ns"] == after["truth"]["world_time_before_ns"], "Rejected paused experiment advanced simulation time")
    checked = inspect_report(session, result, force, namespace) if expected else {"diagnosis": result["error"]}
    print(json.dumps({"scenario": label, "passed": True, "raw_report": str(path)}), flush=True)
    return {"name": label, "passed": True, "raw_report": str(path), "checked": checked}


def inspect_parse_only(session, artifacts, force, namespace):
    import yaml
    master = session.env["ROS_MASTER_URI"]
    names = [namespace+"/driver", namespace+"/observer"]
    before = {name: (rpc(master, "lookupNode", name), rpc(master, "getParam", name)) for name in names}
    override_ns = "cart_b" if namespace.endswith("cart_a") else "cart_a"
    override_force = 4 if force == 2 else 2
    outputs = {}
    for operation in ("nodes", "params"):
        completed = subprocess.run(["bash", "ros_lab.sh", operation, "force_n:="+str(override_force), "robot_ns:="+override_ns],
                                   cwd=str(session.workspace), env=session.env, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, timeout=15, start_new_session=True)
        (artifacts/("parse-"+namespace.rsplit("/", 1)[-1]+"-"+str(force)+"-"+operation+".log")).write_text(completed.stdout+completed.stderr)
        require(completed.returncode == 0, "Read-only launch parsing failed: "+completed.stderr)
        outputs[operation] = completed.stdout
    prefix = "/lesson22/"+override_ns
    require(set(outputs["nodes"].split()) == {prefix+"/driver", prefix+"/observer"}, "arg/include/group parsing did not resolve overridden node names")
    params = yaml.safe_load(outputs["params"])
    require(params[prefix+"/driver/force_n"] == float(override_force) and params[prefix+"/driver/duration_s"] == .3
            and params[prefix+"/observer/model_name"] == "launch_slider", "Parsed launch arg did not become the expected private parameter")
    after = {name: (rpc(master, "lookupNode", name), rpc(master, "getParam", name)) for name in names}
    require(before == after, "--nodes/--dump-params mutated the running nodes/parameter server")
    for name in (prefix+"/driver", prefix+"/observer"):
        try:
            rpc(master, "lookupNode", name)
        except (AssertionError, OSError, xmlrpc.client.Error):
            pass
        else:
            require(name in names, "Parse-only launch started another actual node")
    return {"passed": True, "overrides": {"force_n": override_force, "robot_ns": override_ns}, "resolved_params": params}


def inspect_cancel(result, force, active):
    require(result["cancel"]["measurement_proof"] is False and result["restore"]["cleanup_errors"] == {}, "Cancel/restore falsely claims a measurement or failed")
    rows = result["after_cancel"]
    for row in rows:
        inspect_truth(row)
    measured = fit(rows, .3)
    require(abs(measured["acceleration_m_s2"]) <= .05, "Actual force continued after scoped cancel")
    velocity = result["cancel_truth"]["state"]["linear"][0]
    require(velocity > force*.05 if active else abs(velocity) < 1e-6, "Cancellation did not occur in the requested active/queued phase")
    require(mid(rows[-1]) >= result["command"]["requested_end_time_ns"]+300000000, "Cancel observation did not pass the original job expiry")
    for row in rows:
        require(abs(row["state"]["linear"][0]-velocity) <= .02, "Cancel failed to preserve unforced inertial velocity")
        elapsed = (mid(row)-mid(result["cancel_truth"]))/1e9
        require(abs(row["state"]["position"][0]-result["cancel_truth"]["state"]["position"][0]-velocity*elapsed) <= .01, "Post-cancel displacement is not inertial motion")
    same_state(result["restore"]["before"], result["restore"]["after"])
    require(result["restore"]["pause_before"] == result["restore"]["pause_after"], "Cancel failed to restore entry pause")
    return {"passed": True, "cancelled_while": "active" if active else "queued", "coast_fit": measured, "retained_velocity_m_s": velocity}


def worker(operation, output):
    """Run all rospy operations out of the controller process, preserving raw rows."""
    sys.path.insert(0, str(Path.cwd()))
    import lab
    from config import prepared
    config, record = prepared() if operation != "snapshot" else (None, None)
    if operation == "snapshot":
        # Snapshot must still work while deliberately invalidating saved inputs.
        import config as course_config
        config = course_config.read_settings()
        record = json.loads(Path("build/launch_lab.json").read_text())
    client, result = lab.Client(config, record), {"passed": False, "operation": operation}
    original, paused = None, None
    try:
        if operation == "seed_entry":
            from gazebo_msgs.msg import ModelState
            require(client.physics().pause, "Entry-state restoration fixture requires a paused private world")
            state = ModelState()
            state.model_name, state.reference_frame = "launch_slider", "world"
            state.pose.position.x, state.pose.position.y, state.pose.position.z = .37, -.21, 1.15
            state.pose.orientation.z, state.pose.orientation.w = math.sin(.1), math.cos(.1)
            state.twist.linear.x, state.twist.linear.y, state.twist.angular.z = .11, -.04, .2
            client.apply_state(state)
            result["seeded_entry_truth"] = client.truth()
        elif operation == "snapshot":
            from gazebo_msgs.srv import GetLinkProperties
            physics = client.physics()
            body = client.call("/gazebo/get_link_properties", GetLinkProperties, link_name="launch_slider::body")
            result.update(paused=physics.pause, truth=client.truth(),
                          physics={"time_step": physics.time_step, "max_update_rate": physics.max_update_rate},
                          body={"mass_kg": body.mass, "gravity_mode": body.gravity_mode,
                                "inertia": {key: getattr(body, key) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}})
        else:
            result["readiness"] = client.ready(allow_paused=True)
            paused = client.physics().pause
            client.pause(True)
            original = client.state()
            result["restore"] = {"before": client.common.pose_twist(original.pose, original.twist), "pause_before": paused}
            client.cancel()
            client.reset_state()
            result["command"] = client.pulse()
            start, end = (result["command"][key] for key in ("requested_start_time_ns", "requested_end_time_ns"))

            def collect(target, minimum_rows=10):
                deadline, rows = time.monotonic()+15, []
                while time.monotonic() < deadline:
                    rows.append(client.truth())
                    if mid(rows[-1]) >= target and len(rows) >= minimum_rows:
                        return rows
                    time.sleep(.007)
                raise AssertionError("Bounded independent real-motion collection timed out")

            if operation != "cancel_queued":
                client.pause(False)
                result["before_interrupt"] = collect(start+120000000)
                client.pause(True)
            result["cancel_truth"] = client.truth()
            if operation == "required_exit":
                nodes = result["readiness"]["graph"]["nodes"]
                identities = {role: process_identity(node["pid"], os.environ["GAZEBOTEACH_SESSION_TOKEN"]) for role, node in nodes.items()}
                result["signalled_node"] = identities["driver"]
                signal_owned(identities["driver"], os.environ["GAZEBOTEACH_SESSION_TOKEN"], signal.SIGTERM)
                deadline = time.monotonic()+15
                while time.monotonic() < deadline:
                    live = live_token_processes(os.environ["GAZEBOTEACH_SESSION_TOKEN"])
                    if not any(identity["pid"] in live for identity in identities.values()):
                        break
                    time.sleep(.03)
                else:
                    raise AssertionError("roslaunch required-node exit failed to terminate both teaching nodes")
                result["nodes_gone"] = identities
                result["master_pid_after"] = rpc(os.environ["ROS_MASTER_URI"], "getPid")
                result["physics_step_after"] = client.physics().time_step
            else:
                result["cancel"] = client.cancel()
            client.pause(False)
            rows = collect(max(end+450000000, mid(result["cancel_truth"])+500000000))
            result["after_cancel" if operation != "required_exit" else "after_required_exit"] = rows
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    finally:
        if original is not None:
            errors = {}
            for name, action in (("pause", lambda: client.pause(True)), ("scoped_clear", client.clear_body), ("state", lambda: client.apply_state(original))):
                try:
                    action()
                except Exception as error:
                    errors[name] = str(error)
            try:
                result["restore"]["after"] = client.common.pose_twist(client.state().pose, client.state().twist)
            except Exception as error:
                errors["readback"] = str(error)
            try:
                client.pause(paused)
                result["restore"]["pause_after"] = client.physics().pause
            except Exception as error:
                errors["pause_restore"] = str(error)
            result["restore"]["cleanup_errors"] = errors
            if errors:
                result["passed"] = False
        client.close()
        save(output, result)
    return 0 if result["passed"] else 1


def bad_remap(lesson, directory, sessions, artifacts):
    source = directory/"bad-remap-source"
    shutil.copytree(str(lesson.folder), str(source), ignore=shutil.ignore_patterns("__pycache__", "build"))
    path = source/PACKAGE/"launch/nodes.launch"
    original = path.read_bytes()
    require(original.count(b'to="/gazebo/model_states"') == 1, "Remap factory cannot identify its one controlled fault")
    path.write_bytes(original.replace(b'to="/gazebo/model_states"', b'to="/lesson22/unpublished_model_states"'))
    broken = Lesson(source, lesson.data)
    session = LessonSession(ROOT, broken, storage=Storage(directory/"bad-remap-storage"))
    sessions.append(("bad_remap", session))
    started = time.monotonic()
    try:
        session.start(gui=False, terminals=False)
    except Exception as error:
        require(time.monotonic()-started < 100, "Wrong-remap readiness failed without a useful bounded deadline")
        log = session.log_path.read_text(errors="replace")
        require("Observer input remap/real subscription missing" in log, "Wrong remap failed before the intended actual graph readiness check: "+str(error))
        require(not live_token_processes(session.token), "Wrong-remap failed startup left token-owned processes")
        shutil.copy2(str(path), str(artifacts/"bad-remap-nodes.launch"))
        return {"passed": True, "controlled_fault": "actual observer input remap", "error": str(error), "elapsed_seconds": time.monotonic()-started, "leftovers": []}
    finally:
        session.stop(reason="L22 isolated wrong-remap factory cleanup")
    raise AssertionError("Wrong-remap fresh startup unexpectedly became ready")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--worker", choices=("snapshot", "seed_entry", "cancel_queued", "cancel_active", "required_exit"))
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker, args.report)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == "classic11-noetic:22"]
    require(len(found) == 1, "Expected L22: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": [], "cancellation_checks": []}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l22-") as temporary:
        directory = Path(temporary)
        try:
            for label, force, robot, paused in (("baseline", 2, "cart_a", True), ("force_only", 4, "cart_a", False), ("namespace_only", 2, "cart_b", True)):
                storage = Storage(directory/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    save(workspace/"launch_settings.json", {"force_n": force, "robot_ns": robot})
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                namespace = "/lesson22/"+robot
                if label == "namespace_only":
                    worker_call(session, artifacts, "nontrivial_entry_state", "seed_entry")
                report["scenarios"].append(verify_case(session, artifacts, label, force, namespace))
                report["scenarios"].append({"name": label+"_parse_only", **inspect_parse_only(session, artifacts, force, namespace)})
                if label == "namespace_only":
                    saved = (workspace/"launch_settings.json").read_bytes()
                    try:
                        save(workspace/"launch_settings.json", {"force_n": 4, "robot_ns": robot})
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", force, namespace, False, "reload"))
                    finally:
                        (workspace/"launch_settings.json").write_bytes(saved)
                    rpc(session.env["ROS_MASTER_URI"], "setParam", namespace+"/driver/force_n", 4.0)
                    try:
                        report["scenarios"].append(verify_case(session, artifacts, "live_parameter_cache_mismatch", force, namespace, False, "parameters"))
                    finally:
                        rpc(session.env["ROS_MASTER_URI"], "setParam", namespace+"/driver/force_n", float(force))
                    world = workspace/"world.sdf"
                    saved_world = world.read_bytes()
                    for fault, xpath, value in (("wrong_mass", "./world/model[@name='launch_slider']/link/inertial/mass", "2"),
                                               ("wrong_damping", "./world/model[@name='launch_slider']/link/velocity_decay/linear", ".1"),
                                               ("wrong_timestep", "./world/physics/max_step_size", ".002")):
                        try:
                            tree = ET.fromstring(saved_world)
                            node = tree.find(xpath)
                            require(node is not None, "Missing controlled world fault field")
                            node.text = value
                            world.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, fault, force, namespace, False, "template"))
                        finally:
                            world.write_bytes(saved_world)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_inputs_repaired", force, namespace))
                    for active in (False, True):
                        operation = "cancel_active" if active else "cancel_queued"
                        raw = worker_call(session, artifacts, operation, operation)
                        report["cancellation_checks"].append({"name": operation, **inspect_cancel(raw, force, active)})
                    raw = worker_call(session, artifacts, "required_node_exit", "required_exit")
                    rows = [row for row in raw["after_required_exit"] if mid(row) >= raw["command"]["requested_end_time_ns"]+50000000]
                    for row in raw["after_required_exit"]:
                        inspect_truth(row)
                    coast_fit = fit(rows, .3)
                    require(abs(coast_fit["acceleration_m_s2"]) <= .05 and all(0 < row["state"]["linear"][0] <= force*.301+.02 for row in rows), "Required-node failure left acceleration or an unbounded post-pulse velocity")
                    require(raw["master_pid_after"] == raw["readiness"]["graph"]["master_pid"] and raw["physics_step_after"] == .001
                            and session.server.poll() is None, "Required-node failure terminated independent Master/Gazebo")
                    require(not raw["restore"]["cleanup_errors"] and raw["restore"]["pause_before"] == raw["restore"]["pause_after"], "Required-node failure cleanup failed")
                    same_state(raw["restore"]["before"], raw["restore"]["after"])
                    report["scenarios"].append({"name": "required_node_exit", "passed": True, "coast_fit": coast_fit, "node_identities": raw["nodes_gone"], "master_pid_after": raw["master_pid_after"]})
                session.stop(reason="L22 independent scenario completed")
            report["scenarios"].append({"name": "wrong_remap_fresh_start", **bad_remap(lesson, directory, sessions, artifacts)})
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L22 final acceptance cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                    for index, path in enumerate(sorted((session.session_dir/"ros-home/log").rglob("*"))):
                        if path.is_file():
                            shutil.copy2(str(path), str(artifacts/(label+"-ros-"+str(index)+"-"+path.name)))
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key not in ("scenarios", "cancellation_checks")}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
