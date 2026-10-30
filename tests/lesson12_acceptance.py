#!/usr/bin/env python3
"""L12: distinguish network receipt, application, and actual sliding-door motion.

Pure arithmetic/protocol regressions are labelled separately from physical
acceptance. Physical runs use two fresh private Masters and real native Model
responses, ACK messages, and post-physics joint samples. This test never writes
a body pose, joint position, velocity, or synthetic physical measurement.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import select
import subprocess
import sys
import tempfile
import time
import types
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson06_acceptance import actual_stats
from tests.lesson091_acceptance import control_directory, digest, read_json, require, snapshot, source_hashes
from tests.lesson10_acceptance import bounded_steps

KEY = "classic11-noetic:12"
DT = .001
POSITION_TOLERANCE = .005
SPEED_TOLERANCE = .01
STABLE_STEPS = 200
COMPLETION_SECONDS = 3.
FORCE_LIMIT = 20.
GEOMETRY_TOLERANCE = 1e-5
LOGICAL_IDENTITY_FIELDS = ("run_id", "world", "epoch", "client_id", "seq", "nonce")
IDENTITY_FIELDS = LOGICAL_IDENTITY_FIELDS+("attempt_nonce",)
TRACE_IDENTITY_FIELDS = ("run_id", "epoch", "client_id", "seq", "nonce", "attempt_nonce")


def finite(value, label):
    require(not isinstance(value, bool), label+" cannot be a boolean number")
    number = float(value)
    require(math.isfinite(number), label+" must be finite")
    return number


def close(actual, expected, label, tolerance=1e-8):
    actual, expected = finite(actual, label), finite(expected, label)
    require(abs(actual-expected) <= tolerance,
            "{}: actual {} != expected {} +/- {}".format(label, actual, expected, tolerance))


def quaternion(raw):
    result = [finite(raw.get(name, 0.), "Native quaternion "+name) for name in ("x", "y", "z")]
    result.append(finite(raw.get("w", 1.), "Native quaternion w"))
    close(sum(value*value for value in result), 1., "Native quaternion norm", 1e-6)
    return result


def rotate(vector, quat):
    x, y, z, w = quat
    tx, ty, tz = (2*(y*vector[2]-z*vector[1]), 2*(z*vector[0]-x*vector[2]),
                  2*(x*vector[1]-y*vector[0]))
    return [vector[0]+w*tx+y*tz-z*ty, vector[1]+w*ty+z*tx-x*tz,
            vector[2]+w*tz+x*ty-y*tx]


def world_link_position(model_pose, relative_link_pose):
    """Model messages contain a world model pose and a relative link pose."""
    origin = [finite(model_pose.get("position", {}).get(axis, 0.), "Native model position") for axis in "xyz"]
    offset = [finite(relative_link_pose.get("position", {}).get(axis, 0.), "Native link position") for axis in "xyz"]
    shifted = rotate(offset, quaternion(model_pose.get("orientation", {})))
    return [left+right for left, right in zip(origin, shifted)]


def matches_identity(message, expected):
    return all(name in message and name in expected and type(message.get(name)) is type(expected.get(name))
               and message.get(name) == expected.get(name) for name in IDENTITY_FIELDS)


def successful_ack_matches(message, expected, target, stage):
    if not matches_identity(message, expected) or message.get("stage") != stage:
        return False
    try:
        close(message["target_requested_m"], target, "Successful ACK requested target", 1e-9)
        if stage in ("applied", "completed"):
            close(message["target_applied_m"], target, "Successful ACK applied target", 1e-9)
    except (AssertionError, KeyError, ValueError, TypeError):
        return False
    return stage in ("received", "applied", "completed")


def completion_run(samples, target):
    """Count real post-step intervals; 200 samples represent 200 one-ms steps."""
    longest = current = 0
    for sample in samples:
        q, speed = finite(sample["q_m"], "Actual joint position"), finite(sample["qd_m_s"], "Actual joint velocity")
        good = abs(q-target) <= POSITION_TOLERANCE and abs(speed) <= SPEED_TOLERANCE
        current = current+1 if good else 0
        longest = max(longest, current)
    return longest


def check_panel_geometry(q, position):
    require(len(position) == 3, "Actual panel world position needs three components")
    for actual, expected in zip(position, [q, 0., .6]):
        close(actual, expected, "Native joint/world panel geometry", GEOMETRY_TOLERANCE)


def check_model(model):
    expected = {"mass_kg": 2., "Ixx_kg_m2": 2*(.06**2+.8**2)/12,
                "Iyy_kg_m2": 2*(.6**2+.8**2)/12, "Izz_kg_m2": 2*(.6**2+.06**2)/12,
                "damping_Ns_m": 4., "lower_limit_m": 0., "upper_limit_m": .8,
                "effort_limit_N": 20., "physics_step_s": DT, "pid_p": 80., "pid_i": 0.,
                "pid_d": 20., "pid_command_limit_N": FORCE_LIMIT}
    for name, value in expected.items():
        close(model[name], value, "Native SDK gate rig "+name)
    for name, expected in (("box_size_xyz", [.6, .06, .8]), ("initial_axis_xyz", [1., 0., 0.]),
                           ("initial_world_pose", [0., 0., .6, 1., 0., 0., 0.])):
        require(len(model[name]) == len(expected), "Incomplete actual SDK geometry: "+name)
        for actual, target in zip(model[name], expected):
            close(actual, target, "Native SDK gate geometry "+name)
    require(model["joint_parent"] == "world" and model["joint_child"] == "panel",
            "The actual gate joint is not the declared world-to-panel constraint")
    return model


def check_snapshot(snapshot):
    q = finite(snapshot["q_m"], "Network snapshot native q")
    speed = finite(snapshot["qd_m_s"], "Network snapshot native qdot")
    force = finite(snapshot["force_N"], "Network snapshot actual native force")
    require(-POSITION_TOLERANCE <= q <= .8+POSITION_TOLERANCE, "Actual gate exceeded its bounded stroke")
    require(abs(force) <= FORCE_LIMIT+1e-7, "Actual native gate force exceeded the frozen limit")
    pose = snapshot["panel_world_pose"]
    require(len(pose) == 7, "Actual panel world pose needs xyz and quaternion wxyz")
    check_panel_geometry(q, pose[:3])
    # Quaternion sign is arbitrary, so compare the rotation rather than q.w=+1.
    quat = quaternion(dict(zip(("w", "x", "y", "z"), pose[3:])))
    for actual, expected in zip(rotate([1., 0., 0.], quat), [1., 0., 0.]):
        close(actual, expected, "Native panel world orientation", GEOMETRY_TOLERANCE)
    for actual, expected in zip(rotate([0., 1., 0.], quat), [0., 1., 0.]):
        close(actual, expected, "Native panel world orientation", GEOMETRY_TOLERANCE)
    require(len(snapshot["axis_xyz"]) == 3, "Native joint axis is incomplete")
    for actual, expected in zip(snapshot["axis_xyz"], [1., 0., 0.]):
        close(actual, expected, "Actual prismatic axis", 1e-8)
    require(type(snapshot["iteration"]) is int and snapshot["iteration"] >= 0,
            "Actual iteration must be a nonnegative integer")
    finite(snapshot["sim_time_s"], "Actual simulation clock")
    require(snapshot["sample_stage"] in ("initial", "before_physics", "after_physics"),
            "Physical snapshot did not identify its sampling stage")
    return {"q_m": q, "qd_m_s": speed, "force_N": force,
            "world_position_m": list(pose[:3]), "iteration": snapshot["iteration"],
            "sim_time_s": snapshot["sim_time_s"], "sample_stage": snapshot["sample_stage"]}


def check_native_model(model):
    """Cross-check fields within one genuine entity_info response, at one time."""
    require(model["name"].split("::")[-1] == "transport_gate", "entity_info returned the wrong actual model")
    joints = [joint for joint in model.get("joint", []) if joint["name"].split("::")[-1] == "slide_joint"]
    links = [link for link in model.get("link", []) if link["name"].split("::")[-1] == "panel"]
    require(len(joints) == len(links) == 1, "Native Model lacks the actual panel or slide joint")
    joint, panel = joints[0], links[0]
    require(joint["type"] == "PRISMATIC" and len(joint["angle"]) == 1, "Wrong actual native joint type/dimension")
    q = finite(joint["angle"][0], "Actual Model Joint.angle[0]")
    position = world_link_position(model["pose"], panel["pose"])
    check_panel_geometry(q, position)
    inertia = panel["inertial"]
    for name, value in (("mass", 2.), ("ixx", 2*(.06**2+.8**2)/12),
                        ("iyy", 2*(.6**2+.8**2)/12), ("izz", 2*(.6**2+.06**2)/12)):
        close(inertia[name], value, "Actual entity_info panel inertia "+name)
    return {"native_joint_q_m": q, "native_composed_panel_world_position_m": position,
            "sample_comparison_scope": "within_one_native_Model_response_not_asynchronous_snapshots"}


def read_trace(session):
    path = control_directory(session)/"gate.csv"
    if not path.is_file():
        return []
    labels = {"run_id", "client_id", "nonce", "attempt_nonce", "stage", "config_sha256"}
    result = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            row = {name: value if name in labels else finite(value, "Raw gate CSV "+name)
                   for name, value in raw.items()}
            for name in ("epoch", "seq", "transaction_step", "iteration"):
                require(row[name].is_integer(), "Raw CSV integer field was fractional: "+name)
                row[name] = int(row[name])
            result.append(row)
    return result


def independent_transaction(rows, ticket, events, expected_target, config_sha, motion_required=True):
    selected = [row for row in rows if all(row.get(name) == ticket[name]
                for name in TRACE_IDENTITY_FIELDS)]
    require(selected, "No actual post-physics samples for this exact network transaction")
    require(len(selected) <= 3000, "Gate transaction exceeded the frozen three-simulation-second budget")
    acknowledgements = [event for event in events if matches_identity(event, ticket)]
    stages = [event["stage"] for event in acknowledgements]
    require("applied" in stages and "completed" in stages and "failed" not in stages,
            "This exact nonce lacks a real applied/completed network ACK")
    applied = next(event for event in acknowledgements if event["stage"] == "applied")
    completed = next(event for event in acknowledgements if event["stage"] == "completed")
    require(applied["snapshot"]["sample_stage"] == "before_physics"
            and completed["snapshot"]["sample_stage"] == "after_physics",
            "Applied/completed ACKs use the wrong physical sampling stage")
    for message in acknowledgements:
        close(message["target_requested_m"], expected_target, "ACK requested target")
        if message["stage"] in ("applied", "completed"):
            close(message["target_applied_m"], expected_target, "ACK applied target")
        check_snapshot(message["snapshot"])
    first, previous, maximum_force, consecutive = selected[0], None, 0., 0
    for index, row in enumerate(selected, 1):
        require(row["transaction_step"] == index and row["config_sha256"] == config_sha,
                "Raw actual transaction lost continuity or used stale parameters")
        close(row["dt_s"], DT, "Actual physics step", 1e-10)
        close(row["transaction_elapsed_s"], index*DT, "Actual transaction step duration", 1e-9)
        close(row["target_m"], expected_target, "Raw native PD target")
        for key in ("force_N", "pid_command_N"):
            require(abs(row[key]) <= FORCE_LIMIT+1e-7, "Raw actual native force/command exceeded its frozen cap")
        close(row["force_N"], row["pid_command_N"], "Actual native input-force ledger", 1e-7)
        maximum_force = max(maximum_force, abs(row["force_N"]))
        check_panel_geometry(row["q_after_m"], [row["panel_x_m"], row["panel_y_m"], row["panel_z_m"]])
        require(-POSITION_TOLERANCE <= row["q_after_m"] <= .8+POSITION_TOLERANCE,
                "Raw actual gate position exceeded its bounded stroke")
        for name, value in zip(("axis_x", "axis_y", "axis_z"), (1., 0., 0.)):
            close(row[name], value, "Raw actual prismatic axis", 1e-8)
        qualified = abs(row["q_after_m"]-expected_target) <= POSITION_TOLERANCE and abs(row["qd_after_m_s"]) <= SPEED_TOLERANCE
        consecutive = consecutive+1 if qualified else 0
        close(row["completion_consecutive_s"], consecutive*DT, "Independently counted continuous physical completion interval", 1e-8)
        if previous:
            require(row["iteration"] == previous["iteration"]+1, "Raw physical iterations were skipped or duplicated")
            close(row["sim_time_s"]-previous["sim_time_s"], DT, "Consecutive actual clock interval", 1e-9)
            close(row["q_before_m"], previous["q_after_m"], "Actual joint position across adjacent rows", 1e-9)
            close(row["qd_before_m_s"], previous["qd_after_m_s"], "Actual joint velocity across adjacent rows", 1e-9)
        previous = row
    close(first["sim_time_s"], applied["snapshot"]["sim_time_s"], "Applied/pre-physics and first post-step world timestamp", 1e-9)
    require(first["iteration"] == applied["snapshot"]["iteration"], "Applied ACK belongs to a different actual physics step")
    close(first["force_N"], applied["snapshot"]["force_N"], "Applied ACK current pre-physics native force", 1e-9)
    close(first["pid_command_N"], applied["snapshot"]["pid_command_N"], "Applied ACK current native controller command", 1e-9)
    require(selected[-1]["stage"] == "completed", "Raw transaction did not finish with actual completion")
    post = [{"q_m": row["q_after_m"], "qd_m_s": row["qd_after_m_s"]} for row in selected]
    require(completion_run(post[-STABLE_STEPS:], expected_target) == STABLE_STEPS,
            "Actual completion lacks 200 consecutive position-and-velocity-qualified post intervals")
    require(selected[-1]["iteration"] == completed["snapshot"]["iteration"],
            "Completed ACK refers to a different physical sample")
    close(selected[-1]["sim_time_s"], completed["snapshot"]["sim_time_s"], "Completed ACK actual time", 1e-9)
    close(selected[-1]["q_after_m"], completed["snapshot"]["q_m"], "Completed ACK actual position", 1e-9)
    displacement = selected[-1]["q_after_m"]-first["q_before_m"]
    if motion_required:
        require(abs(displacement) >= .2, "A received message or target echo was mistaken for substantial actual door motion")
        require(maximum_force > 1., "The transaction has no substantial actual force evidence")
    return {"identity": {key: ticket[key] for key in IDENTITY_FIELDS}, "actual_post_step_samples": len(selected),
            "actual_duration_s": len(selected)*DT, "actual_start_q_m": first["q_before_m"],
            "actual_final_q_m": selected[-1]["q_after_m"], "actual_displacement_m": displacement,
            "maximum_actual_force_N": maximum_force, "independent_final_stable_steps": STABLE_STEPS,
            "substantial_motion_required": motion_required,
            "first_actual_sample": first, "last_actual_sample": selected[-1], "matched_network_stages": stages}


def math_regressions():
    """Pure validator fault injection; none of these samples claim to be Gazebo."""
    expected = {"run_id": "boot-a", "world": "default", "epoch": 3,
                "client_id": "numerical", "seq": 5, "nonce": "fresh", "attempt_nonce": "attempt-a"}
    require(matches_identity(dict(expected, stage="received"), expected), "Fresh receipt identity was not recognised")
    for field, value in (("run_id", "old-boot"), ("world", "another-world"), ("epoch", 2),
                         ("client_id", "other-client"), ("seq", 4), ("nonce", "latched-old"),
                         ("attempt_nonce", "concurrent-other-send")):
        require(not matches_identity(dict(expected, **{field: value}), expected),
                "Old or foreign ACK matched the current transaction: "+field)
    stable = [{"q_m": .4, "qd_m_s": .0} for _ in range(200)]
    require(completion_run(stable[:-1], .4) == 199 and completion_run(stable, .4) == 200,
            "Continuous completion window admitted fewer than 200 real post steps")
    broken = [dict(sample) for sample in stable]
    broken[100]["qd_m_s"] = .0101
    require(completion_run(broken, .4) == 100, "A brief actual velocity excursion failed to break continuity")
    theta = math.pi/2
    world_pose = {"position": {"x": 0., "y": 0., "z": 0.},
                  "orientation": {"x": 0., "y": 0., "z": math.sin(theta/2), "w": math.cos(theta/2)}}
    relative_pose = {"position": {"x": 0., "y": -.4, "z": .6}}
    position = world_link_position(world_pose, relative_pose)
    check_panel_geometry(.4, position)
    try:
        check_panel_geometry(.4, [.0, .4, .6])
    except AssertionError:
        false_pose_rejected = True
    else:
        raise AssertionError("A different world-axis displacement was accepted as door motion")
    try:
        completion_run([{"q_m": float("nan"), "qd_m_s": 0.}], .4)
    except AssertionError:
        nonfinite_rejected = True
    else:
        raise AssertionError("Nonfinite actual state satisfied completion")
    return {"passed": True, "kind": "pure_protocol_and_geometry_regressions_not_physics",
            "gazebo_started": False, "foreign_or_latched_identity_rejected": True,
            "completion_real_steps": STABLE_STEPS, "velocity_excursion_breaks_continuity": True,
            "relative_model_link_pose_composed": position, "false_pose_rejected": false_pose_rejected,
            "nonfinite_actual_state_rejected": nonfinite_rejected,
            "legacy_concurrent_ack_schedule": protocol_schedule_regression(),
            "current_vs_ticket_history_regression": event_history_regression()}


def protocol_schedule_regression():
    """A reachable ordering in the old matcher, not an observed Gazebo run."""
    logical = {"run_id": "same-boot", "world": "default", "epoch": 0,
               "client_id": "concurrent-client", "seq": 1, "nonce": "same-logical-command"}
    second = dict(logical, attempt_nonce="second-public-send")
    late_first_ack = dict(logical, attempt_nonce="first-public-send", stage="received",
                          counters={"received": 11, "rejected": 4})
    baseline = {"received": 10, "rejected": 4}
    armed = True
    legacy_identity = all(late_first_ack[name] == second[name] for name in LOGICAL_IDENTITY_FIELDS)
    legacy_counter = late_first_ack["counters"]["received"] > baseline["received"]
    legacy_matched = legacy_identity and armed and legacy_counter
    require(legacy_matched and not matches_identity(late_first_ack, second),
            "The separate send-attempt identifier did not reject the legacy same-identity race")
    valid = dict(second, stage="received", target_requested_m=.4, target_applied_m=0.)
    require(successful_ack_matches(valid, second, .4, "received"),
            "Receipt cannot require a new applied target before the physics thread consumes it")
    require(not successful_ack_matches(dict(valid, target_requested_m=.6), second, .4, "received")
            and not successful_ack_matches(dict(valid, stage="completed", target_applied_m=.6), second, .4, "completed"),
            "An otherwise matching success ACK hid a different requested or applied target")
    return {"kind": "pure_legacy_schedule_analysis_not_observed_physics", "new_physics_run": False,
            "ordering": ["send A and B read identical global counters", "both clients arm",
                         "A is accepted and raises the global received count", "A ACK reaches B after B has armed"],
            "legacy_armed_and_counter_match": legacy_matched, "new_attempt_match": False,
            "same_attempt_wrong_requested_or_applied_target_rejected": True,
            "same_logical_identity": logical, "second_send": second, "foreign_late_ack": late_first_ack}


def event_history_regression():
    """Only this invocation's event list proves a new publication; no physics."""
    counters = {"received": 4, "rejected": 2, "applied": 4, "completed": 4, "reset": 0}
    response = {"command_resent": False, "network_events": [{"event": "ack"}, {"event": "state"}],
                "ticket": {"received_network_events": [{"event": "published_once"}]},
                "bootstrap_rpc": {"payload": {"counters": dict(counters)}},
                "postflight_rpc": {"payload": {"counters": dict(counters)}}}
    assert_continuation_no_new_command(response)
    broken = dict(response, network_events=[{"event": "published_once"}])
    try:
        assert_continuation_no_new_command(broken)
    except AssertionError:
        current_publication_rejected = True
    else:
        raise AssertionError("An actual current publication was hidden by ticket-history filtering")
    changed = dict(response, postflight_rpc={"payload": {"counters": dict(counters, received=5)}})
    try:
        assert_continuation_no_new_command(changed)
    except AssertionError:
        counter_change_rejected = True
    else:
        raise AssertionError("A continuation silently accepted a new command receipt")
    return {"passed": True, "kind": "pure_dictionary_event_scope_regression_not_physics",
            "historical_send_publication_allowed": True, "current_publication_rejected": current_publication_rejected,
            "new_received_counter_rejected": counter_change_rejected, "new_physics_run": False}


def write_parameters(workspace, value):
    (workspace/"parameters.json").write_text(json.dumps({"schema": 1, "open_target_m": value})+"\n", encoding="utf-8")


def configuration_regressions(lesson, private):
    module = types.ModuleType("_l12_config_acceptance")
    module.__file__ = str(lesson.folder/"config.py")
    exec(compile((lesson.folder/"config.py").read_bytes(), module.__file__, "exec"), module.__dict__)
    folder = private/"configuration-only"
    folder.mkdir()
    for name in ("world.sdf", "parameters.json"):
        (folder/name).write_bytes((lesson.folder/name).read_bytes())
    original = (folder/"world.sdf").read_bytes()
    accepted, rejected = [], []
    for value in (.2, .4, .6, .7):
        write_parameters(folder, value)
        actual = module.read_parameters(folder/"parameters.json")
        close(actual["open_target_m"], value, "Current private target accepted without changing it")
        accepted.append(actual)
    for name, value in (("nonfinite", float("nan")), ("infinite", float("inf")),
                        ("boolean", True), ("below_supported_stroke", .19), ("above_supported_stroke", .71)):
        write_parameters(folder, value)
        try:
            module.read_parameters(folder/"parameters.json")
        except ValueError as error:
            require("open_target_m" in str(error), "Configuration error omitted the relevant target diagnosis")
            rejected.append({"name": name, "diagnosis": str(error)})
        else:
            raise AssertionError("Invalid door parameter was accepted: "+name)
    world = folder/"world.sdf"
    tree = ET.parse(str(world))
    tree.getroot().find("world/model[@name='transport_gate']/joint[@name='slide_joint']/child").text = "missing_panel"
    tree.write(str(world), encoding="utf-8", xml_declaration=True)
    try:
        module.read_world(world, lesson.folder/"world.sdf")
    except ValueError as error:
        require("world.sdf" in str(error), "Fixed topology refusal omitted the source file diagnosis")
        topology_diagnosis = str(error)
    else:
        raise AssertionError("A changed panel joint was accepted as the fixed physical experiment")
    world.write_bytes(original)
    module.read_world(world, lesson.folder/"world.sdf")
    require(world.read_bytes() == original, "Configuration-only recovery changed original world bytes")
    return {"passed": True, "kind": "configuration_only_not_physics", "gazebo_started": False,
            "accepted_current_parameters": accepted, "invalid_parameters_rejected": rejected,
            "changed_fixed_topology_rejected": topology_diagnosis, "original_world_bytes_reaccepted": True}


def python_query_guard_regressions(lesson):
    script = '''
import json,sys
sys.path.insert(0,sys.argv[1])
import instrument
calls=[]
def forbidden_native(*args,**kwargs):
    calls.append(args)
    raise AssertionError("Forbidden query reached native transport")
instrument.native=forbidden_native
results=[]
for request in ("entity_delete","world_sdf_save","world_sdf_generate","entity_info"):
    result=instrument.rpc(request,"a"*32,1)
    assert result["passed"] is False and result["transport_initialized"] is False
    assert result["published"] is False and result["network_events"] == []
    results.append({"request":request,"result":result})
assert calls == []
print(json.dumps({"passed":True,"kind":"pure_Python_native_boundary_guard_not_physics","native_calls":0,"checks":results}))
'''
    environment = dict(os.environ, GAZEBOTEACH_LESSON_SOURCE=str(lesson.folder))
    result = subprocess.run([sys.executable, "-c", script, str(lesson.folder)], env=environment,
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
    require(result.returncode == 0, "Python query allowlist did not stop before native transport: "+result.stdout+result.stderr)
    return strict_json(result.stdout.splitlines()[-1])


def native_query_guard_regressions(binary, environment=None):
    """Use an owned TCP sentinel, never a real Master, for forbidden requests."""
    binary = Path(binary).expanduser().resolve()
    require(binary.is_file(), "New SDK binary is required for native pre-network guard checks")
    checks = []
    for request in ("entity_delete", "world_sdf_save", "world_sdf_generate", "entity_info"):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sentinel:
            sentinel.bind(("127.0.0.1", 0))
            sentinel.listen(4)
            safe_env = dict(environment or os.environ)
            safe_env["GAZEBO_MASTER_URI"] = "http://127.0.0.1:{}".format(sentinel.getsockname()[1])
            result = subprocess.run([str(binary), "query", "--request", request, "--nonce", uuid.uuid4().hex,
                                     "--world", "default", "--timeout", "1"], env=safe_env,
                                    text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=7, start_new_session=True)
            ready, _, _ = select.select([sentinel], [], [], 0)
            events = []
            for line in result.stdout.splitlines():
                try:
                    events.append(strict_json(line))
                except ValueError:
                    pass
            failures = [event for event in events if event.get("event") == "client_error"]
            require(result.returncode != 0 and failures and failures[-1].get("transport_initialized") is False,
                    "Native forbidden query did not report pre-initialization refusal: "+result.stdout+result.stderr)
            require(not ready, "Forbidden native query tried connecting even to the isolated TCP sentinel")
            require(not any(event.get("event") in ("rpc_response", "published_once") for event in events),
                    "Forbidden native query published or received an RPC")
            checks.append({"request": request, "returncode": result.returncode, "events": events,
                           "stderr": result.stderr, "real_master_used": False, "sentinel_connection_attempts": 0})
    return {"passed": True, "kind": "actual_new_SDK_binary_pre_network_guard_no_physics",
            "binary": str(binary), "binary_sha256": digest(binary), "checks": checks,
            "physical_sessions_started": 0, "gazebo_master_started": False}


def strict_json(text):
    def invalid_constant(value):
        raise ValueError("Non-standard JSON numeric constant: "+value)
    return json.loads(text, parse_constant=invalid_constant)


def launch_lab(session, arguments):
    """Separate output files keep two actual concurrent senders independent."""
    path = control_directory(session)/("acceptance-"+uuid.uuid4().hex+".client.log")
    wrapper = ("import os,sys; fd=os.open(sys.argv[1],os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600);"
               "os.dup2(fd,1);os.dup2(fd,2);os.close(fd);"
               "os.execv(sys.executable,[sys.executable]+sys.argv[2:])")
    process = session._spawn("L12 bounded SDK client", [sys.executable, "-c", wrapper, str(path),
                             str(session.lesson.folder/"lab.py")]+list(arguments))
    return {"process": process, "path": path, "started": time.monotonic(), "arguments": list(arguments)}


def collect_lab(session, launched, accepted=True, timeout=28):
    """Bound the whole CLI including discovery and teardown, not only its RPC."""
    process = launched["process"]
    timed_out = False
    try:
        code = process.wait(timeout=max(.1, timeout-(time.monotonic()-launched["started"])))
    except subprocess.TimeoutExpired:
        timed_out = True
        terminate_owned(process)
        code = 124
    output = launched["path"].read_text(encoding="utf-8", errors="replace") if launched["path"].is_file() else ""
    parsed = None
    for line in reversed(output.splitlines()):
        try:
            candidate = strict_json(line)
        except ValueError:
            continue
        if isinstance(candidate, dict):
            parsed = candidate
            break
    evidence = {"arguments": launched["arguments"], "returncode": code, "combined_stdout_stderr": output,
                "outer_timeout_s": timeout, "outer_timeout_occurred": timed_out,
                "elapsed_wall_s": time.monotonic()-launched["started"], "data": parsed}
    # Preserve even a failed client's network log before raising an assertion.
    log = control_directory(session)/"acceptance-client-results.jsonl"
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(evidence, ensure_ascii=False, allow_nan=False)+"\n")
    require(not timed_out, "Transport CLI exceeded its owned process-group outer deadline: "+output)
    require(parsed is not None, "Transport CLI emitted no structured result: "+output)
    if accepted is not None:
        require((code == 0) is accepted, "Unexpected CLI outcome: "+repr(launched["arguments"])+"\n"+output)
    return evidence


def lab(session, *arguments, accepted=True, timeout=28):
    return collect_lab(session, launch_lab(session, arguments), accepted=accepted, timeout=timeout)


def read_lines(path):
    if not path.is_file():
        return []
    return [strict_json(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def stored_ack_events(session):
    return read_lines(control_directory(session)/"ack-events.jsonl")


def frozen_csv(session):
    path = control_directory(session)/"gate.csv"
    return digest(path) if path.is_file() else None


def preserve_evidence(session, destination):
    copied = snapshot(session, destination)
    for path in list(control_directory(session).glob("*.jsonl"))+list(control_directory(session).glob("*.client.log")):
        target = destination/path.name
        shutil.copyfile(str(path), str(target))
        copied[path.name] = {"path": str(target), "sha256": digest(target), "bytes": target.stat().st_size}
    return copied


def objects(value):
    """Walk logging containers, without trusting their `passed`/`matched` flags."""
    if isinstance(value, dict):
        yield value
        for item in value.values():
            yield from objects(item)
    elif isinstance(value, list):
        for item in value:
            yield from objects(item)


def network_events(result, kind="ack"):
    events = [event for event in objects(result) if event.get("event") == kind and "payload" in event]
    for event in events:
        require("raw_payload_json" in event and strict_json(event["raw_payload_json"]) == event["payload"],
                "Decoded network event differs from its actual Any.STRING JSON bytes")
    return events


def network_acks(result):
    return [event["payload"] for event in network_events(result)]


def assert_no_current_publication(data):
    current = data.get("network_events")
    require(isinstance(current, list), "This invocation did not retain its own network event list")
    require(not any(event.get("event") == "published_once" for event in current),
            "This invocation published a new actuator command")


def assert_input_counters_unchanged(before, after, label):
    for key in ("received", "rejected"):
        require(before["counters"][key] == after["counters"][key], label+" changed the actual "+key+" counter")


def assert_continuation_no_new_command(data, external_after=None):
    assert_no_current_publication(data)
    require(data.get("command_resent") is False, "Continuation did not explicitly confirm no command resend")
    before = data["bootstrap_rpc"]["payload"]
    after = data["postflight_rpc"]["payload"] if "postflight_rpc" in data else external_after
    require(after is not None, "Continuation has no actual after-state evidence for publication counters")
    assert_input_counters_unchanged(before, after, "Read-only ticket continuation")


def rpc_event(result, request, response_type):
    events = [event for event in objects(result) if event.get("event") == "rpc_response"
              and event.get("request") == request]
    require(events, "No actual native RPC Response evidence for "+request)
    event = events[-1]
    require(type(event["request_id"]) is int and event["request_id"] > 0
            and event["request_id"] == event["response_id"], "Native RPC IDs were not exactly correlated")
    require(event["response"] == "success" and event["response_type"] == response_type
            and event["serialized_data_size"] > 0 and event["response_debug"],
            "Native RPC response status/type/serialized data were not validated")
    return event


def query_snapshot(session):
    nonce = uuid.uuid4().hex
    result = lab(session, "query", "--request", "gazeboteach_transport_snapshot", "--nonce", nonce)
    event = rpc_event(result["data"], "gazeboteach_transport_snapshot", "gazebo.msgs.Any")
    message = event["payload"]
    require(message["query_nonce"] == nonce and message["run_id"] == session.token
            and message["world"] == session.world_name, "Snapshot RPC reused a stale query or another boot/world")
    require(type(message["epoch"]) is int and message["epoch"] >= 0 and not message["error"],
            "Current RPC snapshot has an invalid epoch or server error")
    require(message["config_sha256"] == digest(session.workspace/"parameters.json")
            and message["world_sha256"] == digest(session.workspace/"world.sdf")
            and message["source_sha256"] == digest(session.lesson.folder/"transport_gate.cc")
            and message["client_source_sha256"] == digest(session.lesson.folder/"transport_client.cc"),
            "Actual RPC does not identify the currently loaded private inputs and code")
    check_model(message["model"])
    check_snapshot(message["snapshot"])
    return {"raw_client_result": result, "response": event, "message": message}


def native_entity(session):
    result = lab(session, "entity")
    event = rpc_event(result["data"], "entity_info", "gazebo.msgs.Model")
    return {"raw_client_result": result, "response": event, "independent_geometry": check_native_model(event["payload"])}


def paused_evidence(session):
    stats = actual_stats(session)
    require(stats["paused"], "This protocol assertion requires an actually paused private world")
    rpc, entity = query_snapshot(session), native_entity(session)
    close(rpc["message"]["snapshot"]["q_m"], entity["independent_geometry"]["native_joint_q_m"],
          "Paused network snapshot vs independent native Model q", GEOMETRY_TOLERANCE)
    after = actual_stats(session)
    require(after["paused"] and after["iterations"] == stats["iterations"]
            and after["sim_time_sec_nsec"] == stats["sim_time_sec_nsec"],
            "Read-only snapshot/entity RPC unexpectedly advanced the paused simulation")
    return {"stats": stats, "rpc": rpc, "entity": entity}


def assert_no_execution(before, after, label, allow_received=False):
    require(before["stats"]["iterations"] == after["stats"]["iterations"]
            and before["stats"]["sim_time_sec_nsec"] == after["stats"]["sim_time_sec_nsec"],
            label+" advanced paused physics")
    old, new = before["rpc"]["message"], after["rpc"]["message"]
    for key in ("epoch", "target_applied_m"):
        require(new[key] == old[key], label+" changed the epoch or applied target")
    for name in ("applied", "completed", "reset"):
        require(new["counters"][name] == old["counters"][name], label+" executed or reset a transaction")
    if not allow_received:
        require(new["pending"] == old["pending"] and new["active"] == old["active"]
                and new["counters"]["received"] == old["counters"]["received"],
                label+" changed accepted command state")
        for key in ("client_id", "seq", "nonce", "attempt_nonce"):
            require(new[key] == old[key], label+" changed the accepted command identity")
    for name in ("q_m", "qd_m_s", "force_N", "pid_command_N"):
        close(new["snapshot"][name], old["snapshot"][name], label+" changed cached physical state "+name, 1e-12)
    close(after["entity"]["independent_geometry"]["native_joint_q_m"],
          before["entity"]["independent_geometry"]["native_joint_q_m"], label+" moved the actual native joint", 1e-12)


def identity(session, epoch, client, seq):
    return {"run_id": session.token, "world": session.world_name, "epoch": epoch,
            "client_id": client, "seq": seq, "nonce": uuid.uuid4().hex}


def send(session, expected, target, wait="completed", accepted=True, **overrides):
    ticket = control_directory(session)/("acceptance-ticket-"+uuid.uuid4().hex+".json")
    arguments = ["send", "--client", expected["client_id"], "--seq", str(expected["seq"]),
                 "--nonce", expected["nonce"], "--target", str(target), "--wait", wait, "--ticket", str(ticket)]
    for name, value in overrides.items():
        arguments += ["--"+name.replace("_", "-")]
        if value is not None:
            arguments += [str(value)]
    result = lab(session, *arguments, accepted=accepted)
    actual_ticket = result["data"]["ticket"]
    require(all(actual_ticket[name] == expected[name] for name in LOGICAL_IDENTITY_FIELDS),
            "Public send changed the requested logical command identity")
    require(re.fullmatch(r"[0-9a-f]{32}", actual_ticket["attempt_nonce"]), "Send attempt is not a fresh UUID hex identifier")
    matching = [message for message in network_acks(result["data"]) if matches_identity(message, actual_ticket)]
    require(matching, "The current client has no real network ACK for its nonce")
    stage = wait if accepted else "rejected"
    require(any(message["stage"] == stage for message in matching), "Expected actual network ACK stage "+stage)
    if accepted:
        require(any(successful_ack_matches(message, actual_ticket, float(target), stage) for message in matching),
                "Successful current-attempt ACK does not match this ticket's numeric target")
    for message in matching:
        if message["stage"] in ("received", "applied", "completed"):
            close(message["target_requested_m"], float(target), "Current attempt ACK requested target")
        if message["stage"] in ("applied", "completed"):
            close(message["target_applied_m"], float(target), "Current attempt ACK applied target")
    return {"ticket": {key: actual_ticket[key] for key in IDENTITY_FIELDS}, "ticket_path": str(ticket), "raw_client_result": result,
            "network_events": network_events(result["data"])}


def finish_ticket(session, transaction):
    result = lab(session, "wait", "--ticket", transaction["ticket_path"], "--wait", "completed")
    assert_continuation_no_new_command(result["data"])
    require(all(result["data"]["ticket"][key] == transaction["ticket"][key] for key in IDENTITY_FIELDS),
            "Continuation changed the accepted command or its individual send-attempt identity")
    target = float(result["data"]["ticket"]["target_text"])
    current = network_events({"network_events": result["data"]["network_events"]})
    require(any(successful_ack_matches(event["payload"], transaction["ticket"], target, "completed") for event in current),
            "This continuation subscription did not receive its exact accepted attempt's completed ACK")
    return dict(transaction, continuation_result=result)


def capture_applied_wire(session, transaction, target):
    """Observe the cached actual applied ACK while physics is still paused."""
    before = query_snapshot(session)["message"]
    result = lab(session, "watch", "--seconds", "1")
    assert_no_current_publication(result["data"])
    after = query_snapshot(session)["message"]
    assert_input_counters_unchanged(before, after, "Independent applied-ACK subscription")
    observed = network_events({"network_events": result["data"]["network_events"]})
    applied = [event for event in observed if successful_ack_matches(event["payload"], transaction["ticket"], target, "applied")]
    require(applied, "Independent real subscription did not observe the paused accepted attempt's applied ACK")
    for event in applied:
        check_snapshot(event["payload"]["snapshot"])
        require(event["payload"]["snapshot"]["sample_stage"] == "before_physics",
                "Applied wire ACK did not identify the actual pre-physics force snapshot")
    return dict(transaction, independent_applied_wire_observation=result)


def actual_transaction(session, transaction, target):
    events = network_acks(transaction)
    measured = independent_transaction(read_trace(session), transaction["ticket"], events, target,
                                       digest(session.workspace/"parameters.json"))
    published = [message for message in stored_ack_events(session) if matches_identity(message, transaction["ticket"])
                 and message["stage"] == "applied"]
    require(len(published) == 1, "One accepted transaction was applied more than once")
    return {"transport": transaction, "independent_actual_motion": measured, "server_applied_publications": 1}


def await_ready(session):
    deadline, last = time.monotonic()+15, None
    while time.monotonic() < deadline:
        require(session.running, "Transport gate server exited before loading its physical observer")
        try:
            last = read_json(control_directory(session)/"status.json")
            if last.get("run_id") == session.token and last.get("model"):
                require(not last["error"], "Transport gate failed during initialization: "+last["error"])
                return query_snapshot(session)
        except (OSError, ValueError):
            pass
        time.sleep(.05)
    raise AssertionError("No ready loaded transport gate: "+repr(last))


def introspection(session):
    evidence = []
    for suffix, expected in (("command", "gazebo.msgs.Request"), ("ack", "gazebo.msgs.Any"), ("state", "gazebo.msgs.Any")):
        topic = "/gazebo/{}/transport_lab/{}".format(session.world_name, suffix)
        result = subprocess.run(["gz", "topic", "-w", session.world_name, "-i", topic], env=dict(session.env),
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=6)
        require(re.search(r"\bType:\s*"+re.escape(expected)+r"\b", result.stdout),
                "Actual Master did not report the expected topic wire type: "+result.stdout+result.stderr)
        if suffix == "command":
            require(re.search(r"Subscribers:\s*\S", result.stdout), "Command topic has no real actuator subscriber")
        evidence.append({"topic": topic, "expected_wire_type": expected, "stdout": result.stdout,
                         "stderr": result.stderr, "returncode": result.returncode, "exit_code_alone_is_not_acceptance": True})
    return {"kind": "actual_private_master_type_introspection", "topics": evidence,
            "gazebo_master_uri": session.env["GAZEBO_MASTER_URI"]}


def verify_case(session, name, diagnostic=None):
    before = actual_stats(session)
    files = source_hashes(session.workspace)
    trace_sha = frozen_csv(session)
    old = read_json(control_directory(session)/"status.json")
    old_keys = {tuple(row[field] for field in TRACE_IDENTITY_FIELDS)
                for row in read_trace(session)}
    offset = session.log_path.stat().st_size
    result = session.verify()
    raw = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
    after = actual_stats(session)
    require(before["paused"] == after["paused"], "Canonical transport verification did not preserve actual pause")
    require(files == source_hashes(session.workspace), "Canonical verification changed personal exercise bytes")
    evidence = {"name": name, "verifier_result": result, "raw_verifier_output": raw,
                "before_stats": before, "after_stats": after, "pause_preserved": True,
                "personal_files_unchanged": True}
    if diagnostic:
        require(before["paused"] and result["passed"] is False and diagnostic.lower() in raw.lower(),
                "Invalid/stale personal input lacks its relevant diagnosis: "+diagnostic+"\n"+raw)
        require(before["iterations"] == after["iterations"] and before["sim_time_sec_nsec"] == after["sim_time_sec_nsec"],
                "Invalid personal input advanced physics before refusing control")
        current = read_json(control_directory(session)/"status.json")
        for key in IDENTITY_FIELDS+("stage", "pending", "active", "target_applied_m"):
            require(current[key] == old[key], "Invalid personal input changed current command state: "+key)
        for name in ("received", "applied", "completed", "reset"):
            require(current["counters"][name] == old["counters"][name], "Rejected personal input emitted a transport command")
        require(frozen_csv(session) == trace_sha, "Invalid personal input appended fabricated/new physical samples")
        evidence["rejected_before_transport_control_or_physics"] = True
    else:
        require(result["passed"] is True, "Canonical Transport verification failed:\n"+raw)
        canonical = read_json(control_directory(session)/"verification.json")
        require(canonical["config_sha256"] == digest(session.workspace/"parameters.json")
                and canonical["world_sha256"] == digest(session.workspace/"world.sdf")
                and canonical["parameters"] == read_json(session.workspace/"parameters.json"),
                "Canonical transport measurement used stale private input bytes")
        check_model(canonical["actual_model"])
        transactions = canonical["transactions"]
        require([item["role"] for item in transactions] == ["prepare_closed", "open", "close"],
                "Full verification must perform three real transactions, including closed-state preparation")
        target = canonical["parameters"]["open_target_m"]
        rows, independent = read_trace(session), []
        for item in transactions:
            ticket = item["ticket"]
            require(tuple(ticket[field] for field in TRACE_IDENTITY_FIELDS) not in old_keys,
                    "Canonical verification reused an old physical transaction")
            value = target if item["role"] == "open" else 0.
            events = network_acks(item)
            matching = [event for event in events if matches_identity(event, ticket)]
            require(all(any(event["stage"] == stage for event in matching) for stage in ("received", "applied", "completed")),
                    "A canonical transaction lacks three actual network ACK stages for its new nonce")
            motion = independent_transaction(rows, ticket, events, value, canonical["config_sha256"],
                                             motion_required=item["role"] != "prepare_closed")
            for side in ("entity_before", "entity_after"):
                event = rpc_event(item[side], "entity_info", "gazebo.msgs.Model")
                actual = check_native_model(event["payload"])
                motion[side] = actual
            close(motion["entity_after"]["native_joint_q_m"], value, "Native entity_info actual target endpoint", POSITION_TOLERANCE)
            if item["role"] in ("open", "close"):
                initial = 0. if item["role"] == "open" else target
                close(motion["actual_start_q_m"], initial, "Actual opening/closing initial position", POSITION_TOLERANCE)
            applied = [event for event in stored_ack_events(session) if matches_identity(event, ticket) and event["stage"] == "applied"]
            require(len(applied) == 1, "Canonical transaction was applied more than once")
            independent.append({"role": item["role"], "actual_physics": motion})
        evidence.update(canonical_measurements=canonical, independent_actual_transactions=independent)
    evidence["passed"] = True
    return evidence


def normal_and_isolation(a, b):
    require(a.env["GAZEBO_MASTER_URI"] != b.env["GAZEBO_MASTER_URI"] and a.token != b.token,
            "Transport isolation did not create separate private Masters and boot identities")
    b.pause(True)
    before_b = paused_evidence(b)
    close(before_b["entity"]["independent_geometry"]["native_joint_q_m"], 0., "Private B initially closed", POSITION_TOLERANCE)
    personal = source_hashes(a.workspace)
    a.pause(False)
    initial = query_snapshot(a)["message"]
    opening = send(a, identity(a, initial["epoch"], "master-isolation", 1), .6)
    opened = native_entity(a)
    close(opened["independent_geometry"]["native_joint_q_m"], .6, "A actual open endpoint", POSITION_TOLERANCE)
    # This check deliberately occurs while A remains physically open, not after
    # an open/close cycle which could hide a cross-Master excursion in B.
    after_b = paused_evidence(b)
    assert_no_execution(before_b, after_b, "A command leaking into private Master B")
    require(before_b["rpc"]["message"]["counters"] == after_b["rpc"]["message"]["counters"],
            "Private B received or rejected an A command despite using another Classic Master")
    watched = network_events(opening, "state")
    states = [event["payload"] for event in watched if matches_identity(event["payload"], opening["ticket"])]
    require(len(states) >= 2 and max(message["snapshot"]["q_m"] for message in states)
            -min(message["snapshot"]["q_m"] for message in states) > .01,
            "No changing actual network state stream accompanied the force-driven opening")
    for message in states:
        check_snapshot(message["snapshot"])
    closing = send(a, identity(a, initial["epoch"], "master-isolation", 2), 0.)
    closed = native_entity(a)
    close(closed["independent_geometry"]["native_joint_q_m"], 0., "A actual closed endpoint", POSITION_TOLERANCE)
    require(personal == source_hashes(a.workspace), "Transport commands overwrote A personal exercise files")
    return {"name": "real_default_open_close_and_private_master_isolation", "passed": True,
            "actual_open": actual_transaction(a, opening, .6), "actual_close": actual_transaction(a, closing, 0.),
            "independent_A_open_entity": opened, "independent_A_closed_entity": closed,
            "B_before_while_paused": before_b, "B_while_A_is_still_open": after_b,
            "changing_actual_network_state_samples": len(states),
            "same_full_topics": "/gazebo/default/transport_lab/{command,ack,state}",
            "masters": [a.env["GAZEBO_MASTER_URI"], b.env["GAZEBO_MASTER_URI"]]}


def paused_delivery_and_latching(session):
    session.pause(True)
    personal = source_hashes(session.workspace)
    before = paused_evidence(session)
    expected = identity(session, before["rpc"]["message"]["epoch"], "pause-sequence", 1)
    received = send(session, expected, .4, wait="received")
    accepted_identity = received["ticket"]
    foreign = [event for event in network_events(received) if not matches_identity(event["payload"], accepted_identity)
               and event["payload"].get("stage") == "completed"]
    require(foreign and all(event["matched"] is False for event in foreign),
            "New client did not expose and reject a real latched completion ACK from the previous nonce")
    after_receipt = paused_evidence(session)
    assert_no_execution(before, after_receipt, "Receipt without physical stepping", allow_received=True)
    require(after_receipt["rpc"]["message"]["pending"] is True
            and after_receipt["rpc"]["message"]["active"] is False,
            "Paused transport receipt was incorrectly called physical application")
    require(not any(message["stage"] in ("applied", "completed")
                    for message in network_acks(received) if matches_identity(message, accepted_identity)),
            "Paused client accepted a stale ACK as new physical execution")
    busy = send(session, identity(session, expected["epoch"], "busy-sequence", 1), .2, accepted=False)
    after_busy = paused_evidence(session)
    assert_no_execution(after_receipt, after_busy, "A second command arriving while one is pending")
    duplicate = send(session, expected, .4, wait="received", accepted=False)
    after_duplicate = paused_evidence(session)
    assert_no_execution(after_receipt, after_duplicate, "Duplicate accepted identity")
    acknowledgements = [bounded_steps(session, 20, time.monotonic()+30)]
    moved = paused_evidence(session)
    require(moved["rpc"]["message"]["counters"]["applied"] == before["rpc"]["message"]["counters"]["applied"]+1,
            "Real owned steps did not apply exactly one queued command")
    require(moved["entity"]["independent_geometry"]["native_joint_q_m"]
            > before["entity"]["independent_geometry"]["native_joint_q_m"]+1e-5,
            "Queued command ACK did not cause actual native door motion after stepping")
    received = capture_applied_wire(session, received, .4)
    session.pause(False)
    finished = finish_ticket(session, received)
    session.pause(True)
    final = paused_evidence(session)
    close(final["entity"]["independent_geometry"]["native_joint_q_m"], .4, "Paused continuation actual endpoint", POSITION_TOLERANCE)
    # Re-sending is forbidden: aggregate the receipt and the new subscriber's
    # applied/completed observations for one unchanged identity.
    actual_open = actual_transaction(session, finished, .4)
    session.pause(False)
    closed = send(session, identity(session, expected["epoch"], "pause-sequence", 2), 0.)
    session.pause(True)
    require(personal == source_hashes(session.workspace), "Paused workflow changed personal exercise files")
    return {"name": "paused_receipt_latched_old_ACK_duplicate_and_same_ticket_continuation", "passed": True,
            "before": before, "after_received_without_step": after_receipt,
            "actual_latched_foreign_completion_ACKs": foreign, "duplicate_rejection": duplicate,
            "busy_does_not_supersede_pending_command": busy,
            "owned_step_acknowledgements": acknowledgements, "actual_state_after_20_steps": moved,
            "actual_open": actual_open, "actual_close": actual_transaction(session, closed, 0.),
            "continuation_published_new_command": False}


def safe_protocol_errors(session):
    session.pause(True)
    checks = []
    epoch = query_snapshot(session)["message"]["epoch"]
    cases = (("unsupported_verb", .4, {"verb": "unsupported"}),
             ("missing_optional_target_field", .4, {"omit_target": None}),
             ("nonfinite_target", "nan", {}), ("outside_target_stroke", .71, {}),
             ("foreign_boot", .4, {"run": "foreign-"+uuid.uuid4().hex}),
             ("old_reset_epoch", .4, {"epoch": epoch-1}))
    for name, target, changes in cases:
        before = paused_evidence(session)
        expected = identity(session, epoch, "reject-sequence", 100)
        result = send(session, expected, target, accepted=False, **changes)
        after = paused_evidence(session)
        assert_no_execution(before, after, name)
        rejects = [event for event in network_acks(result) if matches_identity(event, result["ticket"]) and event["stage"] == "rejected"]
        require(rejects and rejects[-1]["reason"], "Safe application error has no actual correlated refusal reason")
        if name == "nonfinite_target":
            require(rejects[-1]["target_requested_m"] is None,
                    "Nonfinite requested target leaked an invalid JSON NaN instead of null")
        checks.append({"name": name, "transport": result, "before": before, "after": after})
    before = paused_evidence(session)
    old_seq = identity(session, epoch, "pause-sequence", 1)
    result = send(session, old_seq, .4, accepted=False)
    after = paused_evidence(session)
    assert_no_execution(before, after, "Stale accepted sequence with a new nonce")
    checks.append({"name": "previous_accepted_sequence_rejected_even_with_fresh_nonce", "transport": result})
    for name, arguments in (
            ("wrong_expected_wire_type_before_publish", ("send", "--target", ".4", "--expect-type", "gazebo.msgs.Pose")),
            ("wrong_topic_without_actuator", ("send", "--target", ".4", "--topic", "/gazebo/default/transport_lab/missing_command")),
            ("unknown_readonly_rpc", ("query", "--request", "gazeboteach_transport_missing", "--nonce", uuid.uuid4().hex, "--timeout", "1"))):
        before = paused_evidence(session)
        result = lab(session, *arguments, accepted=False, timeout=14)
        after = paused_evidence(session)
        assert_no_execution(before, after, name)
        assert_no_current_publication(result["data"])
        assert_input_counters_unchanged(before["rpc"]["message"], after["rpc"]["message"], name)
        checks.append({"name": name, "bounded_client_failure": result, "before": before, "after": after})
    return {"name": "safe_network_rejections_and_bounded_discovery_rpc_failures", "passed": True,
            "checks": checks, "conflicting_wire_type_injected_into_live_master": False,
            "accepted_sequence_will_be_rechecked_with_seq1_after_rejected_seq100": True}


def pending_reset(session):
    session.pause(True)
    personal = source_hashes(session.workspace)
    before = paused_evidence(session)
    old_epoch = before["rpc"]["message"]["epoch"]
    expected = identity(session, old_epoch, "reject-sequence", 1)
    # The earlier invalid seq=100 messages must not poison accepted seq history.
    queued = send(session, expected, .4, wait="received")
    accepted_identity = queued["ticket"]
    require(not any(all(row.get(key) == accepted_identity[key] for key in TRACE_IDENTITY_FIELDS)
                    for row in read_trace(session)),
            "Paused receipt already produced physical transaction samples")
    before_reset = actual_stats(session)
    session.reset_models()
    after_reset = actual_stats(session)
    require(after_reset["paused"] and after_reset["iterations"] >= before_reset["iterations"]
            and after_reset["sim_time"] >= before_reset["sim_time"], "Public model reset rewound the world clock or changed pause")
    acknowledgements = [bounded_steps(session, 20, time.monotonic()+30)]
    reset = paused_evidence(session)
    current = reset["rpc"]["message"]
    require(current["epoch"] == old_epoch+1 and current["pending"] is False and current["active"] is False,
            "Public model reset did not create a new idle epoch and cancel pending work")
    require(current["counters"]["applied"] == before["rpc"]["message"]["counters"]["applied"],
            "A cancelled paused command was nevertheless applied")
    require(not any(all(row.get(key) == accepted_identity[key] for key in TRACE_IDENTITY_FIELDS)
                    for row in read_trace(session)), "Reset jumps became old-epoch physical measurements")
    cancellation = [event for event in stored_ack_events(session) if matches_identity(event, accepted_identity) and event["stage"] == "cancelled"]
    require(len(cancellation) == 1, "Pending reset has no one-time cancellation evidence for the actual old nonce")
    invalid_ticket = lab(session, "wait", "--ticket", queued["ticket_path"], "--wait", "completed", accepted=False)
    after_cancelled_wait = query_snapshot(session)["message"]
    assert_continuation_no_new_command(invalid_ticket["data"], external_after=after_cancelled_wait)
    old_request = identity(session, current["epoch"], "old-epoch-client", 1)
    stale = send(session, old_request, .4, accepted=False, epoch=old_epoch)
    after_stale = paused_evidence(session)
    assert_no_execution(reset, after_stale, "Old epoch command replay after reset")
    close(after_stale["entity"]["independent_geometry"]["native_joint_q_m"], 0., "Public reset actual door closed", POSITION_TOLERANCE)
    session.pause(False)
    opened = send(session, identity(session, current["epoch"], "reject-sequence", 1), .4)
    closed = send(session, identity(session, current["epoch"], "reject-sequence", 2), 0.)
    session.pause(True)
    require(personal == source_hashes(session.workspace), "Public reset or stale-message handling changed private exercise files")
    return {"name": "public_reset_cancels_pending_and_old_epoch_cannot_revive", "passed": True,
            "queued_before_reset": queued, "rejected_high_sequence_did_not_advance_accepted_history": True,
            "before_reset_stats": before_reset, "after_reset_stats": after_reset,
            "idle_refresh_steps": acknowledgements, "new_epoch_actual_evidence": reset,
            "old_nonce_cancelled_once": cancellation[0], "cancelled_ticket_wait": invalid_ticket,
            "old_epoch_rejection": stale, "new_epoch_open": actual_transaction(session, opened, .4),
            "new_epoch_close": actual_transaction(session, closed, 0.), "test_writes_body_pose_or_velocity": False}


def concurrent_logical_duplicate(session):
    """Two real public sends share command identity but own different attempts."""
    session.pause(True)
    before = paused_evidence(session)
    close(before["entity"]["independent_geometry"]["native_joint_q_m"], 0., "Parallel send setup starts closed", POSITION_TOLERANCE)
    logical = identity(session, before["rpc"]["message"]["epoch"], "parallel-sequence", 1)
    targets = [.4, .6]
    launched, paths = [], []
    for target in targets:
        path = control_directory(session)/("parallel-ticket-"+uuid.uuid4().hex+".json")
        arguments = ["send", "--target", str(target), "--client", logical["client_id"], "--seq", str(logical["seq"]),
                     "--nonce", logical["nonce"], "--wait", "received", "--ticket", str(path)]
        launched.append(launch_lab(session, arguments))
        paths.append(path)
    require(all(item["process"].poll() is None for item in launched),
            "Actual concurrent-send setup ran its two clients serially")
    results = [collect_lab(session, item, accepted=None) for item in launched]
    accepted = [index for index, result in enumerate(results) if result["returncode"] == 0]
    require(len(accepted) == 1, "Two attempts of one logical command did not yield exactly one accepted sender")
    tickets = [result["data"]["ticket"] for result in results]
    require(all(all(ticket[key] == logical[key] for key in LOGICAL_IDENTITY_FIELDS) for ticket in tickets),
            "The actual parallel senders did not share their full logical command identity")
    require(all(re.fullmatch(r"[0-9a-f]{32}", ticket["attempt_nonce"]) for ticket in tickets)
            and tickets[0]["attempt_nonce"] != tickets[1]["attempt_nonce"],
            "Public sends did not automatically create distinct send-attempt identities")
    foreign = []
    for index, result in enumerate(results):
        current = [event for event in network_acks(result["data"]) if matches_identity(event, tickets[index])]
        expected_stage = "received" if index == accepted[0] else "rejected"
        require(any(event["stage"] == expected_stage for event in current),
                "One actual sender consumed another attempt's ACK as its own outcome")
        for event in current:
            if event["stage"] in ("received", "rejected"):
                close(event["target_requested_m"], targets[index], "Concurrent attempt ACK target")
        for event in network_events(result["data"]):
            body = event["payload"]
            if all(body.get(key) == logical[key] for key in LOGICAL_IDENTITY_FIELDS) and body["attempt_nonce"] != tickets[index]["attempt_nonce"]:
                require(event["matched"] is False, "Native client matched a real ACK from the other concurrent send attempt")
                foreign.append(event)
    require(foreign, "No real same-logical-identity foreign-attempt ACK was observed by the concurrent clients")
    pending = paused_evidence(session)
    assert_no_execution(before, pending, "Concurrent sends before physical stepping", allow_received=True)
    state, old = pending["rpc"]["message"], before["rpc"]["message"]
    require(state["pending"] is True and state["active"] is False
            and state["counters"]["received"] == old["counters"]["received"]+1
            and state["counters"]["rejected"] == old["counters"]["rejected"]+1,
            "Parallel duplicate sends accepted or rejected the wrong number of commands")
    winner = accepted[0]
    require(matches_identity(state, tickets[winner]), "The pending control identity belongs to the rejected attempt")
    close(state["target_requested_m"], targets[winner], "Only the accepted attempt installs a pending target")
    transaction = {"ticket": {key: tickets[winner][key] for key in IDENTITY_FIELDS}, "ticket_path": str(paths[winner]),
                   "raw_client_result": results[winner]}
    steps = bounded_steps(session, 20, time.monotonic()+30)
    applied = paused_evidence(session)
    require(applied["rpc"]["message"]["counters"]["applied"] == old["counters"]["applied"]+1,
            "Concurrent duplicate logical command was physically applied more than once")
    transaction = capture_applied_wire(session, transaction, targets[winner])
    session.pause(False)
    finished = finish_ticket(session, transaction)
    session.pause(True)
    endpoint = native_entity(session)
    close(endpoint["independent_geometry"]["native_joint_q_m"], targets[winner],
          "Actual door endpoint matches the winner, not the rejected concurrent target", POSITION_TOLERANCE)
    opened = actual_transaction(session, finished, targets[winner])
    session.pause(False)
    closed = send(session, identity(session, logical["epoch"], "parallel-sequence", 2), 0.)
    session.pause(True)
    return {"name": "two_actual_concurrent_sends_one_logical_command_distinct_attempts", "passed": True,
            "parallel_client_results": results, "logical_identity": logical, "targets_m": targets,
            "accepted_attempt": tickets[winner]["attempt_nonce"], "actual_foreign_attempt_ACKs_ignored": foreign,
            "before": before, "pending_after_both_sends": pending, "owned_step_acknowledgement": steps,
            "only_one_physical_application_evidence": applied, "actual_winner_open": opened,
            "actual_close": actual_transaction(session, closed, 0.),
            "legacy_race_physically_reproduced": False,
            "scope": "real new-protocol concurrent attempts; legacy ordering remains separate pure analysis"}


def private_input_errors(session):
    session.pause(True)
    parameter, world = session.workspace/"parameters.json", session.workspace/"world.sdf"
    parameter_bytes, world_bytes = parameter.read_bytes(), world.read_bytes()
    cases = []
    for name, value, diagnosis in (("invalid_saved_nonfinite_target", float("nan"), "open_target_m"),
                                    ("valid_target_saved_without_reload", .5, "重载")):
        try:
            write_parameters(session.workspace, value)
            cases.append(verify_case(session, name, diagnosis))
        finally:
            parameter.write_bytes(parameter_bytes)
    try:
        tree = ET.parse(str(world))
        tree.getroot().find("world/model[@name='transport_gate']/joint[@name='slide_joint']/child").text = "missing_panel"
        tree.write(str(world), encoding="utf-8", xml_declaration=True)
        cases.append(verify_case(session, "saved_missing_panel_topology", "world.sdf"))
    finally:
        world.write_bytes(world_bytes)
    require(parameter.read_bytes() == parameter_bytes and world.read_bytes() == world_bytes,
            "Personal repair failed to restore the exact accepted file bytes")
    return {"name": "invalid_and_stale_private_inputs_refused_before_transport_control", "passed": True,
            "checks": cases, "original_parameter_and_world_bytes_restored": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    parser.add_argument("--client-binary", help="Newly compiled client for --protocol-only; no real Master is contacted")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--math-only", action="store_true")
    mode.add_argument("--configuration-only", action="store_true")
    mode.add_argument("--protocol-only", action="store_true")
    args = parser.parse_args()
    report = {"passed": False, "lesson": KEY, "physical_sessions_started": 0, "scenarios": [],
              "math_regressions": math_regressions()}
    output = Path(args.report).expanduser().resolve() if args.report else None
    artifacts = output.with_name(output.stem+"-artifacts") if output else Path(tempfile.mkdtemp(prefix="gazeboteach-12-evidence-"))
    sessions, lesson, source_before = [], None, None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-12-private-") as folder:
        private = Path(folder)
        try:
            if not args.math_only:
                lessons, errors = load_catalog(ROOT)
                candidates = [candidate for candidate in lessons if candidate.key == KEY]
                require(len(candidates) == 1, "Expected one current L12 course: "+repr(errors))
                lesson = candidates[0]
                source_before = source_hashes(lesson.folder)
                report["configuration_regressions"] = configuration_regressions(lesson, private)
                report["python_pre_network_query_guard"] = python_query_guard_regressions(lesson)
            if args.protocol_only:
                require(args.client_binary, "--protocol-only requires the new --client-binary")
                report["native_pre_network_query_guard"] = native_query_guard_regressions(args.client_binary)
            if not args.math_only and not args.configuration_only and not args.protocol_only:
                a = LessonSession(ROOT, lesson, storage=Storage(private/"A-default"))
                sessions.append(("A-default", a))
                report["physical_sessions_started"] += 1
                a.start(gui=False, terminals=False)
                report["initial_A"] = await_ready(a)
                report["native_pre_network_query_guard"] = native_query_guard_regressions(a.workspace/"build/transport_client", a.env)
                storage = Storage(private/"B-changed-target")
                write_parameters(storage.prepare(lesson), .4)
                b = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("B-changed-target", b))
                report["physical_sessions_started"] += 1
                b.start(gui=False, terminals=False)
                report["initial_B"] = await_ready(b)
                report["actual_type_introspection"] = {"A": introspection(a), "B": introspection(b)}
                report["scenarios"].append(normal_and_isolation(a, b))
                b.pause(True)
                report["scenarios"].append(verify_case(b, "only_open_target_changed_to_point4_three_real_transactions"))
                report["scenarios"].append(paused_delivery_and_latching(b))
                report["scenarios"].append(safe_protocol_errors(b))
                report["scenarios"].append(pending_reset(b))
                report["scenarios"].append(concurrent_logical_duplicate(b))
                report["scenarios"].append(private_input_errors(b))
                report["scenarios"].append(verify_case(b, "repaired_current_personal_bytes_reaccepted_by_real_three_transaction_verification"))
                report["actual_single_parameter_endpoints_m"] = {
                    "default_A": report["scenarios"][0]["actual_open"]["independent_actual_motion"]["actual_final_q_m"],
                    "changed_B": report["scenarios"][1]["independent_actual_transactions"][1]["actual_physics"]["actual_final_q_m"]}
                require(report["actual_single_parameter_endpoints_m"]["default_A"]
                        -report["actual_single_parameter_endpoints_m"]["changed_B"] > .18,
                        "Changing the actual private target did not change the physical open endpoint")
                counts = {}
                for name, session in sessions:
                    keys = {tuple(row[field] for field in TRACE_IDENTITY_FIELDS)
                            for row in read_trace(session)}
                    counts[name] = len(keys)
                require(counts == {"A-default": 2, "B-changed-target": 12},
                        "Formal coverage ran an unexpected number of actual physical transactions: "+repr(counts))
                report["actual_physical_transaction_counts"] = counts
                report["closed_preparation_transactions"] = 2
                report["received_but_cancelled_before_physics_transactions"] = 1
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"] = {}
            report["cleanup_errors"] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L12 private acceptance finished")
                except Exception as error:
                    report["cleanup_errors"].append(name+" stop: "+repr(error))
                try:
                    report["evidence"][name] = preserve_evidence(session, artifacts/name)
                except Exception as error:
                    report["cleanup_errors"].append(name+" evidence: "+repr(error))
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers,
                          source_templates_unchanged=(source_before == source_hashes(lesson.folder)) if lesson else None,
                          scenario_count=len(report["scenarios"]), elapsed_seconds=round(time.monotonic()-started, 3),
                          simultaneous_private_masters=len(sessions) == 2,
                          offline_reanalysis_is_not_new_physical_acceptance=True)
            if any(leftovers.values()) or report["source_templates_unchanged"] is False or report["cleanup_errors"]:
                report["passed"] = False
            if output:
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
