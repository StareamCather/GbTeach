#!/usr/bin/env python3
"""Verify actual Gazebo→JointState→TF data, clock control and one persistent listener."""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid
from config import JOINTS, prepared
from lab import Client, POSES, angle_error, cancellation, cleanup, require


def wait(condition, seconds, reason):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(.005)
    raise RuntimeError("Bounded wait failed: "+reason)


def joint_rows(graph, marker):
    return [row for row in graph.since(marker) if row["source"] == "joint_state"]


def inspect_joint(row):
    data = row["data"]
    require(data["name"] == list(JOINTS) and len(data["position"]) == len(data["velocity"]) == 2,
        "Actual stock JointState names/array lengths differ")
    require(data["effort"] == [] and data["header_frame_id"] == "", "Stock JointState effort/frame contract differs")
    require(all(math.isfinite(value) for value in data["position"]+data["velocity"]), "Actual JointState has nonfinite values")
    require(row["callerid"] == "/gazebo", "JointState is not from the actual Gazebo node")


def error(actual, expected):
    return {"position_m": max(abs(a-b) for a, b in zip(actual["position"], expected["position"])),
        "rotation_rad": angle_error(actual["orientation"], expected["orientation"])}


def pose_phase(client, name):
    requested = POSES[name]
    client.initialize(requested)
    client.refresh()
    snapshot = client.snapshot()
    latest = client.graph.latest("joint_state")
    inspect_joint(latest)
    actual_q = [snapshot["joint_readback"][joint]["position"][0] for joint in JOINTS]
    require(max(abs(a-b) for a, b in zip(actual_q, requested)) < 1e-6, "Actual initialized joint pose differs")
    require(max(abs(a-b) for a, b in zip(latest["data"]["position"], actual_q)) < 1e-6, "Actual JointState is stale relative to paused joints")
    lookups = {link: client.graph.lookup(link) for link in snapshot["link_readback"]}
    errors = {link: error(lookups[link], snapshot["link_readback"][link]["link_state"]) for link in lookups}
    relative = client.graph.lookup("tool_link", target="base_link")
    relative_actual = client.link("tool_link", "tf_arm::base_link")
    errors["relative_tool"] = error(relative, relative_actual["link_state"])
    require(all(value["position_m"] < 1e-5 and value["rotation_rad"] < 1e-5 for value in errors.values()),
        "Actual TF and actual Gazebo link frames differ at a paused stable boundary")
    return {"name": name, "requested_q": requested, "snapshot": snapshot, "lookups": lookups,
        "relative_lookup": relative, "relative_actual": relative_actual, "latest_joint_state": latest, "errors": errors}


def stream_phase(client):
    client.initialize([0, 0])
    marker = client.graph.mark()
    before = client.world_time_ns()
    rate = client.config["joint_state_rate_hz"]
    client.pause(False)
    try:
        def enough():
            rows = joint_rows(client.graph, marker)
            return rows if len(rows) >= rate+1 and (rows[-1]["stamp_ns"]-rows[0]["stamp_ns"])/1e9 >= 1 else None
        wait(enough, 8, "actual JointState sampling window")
    finally:
        client.pause(True)
    time.sleep(.08)
    rows = joint_rows(client.graph, marker)
    for row in rows:
        inspect_joint(row)
    stamps = [row["stamp_ns"] for row in rows]
    require(all(b > a for a, b in zip(stamps, stamps[1:])), "Nominal JointState stamps are not strictly increasing")
    span = (stamps[-1]-stamps[0])/1e9
    actual_rate = (len(stamps)-1)/span
    require(.95*rate <= actual_rate <= 1.05*rate, "Actual JointState ROS-stamp-window rate differs from nominal ±5%")
    events = client.graph.since(marker)
    transforms = [row for row in events if row["source"] == "tf" and row["data"]["transforms"] and
        all(stamps[0] <= value["stamp_ns"] <= stamps[-1] for value in row["data"]["transforms"])]
    require(len(transforms) >= rate//2, "Insufficient actual moving TF observations")
    by_stamp = {row["stamp_ns"]: row for row in rows}
    expected_pairs = {(client.graph.frame("base_link"), client.graph.frame("arm1")), (client.graph.frame("arm1"), client.graph.frame("arm2"))}
    for row in transforms:
        require(row["callerid"] == "/lesson24/robot_state_publisher", "Actual TF authority differs")
        require({(value["parent"], value["child"]) for value in row["data"]["transforms"]} == expected_pairs, "Actual moving TF edges differ")
        for value in row["data"]["transforms"]:
            require(value["stamp_ns"] in by_stamp, "Actual moving TF stamp has no corresponding observed JointState")
    return {"epoch_event_id": marker, "event_ids": [row["event_id"] for row in events],
        "joint_state_event_ids": [row["event_id"] for row in rows], "tf_event_ids": [row["event_id"] for row in transforms],
        "statistics": {"count": len(stamps), "span_s": span, "rate_hz": actual_rate},
        "world_before_ns": before, "world_after_ns": client.world_time_ns(),
        "note": "Rate uses the actual stock ROS-clock stamps, not a claim of native capture timestamps or host receipt rate."}


def late_static(client):
    from tf2_msgs.msg import TFMessage
    received, result = threading.Event(), {}
    subscribed = time.monotonic_ns()
    def receive(message):
        if not received.is_set():
            result.update(row=client.graph.transforms("tf_static", message), received_monotonic_ns=time.monotonic_ns())
            received.set()
    subscription = client.rospy.Subscriber("/tf_static", TFMessage, receive, queue_size=1)
    try:
        require(received.wait(3), "Late static subscriber did not receive actual latched transforms while paused")
    finally:
        subscription.unregister()
    row = result["row"]
    require(row["latching"] == "1" and row["callerid"] == "/lesson24/robot_state_publisher", "Actual static connection is not latched RSP")
    expected = {(client.graph.frame("world"), client.graph.frame("base_link")), (client.graph.frame("arm2"), client.graph.frame("tool_link"))}
    require({(value["parent"], value["child"]) for value in row["data"]["transforms"]} == expected, "Actual static frame edges differ")
    return {"subscribed_monotonic_ns": subscribed, **result, "latching": row["latching"], "callerid": row["callerid"]}


def restore(client, original):
    result = {"before": original, "errors": [], "passed": False}
    with cleanup():
        try:
            client.pause(True)
            for joint in JOINTS:
                try:
                    from gazebo_msgs.srv import JointRequest
                    client.call("/gazebo/clear_joint_forces", JointRequest, joint_name=joint)
                except Exception as exception:
                    result["errors"].append("clear "+joint+": "+str(exception))
            try:
                client.set_q([original["joint_readback"][joint]["position"][0] for joint in JOINTS])
            except Exception as exception:
                result["errors"].append("joint pose: "+str(exception))
            for link, item in original["link_readback"].items():
                try:
                    state = dict(item["link_state"])
                    state.update(linear=[0, 0, 0], angular=[0, 0, 0])
                    client.set_link(link, state)
                except Exception as exception:
                    result["errors"].append("temporary stable pose "+link+": "+str(exception))
            try:
                result["graph_refresh"] = client.refresh()
            except Exception as exception:
                result["errors"].append("TF refresh: "+str(exception))
            for link, item in original["link_readback"].items():
                try:
                    client.set_link(link, item["link_state"])
                except Exception as exception:
                    result["errors"].append("original twist/pose "+link+": "+str(exception))
            result["after"] = client.snapshot()
            differences = {}
            for link in original["link_readback"]:
                before = original["link_readback"][link]["link_state"]
                after = result["after"]["link_readback"][link]["link_state"]
                differences[link] = {**error(after, before), "linear_m_s": max(abs(a-b) for a, b in zip(after["linear"], before["linear"])),
                    "angular_rad_s": max(abs(a-b) for a, b in zip(after["angular"], before["angular"]))}
            result["link_errors"] = differences
            require(all(max(value.values()) < 1e-6 for value in differences.values()), "Restored actual link pose/twist differs")
            for joint in JOINTS:
                for field in ("position", "rate"):
                    require(abs(result["after"]["joint_readback"][joint][field][0]-original["joint_readback"][joint][field][0]) < 1e-5,
                        "Restored actual joint "+field+" differs")
        except Exception as exception:
            result["errors"].append(str(exception))
        finally:
            try:
                client.pause(original["paused"])
                result["pause_after"] = client.physics().pause
                require(result["pause_after"] == original["paused"], "Original actual pause state was not restored")
            except Exception as exception:
                result["errors"].append("pause: "+str(exception))
    result["passed"] = not result["errors"]
    result["note"] = "Physical entry state and pause restored; simulation clock, queued tasks and TF history are not rewound/recreated."
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report")
    args = parser.parse_args()
    path = Path(args.report) if args.report else Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", "."))/("tf-clock-verify-"+uuid.uuid4().hex+".json")
    result = {"passed": False, "trace": [], "pose_phases": [], "step_phases": [], "report_path": str(path.resolve())}
    client, original, paused = None, None, None
    start = time.monotonic()
    try:
        with cancellation():
            config, record = prepared()
            result.update(config=config, settings_sha256=record["settings_sha256"], world_sha256=record["world_sha256"],
                source_sha256=record["source_sha256"], generated_urdf_path=record["generated_urdf_path"], generated_urdf_sha256=record["generated_urdf_sha256"],
                converted_sdf_path=record["converted_sdf_path"], converted_sdf_sha256=record["converted_sdf_sha256"])
            client = Client(config, record)
            result["readiness"] = client.ready(timeout=3)
            result["joint_name_uniqueness"] = client.assert_unique_joints()
            paused = client.physics().pause
            client.pause(True)
            original = client.snapshot()
            original["paused"] = paused
            try:
                result["stream"] = stream_phase(client)
                for name in POSES:
                    result["pose_phases"].append(pose_phase(client, name))
                before = client.clock_snapshot()
                time.sleep(.15)
                after = client.clock_snapshot()
                require(after["world_time_ns"] == before["world_time_ns"] and after["ros_time_ns"] == before["ros_time_ns"], "Actual paused clock advanced")
                require(after["clock_event"]["stamp_ns"] == before["clock_event"]["stamp_ns"], "Actual observed paused clock changed")
                result["pause_phase"] = {"before": before, "after": after, "events": client.graph.since(before["event_id"])}
                for name, steps in (("single_step", 1), ("multi_step", 100)):
                    before = client.clock_snapshot()
                    command = client.step(steps)
                    after = client.clock_snapshot()
                    require(after["world_time_ns"]-before["world_time_ns"] == steps*1000000, "Actual stepped time delta differs")
                    require(abs((after["clock_event"]["stamp_ns"]-before["clock_event"]["stamp_ns"])-steps*1000000) <= 1000000,
                        "Actual /clock step differs beyond one physics tick of callback timing")
                    result["step_phases"].append({"name": name, "steps": steps, "command": command, "before": before, "after": after,
                        "events": client.graph.since(before["event_id"])})
                result["late_static_phase"] = late_static(client)
                missing = "lesson24/base_link"
                try:
                    client.graph.buffer.lookup_transform(client.graph.frame("world"), missing, client.rospy.Time(0), client.rospy.Duration(0))
                except Exception as exception:
                    result["wrong_frame_probe"] = {"queried_frame": missing, "error": type(exception).__name__+": "+str(exception)}
                require("wrong_frame_probe" in result, "A ROS namespace was incorrectly treated as an existing TF frame")
                before = client.clock_snapshot()
                listener, buffer = id(client.graph.listener), id(client.graph.buffer)
                clears = len(client.graph.clear_events)
                client.reset_time()
                post = client.refresh()
                after = client.clock_snapshot()
                events = client.graph.since(before["event_id"])
                clocks = [before["clock_event"]]+[row for row in events if row["source"] == "clock"]
                require(any(b["stamp_ns"] < a["stamp_ns"] for a, b in zip(clocks, clocks[1:])), "Actual /clock backward jump was not observed")
                require(len(client.graph.clear_events) > clears, "Persistent stock TF listener did not clear its dynamic cache after ROS time regression")
                require(listener == id(client.graph.listener) and buffer == id(client.graph.buffer), "Listener/buffer was rebuilt instead of observing reset")
                require(post["lookups"]["tool_link"]["stamp_ns"] < before["clock_event"]["stamp_ns"], "TF lookup still uses the pre-reset dynamic time")
                result["reset_phase"] = {"before": before, "after": after, "events": events,
                    "listener_identity_before": listener, "listener_identity_after": id(client.graph.listener),
                    "buffer_identity_before": buffer, "buffer_identity_after": id(client.graph.buffer),
                    "clear_count_before": clears, "clear_count_after": len(client.graph.clear_events),
                    "clear_events": list(client.graph.clear_events), "post_ready": post}
                result["passed"] = True
            finally:
                result["restore"] = restore(client, original)
                result["trace"] = client.graph.since(0)
                if not result["restore"]["passed"]:
                    result["passed"] = False
                    result["restore_error"] = result["restore"]["errors"]
    except BaseException as exception:
        result["passed"] = False
        result["error"] = type(exception).__name__+": "+str(exception)
    finally:
        if client:
            if original is None and paused is not None:
                with cleanup():
                    try:
                        client.pause(paused)
                    except Exception as exception:
                        result["restore_error"] = "Original snapshot failed; pause restore: "+str(exception)
            client.close()
    result["wall_seconds"] = time.monotonic()-start
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
