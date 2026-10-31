#!/usr/bin/env python3
"""Inspect the real ROS graph and observe actual launch-started node feedback."""
import argparse
from collections import deque
from contextlib import contextmanager
import json
import math
import os
import signal
from pathlib import Path
import sys
import threading
import time
import uuid
import xmlrpc.client as xmlrpc_client
from config import PACKAGE, edit, prepared

_cancelled = threading.Event()
_cleanup_depth = 0


@contextmanager
def cancellation():
    saved = {}
    _cancelled.clear()
    def stop(number, frame):
        if not _cancelled.is_set():
            _cancelled.set()
            if not _cleanup_depth:
                raise RuntimeError("Cancelled by signal "+str(number))
    try:
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            saved[number] = signal.signal(number, stop)
        yield
        require(not _cancelled.is_set(), "Cancelled during cleanup")
    finally:
        for number, handler in saved.items():
            signal.signal(number, handler)


@contextmanager
def cleanup():
    global _cleanup_depth
    _cleanup_depth += 1
    try:
        yield
    finally:
        _cleanup_depth -= 1


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


class Transport(xmlrpc_client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.5
        return connection


def xmlrpc(uri, method, *args):
    with xmlrpc_client.ServerProxy(uri, transport=Transport(), allow_none=True) as proxy:
        code, message, value = getattr(proxy, method)("/gazeboteach_l22_inspector", *args)
        require(code == 1, "ROS XMLRPC "+method+" failed: "+message)
        return value


class Client:
    def __init__(self, config, record):
        import rospy
        from std_msgs.msg import String
        if not rospy.core.is_initialized():
            rospy.init_node("lesson22_client", anonymous=True, disable_signals=True)
        self.rospy, self.config, self.record = rospy, config, record
        sys.path.insert(0, str(Path.cwd()/PACKAGE/"scripts"))
        import common
        self.common = common
        self.created = time.monotonic_ns()
        self.lock, self.rows, self.status, self.errors = threading.Lock(), deque(), {}, []
        self.subscriptions = [rospy.Subscriber(config["namespace"]+"/"+topic, String,
            lambda message, role=role: self.receive(role, message), queue_size=200, buff_size=4194304)
            for role, topic in (("driver", "driver_status"), ("observer", "observer_status"), ("feedback", "state_feedback"))]

    def receive(self, role, message):
        try:
            row = json.loads(message.data)
            row["client_receipt_monotonic_ns"] = time.monotonic_ns()
            with self.lock:
                if role == "feedback":
                    require(row["source_has_header"] is False and row["source_stamp_ns"] is None, "ModelStates must not be assigned a fabricated native stamp")
                    require(len(self.rows) < 10000, "Feedback buffer overflow; no silent dropping")
                    self.rows.append(row)
                else:
                    self.status[role] = row
        except Exception as error:
            with self.lock:
                self.errors.append(type(error).__name__+": "+str(error))

    def drain(self):
        with self.lock:
            require(not self.errors, "; ".join(self.errors))
            rows = list(self.rows)
            self.rows.clear()
            return rows

    def call(self, name, service_type, *args, **kwargs):
        return self.common.call(name, service_type, *args, **kwargs)

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, value):
        from std_srvs.srv import Empty
        self.call("/gazebo/pause_physics" if value else "/gazebo/unpause_physics", Empty)

    def world_time_ns(self):
        from gazebo_msgs.srv import GetWorldProperties
        return round(self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time*1e9)

    def model(self):
        from gazebo_msgs.srv import GetModelState
        return self.call("/gazebo/get_model_state", GetModelState, model_name="launch_slider", relative_entity_name="world")

    def state(self):
        from gazebo_msgs.msg import ModelState
        value = self.model()
        result = ModelState()
        result.model_name, result.reference_frame = "launch_slider", "world"
        result.pose, result.twist = value.pose, value.twist
        return result

    def apply_state(self, state):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, model_state=state)

    def reset_state(self):
        from gazebo_msgs.msg import ModelState
        value = ModelState()
        value.model_name, value.reference_frame = "launch_slider", "world"
        value.pose.position.z, value.pose.orientation.w = 1, 1
        self.apply_state(value)

    def truth(self):
        host_before = time.monotonic_ns()
        before = self.world_time_ns()
        value = self.model()
        after = self.world_time_ns()
        host_after = time.monotonic_ns()
        return {"host_monotonic_before_ns": host_before, "host_monotonic_after_ns": host_after,
            "world_time_before_ns": before, "world_time_after_ns": after,
            "service_header": {"seq": value.header.seq, "stamp_ns": value.header.stamp.to_nsec(), "frame_id": value.header.frame_id},
            "state": self.common.pose_twist(value.pose, value.twist),
            "time_basis": "GetWorldProperties bracket; GetModelState header is the API node ROS-time readback, not a guaranteed native acquisition stamp."}

    def world_ready(self):
        from gazebo_msgs.srv import GetLinkProperties
        body = self.call("/gazebo/get_link_properties", GetLinkProperties, link_name="launch_slider::body")
        require(abs(body.mass-1) < 1e-9 and body.gravity_mode is False, "Actual body mass/gravity differs from the lesson rig")
        require(abs(self.physics().time_step-.001) < 1e-12, "Actual physics timestep differs")
        return {"mass_kg": body.mass, "gravity_mode": body.gravity_mode, "state": self.truth()}

    def graph(self):
        master = os.environ["ROS_MASTER_URI"]
        state = xmlrpc(master, "getSystemState")
        maps = {label: {name: nodes for name, nodes in items} for label, items in zip(("publishers", "subscribers", "services"), state)}
        prefix = self.config["namespace"]
        nodes = {}
        for role in ("driver", "observer"):
            name = prefix+"/"+role
            uri = xmlrpc(master, "lookupNode", name)
            nodes[role] = {"name": name, "uri": uri, "pid": xmlrpc(uri, "getPid")}
        return {"master_uri": master, "master_pid": xmlrpc(master, "getPid"), "nodes": nodes, **maps}

    def ready(self, allow_paused=False):
        prefix, received = self.config["namespace"], []
        deadline, last_error = time.monotonic()+20, ""
        while time.monotonic() < deadline:
            try:
                received.extend(self.drain())
                graph = self.graph()
                require(xmlrpc(os.environ["ROS_MASTER_URI"], "getParam", "/use_sim_time") is True, "Private Master /use_sim_time must be true before starting Gazebo and launch nodes")
                with self.lock:
                    statuses = dict(self.status)
                require(set(statuses) == {"driver", "observer"}, "Waiting for actual latched statuses from both launch nodes")
                for role, status in statuses.items():
                    require(status["node"] == prefix+"/"+role and status["pid"] == graph["nodes"][role]["pid"], "Actual node identity differs from its ROS XMLRPC PID")
                    require(status["session_token"] == os.environ.get("GAZEBOTEACH_SESSION_TOKEN") and status["session_token"], "Node belongs to a different session")
                    require(status["master_uri"] == os.environ["ROS_MASTER_URI"] and Path(status["workspace"]).resolve() == Path.cwd().resolve(), "Node belongs to a different Master/workspace")
                    require(status["source_sha256"] == self.record["source_sha256"] and status["settings_sha256"] == self.record["settings_sha256"], "Running node source/settings differ; reload required")
                driver = prefix+"/driver"
                observer = prefix+"/observer"
                for service in ("start_pulse", "cancel_pulse"):
                    require(driver in graph["services"].get(prefix+"/"+service, []), "Remapped driver service missing: "+service)
                require(observer in graph["subscribers"].get("/gazebo/model_states", []), "Observer input remap/real subscription missing")
                require(observer in graph["publishers"].get(prefix+"/state_feedback", []), "Observer feedback remap missing")
                require("/gazebo" in graph["publishers"].get("/gazebo/model_states", []), "Actual Gazebo ModelStates publisher missing")
                params = xmlrpc(os.environ["ROS_MASTER_URI"], "getParam", driver)
                require(params == {"force_n": float(self.config["force_n"]), "duration_s": .3, "lead_s": .1, "body_name": "launch_slider::body"}, "Actual private driver parameters differ from launch arguments")
                require(statuses["driver"]["parameters"] == params, "Driver cached parameters differ from the current private parameter server")
                observer_params = xmlrpc(os.environ["ROS_MASTER_URI"], "getParam", observer)
                require(observer_params == {"model_name": "launch_slider", "publish_wall_hz": 50.0} and statuses["observer"]["parameters"] == observer_params, "Actual observer private/cached parameters differ")
                expected_names = {"pulse_service": prefix+"/start_pulse", "cancel_service": prefix+"/cancel_pulse", "apply_service": "/gazebo/apply_body_wrench", "clear_service": "/gazebo/clear_body_wrenches", "world_service": "/gazebo/get_world_properties", "status_topic": prefix+"/driver_status"}
                require(statuses["driver"]["resolved_names"] == expected_names, "Driver remap resolution differs")
                for service in ("/gazebo/apply_body_wrench", "/gazebo/clear_body_wrenches", "/gazebo/get_world_properties"):
                    require("/gazebo" in graph["services"].get(service, []), "Existing Gazebo ROS API provider missing: "+service)
                fresh = [row for row in received if row["observer_receipt_monotonic_ns"] >= self.created]
                paused = self.physics().pause
                require(len(fresh) >= 3 or allow_paused and paused, "Waiting for fresh actual feedback after subscription")
                require(all(row["node"] == observer for row in fresh), "Feedback originated in another node")
                require(all(row["model_name"] == "launch_slider" and row["source_type"] == "gazebo_msgs/ModelStates" for row in fresh), "Feedback references another model or ROS type")
                require(all(b["source_message_sequence"] > a["source_message_sequence"] and
                    b["observer_receipt_monotonic_ns"] > a["observer_receipt_monotonic_ns"]
                    for a, b in zip(fresh, fresh[1:])), "Fresh observer events must increase in callback sequence and host receipt time")
                return {"graph": graph, "statuses": statuses, "driver_parameters": params, "fresh_feedback": fresh,
                    "fresh_feedback_count": len(fresh), "paused": paused, "feedback_pending_resume": len(fresh) < 3}
            except Exception as error:
                last_error = type(error).__name__+": "+str(error)
            time.sleep(.03)
        raise RuntimeError("Bounded launch readiness failed: "+last_error)

    def pulse(self):
        from std_srvs.srv import Trigger
        reply = self.call(self.config["namespace"]+"/start_pulse", Trigger)
        require(reply.success, reply.message)
        return json.loads(reply.message)

    def cancel(self):
        from std_srvs.srv import Trigger
        reply = self.call(self.config["namespace"]+"/cancel_pulse", Trigger)
        require(reply.success, reply.message)
        return {"service_response": reply.message, "measurement_proof": False}

    def clear_body(self):
        from gazebo_msgs.srv import BodyRequest
        self.call("/gazebo/clear_body_wrenches", BodyRequest, body_name="launch_slider::body")

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()


def save(path, result):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("world-ready")
    sub.add_parser("ready")
    sub.add_parser("pulse")
    sub.add_parser("cancel")
    sub.add_parser("reset")
    p = sub.add_parser("observe")
    p.add_argument("--seconds", type=float, default=2)
    p.add_argument("--report", type=Path)
    p = sub.add_parser("config")
    p.add_argument("--force", type=int, choices=(2, 4))
    p.add_argument("--namespace", choices=("cart_a", "cart_b"))
    args = parser.parse_args()
    result = {"passed": False}
    started = time.monotonic()
    try:
        if args.command == "config":
            result.update(edit(args.force, args.namespace))
        else:
            config, record = prepared()
            client = Client(config, record)
            try:
                if args.command == "world-ready":
                    result["world"] = client.world_ready()
                else:
                    result["ready"] = client.ready(allow_paused=args.command in ("pulse", "cancel", "reset"))
                    if args.command == "pulse":
                        result["command"] = client.pulse()
                    elif args.command == "cancel":
                        result["cancel"] = client.cancel()
                    elif args.command == "reset":
                        original_pause = client.physics().pause
                        try:
                            client.pause(True)
                            result["cancel"] = client.cancel()
                            client.reset_state()
                            result["state"] = client.truth()
                        finally:
                            client.pause(original_pause)
                    if args.command == "observe":
                        require(math.isfinite(args.seconds) and .5 <= args.seconds <= 5, "Observation seconds must be within[.5,5]")
                        deadline, rows = time.monotonic()+args.seconds, []
                        while time.monotonic() < deadline:
                            rows.extend(client.drain())
                            time.sleep(.01)
                        result["feedback"] = rows
            finally:
                client.close()
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-started
    if getattr(args, "report", None):
        save(args.report, result)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
