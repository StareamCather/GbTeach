#!/usr/bin/env python3
"""Query actual spawned URDF links, joints and physical properties."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid
import xmlrpc.client as rpc
from config import JOINTS, PACKAGE, edit, prepared

POSES = {"zero": [0, 0], "positive_mixed": [math.pi/6, -math.pi/4], "negative_mixed": [-.4, .6]}
_cancelled = threading.Event()
_cleanup_depth = 0


@contextmanager
def cancellation():
    handlers = {}
    _cancelled.clear()
    def stop(number, frame):
        if not _cancelled.is_set():
            _cancelled.set()
            if not _cleanup_depth:
                raise RuntimeError("Cancelled by signal "+str(number))
    try:
        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[number] = signal.signal(number, stop)
        yield
        require(not _cancelled.is_set(), "Cancelled during cleanup")
    finally:
        for number, handler in handlers.items():
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


def quaternion(rpy):
    r, p, y = [value/2 for value in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def angle_error(a, b):
    norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    require(norm > 0, "Invalid quaternion")
    return 2*math.acos(min(1, max(0, abs(sum(x*y for x, y in zip(a, b)))/norm)))


class Transport(rpc.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.5
        return connection


def master(method, *args):
    with rpc.ServerProxy(os.environ["ROS_MASTER_URI"], transport=Transport()) as proxy:
        code, message, value = getattr(proxy, method)("/gazeboteach_l24_inspector", *args)
        require(code == 1, message)
        return value


class Client:
    def __init__(self, config, record):
        import rospy
        from std_msgs.msg import String
        if not rospy.core.is_initialized():
            rospy.init_node("lesson24_client", anonymous=True, disable_signals=True)
        self.rospy, self.config, self.record = rospy, config, record
        sys.path.insert(0, str(Path.cwd()/PACKAGE/"scripts"))
        import common
        self.common = common
        self.lock, self.status, self.errors = threading.Lock(), None, []
        self.subscription = rospy.Subscriber("/lesson24/description_status", String, self.receive, queue_size=1)
        from graph import Graph
        self.graph = Graph(config)

    def receive(self, message):
        try:
            value = json.loads(message.data)
            with self.lock:
                self.status = value
        except Exception as error:
            with self.lock:
                self.errors.append(str(error))

    def call(self, name, service, *args, **kwargs):
        return self.common.call(name, service, *args, **kwargs)

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, paused):
        from std_srvs.srv import Empty
        self.call("/gazebo/pause_physics" if paused else "/gazebo/unpause_physics", Empty)

    def world_time_ns(self):
        from gazebo_msgs.srv import GetWorldProperties
        return round(self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time*1e9)

    def model_properties(self):
        from gazebo_msgs.srv import GetModelProperties
        value = self.call("/gazebo/get_model_properties", GetModelProperties, model_name="tf_arm")
        return {"body_names": list(value.body_names), "joint_names": list(value.joint_names),
                "geom_names": list(value.geom_names), "is_static": value.is_static}

    def link_properties(self, name):
        from gazebo_msgs.srv import GetLinkProperties
        value = self.call("/gazebo/get_link_properties", GetLinkProperties, link_name="tf_arm::"+name)
        return {"mass": value.mass, "gravity_mode": value.gravity_mode,
            "com": {"position": [value.com.position.x, value.com.position.y, value.com.position.z],
                    "orientation": [value.com.orientation.x, value.com.orientation.y, value.com.orientation.z, value.com.orientation.w]},
            "inertia": {key: getattr(value, key) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}}

    def link(self, name, reference="world"):
        from gazebo_msgs.srv import GetLinkState
        host_before = time.monotonic_ns()
        before = self.world_time_ns()
        value = self.call("/gazebo/get_link_state", GetLinkState, link_name="tf_arm::"+name, reference_frame=reference)
        after = self.world_time_ns()
        return {"world_time_before_ns": before, "world_time_after_ns": after,
            "host_monotonic_before_ns": host_before, "host_monotonic_after_ns": time.monotonic_ns(),
            "link_state": self.common.pose_twist(value.link_state.pose, value.link_state.twist),
            "link_name": value.link_state.link_name, "reference_frame": value.link_state.reference_frame}

    def joint(self, name):
        from gazebo_msgs.srv import GetJointProperties
        value = self.call("/gazebo/get_joint_properties", GetJointProperties, joint_name="tf_arm::"+name)
        require(len(value.position) == 1 and len(value.rate) == 1 and all(math.isfinite(x) for x in list(value.position)+list(value.rate)), "Moving joint query returned nonfinite/unsupported position or rate")
        return {"position": list(value.position), "rate": list(value.rate), "reported_type": value.type,
                "reported_damping": list(value.damping), "note": "API reported type/damping are not used to prove actual joint type or damping."}

    def ready(self, timeout=25):
        deadline, last_error = time.monotonic()+timeout, ""
        while time.monotonic() < deadline:
            try:
                require(master("getParam", "/use_sim_time") is True, "Private /use_sim_time must be true")
                for name, expected_value in (("tf_prefix", self.config["tf_prefix"]), ("use_tf_static", True), ("ignore_timestamp", False), ("publish_frequency", 50.0)):
                    require(master("getParam", "/lesson24/robot_state_publisher/"+name) == expected_value, "Actual RSP "+name+" parameter differs; reload required")
                with self.lock:
                    require(not self.errors, "; ".join(self.errors))
                    status = self.status
                require(status is not None, "Waiting for actual description-info node")
                require(status["session_token"] == os.environ.get("GAZEBOTEACH_SESSION_TOKEN") and status["session_token"], "Description node belongs to another session")
                require(status["master_uri"] == os.environ["ROS_MASTER_URI"] and Path(status["workspace"]).resolve() == Path.cwd().resolve(), "Description node belongs to another Master/workspace")
                require(status["source_sha256"] == self.record["source_sha256"] and status["settings_sha256"] == self.record["settings_sha256"], "Running description input differs; reload required")
                description = master("getParam", "/lesson24/robot_description")
                require(hashlib.sha256(description.encode("utf-8")).hexdigest() == self.record["generated_urdf_sha256"] == status["loaded_description_sha256"], "Actual robot_description differs from generated URDF")
                model = self.model_properties()
                require(not model["is_static"], "Whole arm model is static; articulated physics would be disabled")
                expected = self.record["sdf_link_properties"]
                require(set(model["body_names"]) == set(expected), "Actual physical links differ from native converter/preserveFixedJoint")
                require(set(JOINTS) <= set(model["joint_names"]), "Actual two moving joints are missing")
                actual = {name: self.link_properties(name) for name in expected}
                for name, value in actual.items():
                    target = expected[name]
                    require(value["gravity_mode"] is False, "Actual gravity_mode differs on "+name)
                    require(abs(value["mass"]-target["mass"]) < 1e-8, "Actual mass differs on "+name)
                    require(max(abs(a-b) for a, b in zip(value["com"]["position"], target["com_pose"][:3])) < 1e-8, "Actual COM differs on "+name)
                    require(angle_error(value["com"]["orientation"], quaternion(target["com_pose"][3:])) < 1e-6, "Actual inertial orientation differs on "+name)
                    require(max(abs(value["inertia"][key]-target["inertia"][key]) for key in value["inertia"]) < 1e-9, "Actual inertia tensor differs on "+name)
                publishers = dict(master("getSystemState")[0])
                require(publishers.get("/lesson24/joint_states") == ["/gazebo"], "Actual JointState publisher is not the unique Gazebo node")
                require(publishers.get("/tf") == ["/lesson24/robot_state_publisher"] and publishers.get("/tf_static") == ["/lesson24/robot_state_publisher"], "Actual TF publishers differ from the unique RSP node")
                graph = {"deferred": True, "reason": "Paused world: graph authorities checked, fresh dynamic channels require controlled clock advancement"} if self.physics().pause else self.graph.ready(min(15, max(.1, deadline-time.monotonic())))
                return {"status": status, "model": model, "actual_link_properties": actual, "graph": graph,
                    "joint_readback": {name: self.joint(name) for name in JOINTS},
                    "link_readback": {name: self.link(name) for name in expected},
                    "generated_urdf_path": self.record["generated_urdf_path"], "converted_sdf_path": self.record["converted_sdf_path"]}
            except Exception as error:
                last_error = type(error).__name__+": "+str(error)
            time.sleep(.03)
        raise RuntimeError("Bounded URDF spawn/property readiness failed: "+last_error)

    def assert_unique_joints(self):
        from gazebo_msgs.srv import GetModelProperties, GetWorldProperties
        models = self.call("/gazebo/get_world_properties", GetWorldProperties).model_names
        evidence = {}
        for model in models:
            value = self.call("/gazebo/get_model_properties", GetModelProperties, model_name=model)
            evidence[model] = list(value.joint_names)
            if model != "tf_arm":
                require(not set(JOINTS) & {name.rsplit("::", 1)[-1] for name in value.joint_names},
                    "A raw joint basename is shared by another model; scoped force cleanup would not be safe")
        return evidence

    def clear(self):
        from gazebo_msgs.srv import JointRequest
        for name in JOINTS:
            self.call("/gazebo/clear_joint_forces", JointRequest, joint_name=name)
        return {"raw_joint_names": list(JOINTS), "response_is_measurement_proof": False}

    def set_q(self, values):
        from gazebo_msgs.srv import SetModelConfiguration
        self.call("/gazebo/set_model_configuration", SetModelConfiguration, model_name="tf_arm",
            urdf_param_name="/lesson24/robot_description", joint_names=list(JOINTS), joint_positions=list(values))

    def set_link(self, name, state):
        from gazebo_msgs.msg import LinkState
        from gazebo_msgs.srv import SetLinkState
        value = LinkState()
        value.link_name, value.reference_frame = "tf_arm::"+name, "world"
        p, q, v, w = state["position"], state["orientation"], state["linear"], state["angular"]
        # GetLinkState reports velocity at the link origin. This Classic ODE
        # SetLinkState path calls SetLinearVel at the CoM, so the two APIs are
        # not direct inverses when the CoM is offset and angular velocity !=0.
        com = self.link_properties(name)["com"]["position"]
        norm = math.sqrt(sum(component*component for component in q))
        require(math.isfinite(norm) and norm > 0, "Invalid link orientation for velocity restoration")
        x, y, z, scalar = [component/norm for component in q]
        cx, cy, cz = com
        offset = [(1-2*(y*y+z*z))*cx+2*(x*y-z*scalar)*cy+2*(x*z+y*scalar)*cz,
                  2*(x*y+z*scalar)*cx+(1-2*(x*x+z*z))*cy+2*(y*z-x*scalar)*cz,
                  2*(x*z-y*scalar)*cx+2*(y*z+x*scalar)*cy+(1-2*(x*x+y*y))*cz]
        correction = [w[1]*offset[2]-w[2]*offset[1], w[2]*offset[0]-w[0]*offset[2], w[0]*offset[1]-w[1]*offset[0]]
        setter_velocity = [a+b for a, b in zip(v, correction)]
        value.pose.position.x, value.pose.position.y, value.pose.position.z = p
        value.pose.orientation.x, value.pose.orientation.y, value.pose.orientation.z, value.pose.orientation.w = q
        value.twist.linear.x, value.twist.linear.y, value.twist.linear.z = setter_velocity
        value.twist.angular.x, value.twist.angular.y, value.twist.angular.z = w
        self.call("/gazebo/set_link_state", SetLinkState, link_state=value)
        return {"link_name": value.link_name, "reference_frame": "world", "actual_com_local": com,
                "rotated_com_offset_world": offset, "desired_link_origin_linear": list(v),
                "angular_world": list(w), "setter_com_linear": setter_velocity}

    def initialize(self, q):
        require(self.physics().pause, "Joint/velocity initialization requires a paused boundary")
        self.clear()
        self.set_q(q)
        for name in self.record["sdf_link_properties"]:
            value = self.link(name)["link_state"]
            value.update(linear=[0, 0, 0], angular=[0, 0, 0])
            self.set_link(name, value)

    def sample(self):
        host = time.monotonic_ns()
        before = self.world_time_ns()
        joints = {name: self.joint(name) for name in JOINTS}
        after = self.world_time_ns()
        return {"world_time_before_ns": before, "world_time_after_ns": after,
            "host_monotonic_before_ns": host, "host_monotonic_after_ns": time.monotonic_ns(), "joint_readback": joints}

    def snapshot(self):
        return {"paused": self.physics().pause, "world_time_ns": self.world_time_ns(),
            "joint_readback": {name: self.joint(name) for name in JOINTS},
            "link_readback": {name: self.link(name) for name in self.record["sdf_link_properties"]}}

    def clock_snapshot(self):
        return {"world_time_ns": self.world_time_ns(), "ros_time_ns": self.rospy.Time.now().to_nsec(),
            "clock_event": self.graph.latest("clock"), "event_id": self.graph.mark(), "host_monotonic_ns": time.monotonic_ns()}

    def step(self, steps):
        require(type(steps) is int and 1 <= steps <= 1000, "Use 1..1000 actual physics steps")
        require(self.physics().pause, "Step requires an actual paused boundary")
        before = self.world_time_ns()
        command = ["gz", "world", "-w", "lesson24", "-m", str(steps)]
        process = subprocess.run(command, capture_output=True, text=True, timeout=6, check=True)
        expected = before+steps*1000000
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            after = self.world_time_ns()
            if after >= expected:
                require(abs(after-expected) <= 2, "Actual world step delta differs from physics iterations")
                time.sleep(.06)
                return {"argv": command, "stdout": process.stdout, "stderr": process.stderr,
                    "before_world_time_ns": before, "after_world_time_ns": after, "steps": steps}
            time.sleep(.005)
        raise RuntimeError("Bounded actual physics-step acknowledgement timed out")

    def refresh(self):
        self.step(150)
        return self.graph.ready(5)

    def reset_time(self):
        from std_srvs.srv import Empty
        require(self.physics().pause, "Reset simulation requires a paused boundary")
        self.clear()
        self.call("/gazebo/reset_simulation", Empty)
        require(self.world_time_ns() <= 1000000, "Actual simulation time did not reset")

    def effort(self, effort, duration):
        from gazebo_msgs.srv import ApplyJointEffort
        now = self.world_time_ns()
        start = now+50000000
        self.call("/gazebo/apply_joint_effort", ApplyJointEffort, joint_name="tf_arm::lesson24_elbow", effort=effort,
            start_time=self.rospy.Time(start//1000000000, start%1000000000), duration=self.rospy.Duration.from_sec(duration))
        return {"joint_name": "tf_arm::lesson24_elbow", "raw_clear_name": "lesson24_elbow", "effort_nm": effort,
            "duration_ns": round(duration*1e9), "scheduling_world_time_ns": now,
            "requested_start_time_ns": start, "requested_end_time_ns": start+round(duration*1e9)}

    def close(self):
        self.subscription.unregister()
        self.graph.close()


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ready")
    sub.add_parser("reset")
    sub.add_parser("clock-reset")
    p = sub.add_parser("step")
    p.add_argument("--steps", type=int, default=1)
    p = sub.add_parser("pose")
    p.add_argument("--name", choices=tuple(POSES), default="positive_mixed")
    p = sub.add_parser("pulse")
    p.add_argument("--sign", type=int, choices=(-1, 1), default=1)
    p = sub.add_parser("observe")
    p.add_argument("--seconds", type=float, default=2)
    p = sub.add_parser("config")
    p.add_argument("--joint-state-rate", type=int, choices=(20, 40))
    p.add_argument("--tf-prefix", choices=("", "demo"))
    args = parser.parse_args()
    result = {"passed": False}
    started = time.monotonic()
    try:
        if args.command == "config":
            result.update(edit(args.joint_state_rate, args.tf_prefix))
        else:
            config, record = prepared()
            client = Client(config, record)
            try:
                result["readiness"] = client.ready()
                if args.command in ("pose", "reset", "pulse", "clock-reset"):
                    result["joint_name_uniqueness"] = client.assert_unique_joints()
                if args.command in ("pose", "reset"):
                    paused = client.physics().pause
                    try:
                        client.pause(True)
                        client.initialize(POSES[args.name] if args.command == "pose" else [0, 0])
                        result["graph_after_advancement"] = client.refresh()
                        result["snapshot"] = client.snapshot()
                    finally:
                        client.pause(paused)
                elif args.command == "pulse":
                    result["command"] = client.effort(args.sign*.03, .2)
                elif args.command == "step":
                    result["step"] = client.step(args.steps)
                    result["clock"] = client.clock_snapshot()
                elif args.command == "clock-reset":
                    paused = client.physics().pause
                    try:
                        client.pause(True)
                        if any(client.graph.latest(source) is None for source in ("clock", "joint_state", "tf")):
                            # A new listener in a paused world has no old dynamic
                            # epoch. Observe real samples before testing a reset.
                            result["pre_reset_observation"] = client.refresh()
                        result["before"] = client.clock_snapshot()
                        client.reset_time()
                        result["graph_after_reset"] = client.refresh()
                        result["after"] = client.clock_snapshot()
                        result["clear_events"] = list(client.graph.clear_events)
                        result["events"] = client.graph.since(result["before"]["event_id"])
                    finally:
                        client.pause(paused)
                elif args.command == "observe":
                    require(math.isfinite(args.seconds) and .5 <= args.seconds <= 5, "Observation seconds must be within[.5,5]")
                    deadline, values = time.monotonic()+args.seconds, []
                    while time.monotonic() < deadline:
                        values.append(client.sample())
                        time.sleep(.01)
                    result["samples"] = values
            finally:
                client.close()
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-started
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
