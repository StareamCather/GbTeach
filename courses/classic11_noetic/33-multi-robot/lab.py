#!/usr/bin/env python3
"""Finite physical experiments; raw ROS observations and Gazebo service truth."""
import argparse
import copy
import fcntl
import json
import math
from pathlib import Path
import signal
import threading
import time
from config import ROBOTS, identity, prepared


def require(value, message):
    if not value:
        raise RuntimeError(message)


def yaw(q):
    return math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose(message):
    p, q = message.position, message.orientation
    return {"position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w],
            "yaw": yaw([q.x, q.y, q.z, q.w])}


def transform(message):
    p, q = message.transform.translation, message.transform.rotation
    return {"parent": message.header.frame_id, "child": message.child_frame_id,
            "stamp_ns": message.header.stamp.to_nsec(),
            "position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w]}


class Client:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        self.parameters, self.record = prepared()
        self.rospy = rospy
        if not rospy.core.is_initialized():
            rospy.init_node("lesson33_inspector", anonymous=True, disable_signals=True)
        self.lock = threading.Lock()
        self.events, self.latest = [], {}
        self.publishers = {name: rospy.Publisher("/"+name+"/cmd_vel", Twist, queue_size=1) for name in ROBOTS}
        self.subscriptions = []
        for name in ROBOTS:
            self.subscriptions += [rospy.Subscriber("/"+name+"/odom", Odometry,
                lambda msg, robot=name: self.receive(robot+"/odom", msg), queue_size=400),
                rospy.Subscriber("/"+name+"/joint_states", JointState,
                lambda msg, robot=name: self.receive(robot+"/joint_states", msg), queue_size=400),
                rospy.Subscriber("/"+name+"/cmd_vel", Twist,
                lambda msg, robot=name: self.receive(robot+"/cmd_vel", msg), queue_size=100)]
        self.subscriptions += [rospy.Subscriber("/tf", TFMessage, lambda msg: self.receive("tf", msg), queue_size=500),
            rospy.Subscriber("/tf_static", TFMessage, lambda msg: self.receive("tf_static", msg), queue_size=10),
            rospy.Subscriber("/lesson33/configuration", String, lambda msg: self.receive("configuration", msg), queue_size=1)]

    def receive(self, source, message):
        if source == "configuration":
            value = json.loads(message.data)
        elif source in ("tf", "tf_static"):
            for item in message.transforms:
                self.record_event(source+":"+item.child_frame_id, transform(item))
            return
        elif source.endswith("/odom"):
            value = {**pose(message.pose.pose), "stamp_ns": message.header.stamp.to_nsec(),
                "parent": message.header.frame_id, "child": message.child_frame_id,
                "linear_x": message.twist.twist.linear.x, "angular_z": message.twist.twist.angular.z}
        elif source.endswith("/joint_states"):
            value = {"stamp_ns": message.header.stamp.to_nsec(), "name": list(message.name),
                     "position": list(message.position), "velocity": list(message.velocity)}
        else:
            value = {"linear_x": message.linear.x, "angular_z": message.angular.z}
        self.record_event(source, value)

    def record_event(self, source, data):
        with self.lock:
            event = {"id": len(self.events), "source": source, "receive_monotonic_ns": time.monotonic_ns(), "data": data}
            self.events.append(event)
            self.latest[source] = event

    @staticmethod
    def bounded(action, description, timeout=5):
        done, result = threading.Event(), {}
        def invoke():
            try:
                result["value"] = action()
            except Exception as error:
                result["error"] = str(error)
            finally:
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        require(done.wait(timeout), "Wall-clock deadline: "+description)
        require("error" not in result, result.get("error", ""))
        return result.get("value")

    def call(self, path, service, **kwargs):
        def invoke():
            self.rospy.wait_for_service(path, timeout=3)
            proxy = self.rospy.ServiceProxy(path, service)
            try:
                return proxy(**kwargs)
            finally:
                proxy.close()
        response = self.bounded(invoke, path)
        require(not hasattr(response, "success") or response.success,
                path+": "+getattr(response, "status_message", "refused"))
        return response

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        value = self.call("/gazebo/get_physics_properties", GetPhysicsProperties)
        return {"paused": bool(value.pause), "step_s": value.time_step, "max_update_rate": value.max_update_rate}

    def world(self):
        from gazebo_msgs.srv import GetWorldProperties
        value = self.call("/gazebo/get_world_properties", GetWorldProperties)
        return {"sim_time": value.sim_time, "models": list(value.model_names)}

    def pause(self, paused=True):
        from std_srvs.srv import Empty
        before = self.physics()
        if before["paused"] != bool(paused):
            # One mutation, then read-only polling. Never retry a queued command.
            self.call("/gazebo/pause_physics" if paused else "/gazebo/unpause_physics", Empty)
        deadline, stable, previous = time.monotonic()+5, 0, None
        while time.monotonic() < deadline:
            state, now = self.physics(), self.world()["sim_time"]
            if state["paused"] == bool(paused) and (not paused or previous is None or abs(now-previous) < 1e-10):
                stable += 1
            else:
                stable = 0
            if stable >= 3:
                return {"paused": bool(paused), "sim_time": now, "stable_readings": stable}
            previous = now
            time.sleep(.025)
        raise RuntimeError("Actual pause/time state did not stabilize")

    def models(self):
        from gazebo_msgs.srv import GetModelState
        before = self.world()["sim_time"]
        output = {}
        for name in ROBOTS:
            value = self.call("/gazebo/get_model_state", GetModelState, model_name=name, relative_entity_name="world")
            output[name] = {**pose(value.pose), "linear": [value.twist.linear.x, value.twist.linear.y, value.twist.linear.z],
                "angular": [value.twist.angular.x, value.twist.angular.y, value.twist.angular.z]}
        return {"sim_time_before": before, "sim_time_after": self.world()["sim_time"], "models": output}

    def graph(self):
        master = self.rospy.get_master()
        code, message, value = self.bounded(master.getSystemState, "ROS Master graph")
        require(code == 1, "ROS Master refused: "+message)
        return {key: {name: nodes for name, nodes in items} for key, items in zip(("publishers", "subscribers", "services"), value)}

    def ready(self):
        from gazebo_msgs.srv import GetModelProperties
        self.parameters, self.record = prepared()
        state, world = self.physics(), self.world()
        require(set(ROBOTS).issubset(world["models"]), "Both physical robot models must exist")
        require(abs(state["step_s"]-.001) < 1e-10, "Unexpected integration step")
        properties = {}
        for name in ROBOTS:
            item = self.call("/gazebo/get_model_properties", GetModelProperties, model_name=name)
            properties[name] = {"body_names": list(item.body_names), "joint_names": list(item.joint_names), "is_static": bool(item.is_static)}
            require(not item.is_static and {name+"_left_joint", name+"_right_joint"}.issubset(item.joint_names), "Actual dynamic wheels missing")
            prefix = self.bounded(lambda robot=name: self.rospy.get_param("/"+robot+"/tf_prefix"), "tf_prefix")
            require(prefix == "", "Explicit TF IDs require an empty local tf_prefix")
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            with self.lock:
                latest = copy.deepcopy(self.latest)
            if "configuration" in latest and all("tf_static:"+name+"/odom" in latest for name in ROBOTS):
                break
            time.sleep(.03)
        else:
            raise RuntimeError("Persistent world TF bridge/configuration not available")
        configuration = latest["configuration"]["data"]
        require(configuration["identity"] == identity(self.record), "Loaded bridge/source configuration differs; reload/restart")
        graph = self.graph()
        for name in ROBOTS:
            require("/"+name+"/cmd_vel" in graph["subscribers"], "Differential plugin command subscription missing")
            for topic in ("odom", "joint_states"):
                require("/"+name+"/"+topic in graph["publishers"], "Differential plugin publisher missing: "+topic)
            edge = latest["tf_static:"+name+"/odom"]["data"]
            require(edge["parent"] == "world" and edge["position"] == [0, 0, 0] and edge["orientation"] == [0, 0, 0, 1], "Shared-world identity bridge differs")
        return {"ready": True, "parameters": self.parameters, "physics": state, "world": world,
                "model_properties": properties, "configuration": configuration, "graph": graph,
                "dynamic_observation_deferred": state["paused"],
                "note": "Readiness does not advance physics. Use bootstrap for fresh physical odometry/joints/TF."}

    def wait_connections(self):
        deadline = time.monotonic()+4
        while time.monotonic() < deadline and any(pub.get_num_connections() < 1 for pub in self.publishers.values()):
            time.sleep(.02)
        require(all(pub.get_num_connections() >= 1 for pub in self.publishers.values()), "Both cmd_vel subscribers required")

    def command(self, commands):
        from geometry_msgs.msg import Twist
        for name in ROBOTS:
            v, w = commands.get(name, (0, 0))
            require(math.isfinite(v) and math.isfinite(w) and abs(v) <= .3 and abs(w) <= .9, "Bounded finite velocity required")
            message = Twist()
            message.linear.x, message.angular.z = v, w
            self.publishers[name].publish(message)

    def zero(self):
        self.wait_connections()
        # The official plugin holds the last command; actively clear both queues.
        for _ in range(5):
            self.command({})
            time.sleep(.025)

    def advance(self, seconds, commands=None):
        require(math.isfinite(seconds) and 0 < seconds <= 4, "Use 0 < simulation seconds <= 4")
        self.wait_connections()
        try:
            self.command(commands or {})
            self.pause(False)
            start, deadline = self.world()["sim_time"], time.monotonic()+max(20, seconds*12)
            last = start
            while time.monotonic() < deadline:
                self.command(commands or {})
                last = self.world()["sim_time"]
                require(last >= start-1e-8, "Simulation time reset during the physical measurement")
                if last-start >= seconds:
                    break
                time.sleep(.025)
            else:
                raise RuntimeError("Simulation did not advance before wall-clock deadline")
        finally:
            try:
                self.zero()
            finally:
                self.pause(True)
        return {"sim_time_start": start, "sim_time_end": self.world()["sim_time"], "requested_sim_seconds": seconds}

    def snapshot(self, dynamic=False):
        time.sleep(.1)
        actual = self.models()
        with self.lock:
            latest = copy.deepcopy(self.latest)
        result = {**actual, "physics": self.physics(), "latest": latest}
        if dynamic:
            self.check_feedback(result)
        return result

    def check_feedback(self, snapshot):
        latest, now = snapshot["latest"], snapshot["sim_time_after"]
        for name in ROBOTS:
            keys = (name+"/odom", name+"/joint_states", "tf:"+name+"/base_footprint")
            require(all(key in latest for key in keys), "Actual odometry, joint states and TF required for "+name)
            odom, joints, tf = (latest[key]["data"] for key in keys)
            for item in (odom, joints, tf):
                stamp = item["stamp_ns"]/1e9
                require(-.001 <= now-stamp <= .12, "Physical feedback is stale or from before a clock reset")
            require(odom["parent"] == name+"/odom" and odom["child"] == name+"/base_footprint", "Odometry frame collision")
            require(tf["parent"] == name+"/odom" and tf["child"] == name+"/base_footprint", "TF frame collision")
            require(set(joints["name"]) == {name+"_left_joint", name+"_right_joint"}, "JointState came from the wrong robot")
            actual = snapshot["models"][name]
            for item in (odom, tf):
                require(math.dist(item["position"], actual["position"]) < .045, "World odometry/TF disagrees with real Gazebo model position")
                require(abs(angle(yaw(item["orientation"])-actual["yaw"])) < .08, "World odometry/TF disagrees with model yaw")

    def bootstrap(self):
        self.ready()
        self.zero()
        timing = self.advance(.5)
        return {"bootstrap": True, "timing": timing, "snapshot": self.snapshot(dynamic=True)}

    def stop(self):
        try:
            self.zero()
        finally:
            self.pause(True)
        return {"zero_requested": True, "paused_confirmed": True, "snapshot": self.snapshot(),
                "note": "Zero targets and pause confirmed; this operation does not advance a physical stopping interval."}

    def reset(self):
        from std_srvs.srv import Empty
        self.stop()
        self.call("/gazebo/reset_simulation", Empty)  # exactly once
        deadline, stable = time.monotonic()+5, 0
        while time.monotonic() < deadline:
            world, physics = self.world(), self.physics()
            if world["sim_time"] <= 1e-8 and physics["paused"]:
                stable += 1
            else:
                stable = 0
            if stable >= 3:
                self.zero()
                # Preserve history as evidence but invalidate pre-reset dynamic
                # cache entries. A later bootstrap must produce new-clock data.
                with self.lock:
                    self.latest = {key: value for key, value in self.latest.items()
                                   if key == "configuration" or key.startswith("tf_static:") or key.endswith("/cmd_vel")}
                result = self.snapshot()
                for name, y in (("alpha", -.8), ("beta", .8)):
                    position = result["models"][name]["position"]
                    require(abs(position[0]) < .005 and abs(position[1]-y) < .005, "Reset did not restore real spawn pose")
                return {"reset": True, "snapshot": result, "note": "Old dynamic samples are historical. Run bootstrap to obtain new-clock feedback."}
            time.sleep(.025)
        raise RuntimeError("Reset time/pause did not stabilize")

    def phase(self, name, seconds=2):
        require(name in ("alpha", "beta", "both"), "Use alpha/beta/both")
        require(math.isfinite(seconds) and .5 <= seconds <= 3, "Use 0.5 to 3 simulation seconds for a measured phase")
        v, w = self.parameters["linear_speed_mps"], self.parameters["angular_speed_rps"]
        commands = {"alpha": (v, 0)} if name == "alpha" else {"beta": (0, w)} if name == "beta" else {"alpha": (v, 0), "beta": (0, -w)}
        self.advance(.4)
        before = self.snapshot(dynamic=True)
        with self.lock:
            first = len(self.events)
        timing = self.advance(seconds, commands)
        # A finite zero-command tail measures residual motion, not just acceptance.
        tail = self.advance(.6)
        after = self.snapshot(dynamic=True)
        with self.lock:
            events = copy.deepcopy(self.events[first:])
        metrics = {}
        for robot in ROBOTS:
            old, new = before["models"][robot], after["models"][robot]
            dx, dy = new["position"][0]-old["position"][0], new["position"][1]-old["position"][1]
            joints_before = before["latest"][robot+"/joint_states"]["data"]
            joints_after = after["latest"][robot+"/joint_states"]["data"]
            previous = dict(zip(joints_before["name"], joints_before["position"]))
            current = dict(zip(joints_after["name"], joints_after["position"]))
            metrics[robot] = {"distance_m": math.hypot(dx, dy), "body_forward_m": math.cos(old["yaw"])*dx+math.sin(old["yaw"])*dy,
                "yaw_delta_rad": angle(new["yaw"]-old["yaw"]),
                "wheel_delta_rad": {key: current[key]-previous[key] for key in current},
                "terminal_xy_speed_mps": math.hypot(new["linear"][0], new["linear"][1]),
                "terminal_yaw_rate_rps": new["angular"][2]}
            require(metrics[robot]["terminal_xy_speed_mps"] < .04 and abs(metrics[robot]["terminal_yaw_rate_rps"]) < .08,
                    "Real Gazebo velocity did not settle after the zero-command tail: "+robot)
            seen = [event["data"] for event in events if event["source"] == robot+"/cmd_vel"]
            expected_v, expected_w = commands.get(robot, (0, 0))
            require(any(abs(item["linear_x"]-expected_v) < 1e-8 and abs(item["angular_z"]-expected_w) < 1e-8 for item in seen), "Raw expected Twist not observed")
            if robot not in commands:
                require(all(abs(item["linear_x"])+abs(item["angular_z"]) < 1e-8 for item in seen), "Unexpected nonzero command on inactive namespace")
            require(sum(event["source"] == robot+"/odom" for event in events) >= 10, "Too few actual odometry samples")
        a, b = metrics["alpha"], metrics["beta"]
        if name in ("alpha", "both"):
            require(a["body_forward_m"] > v*seconds*.5 and abs(a["yaw_delta_rad"]) < .12, "Alpha failed physical forward drive")
            require(all(value > .3 for value in a["wheel_delta_rad"].values()), "Alpha wheels did not both turn forward")
        else:
            require(a["distance_m"] < .03 and abs(a["yaw_delta_rad"]) < .06, "Beta command moved inactive alpha")
        if name in ("beta", "both"):
            sign = 1 if name == "beta" else -1
            require(sign*b["yaw_delta_rad"] > w*seconds*.45 and b["distance_m"] < .08, "Beta failed physical in-place turn")
            require(sign*b["wheel_delta_rad"]["beta_left_joint"] < -.3 and sign*b["wheel_delta_rad"]["beta_right_joint"] > .3, "Beta wheel directions did not produce the requested turn")
        else:
            require(b["distance_m"] < .03 and abs(b["yaw_delta_rad"]) < .06, "Alpha command moved inactive beta")
        return {"phase": name, "commands": {key: list(commands.get(key, (0, 0))) for key in ROBOTS},
            "timing": timing, "zero_tail": tail, "before": before, "after": after, "metrics": metrics, "events": events, "passed": True}

    def exercise(self):
        ready = self.ready()
        self.bootstrap()
        phases = []
        try:
            for name in ("alpha", "beta", "both"):
                phases.append(self.phase(name))
        finally:
            self.stop()
        return {"passed": True, "parameters": self.parameters, "prepared_identity": identity(self.record), "ready": ready,
                "phases": phases, "end": self.snapshot(dynamic=True),
                "scope": "Real selective and concurrent physical drive; raw commands, odometry, wheel states, TF and Gazebo model-state agreement. No navigation or collision avoidance."}


def handle_signal(signum, frame):
    raise KeyboardInterrupt("Signal "+str(signum))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=("ready", "bootstrap", "status", "pause", "reset", "alpha", "beta", "both", "exercise"))
    parser.add_argument("--seconds", type=float, default=2)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, handle_signal)
    client = Client()
    with (Path("build")/"lab.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.operation in ("alpha", "beta", "both"):
            client.ready()
            try:
                result = client.phase(args.operation, args.seconds)
            finally:
                client.stop()
        else:
            result = {"ready": client.ready, "bootstrap": client.bootstrap, "status": client.snapshot,
                      "pause": client.stop, "reset": client.reset, "exercise": client.exercise}[args.operation]()
        output = Path("build")/(args.operation+"_report.json")
        output.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
        print(json.dumps({"operation": args.operation, "report": str(output), "passed": result.get("passed"),
                          "metrics": result.get("metrics"), "result": result if args.operation in ("ready", "status", "pause", "reset", "bootstrap") else None}, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
