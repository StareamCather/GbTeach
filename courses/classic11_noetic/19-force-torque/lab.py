#!/usr/bin/env python3
"""Read actual native/stock wrench streams without imposing forces or motion."""
import argparse
from collections import deque
import json
import math
import statistics
import threading
import time
from config import edit, prepared

CHANNELS = {
    "native_child": ("/lesson19/native_child", "load_link"),
    "native_parent_to_child": ("/lesson19/native_parent_to_child", "load_link"),
    "native_sensor_x90": ("/lesson19/native_sensor_x90", "sensor_x90"),
    "stock_child": ("/lesson19/stock_wrench", "load_link")}


def vector(value):
    return [value.x, value.y, value.z]


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def expected(config):
    weight = config["mass_kg"]*config["gravity_m_s2"]
    moment = weight*config["arm_m"]
    return {"native_child": {"force": [0, 0, -weight], "torque": [0, moment, 0]},
            "native_parent_to_child": {"force": [0, 0, weight], "torque": [0, -moment, 0]},
            "native_sensor_x90": {"force": [0, -weight, 0], "torque": [0, 0, -moment]},
            "stock_child": {"force": [0, 0, -weight], "torque": [0, moment, 0]}}


def check_row(row, config):
    if row["frame"].lstrip("/") != CHANNELS[row["source"]][1]:
        raise RuntimeError("Actual wrench header frame label is unexpected")
    if any(not math.isfinite(value) for key in ("force", "torque") for value in row[key]):
        raise RuntimeError("Actual wrench contains non-finite values")
    wanted = expected(config)[row["source"]]
    for key, tolerance in (("force", .05), ("torque", .03)):
        if max(abs(a-b) for a, b in zip(row[key], wanted[key])) > tolerance:
            raise RuntimeError("Actual "+row["source"]+" "+key+" differs from signed anchored-load prediction: actual="+str(row[key])+", expected="+str(wanted[key]))


class Client:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import WrenchStamped
        self.rospy = rospy
        rospy.init_node("gazeboteach_wrench_lab", anonymous=True, disable_signals=True)
        self.rows, self.lock = deque(maxlen=6000), threading.Lock()
        self.subscriptions = [rospy.Subscriber(topic, WrenchStamped, lambda message, source=source: self.receive(source, message), queue_size=300)
                              for source, (topic, frame) in CHANNELS.items()]

    def receive(self, source, message):
        row = {"source": source, "stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
               "force": vector(message.wrench.force), "torque": vector(message.wrench.torque)}
        with self.lock:
            self.rows.append(row)

    def drain(self):
        with self.lock:
            rows = list(self.rows)
            self.rows.clear()
        return rows

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()

    def call(self, name, service_type, *args, timeout=4):
        self.rospy.wait_for_service(name, timeout=timeout)
        done, result = threading.Event(), []
        def worker():
            try:
                proxy = self.rospy.ServiceProxy(name, service_type)
                try:
                    result.append((True, proxy(*args)))
                finally:
                    proxy.close()
            except Exception as error:
                result.append((False, error))
            finally:
                done.set()
        threading.Thread(target=worker, daemon=True).start()
        if not done.wait(timeout):
            raise RuntimeError("ROS service wall deadline; control not resent: "+name)
        success, value = result[0]
        if not success:
            raise value
        if hasattr(value, "success") and not value.success:
            raise RuntimeError(name+": "+value.status_message)
        return value

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, paused):
        from std_srvs.srv import Empty
        if self.physics().pause != paused:
            self.call("/gazebo/"+("pause_physics" if paused else "unpause_physics"), Empty)
        if self.physics().pause != paused:
            raise RuntimeError("Actual pause state did not match requested value")

    def sim_time_ns(self):
        from gazebo_msgs.srv import GetWorldProperties
        value = self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time
        if not math.isfinite(value):
            raise RuntimeError("Actual world simulation time is not finite")
        return round(value*1e9)

    def properties(self):
        from gazebo_msgs.srv import GetLinkProperties
        return self.call("/gazebo/get_link_properties", GetLinkProperties, "weighing_rig::load_link")

    def truth(self):
        from gazebo_msgs.srv import GetLinkState
        before = self.sim_time_ns()
        response = self.call("/gazebo/get_link_state", GetLinkState, "weighing_rig::load_link", "world")
        after = self.sim_time_ns()
        value, q = response.link_state, response.link_state.pose.orientation
        return {"clock_before_ns": before, "clock_after_ns": after,
                "position": vector(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
                "linear": vector(value.twist.linear), "angular": vector(value.twist.angular)}


def check_properties(properties, config):
    q = properties.com.orientation
    if not properties.gravity_mode or abs(properties.mass-config["mass_kg"]) > 1e-9:
        raise RuntimeError("Actual load mass/gravity differs from prepared fixture")
    if norm([properties.com.position.x-config["arm_m"], properties.com.position.y, properties.com.position.z]) > 1e-9 or norm([q.x, q.y, q.z]) > 1e-9:
        raise RuntimeError("Actual center of mass differs from prepared lever arm")
    inertia = config["mass_kg"]*.2**2/6
    if any(abs(getattr(properties, key)-inertia) > 1e-10 for key in ("ixx", "iyy", "izz")):
        raise RuntimeError("Actual payload inertia is not the uniform cube inertia about its CoM")
    return {"mass_kg": properties.mass, "gravity_mode": properties.gravity_mode,
            "com_position": vector(properties.com.position), "inertia_diagonal": [properties.ixx, properties.iyy, properties.izz]}


def check_truth(truth, config):
    if norm([a-b for a, b in zip(truth["position"], config["anchor_world_m"])]) > .005:
        raise RuntimeError("Load link origin is not held near actual fixed-joint anchor")
    if norm(truth["linear"]) > .01 or norm(truth["angular"]) > .01 or norm(truth["orientation"][:3]) > .005 or abs(norm(truth["orientation"])-1) > 1e-6:
        raise RuntimeError("Wrench observation is not accompanied by actual static equilibrium")


def ready(config, timeout=12):
    client, found = Client(), {}
    try:
        check_properties(client.properties(), config)
        deadline, last_error = time.monotonic()+timeout, None
        while time.monotonic() < deadline and len(found) < len(CHANNELS):
            for row in client.drain():
                try:
                    check_row(row, config)
                    found[row["source"]] = row
                except RuntimeError as error:
                    last_error = str(error)
            time.sleep(.02)
        if len(found) != len(CHANNELS):
            raise RuntimeError("Actual four-channel wrench readiness timed out: "+str(last_error))
        return {"ready": True, "channels": found}
    finally:
        client.close()


def observe(config, seconds):
    if not math.isfinite(seconds) or not .2 <= seconds <= 30:
        raise ValueError("Observation must bewithin[.2,30]wall seconds")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(.02)
        result = {}
        for source in CHANNELS:
            unique = {row["stamp_ns"]: row for row in rows if row["source"] == source}
            for row in unique.values():
                check_row(row, config)
            result[source] = {"messages": len(unique),
                **{"mean_"+key: [statistics.mean(row[key][axis] for row in unique.values()) for axis in range(3)] if unique else None for key in ("force", "torque")}}
        return result
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    cfg = commands.add_parser("config")
    cfg.add_argument("--mass", type=float)
    cfg.add_argument("--arm", type=float)
    obs = commands.add_parser("observe")
    obs.add_argument("--seconds", type=float, default=3)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.mass is None and args.arm is None:
            parser.error("config requires --mass or --arm")
        result = {"config": edit(args.mass, args.arm), "reload_required": True}
    else:
        config, record = prepared()
        result = ready(config) if args.command == "ready" else observe(config, args.seconds)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
