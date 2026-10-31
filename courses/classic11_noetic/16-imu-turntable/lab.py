#!/usr/bin/env python3
"""Real IMU messages, bounded service calls, and explicit one-shot initial states."""
import argparse
import copy
from collections import deque
import json
import math
import threading
import time
from config import edit, prepared

FIXTURES = {"turntable": ("/lesson16/imu", "imu_link"),
            "freefall": ("/lesson16/freefall_imu", "freefall_link")}
MODELS = {"turntable": "turntable", "freefall": "freefall_probe"}


def number(value):
    value = float(value)
    return value if math.isfinite(value) else ("nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf"))


def vector(value):
    return [number(value.x), number(value.y), number(value.z)]


def imu_row(source, message):
    q = message.orientation
    return {"source": source, "stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
            "orientation": [number(q.x), number(q.y), number(q.z), number(q.w)],
            "angular_velocity": vector(message.angular_velocity), "linear_acceleration": vector(message.linear_acceleration),
            **{key: [number(value) for value in getattr(message, key)] for key in
               ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance")}}


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def quaternion_error(left, right):
    dot = sum(a*b for a, b in zip(left, right))/(norm(left)*norm(right))
    return 2*math.acos(min(1, max(0, abs(dot))))


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def check_row(row):
    if row["frame"] != FIXTURES[row["source"]][1]:
        raise RuntimeError("Unexpected IMU frame label")
    for key in ("orientation", "angular_velocity", "linear_acceleration", "orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"):
        expected = 4 if key == "orientation" else (9 if key.endswith("covariance") else 3)
        if len(row[key]) != expected or any(type(value) not in (int, float) or not math.isfinite(value) for value in row[key]):
            raise RuntimeError("Invalid actual IMU numeric field: "+key)
    if abs(norm(row["orientation"])-1) > 1e-6:
        raise RuntimeError("Actual zero-noise orientation quaternion is not unit length")
    if any(abs(value) > 1e-14 for key in ("orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance") for value in row[key]):
        raise RuntimeError("Stock plugin covariance differs from zero-noise configured semantics")


class Client:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import Imu
        self.rospy = rospy
        rospy.init_node("gazeboteach_imu_lab", anonymous=True, disable_signals=True)
        self.rows, self.lock = deque(maxlen=6000), threading.Lock()
        self.subscriptions = [rospy.Subscriber(topic, Imu, lambda message, source=source: self.receive(source, message), queue_size=500)
                              for source, (topic, frame) in FIXTURES.items()]

    def receive(self, source, message):
        with self.lock:
            self.rows.append(imu_row(source, message))

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
        ok, value = result[0]
        if not ok:
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
            raise RuntimeError("Actual pause state differs from requested value")

    def state(self, source="turntable"):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import GetModelState
        answer = self.call("/gazebo/get_model_state", GetModelState, MODELS[source], "world")
        result = ModelState()
        result.model_name, result.reference_frame = MODELS[source], "world"
        result.pose, result.twist = copy.deepcopy(answer.pose), copy.deepcopy(answer.twist)
        return result

    def apply(self, state):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, state)

    def setup(self, source="turntable", roll=0, speed=0):
        from gazebo_msgs.msg import ModelState
        state = ModelState()
        state.model_name, state.reference_frame = MODELS[source], "world"
        state.pose.position.x, state.pose.position.z = (0, 1) if source == "turntable" else (2, 3)
        state.pose.orientation.x, state.pose.orientation.w = math.sin(roll/2), math.cos(roll/2)
        state.twist.angular.z = speed
        self.apply(state)

    def truth(self, source="turntable"):
        from gazebo_msgs.srv import GetLinkState
        before = self.rospy.Time.now().to_nsec()
        answer = self.call("/gazebo/get_link_state", GetLinkState, MODELS[source]+"::"+FIXTURES[source][1], "world")
        after = self.rospy.Time.now().to_nsec()
        return {"source": source, "clock_before_ns": before, "clock_after_ns": after,
                "state": state_values(answer.link_state)}

    def properties(self, source):
        from gazebo_msgs.srv import GetLinkProperties
        return self.call("/gazebo/get_link_properties", GetLinkProperties, MODELS[source]+"::"+FIXTURES[source][1])


def state_values(state):
    q = state.pose.orientation
    return {"position": vector(state.pose.position), "orientation": [number(q.x), number(q.y), number(q.z), number(q.w)],
            "linear": vector(state.twist.linear), "angular": vector(state.twist.angular)}


def ready(timeout=10):
    client, found = Client(), {}
    try:
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and len(found) < 2:
            for row in client.drain():
                check_row(row)
                found[row["source"]] = row
            time.sleep(0.02)
        if len(found) != 2:
            raise RuntimeError("Actual IMU channels did not become ready")
        return {"ready": True, "sensors": found}
    finally:
        client.close()


def observe(seconds):
    if not math.isfinite(seconds) or not 0.2 <= seconds <= 30:
        raise ValueError("Observation wall duration must be within[.2,30]s")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(.02)
        result = {}
        for source in FIXTURES:
            unique = {row["stamp_ns"]: row for row in rows if row["source"] == source}
            for row in unique.values():
                check_row(row)
            stamps = sorted(unique)
            result[source] = {"messages": len(stamps), "span_s": (stamps[-1]-stamps[0])/1e9 if len(stamps)>1 else 0,
                              "latest": unique[stamps[-1]] if stamps else None}
        return result
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    config_parser = commands.add_parser("config")
    config_parser.add_argument("--speed", type=float)
    config_parser.add_argument("--tilt", type=float)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.speed is None and args.tilt is None:
            parser.error("config requires --speed or --tilt")
        result = {"parameters": edit(args.speed, args.tilt), "reload_required": True}
    else:
        prepared()
        result = ready() if args.command == "ready" else observe(args.seconds)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
