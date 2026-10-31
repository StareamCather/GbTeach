#!/usr/bin/env python3
"""Bounded real LaserScan observations and single-shot wall controls."""
import argparse
import copy
from collections import deque
import json
import math
import statistics
import threading
import time
from config import edit, prepared

TOPICS = {"cpu": "/lesson15/scan", "gpu": "/lesson15/gpu_scan"}
ANGLES = (-90, -60, -45, -30, 0, 30, 45, 60, 90)


def number(value):
    value = float(value)
    return value if math.isfinite(value) else ("nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf"))


def scan_row(source, message):
    return {"source": source, "stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
            **{key: number(getattr(message, key)) for key in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max", "scan_time", "time_increment")},
            "ranges": [number(value) for value in message.ranges], "intensities": [number(value) for value in message.intensities]}


class Client:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import LaserScan
        self.rospy = rospy
        rospy.init_node("gazeboteach_laser_lab", anonymous=True, disable_signals=True)
        self.rows, self.lock = deque(maxlen=4000), threading.Lock()
        self.subscriptions = [rospy.Subscriber(topic, LaserScan, lambda message, name=name: self.receive(name, message), queue_size=200)
                              for name, topic in TOPICS.items()]

    def receive(self, source, message):
        with self.lock:
            self.rows.append(scan_row(source, message))

    def drain(self):
        with self.lock:
            result = list(self.rows)
            self.rows.clear()
        return result

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
            raise RuntimeError("Actual pause state did not match request")

    def state(self):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import GetModelState
        answer = self.call("/gazebo/get_model_state", GetModelState, "target_wall", "world")
        value = ModelState()
        value.model_name, value.reference_frame = "target_wall", "world"
        value.pose, value.twist = copy.deepcopy(answer.pose), copy.deepcopy(answer.twist)
        return value

    def apply(self, value):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, value)

    def move(self, distance):
        from gazebo_msgs.msg import ModelState
        value = ModelState()
        value.model_name, value.reference_frame = "target_wall", "world"
        value.pose.position.x, value.pose.position.z = distance, 1
        value.pose.orientation.w = 1
        self.apply(value)
        actual = state_values(self.state())
        if max(abs(a-b) for a, b in zip(actual["position"], [distance, 0, 1])) > 1e-7 or max(abs(a-b) for a, b in zip(actual["orientation"], [0, 0, 0, 1])) > 1e-7:
            raise RuntimeError("Actual wall pose did not reach requested axis-aligned placement")
        return actual


def state_values(value):
    def vec(item):
        return [item.x, item.y, item.z]
    q = value.pose.orientation
    return {"position": vec(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
            "linear": vec(value.twist.linear), "angular": vec(value.twist.angular)}


def ray_box(angle, xmin, xmax, ymin, ymax):
    near, far = -math.inf, math.inf
    for direction, lower, upper in ((math.cos(angle), xmin, xmax), (math.sin(angle), ymin, ymax)):
        if abs(direction) < 1e-10:
            if not lower <= 0 <= upper:
                return math.inf
            continue
        a, b = sorted((lower/direction, upper/direction))
        near, far = max(near, a), min(far, b)
    return near if far >= max(near, 0) and near >= 0 else math.inf


def expected_range(source, angle, distance):
    wall = ray_box(angle, distance-0.1, distance+0.1, -4, 4)
    center_y = -1.5 if source == "cpu" else 1.5
    contrast = ray_box(angle, 1.3, 1.7, center_y-0.2, center_y+0.2)
    value = min(wall, contrast)
    return value if 0.1 < value < 10 else math.inf


def check_metadata(row, config):
    count = config["beam_count"]
    if len(row["ranges"]) != count or len(row["intensities"]) not in (0, count):
        raise RuntimeError("Actual LaserScan array count differs from prepared samples")
    if row["frame"] not in ("laser_frame", "/laser_frame"):
        raise RuntimeError("Unexpected LaserScan frame label")
    for key, expected in {"angle_min": -math.pi/2, "angle_max": math.pi/2,
                          "angle_increment": math.pi/(count-1), "range_min": 0.1, "range_max": 10,
                          "time_increment": 0, "scan_time": 0}.items():
        value = row[key]
        if not isinstance(value, (int, float)) or not math.isclose(value, expected, abs_tol=2e-6, rel_tol=1e-6):
            raise RuntimeError("Unexpected LaserScan metadata: "+key)
    for value in row["ranges"]:
        if value in ("+inf", "-inf"):
            continue
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0.1 <= value <= 10:
            raise RuntimeError("Invalid actual LaserScan range value")


def checks(row, config, distance):
    check_metadata(row, config)
    results = []
    for degrees in ANGLES:
        requested = math.radians(degrees)
        index = int(round((requested-row["angle_min"])/row["angle_increment"]))
        index = max(0, min(config["beam_count"]-1, index))
        angle = row["angle_min"]+index*row["angle_increment"]
        expected, actual = expected_range(row["source"], angle, distance), row["ranges"][index]
        error = abs(actual-expected) if isinstance(actual, (int, float)) and math.isfinite(expected) else None
        passed = error <= 0.08 if error is not None else (math.isinf(expected) and actual == "+inf")
        results.append({"degrees_requested": degrees, "index": index, "angle_rad": angle,
                        "expected_m": number(expected), "observed_m": actual, "error_m": error, "passed": passed})
    return results


def ready(config, timeout=20):
    client, found = Client(), {}
    deadline = time.monotonic()+timeout
    try:
        while time.monotonic() < deadline and len(found) < 2:
            for row in client.drain():
                check_metadata(row, config)
                result = checks(row, config, config["wall_distance_m"])
                if all(item["passed"] for item in result):
                    found[row["source"]] = {"stamp_ns": row["stamp_ns"], "checks": result}
            time.sleep(0.02)
        if len(found) != 2:
            raise RuntimeError("CPU/GPU actual geometry readiness timed out; GPU requires working display/rendering")
        return {"ready": True, "sensors": found}
    finally:
        client.close()


def observe(config, seconds):
    if not math.isfinite(seconds) or not 0.2 <= seconds <= 30:
        raise ValueError("Observation wall time must be within[0.2,30]s")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(0.02)
        output = {}
        for source in TOPICS:
            unique = {row["stamp_ns"]: row for row in rows if row["source"] == source}
            for row in unique.values():
                check_metadata(row, config)
            times = sorted(unique)
            output[source] = {"messages": len(times), "sim_span_s": (times[-1]-times[0])/1e9 if len(times)>1 else 0,
                              "latest": unique[times[-1]] if times else None}
        return output
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    config_parser = commands.add_parser("config")
    config_parser.add_argument("--distance", type=float)
    config_parser.add_argument("--samples", type=int)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    wall = commands.add_parser("wall")
    wall.add_argument("--distance", type=float, required=True)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.distance is None and args.samples is None:
            parser.error("config requires --distance or --samples")
        result = {"config": edit(args.distance, args.samples), "reload_required": True}
    else:
        config, record = prepared()
        if args.command == "ready":
            result = ready(config)
        elif args.command == "observe":
            result = observe(config, args.seconds)
        else:
            if not math.isfinite(args.distance) or not 2.5 <= args.distance <= 5:
                parser.error("Runtime wall distance must be within[2.5,5]m")
            client, paused = Client(), None
            try:
                paused = client.physics().pause
                client.pause(True)
                before = state_values(client.state())
                after = client.move(args.distance)
                result = {"wall_before": before, "wall_after": after, "paused": paused}
            finally:
                try:
                    if paused is not None:
                        client.pause(paused)
                finally:
                    client.close()
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
