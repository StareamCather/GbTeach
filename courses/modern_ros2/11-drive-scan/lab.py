"""Drive the real Harmonic rover through ROS 2, and compare independent observations."""

from collections import deque
import math
import signal
from statistics import median
import time

import _bootstrap  # Adds the installed course support directory.
from common import ensure_running, move_to, pose, result, topic, yaw

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


def stamp(message):
    value = message.header.stamp
    return value.sec + value.nanosec / 1e9


def native_pose():
    value = pose("rover")
    position = value.get("position", {})
    return [float(position.get(axis, 0)) for axis in "xyz"] + [yaw(value)]


def odom_pose(message):
    position = message.pose.pose.position
    q = message.pose.pose.orientation
    return [position.x, position.y, position.z,
            yaw({"orientation": {key: getattr(q, key) for key in "xyzw"}})]


def wheel_speeds():
    message = topic("/gt/wheels", timeout=4)
    data = {item.get("name"): float(item["axis1"].get("velocity", 0))
            for item in message.get("joint", []) if "axis1" in item}
    names = ("left_joint", "right_joint")
    if not all(name in data and math.isfinite(data[name]) for name in names):
        raise RuntimeError(f"Native JointStatePublisher did not return both wheels: {data}")
    return [data[name] for name in names]


def front_range(scan):
    middle = len(scan.ranges) // 2
    center = scan.ranges[middle - 2:middle + 3]
    if len(scan.ranges) != 181 or len(center) != 5 or not all(
            math.isfinite(value) and scan.range_min < value < scan.range_max for value in center):
        raise RuntimeError("Expected 181 GPU lidar rays with five finite front-wall returns")
    return float(median(center))


def native_front_range():
    scan = topic("/gt/scan", timeout=4)
    values = scan.get("ranges", [])
    center = [float(value) for value in values[88:93]]
    if len(values) != 181 or len(center) != 5 or not all(math.isfinite(value) for value in center):
        raise RuntimeError("Native Gazebo Transport did not return the expected 181-ray scan")
    return float(median(center))


def angle_delta(after, before):
    return (after - before + math.pi) % (2 * math.pi) - math.pi


def experiment():
    ensure_running()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("m11_rover_observer")
    odom = deque(maxlen=5)
    scans = deque(maxlen=5)
    counts = {"odom": 0, "scan": 0}

    def record(queue, key, message):
        if not queue or stamp(message) > stamp(queue[-1]):
            queue.append(message)
            counts[key] += 1

    node.create_subscription(Odometry, "/gt/odom", lambda msg: record(odom, "odom", msg),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/gt/scan", lambda msg: record(scans, "scan", msg),
                             qos_profile_sensor_data)
    command = node.create_publisher(Twist, "/gt/cmd_vel", 10)

    def burst(linear, angular, duration, capture_wheels=False):
        """Measure each command phase using actual simulated Odometry time."""
        start = stamp(odom[-1])
        value = Twist()
        value.linear.x = linear
        value.angular.z = angular
        deadline = time.monotonic() + 14
        next_publish = 0
        wheels = None
        while stamp(odom[-1]) - start < duration:
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(f"Timed out waiting for advancing Odometry in phase {linear, angular}")
            if now >= next_publish:
                command.publish(value)
                next_publish = now + .05
            rclpy.spin_once(node, timeout_sec=.02)
            if capture_wheels and wheels is None and stamp(odom[-1]) - start >= .35:
                wheels = wheel_speeds()
        return stamp(odom[-1]) - start, wheels

    def cancel(signum, _frame):
        raise KeyboardInterrupt(f"Cancelled by signal {signum}")

    previous_signals = {value: signal.signal(value, cancel) for value in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        deadline = time.monotonic() + 12
        while len(odom) < 2 or len(scans) < 2 or command.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"ROS 2 bridge not ready (actual messages {counts})")
            command.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.05)

        # Repeat the prescribed motion from the same physical pose; odometry
        # keeps its accumulated origin, so compare changes rather than origins.
        burst(0., 0., .4)
        move_to("rover", [0., 0., .15])
        burst(0., 0., .65)
        initial, odom_initial = native_pose(), odom_pose(odom[-1])
        initial_scan = scans[-1]
        before = front_range(initial_scan)
        native_before = native_front_range()
        if math.hypot(*initial[:2]) > .05 or abs(initial[3]) > .10 or not 3.45 < before < 3.95:
            raise RuntimeError(f"Unexpected reset pose or wall range: {initial}, {before}")

        drive_seconds, drive_wheels = burst(.25, 0., 2.0, capture_wheels=True)
        burst(0., 0., .55)
        straight, odom_straight = native_pose(), odom_pose(odom[-1])
        straight_scan = scans[-1]
        after = front_range(straight_scan)
        native_after = native_front_range()
        dx = straight[0] - initial[0]
        odom_distance = math.hypot(odom_straight[0] - odom_initial[0],
                                   odom_straight[1] - odom_initial[1])
        if (not .28 < dx < .75 or abs(straight[1] - initial[1]) >= .12 or
                abs(angle_delta(straight[3], initial[3])) >= .15 or
                abs(odom_distance - dx) >= .13 or not all(value > .20 for value in drive_wheels) or
                stamp(straight_scan) <= stamp(initial_scan) or abs((before - after) - dx) >= .10 or
                abs(before - native_before) >= .03 or abs(after - native_after) >= .03):
            raise AssertionError(f"Straight-drive mismatch: real {straight}, odom {odom_straight}, "
                                 f"wheel rates {drive_wheels}, ranges {before}, {after}")

        turn_seconds, turn_wheels = burst(0., .55, 1.35, capture_wheels=True)
        burst(0., 0., .55)
        turned, odom_turned = native_pose(), odom_pose(odom[-1])
        delta_yaw = angle_delta(turned[3], straight[3])
        odom_yaw = angle_delta(odom_turned[3], odom_straight[3])
        if (not .35 < delta_yaw < 1.05 or abs(odom_yaw - delta_yaw) >= .25 or
                math.hypot(turned[0] - straight[0], turned[1] - straight[1]) >= .20 or
                not turn_wheels[0] < -.20 or not turn_wheels[1] > .20):
            raise AssertionError(f"Turn mismatch: real {turned}, odom {odom_turned}, wheel rates {turn_wheels}")

        burst(0., 0., .4)
        stopped = native_pose()
        stop_wheels = wheel_speeds()
        drift_m = math.hypot(stopped[0] - turned[0], stopped[1] - turned[1])
        drift_rad = abs(angle_delta(stopped[3], turned[3]))
        if drift_m >= .03 or drift_rad >= .04 or max(map(abs, stop_wheels)) >= .25:
            raise AssertionError(f"Rover did not settle after zero Twist: {drift_m}, {drift_rad}, {stop_wheels}")

        return {"passed": True, "actual_dx": dx, "odom_dx": odom_distance,
                "laser_front_before": before, "laser_front_after": after,
                "laser_front_delta_m": before - after,
                "native_laser_front_before": native_before, "native_laser_front_after": native_after,
                "scan_stamp_before_s": stamp(initial_scan), "scan_stamp_after_s": stamp(straight_scan),
                "ray_count": len(straight_scan.ranges), "drive_duration_sim_s": drive_seconds,
                "actual_turn_rad": delta_yaw, "odom_turn_rad": odom_yaw, "turn_duration_sim_s": turn_seconds,
                "drive_wheel_rad_s": drive_wheels, "turn_wheel_rad_s": turn_wheels,
                "stopped_wheel_rad_s": stop_wheels, "stop_drift_m": drift_m, "stop_drift_rad": drift_rad,
                "scan_messages": counts["scan"], "odom_messages": counts["odom"]}
    finally:
        try:
            if rclpy.ok():
                # Allow DDS to deliver more than one zero before the writer exits.
                for _ in range(6):
                    command.publish(Twist())
                    rclpy.spin_once(node, timeout_sec=.04)
                    time.sleep(.025)
        finally:
            for number, previous in previous_signals.items():
                signal.signal(number, previous)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    result(experiment())
