"""Check that the actual Harmonic arm and URDF forward kinematics agree."""

from collections import deque
import hashlib
import math
from pathlib import Path
import signal
import time
from uuid import uuid4

import _bootstrap  # Adds the installed course support directory.
from common import FORTRESS, ensure_running, result, topic, yaw

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


def angle_error(first, second):
    return abs((first - second + math.pi) % (2 * math.pi) - math.pi)


def tf_yaw(transform):
    value = transform.rotation
    return yaw({"orientation": {axis: getattr(value, axis) for axis in "xyzw"}})


def tf_xyz(transform):
    value = transform.translation
    return [value.x, value.y, value.z]


def pose_xyz(pose):
    value = pose.get("position", {})
    return [float(value.get(axis, 0)) for axis in "xyz"]


def native_joint():
    joints = topic("/gt/joints", timeout=4).get("joint", [])
    for item in joints:
        if item.get("name") == "shoulder" and "axis1" in item:
            axis = item["axis1"]
            values = float(axis.get("position", 0)), float(axis.get("velocity", 0))
            if all(math.isfinite(value) for value in values):
                return values
    raise RuntimeError(f"Missing native shoulder state: {joints}")


def native_links():
    pose_topic = "/model/urdf_arm/pose" if FORTRESS else "/gt/arm_link_poses"
    poses = topic(pose_topic, timeout=4).get("pose", [])
    links = {item.get("name", "").split("::")[-1]: item for item in poses}
    if not {"base", "arm", "tool"} <= links.keys():
        raise RuntimeError(f"Missing native link poses: {list(links)}")
    return links


def relative_to_base(links, name):
    base, child = links["base"], links[name]
    bx, by, bz = pose_xyz(base)
    x, y, z = pose_xyz(child)
    angle = yaw(base)
    return ([math.cos(angle) * (x - bx) + math.sin(angle) * (y - by),
             -math.sin(angle) * (x - bx) + math.cos(angle) * (y - by), z - bz],
            yaw(child) - angle)


def experiment():
    ensure_running()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("m25_observer_" + uuid4().hex[:8])
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    samples = deque(maxlen=120)
    count = 0

    def observe(message):
        nonlocal count
        try:
            index = message.name.index("shoulder")
            position, velocity = message.position[index], message.velocity[index]
        except (ValueError, IndexError):
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp > 0 and all(math.isfinite(value) for value in (position, velocity)):
            if not samples or stamp > samples[-1][0]:
                samples.append((stamp, position, velocity))
                count += 1

    node.create_subscription(JointState, "/joint_states", observe, qos_profile_sensor_data)
    publisher = node.create_publisher(Float64, "/gt/shoulder", 10)
    parameter_client = node.create_client(GetParameters, "/m25_state_publisher/get_parameters")

    def wait(predicate, description, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            rclpy.spin_once(node, timeout_sec=.025)
        raise RuntimeError("Timed out waiting for " + description)

    def verify_description():
        if not parameter_client.wait_for_service(timeout_sec=8):
            raise RuntimeError("Actual robot_state_publisher parameter services missing")
        future = parameter_client.call_async(GetParameters.Request(names=["robot_description", "use_sim_time"]))
        wait(future.done, "robot_state_publisher parameters")
        description, use_sim_time = future.result().values
        expected = Path("robot.urdf").read_text(encoding="utf-8")
        if description.string_value != expected or not use_sim_time.bool_value:
            raise AssertionError("robot_state_publisher did not load the exact practice URDF with simulation time")
        return hashlib.sha256(expected.encode("utf-8")).hexdigest()

    def stage(target):
        start_ns = samples[-1][0]
        deadline = time.monotonic() + 12
        next_send = 0
        settled = deque(maxlen=3)
        last_ns = start_ns
        while time.monotonic() < deadline and len(settled) < 3:
            now = time.monotonic()
            if now >= next_send:
                publisher.publish(Float64(data=target))
                next_send = now + .075
            rclpy.spin_once(node, timeout_sec=.025)
            stamp, position, velocity = samples[-1]
            if stamp <= last_ns:
                continue
            last_ns = stamp
            if abs(position - target) <= .035 and abs(velocity) <= .06:
                settled.append((stamp, position, velocity))
            else:
                settled.clear()
        if len(settled) < 3:
            raise RuntimeError(f"Shoulder failed to settle at {target} rad; last state {samples[-1]}")

        native_position, native_velocity = native_joint()
        links = native_links()
        arm_pos, arm_yaw = relative_to_base(links, "arm")
        tool_pos, tool_yaw = relative_to_base(links, "tool")
        if abs(native_position - target) > .035 or abs(native_velocity) > .06:
            raise AssertionError(f"Native joint disagrees with settled target: {native_position}, {native_velocity}")
        max_stamp = settled[-1][0]

        def same_stamp_tf():
            for stamp, position, velocity in reversed(samples):
                if stamp < max_stamp:
                    break
                if abs(position - native_position) > .015 or abs(velocity) > .06:
                    continue
                when = Time(nanoseconds=stamp)
                try:
                    arm = buffer.lookup_transform("base", "arm", when)
                    tool = buffer.lookup_transform("world", "tool", when)
                except TransformException:
                    continue
                if all(value.header.stamp.sec * 10**9 + value.header.stamp.nanosec == stamp
                       for value in (arm, tool)):
                    return (stamp, position, velocity, arm.transform, tool.transform)
            return None

        stamp, position, velocity, arm_tf, tool_tf = wait(same_stamp_tf, "TF at a native joint sample's original timestamp")
        measured_arm, measured_tool = tf_xyz(arm_tf), tf_xyz(tool_tf)
        if (math.dist(arm_pos, measured_arm) > .012 or math.dist(tool_pos, measured_tool) > .012
                or angle_error(arm_yaw, tf_yaw(arm_tf)) > .015
                or angle_error(tool_yaw, tf_yaw(tool_tf)) > .015
                or angle_error(arm_yaw, native_position) > .015
                or angle_error(tf_yaw(arm_tf), position) > .015):
            raise AssertionError(f"TF does not agree with native link poses: {arm_pos}, {tool_pos}, {measured_arm}, {measured_tool}")
        return {"target_rad": target, "joint_stamp_ns": stamp, "ros_joint_rad": position,
                "ros_velocity_rad_s": velocity, "native_joint_rad": native_position,
                "native_velocity_rad_s": native_velocity,
                "native_arm_xyz": arm_pos, "native_arm_yaw_rad": arm_yaw, "tf_arm_xyz": measured_arm,
                "tf_arm_yaw_rad": tf_yaw(arm_tf), "native_tool_xyz": tool_pos,
                "native_tool_yaw_rad": tool_yaw, "tf_tool_xyz": measured_tool,
                "tf_tool_yaw_rad": tf_yaw(tool_tf)}

    def cancel(signum, _frame):
        raise KeyboardInterrupt(f"Cancelled by signal {signum}")

    previous = {number: signal.signal(number, cancel)
                for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        description_hash = verify_description()
        wait(lambda: len(samples) >= 2 and publisher.get_subscription_count() > 0,
             "advancing native JointState and matched position-command bridge")
        rows = [stage(.65), stage(-.45), stage(0.)]
        if not (rows[0]["native_tool_xyz"][1] > .35 > rows[1]["native_tool_xyz"][1]
                and rows[1]["native_tool_xyz"][1] < -.15):
            raise AssertionError("Native tool did not move to opposite sides under the two targets")
        neutral_end = samples[-1][0] + 250_000_000
        wait(lambda: samples[-1][0] >= neutral_end, "stationary neutral observation window")
        stopped, stopped_speed = native_joint()
        if abs(stopped) > .035 or abs(stopped_speed) > .03 or abs(stopped - rows[-1]["native_joint_rad"]) > .02:
            raise AssertionError(f"Shoulder did not remain at rest near zero: {stopped}, {stopped_speed}")
        return {"passed": True, "robot_description_sha256": description_hash, "stages": rows,
                "neutral_hold_rad": stopped, "neutral_hold_velocity_rad_s": stopped_speed,
                "joint_state_messages": count}
    finally:
        try:
            if rclpy.ok():
                for _ in range(5):
                    publisher.publish(Float64(data=0.))
                    rclpy.spin_once(node, timeout_sec=.035)
                    time.sleep(.025)
        finally:
            for number, handler in previous.items():
                signal.signal(number, handler)
            listener.unregister()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    result(experiment())
