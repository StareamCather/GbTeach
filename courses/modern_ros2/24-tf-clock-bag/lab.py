"""Capture, validate and replay a real Gazebo TF and simulation-time experiment."""

from collections import defaultdict
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from uuid import uuid4

import _bootstrap  # Adds the installed course support directory.
from common import ensure_running, pose, result, service, topic, yaw

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbag2_interfaces.srv import Resume
import rosbag2_py
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


CLOCK, DYNAMIC, STATIC = "/clock", "/tf", "/tf_static"
CHANNELS = {
    CLOCK: (Clock, "rosgraph_msgs/msg/Clock"),
    DYNAMIC: (TFMessage, "tf2_msgs/msg/TFMessage"),
    STATIC: (TFMessage, "tf2_msgs/msg/TFMessage"),
}
MOUNT = (.45, .20, .15, -.35)
WORLD, RIG, SENSOR = "world", "m24_rig", "m24_sensor"
STATE_TOPIC = "/world/default/stats"
STEP_COUNT, STEP_NS = 20, 1_000_000


def stamp_ns(value):
    return value.sec * 10**9 + value.nanosec


def spin_until(executor, predicate, description, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        observed = predicate()
        if observed:
            return observed
        executor.spin_once(timeout_sec=.025)
    raise RuntimeError("Timed out waiting for " + description)


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(.025, max(0, deadline - time.monotonic())))


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def heading(quaternion):
    return yaw({"orientation": {key: getattr(quaternion, key) for key in "xyzw"}})


def angle_error(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def native_stats():
    message = topic(STATE_TOPIC, timeout=3)
    stamp = message.get("simTime", message.get("sim_time"))
    if not isinstance(stamp, dict) or "iterations" not in message:
        raise RuntimeError(f"Incomplete native WorldStatistics: {message}")
    return {"sim_ns": int(stamp.get("sec", 0)) * 10**9 + int(stamp.get("nsec", 0)),
            "iterations": int(message["iterations"]), "paused": message.get("paused", False),
            "stepping": message.get("stepping", False)}


def set_pause(paused):
    service("/world/default/control", "gz.msgs.WorldControl", "pause: " + str(paused).lower())


def wait_stats(executor, predicate, description):
    def sample():
        observed = native_stats()
        return observed if predicate(observed) else None
    return spin_until(executor, sample, description)


def clock_trial(executor, node, clock_ns):
    set_pause(True)
    baseline = wait_stats(executor, lambda row: row["paused"] and not row["stepping"], "paused native world")
    spin_until(executor, lambda: clock_ns() == baseline["sim_ns"], "ROS clock at exact paused world time")
    spin_for(executor, .22)
    stationary = native_stats()
    if stationary != baseline or clock_ns() != baseline["sim_ns"] or node.get_clock().now().nanoseconds != clock_ns():
        raise AssertionError(f"Native or ROS clock moved during pause: {baseline}, {stationary}")
    service("/world/default/control", "gz.msgs.WorldControl", f"pause: true, multi_step: {STEP_COUNT}")
    target = baseline["iterations"] + STEP_COUNT
    after = wait_stats(executor, lambda row: row["paused"] and not row["stepping"]
                       and row["iterations"] >= target, "completed native multistep")
    spin_until(executor, lambda: clock_ns() == after["sim_ns"]
               and node.get_clock().now().nanoseconds == clock_ns(), "ROS sim-time node at native step end")
    if after["iterations"] != target or after["sim_ns"] - baseline["sim_ns"] != STEP_COUNT * STEP_NS:
        raise AssertionError(f"Native step delta differs from requested physics steps: {baseline}, {after}")
    set_pause(False)
    spin_until(executor, lambda: clock_ns() > after["sim_ns"], "clock resumed")
    return {"paused_stats": baseline, "step_stats": after, "actual_step_count": STEP_COUNT,
            "ros_step_delta_ns": after["sim_ns"] - baseline["sim_ns"]}


def set_rig(position, heading_rad):
    x, y, z = position
    half = heading_rad / 2
    request = (f'name: "tf_rig", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{z: {math.sin(half)}, w: {math.cos(half)}}}')
    service("/world/default/set_pose", "gz.msgs.Pose", request)


def lookup(buffer, when):
    try:
        return buffer.lookup_transform(WORLD, SENSOR, Time(nanoseconds=when))
    except TransformException:
        return None


def check_tf(transform, native, when):
    position = native.get("position", {})
    rig_xyz = [float(position.get(axis, 0)) for axis in "xyz"]
    rig_yaw = yaw(native)
    cx, cy, cz, mount_yaw = MOUNT
    expected = [rig_xyz[0] + math.cos(rig_yaw) * cx - math.sin(rig_yaw) * cy,
                rig_xyz[1] + math.sin(rig_yaw) * cx + math.cos(rig_yaw) * cy,
                rig_xyz[2] + cz]
    measured = xyz(transform.transform.translation)
    measured_yaw = heading(transform.transform.rotation)
    if (math.dist(expected, measured) > .003 or angle_error(measured_yaw, rig_yaw + mount_yaw) > .003
            or stamp_ns(transform.header.stamp) != when):
        raise AssertionError(f"Wrong timed TF composition: {measured}, {expected}, {when}")
    return {"stamp_ns": when, "native_rig_xyz": rig_xyz, "native_rig_yaw_rad": rig_yaw,
            "sensor_world_xyz": measured, "sensor_world_yaw_rad": measured_yaw}


def write_and_read_bag(path, rows):
    storage = rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("cdr", "cdr")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage, converter)
    try:
        for name, (_, type_name) in CHANNELS.items():
            writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=name, type=type_name,
                                                        serialization_format="cdr"))
        for when, name, payload in rows:
            writer.write(name, payload, when)
    finally:
        writer.close()

    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    reread = []
    while reader.has_next():
        name, payload, when = reader.read_next()
        reread.append((when, name, bytes(payload)))
    if reread != rows:
        raise AssertionError("The sqlite3 rosbag2 reader did not return identical CDR rows and storage stamps")
    return reread


def replay(executor, live, clock_ns, clock_publishers, bag, expected, token, observations):
    namespace = "/gt/m24/replay_" + token
    player_name = "m24_player_" + token
    remaps = [f"{name}:={namespace}{name}" for name in CHANNELS]
    observer = Node("m24_replay_reader_" + token, cli_args=["--ros-args", "-r", remaps[1], "-r", remaps[2]])
    executor.add_node(observer)
    buffer = Buffer()
    listener = TransformListener(buffer, observer, spin_thread=False)
    received = defaultdict(list)
    subscriptions = [
        observer.create_subscription(kind, namespace + name,
            lambda payload, name=name: received[name].append(bytes(payload)),
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL if name == STATIC
                       else DurabilityPolicy.VOLATILE), raw=True)
        for name, (kind, _) in CHANNELS.items()
    ]
    resume = observer.create_client(Resume, "/" + player_name + "/resume")
    arguments = ["ros2", "bag", "play", str(bag), "--start-paused", "--disable-keyboard-controls",
                 "--wait-for-all-acked", "2000", "--qos-profile-overrides-path", str(Path("replay_qos.yaml").resolve()),
                 "--remap", "__node:=" + player_name, *remaps]
    log_path = bag.parent / (bag.name + "-player.log")
    process = None
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(arguments, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            spin_until(executor, lambda: process.poll() is not None or
                       (resume.service_is_ready() and all(sub.get_publisher_count() == 1 for sub in subscriptions)),
                       "paused player discovery", timeout=10)
            if process.poll() is not None or any(received.values()):
                raise RuntimeError(f"Player did not remain paused awaiting reader discovery: {log_path}")
            if clock_publishers(live) != expected["live_publishers"]:
                raise AssertionError("Player changed the live /clock writer set")
            live_before = clock_ns()
            if live_before <= expected["last_clock_ns"]:
                raise AssertionError("Live simulation did not advance beyond the captured bag")
            future = resume.call_async(Resume.Request())
            spin_until(executor, future.done, "rosbag2 resume response")
            future.result()
            spin_until(executor, lambda: process.poll() is not None, "rosbag2 playback finish", timeout=12)
            spin_until(executor, lambda: all(len(received[name]) >= len(expected[name]) for name in CHANNELS),
                       "all replay CDR rows after player exit", timeout=4)
            spin_for(executor, .12)
            if (process.returncode != 0 or any(received[name] != expected[name] for name in CHANNELS)
                    or clock_publishers(live) != expected["live_publishers"] or clock_ns() <= live_before):
                raise AssertionError(f"Replayed CDR or live-clock ownership mismatch: {log_path}")
            restored_tf = []
            for when, native in observations:
                transform = spin_until(executor, lambda: lookup(buffer, when), "archived-time replay TF lookup")
                restored_tf.append(check_tf(transform, native, when))
            return {"namespace": namespace, "received_counts": {name: len(received[name]) for name in CHANNELS},
                    "live_clock_before_ns": live_before, "live_clock_after_ns": clock_ns(),
                    "live_clock_publishers_unchanged": True, "historical_tf": restored_tf,
                    "player_log": str(log_path)}
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        listener.unregister()
        executor.remove_node(observer)
        observer.destroy_node()


def experiment():
    original = native_stats()
    rclpy.init()
    executor = SingleThreadedExecutor()
    token = uuid4().hex[:8]
    node = Node("m24_live_" + token, parameter_overrides=[Parameter("use_sim_time", value=True)])
    executor.add_node(node)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    broadcaster = StaticTransformBroadcaster(node)
    latest = {CLOCK: 0, DYNAMIC: 0}
    regressions = []
    last_record_clock = 0
    rows = []
    recording = False

    def source_data(name, payload):
        nonlocal last_record_clock
        message = deserialize_message(payload, CHANNELS[name][0])
        if name == CLOCK:
            when = stamp_ns(message.clock)
            if when < latest[CLOCK]:
                regressions.append((latest[CLOCK], when))
            latest[CLOCK] = when
            if when - last_record_clock < 40_000_000:
                return
            last_record_clock = when
        else:
            if len(message.transforms) != 1:
                return
            transform = message.transforms[0]
            frames = (transform.header.frame_id, transform.child_frame_id)
            if frames != ((WORLD, RIG) if name == DYNAMIC else (RIG, SENSOR)):
                return
            when = stamp_ns(transform.header.stamp)
            if name == DYNAMIC:
                if when <= latest[DYNAMIC]:
                    return
                latest[DYNAMIC] = when
        if recording and when > 0:
            rows.append((when, name, bytes(payload)))

    for name, (kind, _) in CHANNELS.items():
        qos = (QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if name == STATIC else qos_profile_sensor_data)
        node.create_subscription(kind, name, lambda payload, name=name: source_data(name, payload), qos, raw=True)

    def clock_publishers(reader):
        return sorted((item.node_name, item.node_namespace, tuple(item.endpoint_gid))
                      for item in reader.get_publishers_info_by_topic(CLOCK))

    try:
        ensure_running()
        spin_until(executor, lambda: min(latest.values()) > 0, "native clock and dynamic TF bridge")
        clock = clock_trial(executor, node, lambda: latest[CLOCK])
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = RIG, SENSOR
        mount.header.stamp = node.get_clock().now().to_msg()
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = MOUNT[:3]
        mount.transform.rotation.z, mount.transform.rotation.w = math.sin(MOUNT[3] / 2), math.cos(MOUNT[3] / 2)
        recording = True
        broadcaster.sendTransform(mount)
        spin_until(executor, lambda: any(row[1] == STATIC for row in rows), "static transform capture")

        observations = []
        live_tf = []
        for position, angle in (((-.65, -.55, .65), 0.), ((.65, .35, .65), math.pi / 2)):
            set_rig(position, angle)
            native = pose("tf_rig")
            native_position = [float(native.get("position", {}).get(axis, 0)) for axis in "xyz"]
            if math.dist(native_position, position) > .003 or angle_error(yaw(native), angle) > .003:
                raise AssertionError(f"Native rig did not reach requested pose: {native}")
            end = latest[CLOCK] + 350_000_000
            spin_until(executor, lambda: latest[CLOCK] >= end, "recording held rig pose")

            def timed_transform():
                when = latest[DYNAMIC]
                value = lookup(buffer, when)
                return (when, value) if value else None

            when, observed = spin_until(executor, timed_transform, "stamped live sensor transform")
            observations.append((when, native))
            live_tf.append(check_tf(observed, native, when))

        def capture_ready():
            stamps = {name: [when for when, row_name, _ in rows if row_name == name] for name in CHANNELS}
            if (len(stamps[CLOCK]) >= 10 and len(stamps[DYNAMIC]) >= 10 and len(stamps[STATIC]) == 1
                    and stamps[CLOCK][-1] - stamps[CLOCK][0] >= 700_000_000
                    and stamps[DYNAMIC][-1] - stamps[DYNAMIC][0] >= 600_000_000):
                return {name: values[-1] - values[0] for name, values in stamps.items()}
            return None

        # A simulation-time hold can finish before the bridge delivers enough
        # samples. Keep observing the second pose until actual data covers the
        # required span, rather than assuming a clock publication frequency.
        capture_spans = spin_until(executor, capture_ready, "complete native TF and bridged clock capture", timeout=12)
        recording = False

        rows.sort(key=lambda row: (row[0], row[1]))
        path = Path("recordings") / ("m24-" + token)
        path.parent.mkdir(exist_ok=True)
        expected = defaultdict(list)
        for when, name, payload in write_and_read_bag(path, rows):
            message = deserialize_message(payload, CHANNELS[name][0])
            source = stamp_ns(message.clock if name == CLOCK else message.transforms[0].header.stamp)
            if source != when:
                raise AssertionError(f"Original timestamp changed for {name}: {source}, {when}")
            expected[name].append(payload)
        if len(expected[CLOCK]) < 10 or len(expected[DYNAMIC]) < 10 or len(expected[STATIC]) != 1:
            raise AssertionError(f"Capture incomplete: {[(name, len(expected[name])) for name in CHANNELS]}")
        expected["live_publishers"] = clock_publishers(node)
        if len(expected["live_publishers"]) != 1:
            raise AssertionError("Expected one and only one live /clock publisher")
        expected["last_clock_ns"] = max(when for when, name, _ in rows if name == CLOCK)
        spin_until(executor, lambda: latest[CLOCK] > expected["last_clock_ns"], "live clock beyond capture")
        replay_report = replay(executor, node, lambda: latest[CLOCK], clock_publishers,
                               path, expected, token, observations)
        if regressions:
            raise AssertionError(f"Live /clock regressed during capture/replay: {regressions[:5]}")
        return {"passed": True, "clock": clock, "live_tf": live_tf, "bag": str(path),
                "bag_counts": {name: len(expected[name]) for name in CHANNELS},
                "bag_stamp_spans_ns": capture_spans,
                "bag_cdr_sha256": {name: hashlib.sha256(b"".join(expected[name])).hexdigest() for name in CHANNELS},
                "replay": replay_report}
    finally:
        try:
            set_pause(original["paused"])
        finally:
            listener.unregister()
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    result(experiment())
