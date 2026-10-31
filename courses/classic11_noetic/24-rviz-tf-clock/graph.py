#!/usr/bin/env python3
"""Observe actual ROS channels and a persistent stock tf2 listener, without retiming."""
from collections import deque
import threading
import time


class Graph:
    def __init__(self, config):
        import rospy
        import tf2_ros
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        self.rospy, self.config = rospy, config
        self.lock = threading.RLock()
        self.rows, self.clear_events = deque(maxlen=30000), []
        self.sequence = 0
        owner = self
        class AuditedBuffer(tf2_ros.Buffer):
            def clear(self):
                with owner.lock:
                    owner.clear_events.append({"receive_monotonic_ns": time.monotonic_ns(), "ros_time_ns": rospy.Time.now().to_nsec()})
                return super().clear()
        self.buffer = AuditedBuffer(cache_time=rospy.Duration(10), debug=False)
        self.listener = tf2_ros.TransformListener(self.buffer, queue_size=100)
        self.subscriptions = [
            rospy.Subscriber("/clock", Clock, self.clock, queue_size=2000),
            rospy.Subscriber("/lesson24/joint_states", JointState, self.joints, queue_size=100),
            rospy.Subscriber("/tf", TFMessage, lambda message: self.transforms("tf", message), queue_size=100),
            rospy.Subscriber("/tf_static", TFMessage, lambda message: self.transforms("tf_static", message), queue_size=10)]

    def add(self, source, stamp, data, message):
        with self.lock:
            self.sequence += 1
            row = {"event_id": self.sequence, "source": source, "stamp_ns": stamp,
                "receive_monotonic_ns": time.monotonic_ns(), "ros_time_at_receive_ns": self.rospy.Time.now().to_nsec(),
                "callerid": getattr(message, "_connection_header", {}).get("callerid"),
                "latching": getattr(message, "_connection_header", {}).get("latching"), "data": data}
            self.rows.append(row)
            return row

    def clock(self, message):
        self.add("clock", message.clock.to_nsec(), {"clock_ns": message.clock.to_nsec()}, message)

    def joints(self, message):
        self.add("joint_state", message.header.stamp.to_nsec(), {"header_seq": message.header.seq,
            "header_frame_id": message.header.frame_id, "name": list(message.name), "position": list(message.position),
            "velocity": list(message.velocity), "effort": list(message.effort)}, message)

    def transforms(self, source, message):
        return self.add(source, None, {"transforms": [{"stamp_ns": value.header.stamp.to_nsec(),
            "parent": value.header.frame_id, "child": value.child_frame_id,
            "translation": [value.transform.translation.x, value.transform.translation.y, value.transform.translation.z],
            "rotation": [value.transform.rotation.x, value.transform.rotation.y, value.transform.rotation.z, value.transform.rotation.w]}
            for value in message.transforms]}, message)

    def frame(self, name):
        return self.config["tf_prefix"]+"/"+name if self.config["tf_prefix"] else name

    def mark(self):
        with self.lock:
            return self.sequence

    def since(self, marker=0):
        with self.lock:
            return [value for value in self.rows if value["event_id"] > marker]

    def latest(self, source):
        with self.lock:
            return next((value for value in reversed(self.rows) if value["source"] == source), None)

    def lookup(self, child, stamp_ns=0, target="world"):
        stamp = self.rospy.Time(stamp_ns//1000000000, stamp_ns%1000000000)
        value = self.buffer.lookup_transform(self.frame(target), self.frame(child), stamp, self.rospy.Duration(0))
        return {"parent": value.header.frame_id, "child": value.child_frame_id, "stamp_ns": value.header.stamp.to_nsec(),
            "position": [value.transform.translation.x, value.transform.translation.y, value.transform.translation.z],
            "orientation": [value.transform.rotation.x, value.transform.rotation.y, value.transform.rotation.z, value.transform.rotation.w]}

    def ready(self, timeout=15):
        deadline, error = time.monotonic()+timeout, ""
        while time.monotonic() < deadline:
            try:
                rows = {name: self.latest(name) for name in ("clock", "joint_state", "tf", "tf_static")}
                if not all(rows.values()) or rows["clock"]["stamp_ns"] <= 0:
                    raise RuntimeError("Waiting for actual clock/JointState/TF/static channels")
                values = {name: self.lookup(name) for name in ("base_link", "arm1", "arm2", "tool_link")}
                return {"latest_events": rows, "lookups": values,
                    "listener_identity": id(self.listener), "buffer_identity": id(self.buffer)}
            except Exception as exception:
                error = str(exception)
            time.sleep(.01)
        raise RuntimeError("Bounded actual TF graph readiness failed: "+error)

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()
        self.listener.unregister()
