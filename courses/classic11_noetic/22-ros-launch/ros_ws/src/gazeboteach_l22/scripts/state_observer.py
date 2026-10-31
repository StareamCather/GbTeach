#!/usr/bin/env python3
"""Publish selected actual ModelStates values; the original message has no header."""
import threading
import time
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from common import identity, pose_twist, publish


class Observer:
    def __init__(self):
        self.lock, self.sequence, self.previous = threading.Lock(), 0, 0
        self.info = identity()
        self.model = rospy.get_param("~model_name")
        self.rate = float(rospy.get_param("~publish_wall_hz"))
        if self.model != "launch_slider" or self.rate != 50:
            raise RuntimeError("Unexpected observer model or publish rate")
        self.publisher = rospy.Publisher("feedback", String, queue_size=20, latch=True)
        self.status = rospy.Publisher("status", String, queue_size=1, latch=True)
        self.subscription = rospy.Subscriber("model_states", ModelStates, self.receive, queue_size=10, buff_size=4194304)
        publish(self.status, {**self.info, "parameters": {"model_name": self.model, "publish_wall_hz": self.rate},
            "resolved_names": {"input_topic": rospy.resolve_name("model_states"), "feedback_topic": rospy.resolve_name("feedback"), "status_topic": rospy.resolve_name("status")}, "state": "subscribed"})

    def receive(self, message):
        wall = time.monotonic_ns()
        with self.lock:
            self.sequence += 1
            if wall-self.previous < round(1e9/self.rate):
                return
            if self.model not in message.name:
                return
            index = message.name.index(self.model)
            if index >= len(message.pose) or index >= len(message.twist):
                return
            self.previous = wall
            publish(self.publisher, {"model_name": self.model, "source_type": "gazebo_msgs/ModelStates",
                "source_has_header": False, "source_stamp_ns": None, "source_message_sequence": self.sequence,
                "observer_receipt_monotonic_ns": wall, "observer_receipt_ros_time_ns": rospy.Time.now().to_nsec(),
                "node": rospy.get_name(), "state": pose_twist(message.pose[index], message.twist[index])})


if __name__ == "__main__":
    rospy.init_node("observer")
    node = Observer()
    rospy.spin()
