"""Installed lifecycle relay for real Gazebo IMU samples."""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


RAW_TOPIC = "/gt/lifecycle/raw_imu"
OUTPUT_TOPIC = "/gt/lifecycle/sample"


class Sensor(LifecycleNode):
    def __init__(self):
        super().__init__("sensor_lifecycle")
        self.declare_parameter(
            "installed_source", str(Path(__file__).resolve()),
            ParameterDescriptor(read_only=True, description="Module loaded by the installed console entry point"),
        )
        self.output = None
        self.input = None

    def on_configure(self, state):
        self.output = self.create_lifecycle_publisher(Imu, OUTPUT_TOPIC, 16)
        self.input = self.create_subscription(Imu, RAW_TOPIC, self.forward, qos_profile_sensor_data)
        outcome = super().on_configure(state)
        if outcome != TransitionCallbackReturn.SUCCESS:
            self.release_resources()
        return outcome

    def forward(self, message):
        if self.output is not None:
            # LifecyclePublisher permits data delivery only while active.
            self.output.publish(message)

    def release_resources(self):
        if self.input is not None:
            self.destroy_subscription(self.input)
            self.input = None
        if self.output is not None:
            self.destroy_lifecycle_publisher(self.output)
            self.output = None

    def on_cleanup(self, state):
        outcome = super().on_cleanup(state)
        if outcome == TransitionCallbackReturn.SUCCESS:
            self.release_resources()
        return outcome

    def on_shutdown(self, state):
        outcome = super().on_shutdown(state)
        self.release_resources()
        return outcome


def main(args=None):
    rclpy.init(args=args)
    node = Sensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.release_resources()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
