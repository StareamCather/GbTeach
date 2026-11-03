"""Typed ROS 2 controller for the native Harmonic prismatic rail."""

import math
import signal
from threading import Event, Lock
import time

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from gazeboteach_rail_interfaces.action import MoveRail
from gazeboteach_rail_interfaces.srv import ReadRail


LIMIT_M = 1.0
SPEED_LIMIT_M_S = .42


class RailController(Node):
    def __init__(self):
        super().__init__("control_contract")
        self.lock = Lock()
        self.shutdown_requested = Event()
        self.active = False
        self.state = None
        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("gain", 1.2, ParameterDescriptor(
            description="Proportional gain in s^-1 for rail velocity control",
            floating_point_range=[FloatingPointRange(from_value=.3, to_value=3., step=0.)],
        ))
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.command = self.create_publisher(Float64, "/gt/rail/velocity_command", 10)
        self.create_subscription(JointState, "/gt/rail/joints", self.observe,
                                 qos_profile_sensor_data, callback_group=self.callback_group)
        self.create_service(ReadRail, "/gt/rail/state", self.read, callback_group=self.callback_group)
        self.action = ActionServer(
            self, MoveRail, "/gt/rail/move", execute_callback=self.execute,
            goal_callback=self.accept_goal, cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=self.callback_group,
        )

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == "gain" and (
                    type(parameter.value) is not float or not math.isfinite(parameter.value)
                    or not .3 <= parameter.value <= 3.):
                return SetParametersResult(successful=False, reason="gain must be a finite double in [0.3, 3.0]")
        return SetParametersResult(successful=True)

    def observe(self, message):
        try:
            index = message.name.index("rail_joint")
            position, velocity = message.position[index], message.velocity[index]
        except (ValueError, IndexError):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if not all(math.isfinite(value) for value in (position, velocity, stamp)):
            return
        with self.lock:
            if self.state is None or stamp > self.state[2]:
                self.state = (position, velocity, stamp, time.monotonic())

    def snapshot(self):
        with self.lock:
            return self.state

    def fresh(self, sample):
        return sample is not None and time.monotonic() - sample[3] < 1.0

    def read(self, _request, response):
        with self.lock:
            sample, response.active = self.state, self.active
        response.fresh = self.fresh(sample)
        response.command_matched = self.command.get_subscription_count() > 0
        if sample:
            response.position_m, response.velocity_m_s, response.sim_time_s = sample[:3]
        return response

    def accept_goal(self, request):
        if not math.isfinite(request.target_m) or abs(request.target_m) > LIMIT_M:
            return GoalResponse.REJECT
        with self.lock:
            if (self.active or self.shutdown_requested.is_set() or not self.fresh(self.state)
                    or not self.command.get_subscription_count()):
                return GoalResponse.REJECT
            self.active = True
        return GoalResponse.ACCEPT

    def send_velocity(self, velocity):
        self.command.publish(Float64(data=float(velocity)))

    def stop_and_confirm(self):
        deadline = time.monotonic() + 2.0
        last_stamp = self.snapshot()[2]
        slow_samples = 0
        while time.monotonic() < deadline and not self.shutdown_requested.is_set():
            self.send_velocity(0.)
            sample = self.snapshot()
            if self.fresh(sample) and sample[2] > last_stamp:
                last_stamp = sample[2]
                slow_samples = slow_samples + 1 if abs(sample[1]) <= .02 else 0
                if slow_samples >= 3:
                    return sample
            self.shutdown_requested.wait(.025)
        raise RuntimeError("No fresh stationary joint samples after zero command")

    def execute(self, handle):
        target = handle.request.target_m
        result = MoveRail.Result()
        try:
            deadline = time.monotonic() + 18
            start = self.snapshot()[2]
            last_stamp = start
            while not handle.is_cancel_requested and not self.shutdown_requested.is_set():
                sample = self.snapshot()
                if not self.fresh(sample) or time.monotonic() >= deadline or sample[2] - start > 12:
                    raise RuntimeError("Rail feedback stale or movement deadline reached")
                position, velocity, stamp = sample[:3]
                error = target - position
                if abs(error) <= .022:
                    break
                if stamp > last_stamp:
                    gain = self.get_parameter("gain").value
                    command = min(SPEED_LIMIT_M_S, max(-SPEED_LIMIT_M_S, gain * error))
                    self.send_velocity(command)
                    handle.publish_feedback(MoveRail.Feedback(
                        position_m=position, velocity_m_s=velocity, remaining_m=error,
                        command_m_s=command, gain=gain, sim_time_s=stamp,
                    ))
                    last_stamp = stamp
                self.shutdown_requested.wait(.015)
            stopped = self.stop_and_confirm()
            result.final_position_m, result.final_velocity_m_s = stopped[:2]
            if handle.is_cancel_requested:
                handle.canceled()
                result.detail = "Canceled after fresh joint state confirmed stop"
            elif abs(result.final_position_m - target) <= .035:
                handle.succeed()
                result.detail = "Target reached and stop confirmed"
            else:
                handle.abort()
                result.detail = "Stopped outside target tolerance"
        except Exception as error:
            self.send_velocity(0.)
            sample = self.snapshot()
            if sample:
                result.final_position_m, result.final_velocity_m_s = sample[:2]
            result.detail = str(error)
            handle.abort()
            self.get_logger().error(result.detail)
        finally:
            self.send_velocity(0.)
            with self.lock:
                self.active = False
        return result


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RailController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    def stop(_signum, _frame):
        node.shutdown_requested.set()

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, stop)
    try:
        while rclpy.ok() and not node.shutdown_requested.is_set():
            executor.spin_once(timeout_sec=.10)
    finally:
        node.shutdown_requested.set()
        for _ in range(5):
            node.send_velocity(0.)
            time.sleep(.025)
        executor.shutdown(timeout_sec=2)
        node.action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
