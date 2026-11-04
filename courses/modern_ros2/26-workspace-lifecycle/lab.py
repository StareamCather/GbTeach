"""Exercise installed ROS 2 lifecycle resources against a live native IMU source."""

from collections import deque
import hashlib
import math
from pathlib import Path
import signal
import sys
from statistics import median
import time
from uuid import uuid4

import _bootstrap  # Adds the installed course support directory.
from common import ensure_running, result, topic

from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetAvailableTransitions, GetState
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


NAME = "sensor_lifecycle"
RAW_TOPIC = "/gt/lifecycle/raw_imu"
OUTPUT_TOPIC = "/gt/lifecycle/sample"
UNCONFIGURED = State.PRIMARY_STATE_UNCONFIGURED
INACTIVE = State.PRIMARY_STATE_INACTIVE
ACTIVE = State.PRIMARY_STATE_ACTIVE


def angle(message):
    q = message.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def native_gyro():
    native = topic(RAW_TOPIC, timeout=4)
    velocity = native.get("angularVelocity", native.get("angular_velocity", {}))
    value = float(velocity.get("z", 0))
    if not math.isfinite(value) or not .40 < value < .70:
        raise AssertionError(f"Native rotor IMU does not report the expected physical rotation: {native}")
    return value


def experiment():
    ensure_running()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("m26_examiner_" + uuid4().hex[:8])
    raw = deque(maxlen=600)
    output = deque(maxlen=600)
    changes = []
    change = node.create_client(ChangeState, f"/{NAME}/change_state")
    get = node.create_client(GetState, f"/{NAME}/get_state")
    available = node.create_client(GetAvailableTransitions, f"/{NAME}/get_available_transitions")
    params = node.create_client(GetParameters, f"/{NAME}/get_parameters")

    def observe(queue, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp > 0 and all(math.isfinite(value) for value in (message.angular_velocity.z, angle(message))):
            queue.append((stamp, message))

    node.create_subscription(Imu, RAW_TOPIC, lambda message: observe(raw, message), qos_profile_sensor_data)
    node.create_subscription(Imu, OUTPUT_TOPIC, lambda message: observe(output, message), 64)

    def wait(predicate, description, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            rclpy.spin_once(node, timeout_sec=.025)
        raise RuntimeError("Timed out waiting for " + description)

    def done(future):
        wait(future.done, "a lifecycle service response")
        return future.result()

    def state():
        return done(get.call_async(GetState.Request())).current_state.id

    def transition(identifier, expected, success=True):
        response = done(available.call_async(GetAvailableTransitions.Request()))
        available_ids = sorted(item.transition.id for item in response.available_transitions)
        if (identifier in available_ids) != success:
            raise AssertionError(f"Unexpected transitions for this state: {available_ids}, requested {identifier}")
        request = ChangeState.Request()
        if success:
            request.transition.id = identifier
            submitted_as = "id"
        else:
            # Jazzy catches a missing label and returns success=false. Passing
            # an unavailable numeric ID raises from the stock service instead.
            request.transition.label = {Transition.TRANSITION_ACTIVATE: "activate",
                                        Transition.TRANSITION_CLEANUP: "cleanup"}[identifier]
            submitted_as = "label"
        response = done(change.call_async(request))
        actual = state()
        changes.append({"transition_id": identifier, "available_ids": available_ids,
                        "submitted_as": submitted_as, "accepted": response.success, "state_id": actual})
        if response.success != success or actual != expected:
            raise AssertionError(f"Unexpected lifecycle service result: {changes[-1]}")

    def reset():
        current = state()
        if current == ACTIVE:
            transition(Transition.TRANSITION_DEACTIVATE, INACTIVE)
            current = INACTIVE
        if current == INACTIVE:
            transition(Transition.TRANSITION_CLEANUP, UNCONFIGURED)
            current = UNCONFIGURED
        if current != UNCONFIGURED:
            raise RuntimeError(f"Cannot restore lifecycle node from state {current}")

    def check_endpoints(configured):
        def observe_graph():
            subscribers = {item.node_name for item in node.get_subscriptions_info_by_topic(RAW_TOPIC)}
            publishers = {item.node_name for item in node.get_publishers_info_by_topic(OUTPUT_TOPIC)}
            return ((NAME in subscribers) == configured and (NAME in publishers) == configured)
        wait(observe_graph, f"lifecycle endpoint allocation={configured}")

    def input_window(label, require_output=False):
        # Establish a watermark from fresh native sim-time data. Samples queued
        # before a transition cannot satisfy or invalidate the new state window.
        mark = raw[-1][0]
        wait(lambda: raw[-1][0] >= mark + 160_000_000, label + " transition drain")
        mark = raw[-1][0]

        def ready():
            samples = [item for item in raw if item[0] > mark]
            outputs = [item for item in output if item[0] > mark]
            if (len(samples) >= 8 and samples[-1][0] - samples[0][0] >= 360_000_000
                    and (not require_output or len(outputs) >= 6)):
                return samples, outputs
            return None

        samples, outputs = wait(ready, label + " advancing IMU observation")
        if require_output:
            def have_originals():
                raw_stamps = {stamp for stamp, _ in raw}
                return all(stamp in raw_stamps for stamp, _ in outputs)
            wait(have_originals, label + " original stamp join")
            by_time = {stamp: serialize_message(message) for stamp, message in raw}
            for stamp, message in outputs:
                if by_time.get(stamp) != serialize_message(message):
                    raise AssertionError(f"Lifecycle relay altered the IMU payload or timestamp at {stamp}")
        else:
            last_observed = raw[-1][0]
            wait(lambda: raw[-1][0] >= last_observed + 120_000_000, label + " delivery drain")
            outputs = [item for item in output if item[0] > mark]
            if outputs:
                raise AssertionError(f"Output appeared while {label}: {[stamp for stamp, _ in outputs]}")
        gyro = median(message.angular_velocity.z for _, message in samples)
        dyaw = (angle(samples[-1][1]) - angle(samples[0][1]) + math.pi) % (2 * math.pi) - math.pi
        if not .40 < gyro < .70 or not .10 < dyaw < 1.0:
            raise AssertionError(f"Raw sensor did not continue physical rotation in {label}: {gyro}, {dyaw}")
        return {"phase": label, "raw_samples": len(samples), "forwarded_samples": len(outputs),
                "first_raw_stamp_ns": samples[0][0], "last_raw_stamp_ns": samples[-1][0],
                "median_raw_gyro_z_rad_s": gyro, "raw_yaw_delta_rad": dyaw}

    def installed_package():
        prefix = Path(get_package_prefix("gazeboteach_lifecycle")).resolve()
        local = Path("ros_ws/install").resolve()
        if local not in prefix.parents or not (prefix / "lib/gazeboteach_lifecycle/sensor").is_file():
            raise AssertionError(f"Package was not resolved from the practice install space: {prefix}")
        if not params.wait_for_service(timeout_sec=8):
            raise RuntimeError("Installed lifecycle node parameter service missing")
        request = GetParameters.Request(names=["installed_source", "use_sim_time"])
        module, sim = done(params.call_async(request)).values
        loaded = Path(module.string_value).resolve()
        source = Path("ros_ws/src/gazeboteach_lifecycle/gazeboteach_lifecycle/sensor.py").resolve()
        if prefix not in loaded.parents or loaded == source or not sim.bool_value or loaded.read_bytes() != source.read_bytes():
            raise AssertionError(f"Running node did not load the current installed source: {loaded}")
        return {"prefix": str(prefix), "loaded_module": str(loaded),
                "module_sha256": hashlib.sha256(loaded.read_bytes()).hexdigest()}

    def cancel(signum, _frame):
        raise KeyboardInterrupt(f"Cancelled by signal {signum}")

    previous = {number: signal.signal(number, cancel)
                for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    ready = False
    try:
        if not all(client.wait_for_service(timeout_sec=8) for client in (change, get, available)):
            raise RuntimeError("Lifecycle services unavailable")
        ready = True
        package = installed_package()
        wait(lambda: len(raw) >= 3 and raw[-1][0] > raw[0][0], "live bridged Gazebo IMU")
        native = native_gyro()
        reset()
        check_endpoints(False)
        transition(Transition.TRANSITION_ACTIVATE, UNCONFIGURED, success=False)
        cycles = []
        for _ in range(2):
            transition(Transition.TRANSITION_CONFIGURE, INACTIVE)
            check_endpoints(True)
            inactive = input_window("configured_inactive")
            transition(Transition.TRANSITION_ACTIVATE, ACTIVE)
            active = input_window("active", require_output=True)
            transition(Transition.TRANSITION_CLEANUP, ACTIVE, success=False)
            transition(Transition.TRANSITION_DEACTIVATE, INACTIVE)
            check_endpoints(True)
            deactivated = input_window("deactivated")
            transition(Transition.TRANSITION_CLEANUP, UNCONFIGURED)
            check_endpoints(False)
            cleaned = input_window("cleaned_unconfigured")
            cycles.append({"configured": inactive, "active": active, "deactivated": deactivated, "cleaned": cleaned})
        return {"passed": True, "installed_package": package, "native_gyro_z_rad_s": native,
                "cycles": cycles, "transitions": changes, "final_state": state()}
    finally:
        original_error = sys.exc_info()[1]
        try:
            if ready and rclpy.ok():
                reset()
        except Exception as cleanup_error:
            if original_error is None:
                raise
            print(f"Lifecycle cleanup also failed: {cleanup_error}", file=sys.stderr)
        finally:
            for number, handler in previous.items():
                signal.signal(number, handler)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    result(experiment())
