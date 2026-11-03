"""Verify real rail motion, typed actions, cancellation and live parameters."""

import argparse
import math
import signal
import time
from uuid import uuid4

import _bootstrap  # Adds the installed course support directory.
from common import ensure_running, result, xyz

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions

from gazeboteach_rail_interfaces.action import MoveRail
from gazeboteach_rail_interfaces.srv import ReadRail


def feedback_row(message):
    value = message.feedback
    return {"position_m": value.position_m, "velocity_m_s": value.velocity_m_s,
            "remaining_m": value.remaining_m, "command_m_s": value.command_m_s,
            "gain": value.gain, "sim_time_s": value.sim_time_s}


def wait_for_rail(node, state_client, timeout=18):
    """Require both native state progress and the server's actual matched command reader."""
    deadline = time.monotonic() + timeout
    first_stamp = None
    while time.monotonic() < deadline:
        if not state_client.wait_for_service(timeout_sec=.2):
            continue
        future = state_client.call_async(ReadRail.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
        if not future.done():
            state_client.remove_pending_request(future)
            continue
        sample = future.result()
        if sample.fresh and sample.command_matched and math.isfinite(sample.sim_time_s):
            if first_stamp is not None and sample.sim_time_s > first_stamp:
                return sample.sim_time_s
            first_stamp = sample.sim_time_s
        time.sleep(.06)
    raise RuntimeError("Rail readiness requires fresh advancing joint state and a matched velocity bridge")


def ready():
    rclpy.init()
    node = Node("rail_readiness_" + uuid4().hex[:8])
    try:
        client = node.create_client(ReadRail, "/gt/rail/state")
        return {"ready": True, "fresh_sim_time_s": wait_for_rail(node, client)}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def experiment():
    ensure_running()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("rail_client_" + uuid4().hex[:8])
    state_client = node.create_client(ReadRail, "/gt/rail/state")
    action = ActionClient(node, MoveRail, "/gt/rail/move")
    parameters = AsyncParameterClient(node, "/control_contract")
    active = None
    original_gain = None

    def cancel(signum, _frame):
        raise KeyboardInterrupt(f"Canceled by signal {signum}")

    previous_signals = {number: signal.signal(number, cancel)
                        for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}

    def complete(future, timeout=8):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for ROS 2 response")
        return future.result()

    def wait(predicate, description, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError("Timed out waiting for " + description)
            rclpy.spin_once(node, timeout_sec=.04)

    def read_state():
        sample = complete(state_client.call_async(ReadRail.Request()))
        if not sample.fresh or not sample.command_matched or not all(math.isfinite(value) for value in (
                sample.position_m, sample.velocity_m_s, sample.sim_time_s)):
            raise RuntimeError("Rail service has no fresh finite joint state")
        return {"position_m": sample.position_m, "velocity_m_s": sample.velocity_m_s,
                "sim_time_s": sample.sim_time_s, "active": sample.active}

    def get_gain():
        value = complete(parameters.get_parameters(["gain"])).values[0]
        if value.type != Parameter.Type.DOUBLE.value:
            raise RuntimeError("Server gain is not typed as double")
        return value.double_value

    def set_gain(value):
        return complete(parameters.set_parameters([Parameter("gain", value=value)])).results[0]

    def validate_feedback(rows, target):
        if not rows:
            raise AssertionError("No native joint feedback received")
        for index, row in enumerate(rows):
            error = target - row["position_m"]
            expected = min(.42, max(-.42, row["gain"] * error))
            if (not all(math.isfinite(value) for value in row.values())
                    or abs(row["remaining_m"] - error) > 1e-7
                    or abs(row["command_m_s"] - expected) > 1e-7
                    or (index and row["sim_time_s"] <= rows[index - 1]["sim_time_s"])):
                raise AssertionError(f"Feedback disagrees with control law or simulation time: {row}")

    try:
        wait_for_rail(node, state_client)
        if not action.wait_for_server(timeout_sec=8):
            raise RuntimeError("Rail service/action server unavailable")
        if not parameters.wait_for_services(timeout_sec=8):
            raise RuntimeError("Rail parameter services unavailable")

        original_gain = get_gain()
        valid, outside_range, wrong_type = set_gain(.8), set_gain(-1.), set_gain(1)
        if not valid.successful or outside_range.successful or wrong_type.successful or get_gain() != .8:
            raise AssertionError("Valid gain or rejected-update readback did not match")
        initial = read_state()
        native_initial = xyz("rail_carriage")
        if initial["active"] or abs(initial["position_m"] - native_initial[0]) > .025:
            raise AssertionError(f"Unexpected starting state: {initial}, native {native_initial}")
        rejected = complete(action.send_goal_async(MoveRail.Goal(target_m=2.)))
        if rejected.accepted:
            active = rejected
            raise AssertionError("Out-of-range goal was accepted")

        cancel_target = .94 if initial["position_m"] <= 0 else -.94
        cancel_rows = []
        active = complete(action.send_goal_async(
            MoveRail.Goal(target_m=cancel_target),
            feedback_callback=lambda message: cancel_rows.append(feedback_row(message)),
        ))
        if not active.accepted:
            raise AssertionError("Valid cancellation trial was rejected")
        wait(lambda: any(abs(row["position_m"] - initial["position_m"]) > .12
                         and abs(row["velocity_m_s"]) > .05 for row in cancel_rows),
             "the rail to move before cancellation")
        busy = complete(action.send_goal_async(MoveRail.Goal(target_m=0.)))
        if busy.accepted:
            complete(busy.cancel_goal_async())
            raise AssertionError("Concurrent goal was accepted while the rail was busy")
        cancel_reply = complete(active.cancel_goal_async())
        canceled = complete(active.get_result_async())
        active = None
        if not cancel_reply.goals_canceling or canceled.status != GoalStatus.STATUS_CANCELED:
            raise AssertionError(f"Action did not finish canceled: {canceled.status}")
        validate_feedback(cancel_rows, cancel_target)
        stopped = read_state()
        native_stopped = xyz("rail_carriage")
        after = None

        def stopped_later():
            nonlocal after
            after = read_state()
            return after["sim_time_s"] >= stopped["sim_time_s"] + .40

        wait(stopped_later, "a second stationary simulation window")
        native_later = xyz("rail_carriage")
        drift = abs(native_later[0] - native_stopped[0])
        if (abs(canceled.result.final_velocity_m_s) > .02 or stopped["active"] or after["active"] or
                abs(after["velocity_m_s"]) > .02 or drift > .018 or
                abs(native_stopped[0] - canceled.result.final_position_m) > .025 or
                abs(native_stopped[0] - native_initial[0]) < .12 or
                abs(native_stopped[0] - cancel_target) < .15):
            raise AssertionError(f"Cancellation did not stop mid-motion: {stopped}, {after}, native drift {drift}")

        target = after["position_m"] + (.34 if after["position_m"] <= .45 else -.34)
        success_rows = []
        active = complete(action.send_goal_async(
            MoveRail.Goal(target_m=target),
            feedback_callback=lambda message: success_rows.append(feedback_row(message)),
        ))
        if not active.accepted:
            raise AssertionError("Valid target trial was rejected")
        wait(lambda: len(success_rows) >= 3, "initial action feedback at gain 0.8")
        live = set_gain(1.8)
        if not live.successful:
            raise AssertionError("Valid live gain update was rejected")
        completed = complete(active.get_result_async(), timeout=20)
        active = None
        validate_feedback(success_rows, target)
        if (completed.status != GoalStatus.STATUS_SUCCEEDED or get_gain() != 1.8 or
                not all(any(row["gain"] == gain and .035 < abs(row["command_m_s"]) < .419
                            for row in success_rows) for gain in (.8, 1.8))):
            raise AssertionError("No successful target with actual unsaturated commands at both gains")
        final = read_state()
        native_final = xyz("rail_carriage")
        if (final["active"] or abs(native_final[0] - target) > .037 or
                abs(final["velocity_m_s"]) > .02 or abs(final["position_m"] - native_final[0]) > .025 or
                abs(completed.result.final_position_m - native_final[0]) > .025 or
                abs(completed.result.final_velocity_m_s) > .02):
            raise AssertionError(f"Final action result did not match stopped native cart: {final}, {native_final}")

        return {
            "passed": True, "initial_service_state": initial, "native_initial_xyz": native_initial,
            "out_of_range_goal_rejected": True, "concurrent_goal_rejected": True,
            "gain_range_rejected_reason": outside_range.reason, "gain_type_rejected_reason": wrong_type.reason,
            "accepted_initial_gain": .8, "accepted_live_gain": 1.8,
            "cancel_target_m": cancel_target, "cancel_status": canceled.status,
            "cancel_result_position_m": canceled.result.final_position_m,
            "cancel_result_velocity_m_s": canceled.result.final_velocity_m_s,
            "cancel_native_xyz": native_stopped, "cancel_stop_drift_m": drift, "cancel_feedback": cancel_rows,
            "success_target_m": target, "success_status": completed.status,
            "success_result_position_m": completed.result.final_position_m,
            "success_result_velocity_m_s": completed.result.final_velocity_m_s,
            "success_feedback": success_rows, "final_service_state": final, "native_final_xyz": native_final,
        }
    finally:
        try:
            if active is not None and active.accepted and rclpy.ok():
                complete(active.cancel_goal_async(), timeout=3)
                complete(active.get_result_async(), timeout=4)
        finally:
            try:
                if original_gain is not None and rclpy.ok():
                    restored = set_gain(original_gain)
                    if not restored.successful or get_gain() != original_gain:
                        raise RuntimeError("Original rail gain was not restored")
            finally:
                for number, previous in previous_signals.items():
                    signal.signal(number, previous)
                action.destroy()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ready", action="store_true", help="Wait only for the native rail/bridge handshake")
    result(ready() if parser.parse_args().ready else experiment())
