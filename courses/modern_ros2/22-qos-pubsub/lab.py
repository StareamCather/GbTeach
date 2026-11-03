"""Real ROS 2 DDS QoS controls with independently observed Harmonic time."""

import time
from uuid import uuid4

import _bootstrap  # Adds the installed course support directory.
from common import ensure_running, result, topic

import rclpy
from rclpy.duration import Duration
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSPolicyKind, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int32


def qos(depth, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE):
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=reliability, durability=durability)


def spin_until(executor, predicate, description, seconds=6):
    deadline = time.monotonic() + seconds
    while not predicate():
        if time.monotonic() >= deadline:
            raise RuntimeError("Timed out waiting for " + description)
        executor.spin_once(timeout_sec=.03)


def spin_window(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(.025, max(0, deadline - time.monotonic())))


def profile_details(endpoint):
    profile = endpoint.qos_profile
    return {"node": endpoint.node_name, "namespace": endpoint.node_namespace,
            "type": endpoint.topic_type, "reliability": profile.reliability.name,
            "durability": profile.durability.name}


def reliability_trial(executor, source, compatible, incompatible, name):
    good, bad, events = [], [], []
    best_effort = qos(16, ReliabilityPolicy.BEST_EFFORT)
    publisher = source.create_publisher(Int32, name, best_effort)
    compatible.create_subscription(Int32, name, lambda message: good.append(message.data), best_effort)

    def on_incompatible(event):
        events.append({"total_count": event.total_count,
                       "last_policy_kind": QoSPolicyKind(event.last_policy_kind).name})

    incompatible.create_subscription(
        Int32, name, lambda message: bad.append(message.data), qos(16),
        event_callbacks=SubscriptionEventCallbacks(incompatible_qos=on_incompatible),
    )
    spin_until(
        executor,
        lambda: publisher.get_subscription_count() == 1 and bool(events)
        and len(source.get_subscriptions_info_by_topic(name)) == 2,
        "one matched reader, two discovered readers and the DDS incompatible event",
    )
    writers = [profile_details(item) for item in source.get_publishers_info_by_topic(name)]
    readers = [profile_details(item) for item in source.get_subscriptions_info_by_topic(name)]
    if (len(writers) != 1 or writers[0]["node"] != source.get_name() or
            writers[0]["reliability"] != "BEST_EFFORT" or
            {row["node"]: row["reliability"] for row in readers} != {
                compatible.get_name(): "BEST_EFFORT", incompatible.get_name(): "RELIABLE"} or
            any(row["type"] != "std_msgs/msg/Int32" for row in writers + readers)):
        raise AssertionError(f"Unexpected live QoS graph endpoints: {writers}, {readers}")

    sent = list(range(12))
    for number in sent:
        publisher.publish(Int32(data=number))
        spin_window(executor, .045)
    spin_window(executor, .4)
    if (len(set(good)) < 8 or good != sorted(set(good)) or not set(good) <= set(sent) or bad or
            not any(event["last_policy_kind"] == "RELIABILITY" and event["total_count"] > 0
                    for event in events)):
        raise AssertionError(f"Reliability control failed: {good}, {bad}, {events}")
    return {"topic": name, "sent": sent, "compatible_received": good,
            "incompatible_received": bad, "incompatible_events": events,
            "writer_endpoints": writers, "reader_endpoints": readers, "matched_readers": 1}


def durability_trial(executor, source, transient, volatile, name):
    history = qos(3, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = source.create_publisher(Int32, name, history)
    before_join = [10, 11, 12]
    if publisher.get_subscription_count() != 0:
        raise RuntimeError("Late-join test requires a fresh topic with no readers")
    for number in before_join:
        publisher.publish(Int32(data=number))

    retained, live_only = [], []
    transient.create_subscription(Int32, name, lambda message: retained.append(message.data), history)
    volatile.create_subscription(Int32, name, lambda message: live_only.append(message.data), qos(8))
    spin_until(executor, lambda: publisher.get_subscription_count() == 2 and len(retained) >= 3,
               "two matches and replay of the retained writer history")
    spin_window(executor, .35)
    if retained != before_join or live_only:
        raise AssertionError(f"Unexpected late-join history: {retained}, {live_only}")
    publisher.publish(Int32(data=13))
    spin_until(executor, lambda: 13 in retained and 13 in live_only, "live delivery to both late readers")
    if retained != before_join + [13] or live_only != [13]:
        raise AssertionError(f"Unexpected live delivery after join: {retained}, {live_only}")
    return {"topic": name, "published_before_join": before_join, "published_after_join": 13,
            "transient_local_received": retained, "volatile_received": live_only}


def depth_trial(executor, source, shallow, deep, name):
    publisher = source.create_publisher(Int32, name, qos(32))
    shallow_received, deep_received = [], []
    shallow.create_subscription(Int32, name, lambda message: shallow_received.append(message.data), qos(3))
    deep.create_subscription(Int32, name, lambda message: deep_received.append(message.data), qos(16))
    spin_until(executor, lambda: publisher.get_subscription_count() == 2, "both reliable history readers")
    sent = list(range(12))
    # DDS workers still receive while this single executor stops taking samples.
    for number in sent:
        publisher.publish(Int32(data=number))
    acknowledged = publisher.wait_for_all_acked(Duration(seconds=3))
    if not acknowledged or shallow_received or deep_received:
        raise AssertionError("DDS burst was not fully acknowledged with application callbacks held")
    spin_until(executor, lambda: len(deep_received) >= 12 and len(shallow_received) >= 3,
               "both KEEP_LAST queues to drain")
    spin_window(executor, .2)
    if shallow_received != sent[-3:] or deep_received != sent:
        raise AssertionError(f"KEEP_LAST depth mismatch: {shallow_received}, {deep_received}")
    return {"topic": name, "sent": sent, "dds_ack_before_spin": acknowledged,
            "depth_3_received": shallow_received, "depth_16_received": deep_received}


def clock_trial(executor, reader, clocks):
    spin_until(executor, lambda: len(clocks) >= 3 and clocks[-1] > clocks[0], "advancing ROS /clock")
    before = clocks[-1]
    native = topic("/clock", timeout=5).get("sim")
    if not isinstance(native, dict):
        raise RuntimeError("Native Gazebo /clock lacks simulation time")
    native_ns = int(native.get("sec", 0)) * 10**9 + int(native.get("nsec", 0))
    spin_until(executor, lambda: clocks[-1] > native_ns, "bridged time after native sample")
    after = clocks[-1]
    if not 0 < before <= native_ns < after:
        raise AssertionError(f"ROS /clock does not bracket native simulation time: {before}, {native_ns}, {after}")
    return {"bridge_publishers": [profile_details(item) for item in reader.get_publishers_info_by_topic("/clock")],
            "ros_before_ns": before, "native_sim_ns": native_ns, "ros_after_ns": after,
            "ros_clock_advance_s": (after - before) / 1e9}


def experiment():
    ensure_running()
    rclpy.init()
    executor = SingleThreadedExecutor()
    namespace = "/gt/qos/run_" + uuid4().hex[:10]
    nodes = []
    print("M22 DDS experiment namespace: " + namespace, flush=True)

    def node(name):
        value = Node(name, namespace=namespace)
        nodes.append(value)
        executor.add_node(value)
        return value

    try:
        source = node("source")
        first = node("reader_a")
        second = node("reader_b")
        clocks = []

        def on_clock(message):
            ns = message.clock.sec * 10**9 + message.clock.nanosec
            if not clocks or ns > clocks[-1]:
                clocks.append(ns)

        first.create_subscription(Clock, "/clock", on_clock, qos(5, ReliabilityPolicy.BEST_EFFORT))
        reliability = reliability_trial(executor, source, first, second, namespace + "/reliability")
        durability = durability_trial(executor, source, first, second, namespace + "/durability")
        depth = depth_trial(executor, source, first, second, namespace + "/depth")
        clock = clock_trial(executor, first, clocks)
        return {"passed": True, "namespace": namespace, "rmw": get_rmw_implementation_identifier(),
                "reliability": reliability, "durability": durability, "history": depth, "clock": clock}
    finally:
        executor.shutdown()
        for value in reversed(nodes):
            value.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    result(experiment())
