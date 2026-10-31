#!/usr/bin/env python3
"""Own exactly the two shared-world identity TF edges, never robot motion."""
import json
import os
import time
from config import ROBOTS, identity, prepared


def main():
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    _, record = prepared()
    if not os.environ.get("GAZEBOTEACH_SESSION_TOKEN"):
        raise RuntimeError("Start this helper through the lesson's private session")
    rospy.init_node("lesson33_world_bridge")
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []
    for robot in ROBOTS:
        item = TransformStamped()
        item.header.frame_id = "world"
        item.child_frame_id = robot+"/odom"
        item.transform.rotation.w = 1
        # WORLD odometry already contains the spawn offset. Identity is correct.
        transforms.append(item)
    broadcaster.sendTransform(transforms)
    publisher = rospy.Publisher("/lesson33/configuration", String, queue_size=1, latch=True)
    publisher.publish(String(data=json.dumps({"identity": identity(record), "pid": os.getpid(),
        "node": rospy.get_name(), "static_edges": [["world", robot+"/odom"] for robot in ROBOTS],
        "odometry_source": "world"})))
    print(json.dumps({"bridge_ready": True, "node": rospy.get_name(), "pid": os.getpid()}), flush=True)
    # rospy.sleep/spin are unnecessary here: callbacks run in transport threads.
    # Wall sleep remains responsive at simulation time zero and while paused.
    while not rospy.is_shutdown():
        time.sleep(.1)


if __name__ == "__main__":
    main()
