#!/usr/bin/env python3
"""Small shared node helpers; sensor/state values are copied from actual ROS data."""
import json
import os
from pathlib import Path
import sys
import threading
import time
import rospy


def call(name, service_type, *args, **kwargs):
    rospy.wait_for_service(name, timeout=4)
    completed, result = threading.Event(), {}
    def invoke():
        proxy = rospy.ServiceProxy(name, service_type)
        try:
            result["value"] = proxy(*args, **kwargs)
        except Exception as error:
            result["error"] = error
        finally:
            proxy.close()
            completed.set()
    threading.Thread(target=invoke, daemon=True).start()
    if not completed.wait(4):
        raise RuntimeError("Finite ROS service timeout: "+name)
    if "error" in result:
        raise RuntimeError(str(result["error"]))
    value = result["value"]
    if hasattr(value, "success") and not value.success:
        detail = getattr(value, "status_message", "") or getattr(value, "message", "") or "refused"
        raise RuntimeError(name+": "+detail)
    return value


def identity():
    workspace = Path(os.environ["GAZEBOTEACH_WORKSPACE"]).resolve()
    sys.path.insert(0, str(workspace))
    from config import prepared
    config, record = prepared(workspace)
    return {"node": rospy.get_name(), "namespace": rospy.get_namespace(), "pid": os.getpid(),
        "parent_pid": os.getppid(), "master_uri": os.environ.get("ROS_MASTER_URI"),
        "session_token": os.environ.get("GAZEBOTEACH_SESSION_TOKEN"),
        "ros_package_path": os.environ.get("ROS_PACKAGE_PATH"), "workspace": str(workspace),
        "settings_sha256": record["settings_sha256"], "source_sha256": record["source_sha256"], "config": config}


def publish(publisher, value):
    from std_msgs.msg import String
    publisher.publish(String(data=json.dumps(value, separators=(",", ":"), allow_nan=False)))


def pose_twist(pose, twist):
    return {"position": [pose.position.x, pose.position.y, pose.position.z],
        "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
        "linear": [twist.linear.x, twist.linear.y, twist.linear.z],
        "angular": [twist.angular.x, twist.angular.y, twist.angular.z]}
