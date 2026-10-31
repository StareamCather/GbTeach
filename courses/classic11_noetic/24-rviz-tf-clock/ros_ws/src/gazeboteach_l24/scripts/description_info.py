#!/usr/bin/env python3
"""Report the actual loaded URDF input; this is not a physical-model certificate."""
import hashlib
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String
from common import identity, publish


if __name__ == "__main__":
    rospy.init_node("description_info")
    info = identity()
    value = rospy.get_param("/lesson24/robot_description")
    robot = ET.fromstring(value)
    publisher = rospy.Publisher("/lesson24/description_status", String, queue_size=1, latch=True)
    publish(publisher, {**info, "loaded_description_sha256": hashlib.sha256(value.encode("utf-8")).hexdigest(),
        "urdf_links": [item.attrib["name"] for item in robot.findall("link")],
        "urdf_joints": [item.attrib["name"] for item in robot.findall("joint")],
        "note": "Parsed robot_description only. Actual spawned links, properties and motion require independent Gazebo queries."})
    rospy.spin()
