#!/usr/bin/env python3
"""Latched provenance only; all control and state come from installed ROS plugins."""
import json,os,sys
from pathlib import Path
import rospy
from std_msgs.msg import String
sys.path.insert(0,os.environ['GAZEBOTEACH_WORKSPACE'])
from config import prepared
rospy.init_node('configuration')
config,record=prepared(Path(os.environ['GAZEBOTEACH_WORKSPACE']).resolve())
value={'node':rospy.get_name(),'pid':os.getpid(),'master_uri':os.environ.get('ROS_MASTER_URI'),'token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'workspace':os.environ['GAZEBOTEACH_WORKSPACE'],'settings_sha256':record['settings_sha256'],'config':config,'vendor_library_sha256':record['vendor_library_sha256']}
p=rospy.Publisher('/lesson25/configuration',String,queue_size=1,latch=True)
p.publish(String(data=json.dumps(value,sort_keys=True)))
rospy.spin()
