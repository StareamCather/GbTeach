"""Check native rolling sensor provenance and a real IMU message."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rospy
from sensor_msgs.msg import Imu

def main():
    token = os.environ['GAZEBOTEACH_SESSION_TOKEN']
    expected = hashlib.sha256(Path('rolling_lidar.cc').read_bytes()).hexdigest()
    deadline = time.monotonic()+5
    marker = Path('build/native-ready')
    while time.monotonic()<deadline:
        if marker.is_file() and marker.read_text().split()==[token,expected]:
            break
        time.sleep(.05)
    else:
        raise RuntimeError('Native rolling LiDAR plugin did not load the current sensor source')
    rospy.init_node('gt_a50_native_ready',anonymous=True)
    imu = rospy.wait_for_message('/gt_lio/imu',Imu,timeout=6)
    a = imu.linear_acceleration
    if imu.header.stamp.to_nsec()<=0 or not all(math.isfinite(v) for v in (a.x,a.y,a.z)):
        raise RuntimeError('No valid native IMU observation')
    print(json.dumps({'ready':True,'rolling_source_sha256':expected,'imu_stamp_ns':imu.header.stamp.to_nsec(),
                      'imu_frame':imu.header.frame_id,'imu_accel_m_s2':[a.x,a.y,a.z]}))

if __name__ == '__main__':
    main()
