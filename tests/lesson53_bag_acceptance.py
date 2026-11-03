"""Independently audit the A53 bag's wire-level Velodyne point contract."""
import argparse
import hashlib
import math
from pathlib import Path
import struct

import rosbag
from sensor_msgs.msg import PointField

def audit_bag(path):
    point_topic='/gt_lio/replay_points'
    imu_topic='/gt_lio/replay_imu'
    fields={k:(t,n) for k,t,n in (
        ('x',PointField.FLOAT32,1),('y',PointField.FLOAT32,1),('z',PointField.FLOAT32,1),
        ('intensity',PointField.FLOAT32,1),('time',PointField.FLOAT32,1),('ring',PointField.UINT16,1),
    )}
    counts={point_topic:0,imu_topic:0}
    point_last=0
    imu_stamps=[]
    orientations=[]
    quaternion_norm_errors=[]
    native_frames=set()
    scan_times=[]
    with rosbag.Bag(str(path)) as bag:
        assert set(bag.get_type_and_topic_info().topics)==set(counts)
        for topic,msg,record in bag.read_messages():
            counts[topic]+=1
            acquisition=msg.header.stamp.to_nsec()
            if topic==imu_topic:
                assert record.to_nsec()==acquisition
                assert not imu_stamps or acquisition>imu_stamps[-1]
                imu_stamps.append(acquisition)
                q=msg.orientation
                norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                # Native orientation is preserved; quantify floating-point deviation
                # separately from requiring a finite, near-unit quaternion.
                assert abs(norm-1)<1e-4 and msg.orientation_covariance[0]>=0
                quaternion_norm_errors.append(abs(norm-1))
                orientations.append([q.x,q.y,q.z,q.w])
                native_frames.add(msg.header.frame_id)
                continue
            assert {f.name:(f.datatype,f.count) for f in msg.fields}==fields
            assert acquisition>point_last and (not point_last or acquisition-point_last==100_000_000)
            assert msg.height==1 and not msg.is_bigendian and msg.width>=500
            index={f.name:f.offset for f in msg.fields}
            assert len(msg.data)==msg.row_step==msg.point_step*msg.width
            rings=set()
            last=-1.
            for pos in range(0,len(msg.data),msg.point_step):
                offset,=struct.unpack_from('<f',msg.data,pos+index['time'])
                ring,=struct.unpack_from('<H',msg.data,pos+index['ring'])
                assert last<=offset<=.1 and 0<=ring<16
                last=offset
                rings.add(ring)
            assert rings==set(range(16))
            assert abs(last-.0995)<1e-7
            assert abs(record.to_nsec()-acquisition-round(last*1e9))<20
            scan_times.append((acquisition,record.to_nsec()))
            point_last=acquisition
    assert counts[point_topic]==340 and counts[imu_topic]>6750
    assert imu_stamps[0]<scan_times[0][0] and imu_stamps[-1]>scan_times[-1][-1]
    assert native_frames=={'sensor_body'} and max(abs(q[2]) for q in orientations)>.9
    return {'native_imu_frame':next(iter(native_frames)),'near_unit_native_imu_orientations':len(orientations),
            'max_native_quaternion_norm_deviation':max(quaternion_norm_errors),'counts':counts,'time_field_unit':'s','ring_datatype':'uint16',
            'first_scan':scan_times[0],'last_scan':scan_times[-1],
            'first_imu_ns':imu_stamps[0],'last_imu_ns':imu_stamps[-1],
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

if __name__=='__main__':
    import json
    parser=argparse.ArgumentParser()
    parser.add_argument('bag',type=Path)
    print(json.dumps(audit_bag(parser.parse_args().bag),indent=2))
