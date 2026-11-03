#!/usr/bin/env python3
"""Capture actual Gazebo sensors, replay their ROS bag, audit FAST-LIO1 output."""
import argparse
from bisect import bisect_left
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid

import numpy as np

from audit import SCAN_PERIOD_NS, native_records, require, trajectory_metrics, vector


RAW_IMU = '/gt_lio/imu'
REPLAY_IMU = '/gt_lio/replay_imu'
REPLAY_LIDAR = '/livox/lidar'
SURFACE = '/laser_cloud_flat'
CORNER = '/laser_cloud_sharp'
MAP = '/Laser_map'
ODOMETRY = '/aft_mapped_to_init'
LIVOX_MSG_MD5 = 'e4d6829bdfe657cb6c21a746c86b21a6'
UPSTREAM = '6e1fa94351316eafac5fcf55c5fcd575f3e89829'


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            digest.update(data)
    return digest.hexdigest()


def wait_until(check, timeout, description):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        result = check()
        if result is not None and result is not False:
            return result
        time.sleep(.04)
    raise RuntimeError('Wall deadline expired: '+description)


def point_tuple(obj):
    return [obj.x, obj.y, obj.z]


def quaternion_tuple(obj):
    return [obj.x, obj.y, obj.z, obj.w]


def caller(message):
    return getattr(message, '_connection_header', {}).get('callerid', '')


class Collector:
    def __init__(self, callback, limit=10_000):
        self.callback = callback
        self.rows = deque(maxlen=limit)
        self.error = None
        self.lock = threading.Lock()

    def ingest(self, message):
        try:
            row = self.callback(message)
            with self.lock:
                self.rows.append(row)
        except Exception as error:
            with self.lock:
                self.error = str(error)

    def snapshot(self):
        with self.lock:
            require(self.error is None, 'ROS message audit failed: '+str(self.error))
            return list(self.rows)


def ros_time(rospy, stamp_ns):
    return rospy.Time(*divmod(stamp_ns, 1_000_000_000))


def capture(directory, rospy, imu_class, request_id):
    collector = Collector(lambda msg: msg, limit=20_000)
    subscription = rospy.Subscriber(RAW_IMU, imu_class, collector.ingest, queue_size=1000)
    try:
        wait_until(lambda: len(collector.snapshot()) >= 8, 12., 'native Gazebo IMU samples')
        request = Path('build/capture.request')
        require(not request.exists(), 'An earlier native capture request is still pending')
        tmp = request.with_suffix('.'+request_id+'.tmp')
        tmp.write_text(request_id+'\n', encoding='ascii')
        tmp.replace(request)
        done_path = Path('build/capture.done')
        def completed():
            if done_path.is_file():
                value = json.loads(done_path.read_text(encoding='utf-8'))
                if value.get('request_id') == request_id:
                    require(value.get('ok') is True, 'Native capture failed: '+str(value))
                    return value
            return None
        done = wait_until(completed, 55., 'native rolling capture (Gazebo must advance)')
        write_json(directory/'capture-done.json', done)
        raw = directory/'native.jsonl'
        shutil.copyfile('build/native.jsonl', raw)
        scans, truth = native_records(raw, done)
        end_ns = scans[-1]['end_ns']
        wait_until(lambda: collector.snapshot()[-1].header.stamp.to_nsec() >= end_ns+150_000_000,
                   8., 'native IMU after last rolling scan')
        imu = [msg for msg in collector.snapshot()
               if scans[0]['stamp_ns']-100_000_000 <= msg.header.stamp.to_nsec() <= end_ns+200_000_000]
        return raw, scans, truth, imu
    finally:
        subscription.unregister()


def audit_imu(scans, imu):
    stamps = [msg.header.stamp.to_nsec() for msg in imu]
    require(len(stamps) >= 100 and min(np.diff(stamps)) > 0, 'Native IMU stamps are insufficient or not increasing')
    require(stamps[0] <= scans[0]['stamp_ns'] and stamps[-1] >= scans[-1]['end_ns'],
            'Native IMU does not bracket the raw scan window')
    gaps = np.diff(stamps)
    require(int(max(gaps)) <= 25_000_000, 'Native IMU has a gap larger than 25 ms')
    acc = [vector(point_tuple(msg.linear_acceleration), 3, 'native IMU acceleration') for msg in imu]
    for msg in imu:
        vector(point_tuple(msg.angular_velocity), 3, 'native IMU angular velocity')
    callers = {caller(msg) for msg in imu}
    frames = {msg.header.frame_id for msg in imu}
    require(len(callers) == 1 and '' not in callers and len(frames) == 1 and '' not in frames,
            'Native IMU has ambiguous publisher/frame')
    return {'topic': RAW_IMU, 'publishers': sorted(callers), 'frames': sorted(frames),
            'count': len(imu), 'median_dt_ns': float(np.median(gaps)), 'largest_dt_ns': int(max(gaps)),
            'first_ns': stamps[0], 'last_ns': stamps[-1],
            'median_accel_norm_m_s2': float(np.median(np.linalg.norm(acc, axis=1)))}


def lidar_message(rospy, custom_msg, custom_point, scan):
    message = custom_msg()
    message.header.stamp = ros_time(rospy, scan['stamp_ns'])
    message.header.frame_id = 'gt_lio_lidar'
    message.timebase = scan['stamp_ns']
    message.lidar_id = 0
    message.rsvd = [0, 0, 0]
    for x, y, z, reflectivity, ring, offset in scan['points']:
        message.points.append(custom_point(offset_time=offset, x=x, y=y, z=z,
                                          reflectivity=reflectivity, tag=0x10, line=ring))
    message.point_num = len(message.points)
    return message


def write_bag(path, scans, imu, rospy, rosbag, custom_msg, custom_point):
    # Header remains the first acquisition time. A complete sweep becomes
    # available to the receiver only when its last ray has been acquired.
    timeline = [(scan['end_ns'], 1, scan) for scan in scans]
    timeline.extend((msg.header.stamp.to_nsec(), 0, msg) for msg in imu)
    with rosbag.Bag(str(path), 'w') as bag:
        for stamp, kind, data in sorted(timeline, key=lambda record: record[:2]):
            if kind:
                bag.write(REPLAY_LIDAR, lidar_message(rospy, custom_msg, custom_point, data), t=ros_time(rospy, stamp))
            else:
                bag.write(REPLAY_IMU, data, t=ros_time(rospy, stamp))
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        require(set(info) == {REPLAY_LIDAR, REPLAY_IMU}, 'Replay bag includes an unexpected topic')
        require(info[REPLAY_LIDAR].message_count == len(scans) and info[REPLAY_IMU].message_count == len(imu),
                'Replay bag message counts differ from verified input')
        require(info[REPLAY_LIDAR].msg_type == 'livox_ros_driver/CustomMsg'
                and info[REPLAY_IMU].msg_type == 'sensor_msgs/Imu', 'Replay bag type mismatch')


def stop_child(process):
    if process is None or process.poll() is not None:
        return
    for sent, timeout in [(signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sent)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError('Owned subprocess did not stop')


def save_map(directory, message, point_cloud2):
    # Convert just one retained ROS cloud after replay. Each subscriber callback
    # can replace the retained message without scanning or writing the full map.
    points = [point for point in point_cloud2.read_points(message, ('x', 'y', 'z'), skip_nans=True)
              if all(math.isfinite(value) for value in point)]
    require(points, 'Final retained map cloud has no finite XYZ point')
    path = directory/'laser-map.pcd'
    with path.open('w', encoding='ascii', newline='\n') as output:
        output.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        output.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(points), len(points)))
        output.writelines('%.9g %.9g %.9g\n' % tuple(point) for point in points)
    return {'path': str(path), 'sha256': sha256(path), 'format': 'PCD v0.7 ASCII XYZ',
            'point_count': len(points), 'original_point_count': message.width*message.height,
            'frame_id': message.header.frame_id, 'stamp_ns': message.header.stamp.to_nsec(),
            'topic': MAP, 'publisher': caller(message)}


def replay_and_observe(directory, receipt, launch_file, bag_path, rospy, point_cloud2, cloud_class, odom_class):
    def cloud_data(message):
        first = next(point_cloud2.read_points(message, ('x', 'y', 'z'), skip_nans=True), None)
        return {'stamp_ns': message.header.stamp.to_nsec(), 'count': message.width*message.height,
                'frame': message.header.frame_id, 'caller': caller(message),
                'contains_finite_point': first is not None and all(math.isfinite(value) for value in first)}

    def surfaces(message):
        row = cloud_data(message)
        values = [offset for offset, in point_cloud2.read_points(message, ('curvature',), skip_nans=False)]
        require(all(math.isfinite(value) and 0 <= value <= SCAN_PERIOD_NS/1e6 for value in values),
                'Feature offset is not finite milliseconds')
        require(all(a <= b for a, b in zip(values, values[1:])), 'Feature times are not sorted')
        row['processed_end_ns'] = row['stamp_ns']+round(values[-1]*1e6) if values else None
        return row

    def odom(message):
        pose = message.pose.pose
        return {'stamp_ns': message.header.stamp.to_nsec(), 'position': point_tuple(pose.position),
                'orientation': quaternion_tuple(pose.orientation), 'frame': message.header.frame_id,
                'child_frame': message.child_frame_id, 'caller': caller(message)}

    latest_map = deque(maxlen=1)

    def maps(message):
        row = cloud_data(message)
        if row['contains_finite_point']:
            latest_map.append(message)
        return row

    callbacks = {SURFACE: (cloud_class, surfaces), CORNER: (cloud_class, cloud_data),
                 MAP: (cloud_class, maps), ODOMETRY: (odom_class, odom)}
    streams = {topic: Collector(callback) for topic, (_, callback) in callbacks.items()}
    subscriptions = [rospy.Subscriber(topic, cls, streams[topic].ingest, queue_size=100,
                                      buff_size=4*1024*1024)
                     for topic, (cls, _) in callbacks.items()]
    ros_home = directory/'ros-home'
    (ros_home/'Log').mkdir(parents=True)
    env = dict(os.environ, ROS_HOME=str(ros_home), ROS_LOG_DIR=str(ros_home/'log'))
    launch_command = ['bash', '-c', 'source "$1" && shift && exec roslaunch "$@"',
                      'bash', receipt['setup_bash'], str(launch_file)]
    replay_command = ['rosbag', 'play', '--wait-for-subscribers', '--rate', '0.8', '--quiet', str(bag_path)]
    launch = player = None
    try:
        with (directory/'estimator.log').open('wb') as mapping_log, (directory/'replay.log').open('wb') as player_log:
            launch = subprocess.Popen(launch_command, env=env, stdout=mapping_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            player = subprocess.Popen(replay_command, env=env, stdout=player_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            def replay_finished():
                require(launch.poll() is None, 'FAST-LIO1 estimator exited during replay; see estimator.log')
                status = player.poll()
                if status is None:
                    return None
                require(status == 0, 'rosbag play failed; see replay.log')
                return True
            wait_until(replay_finished, 50., 'first-generation FAST-LIO rosbag replay')
            # Last published packet may still be in ROS TCP/callback queues.
            deadline = time.monotonic()+3.
            latest = -1
            stable = 0
            while time.monotonic() < deadline:
                require(launch.poll() is None, 'FAST-LIO1 estimator exited before output drain')
                count = len(streams[ODOMETRY].snapshot())
                stable = stable+1 if count == latest else 0
                latest = count
                if stable >= 5 and count >= 20:
                    break
                time.sleep(.1)
    finally:
        try:
            stop_child(player)
        finally:
            try:
                stop_child(launch)
            finally:
                for subscription in subscriptions:
                    subscription.unregister()
                for topic, stream in streams.items():
                    write_json(directory/(topic.rsplit('/', 1)[-1]+'.json'), stream.snapshot())
    map_artifact = save_map(directory, latest_map[-1], point_cloud2) if latest_map else None
    return {topic: stream.snapshot() for topic, stream in streams.items()}, map_artifact


def audit_outputs(streams, scans):
    surface, odometry = streams[SURFACE], streams[ODOMETRY]
    map_rows = [row for row in streams[MAP] if row['contains_finite_point']]
    require(len(surface) >= 20 and len(odometry) >= 20 and map_rows, 'Estimator produced too little feature/map/odometry evidence')
    require({row['caller'] for row in surface} == {'/feature_extract'}
            and {row['caller'] for row in odometry+map_rows} == {'/laserMapping'},
            'Outputs do not come from the launched first-generation nodes')
    raw_end = {row['stamp_ns']: row['end_ns'] for row in scans}
    ends = []
    for row in surface:
        if row['processed_end_ns'] is not None:
            end = row['processed_end_ns']
            require(row['stamp_ns'] in raw_end and row['stamp_ns'] <= end <= raw_end[row['stamp_ns']]+1000,
                    'Feature timestamp lies outside its acquired raw scan')
            ends.append(end)
    require(ends, 'No nonempty surface feature scan')
    ends.sort()
    for row in odometry:
        stamp = row['stamp_ns']
        index = bisect_left(ends, stamp)
        nearest = min(abs(candidate-stamp) for candidate in ends[max(0, index-1):index+1])
        require(nearest <= 1000, 'Odometry stamp is not a processed feature scan end')
    return {'surface_messages': len(surface), 'nonempty_surface_messages': len(ends),
            'corner_messages': len(streams[CORNER]), 'nonempty_map_messages': len(map_rows),
            'max_map_points': max(row['count'] for row in map_rows),
            'odometry_messages': len(odometry), 'odometry_frame_pairs': sorted({(row['frame'], row['child_frame']) for row in odometry}),
            'odom_time_basis': 'processed surface feature scan end', 'max_matching_tolerance_ns': 1000}


def run():
    import fcntl
    import rosbag
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import Imu, PointCloud2

    receipt_path = Path('build/fast-lio1.json')
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    require(receipt['algorithm_commit'] == UPSTREAM, 'Build receipt is not the pinned first-generation FAST-LIO')
    for binary in ('feature_extract', 'laser_mapping'):
        require(sha256(Path(receipt[binary])) == receipt[binary+'_sha256'], 'Built executable differs from receipt: '+binary)
    sys.path.insert(0, receipt['python_path'])
    from livox_ros_driver.msg import CustomMsg, CustomPoint
    require(CustomMsg._md5sum == LIVOX_MSG_MD5, 'Generated Livox ROS message type hash does not match')
    launch_file = (Path.cwd()/'mapping.launch').resolve()
    require(launch_file.is_file(), 'Course mapping.launch is missing')
    request_id = uuid.uuid4().hex
    directory = (Path('results')/('run-'+request_id)).resolve()
    directory.mkdir(parents=True)
    report = {'schema': 1, 'passed': False, 'request_id': request_id, 'run_directory': str(directory),
              'algorithm': 'FAST-LIO first generation', 'upstream_commit': UPSTREAM,
              'build_receipt': receipt, 'build_receipt_sha256': sha256(receipt_path),
              'inputs': {'lidar': REPLAY_LIDAR, 'imu': REPLAY_IMU, 'truth_published': False,
                         'lidar_point_stamp': 'header stamp + offset_time (ns)',
                         'scan_bag_record_time': 'last acquired raw ray time',
                         'extrinsic_T_IL_m': [.04165, .02326, -.0284], 'extrinsic_R_IL': 'identity',
                         'truth_frame': 'IMU link pose in Gazebo world'}}
    started = time.monotonic()
    try:
        rospy.init_node('gt_lio_a50_adapter', anonymous=True, disable_signals=True)
        with Path('build/capture.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raw_path, scans, truth, imu = capture(directory, rospy, Imu, request_id)
            report['capture'] = {'native_records': str(raw_path), 'sha256': sha256(raw_path),
                                 'scans': len(scans), 'truth_samples': len(truth),
                                 'points': sum(len(scan['points']) for scan in scans),
                                 'first_scan_ns': scans[0]['stamp_ns'], 'last_scan_end_ns': scans[-1]['end_ns'],
                                 'scan_span_min_ns': min(row['end_ns']-row['stamp_ns'] for row in scans),
                                 'scan_span_max_ns': max(row['end_ns']-row['stamp_ns'] for row in scans)}
            report['imu'] = audit_imu(scans, imu)
            bag_path = directory/'sensors.bag'
            write_bag(bag_path, scans, imu, rospy, rosbag, CustomMsg, CustomPoint)
            report['bag'] = {'path': str(bag_path), 'sha256': sha256(bag_path), 'published_clock': False,
                             'lidar_msg_md5': LIVOX_MSG_MD5, 'topics': [REPLAY_LIDAR, REPLAY_IMU]}
            streams, map_artifact = replay_and_observe(
                directory, receipt, launch_file, bag_path, rospy, point_cloud2, PointCloud2, Odometry)
            report['outputs'] = audit_outputs(streams, scans)
            report['map_artifact'] = map_artifact
            metrics, samples = trajectory_metrics(truth, streams[ODOMETRY])
            report['metrics'] = metrics
            write_json(directory/'trajectory-comparison.json', samples)
            require(metrics['matched'] >= .95*len(streams[ODOMETRY]), 'Insufficient timestamp-matched trajectory coverage')
            require(metrics['truth_max_displacement_m'] > .5 and metrics['estimate_max_displacement_m'] > .2,
                    'Trajectory lacks meaningful physical/estimated motion')
            require(metrics['position_rmse_m'] < .65 and metrics['orientation_rmse_rad'] < .35,
                    'First-pose-aligned trajectory errors exceed the teaching scenario bounds')
            report['passed'] = True
        return report
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        report['elapsed_wall_s'] = time.monotonic()-started
        write_json(directory/'report.json', report)
        write_json('build/verification.json', report)
        print(json.dumps({key: report.get(key) for key in ('passed', 'run_directory', 'metrics', 'error')},
                         ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['run'])
    parser.parse_args()
    install_handlers()
    run()


def install_handlers():
    for number in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, cancel)


def cancel(number, frame):
    raise KeyboardInterrupt('FAST-LIO experiment interrupted by signal '+str(number))


if __name__ == '__main__':
    main()
