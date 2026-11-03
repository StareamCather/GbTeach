#!/usr/bin/env python3
"""Compare two official FAST-LIO2 modes with an identical native sensor bag."""
import argparse
from bisect import bisect_right
from collections import deque
import csv
from decimal import Decimal
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid

import numpy as np

from audit import native_records, require, trajectory_metrics, vector


RAW_IMU = '/gt_lio/imu'
REPLAY_IMU = '/gt_lio/replay_imu'
REPLAY_LIDAR = '/gt_lio/replay_points'
UPSTREAM = '7cc4175de6f8ba2edf34bab02a42195b141027e9'
IKD_TREE = 'e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4'
MODES = ('direct', 'feature')
# ROS PointField constants are stable wire values: FLOAT32=7, UINT16=4.
POINT_LAYOUT = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                ('time', 16, 7), ('ring', 20, 4)]


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


def save_map(directory, message, point_cloud2, topic, filename):
    # Convert just one retained ROS cloud after replay. Each subscriber callback
    # can replace the retained message without scanning or writing the full map.
    points = [point for point in point_cloud2.read_points(message, ('x', 'y', 'z'), skip_nans=True)
              if all(math.isfinite(value) for value in point)]
    require(points, 'Final retained map cloud has no finite XYZ point')
    path = directory/filename
    with path.open('w', encoding='ascii', newline='\n') as output:
        output.write('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        output.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n' % (len(points), len(points)))
        output.writelines('%.9g %.9g %.9g\n' % tuple(point) for point in points)
    return {'path': str(path), 'sha256': sha256(path), 'format': 'PCD v0.7 ASCII XYZ',
            'point_count': len(points), 'original_point_count': message.width*message.height,
            'frame_id': message.header.frame_id, 'stamp_ns': message.header.stamp.to_nsec(),
            'topic': topic, 'publisher': caller(message)}


def velodyne_cloud(rospy, point_cloud2, point_field, header_cls, scan):
    header = header_cls(stamp=ros_time(rospy, scan['stamp_ns']), frame_id='gt_lio_lidar')
    fields = [point_field(name=name, offset=offset, datatype=datatype, count=1)
              for name, offset, datatype in POINT_LAYOUT]
    points = [(x, y, z, float(reflectivity), offset_ns/1e9, ring)
              for x, y, z, reflectivity, ring, offset_ns in scan['points']]
    return point_cloud2.create_cloud(header, fields, points)


def write_bag(path, scans, imu, rospy, rosbag, point_cloud2, point_field, header_cls):
    timeline = [(scan['end_ns'], 1, scan) for scan in scans]
    timeline.extend((msg.header.stamp.to_nsec(), 0, msg) for msg in imu)
    with rosbag.Bag(str(path), 'w') as bag:
        for stamp, kind, data in sorted(timeline, key=lambda row: row[:2]):
            time_stamp = ros_time(rospy, stamp)
            if kind:
                msg = velodyne_cloud(rospy, point_cloud2, point_field, header_cls, data)
                bag.write(REPLAY_LIDAR, msg, t=time_stamp)
            else:
                bag.write(REPLAY_IMU, data, t=time_stamp)
    with rosbag.Bag(str(path), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        require(set(info) == {REPLAY_LIDAR, REPLAY_IMU}, 'Replay bag includes an unexpected topic')
        require(info[REPLAY_LIDAR].msg_type == 'sensor_msgs/PointCloud2'
                and info[REPLAY_IMU].msg_type == 'sensor_msgs/Imu', 'Replay bag type mismatch')
        require(info[REPLAY_LIDAR].message_count == len(scans) and info[REPLAY_IMU].message_count == len(imu),
                'Replay bag message counts differ from audited capture')
        for (_, msg, record_time), scan in zip(bag.read_messages(topics=[REPLAY_LIDAR]), scans):
            layout = [(field.name, field.offset, field.datatype) for field in msg.fields]
            require(layout == POINT_LAYOUT and all(field.count == 1 for field in msg.fields)
                    and msg.width*msg.height == len(scan['points']), 'Velodyne wire layout does not match')
            require(msg.header.stamp.to_nsec() == scan['stamp_ns'] and record_time.to_nsec() == scan['end_ns'],
                    'Bag header/record stamps no longer match native acquisition')


def parse_diagnostics(log_path):
    pre, processed = {}, {}
    for line in log_path.read_text(encoding='utf-8', errors='replace').splitlines():
        for marker, dest in [('GT_FAST_LIO2_PRE', pre), ('GT_FAST_LIO2_SCAN', processed)]:
            if marker not in line:
                continue
            fields = dict(token.split('=', 1) for token in line.split(marker, 1)[1].split() if '=' in token)
            key = 'stamp' if dest is pre else 'start'
            stamp = int(Decimal(fields[key])*1_000_000_000)
            require(stamp > 0 and stamp not in dest, 'Repeated/invalid native timing diagnostic stamp')
            result = {'stamp_ns': stamp, 'points': int(fields['points']), 'seconds': float(fields['seconds'])}
            require(result['points'] >= 0 and math.isfinite(result['seconds']) and result['seconds'] > 0,
                    'Invalid FAST-LIO2 processing diagnostic')
            if dest is processed:
                result['end_ns'] = int(Decimal(fields['end'])*1_000_000_000)
                for count in ('downsampled', 'effective'):
                    result[count] = int(fields[count])
                    require(result[count] >= 0, 'Negative FAST-LIO2 point count')
                map_points = int(fields['map_points'])
                # KD_TREE::validnum() cannot take its lock during some rebuilds.
                require(map_points >= -1, 'Invalid FAST-LIO2 map point count')
                result['map_points'] = None if map_points == -1 else map_points
            dest[stamp] = result
    require(pre and processed, 'Pinned mapper timing diagnostics missing')
    return pre, processed


def cloud_metadata(message, point_cloud2):
    first = next(point_cloud2.read_points(message, ('x', 'y', 'z'), skip_nans=True), None)
    return {'stamp_ns': message.header.stamp.to_nsec(), 'count': message.width*message.height,
            'frame': message.header.frame_id, 'caller': caller(message),
            'contains_finite_point': first is not None and all(math.isfinite(value) for value in first)}


def observe_mode(directory, receipt, launch_file, bag_path, mode, request_id, rospy,
                 point_cloud2, cloud_class, odom_class):
    namespace = 'gt_a51_'+mode+'_'+request_id[:10]
    topics = {name: '/'+namespace+'/'+name for name in ('odometry', 'registered', 'map')}
    source_node = '/'+namespace+'/laserMapping'
    latest_map = deque(maxlen=1)

    def map_data(message):
        row = cloud_metadata(message, point_cloud2)
        if row['contains_finite_point']:
            latest_map.append(message)
        return row

    def odom_data(message):
        pose = message.pose.pose
        return {'stamp_ns': message.header.stamp.to_nsec(), 'position': point_tuple(pose.position),
                'orientation': quaternion_tuple(pose.orientation), 'frame': message.header.frame_id,
                'child_frame': message.child_frame_id, 'caller': caller(message)}

    callbacks = {'odometry': (odom_class, odom_data), 'map': (cloud_class, map_data),
                 'registered': (cloud_class, lambda msg: cloud_metadata(msg, point_cloud2))}
    streams = {name: Collector(callback) for name, (_, callback) in callbacks.items()}
    subscribers = [rospy.Subscriber(topics[name], cls, streams[name].ingest, queue_size=100,
                                    buff_size=4*1024*1024)
                   for name, (cls, _) in callbacks.items()]
    ros_home = directory/'ros-home'
    (ros_home/'Log').mkdir(parents=True)
    env = dict(os.environ, ROS_HOME=str(ros_home), ROS_LOG_DIR=str(ros_home/'log'))
    launch_command = ['bash', '-c', 'source "$1" && shift && exec roslaunch "$@"', 'bash',
                      receipt['setup_bash'], str(launch_file), 'mode_namespace:='+namespace,
                      'features:='+('true' if mode == 'feature' else 'false')]
    replay_command = ['rosbag', 'play', '--wait-for-subscribers', '--rate', '0.8', '--quiet', str(bag_path)]
    launch = player = None
    log_path = directory/'estimator.log'
    try:
        with log_path.open('wb') as mapping_log, (directory/'replay.log').open('wb') as player_log:
            launch = subprocess.Popen(launch_command, env=env, stdout=mapping_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            player = subprocess.Popen(replay_command, env=env, stdout=player_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            def replay_finished():
                require(launch.poll() is None, 'FAST-LIO2 mapper exited during replay; see estimator.log')
                status = player.poll()
                if status is None:
                    return None
                require(status == 0, 'rosbag play failed; see replay.log')
                return True
            wait_until(replay_finished, 50., 'FAST-LIO2 '+mode+' rosbag replay')
            deadline = time.monotonic()+3.
            latest = -1
            stable = 0
            while time.monotonic() < deadline:
                require(launch.poll() is None, 'FAST-LIO2 mapper exited before output drain')
                count = len(streams['odometry'].snapshot())
                stable = stable+1 if count == latest else 0
                latest = count
                if stable >= 5 and count >= 20:
                    break
                time.sleep(.1)
            configured_mode = rospy.get_param('/'+namespace+'/feature_extract_enable')
            require(configured_mode is (mode == 'feature'), 'FAST-LIO2 mode parameter differs from requested mode')
    finally:
        try:
            stop_child(player)
        finally:
            try:
                stop_child(launch)
            finally:
                for sub in subscribers:
                    sub.unregister()
                for name, stream in streams.items():
                    write_json(directory/(name+'.json'), stream.snapshot())

    outputs = {name: stream.snapshot() for name, stream in streams.items()}
    for name, rows in outputs.items():
        require(rows and {row['caller'] for row in rows} == {source_node},
                'Missing or foreign source on '+name+' in '+mode+' mode')
    require(latest_map, 'Mapper did not produce a finite local-map cloud')
    artifact = save_map(directory, latest_map[-1], point_cloud2, topics['map'], 'laser-map.pcd')
    pre, processed = parse_diagnostics(log_path)
    write_json(directory/'native-timing.json', {'preprocess': list(pre.values()), 'mapping': list(processed.values())})
    return {'mode': mode, 'feature_extract_enable': configured_mode, 'topics': topics, 'map_artifact': artifact,
            'odometry': outputs['odometry'], 'registered_messages': len(outputs['registered']),
            'local_map_messages': len(outputs['map']), 'preprocess': pre, 'processed': processed}


def audit_mode(scans, truth, observations, directory):
    raw = {row['stamp_ns']: row['end_ns'] for row in scans}
    scan_starts = sorted(raw)
    pre = observations['preprocess']
    processed = observations['processed']
    odometry = observations['odometry']
    require(len(odometry) >= 20 and len(processed) >= 20, 'Too few FAST-LIO2 poses or processed scans')
    require(set(pre) <= set(raw) and set(processed) <= set(pre),
            'Native preprocessing/mapping log uses an unknown scan start')
    for stamp, row in processed.items():
        require(stamp <= row['end_ns'] <= raw[stamp]+1_000, 'Mapper processed end lies outside acquired sweep')
    actual_ends = sorted(row['end_ns'] for row in processed.values())
    for row in odometry:
        stamp = row['stamp_ns']
        index = bisect_right(actual_ends, stamp)
        candidates = actual_ends[max(0, index-1):index+1]
        require(candidates and min(abs(value-stamp) for value in candidates) <= 1_000,
                'Odometry stamp differs from actual processed scan end')
        scan = bisect_right(scan_starts, stamp)-1
        require(scan >= 0 and stamp <= raw[scan_starts[scan]]+1_000, 'Odometry outside actual acquired sweep')
    metrics, samples = trajectory_metrics(truth, odometry)
    write_json(directory/'trajectory-comparison.json', samples)
    summary = {key: value for key, value in observations.items()
               if key not in ('odometry', 'preprocess', 'processed')}
    map_counts = [row['map_points'] for row in processed.values() if row['map_points'] is not None]
    summary.update({'metrics': metrics, 'odometry_messages': len(odometry),
                    'preprocess_events': len(pre), 'processed_events': len(processed),
                    'max_ikd_tree_valid_points': max(map_counts) if map_counts else None,
                    'ikd_tree_valid_point_counts_available': len(map_counts)})
    write_json(directory/'report.json', summary)
    require(metrics['matched'] >= .95*len(odometry), 'Insufficient timestamp-matched trajectory coverage')
    require(metrics['truth_max_displacement_m'] > .5 and metrics['estimate_max_displacement_m'] > .2,
            'Trajectory lacks meaningful actual/estimated motion')
    require(metrics['position_rmse_m'] < .65 and metrics['orientation_rmse_rad'] < .35,
            'First-pose-aligned trajectory errors exceed scenario bounds for '+summary['mode'])
    return summary


def compare_timing(direct, feature):
    """Pair both timing paths by original bag scan start, excluding warm-up differences."""
    starts = sorted(set(direct['processed']) & set(feature['processed']))
    require(len(starts) >= 20, 'Too few same-input processed scans for timing comparison')
    paired = []
    for stamp in starts:
        row = {'scan_start_ns': stamp}
        for mode, observation in [('direct', direct), ('feature', feature)]:
            pre, mapping = observation['preprocess'][stamp], observation['processed'][stamp]
            row[mode] = {'preprocess_ms': 1000*pre['seconds'], 'mapping_ms': 1000*mapping['seconds'],
                         'sum_measured_ms': 1000*(pre['seconds']+mapping['seconds']),
                         'preprocessed_points': pre['points'], 'downsampled_points': mapping['downsampled'],
                         'effective_correspondences': mapping['effective'], 'map_points': mapping['map_points']}
        paired.append(row)
    def stats(mode, key):
        values = [row[mode][key] for row in paired if row[mode][key] is not None]
        return {'median': float(np.median(values)) if values else None,
                'p95': float(np.percentile(values, 95)) if values else None, 'available_count': len(values)}
    keys = ('preprocess_ms', 'mapping_ms', 'sum_measured_ms', 'preprocessed_points',
            'downsampled_points', 'effective_correspondences', 'map_points')
    summary = {mode: {key: stats(mode, key) for key in keys} for mode in MODES}
    summary.update(paired_scans=len(paired), pairing='exact original bag scan-start timestamp',
                   timing_scope='Native monotonic stage timers. Mapping includes the odometry publish call. The sum excludes ROS transport, replay pacing, queue wait, later point-cloud/path publishing and diagnostic map export.')
    return summary, paired


def run():
    import fcntl
    import rosbag
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import Header

    receipt_path = Path('build/fast-lio2.json')
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    require(receipt['algorithm_commit'] == UPSTREAM and receipt['ikd_tree_commit'] == IKD_TREE,
            'Build receipt does not use the pinned FAST-LIO2 and ikd-Tree commits')
    require(sha256(Path(receipt['laser_mapping'])) == receipt['laser_mapping_sha256'],
            'Built mapper executable differs from receipt')
    launch_file = (Path.cwd()/'mapping.launch').resolve()
    require(launch_file.is_file(), 'Course mapping.launch is missing')
    request_id = uuid.uuid4().hex
    directory = (Path('results')/('run-'+request_id)).resolve()
    directory.mkdir(parents=True)
    report = {'schema': 1, 'passed': False, 'request_id': request_id, 'run_directory': str(directory),
              'algorithm': 'FAST-LIO2', 'upstream_commit': UPSTREAM, 'ikd_tree_commit': IKD_TREE,
              'build_receipt': receipt, 'build_receipt_sha256': sha256(receipt_path), 'modes': {},
              'inputs': {'lidar': REPLAY_LIDAR, 'imu': REPLAY_IMU, 'truth_published': False,
                         'velodyne_point_layout': POINT_LAYOUT, 'timestamp_unit': 'seconds',
                         'scan_bag_record_time': 'last acquired raw ray time',
                         'extrinsic_T_IL_m': [.04165, .02326, -.0284], 'extrinsic_R_IL': 'identity',
                         'truth_frame': 'IMU link pose in Gazebo world'}}
    started = time.monotonic()
    try:
        rospy.init_node('gt_lio_a51_adapter', anonymous=True, disable_signals=True)
        with Path('build/capture.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raw_path, scans, truth, imu = capture(directory, rospy, Imu, request_id)
            report['capture'] = {'native_records': str(raw_path), 'sha256': sha256(raw_path),
                                 'scans': len(scans), 'truth_samples': len(truth),
                                 'points': sum(len(scan['points']) for scan in scans),
                                 'first_scan_ns': scans[0]['stamp_ns'], 'last_scan_end_ns': scans[-1]['end_ns']}
            report['imu'] = audit_imu(scans, imu)
            bag_path = directory/'sensors.bag'
            write_bag(bag_path, scans, imu, rospy, rosbag, point_cloud2, PointField, Header)
            digest = sha256(bag_path)
            report['bag'] = {'path': str(bag_path), 'sha256': digest, 'published_clock': False,
                             'topics': [REPLAY_LIDAR, REPLAY_IMU], 'replay_rate': .8}
            observations = {}
            for mode in MODES:
                mode_directory = directory/mode
                mode_directory.mkdir()
                observation = observe_mode(mode_directory, receipt, launch_file, bag_path, mode,
                                           request_id, rospy, point_cloud2, PointCloud2, Odometry)
                require(sha256(bag_path) == digest, 'Replay bag changed during '+mode+' mode')
                observations[mode] = observation
                summary = audit_mode(scans, truth, observation, mode_directory)
                summary['input_bag_sha256'] = digest
                report['modes'][mode] = summary
            timing, paired = compare_timing(observations['direct'], observations['feature'])
            timing_path = directory/'paired-timing.json'
            write_json(timing_path, paired)
            timing['paired_rows'] = str(timing_path)
            report['timing'] = timing
            report['passed'] = True
        return report
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        report['elapsed_wall_s'] = time.monotonic()-started
        write_json(directory/'report.json', report)
        write_json('build/verification.json', report)
        print(json.dumps({key: report.get(key) for key in ('passed', 'run_directory', 'timing', 'error')},
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
    raise KeyboardInterrupt('FAST-LIO2 experiment interrupted by signal '+str(number))


if __name__ == '__main__':
    main()
