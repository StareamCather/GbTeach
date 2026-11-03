#!/usr/bin/env python3
"""Compare the same LIO-SAM input with loop closure disabled and enabled."""
import argparse
from collections import deque
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

from audit import (EXTRINSIC_T_IL, front_end_summary, graph_summary, lidar_truth, native_records, parse_factor_log,
                   physical_revisit, quaternion, require, same_stamps, trajectory_metrics, vector)


RAW_IMU = '/gt_lio/imu'
REPLAY_IMU = '/gt_lio/replay_imu'
REPLAY_LIDAR = '/gt_lio/replay_points'
UPSTREAM = '0be1fbe6275fb8366d5b800af4fc8c76a885c869'
MODES = ('loop_off', 'loop_on')
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
        done = wait_until(completed, 85., 'native rolling capture (Gazebo must advance)')
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
    cloud = point_cloud2.create_cloud(header, fields, points)
    # native_records has already validated all XYZ values. Stock LIO-SAM
    # imageProjection rejects a cloud marked non-dense before ring projection.
    cloud.is_dense = True
    return cloud


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
            require(msg.is_dense and layout == POINT_LAYOUT and all(field.count == 1 for field in msg.fields)
                    and msg.width*msg.height == len(scan['points']), 'Velodyne wire layout does not match')
            require(msg.header.stamp.to_nsec() == scan['stamp_ns'] and record_time.to_nsec() == scan['end_ns'],
                    'Bag header/record stamps no longer match native acquisition')


def cloud_data(message, point_cloud2):
    first = next(point_cloud2.read_points(message, ('x', 'y', 'z'), skip_nans=True), None)
    return {'stamp_ns': message.header.stamp.to_nsec(), 'count': message.width*message.height,
            'frame': message.header.frame_id, 'caller': caller(message),
            'contains_finite_point': first is not None and all(math.isfinite(value) for value in first)}


def pose_data(stamp, pose):
    return {'stamp_ns': stamp.to_nsec(), 'position': point_tuple(pose.position),
            'orientation': quaternion_tuple(pose.orientation)}


def observe_mode(directory, receipt, launch_file, bag_path, mode, request_id, expected_last_ns,
                 rospy, point_cloud2, cloud_class, odom_class, path_class):
    namespace = 'gt_a53_'+mode+'_'+request_id[:10]
    source_node = '/'+namespace+'/mapOptimization'
    prefix = '/'+namespace+'/lio_sam/mapping/'
    topics = {name: prefix+name for name in ('odometry', 'map_global', 'path')}
    topics['imu_incremental'] = '/'+namespace+'/odometry/imu_incremental'
    retained_maps = deque(maxlen=64)
    retained_paths = deque(maxlen=450)

    def odom(message):
        return dict(pose_data(message.header.stamp, message.pose.pose), caller=caller(message),
                    frame=message.header.frame_id, child_frame=message.child_frame_id)

    def cloud(message):
        row = cloud_data(message, point_cloud2)
        if row['contains_finite_point']:
            retained_maps.append(message)
        return row

    def path(message):
        if message.poses:
            retained_paths.append(message)
        return {'stamp_ns': message.header.stamp.to_nsec(), 'caller': caller(message),
                'frame': message.header.frame_id, 'poses': len(message.poses)}

    def imu_incremental(message):
        return {'stamp_ns': message.header.stamp.to_nsec(), 'caller': caller(message)}

    callbacks = {'odometry': (odom_class, odom), 'map_global': (cloud_class, cloud),
                 'path': (path_class, path), 'imu_incremental': (odom_class, imu_incremental)}
    streams = {name: Collector(callback, limit=20_000 if name == 'imu_incremental' else 1000)
               for name, (_, callback) in callbacks.items()}
    subscribers = [rospy.Subscriber(topics[name], cls, streams[name].ingest, queue_size=200,
                                    buff_size=8*1024*1024)
                   for name, (cls, _) in callbacks.items()]
    ros_home = directory/'ros-home'
    (ros_home/'Log').mkdir(parents=True)
    env = dict(os.environ, ROS_HOME=str(ros_home), ROS_LOG_DIR=str(ros_home/'log'))
    launch_command = ['bash', '-c', 'source "$1" && shift && exec roslaunch "$@"', 'bash',
                      receipt['setup_bash'], str(launch_file), 'mode_namespace:='+namespace,
                      'loop_enabled:='+('true' if mode == 'loop_on' else 'false')]
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
                require(launch.poll() is None, 'LIO-SAM node exited during replay; see estimator.log')
                status = player.poll()
                if status is None:
                    return None
                require(status == 0, 'rosbag play failed; see replay.log')
                return True
            wait_until(replay_finished, 80., 'LIO-SAM '+mode+' rosbag replay')
            # Stock map_global publishes on a background 0.2 Hz ROS timer.
            # Wait for a final map publication reflecting the processed tail.
            deadline = time.monotonic()+14.
            while time.monotonic() < deadline:
                require(launch.poll() is None, 'LIO-SAM node exited before output drain')
                maps = streams['map_global'].snapshot()
                if maps and maps[-1]['stamp_ns'] >= expected_last_ns-1_000_000_000:
                    break
                time.sleep(.1)
            configured = rospy.get_param('/'+namespace+'/lio_sam/loopClosureEnableFlag')
            require(configured is (mode == 'loop_on'), 'LIO-SAM loop configuration differs from selected mode')
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
    output = {name: stream.snapshot() for name, stream in streams.items()}
    for name in ('odometry', 'map_global', 'path'):
        require(output[name] and {row['caller'] for row in output[name]} == {source_node},
                'Missing or foreign LIO-SAM publisher on '+name)
    require(output['imu_incremental'] and {row['caller'] for row in output['imu_incremental']}
            == {'/'+namespace+'/imuPreintegration'}, 'Missing genuine IMU preintegration output')
    require(retained_maps and retained_paths, 'LIO-SAM did not publish finite global map and keyframe path')
    events = parse_factor_log(log_path)
    write_json(directory/'factor-events.json', events)
    final_map = save_map(directory, retained_maps[-1], point_cloud2, topics['map_global'], 'global-map.pcd')
    final_path = [pose_data(item.header.stamp, item.pose) for item in retained_paths[-1].poses]
    write_json(directory/'keyframe-path-final.json', final_path)
    loop_artifacts = {}
    before_path = None
    if events['loop']:
        applied = min(row['stamp_ns'] for row in events['loop'])
        before_maps = [msg for msg in retained_maps if msg.header.stamp.to_nsec() < applied]
        after_maps = [msg for msg in retained_maps if msg.header.stamp.to_nsec() >= applied]
        before_paths = [msg for msg in retained_paths if msg.header.stamp.to_nsec() < applied]
        require(before_maps and after_maps and before_paths, 'Missing published map/path around applied loop')
        loop_artifacts['map_before_first_loop'] = save_map(
            directory, before_maps[-1], point_cloud2, topics['map_global'], 'map-before-loop.pcd')
        loop_artifacts['map_after_first_loop'] = final_map
        before_path = [pose_data(item.header.stamp, item.pose) for item in before_paths[-1].poses]
        write_json(directory/'keyframe-path-before-loop.json', before_path)
        loop_artifacts['keyframe_path_before_loop'] = str(directory/'keyframe-path-before-loop.json')
    return {'mode': mode, 'loop_closure_enabled': configured, 'topics': topics, 'events': events,
            'odometry': output['odometry'], 'imu_incremental_messages': len(output['imu_incremental']),
            'final_path': final_path, 'before_path': before_path, 'final_map': final_map,
            'loop_artifacts': loop_artifacts}


def audit_mode(scans, truth, observations, directory):
    starts = {scan['stamp_ns'] for scan in scans}
    odometry = observations['odometry']
    path = observations['final_path']
    require(len(odometry) >= 50 and len(path) >= 20
            and all(row['stamp_ns'] in starts for row in odometry+path),
            'Insufficient scan-start-matched LIO-SAM poses')
    metrics, comparison = trajectory_metrics(truth, odometry)
    write_json(directory/'trajectory-online-comparison.json', comparison)
    final_metrics, final_comparison = trajectory_metrics(truth, path)
    write_json(directory/'trajectory-final-keyframes-comparison.json', final_comparison)
    factors = graph_summary(observations['events'], scans, observations['loop_closure_enabled'])
    require(len(path) == factors['mapping_batches'], 'Final published keyframe path disagrees with inserted key count')
    summary = {key: observations[key] for key in ('mode', 'loop_closure_enabled', 'topics', 'imu_incremental_messages',
                                                   'final_map', 'loop_artifacts')}
    summary.update(online_trajectory=metrics, final_keyframe_trajectory=final_metrics, factors=factors,
                   front_end=front_end_summary(observations['events'], scans),
                   keyframe_path_final=str(directory/'keyframe-path-final.json'))
    if observations['before_path'] is not None:
        before, after = same_stamps(observations['before_path'], path)
        require(len(before) >= 10, 'Too few common keyframes across loop optimization')
        revisions = [{'stamp_ns': old['stamp_ns'], 'position_before_m': old['position'],
                      'position_after_m': new['position'],
                      'position_shift_m': float(np.linalg.norm(np.asarray(new['position'])-np.asarray(old['position'])))}
                     for old, new in zip(before, after)]
        write_json(directory/'historical-keyframe-revisions.json', revisions)
        summary['loop_artifacts'].update(historical_keyframe_revisions=str(directory/'historical-keyframe-revisions.json'),
                                         shared_historical_keyframes=len(revisions),
                                         max_historical_keyframe_shift_m=max(row['position_shift_m'] for row in revisions))
    write_json(directory/'report.json', summary)
    for result, poses in ((metrics, odometry), (final_metrics, path)):
        require(result['matched'] >= .95*len(poses) and result['truth_max_displacement_m'] > 3.,
                'Insufficient overlapping moving trajectory evidence')
        require(result['position_rmse_m'] < 1.25 and result['orientation_rmse_rad'] < .45,
                'LIO-SAM trajectory error exceeded this scenario bound')
    return summary


def compare_on_off(truth, off, on, directory):
    comparisons = {}
    for path_kind, label in [('odometry', 'online'), ('final_path', 'final_keyframes')]:
        before, after = same_stamps(off[path_kind], on[path_kind])
        require(len(before) >= 20, 'Too few shared physical stamps for loop on/off '+label+' comparison')
        metric_off, rows_off = trajectory_metrics(truth, before)
        metric_on, rows_on = trajectory_metrics(truth, after)
        require(metric_off['matched'] == metric_on['matched'] and metric_off['anchor_stamp_ns'] == metric_on['anchor_stamp_ns'],
                'Loop-mode trajectory comparison does not share the same truth anchor and coverage')
        path = directory/('shared-'+label+'-trajectory.json')
        write_json(path, {'loop_off': rows_off, 'loop_on': rows_on})
        comparisons[label] = {'shared_stamps': metric_on['matched'],
                              'first_stamp_ns': rows_on[0]['stamp_ns'], 'last_stamp_ns': rows_on[-1]['stamp_ns'],
                              'loop_off': metric_off, 'loop_on': metric_on, 'paired_rows': str(path),
                              'position_rmse_delta_on_minus_off_m': metric_on['position_rmse_m']-metric_off['position_rmse_m']}
    return comparisons


def run():
    import fcntl
    import rosbag
    import rospy
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from std_msgs.msg import Header

    receipt_path = Path('build/lio-sam.json')
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    require(receipt['algorithm_commit'] == UPSTREAM, 'Build receipt is not the pinned LIO-SAM commit')
    for executable in ('image_projection', 'feature_extraction', 'map_optimization', 'imu_preintegration'):
        require(sha256(Path(receipt[executable])) == receipt[executable+'_sha256'],
                'Built LIO-SAM binary differs from receipt: '+executable)
    launch_file = (Path.cwd()/'mapping.launch').resolve()
    require(launch_file.is_file(), 'Course mapping.launch is missing')
    request_id = uuid.uuid4().hex
    directory = (Path('results')/('run-'+request_id)).resolve()
    directory.mkdir(parents=True)
    report = {'schema': 1, 'passed': False, 'request_id': request_id, 'run_directory': str(directory),
              'algorithm': 'LIO-SAM', 'upstream_commit': UPSTREAM,
              'build_receipt': receipt, 'build_receipt_sha256': sha256(receipt_path), 'modes': {},
              'inputs': {'lidar': REPLAY_LIDAR, 'imu': REPLAY_IMU, 'truth_published': False,
                         'velodyne_point_layout': POINT_LAYOUT, 'point_cloud_is_dense': True, 'timestamp_unit': 'seconds',
                         'scan_bag_record_time': 'last acquired raw ray time', 'map_pose_stamp': 'scan start',
                         'rig_T_IL_m': EXTRINSIC_T_IL.tolist(), 'lio_sam_extrinsicTrans_m': (-EXTRINSIC_T_IL).tolist(),
                         'extrinsic_rotation': 'identity', 'imu_orientation_source': 'native Gazebo ideal IMU sensor',
                         'truth_frame': 'LiDAR origin via native offline IMU-link pose and rigid lever arm',
                         'gps_input': False, 'external_loop_hints': False}}
    started = time.monotonic()
    try:
        rospy.init_node('gt_lio_a53_adapter', anonymous=True, disable_signals=True)
        with Path('build/capture.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raw_path, scans, imu_truth, imu = capture(directory, rospy, Imu, request_id)
            truth = lidar_truth(imu_truth)
            report['capture'] = {'native_records': str(raw_path), 'sha256': sha256(raw_path), 'scans': len(scans),
                                 'truth_samples': len(truth), 'points': sum(len(scan['points']) for scan in scans),
                                 'first_scan_ns': scans[0]['stamp_ns'], 'last_scan_end_ns': scans[-1]['end_ns'],
                                 'revisit': physical_revisit(truth)}
            require(report['capture']['revisit']['physical_distance_travelled_m'] > 8.
                    and report['capture']['revisit']['revisit_distance_m'] < .35,
                    'Native rig did not make a substantial physical revisit')
            report['imu'] = audit_imu(scans, imu)
            for message in imu:
                quaternion(quaternion_tuple(message.orientation))
                require(message.orientation_covariance[0] >= 0, 'Native IMU does not supply valid orientation')
            report['imu']['orientation_samples_validated'] = len(imu)
            bag_path = directory/'sensors.bag'
            write_bag(bag_path, scans, imu, rospy, rosbag, point_cloud2, PointField, Header)
            digest = sha256(bag_path)
            report['bag'] = {'path': str(bag_path), 'sha256': digest, 'published_clock': False,
                             'topics': [REPLAY_LIDAR, REPLAY_IMU], 'replay_rate': .8}
            observations = {}
            for mode in MODES:
                print('Replaying the same sensor bag for LIO-SAM '+mode, flush=True)
                mode_directory = directory/mode
                mode_directory.mkdir()
                observation = observe_mode(mode_directory, receipt, launch_file, bag_path, mode, request_id,
                                           scans[-1]['end_ns'], rospy, point_cloud2, PointCloud2, Odometry, PathMsg)
                require(sha256(bag_path) == digest, 'Replay bag changed during '+mode)
                observations[mode] = observation
                summary = audit_mode(scans, truth, observation, mode_directory)
                summary['input_bag_sha256'] = digest
                report['modes'][mode] = summary
            report['common_timestamp_comparison'] = compare_on_off(
                truth, observations['loop_off'], observations['loop_on'], directory)
            report['passed'] = True
        return report
    except BaseException as error:
        report['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        report['elapsed_wall_s'] = time.monotonic()-started
        write_json(directory/'report.json', report)
        write_json('build/verification.json', report)
        print(json.dumps({key: report.get(key) for key in ('passed', 'run_directory', 'error')},
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
    raise KeyboardInterrupt('LIO-SAM experiment interrupted by signal '+str(number))


if __name__ == '__main__':
    main()
