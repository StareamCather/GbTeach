#!/usr/bin/env python3
"""Independent two-robot ROS namespace/TF and real wheel-driven physics acceptance.

Each case owns fresh Storage, ROS and Gazebo masters. A separate rospy process
publishes real Twist inputs and retains original message bytes, ROS stamps,
receipt clocks, graph connections and Gazebo service readbacks. It never calls
the course motion implementation, teleports models or resets measurement poses.
The world-source odometry oracle follows the installed upstream Noetic plugin:
global Model::WorldPose, body-frame linear twist, and the same pose/stamp in TF.
"""
import argparse
import base64
import io
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import sys
import tempfile
import threading
import time
import uuid
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson23_acceptance import Probe as ServiceProbe
from tests.lesson23_acceptance import hashes, ros_value, save, sha

ROBOTS = ('alpha', 'beta')
SOURCE_URL = ('https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/'
              '2.9.3/gazebo_plugins/src/gazebo_ros_diff_drive.cpp')
MOVE_SECONDS, STOP_SECONDS = 2., 1.


def require(value, message):
    if not value:
        raise AssertionError(message)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(pose):
    q = pose['orientation']
    x, y, z, w = [q[k] for k in ('x', 'y', 'z', 'w')]
    require(abs(x*x+y*y+z*z+w*w-1) < .01, 'Actual quaternion is not normalized')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def state(row, robot):
    return row['models'][robot]


def sim(row):
    return (row['world_time_before_ns']+row['world_time_after_ns'])/2e9


def distance(a, b):
    return math.hypot(a['position']['x']-b['position']['x'], a['position']['y']-b['position']['y'])


def body_velocity(value):
    heading = yaw(value['pose'])
    v = value['twist']['linear']
    return (math.cos(heading)*v['x']+math.sin(heading)*v['y'],
            math.cos(heading)*v['y']-math.sin(heading)*v['x'])


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.
        return connection


def rpc(uri, method, caller, *args):
    with xmlrpc.client.ServerProxy(uri, transport=Transport(), allow_none=True) as proxy:
        code, message, value = getattr(proxy, method)(caller, *args)
    require(code == 1, 'Actual ROS XMLRPC failed '+method+': '+message)
    return value


def process(pid, token):
    require(pid in live_token_processes(token), 'ROS node PID is outside this private session')
    folder = Path('/proc', str(pid))
    maps = folder.joinpath('maps').read_text()
    libraries = sorted(set(line.split()[-1] for line in maps.splitlines()
                           if 'libgazebo_ros_diff_drive.so' in line or 'libgazebo_ros_api_plugin.so' in line))
    return {'pid': pid, 'start_time': folder.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19],
            'argv': [v.decode(errors='replace') for v in folder.joinpath('cmdline').read_bytes().split(b'\0') if v],
            'mapped_native_libraries': {v: sha(v) for v in libraries},
            'native_library_map_lines': [v for v in maps.splitlines() if any(lib in v for lib in libraries)]}


class Probe(ServiceProbe):
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        # A unique identity also prevents stale connections after a prior worker.
        self.worker_uuid = uuid.uuid4().hex
        rospy.init_node('independent_l33_'+self.worker_uuid, disable_signals=True)
        self.rospy, self.events, self.commands, self.lock = rospy, [], [], threading.Lock()
        self.subscribers, self.publishers = [], {}
        for robot in ROBOTS:
            self.publishers[robot] = rospy.Publisher('/'+robot+'/cmd_vel', Twist, queue_size=1, latch=False)
            for suffix, kind in (('odom', Odometry), ('joint_states', JointState), ('cmd_vel', Twist)):
                topic = '/'+robot+'/'+suffix
                self.subscribers.append(rospy.Subscriber(topic, kind,
                    lambda msg, name=topic: self.receive(name, msg), queue_size=1000))
        for topic in ('/tf', '/tf_static'):
            self.subscribers.append(rospy.Subscriber(topic, TFMessage,
                lambda msg, name=topic: self.receive(name, msg), queue_size=1000))

    def receive(self, topic, msg):
        wire = io.BytesIO()
        msg.serialize(wire)
        received = time.monotonic_ns()
        row = {'topic': topic, 'source_type': msg._type, 'receive_monotonic_ns': received,
               'ros_time_at_receive_ns': self.rospy.Time.now().to_nsec(),
               'connection_header': dict(getattr(msg, '_connection_header', {})),
               'serialized_ros_base64': base64.b64encode(wire.getvalue()).decode('ascii'),
               'data': ros_value(msg)}
        row['source_stamp_ns'] = msg.header.stamp.to_nsec() if hasattr(msg, 'header') else None
        if hasattr(msg, 'transforms'):
            row['transform_stamp_ns'] = [v.header.stamp.to_nsec() for v in msg.transforms]
        with self.lock:
            row['event_id'] = len(self.events)+1
            self.events.append(row)

    def pause(self, paused):
        from std_srvs.srv import Empty
        self.call('/gazebo/'+('pause_physics' if paused else 'unpause_physics'), Empty)

    def snapshot(self, extended=False):
        from gazebo_msgs.srv import GetModelState, GetWorldProperties
        before_host = time.monotonic_ns()
        before = self.call('/gazebo/get_world_properties', GetWorldProperties)
        models = {name: ros_value(self.call('/gazebo/get_model_state', GetModelState,
                  model_name=name, relative_entity_name='world')) for name in ROBOTS}
        after = self.call('/gazebo/get_world_properties', GetWorldProperties)
        row = {'host_monotonic_before_ns': before_host, 'host_monotonic_after_ns': time.monotonic_ns(),
               'world_time_before_ns': round(before.sim_time*1e9),
               'world_time_after_ns': round(after.sim_time*1e9),
               'world_before': ros_value(before), 'world_after': ros_value(after), 'models': models,
               'time_basis': 'World simulation-time service brackets; GetModelState header is API ROS time, not a guaranteed native acquisition stamp.'}
        if extended:
            from gazebo_msgs.srv import GetJointProperties, GetModelProperties, GetPhysicsProperties
            row['properties'] = {name: ros_value(self.call('/gazebo/get_model_properties', GetModelProperties,
                                 model_name=name)) for name in ROBOTS}
            row['joints'] = {name: {side: ros_value(self.call('/gazebo/get_joint_properties', GetJointProperties,
                joint_name=name+'::'+name+'_'+side+'_joint')) for side in ('left', 'right')} for name in ROBOTS}
            row['physics'] = ros_value(self.call('/gazebo/get_physics_properties', GetPhysicsProperties))
        return row

    def publish(self, inputs, label):
        from geometry_msgs.msg import Twist
        for name in ROBOTS:
            msg = Twist()
            msg.linear.x, msg.angular.z = inputs[name]
            row = {'command_id': len(self.commands)+1, 'label': label, 'robot': name,
                   'topic': '/'+name+'/cmd_vel', 'publish_monotonic_ns': time.monotonic_ns(),
                   'ros_time_at_publish_ns': self.rospy.Time.now().to_nsec(),
                   'connected_subscribers': self.publishers[name].get_num_connections(), 'data': ros_value(msg)}
            self.publishers[name].publish(msg)
            self.commands.append(row)

    def ready(self):
        deadline = time.monotonic()+8
        while not all(p.get_num_connections() >= 2 for p in self.publishers.values()):
            require(time.monotonic() < deadline, 'Native command subscriber and raw input recorder did not handshake')
            time.sleep(.02)
        return self.graph()

    def graph(self):
        caller, master = self.rospy.get_name(), os.environ['ROS_MASTER_URI']
        graph = rpc(master, 'getSystemState', caller)
        pub, sub, srv = [{topic: nodes for topic, nodes in rows} for rows in graph]
        require(rpc(master, 'getParam', caller, '/use_sim_time') is True, 'Private Master does not use simulation time')
        require(srv.get('/gazebo/get_model_state') == ['/gazebo'], 'Truth service is not provided by actual /gazebo')
        for name in ROBOTS:
            command = '/'+name+'/cmd_vel'
            require(set(pub.get(command, [])) == {caller}, 'Unexpected command publisher can interfere with the experiment')
            require(set(sub.get(command, [])) == {caller, '/gazebo'}, 'Namespaced command subscriber topology differs')
            require(pub.get('/'+name+'/odom') == ['/gazebo'], 'Odometry does not originate in actual Gazebo')
            require(pub.get('/'+name+'/joint_states') == ['/gazebo'], 'Wheel feedback does not originate in actual Gazebo')
        require('/cmd_vel' not in sub and '/odom' not in pub, 'An unscoped drive topic leaks out of robot namespaces')
        nodes = {}
        for name in sorted({caller, '/gazebo'} | set(pub.get('/tf_static', []))):
            uri = rpc(master, 'lookupNode', caller, name)
            pid = rpc(uri, 'getPid', caller)
            nodes[name] = {'uri': uri, 'process': process(pid, os.environ['GAZEBOTEACH_SESSION_TOKEN']),
                           'bus_info': rpc(uri, 'getBusInfo', caller)}
        gazebo = nodes['/gazebo']
        names = {Path(v).name for v in gazebo['process']['mapped_native_libraries']}
        require(names == {'libgazebo_ros_api_plugin.so', 'libgazebo_ros_diff_drive.so'},
                'Actual Gazebo process has not mapped both native ROS API and wheel drive plugins')
        native_inputs = [row for row in gazebo['bus_info'] if row[2] == 'i' and row[5]]
        for robot in ROBOTS:
            require(sum(row[4] == '/'+robot+'/cmd_vel' for row in native_inputs) == 1,
                    'Actual Gazebo TCPROS input connection is missing or duplicated')
        return {'master_uri': master, 'gazebo_master_uri': os.environ['GAZEBO_MASTER_URI'],
                'master_process': process(rpc(master, 'getPid', caller), os.environ['GAZEBOTEACH_SESSION_TOKEN']),
                'system_state': graph, 'topic_types': rpc(master, 'getTopicTypes', caller), 'nodes': nodes,
                'worker_uuid': self.worker_uuid,
                'native_topic_isolation_note': 'Both plugins share the /gazebo process; the two independently connected topic names and selective physical responses establish isolation.'}

    def collect(self, duration, inputs, label):
        rows, deadline = [], time.monotonic()+max(20, duration*8)
        start = self.snapshot()
        rows.append(start)
        self.last_collection = {'label': label, 'truth': rows}
        while sim(rows[-1])-sim(start) < duration:
            require(time.monotonic() < deadline, 'Simulation failed to advance during '+label)
            self.publish(inputs, label)
            time.sleep(.02)
            rows.append(self.snapshot())
        return rows

    def phase(self, name, inputs):
        zeros = {robot: (0., 0.) for robot in ROBOTS}
        with self.lock:
            first = len(self.events)
        entry = self.snapshot(extended=True)
        result = {'name': name, 'inputs': inputs, 'entry': entry}
        self.last_phase = result
        self.publish(inputs, name+'_paused_input')
        time.sleep(.04)  # Native plugin's wall callback queue is active while paused.
        self.pause(False)
        result['moving_truth'] = self.collect(MOVE_SECONDS, inputs, name)
        self.pause(True)
        result['moving_end'] = self.snapshot(extended=True)
        self.publish(zeros, name+'_stop')
        time.sleep(.04)
        self.pause(False)
        result['stopping_truth'] = self.collect(STOP_SECONDS, zeros, name+'_stopping')
        self.pause(True)
        result['final'] = self.snapshot(extended=True)
        time.sleep(.12)
        with self.lock:
            result['event_ids'] = [v['event_id'] for v in self.events[first:]]
        return result


def inspect_snapshot(row):
    require(row['world_time_before_ns'] <= row['world_time_after_ns'] and
            row['host_monotonic_before_ns'] <= row['host_monotonic_after_ns'], 'Truth brackets are reversed')
    require({'alpha', 'beta'} <= set(row['world_after']['model_names']), 'Actual world lost one robot identity')
    for robot in ROBOTS:
        value = state(row, robot)
        require(value['success'], 'Actual GetModelState query failed')
        values = [v for key in ('position', 'orientation') for v in value['pose'][key].values()]
        values += [v for key in ('linear', 'angular') for v in value['twist'][key].values()]
        require(all(math.isfinite(v) for v in values), 'Actual model pose/twist contains nonfinite values')
        yaw(value['pose'])


def inspect_frames(events):
    parents, callers, samples = {}, {}, {}
    for event in events:
        if event['topic'] not in ('/tf', '/tf_static'):
            continue
        for transform in event['data']['transforms']:
            child, parent = transform['child_frame_id'], transform['header']['frame_id']
            parents.setdefault(child, set()).add(parent)
            callers.setdefault(child, set()).add(event['connection_header'].get('callerid'))
            samples.setdefault(child, []).append((event, transform))
    expected = {robot+'/odom': 'world' for robot in ROBOTS}
    expected.update({robot+'/base_footprint': robot+'/odom' for robot in ROBOTS})
    require(set(parents) == set(expected), 'TF contains missing, leaked or duplicate unscoped children: '+repr(parents))
    for child, parent in expected.items():
        require(parents[child] == {parent} and len(callers[child]) == 1, 'TF child has conflicting parent/authority: '+child)
        if child.endswith('/odom'):
            require(all(event['topic'] == '/tf_static' for event, _ in samples[child]), 'World anchor is not actual static TF')
            for _, transform in samples[child]:
                value = transform['transform']
                require(max(abs(v) for v in value['translation'].values()) < 1e-10 and
                        abs(value['rotation']['w']-1) < 1e-10 and
                        max(abs(value['rotation'][key]) for key in ('x', 'y', 'z')) < 1e-10,
                        'World-source odometry anchor applies an incorrect spawn transform')
        else:
            require(callers[child] == {'/gazebo'}, 'Dynamic robot pose TF is not native Gazebo output')
    return samples, {'child_to_parent': {k: list(v)[0] for k, v in parents.items()},
                     'child_authority': {k: list(v)[0] for k, v in callers.items()}}


def inspect_feedback(events, truth):
    samples, result = inspect_frames(events)
    result['robots'] = {}
    for robot in ROBOTS:
        odom = [v for v in events if v['topic'] == '/'+robot+'/odom']
        joint = [v for v in events if v['topic'] == '/'+robot+'/joint_states']
        require(len(odom) >= 150 and len(joint) >= 150, 'Insufficient actual odometry/wheel stream')
        require(all(v['source_stamp_ns'] >= u['source_stamp_ns'] for u, v in zip(odom, odom[1:])), 'Odometry source stamps regress')
        transforms = {v['header']['stamp']['secs']*1000000000+v['header']['stamp']['nsecs']: v
                      for _, v in samples[robot+'/base_footprint']}
        matched, max_pose, max_yaw, max_v, max_w = 0, 0., 0., 0., 0.
        for event in odom[::max(1, len(odom)//100)]:
            value = event['data']
            require(value['header']['frame_id'] == robot+'/odom' and value['child_frame_id'] == robot+'/base_footprint',
                    'Actual odometry frame IDs leak/collide')
            require(event['connection_header'].get('callerid') == '/gazebo', 'Odometry source authority differs')
            stamp = event['source_stamp_ns']
            require(stamp in transforms, 'Native odometry has no same-stamp TF counterpart')
            transform = transforms[stamp]['transform']
            require(max(abs(transform['translation'][k]-value['pose']['pose']['position'][k]) for k in ('x', 'y', 'z')) < 1e-9 and
                    max(abs(transform['rotation'][k]-value['pose']['pose']['orientation'][k]) for k in ('x', 'y', 'z', 'w')) < 1e-9,
                    'Same-stamp odometry and TF pose disagree')
            nearest = min(truth, key=lambda row: abs(sim(row)-stamp/1e9))
            dt = abs(sim(nearest)-stamp/1e9)
            if dt > .12:
                continue
            actual, pose = state(nearest, robot), value['pose']['pose']
            pe = distance(actual['pose'], pose)
            ye = abs(angle(yaw(actual['pose'])-yaw(pose)))
            v, side = body_velocity(actual)
            ve = max(abs(v-value['twist']['twist']['linear']['x']), abs(side-value['twist']['twist']['linear']['y']))
            we = abs(actual['twist']['angular']['z']-value['twist']['twist']['angular']['z'])
            speed = math.hypot(actual['twist']['linear']['x'], actual['twist']['linear']['y'])
            require(pe < .025+speed*dt and ye < .065+abs(actual['twist']['angular']['z'])*dt,
                    'World-source odometry pose disagrees with independent Gazebo truth')
            # At abrupt command boundaries different acquisition instants can
            # legitimately straddle a motor update. Compare steady-state twist.
            if dt < .025:
                max_v, max_w = max(max_v, ve), max(max_w, we)
            max_pose, max_yaw, matched = max(max_pose, pe), max(max_yaw, ye), matched+1
        require(matched >= 35, 'Too few odometry/TF samples independently matched to native truth time')
        expected_joints = {robot+'_left_joint', robot+'_right_joint'}
        require(all(set(v['data']['name']) == expected_joints and len(v['data']['position']) == 2 for v in joint),
                'Actual wheel JointState identities/positions differ')
        # Upstream 2.9.3 publishes positions only; no synthetic velocity field.
        require(all(not v['data']['velocity'] and not v['data']['effort'] for v in joint),
                'Unexpected wheel feedback semantics for installed upstream plugin')
        result['robots'][robot] = {'odom_count': len(odom), 'wheel_message_count': len(joint),
            'truth_matched_count': matched, 'max_pose_error_m': max_pose, 'max_yaw_error_rad': max_yaw,
            'diagnostic_nearby_twist_error_mps': max_v, 'diagnostic_nearby_angular_error_rps': max_w}
    return result


def inspect_phase(result, events):
    rows, final = result['moving_truth'], result['final']
    require(len(rows) >= 15 and sim(rows[-1])-sim(rows[0]) >= MOVE_SECONDS, 'Insufficient real physical trajectory')
    checked = {'name': result['name'], 'robots': {}}
    for robot in ROBOTS:
        entry, end = state(result['entry'], robot), state(result['moving_end'], robot)
        before, after = entry['pose'], end['pose']
        dx, dy = after['position']['x']-before['position']['x'], after['position']['y']-before['position']['y']
        heading = yaw(before)
        forward = math.cos(heading)*dx+math.sin(heading)*dy
        lateral = -math.sin(heading)*dx+math.cos(heading)*dy
        turn = sum(angle(yaw(state(b, robot)['pose'])-yaw(state(a, robot)['pose']))
                   for a, b in zip([result['entry']]+rows, rows+[result['moving_end']]))
        v_cmd, w_cmd = result['inputs'][robot]
        native_v = [body_velocity(state(row, robot))[0] for row in rows]
        native_w = [state(row, robot)['twist']['angular']['z'] for row in rows]
        span = sim(result['moving_end'])-sim(result['entry'])
        if v_cmd:
            require(forward > v_cmd*MOVE_SECONDS*.55 and abs(forward-v_cmd*span) < .08+v_cmd*span*.25,
                    robot+' command did not create expected real forward displacement')
            require(abs(lateral) < .055 and abs(turn) < .15, robot+' forward command produced excessive lateral drift/yaw')
            require(max(native_v) > v_cmd*.65, robot+' has no actual positive physical forward velocity')
        elif w_cmd:
            require(turn*w_cmd > 0 and abs(turn) > abs(w_cmd)*MOVE_SECONDS*.45 and
                    abs(turn-w_cmd*span) < .2+abs(w_cmd*span)*.35,
                    robot+' turn command has wrong physical yaw sign/magnitude')
            require(math.hypot(dx, dy) < .08 and max(w_cmd*v for v in native_w) > w_cmd*w_cmd*.5,
                    robot+' lacks actual physical angular response or drifts too far')
        else:
            require(math.hypot(dx, dy) < .03 and abs(turn) < .06 and
                    max(abs(v) for v in native_v) < .04 and max(abs(v) for v in native_w) < .08,
                    robot+' moved in response to the other robot namespace command')
        # Real wheel joint angles must move, with differential-drive signs.
        deltas = {side: result['moving_end']['joints'][robot][side]['position'][0]
                  -result['entry']['joints'][robot][side]['position'][0] for side in ('left', 'right')}
        if v_cmd:
            require(min(deltas.values()) > 1., 'Forward body motion lacks actual positive wheel rotation')
        elif w_cmd:
            require(deltas['left']*w_cmd < -.15 and deltas['right']*w_cmd > .15,
                    'Turn body motion lacks opposite physical wheel rotation')
        tail = [row for row in result['stopping_truth'] if sim(final)-sim(row) < .3]
        require(len(tail) >= 4, 'Stop verification lacks a real terminal sample window')
        for row in tail+[final]:
            value = state(row, robot)
            require(math.hypot(value['twist']['linear']['x'], value['twist']['linear']['y']) < .025 and
                    abs(value['twist']['angular']['z']) < .06, 'Zero command did not stop actual physical velocity')
        require(distance(state(tail[0], robot)['pose'], state(final, robot)['pose']) < .012,
                'Robot continues physically travelling in the stop tail')
        # At the paused final boundary, native odometry must also report stop.
        candidates = [event for event in events if event['topic'] == '/'+robot+'/odom' and
                      event['event_id'] in result['event_ids']]
        require(candidates, 'Phase has no actual odometry feedback')
        moving_candidates = [event for event in candidates if event['source_stamp_ns']/1e9 <= sim(result['moving_end'])]
        require(moving_candidates, 'Physical motion phase lacks actual odometry')
        moving_event = moving_candidates[-1]
        require(0 <= sim(result['moving_end'])-moving_event['source_stamp_ns']/1e9 < .12,
                'Actual moving odometry is too old for the paused native checkpoint')
        moving_feedback = moving_event['data']['twist']['twist']
        native_forward, native_side = body_velocity(end)
        require(max(abs(moving_feedback['linear']['x']-native_forward),
                    abs(moving_feedback['linear']['y']-native_side)) < .035 and
                abs(moving_feedback['angular']['z']-end['twist']['angular']['z']) < .10,
                'Actual moving odometry velocity disagrees with independently queried Gazebo velocity')
        if v_cmd:
            require(moving_feedback['linear']['x'] > v_cmd*.6, 'Actual odometry never confirms forward physical velocity')
        elif w_cmd:
            require(moving_feedback['angular']['z']*w_cmd > w_cmd*w_cmd*.4,
                    'Actual odometry never confirms physical yaw velocity/sign')
        feedback = candidates[-1]['data']['twist']['twist']
        require(math.hypot(feedback['linear']['x'], feedback['linear']['y']) < .025 and abs(feedback['angular']['z']) < .06,
                'Actual odometry did not report the stopped robot')
        checked['robots'][robot] = {'forward_m': forward, 'lateral_m': lateral, 'yaw_rad': turn,
            'actual_elapsed_sim_seconds': span, 'peak_native_forward_mps': max(native_v),
            'peak_abs_native_yaw_rps': max(abs(v) for v in native_w), 'physical_wheel_delta_rad': deltas,
            'moving_odom_twist': moving_feedback,
            'stopped_native_twist': state(final, robot)['twist'], 'stopped_odom_twist': feedback}
    return checked


def exercise(probe, parameters):
    zeros = {name: (0., 0.) for name in ROBOTS}
    result = {'parameters': parameters, 'phases': []}
    probe.progress = result
    result['initial_graph'] = probe.ready()
    probe.publish(zeros, 'warmup_zero')
    time.sleep(.05)
    probe.pause(False)
    result['warmup_truth'] = probe.collect(.5, zeros, 'warmup_zero')
    probe.pause(True)
    result['initial'] = probe.snapshot(extended=True)
    for robot, expected_y in (('alpha', -.8), ('beta', .8)):
        pose = state(result['initial'], robot)['pose']
        require(abs(pose['position']['x']) < .025 and abs(pose['position']['y']-expected_y) < .025
                and abs(yaw(pose)) < .06, 'Actual distinct model spawn identity/position differs')
    v, w = parameters['linear_speed_mps'], parameters['angular_speed_rps']
    for name, inputs in (('alpha_forward_beta_stationary', {'alpha': (v, 0.), 'beta': (0., 0.)}),
                         ('beta_positive_turn_alpha_stationary', {'alpha': (0., 0.), 'beta': (0., w)}),
                         ('simultaneous_different_commands', {'alpha': (v, 0.), 'beta': (0., -w)})):
        phase = probe.phase(name, inputs)
        result['phases'].append(phase)
        phase['checked'] = inspect_phase(phase, probe.events)
    result['final'] = probe.snapshot(extended=True)
    result['final_graph'] = probe.graph()
    truth = result['warmup_truth']+[result['initial']]
    for phase in result['phases']:
        truth += [phase['entry']]+phase['moving_truth']+[phase['moving_end']]+phase['stopping_truth']+[phase['final']]
    for row in truth:
        inspect_snapshot(row)
    require(result['initial']['properties'] == result['final']['properties'], 'Actual source robot model identities/topology changed')
    for robot in ROBOTS:
        properties = result['initial']['properties'][robot]
        require(not properties['is_static'] and
                {robot+'_left_joint', robot+'_right_joint'} <= set(properties['joint_names']),
                'Actual robot is not a dynamic model with the two expected physical wheel joints')
    require(result['final']['physics']['pause'], 'Independent test did not leave the owned world paused')
    with probe.lock:
        recorded = list(probe.events)
    require(all(b['event_id'] > a['event_id'] for a, b in zip(recorded, recorded[1:])), 'Raw callback append order changed')
    # Independent rospy topic callback threads can acquire the lock in a
    # different order from their entry clocks. Keep both clocks untouched and
    # check receipt order only within each original TCPROS publisher stream.
    for topic in {event['topic'] for event in recorded}:
        stream = [event for event in recorded if event['topic'] == topic]
        for caller in {event['connection_header'].get('callerid') for event in stream}:
            source = [event for event in stream if event['connection_header'].get('callerid') == caller]
            require(all(b['receive_monotonic_ns'] >= a['receive_monotonic_ns'] for a, b in zip(source, source[1:])),
                    'Original callback clock order changed within a publisher stream')
    result['feedback_checks'] = inspect_feedback(recorded, truth)
    result['source_model_identity_unchanged'] = True
    return result


def worker(output):
    result, probe = {'passed': False}, None
    try:
        parameters = json.loads(Path('parameters.json').read_text())
        probe = Probe()
        result['exercise'] = exercise(probe, parameters)
        result['passed'] = True
    except Exception as error:
        result['error'] = type(error).__name__+': '+str(error)
        if probe is not None:
            result['partial_exercise'] = getattr(probe, 'progress', None)
            result['last_phase'] = getattr(probe, 'last_phase', None)
            result['last_collection'] = getattr(probe, 'last_collection', None)
            try:
                result['failure_snapshot_before_cleanup'] = probe.snapshot(extended=True)
            except Exception as diagnostic:
                result['failure_snapshot_error'] = str(diagnostic)
    finally:
        if probe is not None:
            with probe.lock:
                result['unmodified_ros_events'] = list(probe.events)
            result['controller_inputs'] = list(probe.commands)
            result['worker_uuid'] = probe.worker_uuid
        output.parent.mkdir(parents=True, exist_ok=True)
        save(output, result)  # Preserve diagnostics before any emergency command.
        if probe is not None:
            try:
                probe.publish({name: (0., 0.) for name in ROBOTS}, 'worker_exit_zero')
            except Exception:
                pass
    return 0 if result['passed'] else 1


def run_worker(session, artifacts, name):
    # Unique report and log names ensure an old successful file cannot satisfy
    # this run. Commands are launched into a token-owned process group.
    identity = uuid.uuid4().hex
    output = artifacts/(name+'-'+identity+'.json')
    log = output.with_suffix('.log')
    argv = [sys.executable, str(Path(__file__).resolve()), '--worker', '--report', str(output)]
    command = shlex.join(argv)+' > '+shlex.quote(str(log))+' 2>&1'
    child = session._spawn('Independent L33 physics and namespace observer', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        status = child.wait(timeout=150)
    except BaseException:
        terminate_owned(child)
        raise
    require(output.is_file(), 'Independent worker produced no report; inspect '+str(log))
    raw = json.loads(output.read_text())
    require(status == 0 and raw['passed'], 'Independent two-robot acceptance failed: '+str(raw.get('error'))+'; '+str(output))
    return raw, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--worker', action='store_true')
    args = parser.parse_args()
    if args.worker:
        return worker(args.report)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == 'classic11-noetic:33']
    require(len(found) == 1, 'Expected one published L33 manifest: '+repr(errors))
    lesson, started, sessions = found[0], time.monotonic(), []
    source = hashes(lesson.folder)
    report = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'test_algorithm': __doc__, 'official_plugin_semantics_source': SOURCE_URL,
              'graphical_checks_performed': False, 'test_writes_model_pose_or_velocity': False}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l33-private-') as temporary:
        try:
            cases = (('baseline', {'linear_speed_mps': .20, 'angular_speed_rps': .60}),
                     ('linear_speed_only', {'linear_speed_mps': .30, 'angular_speed_rps': .60}),
                     ('angular_speed_only', {'linear_speed_mps': .20, 'angular_speed_rps': .90}))
            for name, parameters in cases:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', parameters)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                ros_home = session.session_dir/'ros-home'
                (ros_home/'log').mkdir(parents=True, exist_ok=True)
                session.env.update(ROS_HOME=str(ros_home), ROS_LOG_DIR=str(ros_home/'log'))
                require(len({s.token for _, s in sessions}) == len(sessions) and
                        len({s.env['ROS_MASTER_URI'] for _, s in sessions}) == len(sessions) and
                        len({s.env['GAZEBO_MASTER_URI'] for _, s in sessions}) == len(sessions),
                        'Cases reused a session identity or transport master')
                session.start(gui=False, terminals=False)
                immutable = hashes(workspace)
                raw, path = run_worker(session, artifacts, name)
                result = raw['exercise']
                require(hashes(workspace) == immutable, 'Independent physical test modified practice inputs')
                case = {'name': name, 'passed': True, 'parameters': parameters, 'raw_report': str(path),
                        'session_token': session.token, 'ros_master_uri': session.env['ROS_MASTER_URI'],
                        'gazebo_master_uri': session.env['GAZEBO_MASTER_URI'],
                        'worker_uuid': raw['worker_uuid'], 'input_sha256': immutable,
                        'checked_phases': [p['checked'] for p in result['phases']],
                        'feedback_checks': result['feedback_checks'], 'source_model_identity_unchanged': True}
                report['scenarios'].append(case)
                for relative in ('parameters.json', 'world.sdf', 'build/multi_robot.json'):
                    if (workspace/relative).is_file():
                        shutil.copy2(str(workspace/relative), str(artifacts/(name+'-'+Path(relative).name)))
                # Evidence is already outside disposable Storage before cleanup.
                session.stop(reason='L33 independent physical scenario complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            base, linear, angular = [case['checked_phases'] for case in report['scenarios']]
            base_dx = base[0]['robots']['alpha']['forward_m']
            linear_dx = linear[0]['robots']['alpha']['forward_m']
            base_yaw = base[1]['robots']['beta']['yaw_rad']
            angular_yaw = angular[1]['robots']['beta']['yaw_rad']
            require(linear_dx > base_dx*1.25 and linear_dx-base_dx > .10,
                    'Changing only linear speed did not measurably increase native physical displacement')
            require(angular_yaw > base_yaw*1.20 and angular_yaw-base_yaw > .20,
                    'Changing only angular speed did not measurably increase native physical yaw')
            require(abs(linear[1]['robots']['beta']['yaw_rad']-base_yaw) < .12 and
                    abs(angular[0]['robots']['alpha']['forward_m']-base_dx) < .06,
                    'Single-variable cases unexpectedly changed the other physical command response')
            report['single_variable_physical_effects'] = {'baseline_forward_m': base_dx,
                'linear_only_forward_m': linear_dx, 'baseline_positive_yaw_rad': base_yaw,
                'angular_only_positive_yaw_rad': angular_yaw}
            report['passed'] = True
        except Exception as error:
            report['error'] = type(error).__name__+': '+str(error)
        finally:
            report['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    # Copy startup/partial diagnostics before stopping as well.
                    if session.log_path.exists():
                        shutil.copy2(str(session.log_path), str(artifacts/(name+'-before-cleanup-runtime.log')))
                    session.stop(reason='L33 final independent cleanup')
                    if session.log_path.exists():
                        shutil.copy2(str(session.log_path), str(artifacts/(name+'-runtime.log')))
                    metadata = session.session_dir/'session.json'
                    if metadata.is_file():
                        shutil.copy2(str(metadata), str(artifacts/(name+'-session.json')))
                    ros_logs = session.session_dir/'ros-home/log'
                    if ros_logs.is_dir():
                        shutil.copytree(str(ros_logs), str(artifacts/(name+'-ros-logs')), dirs_exist_ok=True)
                except Exception as error:
                    report['cleanup_errors'].append(name+': '+str(error))
            deadline = time.monotonic()+5
            while True:
                report['leftovers'] = {session.token: live_token_processes(session.token) for _, session in sessions}
                if not any(report['leftovers'].values()) or time.monotonic() >= deadline:
                    break
                time.sleep(.05)
            report['source_templates_unchanged'] = source == hashes(lesson.folder)
            report['elapsed_seconds'] = round(time.monotonic()-started, 3)
            if report['cleanup_errors'] or any(report['leftovers'].values()) or not report['source_templates_unchanged']:
                report['passed'] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key not in ('scenarios', 'source_sha256')},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
