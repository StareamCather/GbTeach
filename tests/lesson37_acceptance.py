#!/usr/bin/env python3
"""Independently observe a real two-joint torque-driven arm pushing a real block.

The separate rospy worker never imports lesson kinematics or measurements. It
queries actual Gazebo joints, links and block poses, retains original serialized
stock force/torque and contact messages, and brackets every sensor receipt with
native world queries. A disabled-actuator failure is preserved before a native
model reset and successful recovery. A fresh duration-only case must change the
actual trajectory timing. Terminal HOLD is measured while physics advances;
holding position may require nonzero real torque and contact force.

CSV coverage is duration-aware: at least 12 unique increasing sample times,
duration+.8 s of real time span, gaps no larger than .75 s, PUSH samples covering
both trajectory ends within .65 s, and measured hold/terminal stages. Every row
is recomputed; at least 80% must match independent native poses and distinct
original FT stamps. Formal v1's valid 4 s run had 18 rows over 6.7875 s (10 PUSH,
2 HOLD, 1 STOPPING); a fixed 20-row cutoff wrongly rejected that complete run.
"""
import argparse
import base64
import csv
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
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson23_acceptance import Probe as ServiceProbe
from tests.lesson23_acceptance import hashes, ros_value, save, sha
from tests.lesson33_acceptance import rpc, yaw
from tests.lesson36_acceptance import proc, receipt_world_interval

ARM, BLOCK = 'push_arm', 'push_block'
JOINTS = ('shoulder_joint', 'elbow_joint')
LINKS = ('upper_arm', 'forearm', 'pusher')
CONTROL = '/lesson37'
TOPICS = {'state': CONTROL+'/state', 'configuration': CONTROL+'/configuration',
          'wrench': '/arm_push/wrench', 'contact': '/arm_push/contacts'}
LENGTH = .35
OFFSET = math.acos(4./7.)
FT_SOURCE = ('https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/'
             '2.9.3/gazebo_plugins/src/gazebo_ros_ft_sensor.cpp')


def require(value, message):
    if not value:
        raise AssertionError(message)


def sim(row):
    return (row['world_time_before_ns']+row['world_time_after_ns'])/2e9


def xy(pose):
    return tuple(pose['position'][key] for key in ('x', 'y'))


def tip(row):
    return xy(row['links']['pusher']['link_state']['pose'])


def block(row):
    return xy(row['models'][BLOCK]['pose'])


def length(value):
    return math.sqrt(sum(v*v for v in value))


def separation(a, b):
    return length([x-y for x, y in zip(a, b)])


def fk(row):
    a = -OFFSET+row['joints'][JOINTS[0]]['position'][0]
    b = 2*OFFSET+row['joints'][JOINTS[1]]['position'][0]
    return (LENGTH*(math.cos(a)+math.cos(a+b)), LENGTH*(math.sin(a)+math.sin(a+b)))


def desired(elapsed, duration):
    u = min(1., max(0., (elapsed-.8)/duration))
    return (.4+.24*u*u*(3-2*u), 0.)


def state(event):
    return json.loads(event['data']['data'])


def window(events, start, end):
    return [event for event in events if start <= event['receive_monotonic_ns'] <= end]


class Probe(ServiceProbe):
    def __init__(self):
        import rospy
        from gazebo_msgs.msg import ContactsState
        from geometry_msgs.msg import WrenchStamped
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        self.worker_uuid = uuid.uuid4().hex
        rospy.init_node('independent_l37_'+self.worker_uuid, disable_signals=True)
        self.rospy, self.lock = rospy, threading.Lock()
        self.recording, self.events, self.cache = True, [], {}
        self.epoch, self.latest_clock = 0, None
        self.latest_ros_time = 0
        self.truth, self.actions = [], []
        self.initial_native_fence = self.snapshot()
        kinds = {'state': String, 'configuration': String, 'wrench': WrenchStamped, 'contact': ContactsState}
        self.subscribers = [rospy.Subscriber(TOPICS[key], kind,
            lambda msg, topic=TOPICS[key]: self.receive(topic, msg), queue_size=2000)
            for key, kind in kinds.items()]
        self.subscribers.append(rospy.Subscriber('/clock', Clock,
            lambda msg: self.receive('/clock', msg), queue_size=4000))

    def receive(self, topic, msg):
        wire = io.BytesIO()
        msg.serialize(wire)
        row = {'topic': topic, 'source_type': msg._type,
               'receive_monotonic_ns': time.monotonic_ns(),
               'ros_time_at_receive_ns': self.rospy.Time.now().to_nsec(),
               'source_stamp_ns': msg.header.stamp.to_nsec() if hasattr(msg, 'header') else None,
               'connection_header': dict(getattr(msg, '_connection_header', {})),
               'serialized_ros_base64': base64.b64encode(wire.getvalue()).decode('ascii'),
               'data': ros_value(msg)}
        with self.lock:
            if not self.recording:
                return
            # Epochs use the local actual ROS clock (never sensor headers).
            # rospy's internal /clock callback may run before our observer's
            # own /clock callback, including during a native reset ACK.
            now = row['ros_time_at_receive_ns']
            if now+100000000 < self.latest_ros_time:
                self.epoch += 1
                self.cache.clear()
            self.latest_ros_time = now
            if topic == '/clock':
                stamp = msg.clock.to_nsec()
                row['clock_message_stamp_ns'] = stamp
                self.latest_clock = stamp
            row['clock_epoch'], row['event_id'] = self.epoch, len(self.events)+1
            self.events.append(row)
            self.cache[topic] = row

    def snapshot(self, extended=False):
        from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelState
        from gazebo_msgs.srv import GetLinkProperties, GetModelProperties, GetPhysicsProperties, GetWorldProperties
        started = time.monotonic_ns()
        before = self.call('/gazebo/get_world_properties', GetWorldProperties)
        models = {name: ros_value(self.call('/gazebo/get_model_state', GetModelState,
            model_name=name, relative_entity_name='world')) for name in (ARM, BLOCK)}
        joints = {name: ros_value(self.call('/gazebo/get_joint_properties', GetJointProperties,
            joint_name=ARM+'::'+name)) for name in JOINTS}
        links = {name: ros_value(self.call('/gazebo/get_link_state', GetLinkState,
            link_name=ARM+'::'+name, reference_frame='world')) for name in ('pusher',)}
        after = self.call('/gazebo/get_world_properties', GetWorldProperties)
        row = {'host_monotonic_before_ns': started, 'host_monotonic_after_ns': time.monotonic_ns(),
            'world_time_before_ns': round(before.sim_time*1e9), 'world_time_after_ns': round(after.sim_time*1e9),
            'world_before': ros_value(before), 'world_after': ros_value(after),
            'models': models, 'joints': joints, 'links': links,
            'time_basis': 'Sequential actual native world brackets; service states are not an atomic synchronized capture.'}
        if extended:
            row['properties'] = {name: ros_value(self.call('/gazebo/get_model_properties', GetModelProperties,
                model_name=name)) for name in (ARM, BLOCK)}
            row['link_properties'] = {name: ros_value(self.call('/gazebo/get_link_properties', GetLinkProperties,
                link_name=ARM+'::'+name)) for name in LINKS}
            row['block_properties'] = ros_value(self.call('/gazebo/get_link_properties', GetLinkProperties,
                link_name=BLOCK+'::link'))
            row['physics'] = ros_value(self.call('/gazebo/get_physics_properties', GetPhysicsProperties))
        with self.lock:
            row['clock_epoch'] = self.epoch
        self.truth.append(row)
        return row

    def pause(self, value):
        from std_srvs.srv import Empty
        return self.call('/gazebo/'+('pause_physics' if value else 'unpause_physics'), Empty)

    def action(self, name, enabled=None):
        from std_srvs.srv import SetBool, Trigger
        path = CONTROL+'/'+name
        kind = SetBool if enabled is not None else Trigger
        before, done, result = time.monotonic_ns(), threading.Event(), {}
        self.rospy.wait_for_service(path, timeout=4)
        def invoke():
            client = self.rospy.ServiceProxy(path, kind)
            try:
                result['value'] = client(data=enabled) if enabled is not None else client()
            except Exception as error:
                result['error'] = error
            finally:
                client.close()
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        deadline = time.monotonic()+18
        self.snapshot()
        while not done.wait(.02):
            require(time.monotonic() < deadline, 'Controller action wall timeout: '+path)
            self.snapshot()
        self.snapshot()
        if 'error' in result:
            raise result['error']
        response = result['value']
        require(response.success, 'Controller action rejected '+path+': '+response.message)
        row = {'name': name, 'enabled': enabled, 'host_before_ns': before,
            'host_after_ns': time.monotonic_ns(), 'response': ros_value(response)}
        self.actions.append(row)
        return row

    def reset_world(self):
        """Native all-model position reset, preserving the real FT clock.

        Stock Noetic 2.9.3 FT UpdateChild has no negative-time reset guard.
        Resetting simulation time can silence it until the old time catches
        up. Native reset_world resets every model while preserving that clock.
        """
        from std_srvs.srv import Empty
        self.pause(True)
        before = self.snapshot()
        host_before = time.monotonic_ns()
        response = self.call('/gazebo/reset_world', Empty)
        after = self.snapshot()
        require(sim(after) >= sim(before)-.002, 'Native model reset unexpectedly rewound simulation time')
        require(separation(tip(after), (.4, 0.)) < .008 and separation(block(after), (.51, 0.)) < .008 and
                max(abs(after['joints'][name]['position'][0]) for name in JOINTS) < .02,
                'Native reset_world did not restore actual arm joints, pusher and block poses')
        row = {'name': '/gazebo/reset_world', 'host_before_ns': host_before,
               'host_after_ns': time.monotonic_ns(), 'response': ros_value(response),
               'before_native': before, 'after_native': after,
               'clock_preserved': True, 'position_reset': 'All native world models; no manual pose/velocity service assignment.'}
        self.actions.append(row)
        self.action('reset')
        return row

    def collect(self, seconds, label, min_samples=2):
        rows, deadline = [self.snapshot()], time.monotonic()+max(15., seconds*8)
        self.last_collection = {'label': label, 'truth': rows}
        while sim(rows[-1])-sim(rows[0]) < seconds or len(rows) < min_samples:
            require(time.monotonic() < deadline, 'Native world clock stalled: '+label)
            time.sleep(.02)
            rows.append(self.snapshot())
        return rows

    def until(self, seconds, label, since_ns):
        rows, deadline = [self.snapshot()], time.monotonic()+max(30., seconds*8)
        self.last_collection = {'label': label, 'truth': rows}
        while True:
            with self.lock:
                event = self.cache.get(TOPICS['state'])
            phase = state(event).get('phase') if event else None
            if event and event['receive_monotonic_ns'] >= since_ns and phase in ('SUCCEEDED', 'FAILED'):
                return {'terminal': phase, 'truth': rows}
            require(time.monotonic() < deadline and sim(rows[-1])-sim(rows[0]) < seconds,
                    'Bounded mission timeout '+label+'; state='+str(phase))
            time.sleep(.02)
            rows.append(self.snapshot())

    def freeze(self):
        # Physics is already paused. Give finite subscriber queues a bounded
        # drain while continuing native fences, then close one finite cohort.
        for _ in range(3):
            self.snapshot()
            time.sleep(.02)
        with self.lock:
            self.recording = False
            cutoff, events = time.monotonic_ns(), list(self.events)
        closing = self.snapshot(extended=True)
        self.cohort = {'cutoff_monotonic_ns': cutoff, 'original_event_count': len(events),
            'last_event_id': events[-1]['event_id'] if events else None,
            'sensor_count': sum(e['topic'] in (TOPICS['wrench'], TOPICS['contact']) for e in events),
            'closing_native_host_bracket_ns': [closing['host_monotonic_before_ns'], closing['host_monotonic_after_ns']],
            'closing_native_world_bracket_ns': [closing['world_time_before_ns'], closing['world_time_after_ns']]}
        return events, closing

    def ready(self):
        caller, master, deadline = self.rospy.get_name(), os.environ['ROS_MASTER_URI'], time.monotonic()+12
        while True:
            pub, sub, srv = [{k: v for k, v in value} for value in rpc(master, 'getSystemState', caller)]
            if all(topic in pub for topic in TOPICS.values()) and all(CONTROL+'/'+key in srv for key in ('start', 'reset', 'enable_arm')):
                break
            require(time.monotonic() < deadline, 'Real sensor/controller ABI not ready')
            time.sleep(.05)
        require(rpc(master, 'getParam', caller, '/use_sim_time') is True, 'Private master does not use simulation time')
        for key in ('wrench', 'contact'):
            require(pub[TOPICS[key]] == ['/gazebo'], 'Sensor has non-native authority: '+key)
        require(srv.get('/gazebo/get_link_state') == ['/gazebo'] and srv.get('/gazebo/get_model_state') == ['/gazebo'],
                'Independent native truth services have unexpected authority')
        require(len(pub[TOPICS['state']]) == 1 and pub[TOPICS['configuration']] == pub[TOPICS['state']],
                'Mission configuration/state authorities differ')
        nodes = {}
        for name in sorted({caller, '/gazebo'} | set(pub[TOPICS['state']])):
            uri = rpc(master, 'lookupNode', caller, name)
            nodes[name] = {'uri': uri, 'process': proc(rpc(uri, 'getPid', caller)),
                           'bus_info': rpc(uri, 'getBusInfo', caller)}
        native, exact = nodes['/gazebo']['process'], Path('/usr/bin/gzserver-11.15.1')
        require(exact.is_file() and os.path.samefile(native['exe'], exact) and exact.read_bytes()[:4] == b'\x7fELF',
                'Actual server is not exact installed versioned Classic 11.15.1 ELF')
        native['exe_sha256'] = sha(exact)
        native['maps_text'] = Path('/proc', str(native['pid']), 'maps').read_text()
        libraries = sorted(set(line.split()[-1] for line in native['maps_text'].splitlines()
                               if line.split() and line.split()[-1].startswith('/') and '.so' in line))
        native['mapped_libraries'] = libraries
        for filename in ('libgazebo_ros_api_plugin.so', 'libgazebo_ros_ft_sensor.so', 'libgazebo_ros_bumper.so'):
            expected = Path('/opt/ros/noetic/lib')/filename
            require(expected.is_file() and any(os.path.samefile(path, expected) for path in libraries if Path(path).is_file()),
                    'Actual mapped stock plugin differs: '+filename)
        with self.lock:
            latest = self.cache.get(TOPICS['state'])
        metadata = state(latest).get('native_client') if latest else None
        require(isinstance(metadata, dict) and isinstance(metadata.get('pid'), int),
                'Mission does not expose its one persistent native client identity')
        client = proc(metadata['pid'])
        client['ppid'] = int(Path('/proc', str(client['pid']), 'stat').read_text().rsplit(') ', 1)[1].split()[1])
        client['pgid'], client['sid'], client['uid'] = os.getpgid(client['pid']), os.getsid(client['pid']), os.getuid()
        controller_pid = nodes[pub[TOPICS['state']][0]]['process']['pid']
        require(client['ppid'] == controller_pid == metadata['ppid'] and
                client['pgid'] == client['pid'] == client['sid'] == metadata['pgid'] == metadata['sid'] and
                metadata['uid'] == client['uid'] and metadata['session_token'] == client['token'],
                'Persistent native client lacks its actual controller parent, independent process session or primary UID/token')
        record = json.loads(Path('build/arm_push.json').read_text())
        target = Path(record['cache'])/'arm_client'
        require(client['start_ticks'] == metadata['start_time'] and os.path.samefile(client['exe'], target) and
                target.read_bytes()[:4] == b'\x7fELF' and sha(target) == record['products']['arm_client'],
                'Reported persistent client is not the actual token-owned, start-time-matched prepared native ELF')
        return {'system_state': [list(pub.items()), list(sub.items()), list(srv.items())],
            'topic_types': rpc(master, 'getTopicTypes', caller), 'nodes': nodes,
            'worker_uuid': self.worker_uuid, 'master_process': proc(rpc(master, 'getPid', caller)),
            'ros_master_uri': master, 'gazebo_master_uri': os.environ['GAZEBO_MASTER_URI'],
            'persistent_native_client': {'reported': metadata, 'process': client,
                'prepared_target': str(target.resolve()), 'prepared_target_sha256': sha(target)}}


def check_kinematics(rows):
    maximum = 0.
    for row in rows:
        require(row['world_time_before_ns'] <= row['world_time_after_ns'] and
                row['host_monotonic_before_ns'] <= row['host_monotonic_after_ns'], 'Native acquisition brackets reversed')
        require(all(math.isfinite(v) for values in (tip(row), block(row)) for v in values), 'Native body position is not finite')
        error = separation(fk(row), tip(row))
        width = (row['world_time_after_ns']-row['world_time_before_ns'])/1e9
        rate = sum(abs(row['joints'][name]['rate'][0]) for name in JOINTS)
        require(error < .006+.7*rate*width, 'Physical pusher link disagrees with measured two-joint FK and initial offsets')
        maximum = max(maximum, error)
    return {'native_samples': len(rows), 'maximum_fk_link_error_m': maximum,
            'geometry_m': [LENGTH, LENGTH], 'native_zero_offsets_rad': [-OFFSET, 2*OFFSET]}


def state_phase(events, row):
    previous = [e for e in events if e['topic'] == TOPICS['state'] and
                e['receive_monotonic_ns'] <= row['host_monotonic_before_ns']]
    return state(previous[-1]).get('phase') if previous else None


def check_hold(rows):
    require(len(rows) >= 8 and sim(rows[-1])-sim(rows[0]) >= .6, 'Terminal hold lacks an advancing native world window')
    max_error = max(separation(tip(row), (.64, 0.)) for row in rows)
    max_tip_speed = max(length([row['links']['pusher']['link_state']['twist']['linear'][k] for k in ('x', 'y')]) for row in rows)
    max_block_speed = max(length([row['models'][BLOCK]['twist']['linear'][k] for k in ('x', 'y')]) for row in rows)
    max_joint_rate = max(abs(row['joints'][name]['rate'][0]) for row in rows for name in JOINTS)
    require(max_error < .025 and max_tip_speed < .06 and max_block_speed < .06 and max_joint_rate < .4,
            'Actual unpaused terminal target hold is not stable')
    return {'actual_sim_duration_s': sim(rows[-1])-sim(rows[0]), 'max_target_error_m': max_error,
        'max_tip_speed_mps': max_tip_speed, 'max_block_speed_mps': max_block_speed,
        'max_abs_joint_rate_radps': max_joint_rate,
        'torque_semantics': 'Nonzero PD holding torque and contact force are permitted.'}


def crossing_time(rows, x):
    for before, after in zip(rows, rows[1:]):
        if tip(before)[0] <= x <= tip(after)[0] and tip(after)[0] > tip(before)[0]:
            fraction = (x-tip(before)[0])/(tip(after)[0]-tip(before)[0])
            return sim(before)+fraction*(sim(after)-sim(before))
    raise AssertionError('Actual pusher never crossed x='+str(x))


def check_success(mission, events, duration):
    rows = mission['truth']
    require(mission['terminal'] == 'SUCCEEDED' and len(rows) >= 30, 'Physical pushing mission did not succeed')
    require(separation(tip(rows[0]), (.4, 0.)) < .025, 'Physical pusher did not start from folded-arm spawn')
    moved = block(rows[-1])[0]-block(rows[0])[0]
    require(moved > .10 and .64 < block(rows[-1])[0] < .85, 'Real floor block was not pushed to the right by the arm')
    require(max(abs(block(row)[1]) for row in rows) < .045 and max(abs(tip(row)[1]) for row in rows) < .035,
            'Physical arm/block left the intended horizontal push line')
    deltas = {name: max(row['joints'][name]['position'][0] for row in rows)-
                    min(row['joints'][name]['position'][0] for row in rows) for name in JOINTS}
    require(min(deltas.values()) > .15 and min(max(abs(row['joints'][name]['rate'][0]) for row in rows) for name in JOINTS) > .035,
            'Block translation lacks actual two-joint rotation and rate')
    phases = []
    for event in events:
        if event['topic'] == TOPICS['state']:
            phase = state(event).get('phase')
            if not phases or phases[-1] != phase:
                phases.append(phase)
    required, cursor = ['SETTLE', 'PUSH', 'HOLD', 'STOPPING', 'SUCCEEDED'], 0
    for phase in phases:
        if cursor < len(required) and phase == required[cursor]:
            cursor += 1
    require(cursor == len(required) and 'FAILED' not in phases, 'Finite arm phase order is invalid: '+repr(phases))
    states = [state(event) for event in events if event['topic'] == TOPICS['state']]
    starts = [value['native']['start_sim'] for value in states if isinstance(value.get('native'), dict) and
              value['native'].get('start_sim', -1) >= 0]
    require(starts and max(starts)-min(starts) < .002, 'Mission lacks one actual native trajectory-start time')
    start_sim = starts[0]
    tracking = [separation(tip(row), desired(sim(row)-start_sim, duration)) for row in rows]
    require(max(tracking) < .035, 'Actual tool does not track independently reconstructed smoothstep desired path')
    hold = check_hold(mission['hold_truth'])
    measured_duration = crossing_time(rows, .61)-crossing_time(rows, .46)
    return {'phase_order': phases, 'block_displacement_m': moved, 'final_block_xy_m': block(rows[-1]),
        'joint_rotation_ranges_rad': deltas, 'native_crossing_046_to_061_duration_s': measured_duration,
        'native_mission_duration_s': sim(rows[-1])-sim(rows[0]), 'native_start_sim_s': start_sim,
        'independent_tracking_max_error_m': max(tracking),
        'independent_tracking_mean_error_m': sum(tracking)/len(tracking), 'terminal_hold': hold}


def check_failure(mission, events):
    rows = mission['truth']
    require(mission['terminal'] == 'FAILED', 'Disabled real actuation did not produce explicit finite failure')
    excursion = max(separation(block(row), block(rows[0])) for row in rows)
    tip_excursion = max(separation(tip(row), tip(rows[0])) for row in rows)
    final_error = separation(tip(rows[-1]), (.64, 0.))
    require(excursion < .025 and tip_excursion < .035 and final_error > .15,
            'Disabled actuator fault moved physical block/tool or hid actual target error')
    states = [state(event) for event in events if event['topic'] == TOPICS['state']]
    desires = [value.get('desired') for value in states if isinstance(value.get('desired'), dict)]
    require(desires and max(float(value.get('x', value.get('x_m', 0))) for value in desires) > .62,
            'Fault did not retain the moving desired trajectory while actual actuation was disabled')
    native = [value['native'] for value in states if isinstance(value.get('native'), dict)]
    require(len(native) >= 20 and all(value['enabled'] is False and abs(value['tau1']) < 1e-9 and
            abs(value['tau2']) < 1e-9 for value in native), 'Disabled fault lacks sustained actual native zero-torque telemetry')
    require(2. < sim(rows[-1])-sim(rows[0]) < 18., 'Disabled actuator failure was not bounded in actual simulation time')
    return {'terminal': mission['terminal'], 'maximum_block_excursion_m': excursion,
        'maximum_tip_excursion_m': tip_excursion, 'actual_final_target_error_m': final_error,
        'actual_sim_duration_s': sim(rows[-1])-sim(rows[0]), 'native_disabled_zero_torque_samples': len(native)}


def check_sensors(events, truth):
    from gazebo_msgs.msg import ContactsState
    from geometry_msgs.msg import WrenchStamped
    sensors = [event for event in events if event['topic'] in (TOPICS['wrench'], TOPICS['contact'])]
    require(sum(e['topic'] == TOPICS['wrench'] for e in sensors) >= 100 and
            sum(e['topic'] == TOPICS['contact'] for e in sensors) >= 100, 'Real force/contact streams are absent or too short')
    intervals, by_epoch = [], {}
    for event in sensors:
        require(event['connection_header'].get('callerid') == '/gazebo', 'Original force/contact message authority differs')
        kind = WrenchStamped if event['topic'] == TOPICS['wrench'] else ContactsState
        decoded = kind()
        decoded.deserialize(base64.b64decode(event['serialized_ros_base64']))
        require(ros_value(decoded) == event['data'], 'Stored sensor data differs from its original serialized ROS bytes')
        intervals.append(receipt_world_interval(event, truth))
        by_epoch.setdefault((event['topic'], event['clock_epoch']), []).append(event['source_stamp_ns'])
        if event['topic'] == TOPICS['wrench']:
            require(event['data']['header']['frame_id'] == 'pusher', 'Stock FT axes must be the actual fixed-joint child link')
            wrench = event['data']['wrench']
            require(all(math.isfinite(wrench[k][axis]) for k in ('force', 'torque') for axis in ('x', 'y', 'z')),
                    'Real FT wrench contains nonfinite values')
    for stamps in by_epoch.values():
        require(all(b >= a for a, b in zip(stamps, stamps[1:])), 'Sensor source time regressed without actual clock reset')
    return {'original_sensor_count': len(sensors), 'native_bracketed_sensor_count': len(intervals),
        'native_receipt_intervals': intervals,
        'max_conservative_physical_age_upper_s': max(value['physical_age_interval_s'][1] for value in intervals),
        'source_frame_and_direction': 'Stock 2.9.3 joint GetForceTorque(0).body2: child axes, child_to_parent.',
        'official_source': FT_SOURCE,
        'time_semantics': 'WorldUpdateBegin header can refer to prior physics feedback; service observations are sequential brackets.'}


def check_push_force(events, rows, air_rows):
    # The sphere is also a gravity-loaded body. Horizontal force isolates push
    # load without falsely claiming that the full air wrench should be zero.
    wrenches = [e for e in events if e['topic'] == TOPICS['wrench']]
    contacts = [e for e in events if e['topic'] == TOPICS['contact']]
    air_lo, air_hi = air_rows[0]['host_monotonic_before_ns'], air_rows[-1]['host_monotonic_after_ns']
    air = [e for e in wrenches if air_lo <= e['receive_monotonic_ns'] <= air_hi]
    pairs, collision_states = {}, 0
    for event in contacts:
        for item in event['data']['states']:
            pair = (item['collision1_name'], item['collision2_name'])
            if any(ARM+'::pusher::' in name for name in pair) and any(BLOCK+'::link::' in name for name in pair):
                collision_states += 1
                pairs[event['event_id']] = event
    pairs = list(pairs.values())
    require(len(air) >= 8 and len(pairs) >= 8, 'No measured air window or genuine pusher/block collision pair')
    horizontal = lambda e: math.hypot(e['data']['wrench']['force']['x'], e['data']['wrench']['force']['y'])
    air_mean = sum(horizontal(e) for e in air)/len(air)
    # Compare a sustained collision cohort, avoiding a single impact spike.
    pair_times = [event['source_stamp_ns']/1e9 for event in pairs]
    pushing = [e for e in wrenches if any(abs(e['source_stamp_ns']/1e9-t) < .035 for t in pair_times)]
    require(len(pushing) >= 15, 'Insufficient original FT feedback during actual pusher/block collision')
    push_mean = sum(horizontal(e) for e in pushing)/len(pushing)
    require(air_mean < .18 and push_mean > .15 and push_mean > air_mean+.10,
            'Real child-frame horizontal FT does not distinguish air from pushing contact')
    moving = [row for row in rows if block(row)[0] > block(rows[0])[0]+.02 and
              abs(tip(row)[0]+.04-(block(row)[0]-.06)) < .04]
    require(len(moving) >= 5, 'Contact messages lack independently measured sphere/block geometry and translation')
    return {'air_horizontal_force_mean_n': air_mean, 'contact_horizontal_force_mean_n': push_mean,
        'air_wrench_messages': len(air), 'contact_wrench_messages': len(pushing), 'real_collision_messages': len(pairs),
        'real_collision_state_items': collision_states,
        'air_force_z_mean_n': sum(e['data']['wrench']['force']['z'] for e in air)/len(air),
        'geometry_and_translation_native_samples': len(moving),
        'frame_note': 'Horizontal norm is invariant under the actual Z-axis tool rotation; vertical weight and dynamic inertia remain in raw wrench.'}


def check_artifacts(events, native_rows, expected_phase, duration, torque_limit):
    states = [state(event) for event in events if event['topic'] == TOPICS['state']]
    require(states and states[-1].get('run_id'), 'Physical mission has no preserved unique run identity')
    folder = Path('build/runs')/states[-1]['run_id']
    names = ('measurements.csv', 'measurements.svg', 'raw_ros.jsonl', 'events.jsonl', 'result.json')
    require(all((folder/name).is_file() for name in names), 'Physical mission did not preserve measurements, SVG, raw sensor and event logs')
    result = json.loads((folder/'result.json').read_text())
    require(result['phase'] == expected_phase and result['run_id'] == states[-1]['run_id'], 'Saved run result has wrong terminal phase/identity')
    client = result.get('native_client')
    require(isinstance(client, dict) and all(value.get('native_client') == client for value in states if value.get('run_id') == result['run_id']),
            'Saved run and live states do not preserve one persistent native client PID/start time')
    command_events = [json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines() if line.strip()]
    commands = [value['data'] for value in command_events if value['source'] == 'native_command']
    require(commands and all(value['process'] == client and value['ack']['id'] == value['request_id'] for value in commands),
            'Saved native commands replaced the persistent client or lack matching original native request/ACK identities')
    require(len({value['request_id'] for value in commands}) == len(commands), 'Native stream reused a per-command request identity')
    with (folder/'measurements.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    require(len(rows) >= 12 and result['measurement_count'] == len(rows), 'Saved measurement cohort is too short or differs from its actual reported count')
    times = [float(row['sim_time']) for row in rows]
    require(all(b > a for a, b in zip(times, times[1:])), 'Saved measurement times are not unique and strictly increasing')
    span, max_gap = times[-1]-times[0], max(b-a for a, b in zip(times, times[1:]))
    require(span >= duration+.8 and max_gap <= .75, 'Saved measurement cohort does not span the physical trajectory with bounded sample gaps')
    require(result.get('start_sim') is not None, 'Saved physical mission lacks an actual native start time')
    push = [float(row['sim_time'])-result['start_sim'] for row in rows if row['phase'] == 'PUSH']
    require(len(push) >= math.ceil(duration/.65) and abs(push[0]-.8) <= .65 and
            abs(push[-1]-(.8+duration)) <= .65, 'Saved PUSH measurements do not cover both actual trajectory ends at the required temporal density')
    hold_count = sum(row['phase'] in ('HOLD', 'STOPPING') for row in rows)
    require(hold_count >= 2 and rows[-1]['phase'] == expected_phase,
            'Saved physical mission lacks measured holding and terminal stages')
    coverage = {'actual_row_count': len(rows), 'actual_span_s': span, 'maximum_sample_gap_s': max_gap,
        'push_row_count': len(push), 'push_elapsed_edges_s': [push[0], push[-1]],
        'required_push_row_count': math.ceil(duration/.65), 'hold_or_stopping_rows': hold_count,
        'phase_counts': {phase: sum(row['phase'] == phase for row in rows) for phase in dict.fromkeys(row['phase'] for row in rows)},
        'minimum_actual_span_s': duration+.8, 'maximum_allowed_gap_s': .75,
        'allowed_push_edge_error_s': .65}
    force_by_stamp = {event['source_stamp_ns']: event for event in events if event['topic'] == TOPICS['wrench']}
    wire = {key: set(event['serialized_ros_base64'] for event in events if event['topic'] == TOPICS[key])
            for key in ('wrench', 'contact')}
    raw = [json.loads(line) for line in (folder/'raw_ros.jsonl').read_text().splitlines() if line.strip()]
    raw_matches = 0
    for item in raw:
        key = 'contact' if item['source'] == 'contacts' else item['source']
        require(key in wire and item['serialized_ros_base64'] in wire[key],
                'Course raw sensor bytes are absent from independently received original Gazebo stream')
        raw_matches += 1
    require(raw_matches >= 100, 'Saved raw sensor evidence is too short')
    maximum_native_error, matched_native, unique_forces = 0., 0, set()
    for item in rows:
        values = {key: float(value) for key, value in item.items() if key != 'phase'}
        require(all(math.isfinite(value) for value in values.values()), 'Saved measurement contains nonfinite numeric value')
        require(abs(values['error_m']-math.hypot(values['tip_x_m']-values['desired_x_m'], values['tip_y_m']-values['desired_y_m'])) < 1e-8,
                'Saved actual tracking error differs from actual/desired tool coordinates')
        require(abs(values['tau1_nm']) <= torque_limit+1e-8 and abs(values['tau2_nm']) <= torque_limit+1e-8,
                'Saved native actual PD torque exceeds the acknowledged physical limit')
        if item['phase'] != 'IDLE' and result.get('start_sim') is not None:
            target = desired(values['sim_time']-result['start_sim'], duration)
            require(separation((values['desired_x_m'], values['desired_y_m']), target) < 1e-7,
                    'Saved desired trajectory differs from independent smoothstep construction')
        stamp = int(item['force_stamp_ns'])
        require(stamp in force_by_stamp, 'Saved measurement references force absent from original independent ROS stream')
        force = force_by_stamp[stamp]['data']['wrench']['force']
        require(max(abs(values['force_'+axis+'_n']-force[axis]) for axis in ('x', 'y', 'z')) < 1e-8 and
                abs(values['horizontal_force_n']-math.hypot(force['x'], force['y'])) < 1e-8,
                'Saved actual force differs from original decoded real joint wrench')
        unique_forces.add(stamp)
        nearest = min(native_rows, key=lambda row: abs(sim(row)-values['sim_time']))
        dt = abs(sim(nearest)-values['sim_time'])
        if dt < .13:
            error = separation(tip(nearest), (values['tip_x_m'], values['tip_y_m']))
            require(error < .012+.15*dt and separation(block(nearest), (values['block_x_m'], values['block_y_m'])) < .014+.15*dt,
                    'Course actual tool/block trajectory disagrees with independent native physical poses')
            maximum_native_error, matched_native = max(maximum_native_error, error), matched_native+1
    required_matches = math.ceil(.8*len(rows))
    require(matched_native >= required_matches and len(unique_forces) >= required_matches,
            'Fewer than 80 percent of saved rows have independent native pose matches or distinct original FT stamps')
    # Validate every displayed SVG point against its persisted measured series.
    svg = ET.parse(str(folder/'measurements.svg')).getroot()
    lines = [node for node in svg.iter() if node.tag.rsplit('}', 1)[-1] == 'polyline']
    groups = [(('tip_x_m', 'desired_x_m', 'block_x_m'), .35, .82),
              (('tip_y_m', 'block_y_m'), -.08, .08),
              (('error_m',), 0., max(.03, max(float(r['error_m']) for r in rows)*1.1)),
              (('horizontal_force_n',), 0., max(1., max(float(r['horizontal_force_n']) for r in rows)*1.1))]
    require(len(lines) == 7, 'Saved measurement SVG has incomplete actual/desired/error/force series')
    first, span, cursor = float(rows[0]['sim_time']), max(.1, float(rows[-1]['sim_time'])-float(rows[0]['sim_time'])), 0
    for panel, (keys, low, high) in enumerate(groups):
        for key in keys:
            points = [tuple(float(value) for value in point.split(',')) for point in lines[cursor].get('points').split()]
            require(len(points) == len(rows), 'SVG measurement series lost or invented samples')
            for point, item in zip(points, rows):
                expected = (70+940*(float(item['sim_time'])-first)/span,
                            65+panel*190+145-120*(float(item[key])-low)/(high-low))
                require(separation(point, expected) < .008, 'SVG visualized point differs from actual measured CSV')
            cursor += 1
    return {'run_id': result['run_id'], 'directory': str(folder.resolve()), 'measurement_rows': len(rows),
        'temporal_coverage': coverage, 'minimum_required_native_and_distinct_force_matches': required_matches,
        'persistent_native_client': client, 'matching_native_stream_command_ack_count': len(commands),
        'matched_native_pose_rows': matched_native, 'maximum_tool_native_match_error_m': maximum_native_error,
        'original_force_stamps': len(unique_forces), 'original_serialized_sensor_rows': raw_matches,
        'validated_svg_series': cursor, 'files_sha256': {name: sha(folder/name) for name in names}}


def source_proof(graph):
    source = Path('arm_plugin.cc').read_text()
    forbidden = ('SetWorldPose', 'SetRelativePose', 'SetPose', 'SetPosition', 'SetVelocity', 'SetLinearVel', 'SetAngularVel')
    require('SetForce' in source and not any(name in source for name in forbidden),
            'Arm must actuate real physical joints by torque, without pose/velocity assignment')
    for path in (Path('mission.py'), Path('lab.py')):
        require(not any(name in path.read_text() for name in ('set_model_state', 'set_link_state', 'SetModelState', 'SetLinkState')),
                'Controller contains link/model teleport services')
    records = list(Path('build').glob('*.json'))
    records = [path for path in records if isinstance(json.loads(path.read_text()), dict) and
               'source_sha256' in json.loads(path.read_text()) and 'products' in json.loads(path.read_text())]
    require(len(records) == 1, 'Expected one frozen native arm preparation record')
    record = json.loads(records[0].read_text())
    require(all(sha(name) == digest for name, digest in record['source_sha256'].items()), 'Prepared native inputs changed')
    native = graph['nodes']['/gazebo']['process']
    package = Path('/opt/ros/noetic/share/gazebo_plugins/package.xml')
    require(package.is_file() and ET.parse(str(package)).findtext('version') == '2.9.3',
            'Installed native gazebo_plugins package is not exact Noetic 2.9.3')
    for value in record['ros_plugins'].values():
        require(sha(value['path']) == value['sha256'], 'Installed stock ROS plugin differs from frozen actual preparation identity')
    world_path = Path('world.sdf').resolve()
    require(sha(world_path) == record['world_sha256'] and any(Path(value).resolve() == world_path for value in native['argv'] if value.endswith('.sdf')),
            'Actual gzserver world differs from exact prepared physical world')
    world = ET.parse(str(world_path)).find('world')
    arm, object_model = world.find("model[@name='"+ARM+"']"), world.find("model[@name='"+BLOCK+"']")
    require(arm is not None and object_model is not None, 'Real arm or block missing from loaded world')
    require(world.findtext('gravity').split() == ['0', '0', '-9.81'], 'Loaded physical world gravity differs')
    for name in JOINTS:
        joint = arm.find("joint[@name='"+name+"']")
        require(joint is not None and joint.get('type') == 'revolute' and joint.findtext('axis/xyz').split() == ['0', '0', '1'],
                'Physical arm requires two actual Z-axis revolute joints')
    wrist = arm.find("joint[@name='wrist_joint']")
    require(wrist is not None and wrist.get('type') == 'fixed' and wrist.findtext('child') == 'pusher' and
            wrist.findtext('physics/provide_feedback') == 'true', 'Real fixed wrist joint feedback contract missing')
    ft = arm.find("plugin[@filename='libgazebo_ros_ft_sensor.so']")
    require(ft is not None and ft.findtext('jointName') == 'wrist_joint' and ft.findtext('topicName').endswith('wrench'),
            'Real stock FT model plugin on the actual wrist is missing')
    require(abs(float(object_model.findtext('link/inertial/mass'))-.2) < 1e-9 and
            abs(float(arm.findtext("link[@name='pusher']/inertial/mass"))-.05) < 1e-9,
            'Actual block/pusher source mass differs')
    plugin = next((node for node in arm.findall('plugin') if node.get('filename') != 'libgazebo_ros_ft_sensor.so'), None)
    require(plugin is not None, 'Native physical arm controller plugin missing')
    filename = plugin.get('filename')
    candidates = [Path('build')/filename] + [Path(value)/filename for value in os.environ.get('GAZEBO_PLUGIN_PATH', '').split(':') if value]
    expected = next((path for path in candidates if path.is_file()), None)
    require(expected is not None, 'Prepared native physical arm ELF not found')
    mapped = [Path(value) for value in native['mapped_libraries'] if Path(value).is_file()]
    require(expected.read_bytes()[:4] == b'\x7fELF' and any(os.path.samefile(expected, path) for path in mapped),
            'Actual gzserver did not map exact prepared/cached physical arm ELF')
    require(expected.resolve().parent == Path(record['cache']).resolve() and sha(expected) == record['products'][expected.name],
            'Loaded arm ELF differs from content-addressed prepared product')
    return {'record': str(records[0]), 'plugin_target': str(expected.resolve()), 'plugin_sha256': sha(expected),
        'arm_source_sha256': sha('arm_plugin.cc'), 'motion_primitive': 'Joint::SetForce',
        'stock_ft_sha256': sha('/opt/ros/noetic/lib/libgazebo_ros_ft_sensor.so'), 'official_ft_source': FT_SOURCE}


def exercise(probe, output, held_fault):
    parameters = json.loads(Path('parameters.json').read_text())
    result = {'parameters': parameters, 'held_actuation_fault_requested': held_fault}
    probe.progress = result
    result['initial_graph'] = probe.ready()
    result['native_source_proof'] = source_proof(result['initial_graph'])
    enabled = probe.action('enable_arm', True)
    acknowledged = json.loads(enabled['response']['message'])
    require(acknowledged['enabled'] is True and acknowledged['torque_limit_nm'] == parameters['torque_limit_nm'] and
            acknowledged['push_duration_s'] == parameters['push_duration_s'], 'Actual native enabled/cap/duration ACK differs from immutable experiment inputs')
    result['native_configuration_ack'] = acknowledged
    probe.pause(False)
    result['warmup_truth'] = probe.collect(.6, 'sensor_warmup')
    probe.pause(True)
    result['initial'] = probe.snapshot(extended=True)
    require(separation(tip(result['initial']), (.4, 0.)) < .025 and abs(block(result['initial'])[0]-.51) < .025,
            'Actual initial arm/block physical poses differ')
    require(all(not value['is_static'] for value in result['initial']['properties'].values()), 'Arm or block declared static')
    require(abs(result['initial']['block_properties']['mass']-.2) < 1e-9 and
            abs(result['initial']['link_properties']['pusher']['mass']-.05) < 1e-9,
            'Actual native physical masses differ from prepared source')
    world = ET.parse('world.sdf').find('world')
    for name, properties in list(result['initial']['link_properties'].items())+[(BLOCK, result['initial']['block_properties'])]:
        link = world.find("model[@name='"+(BLOCK if name == BLOCK else ARM)+"']/link"+
                          ("[@name='"+name+"']" if name != BLOCK else ''))
        require(properties['gravity_mode'] is True and abs(properties['mass']-float(link.findtext('inertial/mass'))) < 1e-9,
                'Actual native body mass/gravity disagrees with prepared physical source: '+name)
        for component in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            require(abs(properties[component]-float(link.findtext('inertial/inertia/'+component, '0'))) < 1e-8,
                    'Actual body inertia differs from prepared physical source: '+name+'/'+component)
    gravity = result['initial']['physics']['gravity']
    require(abs(gravity['x']) < 1e-9 and abs(gravity['y']) < 1e-9 and abs(gravity['z']+9.81) < 1e-9,
            'Native physics gravity differs from prepared world')
    if held_fault:
        probe.reset_world()
        probe.action('enable_arm', False)
        probe.pause(False)
        result['fault_air_truth'] = probe.collect(.4, 'disabled_air')
        action = probe.action('start')
        fault = probe.until(18., 'disabled_actuator', action['host_before_ns'])
        fault['hold_truth'] = probe.collect(.7, 'disabled_terminal_tail', min_samples=8)
        probe.pause(True)
        fault['closing_native_fence'] = probe.snapshot(extended=True)
        with probe.lock:
            fault_all_events = list(probe.events)
            fault_events = window(fault_all_events, action['host_before_ns'], fault['hold_truth'][-1]['host_monotonic_after_ns'])
        result['held_actuation_fault'] = fault
        save(output.with_name(output.stem+'-disabled-failure.json'), {'fault': fault, 'original_ros_events': fault_events, 'actions': list(probe.actions)})
        fault['independent_checks'] = check_failure(fault, fault_events)
        fault['artifact_checks'] = check_artifacts(fault_all_events, probe.truth, 'FAILED', parameters['push_duration_s'], parameters['torque_limit_nm'])
        result['held_actuation_fault'] = fault
        save(output.with_name(output.stem+'-disabled-failure.json'), {'fault': fault, 'original_ros_events': fault_events, 'actions': list(probe.actions)})
        states = [state(event) for event in fault_events if event['topic'] == TOPICS['state']]
        if states and states[-1].get('run_id'):
            folder = Path('build/runs')/states[-1]['run_id']
            if folder.is_dir():
                shutil.copytree(str(folder), str(output.with_name(output.stem+'-disabled-course-files')), dirs_exist_ok=True)
        before = probe.snapshot()
        probe.action('enable_arm', True)
        reset = probe.reset_world()
        after = probe.snapshot()
        result['explicit_recovery'] = {'before_reset': before, 'after_reset': after,
            'native_world_model_reset': reset, 'clock_preserved': sim(after) >= sim(before)-.002,
            'stock_ft_clock_reset_caveat': '2.9.3 UpdateChild throttle has no negative-time guard; native reset_world restores all model poses and keeps clock monotonic.'}
    else:
        probe.reset_world()
    probe.pause(False)
    result['pre_success_truth'] = probe.collect(.5, 'measured_air_before_success')
    action = probe.action('start')
    mission = probe.until(18., 'physical_push_success', action['host_before_ns'])
    mission['hold_truth'] = probe.collect(.85, 'unpaused_success_target_hold', min_samples=8)
    probe.pause(True)
    events, result['final'] = probe.freeze()
    mission_events = window(events, action['host_before_ns'], mission['hold_truth'][-1]['host_monotonic_after_ns'])
    mission['independent_checks'] = check_success(mission, mission_events, parameters['push_duration_s'])
    result['successful_mission'] = mission
    result['artifact_checks'] = check_artifacts(events, probe.truth, 'SUCCEEDED', parameters['push_duration_s'], parameters['torque_limit_nm'])
    force_events = window(events, result['pre_success_truth'][0]['host_monotonic_before_ns'], mission['hold_truth'][-1]['host_monotonic_after_ns'])
    result['push_force_checks'] = check_push_force(force_events, mission['truth'], result['pre_success_truth'])
    result['kinematic_checks'] = check_kinematics(probe.truth)
    result['sensor_checks'] = check_sensors(events, probe.truth)
    result['observation_cohort'] = probe.cohort
    require(probe.cohort['sensor_count'] == result['sensor_checks']['native_bracketed_sensor_count'], 'Persisted sensor cohort differs from validated finite cohort')
    result['final_graph'] = probe.ready()
    require(result['initial_graph']['persistent_native_client'] == result['final_graph']['persistent_native_client'],
            'Mission replaced its persistent native client during physical fault/recovery/mission observation')
    require(result['final']['physics']['pause'], 'Observer did not leave owned world paused')
    require(result['initial']['properties'] == result['final']['properties'], 'Actual arm/block identity or topology changed')
    return result


def worker(output, held_fault):
    result, probe = {'passed': False}, None
    try:
        output.parent.mkdir(parents=True, exist_ok=True)
        probe = Probe()
        result['exercise'] = exercise(probe, output, held_fault)
        result['passed'] = True
    except Exception as error:
        import traceback
        result['error'], result['traceback'] = type(error).__name__+': '+str(error), traceback.format_exc()
        if probe is not None:
            result['partial_exercise'] = getattr(probe, 'progress', None)
            result['last_collection'] = getattr(probe, 'last_collection', None)
            try:
                result['failure_snapshot_before_cleanup'] = probe.snapshot(extended=True)
            except Exception as diagnostic:
                result['failure_snapshot_error'] = str(diagnostic)
    finally:
        if probe is not None:
            with probe.lock:
                result['unmodified_ros_events'] = list(probe.events)
            result['native_truth'], result['control_actions'] = list(probe.truth), list(probe.actions)
            result['worker_uuid'], result['observation_cohort'] = probe.worker_uuid, getattr(probe, 'cohort', None)
        save(output, result)
        if probe is not None:
            try:
                probe.pause(True)
            except Exception:
                pass
    return 0 if result['passed'] else 1


def run_worker(session, artifacts, name, held_fault):
    output = artifacts/(name+'-'+uuid.uuid4().hex+'.json')
    log = output.with_suffix('.log')
    argv = [sys.executable, str(Path(__file__).resolve()), '--worker', '--report', str(output)]
    if held_fault:
        argv.append('--held-fault')
    child = session._spawn('Independent L37 ROS/physics observer', ['bash', '--noprofile', '--norc', '-c',
        shlex.join(argv)+' > '+shlex.quote(str(log))+' 2>&1'])
    try:
        status = child.wait(timeout=300)
    except BaseException:
        terminate_owned(child)
        raise
    require(output.is_file(), 'Independent worker produced no report; inspect '+str(log))
    raw = json.loads(output.read_text())
    require(status == 0 and raw['passed'], 'Independent arm acceptance failed: '+str(raw.get('error'))+'; '+str(output))
    return raw, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--held-fault', action='store_true')
    args = parser.parse_args()
    if args.worker:
        return worker(args.report.resolve(), args.held_fault)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == 'classic11-noetic:37']
    require(len(found) == 1, 'Expected one published L37 manifest: '+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    original_source = hashes(lesson.folder)
    report = {'passed': False, 'lesson': lesson.key, 'source_sha256': original_source, 'scenarios': [],
        'test_algorithm': __doc__, 'graphical_checks_performed': False,
        'test_writes_model_pose_or_velocity': False, 'test_publishes_force_or_joint_command': False,
        'official_stock_ft_source': FT_SOURCE}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l37-private-') as directory:
        try:
            baseline = json.loads((lesson.folder/'parameters.json').read_text())
            require(baseline == {'push_duration_s': 6, 'torque_limit_nm': 6}, 'Default physical parameter ABI differs')
            for name, parameters, fault in (
                    ('baseline_disabled_fault_and_explicit_recovery', baseline, True),
                    ('push_duration_only', {**baseline, 'push_duration_s': 4}, False)):
                storage = Storage(Path(directory)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', parameters)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                ros_home = session.session_dir/'ros-home'
                (ros_home/'log').mkdir(parents=True, exist_ok=True)
                session.env.update(ROS_HOME=str(ros_home), ROS_LOG_DIR=str(ros_home/'log'))
                require(len({s.token for _, s in sessions}) == len(sessions) and
                        len({s.env['ROS_MASTER_URI'] for _, s in sessions}) == len(sessions) and
                        len({s.env['GAZEBO_MASTER_URI'] for _, s in sessions}) == len(sessions), 'Cases reused private identity or transport master')
                session.start(gui=False, terminals=False)
                immutable = hashes(workspace)
                raw, path = run_worker(session, artifacts, name, fault)
                require(hashes(workspace) == immutable, 'Physical experiment modified immutable learner inputs')
                result = raw['exercise']
                report['scenarios'].append({'name': name, 'passed': True, 'parameters': parameters,
                    'raw_report': str(path), 'session_token': session.token,
                    'ros_master_uri': session.env['ROS_MASTER_URI'], 'gazebo_master_uri': session.env['GAZEBO_MASTER_URI'],
                    'worker_uuid': raw['worker_uuid'], 'input_sha256': immutable,
                    'physical_mission_checks': result['successful_mission']['independent_checks'],
                    'push_force_checks': result['push_force_checks'], 'kinematic_checks': result['kinematic_checks'],
                    'artifact_checks': result['artifact_checks'],
                    'sensor_checks': {key: value for key, value in result['sensor_checks'].items() if key != 'native_receipt_intervals'},
                    'observation_cohort': result['observation_cohort'],
                    'held_failure_checks': result.get('held_actuation_fault', {}).get('independent_checks')})
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
                session.stop(reason='L37 physical scenario complete')
            base, faster = [case['physical_mission_checks']['native_crossing_046_to_061_duration_s'] for case in report['scenarios']]
            require(base > faster*1.25 and base-faster > .45 and base < faster*1.8,
                    'Duration-only change did not measurably alter actual physical pusher transit time')
            report['single_variable_physical_effects'] = {'baseline_native_transit_s': base,
                'duration_only_native_transit_s': faster, 'observed_speed_ratio': base/faster,
                'unchanged_parameter': 'torque_limit_nm=6'}
            report['passed'] = True
        except Exception as error:
            report['error'] = type(error).__name__+': '+str(error)
        finally:
            report['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    if session.log_path.exists():
                        shutil.copy2(str(session.log_path), str(artifacts/(name+'-before-cleanup-runtime.log')))
                    for relative in ('parameters.json', 'world.sdf'):
                        source = session.workspace/relative
                        if source.is_file():
                            shutil.copy2(str(source), str(artifacts/(name+'-'+Path(relative).name)))
                    for source in (session.workspace/'build').glob('*.json'):
                        shutil.copy2(str(source), str(artifacts/(name+'-'+source.name)))
                    runs = session.workspace/'build/runs'
                    if runs.is_dir():
                        shutil.copytree(str(runs), str(artifacts/(name+'-course-runs')), dirs_exist_ok=True)
                    session.stop(reason='L37 final independent owned cleanup')
                    native_stderr = session.workspace/'build/native-client.stderr.log'
                    if native_stderr.is_file():
                        shutil.copy2(str(native_stderr), str(artifacts/(name+'-native-client.stderr.log')))
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
                    try:
                        session.stop(reason='L37 fallback owned cleanup after artifact-copy failure')
                    except Exception as cleanup:
                        report['cleanup_errors'].append(name+' stop: '+str(cleanup))
            deadline = time.monotonic()+5
            while True:
                report['leftovers'] = {session.token: live_token_processes(session.token) for _, session in sessions}
                if not any(report['leftovers'].values()) or time.monotonic() >= deadline:
                    break
                time.sleep(.05)
            report['source_templates_unchanged'] = original_source == hashes(lesson.folder)
            report['elapsed_seconds'] = round(time.monotonic()-started, 3)
            if report['cleanup_errors'] or any(report['leftovers'].values()) or not report['source_templates_unchanged']:
                report['passed'] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key not in ('source_sha256', 'scenarios')}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
