#!/usr/bin/env python3
"""Independent physical warehouse mission, blocked-door recovery and measured map.

An isolated rospy worker observes original serialized ROS messages and brackets
its own Gazebo model/joint service measurements with actual world time. It does
not import the lesson controller or its measurement helpers. Success requires
real wheel motion, a stationary blocked wait, a physical sliding door, measured
laser clearance before crossing, and a sustained zero-velocity goal. The same
physical robot must recover after a bounded closed-door failure and explicit
reset. Fresh Storage and transport masters isolate the single-variable case.
The inherited caster/ODE contacts exhibit measured passive drift around .0064
m/s under zero Twist. A bounded duration-aware maximum-excursion envelope
checks waiting without confusing this drift with commanded forward travel.
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
from tests.lesson33_acceptance import angle, body_velocity, distance, rpc, yaw

# Fixed public lesson ABI. These constants deliberately do not import config.py.
ROBOT, DOOR = 'warehouse_bot', 'warehouse_door'
NS = '/warehouse'
CONTROL = '/lesson36'
WHEELS = ('warehouse_left_joint', 'warehouse_right_joint')
DOOR_JOINT = 'slide_joint'
SENSOR_X, SENSOR_Z = .15, .2202
DOOR_X, GOAL_X = 0., 2.2
TOPICS = {'scan': NS+'/scan', 'odom': NS+'/odom', 'wheels': NS+'/joint_states',
          'command': NS+'/cmd_vel', 'state': CONTROL+'/state', 'contact': NS+'/bumper'}
FRAMES = {'odom': 'warehouse/odom', 'base': 'warehouse/base_footprint', 'laser': 'warehouse/laser'}
SOURCE_URL = ('https://raw.githubusercontent.com/ros-simulation/gazebo_ros_pkgs/'
              '2.9.3/gazebo_plugins/src/gazebo_ros_diff_drive.cpp')


def require(value, message):
    if not value:
        raise AssertionError(message)


def finite_json(value):
    """Keep original ROS bytes while representing legal LaserScan infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return '+inf' if value > 0 else '-inf' if value < 0 else 'nan'
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [finite_json(v) for v in value]
    return value


def sim(row):
    return (row['world_time_before_ns']+row['world_time_after_ns'])/2e9


def robot(row):
    return row['models'][ROBOT]


def speed(row):
    v = robot(row)['twist']['linear']
    return math.hypot(v['x'], v['y'])


def xposition(row):
    return robot(row)['pose']['position']['x']


def stamp_ns(header):
    return header['stamp']['secs']*1000000000+header['stamp']['nsecs']


def front_range(event, half_angle=.12):
    value = event['data']
    ranges = [r for i, r in enumerate(value['ranges'])
              if abs(value['angle_min']+i*value['angle_increment']) <= half_angle
              and isinstance(r, (int, float)) and math.isfinite(r)
              and value['range_min'] <= r <= value['range_max']]
    return min(ranges) if ranges else math.inf


def proc(pid):
    folder = Path('/proc', str(pid))
    environment = dict(v.split('=', 1) for v in folder.joinpath('environ').read_bytes().decode().split('\0') if '=' in v)
    require(folder.stat().st_uid == os.getuid(), 'Observed process is not user-owned')
    require(environment.get('GAZEBOTEACH_SESSION_TOKEN') == os.environ['GAZEBOTEACH_SESSION_TOKEN'],
            'Observed process did not inherit the primary private-session token')
    return {'pid': pid, 'start_ticks': folder.joinpath('stat').read_text().rsplit(') ', 1)[1].split()[19],
            'exe': str(folder.joinpath('exe').resolve()),
            'argv': [v.decode(errors='replace') for v in folder.joinpath('cmdline').read_bytes().split(b'\0') if v],
            'token': environment['GAZEBOTEACH_SESSION_TOKEN'],
            'ros_master_uri': environment.get('ROS_MASTER_URI'),
            'gazebo_master_uri': environment.get('GAZEBO_MASTER_URI')}


class Probe(ServiceProbe):
    def __init__(self):
        import rospy
        from gazebo_msgs.msg import ContactsState
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState, LaserScan
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        self.worker_uuid = uuid.uuid4().hex
        rospy.init_node('independent_l36_'+self.worker_uuid, disable_signals=True)
        self.rospy, self.lock, self.events = rospy, threading.Lock(), []
        self.recording = True
        self.epoch, self.latest_stamp, self.cache = 0, 0, {}
        self.actions, self.truth, self.subscribers = [], [], []
        # Establish the lower native-time fence before accepting a sensor
        # callback. Later cohort fences supply a query after every receipt.
        self.initial_native_fence = self.snapshot()
        types = {'scan': LaserScan, 'odom': Odometry, 'wheels': JointState,
                 'command': Twist, 'state': String, 'contact': ContactsState}
        for key, kind in types.items():
            name = TOPICS[key]
            self.subscribers.append(rospy.Subscriber(name, kind,
                lambda msg, topic=name: self.receive(topic, msg), queue_size=2000))
        for name in ('/tf', '/tf_static'):
            self.subscribers.append(rospy.Subscriber(name, TFMessage,
                lambda msg, topic=name: self.receive(topic, msg), queue_size=2000))
        self.subscribers.append(rospy.Subscriber('/clock', Clock,
            lambda msg: self.receive('/clock', msg), queue_size=4000))

    def receive(self, topic, msg):
        wire = io.BytesIO()
        msg.serialize(wire)
        received, now = time.monotonic_ns(), self.rospy.Time.now().to_nsec()
        row = {'topic': topic, 'source_type': msg._type, 'receive_monotonic_ns': received,
               'ros_time_at_receive_ns': now,
               'source_stamp_ns': msg.header.stamp.to_nsec() if hasattr(msg, 'header') else None,
               'connection_header': dict(getattr(msg, '_connection_header', {})),
               'serialized_ros_base64': base64.b64encode(wire.getvalue()).decode('ascii'),
               'data': finite_json(ros_value(msg))}
        if hasattr(msg, 'clock'):
            row['clock_message_stamp_ns'] = msg.clock.to_nsec()
        with self.lock:
            if not self.recording:
                return
            # Cross-topic callback scheduling can reorder receipt wall clocks.
            # Detect actual /clock reversal, not old buffered sensor stamps.
            if now+100000000 < self.latest_stamp:
                self.epoch += 1
                self.cache.clear()
            self.latest_stamp = max(now, self.latest_stamp) if not self.epoch else now
            row['clock_epoch'], row['event_id'] = self.epoch, len(self.events)+1
            self.events.append(row)
            self.cache[topic] = row

    def freeze_observation(self):
        with self.lock:
            self.recording = False
            cutoff = time.monotonic_ns()
            events = list(self.events)
        # This query begins after every accepted callback. Callback threads
        # racing with the cutoff cannot append an unvalidated sensor event.
        closing = self.snapshot(extended=True)
        self.observation_cohort = {'cutoff_monotonic_ns': cutoff,
            'original_event_count': len(events), 'last_event_id': events[-1]['event_id'] if events else None,
            'recorded_scan_count': sum(e['topic'] == TOPICS['scan'] for e in events),
            'serialized_clock_count': sum(e['topic'] == '/clock' for e in events),
            'closing_native_query_host_bracket_ns': [closing['host_monotonic_before_ns'], closing['host_monotonic_after_ns']],
            'closing_world_time_bracket_ns': [closing['world_time_before_ns'], closing['world_time_after_ns']],
            'cohort_rule': 'Callbacks appended under observer lock before cutoff; later callbacks are outside this finite observation window.'}
        return events, closing

    def pause(self, value):
        from std_srvs.srv import Empty
        return self.call('/gazebo/'+('pause_physics' if value else 'unpause_physics'), Empty)

    def action(self, name, enabled=None):
        from std_srvs.srv import SetBool, Trigger
        before = time.monotonic_ns()
        path, kind = CONTROL+'/'+name, SetBool if enabled is not None else Trigger
        self.rospy.wait_for_service(path, timeout=4)
        done, result = threading.Event(), {}
        def invoke():
            client = self.rospy.ServiceProxy(path, kind)
            try:
                result['response'] = client(data=enabled) if enabled is not None else client()
            except Exception as error:
                result['error'] = error
            finally:
                client.close()
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        # Reset owns two finite native command/ACK transactions. Its bounded
        # wall deadline is independent of paused simulation time.
        deadline = time.monotonic()+18
        # Keep native world brackets dense while the independent service
        # thread waits on finite native ACKs. There is no Gazebo observer
        # thread and no race over a shared service proxy.
        self.snapshot()
        while not done.wait(.025):
            require(time.monotonic() < deadline, 'Independent controller action wall timeout: '+path)
            self.snapshot()
        self.snapshot()
        if 'error' in result:
            raise result['error']
        response = result['response']
        require(response.success, 'Actual controller action rejected '+path+': '+response.message)
        row = {'name': name, 'enabled': enabled, 'host_before_ns': before,
               'host_after_ns': time.monotonic_ns(), 'ros_time_ns': self.rospy.Time.now().to_nsec(),
               'response': ros_value(response)}
        self.actions.append(row)
        return row

    def snapshot(self, extended=False):
        from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelProperties, GetModelState
        from gazebo_msgs.srv import GetPhysicsProperties, GetWorldProperties
        started = time.monotonic_ns()
        before = self.call('/gazebo/get_world_properties', GetWorldProperties)
        models = {name: ros_value(self.call('/gazebo/get_model_state', GetModelState,
                  model_name=name, relative_entity_name='world')) for name in (ROBOT, DOOR)}
        joints = {name: ros_value(self.call('/gazebo/get_joint_properties', GetJointProperties,
                  joint_name=scoped)) for name, scoped in
                  [('left', ROBOT+'::'+WHEELS[0]), ('right', ROBOT+'::'+WHEELS[1]), ('door', DOOR+'::'+DOOR_JOINT)]}
        panel = ros_value(self.call('/gazebo/get_link_state', GetLinkState,
                         link_name=DOOR+'::panel', reference_frame='world'))
        after = self.call('/gazebo/get_world_properties', GetWorldProperties)
        row = {'host_monotonic_before_ns': started, 'host_monotonic_after_ns': time.monotonic_ns(),
               'world_time_before_ns': round(before.sim_time*1e9), 'world_time_after_ns': round(after.sim_time*1e9),
               'world_before': ros_value(before), 'world_after': ros_value(after), 'models': models, 'joints': joints,
               'door_panel': panel,
               'time_basis': 'Actual world-time service brackets; GetModelState header is API ROS time, not native acquisition time.'}
        if extended:
            row['properties'] = {name: ros_value(self.call('/gazebo/get_model_properties', GetModelProperties,
                model_name=name)) for name in (ROBOT, DOOR)}
            row['physics'] = ros_value(self.call('/gazebo/get_physics_properties', GetPhysicsProperties))
        with self.lock:
            row['clock_epoch'] = self.epoch
            row['latest_event_ids'] = {topic: event['event_id'] for topic, event in self.cache.items()}
        self.truth.append(row)
        return row

    def latest_state(self):
        with self.lock:
            event = self.cache.get(TOPICS['state'])
        if not event:
            return None
        try:
            value = json.loads(event['data']['data'])
            return value.get('phase') if isinstance(value, dict) else value
        except ValueError:
            return event['data']['data']

    def collect(self, seconds, label):
        rows, wall_deadline = [], time.monotonic()+max(15., seconds*8)
        start = self.snapshot()
        rows.append(start)
        self.last_collection = {'label': label, 'truth': rows}
        while sim(rows[-1])-sim(start) < seconds:
            require(time.monotonic() < wall_deadline, 'World clock stalled during '+label)
            time.sleep(.025)
            rows.append(self.snapshot())
        return rows

    def until(self, terminal, seconds, label, since_ns=0):
        rows, wall_deadline = [], time.monotonic()+max(30., seconds*8)
        start = self.snapshot()
        rows.append(start)
        self.last_collection = {'label': label, 'truth': rows}
        while True:
            current = self.latest_state()
            with self.lock:
                fresh = self.cache.get(TOPICS['state'], {}).get('receive_monotonic_ns', 0) >= since_ns
            if fresh and current in terminal:
                return {'terminal': current, 'truth': rows, 'terminal_snapshot': rows[-1]}
            require(time.monotonic() < wall_deadline and sim(rows[-1])-sim(start) < seconds,
                    label+' exceeded bounded wall/simulation time; last state='+str(current))
            time.sleep(.025)
            rows.append(self.snapshot())

    def ready(self):
        caller, master = self.rospy.get_name(), os.environ['ROS_MASTER_URI']
        deadline = time.monotonic()+10
        needed = [TOPICS[k] for k in ('scan', 'odom', 'wheels', 'command', 'state', 'contact')]
        while True:
            pub, sub, srv = [{key: nodes for key, nodes in value} for value in rpc(master, 'getSystemState', caller)]
            if all(topic in pub for topic in needed) and all(CONTROL+'/'+key in srv for key in ('start', 'reset', 'enable_door')):
                break
            require(time.monotonic() < deadline, 'Controller or native sensor/drive ABI is not ready')
            time.sleep(.05)
        require(rpc(master, 'getParam', caller, '/use_sim_time') is True, 'Actual Master must use simulation time')
        for key in ('scan', 'odom', 'wheels', 'contact'):
            require(pub[TOPICS[key]] == ['/gazebo'], 'Sensor/drive feedback is not native Gazebo: '+key)
        require(len(pub[TOPICS['command']]) == 1 and pub[TOPICS['command']][0] != caller,
                'Mission must have one autonomous command authority')
        command_authority = pub[TOPICS['command']][0]
        require({'/gazebo', caller} <= set(sub[TOPICS['command']]) <= {'/gazebo', caller, command_authority},
                'Unexpected command subscriber topology')
        require(srv.get('/gazebo/get_model_state') == ['/gazebo'], 'Ground truth service authority differs')
        nodes = {}
        for name in sorted({caller, '/gazebo'} | set(pub[TOPICS['command']]) | set(pub.get('/tf_static', []))):
            uri = rpc(master, 'lookupNode', caller, name)
            pid = rpc(uri, 'getPid', caller)
            nodes[name] = {'uri': uri, 'process': proc(pid), 'bus_info': rpc(uri, 'getBusInfo', caller)}
        native = nodes['/gazebo']['process']
        target = Path('/usr/bin/gzserver-11.15.1')
        require(target.is_file() and os.path.samefile(native['exe'], target),
                'Actual server is not the exact installed versioned Classic 11.15.1 ELF')
        require(target.read_bytes()[:4] == b'\x7fELF', 'Actual server must be ELF')
        maps = Path('/proc', str(native['pid']), 'maps').read_text()
        native['maps_text'] = maps
        native['exe_sha256'] = sha(target)
        libraries = sorted(set(line.split()[-1] for line in maps.splitlines()
                               if '.so' in line and (line.split()[-1].startswith('/'))))
        native['mapped_plugin_paths'] = [p for p in libraries if 'gazebo_ros_' in p or 'warehouse' in p]
        for filename in ('libgazebo_ros_api_plugin.so', 'libgazebo_ros_diff_drive.so', 'libgazebo_ros_laser.so', 'libgazebo_ros_bumper.so'):
            expected = Path('/opt/ros/noetic/lib')/filename
            require(expected.is_file() and any(os.path.samefile(p, expected) for p in libraries if Path(p).is_file()),
                    'Actual mapped plugin differs from installed exact native target '+filename)
        return {'system_state': [list(pub.items()), list(sub.items()), list(srv.items())],
                'topic_types': rpc(master, 'getTopicTypes', caller), 'nodes': nodes,
                'worker_uuid': self.worker_uuid, 'master_process': proc(rpc(master, 'getPid', caller)),
                'ros_master_uri': master, 'gazebo_master_uri': os.environ['GAZEBO_MASTER_URI']}


def state_data(event):
    return json.loads(event['data']['data'])


def window(events, start, end):
    return [e for e in events if start <= e['receive_monotonic_ns'] <= end]


def state_at(events, row):
    candidates = [e for e in events if e['topic'] == TOPICS['state']
                  and e['receive_monotonic_ns'] <= row['host_monotonic_before_ns']]
    return state_data(candidates[-1]).get('phase') if candidates else None


def check_stopped(rows, message, duration=.25):
    require(len(rows) >= 3 and sim(rows[-1])-sim(rows[0]) >= duration, message+' has no sustained measurement window')
    span = sim(rows[-1])-sim(rows[0])
    speeds = [speed(row) for row in rows]
    angular = [abs(robot(row)['twist']['angular']['z']) for row in rows]
    require(max(speeds) < .028 and max(angular) < .08,
            message+' has nonzero actual model velocity')
    # Formal v1 retained 92 native samples over 5.6555 s: maximum .006421
    # m/s, .029160 m drift away from the door, and actual zero Twist. This
    # matches the inherited L33 caster/ODE contact behavior. Keep short stop
    # windows strict, bound long passive drift, and check every excursion.
    cap = min(.06, max(.018, .006+.008*span))
    excursion = max(distance(robot(rows[0])['pose'], robot(row)['pose']) for row in rows)
    require(excursion < cap,
            message+' continues travelling despite zero command')
    return {'samples': len(rows), 'actual_sim_duration_s': span,
            'max_native_speed_mps': max(speeds), 'mean_native_speed_mps': sum(speeds)/len(speeds),
            'max_abs_native_yaw_rate_rps': max(angular), 'max_xy_excursion_m': excursion,
            'allowed_xy_excursion_m': cap, 'instantaneous_speed_limit_mps': .028,
            'instantaneous_yaw_rate_limit_rps': .08,
            'signed_x_drift_m': xposition(rows[-1])-xposition(rows[0]),
            'signed_y_drift_m': robot(rows[-1])['pose']['position']['y']-robot(rows[0])['pose']['position']['y']}


def receipt_world_interval(event, truth):
    """Bound world time at callback receipt with independent native queries.

The native sample in a wholly earlier query is a lower bound; the sample in
    a wholly later query is an upper bound. A callback inside a query is
    bracketed by its neighbouring complete queries rather than pretending a
    service result has an atomic acquisition stamp. ROS /clock delivery is a
    separate asynchronous stream and its apparent age remains diagnostic.
    """
    received, epoch = event['receive_monotonic_ns'], event['clock_epoch']
    same = [row for row in truth if row['clock_epoch'] == epoch]
    past = [row for row in same if row['host_monotonic_after_ns'] <= received]
    future = [row for row in same if row['host_monotonic_before_ns'] >= received]
    require(past and future, 'Every captured scan must have native queries physically before and after its host receipt')
    before = max(past, key=lambda row: row['host_monotonic_after_ns'])
    after = min(future, key=lambda row: row['host_monotonic_before_ns'])
    lower, upper = before['world_time_after_ns'], after['world_time_before_ns']
    source = event['source_stamp_ns']
    sim_width = (upper-lower)/1e9
    wall_width = (after['host_monotonic_after_ns']-before['host_monotonic_before_ns'])/1e9
    require(0 <= sim_width <= .4 and 0 <= wall_width <= .5,
            'Independent native scan-receipt interval is too uncertain')
    require(source <= upper+1000000 and upper-source <= 600000000,
            'Actual scan source header is future or stale against independent native receipt-time bounds')
    return {'event_id': event['event_id'], 'clock_epoch': epoch, 'source_stamp_ns': source,
            'receipt_monotonic_ns': received, 'lower_world_time_ns': lower, 'upper_world_time_ns': upper,
            'lower_query_host_bracket_ns': [before['host_monotonic_before_ns'], before['host_monotonic_after_ns']],
            'upper_query_host_bracket_ns': [after['host_monotonic_before_ns'], after['host_monotonic_after_ns']],
            'physical_age_interval_s': [(lower-source)/1e9, (upper-source)/1e9],
            'native_interval_sim_width_s': sim_width, 'native_interval_outer_wall_width_s': wall_width,
            'latest_delivered_ros_clock_age_diagnostic_s': (event['ros_time_at_receive_ns']-source)/1e9}


def check_scan(events, truth):
    scans = [e for e in events if e['topic'] == TOPICS['scan']]
    require(len(scans) >= 30, 'Too few original LaserScan messages')
    lags, matched, closed, intervals = [], 0, [], []
    by_epoch = {}
    for event in scans:
        by_epoch.setdefault(event['clock_epoch'], []).append(event['source_stamp_ns'])
        value, stamp = event['data'], event['source_stamp_ns']
        require(event['connection_header'].get('callerid') == '/gazebo' and value['header']['frame_id'] == FRAMES['laser'],
                'Laser data authority or frame is wrong')
        require(len(value['ranges']) >= 120 and value['angle_increment'] > 0 and
                abs(value['angle_max']-value['angle_min']-(len(value['ranges'])-1)*value['angle_increment']) < 1e-5,
                'Actual LaserScan beam geometry is inconsistent')
        lags.append((event['ros_time_at_receive_ns']-stamp)/1e9)
        intervals.append(receipt_world_interval(event, truth))
        same_epoch = [row for row in truth if row['clock_epoch'] == event['clock_epoch']]
        nearest = min(same_epoch, key=lambda row: abs(sim(row)-stamp/1e9))
        dt = abs(sim(nearest)-stamp/1e9)
        if dt <= .12 and nearest['joints']['door']['position'][0] < .03 and xposition(nearest) < -.45:
            actual = front_range(event)
            # Door near face is read from fixed source geometry independently.
            source = ET.parse('world.sdf').find("world/model[@name='"+DOOR+"']")
            panel = source.find("link[@name='panel']")
            require(panel is not None, 'Door panel source ABI differs')
            model_x = float(source.findtext('pose', '0 0 0 0 0 0').split()[0])
            panel_x = float(panel.findtext('pose', '0 0 0 0 0 0').split()[0])
            collision = panel.find('collision')
            collision_x = float(collision.findtext('pose', '0 0 0 0 0 0').split()[0])
            half_width = float(collision.findtext('geometry/box/size').split()[0])/2
            expected = model_x+panel_x+collision_x-half_width-xposition(nearest)-SENSOR_X
            require(math.isfinite(actual) and abs(actual-expected) < .09+speed(nearest)*dt,
                    'Closed-door echo differs from independently measured robot pose and source geometry')
            closed.append({'event_id': event['event_id'], 'actual_front_m': actual, 'expected_m': expected,
                           'world_time_bracket_ns': [nearest['world_time_before_ns'], nearest['world_time_after_ns']]})
        matched += 1
    for stamps in by_epoch.values():
        require(all(b >= a for a, b in zip(stamps, stamps[1:])), 'Laser source stamps regressed without clock rewind')
    require(len(closed) >= 3, 'No independent measured closed-door scan window')
    return {'original_scan_count': matched, 'closed_echo_examples': closed[:5],
            'latest_delivered_ros_clock_age_diagnostic_min_s': min(lags),
            'latest_delivered_ros_clock_age_diagnostic_max_s': max(lags),
            'serialized_clock_message_count': sum(e['topic'] == '/clock' for e in events),
            'native_receipt_intervals': intervals, 'native_bracketed_scan_count': len(intervals),
            'maximum_conservative_physical_age_upper_s': max(row['physical_age_interval_s'][1] for row in intervals),
            'maximum_native_interval_sim_width_s': max(row['native_interval_sim_width_s'] for row in intervals),
            'maximum_native_interval_outer_wall_width_s': max(row['native_interval_outer_wall_width_s'] for row in intervals),
            'physical_age_upper_limit_s': .6, 'native_interval_sim_width_limit_s': .4,
            'native_interval_outer_wall_width_limit_s': .5,
            'time_basis': 'Conservative receipt-time interval from native world queries physically before/after each callback; /clock queue age is diagnostic, not atomic acquisition time.'}


def check_feedback(events, truth):
    odom = [e for e in events if e['topic'] == TOPICS['odom']]
    wheels = [e for e in events if e['topic'] == TOPICS['wheels']]
    require(len(odom) >= 100 and len(wheels) >= 100, 'No sustained native wheel and odometry streams')
    samples, parents, authorities = {}, {}, {}
    for event in events:
        if event['topic'] not in ('/tf', '/tf_static'):
            continue
        for transform in event['data']['transforms']:
            child = transform['child_frame_id']
            parents.setdefault(child, set()).add(transform['header']['frame_id'])
            authorities.setdefault(child, set()).add(event['connection_header'].get('callerid'))
            samples.setdefault(child, {})[stamp_ns(transform['header'])] = (event, transform)
    expected = {FRAMES['odom']: 'world', FRAMES['base']: FRAMES['odom'], FRAMES['laser']: FRAMES['base']}
    require(set(parents) == set(expected), 'Actual TF graph has missing, unscoped or duplicate children: '+repr(parents))
    for child, parent in expected.items():
        require(parents[child] == {parent} and len(authorities[child]) == 1, 'Actual TF child has conflicting parent/authority')
    require(authorities[FRAMES['base']] == {'/gazebo'}, 'Dynamic odometry TF is not native Gazebo')
    for child, position in ((FRAMES['odom'], (0., 0., 0.)), (FRAMES['laser'], (SENSOR_X, 0., SENSOR_Z))):
        for event, transform in samples[child].values():
            value = transform['transform']
            require(event['topic'] == '/tf_static' and
                    max(abs(value['translation'][k]-v) for k, v in zip(('x', 'y', 'z'), position)) < 1e-7 and
                    abs(value['rotation']['w']-1) < 1e-7 and
                    max(abs(value['rotation'][k]) for k in ('x', 'y', 'z')) < 1e-7,
                    'Actual static TF differs from world-source odometry identity or physical laser extrinsics')
    maximum, matched, lag = 0., 0, []
    for event in odom[::max(1, len(odom)//250)]:
        value, stamp = event['data'], event['source_stamp_ns']
        require(event['connection_header'].get('callerid') == '/gazebo' and
                value['header']['frame_id'] == FRAMES['odom'] and value['child_frame_id'] == FRAMES['base'],
                'Actual odometry authority/frames differ')
        require(stamp in samples[FRAMES['base']], 'Native odometry lacks a same-stamp TF')
        transform = samples[FRAMES['base']][stamp][1]['transform']
        require(max(abs(transform['translation'][k]-value['pose']['pose']['position'][k]) for k in ('x', 'y', 'z')) < 1e-9,
                'Same-stamp odometry and TF position disagree')
        require(max(abs(transform['rotation'][k]-value['pose']['pose']['orientation'][k]) for k in ('x', 'y', 'z', 'w')) < 1e-9,
                'Same-stamp odometry and TF quaternion disagree')
        lag.append((event['ros_time_at_receive_ns']-stamp)/1e9)
        same_epoch = [row for row in truth if row['clock_epoch'] == event['clock_epoch']]
        nearest = min(same_epoch, key=lambda row: abs(sim(row)-stamp/1e9))
        dt = abs(sim(nearest)-stamp/1e9)
        if dt <= .12:
            error = distance(robot(nearest)['pose'], value['pose']['pose'])
            require(error < .025+speed(nearest)*dt and
                    abs(angle(yaw(robot(nearest)['pose'])-yaw(value['pose']['pose']))) < .075,
                    'World-source odometry disagrees with independently acquired native model pose')
            maximum, matched = max(maximum, error), matched+1
    require(matched >= 40, 'Too few independent Gazebo-ground-truth odometry matches')
    for event in wheels:
        value = event['data']
        require(set(value['name']) == set(WHEELS) and len(value['position']) == 2 and
                not value['velocity'] and not value['effort'], 'Native 2.9.3 wheel JointState ABI differs')
    contacts = [e for e in events if e['topic'] == TOPICS['contact']]
    require(len(contacts) >= 20, 'Bumper absence was confused with a measured no-contact result')
    require(all(not e['data']['states'] for e in contacts), 'Actual robot bumper reported contact with warehouse geometry')
    return {'odom_count': len(odom), 'wheel_count': len(wheels), 'matched_ground_truth_count': matched,
            'max_ground_truth_pose_error_m': maximum, 'measured_odom_receipt_lag_max_s': max(lag),
            'contact_messages_empty': len(contacts), 'tf_child_to_parent': {k: next(iter(v)) for k, v in parents.items()}}


def check_mission(result, events):
    rows = result['truth']
    start, end = rows[0], rows[-1]
    require(result['terminal'] == 'SUCCEEDED', 'Controller did not finish successful mission')
    require(len(rows) >= 50 and xposition(end) > GOAL_X-.12 and xposition(end) < GOAL_X+.12,
            'Actual physical robot did not arrive at the goal')
    phases = [state_data(e)['phase'] for e in events if e['topic'] == TOPICS['state']]
    ordered = []
    for phase in phases:
        if not ordered or ordered[-1] != phase:
            ordered.append(phase)
    required = ['SCAN', 'APPROACH', 'WAIT_DOOR', 'PASS', 'GOAL', 'STOPPING', 'SUCCEEDED']
    cursor = 0
    for phase in ordered:
        if cursor < len(required) and phase == required[cursor]:
            cursor += 1
    require(cursor == len(required) and 'FAILED' not in ordered, 'Finite mission phases are incomplete/out of order: '+repr(ordered))
    waiting = [row for row in rows if state_at(events, row) == 'WAIT_DOOR']
    require(waiting and max(xposition(row) for row in waiting) < -.35, 'Robot did not wait before the door')
    stationary_wait = [row for row in waiting if sim(row)-sim(waiting[0]) > .15]
    wait_stop = check_stopped(stationary_wait, 'Blocked-door wait', duration=.18)
    crossing = next((row for row in rows if xposition(row) >= DOOR_X-.3), None)
    require(crossing is not None, 'Actual robot never entered the door plane')
    scans = [e for e in events if e['topic'] == TOPICS['scan'] and
             e['source_stamp_ns']/1e9 <= sim(crossing) and
             e['receive_monotonic_ns'] <= crossing['host_monotonic_before_ns']]
    require(scans, 'Crossing lacks a prior actual LaserScan')
    clear = [e for e in scans if front_range(e) > 1.2 and
             any(abs(sim(row)-e['source_stamp_ns']/1e9) < .12 and row['joints']['door']['position'][0] > 1.05 for row in rows)]
    require(clear and sim(crossing)-clear[-1]['source_stamp_ns']/1e9 < .25,
            'Robot crossed without a fresh independently measured clear laser and physically open door')
    require(max(row['joints']['door']['position'][0] for row in rows) > 1.10 and
            max(abs(row['joints']['door']['rate'][0]) for row in rows) > .1,
            'Door opening lacks measured physical displacement/rate')
    deltas = {key: end['joints'][key]['position'][0]-start['joints'][key]['position'][0] for key in ('left', 'right')}
    require(min(deltas.values()) > 8 and max(speed(row) for row in rows) > .15,
            'Goal displacement lacks real wheel joint rotation and physical velocity')
    require(max(abs(robot(row)['pose']['position']['y']) for row in rows) < .13,
            'Robot drifted outside the intended warehouse corridor')
    terminal = result['stop_truth']
    goal_stop = check_stopped(terminal, 'Goal stop', duration=.5)
    require(terminal[-1]['joints']['door']['position'][0] < .10, 'Actual door did not physically close after passage')
    commands = [e for e in events if e['topic'] == TOPICS['command'] and
                e['receive_monotonic_ns'] >= terminal[0]['host_monotonic_before_ns']]
    require(len(commands) >= 5 and all(abs(e['data']['linear']['x']) < 1e-10 and
            abs(e['data']['angular']['z']) < 1e-10 for e in commands), 'Goal stop lacks sustained actual zero Twist messages')
    steady = [speed(row) for row in rows if .9 < xposition(row) < 1.7]
    require(len(steady) >= 5, 'Too few common-corridor speed measurements')
    return {'phase_order': ordered, 'goal_x_m': xposition(end), 'wheel_delta_rad': deltas,
            'maximum_door_travel_m': max(row['joints']['door']['position'][0] for row in rows),
            'fresh_clear_scan_event_before_crossing': clear[-1]['event_id'],
            'goal_native_twist': robot(terminal[-1])['twist'], 'steady_native_speed_mean_mps': sum(steady)/len(steady),
            'independent_sim_duration_s': sim(end)-sim(start), 'wait_stop_measurements': wait_stop,
            'goal_stop_measurements': goal_stop, 'goal_zero_twist_messages': len(commands)}


def check_failure(result, events):
    rows = result['truth']
    require(result['terminal'] == 'FAILED', 'Held-closed door did not produce explicit bounded failure')
    require(max(xposition(row) for row in rows) < -.35 and
            max(abs(row['joints']['door']['position'][0]) for row in rows) < .08,
            'Held-door failure crossed the obstruction or opened the disabled door')
    waiting = [row for row in rows if state_at(events, row) == 'WAIT_DOOR']
    require(waiting and 5. <= sim(waiting[-1])-sim(waiting[0]) < 7.5,
            'Closed-door wait failed outside the documented finite timeout')
    settled_wait = [row for row in waiting if sim(row)-sim(waiting[0]) > .25]
    wait_stop = check_stopped(settled_wait, 'Fault wait', duration=4.)
    towards = max(xposition(row)-xposition(waiting[0]) for row in waiting)
    require(towards <= .01, 'Zero-command fault wait travelled towards the blocked door')
    wait_commands = [e for e in events if e['topic'] == TOPICS['command'] and
                     settled_wait[0]['host_monotonic_before_ns'] <= e['receive_monotonic_ns'] <=
                     settled_wait[-1]['host_monotonic_after_ns']]
    require(len(wait_commands) >= 20 and all(abs(e['data']['linear']['x']) < 1e-10 and
            abs(e['data']['angular']['z']) < 1e-10 for e in wait_commands), 'Fault wait lacks sustained actual zero Twist')
    terminal_stop = check_stopped(result['stop_truth'], 'Fault terminal stop', duration=.4)
    terminal_events = window(events, result['stop_truth'][0]['host_monotonic_before_ns'],
                            result['stop_truth'][-1]['host_monotonic_after_ns'])
    commands = [e for e in terminal_events if e['topic'] == TOPICS['command']]
    require(len(commands) >= 5 and all(abs(e['data']['linear']['x']) < 1e-10 and abs(e['data']['angular']['z']) < 1e-10
            for e in commands), 'Bounded failure did not retain zero commands')
    return {'terminal': result['terminal'], 'maximum_x_m': max(xposition(row) for row in rows),
            'wait_duration_s': sim(waiting[-1])-sim(waiting[0]),
            'native_terminal_twist': robot(result['stop_truth'][-1])['twist'],
            'wait_stop_measurements': wait_stop, 'maximum_towards_door_excursion_m': towards,
            'allowed_towards_door_excursion_m': .01, 'wait_zero_twist_messages': len(wait_commands),
            'terminal_stop_measurements': terminal_stop, 'terminal_zero_twist_messages': len(commands)}


def check_map(events):
    state_events = [e for e in events if e['topic'] == TOPICS['state']]
    states = [state_data(e) for e in state_events]
    require(states and states[-1].get('run_id'), 'Successful mission has no unique run identity')
    folder = Path('build/runs')/states[-1]['run_id']
    required_files = ('points.csv', 'map.svg', 'result.json', 'events.jsonl', 'raw_ros.jsonl')
    require(all((folder/name).is_file() for name in required_files), 'Successful mission did not preserve measured map and raw logs')
    with (folder/'points.csv').open(newline='') as handle:
        points = list(csv.DictReader(handle))
    required = {'scan_stamp_ns', 'beam_index', 'range_m', 'angle_rad', 'odom_stamp_ns',
                'robot_x_m', 'robot_y_m', 'robot_yaw_rad', 'x_m', 'y_m'}
    require(len(points) >= 300 and required <= set(points[0]), 'Measured-point CSV lacks reproducible sensor evidence columns')
    epoch = state_events[-1]['clock_epoch']
    scans = {e['source_stamp_ns']: e for e in events if e['topic'] == TOPICS['scan'] and e['clock_epoch'] == epoch}
    odometry = {e['source_stamp_ns']: e for e in events if e['topic'] == TOPICS['odom'] and e['clock_epoch'] == epoch}
    maximum_error, scan_stamps, matched = 0., set(), 0
    projected = []
    for point in points:
        scan_stamp, odom_stamp, index = int(point['scan_stamp_ns']), int(point['odom_stamp_ns']), int(point['beam_index'])
        require(scan_stamp in scans and odom_stamp in odometry, 'Map references a scan/odometry stamp absent from independent original ROS stream')
        require(0 <= scan_stamp-odom_stamp <= 120000000, 'Mapping used future/stale odometry')
        scan, odom = scans[scan_stamp]['data'], odometry[odom_stamp]['data']['pose']['pose']
        require(0 <= index < len(scan['ranges']), 'Map beam index is invalid')
        actual_range = scan['ranges'][index]
        require(isinstance(actual_range, (int, float)) and math.isfinite(actual_range), 'Map invented a finite point from a missing echo')
        heading, beam = yaw(odom), scan['angle_min']+index*scan['angle_increment']
        rx, ry = odom['position']['x'], odom['position']['y']
        expected = (rx+SENSOR_X*math.cos(heading)+actual_range*math.cos(heading+beam),
                    ry+SENSOR_X*math.sin(heading)+actual_range*math.sin(heading+beam))
        actual = (float(point['x_m']), float(point['y_m']))
        error = math.hypot(actual[0]-expected[0], actual[1]-expected[1])
        require(error < 2e-5 and abs(float(point['range_m'])-actual_range) < 1e-6 and
                abs(float(point['angle_rad'])-beam) < 1e-6 and
                abs(float(point['robot_x_m'])-rx) < 1e-6 and abs(float(point['robot_y_m'])-ry) < 1e-6 and
                abs(angle(float(point['robot_yaw_rad'])-heading)) < 1e-6,
                'CSV point differs from independently decoded measured scan, preceding odometry and TF extrinsics')
        maximum_error, matched = max(maximum_error, error), matched+1
        scan_stamps.add(scan_stamp)
        projected.append(actual)
    require(len(scan_stamps) >= 20, 'Map is not accumulated over a measured moving mission')
    svg = ET.parse(str(folder/'map.svg')).getroot()
    marks = [node for node in svg.iter() if node.tag.rsplit('}', 1)[-1] == 'circle']
    require(len(marks) == len(points), 'SVG point count differs from the measured CSV')
    for node, point in zip(marks, projected):
        # The generated artifact declares the simple world-coordinate viewBox.
        require(abs(float(node.get('cx'))-point[0]) < 2e-5 and
                abs(float(node.get('cy'))+point[1]) < 2e-5,
                'SVG mark differs from measured CSV point (world X, negative world Y)')
    return {'run_id': states[-1]['run_id'], 'directory': str(folder.resolve()),
            'original_scan_count': len(scan_stamps), 'recomputed_point_count': matched,
            'maximum_projection_error_m': maximum_error,
            'files_sha256': {name: sha(folder/name) for name in required_files},
            'claim': 'Odometry-projected measured laser endpoints; no SLAM/map optimization claim.'}


def source_proof(graph):
    source = Path('door_plugin.cc').read_text()
    forbidden = ('SetWorldPose', 'SetRelativePose', 'SetPose', 'SetPosition', 'SetVelocity', 'SetLinearVel', 'SetAngularVel')
    require('SetForce(0,force)' in source and not any(name in source for name in forbidden),
            'Door source uses pose/velocity assignment instead of the required force-driven joint')
    for name in ('mission.py', 'lab.py', 'verify.py'):
        contents = Path(name).read_text()
        require(not any(value in contents for value in ('set_model_state', 'set_link_state', 'SetModelState', 'SetLinkState')),
                'Mission source contains model/link teleport services')
    native = graph['nodes']['/gazebo']['process']
    record = json.loads(Path('build/warehouse.json').read_text())
    require(all(sha(name) == digest for name, digest in record['source_sha256'].items()),
            'Source files differ from frozen native preparation inputs')
    loaded_world = Path('world.sdf').resolve()
    require(sha(loaded_world) == record['world_sha256'] and
            any(Path(value).resolve() == loaded_world for value in native['argv'] if value.endswith('.sdf')),
            'Actual server world differs from the exact prepared physical input')
    world = ET.parse('world.sdf').find('world')
    door = world.find("model[@name='"+DOOR+"']")
    plugin = door.find('plugin')
    require(plugin is not None, 'Physical door plugin is missing')
    filename = plugin.get('filename')
    expected = Path('build')/filename
    if not expected.is_file():
        candidates = [Path(entry)/filename for entry in os.environ.get('GAZEBO_PLUGIN_PATH', '').split(':') if entry]
        candidates = [candidate for candidate in candidates if candidate.is_file()]
        require(len(candidates) == 1, 'Cannot establish exact cached native door target')
        expected = candidates[0]
    mapped = [Path(line.split()[-1]) for line in native['maps_text'].splitlines()
              if line.split() and line.split()[-1].startswith('/') and Path(line.split()[-1]).is_file()]
    require(expected.read_bytes()[:4] == b'\x7fELF' and any(os.path.samefile(expected, actual) for actual in mapped),
            'Actual server did not map the exact prepared/cached native door target')
    require(expected.resolve().parent == Path(record['cache']).resolve() and
            sha(expected) == record['products'][expected.name],
            'Loaded native door target differs from frozen content-keyed cached product')
    joint = door.find("joint[@name='"+DOOR_JOINT+"']")
    require(joint is not None and joint.get('type') == 'prismatic' and joint.findtext('axis/xyz').split() == ['0', '1', '0'],
            'Native door is not the expected physical Y-axis prismatic joint')
    robot_source = world.find("model[@name='"+ROBOT+"']")
    drive = robot_source.find("plugin[@filename='libgazebo_ros_diff_drive.so']")
    require(drive is not None and drive.findtext('odometrySource') == 'world', 'Real stock differential-wheel drive is missing')
    return {'plugin_target': str(expected.resolve()), 'plugin_sha256': sha(expected),
            'door_source_sha256': sha('door_plugin.cc'), 'motion_primitive': 'Joint::SetForce',
            'physical_joint_type': joint.get('type'), 'official_world_odometry_source': SOURCE_URL}


def check_panel_motion(rows):
    reference = min(rows, key=lambda row: abs(row['joints']['door']['position'][0]))
    q0 = reference['joints']['door']['position'][0]
    y0 = reference['door_panel']['link_state']['pose']['position']['y']
    maximum_error, moved = 0., 0.
    for row in rows:
        q = row['joints']['door']['position'][0]
        actual = row['door_panel']['link_state']['pose']['position']['y']-y0
        dt = (row['world_time_after_ns']-row['world_time_before_ns'])/1e9
        error = abs(actual-(q-q0))
        require(error < .025+dt*abs(row['joints']['door']['rate'][0]), 'Actual panel Y motion differs from physical prismatic joint travel')
        maximum_error, moved = max(maximum_error, error), max(moved, abs(actual))
    require(moved > 1.05, 'Door joint feedback lacks independent physical panel translation')
    return {'maximum_panel_translation_m': moved, 'maximum_joint_link_error_m': maximum_error}


def exercise(probe, output, held_fault):
    parameters = json.loads(Path('parameters.json').read_text())
    result = {'parameters': parameters, 'held_closed_fault_requested': held_fault}
    probe.progress = result
    result['initial_graph'] = probe.ready()
    result['native_source_proof'] = source_proof(result['initial_graph'])
    acknowledged = probe.action('enable_door', True)
    cap_ack = json.loads(acknowledged['response']['message'])
    require(cap_ack.get('force_cap') == parameters['door_force_n'] and cap_ack.get('enabled') is True,
            'Actual acknowledged native door force cap differs from frozen parameters')
    result['native_force_cap_acknowledgement'] = cap_ack
    probe.pause(False)
    result['warmup_truth'] = probe.collect(.7, 'native_sensor_warmup')
    result['initial'] = probe.snapshot(extended=True)
    require(abs(xposition(result['initial'])+1.6) < .06 and speed(result['initial']) < .03,
            'Actual robot did not begin at the documented physical spawn, stationary')
    for name, value in result['initial']['properties'].items():
        require(not value['is_static'], 'Physical robot/door unexpectedly declared static: '+name)
    if held_fault:
        probe.action('reset')
        probe.action('enable_door', False)
        probe.pause(False)
        result['fault_closed_truth'] = probe.collect(.5, 'disabled_closed_door')
        action = probe.action('start')
        fault = probe.until({'SUCCEEDED', 'FAILED'}, 18., 'held_closed_fault', action['host_before_ns'])
        fault['terminal_boundary_snapshot'] = probe.snapshot(extended=True)
        probe.pause(False)
        fault['stop_truth'] = probe.collect(.7, 'failed_zero_tail')
        probe.pause(True)
        fault['closing_native_fence'] = probe.snapshot(extended=True)
        with probe.lock:
            fault_events = window(list(probe.events), action['host_before_ns'], fault['stop_truth'][-1]['host_monotonic_after_ns'])
        fault['independent_checks'] = check_failure(fault, fault_events)
        result['held_closed_fault'] = fault
        # Preserve original failure and native measurements before recovery can
        # clear the controller cache or replace a current-run convenience file.
        save(output.with_name(output.stem+'-held-failure.json'), finite_json({
            'fault': fault, 'original_ros_events': fault_events, 'actions': list(probe.actions)}))
        states = [state_data(e) for e in fault_events if e['topic'] == TOPICS['state']]
        if states and states[-1].get('run_id'):
            folder = Path('build/runs')/states[-1]['run_id']
            if folder.is_dir():
                shutil.copytree(str(folder), str(output.with_name(output.stem+'-held-course-files')), dirs_exist_ok=True)
        before = probe.snapshot()
        probe.action('enable_door', True)
        probe.action('reset')
        after = probe.snapshot()
        result['explicit_recovery'] = {'before_reset': before, 'after_reset': after,
            'actual_reset_displacement_m': distance(robot(before)['pose'], robot(after)['pose']),
            'clock_rewound': sim(after) < sim(before)}
        require(probe.latest_state() not in ('PASS', 'GOAL', 'SUCCEEDED'), 'Explicit reset retained terminal mission phase')
    else:
        probe.action('reset')
    probe.pause(False)
    result['pre_success_truth'] = probe.collect(.5, 'closed_sensor_window_before_start')
    action = probe.action('start')
    mission = probe.until({'SUCCEEDED', 'FAILED'}, 42., 'autonomous_success', action['host_before_ns'])
    mission['terminal_boundary_snapshot'] = probe.snapshot(extended=True)
    probe.pause(False)
    mission['stop_truth'] = probe.collect(.9, 'successful_goal_zero_tail')
    probe.pause(True)
    all_events, closing_fence = probe.freeze_observation()
    mission_events = window(all_events, action['host_before_ns'], mission['stop_truth'][-1]['host_monotonic_after_ns'])
    mission['independent_checks'] = check_mission(mission, mission_events)
    result['successful_mission'] = mission
    result['map_checks'] = check_map(all_events)
    result['final'] = closing_fence
    result['final_graph'] = probe.ready()
    for row in probe.truth:
        require(row['world_time_before_ns'] <= row['world_time_after_ns'] and
                row['host_monotonic_before_ns'] <= row['host_monotonic_after_ns'], 'World/receipt-time brackets reversed')
        require(all(math.isfinite(v) for key in ('position', 'orientation') for v in robot(row)['pose'][key].values()),
                'Actual physical model pose contains nonfinite values')
    result['physical_panel_checks'] = check_panel_motion(probe.truth)
    result['scan_checks'] = check_scan(all_events, probe.truth)
    result['observation_cohort'] = dict(probe.observation_cohort)
    result['observation_cohort']['validated_scan_count'] = result['scan_checks']['native_bracketed_scan_count']
    require(result['observation_cohort']['recorded_scan_count'] == result['observation_cohort']['validated_scan_count'],
            'Persisted original scan cohort differs from the independently validated cohort')
    result['feedback_checks'] = check_feedback(all_events, probe.truth)
    require(result['initial']['properties'] == result['final']['properties'], 'Physical model identity/topology changed during mission')
    require(result['final']['physics']['pause'], 'Independent observer did not leave the owned world paused')
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
        result['error'] = type(error).__name__+': '+str(error)
        result['traceback'] = traceback.format_exc()
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
            result['native_truth'] = list(probe.truth)
            result['control_actions'] = list(probe.actions)
            result['worker_uuid'] = probe.worker_uuid
            result['observation_cohort'] = getattr(probe, 'observation_cohort', None)
        # The failed physical state, original messages and all actions survive
        # before an emergency stop/pause or owned-process cleanup occurs.
        save(output, finite_json(result))
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
    child = session._spawn('Independent L36 ROS/physics observer', ['bash', '--noprofile', '--norc', '-c',
                           shlex.join(argv)+' > '+shlex.quote(str(log))+' 2>&1'])
    try:
        status = child.wait(timeout=420)
    except BaseException:
        terminate_owned(child)
        raise
    require(output.is_file(), 'Independent worker produced no report; inspect '+str(log))
    raw = json.loads(output.read_text())
    require(status == 0 and raw['passed'], 'Independent warehouse acceptance failed: '+str(raw.get('error'))+'; '+str(output))
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
    found = [lesson for lesson in lessons if lesson.key == 'classic11-noetic:36']
    require(len(found) == 1, 'Expected one published L36 manifest: '+repr(errors))
    lesson, started, sessions = found[0], time.monotonic(), []
    original_source = hashes(lesson.folder)
    report = {'passed': False, 'lesson': lesson.key, 'source_sha256': original_source,
              'scenarios': [], 'test_algorithm': __doc__, 'graphical_checks_performed': False,
              'test_writes_model_pose_or_velocity': False, 'test_publishes_cmd_vel': False,
              'official_world_odometry_source': SOURCE_URL}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l36-private-') as directory:
        try:
            baseline = json.loads((lesson.folder/'parameters.json').read_text())
            require(baseline == {'linear_speed_mps': .25, 'door_force_n': 20}, 'Default parameter ABI differs')
            for name, parameters, held_fault in (
                    ('baseline_fault_and_explicit_recovery', baseline, True),
                    ('linear_speed_only', {**baseline, 'linear_speed_mps': .30}, False)):
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
                        len({s.env['GAZEBO_MASTER_URI'] for _, s in sessions}) == len(sessions),
                        'Physical cases reused a session token or transport master')
                session.start(gui=False, terminals=False)
                immutable = hashes(workspace)
                raw, path = run_worker(session, artifacts, name, held_fault)
                result = raw['exercise']
                require(hashes(workspace) == immutable, 'Physical experiment modified immutable practice inputs')
                case = {'name': name, 'passed': True, 'parameters': parameters, 'raw_report': str(path),
                        'session_token': session.token, 'ros_master_uri': session.env['ROS_MASTER_URI'],
                        'gazebo_master_uri': session.env['GAZEBO_MASTER_URI'], 'worker_uuid': raw['worker_uuid'],
                        'input_sha256': immutable, 'physical_mission_checks': result['successful_mission']['independent_checks'],
                        'feedback_checks': result['feedback_checks'], 'scan_checks': result['scan_checks'],
                        'observation_cohort': result['observation_cohort'],
                        'map_checks': result['map_checks'], 'physical_panel_checks': result['physical_panel_checks'],
                        'held_failure_checks': result.get('held_closed_fault', {}).get('independent_checks')}
                report['scenarios'].append(case)
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
                session.stop(reason='L36 physical scenario complete')
            base, faster = [case['physical_mission_checks']['steady_native_speed_mean_mps'] for case in report['scenarios']]
            require(.21 < base < .29 and .26 < faster < .34 and faster > base*1.12 and faster-base > .025,
                    'Single-variable speed change did not measurably change actual native physical velocity')
            report['single_variable_physical_effects'] = {'baseline_native_speed_mps': base,
                'speed_only_native_speed_mps': faster, 'unchanged_parameter': 'door_force_n=20'}
            report['passed'] = True
        except Exception as error:
            report['error'] = type(error).__name__+': '+str(error)
        finally:
            report['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    # Persist every partial/native/controller artifact first.
                    if session.log_path.exists():
                        shutil.copy2(str(session.log_path), str(artifacts/(name+'-before-cleanup-runtime.log')))
                    for relative in ('parameters.json', 'world.sdf', 'build/warehouse.json'):
                        source = session.workspace/relative
                        if source.is_file():
                            shutil.copy2(str(source), str(artifacts/(name+'-'+Path(relative).name)))
                    runs = session.workspace/'build/runs'
                    if runs.is_dir():
                        shutil.copytree(str(runs), str(artifacts/(name+'-course-runs')), dirs_exist_ok=True)
                    session.stop(reason='L36 final independent owned cleanup')
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
                        session.stop(reason='L36 fallback owned cleanup after artifact-copy failure')
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
    print(json.dumps({key: value for key, value in report.items() if key not in ('source_sha256', 'scenarios')},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
