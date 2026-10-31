#!/usr/bin/env python3
"""Independent native-pose physics and lifecycle checks for the L26 ModelPlugin.

The SDK client is used only as a transport wire reader. This test decodes its
unmodified native protobuf text itself, unwraps rotor quaternions and compares
source-time angular displacement with I*domega/dt = drive - drag*omega. Plugin
telemetry is explicitly instrumentation, not the independent physical truth.
No position or velocity setters are used. Gazebo's model-only reset preserves
the native internal rotor angle while clearing velocity; angular displacement
comparisons therefore retain that measured offset. Every native event remains
in raw JSONL.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model

INERTIA = .015
WORLD, MODEL, ROTOR = 'lesson26', 'rotor_lab', 'rotor'


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def hashes(folder):
    return {p.relative_to(folder).as_posix(): sha(p) for p in sorted(Path(folder).rglob('*'))
            if p.is_file() and not {'__pycache__', 'build', '.git'} & set(p.relative_to(folder).parts)}


def events(path, live=False):
    result, other, incomplete = [], [], None
    raw = Path(path).read_text(encoding='utf-8') if Path(path).exists() else ''
    lines = raw.splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            if index == len(lines)-1 and not raw.endswith('\n') and live:
                incomplete = line
            else:
                other.append(line)
            continue
        require(isinstance(row, dict), 'Wire JSON record is not an object')
        require(row.get('source') not in ('client_error', 'decode_error'), 'Wire reader error: '+repr(row))
        result.append(row)
    return result, {'non_json_lines': other, 'trailing_partial_line': incomplete}


def run(session, argv, output, timeout=12):
    command = shlex.join([str(x) for x in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L26 command', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    status = run(session, [session.workspace/'build/model_observer', mode, *arguments], path)
    require(status == 0, 'Actual wire command failed: '+str(path))
    rows, diagnostics = events(path)
    expected = {'stats': 'world_stats', 'snapshot': 'snapshot', 'entity': 'entity', 'control': 'control_sent'}[mode]
    matches = [row for row in rows if row.get('source') == expected]
    require(len(matches) == 1, 'Expected one '+expected+' native wire result: '+str(path))
    result = matches[0]
    result['_wire_artifact'] = str(path)
    result['_wire_sha256'] = sha(path)
    result['_wire_diagnostics'] = diagnostics
    return result


def stamp(message, field):
    value = first(message, field)
    require(isinstance(value, dict), 'Native source timestamp missing: '+field)
    return int(first(value, 'sec', '0'))*1000000000+int(first(value, 'nsec', '0'))


def statistics(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native statistics omit pause state')
    return {'sim_time_ns': stamp(native, 'sim_time'), 'iterations': int(first(native, 'iterations')),
            'paused': first(native, 'paused') == 'true', 'raw_wire': row}


def plugin(row):
    value = row['payload']
    require(value['model'] == MODEL and value['joint'].split('::')[-1] == 'fan_joint',
            'Plugin instrumentation identifies the wrong model/joint')
    if row.get('source') == 'snapshot':
        require(row['request_id'] == row['response_id'] and value.get('query_nonce'),
                'Plugin RPC correlation is missing')
    return value


def quaternion_yaw(pose):
    quaternion = first(pose, 'orientation', {})
    values = [float(first(quaternion, name, '0' if name != 'w' else '1')) for name in ('x', 'y', 'z', 'w')]
    require(all(math.isfinite(x) for x in values), 'Native quaternion is not finite')
    x, y, z, w = values
    require(abs(sum(v*v for v in values)-1) < 1e-6, 'Native quaternion is not normalized')
    require(abs(x)+abs(y) < 1e-5, 'Actual rotor motion is not about the intended vertical axis')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def entity_rotor(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'name') == MODEL, 'Native entity response is not the actual rotor model')
    links = [link for link in native.get('link', []) if str(first(link, 'name')).split('::')[-1] == ROTOR]
    require(len(links) == 1, 'Native model response lacks one unique rotor link')
    rotor = links[0]
    rotor_id = int(first(rotor, 'id'))
    require(rotor_id > 0, 'Native entity response lacks the rotor entity ID')
    inertial = first(rotor, 'inertial')
    require(isinstance(inertial, dict), 'Native rotor inertia missing')
    izz = float(first(inertial, 'izz'))
    require(abs(izz-INERTIA) < 1e-10 and abs(float(first(inertial, 'mass'))-.48) < 1e-10,
            'Actual native rotor mass/inertia differ from the physical comparison')
    return {'yaw_rad': quaternion_yaw(first(rotor, 'pose')), 'inertia_kg_m2': izz, 'rotor_entity_id': rotor_id,
            'native_rotor_link': rotor, 'native_model': native, 'raw_wire': row}


def pose_rows(records, rotor_id, start_time_ns):
    result, duplicates, boundary_packets = [], 0, 0
    previous_time, previous_yaw, unwrapped = None, None, None
    for row in records:
        if row.get('source') != 'pose':
            continue
        native = parsed_model(row['raw_protobuf'])
        source_time = stamp(native, 'time')
        if source_time <= start_time_ns:
            # Model-reset poses can have the same clock as a pre-reset cached
            # publication. Both stay in raw JSONL; neither is a motion sample.
            boundary_packets += 1
            continue
        selected = [pose for pose in native.get('pose', []) if int(first(pose, 'id', '-1')) == rotor_id]
        if not selected:
            continue
        require(len(selected) == 1, 'Native pose packet repeats the rotor link')
        require(str(first(selected[0], 'name')).split('::')[-1] == ROTOR,
                'Native pose ID matches but rotor entity name differs')
        yaw = quaternion_yaw(selected[0])
        if previous_time is not None:
            require(source_time >= previous_time, 'Native pose source clock regressed without a time-reset request')
            if source_time == previous_time:
                duplicates += 1
                continue
            delta = math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw))
            require(abs(delta) < .5, 'Native pose sampling is too sparse to unwrap rotation unambiguously')
            unwrapped += delta
        else:
            unwrapped = yaw
        result.append({'time_ns': source_time, 'yaw_wrapped_rad': yaw, 'yaw_unwrapped_rad': unwrapped,
                       'receive_monotonic_ns': row['receive_monotonic_ns']})
        previous_time, previous_yaw = source_time, yaw
    return result, duplicates, boundary_packets


def analyze_motion(records, config, t0_ns, initial_yaw, rotor_id):
    poses, duplicates, boundary_packets = pose_rows(records, rotor_id, t0_ns)
    require(len(poses) >= 70, 'Too few independent native rotor orientations')
    window = [row for row in poses if .03 <= (row['time_ns']-t0_ns)/1e9 <= 3.25]
    require(len(window) >= 60 and (window[-1]['time_ns']-window[0]['time_ns'])/1e9 >= 2.8,
            'Native pose source timestamps do not cover the physical response')
    # Compare increments from the first actual orientation, avoiding a 2*pi
    # branch choice and retaining the angle preserved by a model-only reset.
    # The velocity is zero because Gazebo itself cleared physics states.
    tau, drag = config['drive_torque'], config['drag']
    terminal, time_constant = tau/drag, INERTIA/drag
    def angle(t):
        return terminal*(t-time_constant*(-math.expm1(-t/time_constant)))
    first_time = (window[0]['time_ns']-t0_ns)/1e9
    theta0 = window[0]['yaw_unwrapped_rad']
    residuals = [(row['yaw_unwrapped_rad']-theta0)-(angle((row['time_ns']-t0_ns)/1e9)-angle(first_time))
                 for row in window]
    tolerance = .025
    require(max(map(abs, residuals)) < tolerance,
            'Independent native angular displacement does not match actual torque/drag dynamics: '+repr(max(map(abs, residuals))))
    intervals = []
    left = 0
    for right in range(1, len(window)):
        dt = (window[right]['time_ns']-window[left]['time_ns'])/1e9
        if dt < .04:
            continue
        require(dt < .15, 'Native pose gap is too large for the declared finite-difference comparison')
        a, b = window[left], window[right]
        t1, t2 = (a['time_ns']-t0_ns)/1e9, (b['time_ns']-t0_ns)/1e9
        actual = (b['yaw_unwrapped_rad']-a['yaw_unwrapped_rad'])/dt
        expected = (angle(t2)-angle(t1))/dt
        intervals.append({'start_time_ns': a['time_ns'], 'end_time_ns': b['time_ns'],
                          'native_finite_difference_rad_s': actual, 'analytic_interval_mean_rad_s': expected,
                          'residual_rad_s': actual-expected})
        left = right
    require(len(intervals) >= 30 and max(abs(row['residual_rad_s']) for row in intervals) < .06,
            'Native quaternion finite differences do not match physical response')
    native_tail = [row['native_finite_difference_rad_s'] for row in intervals[-5:]]
    require(all(value*tau > 0 for value in native_tail), 'Actual native rotor turns in the wrong direction')
    plugin_rows = [plugin(row) for row in records if row.get('source') == 'plugin'
                   and row['payload'].get('sim_time_s', -1) >= t0_ns/1e9]
    require(len(plugin_rows) >= 50, 'Actual plugin instrumentation was not observed')
    for row in plugin_rows:
        require(abs(row['drive_torque']-tau) < 1e-12 and abs(row['drag']-drag) < 1e-12,
                'Loaded plugin parameters differ from the independently selected physical case')
        if row.get('last_hook') == 'update':
            require(abs(row['applied_torque_nm']-(tau-drag*row['omega_rad_s'])) < 1e-10,
                    'Actual callback force instrumentation differs from the compiled force rule')
    cross_errors = []
    for interval in intervals:
        mid = (interval['start_time_ns']+interval['end_time_ns'])/2e9
        nearest = min(plugin_rows, key=lambda row: abs(row['sim_time_s']-mid))
        if abs(nearest['sim_time_s']-mid) <= .03:
            cross_errors.append(interval['native_finite_difference_rad_s']-nearest['omega_rad_s'])
    require(len(cross_errors) >= 25 and max(map(abs, cross_errors)) < .08,
            'Plugin velocity instrumentation disagrees with independent native pose motion')
    return {'passed': True, 'native_pose_rows': len(poses), 'pose_duplicates_retained_in_raw': duplicates,
            'pre_epoch_or_reset_boundary_pose_packets_retained_in_raw': boundary_packets,
            'source_window_seconds': [(row['time_ns']-t0_ns)/1e9 for row in (window[0], window[-1])],
            'native_initial_entity_yaw_rad': initial_yaw, 'native_first_yaw_rad': window[0]['yaw_wrapped_rad'],
            'native_rotor_entity_id': rotor_id,
            'inertia_kg_m2': INERTIA, 'analytic_terminal_rad_s': terminal, 'time_constant_s': time_constant,
            'maximum_angle_increment_residual_rad': max(map(abs, residuals)), 'angle_tolerance_rad': tolerance,
            'finite_difference_intervals': intervals, 'native_tail_mean_rad_s': sum(native_tail)/len(native_tail),
            'maximum_native_vs_plugin_velocity_error_rad_s': max(map(abs, cross_errors)),
            'angle_residuals_rad': residuals,
            'interpretation': 'Native pose is independent truth; plugin values are pre-integration callback instrumentation.'}


def mapped_library(session, artifacts, name):
    record = json.loads((session.workspace/'build/model_lab.json').read_text(encoding='utf-8'))
    path = (session.workspace/'build/lib/libgazeboteach_l26_model.so').resolve()
    require(sha(path) == record['library_sha256'], 'Prepared ModelPlugin ELF hash differs')
    candidates = []
    for pid in live_token_processes(session.token):
        proc = Path('/proc', str(pid))
        try:
            if (proc/'comm').read_text(encoding='utf-8').strip() != 'gzserver':
                continue
            lines = []
            for line in (proc/'maps').read_text(encoding='utf-8').splitlines():
                fields = line.split(None, 5)
                if len(fields) == 6 and Path(fields[5]).resolve() == path:
                    info = path.stat()
                    major, minor = [int(value, 16) for value in fields[3].split(':')]
                    require(int(fields[4]) == info.st_ino and (major, minor) == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'Native ModelPlugin mapping does not identify the expected ELF inode/device')
                    lines.append(line)
            if lines:
                target = artifacts/(name+'-plugin.maps')
                target.write_text('\n'.join(lines)+'\n', encoding='utf-8')
                candidates.append({'pid': pid, 'path': str(path), 'library_sha256': sha(path),
                                   'maps': str(target), 'maps_sha256': sha(target)})
        except OSError:
            continue
    require(len(candidates) == 1, 'Expected one owned native server mapping the actual ModelPlugin')
    return candidates[0]


def reset(session, artifacts, label):
    before = statistics(wire(session, artifacts, label+'-before', 'stats'))
    old = plugin(wire(session, artifacts, label+'-plugin-before', 'snapshot'))
    native_before = entity_rotor(wire(session, artifacts, label+'-entity-before', 'entity'))
    wire(session, artifacts, label+'-request', 'control', '--reset-models')
    deadline = time.monotonic()+5
    polls = []
    while True:
        row = wire(session, artifacts, label+'-plugin-after-'+str(len(polls)), 'snapshot')
        state = plugin(row)
        polls.append(row)
        if state['reset_count'] >= old['reset_count']+1:
            break
        require(time.monotonic() < deadline, 'Real ModelPlugin Reset hook was not observed')
        time.sleep(.03)
    after = statistics(wire(session, artifacts, label+'-after', 'stats'))
    native = entity_rotor(wire(session, artifacts, label+'-entity', 'entity'))
    reset_delta = after['sim_time_ns']-before['sim_time_ns']
    # Classic can consume one native reset-management tick before Update
    # returns without an ordinary WorldUpdateBegin callback. This is a model
    # reset, not a time reset; retain that tick instead of falsifying its clock.
    require(before['paused'] and after['paused'] and 0 <= reset_delta <= 1000000,
            'A paused model reset rewound time, unpaused, or consumed more than one reset-management tick')
    epoch_ns = round(state['epoch_time_s']*1e9)
    require(before['sim_time_ns'] <= epoch_ns <= after['sim_time_ns'],
            'Actual Reset-hook epoch is not bracketed by native simulation clocks')
    require(state['load_count'] == old['load_count'] == 1
            and state['init_count'] == old['init_count'] and old['init_count'] >= 1,
            'Model Reset unexpectedly re-ran Load or Init')
    # Classic Base::MODEL reset resets whole-model state and physics, not this
    # child link's internal hinge angle. Assert native preservation explicitly
    # rather than using a false zero-angle expectation or setting a pose.
    yaw_change = math.atan2(math.sin(native['yaw_rad']-native_before['yaw_rad']),
                            math.cos(native['yaw_rad']-native_before['yaw_rad']))
    require(state['epoch_updates'] == 0 and abs(yaw_change) < 1e-7 and abs(state['omega_rad_s']) < 1e-7,
            'Model-only reset did not preserve native rotor angle, clear velocity and reset bookkeeping')
    return {'before': before, 'after': after, 'plugin_before': old, 'plugin_after': state,
            'native_before': native_before, 'native_after': native, 'native_yaw_change_on_reset_rad': yaw_change,
            'unmodified_reset_poll_responses': polls,
            'reset_clock_delta_ns': reset_delta, 'actual_reset_epoch_ns': epoch_ns,
            'note': 'Model-only reset preserves the internal hinge angle and clears physics velocities. A reset-management tick need not invoke an ordinary plugin update.'}


def rejected_saved_input(session, artifacts):
    before = statistics(wire(session, artifacts, 'saved-fault-before', 'stats'))
    old = plugin(wire(session, artifacts, 'saved-fault-plugin-before', 'snapshot'))
    path = session.workspace/'parameters.json'
    original, expected = path.read_bytes(), hashes(session.workspace)
    try:
        value = json.loads(original.decode('utf-8'))
        value['drive_torque'] += .005
        save(path, value)
        output = artifacts/'saved-without-reload.log'
        status = run(session, [sys.executable, str(session.workspace/'lab.py'), 'ready'], output, timeout=15)
        require(status != 0, 'Saved-but-not-reloaded parameters were not rejected')
    finally:
        path.write_bytes(original)
    after = statistics(wire(session, artifacts, 'saved-fault-after', 'stats'))
    state = plugin(wire(session, artifacts, 'saved-fault-plugin-after', 'snapshot'))
    require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations']
            and state['updates'] == old['updates'] and state['q_rad'] == old['q_rad'],
            'Rejected saved-input preflight mutated the paused physical experiment')
    require(hashes(session.workspace) == expected, 'Rejected preflight changed other personal source files')
    wire(session, artifacts, 'exact-input-repaired-ready', 'snapshot')
    status = run(session, [sys.executable, str(session.workspace/'lab.py'), 'ready'],
                 artifacts/'exact-input-repaired-ready.log', timeout=15)
    require(status == 0, 'Exact original input bytes did not repair readiness')
    return {'name': 'saved_without_reload_and_exact_repair', 'passed': True, 'before': before, 'after': after,
            'negative_log': str(output), 'physics_not_advanced': True}


def case(session, artifacts, name, config, extended):
    session.pause(True)
    initial = reset(session, artifacts, name+'-initial-reset')
    t0 = initial['actual_reset_epoch_ns']
    save(artifacts/(name+'-initial-native-state.json'), {'parameters': config, 'initial_reset': initial})
    observed = artifacts/(name+'-native-motion.jsonl')
    command = shlex.join([str(session.workspace/'build/model_observer'), 'watch', '--seconds', '25'])
    process = session._spawn('Independent native pose/statistics wire',
                             ['bash', '--noprofile', '--norc', '-c', command+' > '+shlex.quote(str(observed))])
    try:
        deadline = time.monotonic()+5
        while True:
            rows, _ = events(observed, live=True)
            if any(row.get('source') == 'client_ready' for row in rows) and any(row.get('source') == 'world_stats' for row in rows):
                break
            require(process.poll() is None and time.monotonic() < deadline, 'Native observer did not connect')
            time.sleep(.03)
        session.pause(False)
        deadline = time.monotonic()+20
        while True:
            rows, _ = events(observed, live=True)
            latest = [statistics(row) for row in rows if row.get('source') == 'world_stats']
            if latest and latest[-1]['sim_time_ns']-t0 >= 3350000000:
                break
            require(process.poll() is None and time.monotonic() < deadline, 'Native world stream did not advance')
            time.sleep(.04)
        session.pause(True)
        time.sleep(.12)
    finally:
        if process.poll() is None:
            terminate_owned(process)
    rows, diagnostics = events(observed, live=True)
    motion = analyze_motion(rows, config, t0, initial['native_after']['yaw_rad'], initial['native_after']['rotor_entity_id'])
    motion.update(raw_jsonl=str(observed), raw_jsonl_sha256=sha(observed), wire_diagnostics=diagnostics)
    output = {'name': name, 'passed': True, 'parameters': config, 'initial_reset': initial, 'motion': motion}
    if extended:
        output['saved_input_guard'] = rejected_saved_input(session, artifacts)
        before = statistics(wire(session, artifacts, 'paused-before', 'stats'))
        state_before = plugin(wire(session, artifacts, 'paused-plugin-before', 'snapshot'))
        time.sleep(.25)
        after = statistics(wire(session, artifacts, 'paused-after', 'stats'))
        state_after = plugin(wire(session, artifacts, 'paused-plugin-after', 'snapshot'))
        require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations']
                and state_before['updates'] == state_after['updates'], 'Paused world advanced time, ticks or plugin callbacks')
        entity_before = entity_rotor(wire(session, artifacts, 'step-entity-before', 'entity'))
        session.step(100)
        stepped = statistics(wire(session, artifacts, 'step-after', 'stats'))
        state_stepped = plugin(wire(session, artifacts, 'step-plugin-after', 'snapshot'))
        entity_after = entity_rotor(wire(session, artifacts, 'step-entity-after', 'entity'))
        require(stepped['paused'] and stepped['iterations']-after['iterations'] == 100
                and stepped['sim_time_ns']-after['sim_time_ns'] == 100000000,
                'One 100-tick publication did not cause exactly 0.1 native simulation seconds')
        require(state_stepped['updates']-state_after['updates'] == 100,
                'ModelPlugin actual update callbacks did not match the 100 native ticks')
        delta = math.atan2(math.sin(entity_after['yaw_rad']-entity_before['yaw_rad']),
                           math.cos(entity_after['yaw_rad']-entity_before['yaw_rad']))
        tc, terminal = INERTIA/config['drag'], config['drive_torque']/config['drag']
        a, b = (after['sim_time_ns']-t0)/1e9, (stepped['sim_time_ns']-t0)/1e9
        predicted = terminal*((b-a)+tc*(math.exp(-b/tc)-math.exp(-a/tc)))
        require(abs(delta-predicted) < .012, 'Independent native rotor pose did not advance consistently with the actual 100 ticks')
        output['pause_and_100_ticks'] = {'passed': True, 'before': before, 'paused_after': after,
            'stepped': stepped, 'plugin_before': state_after, 'plugin_after': state_stepped,
            'native_yaw_delta_rad': delta, 'analytic_yaw_delta_rad': predicted}
        output['actual_model_reset'] = reset(session, artifacts, 'post-motion-reset')
    save(artifacts/(name+'-analysis.json'), output)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(item for item in lessons if item.id == '26' and item.profile == 'classic11-noetic')
    source = hashes(lesson.folder)
    started, sessions = time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': False, 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l26-') as temporary:
        try:
            for name, config in [('baseline', {'drive_torque': .03, 'drag': .015}),
                                 ('torque_only', {'drive_torque': -.03, 'drag': .015}),
                                 ('drag_only', {'drive_torque': .03, 'drag': .03})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=False, terminals=False)
                mapping = mapped_library(session, artifacts, name)
                save(artifacts/(name+'-actual-library.json'), mapping)
                shutil.copy2(str(workspace/'build/model_lab.json'), str(artifacts/(name+'-model_lab.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(name+'-generated-world.sdf')))
                row = case(session, artifacts, name, config, extended=name == 'baseline')
                row['actual_model_plugin_mapping'] = mapping
                row['prepared_input_sha256'] = hashes(workspace)
                result['scenarios'].append(row)
                session.stop(reason='L26 independent physical case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            baseline, reversed_case, more_drag = result['scenarios']
            require(baseline['motion']['native_tail_mean_rad_s'] > 1.8
                    and reversed_case['motion']['native_tail_mean_rad_s'] < -1.8,
                    'Opposite real drive torques did not produce opposite rotor motion')
            require(.9 < more_drag['motion']['native_tail_mean_rad_s'] < 1.05,
                    'Doubling only viscous drag did not change actual terminal motion toward 1 rad/s')
            result['passed'] = True
            result['active_case'] = None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason='L26 formal acceptance cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/(name+'-runtime.log')))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/(name+'-session.json')))
                except Exception as error:
                    result['cleanup_errors'].append(name+': '+repr(error))
            result['leftovers'] = {session.token: live_token_processes(session.token) for _, session in sessions}
            if result['cleanup_errors'] or any(result['leftovers'].values()):
                result['passed'] = False
            result['source_templates_unchanged'] = hashes(lesson.folder) == source
            if not result['source_templates_unchanged']:
                result['passed'] = False
            result['elapsed_seconds'] = round(time.monotonic()-started, 3)
            save(report_path, result)
    print(json.dumps({key: value for key, value in result.items() if key not in ('scenarios', 'source_sha256')},
                     ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
