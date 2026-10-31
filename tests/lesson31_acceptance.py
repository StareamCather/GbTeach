#!/usr/bin/env python3
"""Independent L31 process plugin, real gravity, sampling and reset acceptance.

Native Physics, Scene, Model, PoseStamped and WorldStatistics protobufs supply
the physical evidence. SystemPlugin state is instrumentation, checked against
those independent results. Three fresh worlds vary gravity or recording stride
alone. One WorldControl publication is sent per step batch and never replayed.
Prephysics sample labels use the already-incremented world time, whereas their
poses must match the independent entity response from the preceding ODE tick.
Original wire arrival order and bytes are retained, including reset boundaries.
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
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model

WORLD, MODEL, STEP_NS, TOTAL_STEPS = 'lesson31', 'falling_cube', 1000000, 200


def require(value, message):
    if not value:
        raise AssertionError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def hashes(folder):
    folder = Path(folder)
    return {p.relative_to(folder).as_posix(): sha(p) for p in sorted(folder.rglob('*'))
            if p.is_file() and not {'__pycache__', 'build', '.git'} & set(p.relative_to(folder).parts)}


def events(path, live=False):
    raw = Path(path).read_text(encoding='utf-8') if Path(path).exists() else ''
    rows, noise, partial = [], [], None
    lines = raw.splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            if live and index == len(lines)-1 and not raw.endswith('\n'):
                partial = line
            else:
                noise.append(line)
            continue
        require(isinstance(row, dict), 'Wire result is not an object')
        require(row.get('source') not in ('client_error', 'decode_error'), 'Native observer error: '+repr(row))
        rows.append(row)
    return rows, {'non_json_lines': noise, 'trailing_partial_line': partial}


def run(session, argv, output, timeout=16):
    command = shlex.join([str(v) for v in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L31 native query', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    require(run(session, [session.workspace/'build/system_observer', mode, *arguments], path) == 0,
            'Actual native command failed: '+str(path))
    rows, diagnostics = events(path)
    source = {'snapshot': 'snapshot', 'stats': 'world_stats', 'entity': 'entity',
              'world': 'world', 'physics': 'physics', 'control': 'control_sent'}[mode]
    selected = [v for v in rows if v.get('source') == source]
    require(len(selected) == 1, 'Expected one native '+source+' result: '+str(path))
    result = selected[0]
    result['_wire_artifact'], result['_wire_sha256'], result['_wire_diagnostics'] = str(path), sha(path), diagnostics
    return result


def stamp(native, field):
    value = first(native, field)
    require(isinstance(value, dict), 'Missing native timestamp: '+field)
    return int(first(value, 'sec', '0'))*1000000000+int(first(value, 'nsec', '0'))


def stats(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native pause state absent')
    return {'sim_time_ns': stamp(native, 'sim_time'), 'iterations': int(first(native, 'iterations')),
            'paused': first(native, 'paused') == 'true', 'raw_wire': row}


def plugin(row):
    if row['source'] == 'plugin':
        native = parsed_model(row['raw_protobuf'])
        require(first(native, 'type') == 'STRING', 'Custom native message is not Any.STRING')
        value = json.loads(first(native, 'string_value'))
        require(value == row['payload'], 'Custom mirror differs from original Any bytes')
    else:
        require(row.get('request_id') == row.get('response_id') and row['payload'].get('query_nonce'),
                'Snapshot lacks matching request ID and nonce')
        value = row['payload']
    require(value['world'] == WORLD and value['plugin_type'] == 'SystemPlugin'
            and value['scope'] == 'gzserver_process' and not value['last_error'], 'Wrong process plugin or actual error')
    return value


def position(model):
    pose = first(model, 'pose')
    vector = first(pose, 'position')
    require(isinstance(vector, dict), 'Native position missing')
    result = [float(first(vector, axis, '0')) for axis in ('x', 'y', 'z')]
    require(all(math.isfinite(v) for v in result), 'Native position is not finite')
    return result


def entity(session, artifacts, label, name=MODEL):
    native = parsed_model(wire(session, artifacts, label, 'entity', '--name', name)['raw_protobuf'])
    require(str(first(native, 'name')).split('::')[-1] == name and int(first(native, 'id')) > 0,
            'Native entity reply has the wrong identity')
    return native


def box_size(model):
    links = model.get('link', [])
    require(len(links) == 1 and len(links[0].get('collision', [])) == 1, 'Native collision topology changed')
    box = first(first(first(links[0]['collision'][0], 'geometry'), 'box'), 'size')
    return [float(first(box, axis)) for axis in ('x', 'y', 'z')]


def physics(session, artifacts, label, gravity):
    row = wire(session, artifacts, label, 'physics')
    native = parsed_model(row['raw_protobuf'])
    vector = first(native, 'gravity')
    require(isinstance(vector, dict), 'Native Physics lacks actual gravity')
    actual = [float(first(vector, axis, '0')) for axis in ('x', 'y', 'z')]
    require(max(abs(a-b) for a, b in zip(actual, (0, 0, -gravity))) < 1e-10,
            'Actual native physics gravity differs from expected '+str(gravity))
    require(abs(float(first(native, 'max_step_size'))-.001) < 1e-12, 'Actual native physics dt changed')
    return {'gravity_m_s2': actual, 'max_step_size_s': .001, 'raw_wire': row}


def native_scene(session, artifacts, label):
    row = wire(session, artifacts, label, 'world')
    native = parsed_model(row['raw_protobuf'])
    models = {str(first(m, 'name')).split('::')[-1]: m for m in native.get('model', [])}
    require(set(models) == {'floor', MODEL}, 'Actual native scene has unexpected models')
    floor = entity(session, artifacts, label+'-floor', 'floor')
    require(first(floor, 'is_static') == 'true', 'Actual floor is not static')
    require(max(abs(a-b) for a, b in zip(box_size(floor), (6, 5, .2))) < 1e-10,
            'Actual floor collision geometry differs')
    require(abs(position(floor)[2]+.1) < 1e-10, 'Actual floor top is not at z=0')
    return {'raw_wire': row, 'models': models, 'native_floor_entity': floor}


def mapped_library(session, artifacts, label, record):
    library = (session.workspace/'build/lib/libgazeboteach_l31_system.so').resolve()
    require(sha(library) == record['library_sha256'], 'Prepared SystemPlugin ELF hash differs')
    info, found, clients = library.stat(), [], []
    for pid in live_token_processes(session.token):
        proc = Path('/proc')/str(pid)
        try:
            kind = (proc/'comm').read_text().strip()
            if kind not in ('gzserver', 'gzclient'):
                continue
            matches = []
            for line in (proc/'maps').read_text().splitlines():
                fields = line.split(None, 5)
                if len(fields) == 6 and Path(fields[5]).resolve() == library:
                    device = tuple(int(v, 16) for v in fields[3].split(':'))
                    require(int(fields[4]) == info.st_ino and device == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'Actual mapped SystemPlugin inode/device differs')
                    matches.append(line)
            if matches:
                path = artifacts/(label+'-'+kind+'-'+str(pid)+'.maps')
                path.write_text('\n'.join(matches)+'\n', encoding='utf-8')
                argv = (proc/'cmdline').read_bytes().decode().strip('\0').split('\0')
                item = {'pid': pid, 'path': str(library), 'library_sha256': sha(library), 'argv': argv,
                        'maps': str(path), 'maps_sha256': sha(path)}
                (found if kind == 'gzserver' else clients).append(item)
        except OSError:
            continue
    require(len(found) == 1 and not clients, 'SystemPlugin must map in exactly one owned gzserver')
    argv = found[0]['argv']
    require('-u' in argv and '-s' in argv and argv[argv.index('-s')+1] == library.name,
            'Actual server was not started paused with the process SystemPlugin argument')
    return {'server': found[0], 'client_mappings': clients}


def lifecycle(state):
    require(state['load_count'] == state['init_count'] == state['world_created_count'] == 1
            and state['reset_count'] == 0, 'Actual SystemPlugin lifecycle differs from Classic11 setup/World reset')
    hooks = state['lifecycle']
    require([h['sequence'] for h in hooks] == list(range(1, len(hooks)+1)), 'Actual lifecycle sequence has gaps')
    require([h['hook'] for h in hooks[:3]] == ['load', 'init', 'world_created'],
            'SystemPlugin Load/Init/world-created order differs')
    require(all(not h['world_available'] and h['updates'] == 0 for h in hooks[:2])
            and hooks[2]['world_available'], 'Load/Init accessed a world before its binding event')
    require(all(h['named_world_exists'] is False for h in hooks[:2])
            and hooks[2]['named_world_exists'] is True,
            'Actual physics::has_world measurement differs from pre-world Load/Init and world-created order')
    return {'actual_counters': {k: state[k] for k in ('load_count', 'init_count', 'reset_count', 'world_created_count')},
            'actual_lifecycle': hooks,
            'world_available_meaning': 'This SystemPlugin instance has a bound WorldPtr; not a global world-count measurement.',
            'classic11_call_chain': 'gazebo_shared.setup: SystemPlugin.Load -> transport init/run -> SystemPlugin.Init, before Server.LoadWorld; World.Reset resets world plugins, not process SystemPlugins.'}


def watcher(session, artifacts, label):
    path, stop = artifacts/(label+'.jsonl'), artifacts/(label+'.stop')
    if stop.exists():
        stop.unlink()
    command = shlex.join([str(session.workspace/'build/system_observer'), 'watch', '--seconds', '90', '--stop-file', str(stop)])
    process = session._spawn('Independent L31 wire observation',
                             ['bash', '--noprofile', '--norc', '-c', command+' > '+shlex.quote(str(path))+' 2>&1'])
    deadline = time.monotonic()+6
    try:
        while True:
            rows, _ = events(path, live=True)
            if any(v.get('source') == 'client_ready' for v in rows) and any(v.get('source') == 'world_stats' for v in rows):
                return process, path, stop
            require(process.poll() is None and time.monotonic() < deadline, 'Native watcher did not connect')
            time.sleep(.04)
    except BaseException:
        stop_watcher(process, stop)
        raise


def stop_watcher(process, stop):
    if process.poll() is None:
        stop.write_text('Completed independent L31 observations\n', encoding='utf-8')
        try:
            process.wait(timeout=7)
        except BaseException:
            terminate_owned(process)
            raise
    require(process.returncode == 0, 'Observer did not complete its normal SDK IO stop/join path')


def advance(session, artifacts, label, before, count):
    require(before['paused'] and count > 0, 'Independent stepping requires paused physics and a positive tick count')
    sent = wire(session, artifacts, label+'-single-control', 'control', '--steps', str(count))
    control = parsed_model(sent['raw_protobuf'])
    require(first(control, 'pause') == 'true' and int(first(control, 'multi_step')) == count,
            'Native one-shot control bytes differ from requested ticks')
    target = before['iterations']+count
    deadline, polls = time.monotonic()+7, []
    while True:
        current = stats(wire(session, artifacts, label+'-clock-'+str(len(polls)), 'stats'))
        polls.append(current)
        if current['iterations'] >= target:
            break
        require(time.monotonic() < deadline, 'Actual native ticks did not complete; control was not replayed')
    require(current['paused'] and current['iterations'] == target
            and current['sim_time_ns']-before['sim_time_ns'] == count*STEP_NS,
            'Single native publication did not produce exactly the requested paused ticks/time')
    return current, {'passed': True, 'single_native_publication': sent, 'requested_steps': count,
                     'world_before': before, 'world_after': current, 'completion_polls': polls}


def checkpoint(session, artifacts, label):
    state = plugin(wire(session, artifacts, label+'-plugin', 'snapshot'))
    body = entity(session, artifacts, label+'-cube')
    require(max(abs(a-.4) for a in box_size(body)) < 1e-12, 'Actual falling cube collision is not .4m')
    return state, body


def prephysics_check(measured, prior_body, after_world, ordinal, gravity):
    require(measured['phase'] == 'world_update_begin_prephysics', 'Sample phase is not explicitly prephysics')
    require(measured['epoch_updates'] == measured['world_iterations'] == ordinal
            and measured['sim_time_ns'] == after_world['sim_time_ns'] == ordinal*STEP_NS
            and abs(measured['sim_time_s']-ordinal*.001) < 1e-12,
            'Prephysics clock label/ordinal differs from independently completed native ticks')
    require(measured['model_id'] == int(first(prior_body, 'id')), 'Prephysics sample changed the native model identity')
    actual = position(prior_body)
    recorded = [measured[k] for k in ('x_m', 'y_m', 'z_m')]
    require(max(abs(a-b) for a, b in zip(recorded, actual)) < 1e-8,
            'Prephysics pose differs from independent entity response immediately before this ODE tick')
    require(abs(measured['actual_gravity_z']+gravity) < 1e-10, 'Prephysics sample gravity mirror differs')
    return {'passed': True, 'recorded_prephysics': measured, 'independent_prior_native_entity': prior_body,
            'native_completed_world': after_world, 'pose_max_error_m': max(abs(a-b) for a, b in zip(recorded, actual)),
            'interpretation': 'Time/iterations increment before WorldUpdateBegin; its pose precedes the corresponding ODE integration.'}


def trajectory(rows, model_id, gravity):
    values, duplicates, previous = [], 0, None
    for row in rows:
        if row.get('source') != 'pose':
            continue
        packet = parsed_model(row['raw_protobuf'])
        found = [p for p in packet.get('pose', []) if int(first(p, 'id', '-1')) == model_id]
        if not found:
            continue
        require(len(found) == 1 and str(first(found[0], 'name')).split('::')[-1] == MODEL,
                'Native pose stream ID/name mismatch')
        native_time, xyz = stamp(packet, 'time'), position({'pose': [found[0]]})
        if previous:
            require(native_time >= previous['sim_time_ns'], 'Native pose time regressed before any reset')
            if native_time == previous['sim_time_ns']:
                require(xyz == previous['position_m'], 'Repeated native pose timestamp carries changed coordinates')
                duplicates += 1
                continue
        expected = 2-.5*gravity*(native_time/1e9)**2
        require(abs(xyz[2]-expected) < .004, 'Actual native pose stream disagrees with independent free fall')
        value = {'sim_time_ns': native_time, 'position_m': xyz, 'continuous_free_fall_z_m': expected,
                 'height_residual_m': xyz[2]-expected}
        values.append(value)
        previous = value
    require(len(values) >= 3 and values[-1]['sim_time_ns'] > values[0]['sim_time_ns'],
            'Too few independent native pose source times to establish actual motion')
    require(values[-1]['position_m'][2] < values[0]['position_m'][2]-.02, 'Native pose stream did not show falling motion')
    return {'passed': True, 'original_arrival_order': values, 'duplicate_packets_retained_in_raw': duplicates}


def sampling(rows, final, config):
    stride = config['record_every_steps']
    expected = list(range(stride, TOTAL_STEPS+1, stride))
    actual = final['recent_records']
    require(final['updates'] == final['epoch_updates'] == TOTAL_STEPS
            and final['record_count'] == final['epoch_record_count'] == len(expected),
            'Actual callback/sample counters do not match independently measured native ticks')
    require([v['epoch_updates'] for v in actual] == expected, 'Actual recorded stride/first sample/count differs')
    require([v['sim_time_ns'] for v in actual] == [v*STEP_NS for v in expected], 'Actual sample source times differ from native tick cadence')
    wire_records, previous, duplicate = [], None, 0
    for row in rows:
        if row.get('source') != 'plugin':
            continue
        state = plugin(row)
        require(state['record_every_steps'] == stride, 'Actual published recording stride differs')
        sample = state['last_record']
        if sample is None:
            continue
        identity = sample['epoch_updates']
        if previous is not None:
            require(identity >= previous['epoch_updates'], 'Custom sample arrival time regressed before reset')
            if identity == previous['epoch_updates']:
                require(sample == previous, 'Same custom sample identity carries different values')
                duplicate += 1
                continue
        wire_records.append(sample)
        previous = sample
    require([v['epoch_updates'] for v in wire_records] == expected, 'Continuous observer did not receive every actual stride sample')
    require(wire_records == actual, 'Actual published sample bytes differ from retained plugin record history')
    return {'passed': True, 'expected_ordinals_from_native_tick_count': expected, 'actual_records': actual,
            'actual_sample_count': len(actual), 'published_records_in_arrival_order': wire_records,
            'duplicates_retained_in_raw': duplicate, 'prephysics_label_convention': 'dt*n label; pose after n-1 physical integrations'}


def physical_case(session, artifacts, label, config):
    initial = stats(wire(session, artifacts, label+'-initial-world', 'stats'))
    require(initial['paused'] and initial['sim_time_ns'] == initial['iterations'] == 0, 'Fresh actual world did not start paused at zero')
    scene = native_scene(session, artifacts, label+'-initial-scene')
    initial_state, initial_body = checkpoint(session, artifacts, label+'-initial')
    require(abs(position(initial_body)[2]-2) < 1e-10, 'Fresh real cube does not start at z=2')
    require(initial_state['updates'] == initial_state['record_count'] == 0 and not initial_state['gravity_applied'],
            'Initial paused SystemPlugin falsely applied gravity or reported physical updates')
    initial_physics = physics(session, artifacts, label+'-sdf-gravity-before-first-update', 9.8)
    lifecycle_result = lifecycle(initial_state)
    process, path, stop = watcher(session, artifacts, label+'-motion-wire')
    pulses = []
    try:
        first_world, pulse = advance(session, artifacts, label+'-first-tick', initial, 1)
        pulses.append(pulse)
        first_state, first_body = checkpoint(session, artifacts, label+'-first-tick')
        actual_physics = physics(session, artifacts, label+'-gravity-after-first-update', config['gravity_m_s2'])
        require(first_state['gravity_applied'] and first_state['updates'] == first_state['epoch_updates'] == 1
                and first_state['record_count'] == 0, 'First real tick failed to apply gravity exactly once before recording stride')
        first_check = prephysics_check(first_state['last_prephysics'], initial_body, first_world, 1, config['gravity_m_s2'])
        require(position(first_body)[2] < position(initial_body)[2], 'Native cube did not physically integrate the first gravity tick')
        n = config['record_every_steps']
        before_record, pulse = advance(session, artifacts, label+'-before-first-record', first_world, n-2)
        pulses.append(pulse)
        before_state, before_body = checkpoint(session, artifacts, label+'-before-first-record')
        require(before_state['record_count'] == 0, 'A sample was recorded before the configured real tick stride')
        record_world, pulse = advance(session, artifacts, label+'-first-record-tick', before_record, 1)
        pulses.append(pulse)
        record_state, record_body = checkpoint(session, artifacts, label+'-first-record-tick')
        require(record_state['record_count'] == 1, 'The first actual stride boundary did not create exactly one record')
        first_record_check = prephysics_check(record_state['last_record'], before_body, record_world, n, config['gravity_m_s2'])
        before_final, pulse = advance(session, artifacts, label+'-before-final-record', record_world, TOTAL_STEPS-1-n)
        pulses.append(pulse)
        before_final_state, before_final_body = checkpoint(session, artifacts, label+'-before-final-record')
        final_world, pulse = advance(session, artifacts, label+'-final-record-tick', before_final, 1)
        pulses.append(pulse)
        final, final_body = checkpoint(session, artifacts, label+'-final-record-tick')
        last_check = prephysics_check(final['last_record'], before_final_body, final_world, TOTAL_STEPS, config['gravity_m_s2'])
        expected_z = 2-.5*config['gravity_m_s2']*(TOTAL_STEPS*.001)**2
        actual_z = position(final_body)[2]
        require(abs(actual_z-expected_z) < .004 and actual_z < 1.93, 'Native final cube does not show the expected gravity-dependent fall')
        require(first(initial_body, 'id') == first(final_body, 'id'), 'Experiment replaced the real falling cube')
        physics(session, artifacts, label+'-final-native-physics', config['gravity_m_s2'])
        deadline = time.monotonic()+4
        while True:
            rows, _ = events(path, live=True)
            published = [plugin(v) for v in rows if v.get('source') == 'plugin']
            if published and published[-1]['epoch_updates'] == TOTAL_STEPS:
                break
            require(time.monotonic() < deadline, 'Last actual sample publication was not received')
            time.sleep(.04)
    finally:
        stop_watcher(process, stop)
    rows, diagnostics = events(path)
    return {'passed': True, 'parameters': config, 'initial_native_world': initial, 'initial_native_scene': scene,
            'initial_native_cube': initial_body, 'initial_plugin': initial_state, 'lifecycle': lifecycle_result,
            'initial_sdf_native_gravity': initial_physics, 'native_gravity_after_first_update': actual_physics,
            'one_shot_step_pulses': pulses, 'first_update_prephysics': first_check, 'first_sample_prephysics': first_record_check,
            'last_sample_prephysics': last_check, 'final_native_world': final_world, 'final_native_cube': final_body,
            'final_plugin': final, 'native_actual_height_m': actual_z, 'independent_continuous_height_m': expected_z,
            'native_height_residual_m': actual_z-expected_z,
            'sampling': sampling(rows, final, config), 'native_pose_trajectory': trajectory(rows, int(first(final_body, 'id')), config['gravity_m_s2']),
            'raw_jsonl': str(path), 'raw_jsonl_sha256': sha(path), 'wire_diagnostics': diagnostics,
            'observer_end': {'returncode': process.poll(), 'stop_file': str(stop), 'normal_stop_file_exit': True}}


def saved_input_guard(session, artifacts, config):
    before = stats(wire(session, artifacts, 'saved-before-world', 'stats'))
    state_before, body_before = checkpoint(session, artifacts, 'saved-before')
    original, personal = (session.workspace/'parameters.json').read_bytes(), hashes(session.workspace)
    log = artifacts/'saved-without-reload.log'
    try:
        save(session.workspace/'parameters.json', dict(config, gravity_m_s2=4.9))
        require(run(session, ['python3', 'lab.py', 'ready'], log) != 0, 'Saved but unreloaded gravity was silently accepted')
        require(any(v in log.read_text(encoding='utf-8').lower() for v in ('reload', 'restart')), 'Stale input has no reload diagnosis')
        after = stats(wire(session, artifacts, 'saved-after-world', 'stats'))
        state_after, body_after = checkpoint(session, artifacts, 'saved-after')
        require(before['paused'] and after['paused'] and before['sim_time_ns'] == after['sim_time_ns']
                and before['iterations'] == after['iterations'] and state_before['updates'] == state_after['updates']
                and state_before['record_count'] == state_after['record_count'] and body_before == body_after,
                'Rejected saved input advanced physics, records, or actual entity state')
        physics(session, artifacts, 'saved-still-native-gravity', config['gravity_m_s2'])
    finally:
        (session.workspace/'parameters.json').write_bytes(original)
    require(hashes(session.workspace) == personal, 'Stale input test changed other personal source bytes')
    require(run(session, ['python3', 'lab.py', 'ready'], artifacts/'saved-exact-repair.log') == 0, 'Exact original bytes did not repair readiness')
    return {'passed': True, 'before': before, 'after': after, 'negative_log': str(log),
            'original_parameters_sha256': hashlib.sha256(original).hexdigest(), 'exact_input_bytes_restored': True}


def reset_checks(session, artifacts, config):
    before = stats(wire(session, artifacts, 'reset-before-world', 'stats'))
    state_before, body_before = checkpoint(session, artifacts, 'reset-before')
    time.sleep(.2)
    still = stats(wire(session, artifacts, 'paused-quiet-world', 'stats'))
    quiet, body_quiet = checkpoint(session, artifacts, 'paused-quiet')
    require(still['paused'] and still['iterations'] == before['iterations'] and still['sim_time_ns'] == before['sim_time_ns']
            and quiet['updates'] == state_before['updates'] and body_quiet == body_before, 'Paused native world continued advancing')
    control = wire(session, artifacts, 'full-reset-single-control', 'control', '--reset-all')
    command = parsed_model(control['raw_protobuf'])
    require(first(first(command, 'reset'), 'all') == 'true', 'Native reset command did not explicitly request full reset')
    deadline, polls = time.monotonic()+7, []
    while True:
        current = plugin(wire(session, artifacts, 'full-reset-plugin-'+str(len(polls)), 'snapshot'))
        polls.append(current)
        if current['world_reset_event_count'] == state_before['world_reset_event_count']+1:
            break
        require(time.monotonic() < deadline, 'Owned world-reset event did not follow native full reset')
    reset_world = stats(wire(session, artifacts, 'full-reset-native-world', 'stats'))
    reset_body = entity(session, artifacts, 'full-reset-native-cube')
    require(reset_world['paused'] and reset_world['sim_time_ns'] == reset_world['iterations'] == 0,
            'Actual full reset did not rewind the native clock and iterations to zero')
    require(abs(position(reset_body)[2]-2) < 1e-10 and first(reset_body, 'id') == first(body_before, 'id'),
            'Full reset failed to restore the original real cube pose/identity')
    require(current['load_count'] == state_before['load_count'] == 1 and current['init_count'] == state_before['init_count'] == 1
            and current['reset_count'] == state_before['reset_count'] == 0 and current['updates'] == state_before['updates']
            and current['record_count'] == state_before['record_count'] and current['epoch_updates'] == current['epoch_record_count'] == 0
            and current['epoch'] == state_before['epoch']+1 and not current['gravity_applied'],
            'World reset incorrectly reloaded/reset the process plugin or counted an ordinary physical callback')
    lifecycle(current)
    resumed_world, pulse = advance(session, artifacts, 'post-reset-first-tick', reset_world, 1)
    post, post_body = checkpoint(session, artifacts, 'post-reset-first-tick')
    require(post['epoch_updates'] == 1 and post['updates'] == state_before['updates']+1 and post['reset_count'] == 0,
            'First real post-reset callback failed to start the new world epoch')
    require(post['gravity_applied'] and post['epoch_record_count'] == 0 and position(post_body)[2] < 2,
            'New world epoch did not physically reapply gravity before its first ODE tick')
    physics(session, artifacts, 'post-reset-actual-native-gravity', config['gravity_m_s2'])
    return {'passed': True, 'quiet_paused_native_world': still, 'single_full_reset_control': control,
            'before_world': before, 'before_plugin': state_before, 'after_world': reset_world, 'after_plugin': current,
            'after_native_cube': reset_body, 'actual_lifecycle_after_reset': current['lifecycle'],
            'post_reset_single_step': pulse, 'post_reset_world': resumed_world, 'post_reset_plugin': post,
            'distinction': 'World reset restored model/time and emitted the global worldReset event; the process object was retained and SystemPlugin.Reset was not dispatched.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(v for v in lessons if v.id == '31' and v.profile == 'classic11-noetic')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': False, 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l31-') as temporary:
        try:
            for name, config in [('baseline', {'gravity_m_s2': 9.8, 'record_every_steps': 10}),
                                 ('gravity_only', {'gravity_m_s2': 4.9, 'record_every_steps': 10}),
                                 ('stride_only', {'gravity_m_s2': 9.8, 'record_every_steps': 20})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=False, terminals=False)
                record = json.loads((workspace/'build/system_lab.json').read_text(encoding='utf-8'))
                require(record['source_sha256']['system_plugin.cc'] == source['system_plugin.cc'], 'Compiled source differs from the original template')
                require(all(sha(workspace/p) == expected for p, expected in record['source_sha256'].items()), 'Prepared source hashes differ')
                require(sha(workspace/'parameters.json') == record['config_sha256'], 'Prepared config SHA differs')
                require(sha(workspace/'build/system_observer') == record['observer_sha256']
                        and sha(workspace/'world.sdf') == record['world_sha256'], 'Actual observer/world differs from prepared hashes')
                require(not ET.parse(str(workspace/'world.sdf')).getroot().findall('.//plugin'), 'SystemPlugin was put into SDF instead of server -s')
                mapping = mapped_library(session, artifacts, name, record)
                row = physical_case(session, artifacts, name, config)
                actual = row['final_plugin']
                require(actual['process_id'] == mapping['server']['pid'] and actual['source_sha256'] == record['source_sha256']['system_plugin.cc']
                        and actual['config_sha256'] == record['config_sha256'] and actual['gravity_m_s2'] == config['gravity_m_s2']
                        and actual['record_every_steps'] == config['record_every_steps'], 'Actual process plugin source/config differs from the fresh prepared inputs')
                if name == 'baseline':
                    row['saved_input_guard'] = saved_input_guard(session, artifacts, config)
                    row['full_world_reset'] = reset_checks(session, artifacts, config)
                row['name'], row['actual_process_mapping'], row['prepared_input_sha256'] = name, mapping, hashes(workspace)
                save(artifacts/(name+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/system_lab.json'), str(artifacts/(name+'-system_lab.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(name+'-generated-world.sdf')))
                result['scenarios'].append(row)
                session.stop(reason='L31 independent process/physics case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            baseline, weaker, stride = result['scenarios']
            drop = 2-baseline['native_actual_height_m']
            weak_drop = 2-weaker['native_actual_height_m']
            require(drop > .15 and abs(drop/weak_drop-2) < .03, 'Halving only applied gravity did not halve the independent native fall')
            require(abs(stride['native_actual_height_m']-baseline['native_actual_height_m']) < 1e-7,
                    'Changing only recording stride changed actual native falling motion')
            require(baseline['sampling']['actual_sample_count'] == 2*stride['sampling']['actual_sample_count'],
                    'Doubling only recording stride failed to halve actual recorded source-time samples')
            result['cross_case_results'] = {'baseline_native_height_m': baseline['native_actual_height_m'],
                'gravity_only_native_height_m': weaker['native_actual_height_m'], 'stride_only_native_height_m': stride['native_actual_height_m'],
                'native_drop_ratio': drop/weak_drop, 'baseline_actual_records': baseline['sampling']['actual_sample_count'],
                'stride_only_actual_records': stride['sampling']['actual_sample_count']}
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason='L31 independent formal acceptance cleanup')
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
    print(json.dumps({k: v for k, v in result.items() if k not in ('scenarios', 'source_sha256')}, ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
