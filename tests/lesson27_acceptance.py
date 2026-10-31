#!/usr/bin/env python3
"""Independent L27 native factory, free-fall, lifecycle and scoped deletion checks.

Only the SDK client's unmodified protobuf text supplies independent physical
evidence. Plugin counters and cached poses are instrumentation. Fresh personal
workspaces change count or drop height separately; no lab.batch/verify result
is used as the independent oracle. Every wire packet remains in raw JSONL.
A fourth count-one/high-precision-height case is a separate SDK serialization
regression, not another pedagogical single-variable comparison.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import statistics as numeric_statistics
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

WORLD, SENTINEL = 'lesson27', 'external_sentinel'
GROUND_TOP, SIDE, GRAVITY = .1, .16, 9.8


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
        require(isinstance(row, dict), 'Wire JSON is not an object')
        require(row.get('source') not in ('client_error', 'decode_error'), 'Wire client error: '+repr(row))
        result.append(row)
    return result, {'non_json_lines': other, 'trailing_partial_line': incomplete}


def run(session, argv, output, timeout=15):
    command = shlex.join([str(x) for x in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L27 command', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    code = run(session, [session.workspace/'build/world_observer', mode, *arguments], path)
    require(code == 0, 'Actual wire command failed: '+str(path))
    rows, diagnostics = events(path)
    expected = {'stats': 'world_stats', 'snapshot': 'snapshot', 'entity': 'entity',
                'world': 'world', 'command': 'snapshot', 'control': 'control_sent'}[mode]
    selected = [row for row in rows if row.get('source') == expected]
    require(len(selected) == 1, 'Expected one '+expected+' result: '+str(path))
    result = selected[0]
    result['_wire_artifact'], result['_wire_sha256'], result['_wire_diagnostics'] = str(path), sha(path), diagnostics
    return result


def stamp(message, field):
    value = first(message, field)
    require(isinstance(value, dict), 'Native source timestamp absent: '+field)
    return int(first(value, 'sec', '0'))*1000000000+int(first(value, 'nsec', '0'))


def statistics(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native pause state absent')
    return {'sim_time_ns': stamp(native, 'sim_time'), 'iterations': int(first(native, 'iterations')),
            'paused': first(native, 'paused') == 'true', 'raw_wire': row}


def snapshot(row):
    require(row['request_id'] == row['response_id'] and row['payload'].get('query_nonce'), 'RPC correlation absent')
    state = row['payload']
    require(state['world'] == WORLD and not state['last_error'], 'Wrong native world or plugin error: '+repr(state))
    return state


def scene(row):
    native = parsed_model(row['raw_protobuf'])
    result = {}
    for model in native.get('model', []):
        name = str(first(model, 'name')).split('::')[-1]
        require(name not in result, 'Native Scene repeats a model identity')
        result[name] = model
    require(SENTINEL in result, 'Native Scene lacks the external sentinel')
    return result


def position(model):
    pose = first(model, 'pose')
    require(isinstance(pose, dict), 'Actual native pose absent')
    xyz = first(pose, 'position')
    require(isinstance(xyz, dict), 'Actual native position absent')
    result = [float(first(xyz, axis, '0')) for axis in ('x', 'y', 'z')]
    require(all(math.isfinite(value) for value in result), 'Nonfinite native position')
    return result


def entity(row, name):
    native = parsed_model(row['raw_protobuf'])
    require(str(first(native, 'name')).split('::')[-1] == name, 'Actual entity query returned the wrong model')
    require(int(first(native, 'id')) > 0, 'Actual native entity ID absent')
    return native


def actual_models(session, artifacts, label):
    return scene(wire(session, artifacts, label, 'world'))


def paused(session, artifacts, label):
    session.pause(True)
    value = statistics(wire(session, artifacts, label, 'stats'))
    require(value['paused'], 'Actual world was not paused')
    return value


def wait_time(session, artifacts, label, target_ns, timeout=12):
    deadline, index = time.monotonic()+timeout, 0
    while True:
        row = statistics(wire(session, artifacts, label+'-'+str(index), 'stats'))
        if row['sim_time_ns'] >= target_ns:
            return row
        require(time.monotonic() < deadline, 'Native source clock did not reach the bounded target')
        index += 1


def wait_idle(session, artifacts, label):
    deadline, polls = time.monotonic()+12, []
    while True:
        row = snapshot(wire(session, artifacts, label+'-'+str(len(polls)), 'snapshot'))
        polls.append(row)
        if row['phase'] == 'idle' and not row['models']:
            return polls
        require(time.monotonic() < deadline, 'Actual world-plugin cleanup did not complete')


def mapped_library(session, artifacts, name):
    record = json.loads((session.workspace/'build/world_lab.json').read_text(encoding='utf-8'))
    path = (session.workspace/'build/lib/libgazeboteach_l27_world.so').resolve()
    require(sha(path) == record['library_sha256'], 'Prepared WorldPlugin ELF differs')
    mappings = []
    for pid in live_token_processes(session.token):
        proc = Path('/proc', str(pid))
        try:
            if (proc/'comm').read_text().strip() != 'gzserver':
                continue
            lines = []
            for line in (proc/'maps').read_text().splitlines():
                fields = line.split(None, 5)
                if len(fields) == 6 and Path(fields[5]).resolve() == path:
                    info = path.stat()
                    major, minor = [int(value, 16) for value in fields[3].split(':')]
                    require(int(fields[4]) == info.st_ino and (major, minor) == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'WorldPlugin mapping inode/device differs from the prepared binary')
                    lines.append(line)
            if lines:
                target = artifacts/(name+'-plugin.maps')
                target.write_text('\n'.join(lines)+'\n', encoding='utf-8')
                mappings.append({'pid': pid, 'path': str(path), 'library_sha256': sha(path),
                                 'maps': str(target), 'maps_sha256': sha(target)})
        except OSError:
            continue
    require(len(mappings) == 1, 'Expected one owned native server mapping the exact WorldPlugin')
    return mappings[0]


def fall_samples(records, name, native_id):
    result, duplicates = [], 0
    for row in records:
        if row.get('source') != 'pose':
            continue
        packet = parsed_model(row['raw_protobuf'])
        time_ns = stamp(packet, 'time')
        found = [p for p in packet.get('pose', []) if int(first(p, 'id', '-1')) == native_id]
        if not found:
            continue
        require(len(found) == 1 and str(first(found[0], 'name')).split('::')[-1] == name,
                'Native model pose ID/name correspondence differs')
        if result:
            require(time_ns >= result[-1]['time_ns'], 'Native source time regressed without reset')
            if time_ns == result[-1]['time_ns']:
                duplicates += 1
                continue
        result.append({'time_ns': time_ns, 'position_m': position({'pose': [found[0]]}),
                       'receive_monotonic_ns': row['receive_monotonic_ns']})
    require(len(result) >= 8, 'Too few independent native falling model poses: '+name)
    return result, duplicates


def analyze_fall(records, name, model, config):
    samples, duplicates = fall_samples(records, name, int(first(model, 'id')))
    heights = [row['position_m'][2] for row in samples]
    require(abs(max(heights)-config['drop_height']) < .08, 'Native model did not begin near the requested physical drop height')
    require(max(heights)-min(heights) > .3, 'No actual native falling displacement')
    require(abs(position(model)[2]-(GROUND_TOP+SIDE/2)) < .012, 'Native cube did not settle on the actual ground')
    air = [row for row in samples if row['position_m'][2] > GROUND_TOP+SIDE/2+.08]
    # Three wide source-time intervals determine acceleration without assuming
    # that request time equals insertion time or that first observed v is zero.
    selected = []
    for row in air:
        if not selected or row['time_ns']-selected[-1]['time_ns'] >= 50000000:
            selected.append(row)
    accelerations = []
    for a, b, c in zip(selected, selected[1:], selected[2:]):
        dt1, dt2 = (b['time_ns']-a['time_ns'])/1e9, (c['time_ns']-b['time_ns'])/1e9
        v1 = (b['position_m'][2]-a['position_m'][2])/dt1
        v2 = (c['position_m'][2]-b['position_m'][2])/dt2
        accelerations.append(-2*(v2-v1)/(dt1+dt2))
    require(len(accelerations) >= 2 and all(abs(value-GRAVITY) < 1 for value in accelerations)
            and abs(numeric_statistics.median(accelerations)-GRAVITY) < .35,
            'Native free-fall finite differences do not measure the actual 9.8m/s² gravity: '+repr(accelerations))
    links = model.get('link', [])
    require(len(links) == 1, 'Native inserted cube link count differs')
    inertial = first(links[0], 'inertial')
    require(abs(float(first(inertial, 'mass'))-.5) < 1e-10, 'Actual inserted cube mass differs')
    theoretical_inertia = .5*SIDE*SIDE/6
    # Installed SDFormat 9's Param::GetAsString uses six significant digits.
    # A real SDK microprobe confirms Root::Clone converts the original double
    # through that string; Classic's factory unconditionally takes this Clone
    # path even for InsertModelString. Compare the actual SDK-transmitted value
    # with the SAME 1e-10 tolerance; do not rewrite the measured native message
    # or silently treat the quantized inertia as the exact geometric theory.
    sdk_factory_expected = float(format(theoretical_inertia, '.6g'))
    actual_inertias = {field: float(first(inertial, field)) for field in ('ixx', 'iyy', 'izz')}
    require(all(abs(value-sdk_factory_expected) < 1e-10 for value in actual_inertias.values()),
            'Actual inserted cube inertia differs from the installed SDK factory serialization')
    inertia_comparison = {'theoretical_cube_diagonal_kg_m2': theoretical_inertia,
                          'sdk_factory_expected_diagonal_kg_m2': sdk_factory_expected,
                          'native_actual_per_axis_kg_m2': actual_inertias,
                          'comparison_absolute_tolerance_kg_m2': 1e-10,
                          'sdk_copy_significant_digits': 6,
                          'theory_absolute_error_per_axis_kg_m2': {
                              field: abs(value-theoretical_inertia) for field, value in actual_inertias.items()},
                          'theory_relative_error_per_axis': {
                              field: abs(value-theoretical_inertia)/theoretical_inertia
                              for field, value in actual_inertias.items()},
                          'basis': 'Actual installed SDFormat9 direct-read/GetAsString/Root::Clone microprobe; '
                                   'native protobuf values remain unchanged. Translational free-fall is checked independently.'}
    return {'name': name, 'native_id': int(first(model, 'id')), 'native_samples': samples,
            'duplicate_timestamps_retained_in_raw': duplicates, 'maximum_native_z_m': max(heights),
            'minimum_native_z_m': min(heights), 'settled_native_position_m': position(model),
            'independent_downward_accelerations_m_s2': accelerations,
            'median_downward_acceleration_m_s2': numeric_statistics.median(accelerations),
            'inertia_serialization_comparison': inertia_comparison}


def physical_batch(session, artifacts, label, config, prefix):
    initial_world = paused(session, artifacts, label+'-paused')
    before = actual_models(session, artifacts, label+'-scene-before')
    require(not any(name.startswith(prefix) for name in before), 'Fresh batch already contains private entities')
    observed = artifacts/(label+'-native-motion.jsonl')
    command = shlex.join([str(session.workspace/'build/world_observer'), 'watch', '--seconds', '30'])
    process = session._spawn('Independent native factory/pose wire',
                             ['bash', '--noprofile', '--norc', '-c', command+' > '+shlex.quote(str(observed))])
    try:
        deadline = time.monotonic()+5
        while True:
            rows, _ = events(observed, live=True)
            if any(row.get('source') == 'client_ready' for row in rows) and any(row.get('source') == 'world_stats' for row in rows):
                break
            require(process.poll() is None and time.monotonic() < deadline, 'Native wire watcher did not connect')
            time.sleep(.04)
        accepted = snapshot(wire(session, artifacts, label+'-batch-request', 'command', '--action', 'batch'))
        require(accepted['accepted'], 'Single batch request was rejected')
        session.pause(False)
        # Queue processing is wall-clock asynchronous. Permit it to complete,
        # then independently verify all real native entities and their fall.
        target = initial_world['sim_time_ns']+int(((config['count']-1)*.35+
                    math.sqrt(2*(config['drop_height']-.18)/GRAVITY)+1.3)*1e9)
        wait_time(session, artifacts, label+'-running', target, timeout=20)
        session.pause(True)
        state = snapshot(wire(session, artifacts, label+'-plugin-after', 'snapshot'))
        native_scene = actual_models(session, artifacts, label+'-scene-after')
    finally:
        if process.poll() is None:
            terminate_owned(process)
    rows, diagnostics = events(observed, live=True)
    names = sorted(name for name in native_scene if name.startswith(prefix))
    require(len(names) == config['count'], 'Actual native Scene inserted count differs from the exercise')
    require(state['phase'] == 'complete' and state['count'] == config['count']
            and state['requested'] == state['observed'] == config['count'], 'Plugin completion instrumentation differs')
    require({item['name'] for item in state['models']} == set(names), 'Plugin names differ from actual native Scene')
    actual_entities = {name: entity(wire(session, artifacts, label+'-'+name+'-entity', 'entity', '--name', name), name)
                       for name in names}
    motion = [analyze_fall(rows, name, actual_entities[name], config) for name in names]
    # One sample near the support could be a cube merely passing through it.
    # Observe two additional real 100-tick intervals and query actual entities
    # while paused, retaining both clocks and all positions as stability data.
    stable = [{'world': statistics(wire(session, artifacts, label+'-stable-0-clock', 'stats')),
               'positions_m': {name: position(model) for name, model in actual_entities.items()}}]
    for index in (1, 2):
        session.step(100)
        current_clock = statistics(wire(session, artifacts, label+'-stable-'+str(index)+'-clock', 'stats'))
        require(current_clock['paused'] and current_clock['sim_time_ns']-stable[-1]['world']['sim_time_ns'] == 100000000,
                'A support-stability interval did not advance exactly 100 actual ticks')
        current_entities = {name: entity(wire(session, artifacts, label+'-stable-'+str(index)+'-'+name, 'entity', '--name', name), name)
                            for name in names}
        stable.append({'world': current_clock, 'positions_m': {name: position(model) for name, model in current_entities.items()}})
    for name in names:
        zs = [row['positions_m'][name][2] for row in stable]
        require(all(abs(z-(GROUND_TOP+SIDE/2)) < .012 for z in zs)
                and max(abs(b-a)/.1 for a, b in zip(zs, zs[1:])) < .01,
                'Native cube is only passing near support, not stable across actual source-time intervals')
    require(before[SENTINEL] == native_scene[SENTINEL], 'Factory modified the external sentinel')
    return {'passed': True, 'parameters': config, 'native_names': names, 'native_entities': actual_entities,
            'native_falls': motion, 'plugin_after': state, 'raw_jsonl': str(observed),
            'raw_jsonl_sha256': sha(observed), 'wire_diagnostics': diagnostics,
            'native_support_stability': stable, 'support_stability_velocity_limit_m_s': .01}


def saved_input_guard(session, artifacts):
    before = statistics(wire(session, artifacts, 'saved-input-before', 'stats'))
    state_before = snapshot(wire(session, artifacts, 'saved-input-plugin-before', 'snapshot'))
    path = session.workspace/'parameters.json'
    original, original_hashes = path.read_bytes(), hashes(session.workspace)
    try:
        value = json.loads(original.decode('utf-8'))
        value['count'] += 1
        save(path, value)
        code = run(session, [sys.executable, session.workspace/'lab.py', 'ready'], artifacts/'saved-without-reload.log')
        require(code != 0, 'Saved but not reloaded count was accepted')
    finally:
        path.write_bytes(original)
    after = statistics(wire(session, artifacts, 'saved-input-after', 'stats'))
    state_after = snapshot(wire(session, artifacts, 'saved-input-plugin-after', 'snapshot'))
    require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations']
            and state_before['updates'] == state_after['updates'], 'Rejected stale input advanced the paused experiment')
    require(hashes(session.workspace) == original_hashes, 'Stale-input check changed other personal source bytes')
    code = run(session, [sys.executable, session.workspace/'lab.py', 'ready'], artifacts/'exact-input-repaired.log')
    require(code == 0, 'Exact original parameter bytes did not repair readiness')
    return {'passed': True, 'before': before, 'after': after, 'exact_input_bytes_restored': True}


def extended_checks(session, artifacts, prefix):
    result = {'saved_input_guard': saved_input_guard(session, artifacts)}
    before = statistics(wire(session, artifacts, 'pause-before', 'stats'))
    plugin_before = snapshot(wire(session, artifacts, 'pause-plugin-before', 'snapshot'))
    time.sleep(.2)
    after = statistics(wire(session, artifacts, 'pause-after', 'stats'))
    plugin_after = snapshot(wire(session, artifacts, 'pause-plugin-after', 'snapshot'))
    require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations']
            and plugin_before['updates'] == plugin_after['updates'], 'Paused world advanced physics or callbacks')
    session.step(100)
    stepped = statistics(wire(session, artifacts, '100-steps-after', 'stats'))
    plugin_stepped = snapshot(wire(session, artifacts, '100-steps-plugin', 'snapshot'))
    require(stepped['paused'] and stepped['iterations']-after['iterations'] == 100
            and stepped['sim_time_ns']-after['sim_time_ns'] == 100000000
            and plugin_stepped['updates']-plugin_after['updates'] == 100, '100 native ticks/callbacks did not advance exactly .1s')
    result['pause_and_100_ticks'] = {'passed': True, 'before': before, 'paused_after': after, 'stepped': stepped,
                                    'plugin_before': plugin_after, 'plugin_stepped': plugin_stepped}
    models_before = actual_models(session, artifacts, 'model-reset-scene-before')
    wire(session, artifacts, 'model-reset-once', 'control', '--reset-models')
    deadline, polls = time.monotonic()+5, []
    while True:
        value = statistics(wire(session, artifacts, 'model-reset-stats-'+str(len(polls)), 'stats'))
        polls.append(value)
        if value['iterations'] >= stepped['iterations']+1:
            break
        require(time.monotonic() < deadline, 'Model-only reset management tick was not observed')
    model_plugin = snapshot(wire(session, artifacts, 'model-reset-plugin', 'snapshot'))
    models_after = actual_models(session, artifacts, 'model-reset-scene-after')
    require(value['paused'] and value['iterations'] == stepped['iterations']+1
            and value['sim_time_ns']-stepped['sim_time_ns'] == 1000000, 'Native model reset rewound time or advanced unexpected ticks')
    require(model_plugin['reset_count'] == plugin_stepped['reset_count']
            and model_plugin['updates'] == plugin_stepped['updates'], 'Model-only reset unexpectedly invoked WorldPlugin Reset/update')
    require({name: first(m, 'id') for name, m in models_before.items()} ==
            {name: first(m, 'id') for name, m in models_after.items()}, 'Model-only reset inserted/deleted actual native models')
    result['model_only_reset'] = {'passed': True, 'world_before': stepped, 'world_after': value,
                                   'plugin_before': plugin_stepped, 'plugin_after': model_plugin}
    wire(session, artifacts, 'full-reset-once', 'control', '--reset-all')
    deadline, full_polls = time.monotonic()+5, []
    while True:
        current = snapshot(wire(session, artifacts, 'full-reset-plugin-'+str(len(full_polls)), 'snapshot'))
        full_polls.append(current)
        if current['reset_count'] == model_plugin['reset_count']+1:
            break
        require(time.monotonic() < deadline, 'Full world reset did not invoke WorldPlugin::Reset')
    reset_world = statistics(wire(session, artifacts, 'full-reset-clock', 'stats'))
    require(reset_world['paused'] and reset_world['sim_time_ns'] <= 1000000
            and reset_world['sim_time_ns'] < value['sim_time_ns'], 'Full reset did not rewind the actual world clock')
    require(current['load_count'] == model_plugin['load_count'] == 1
            and current['init_count'] == model_plugin['init_count'] == 1 and current['epoch_updates'] == 0,
            'Full reset reloaded the plugin or falsely counted an ordinary callback')
    session.pause(False)
    clean_polls = wait_idle(session, artifacts, 'full-reset-deferred-clean')
    paused(session, artifacts, 'full-reset-clean-paused')
    clean_scene = actual_models(session, artifacts, 'full-reset-clean-scene')
    require(not any(name.startswith(prefix) for name in clean_scene), 'Full-reset deferred cleanup left private models')
    require(models_before[SENTINEL] == clean_scene[SENTINEL], 'Full-reset cleanup modified the external static sentinel')
    result['full_world_reset'] = {'passed': True, 'world_before': value, 'world_after_reset': reset_world,
                                  'plugin_after_reset': current, 'cleanup_polls': clean_polls}
    # Pause after exactly the first queued factory tick. Its item has not yet
    # passed a second ordinary plugin callback, even if World already inserted
    # it asynchronously. Clear must wait for actual identity and reclamation.
    accepted = snapshot(wire(session, artifacts, 'inflight-batch-once', 'command', '--action', 'batch'))
    require(accepted['accepted'], 'In-flight batch request rejected')
    session.step(1)
    inflight = snapshot(wire(session, artifacts, 'inflight-before-clear', 'snapshot'))
    require(inflight['requested'] == 1 and inflight['observed'] == 0 and len(inflight['pending_names']) == 1,
            'Exactly one factory item was not pending before clear')
    native_inflight = actual_models(session, artifacts, 'inflight-native-scene')
    accepted_clear = snapshot(wire(session, artifacts, 'inflight-clear-once', 'command', '--action', 'clear'))
    require(accepted_clear['accepted'], 'Single in-flight clear was rejected')
    start = statistics(wire(session, artifacts, 'inflight-clear-clock', 'stats'))
    session.pause(False)
    cleared = wait_idle(session, artifacts, 'inflight-clearing')
    wait_time(session, artifacts, 'inflight-late-factory-guard', start['sim_time_ns']+1000000000)
    paused(session, artifacts, 'inflight-final-paused')
    final_scene = actual_models(session, artifacts, 'inflight-final-scene')
    require(not any(name.startswith(prefix) for name in final_scene), 'A late asynchronous factory escaped scoped clear')
    require(native_inflight[SENTINEL] == final_scene[SENTINEL], 'Scoped clear modified the external sentinel')
    result['inflight_clear_and_external_isolation'] = {'passed': True, 'inflight_plugin': inflight,
        'native_scene_before_clear': native_inflight, 'cleanup_polls': cleared, 'native_scene_after_late_guard': final_scene,
        'late_guard_sim_seconds': 1.0}
    return result


def precision_boundary_reset(session, artifacts, config, prefix):
    """Run the real course tool once, then independently inspect its effects."""
    label = 'precision-boundary-model-reset'
    parameter_file = session.workspace/'parameters.json'
    requested_bytes = parameter_file.read_bytes()
    requested = json.loads(requested_bytes.decode('utf-8'))
    require(requested == config, 'Boundary regression did not retain the original requested JSON values')
    requested_height = requested['drop_height']
    sdk_expected = float(format(requested_height, '.6g'))
    require(abs(sdk_expected-requested_height) > 1e-7, 'Boundary input does not distinguish SDK precision from the original request')
    original_path = artifacts/'precision-boundary-requested-parameters.json'
    original_path.write_bytes(requested_bytes)
    before_world = paused(session, artifacts, label+'-before-world')
    before_plugin = snapshot(wire(session, artifacts, label+'-before-plugin', 'snapshot'))
    before_scene = actual_models(session, artifacts, label+'-before-scene')
    before_ids = {name: int(first(model, 'id')) for name, model in before_scene.items()}
    log = artifacts/(label+'-tool.log')
    code = run(session, [sys.executable, session.workspace/'lab.py', 'model-reset'], log, timeout=25)
    require(code == 0, 'Course model-reset rejected the actual high-precision SDK input; see '+str(log))
    # None of these assertions uses the course tool's returned data as truth.
    after_world = statistics(wire(session, artifacts, label+'-after-world', 'stats'))
    after_plugin = snapshot(wire(session, artifacts, label+'-after-plugin', 'snapshot'))
    after_scene = actual_models(session, artifacts, label+'-after-scene')
    after_ids = {name: int(first(model, 'id')) for name, model in after_scene.items()}
    require(before_ids == after_ids, 'Boundary model reset changed the actual native entity IDs')
    require(after_world['paused'] and after_world['iterations']-before_world['iterations'] == 1
            and after_world['sim_time_ns']-before_world['sim_time_ns'] == 1000000,
            'Boundary model reset did not retain pause and its single actual 1ms management tick')
    require(after_plugin['reset_count'] == before_plugin['reset_count']
            and after_plugin['updates'] == before_plugin['updates']
            and after_plugin['load_count'] == before_plugin['load_count'] == 1
            and after_plugin['init_count'] == before_plugin['init_count'] == 1,
            'Boundary model-only reset unexpectedly invoked a WorldPlugin lifecycle/update callback')
    names = sorted(name for name in after_scene if name.startswith(prefix))
    require(len(names) == 1, 'Boundary regression does not contain the original single actual cube')
    measurements = []
    for name in names:
        model = entity(wire(session, artifacts, label+'-'+name+'-entity', 'entity', '--name', name), name)
        actual_height = position(model)[2]
        require(int(first(model, 'id')) == before_ids[name], 'Boundary entity query changed the actual cube ID')
        require(abs(actual_height-sdk_expected) < 1e-7,
                'Boundary model-reset native height differs from the installed SDK factory expectation')
        measurements.append({'name': name, 'native_id': before_ids[name],
                             'requested_height_m': requested_height, 'sdk_factory_expected_height_m': sdk_expected,
                             'native_actual_height_m': actual_height,
                             'native_minus_requested_m': actual_height-requested_height,
                             'native_minus_sdk_expected_m': actual_height-sdk_expected,
                             'absolute_native_vs_sdk_tolerance_m': 1e-7,
                             'native_entity': model})
    require(parameter_file.read_bytes() == requested_bytes, 'Course model-reset rewrote the original requested JSON bytes')
    require(before_scene[SENTINEL] == after_scene[SENTINEL], 'Boundary model reset changed the external static sentinel')
    return {'passed': True, 'purpose': 'Additional SDK9 precision/count-one regression; not a teaching single-variable comparison',
            'course_tool_log': str(log), 'course_tool_log_sha256': sha(log),
            'world_before': before_world, 'world_after': after_world,
            'plugin_before': before_plugin, 'plugin_after': after_plugin,
            'native_ids_before': before_ids, 'native_ids_after': after_ids,
            'native_height_measurements': measurements,
            'requested_parameters_original_bytes': str(original_path),
            'requested_parameters_sha256': hashlib.sha256(requested_bytes).hexdigest(),
            'requested_parameters_bytes_unchanged': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(item for item in lessons if item.id == '27' and item.profile == 'classic11-noetic')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': False, 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l27-') as temporary:
        try:
            for name, config in [('baseline', {'count': 3, 'drop_height': 1.2}),
                                 ('count_only', {'count': 5, 'drop_height': 1.2}),
                                 ('height_only', {'count': 3, 'drop_height': 1.8}),
                                 ('precision_boundary', {'count': 1, 'drop_height': 1.23456789})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=False, terminals=False)
                mapping = mapped_library(session, artifacts, name)
                record = json.loads((workspace/'build/world_lab.json').read_text(encoding='utf-8'))
                require(record['source_sha256']['world_plugin.cc'] == source['world_plugin.cc'], 'Compiled plugin source differs')
                prefix = record['session_prefix']
                row = physical_batch(session, artifacts, name, config, prefix)
                if name == 'baseline':
                    row.update(extended_checks(session, artifacts, prefix))
                if name == 'precision_boundary':
                    row['scenario_kind'] = 'additional_precision_regression_not_teaching_single_variable'
                    row['course_tool_model_reset_and_native_crosscheck'] = precision_boundary_reset(
                        session, artifacts, config, prefix)
                row['name'], row['actual_world_plugin_mapping'] = name, mapping
                row['prepared_input_sha256'] = hashes(workspace)
                save(artifacts/(name+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/world_lab.json'), str(artifacts/(name+'-world_lab.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(name+'-generated-world.sdf')))
                result['scenarios'].append(row)
                session.stop(reason='L27 independent physical case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            low = [item['maximum_native_z_m'] for item in result['scenarios'][0]['native_falls']]
            high = [item['maximum_native_z_m'] for item in result['scenarios'][2]['native_falls']]
            require(abs((sum(high)/len(high)-sum(low)/len(low))-.6) < .08, 'Changing only drop height did not move actual spawn height')
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason='L27 formal acceptance cleanup')
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
