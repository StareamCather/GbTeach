#!/usr/bin/env python3
"""Independent L28 sensor geometry, source-time filtering and lifecycle checks.

The native LaserScanStamped protobuf text is the distance truth. The custom
Any.STRING is decoded separately and compared with those actual beams, not
accepted as its own oracle. A continuous private observer retains every wire
record while actual model moves produce near, far, missing and recovered echoes.
Source-time recurrence and alarm changes distinguish threshold-only/alpha-only
fresh workspaces. Physical ticks and sensor callback counts are separate facts.
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

WORLD, TARGET = 'lesson28', 'target_box'
SENSOR_SUFFIX = 'scanner::link::proximity'


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
    path = Path(path)
    text = path.read_text(encoding='utf-8') if path.exists() else ''
    result, noise, partial = [], [], None
    lines = text.splitlines()
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            if live and index == len(lines)-1 and not text.endswith('\n'):
                partial = line
            else:
                noise.append(line)
            continue
        require(isinstance(row, dict), 'Native wire record is not an object')
        require(row.get('source') not in ('client_error', 'decode_error'), 'Actual observer failed: '+repr(row))
        result.append(row)
    return result, {'non_json_lines': noise, 'trailing_partial_line': partial}


def run(session, argv, output, timeout=15):
    command = shlex.join([str(v) for v in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L28 command', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    code = run(session, [session.workspace/'build/sensor_observer', mode, *arguments], path)
    require(code == 0, 'Actual native command failed: '+str(path))
    rows, diagnostics = events(path)
    source = {'snapshot': 'snapshot', 'stats': 'world_stats', 'entity': 'entity',
              'world': 'world', 'move': 'move_sent', 'control': 'control_sent'}[mode]
    values = [row for row in rows if row.get('source') == source]
    require(len(values) == 1, 'Expected one native '+source+' result: '+str(path))
    row = values[0]
    row['_wire_artifact'], row['_wire_sha256'], row['_wire_diagnostics'] = str(path), sha(path), diagnostics
    return row


def stamp(message, field):
    part = first(message, field)
    require(isinstance(part, dict), 'Native source stamp absent: '+field)
    return int(first(part, 'sec', '0'))*1000000000+int(first(part, 'nsec', '0'))


def stats(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native pause state absent')
    return {'sim_time_ns': stamp(native, 'sim_time'), 'iterations': int(first(native, 'iterations')),
            'paused': first(native, 'paused') == 'true', 'raw_wire': row}


def plugin(row):
    if row['source'] == 'plugin':
        native = parsed_model(row['raw_protobuf'])
        require(first(native, 'type') == 'STRING', 'Custom native message is not Any.STRING')
        payload = json.loads(first(native, 'string_value'))
        require(payload == row['payload'], 'Observer mirror differs from actual Any bytes')
    else:
        require(row['request_id'] == row['response_id'] and row['payload'].get('query_nonce'),
                'Snapshot native response correlation absent')
        payload = row['payload']
    require(payload['world'] == WORLD and payload['sensor_scoped_name'].endswith(SENSOR_SUFFIX),
            'Custom output comes from another world or sensor')
    require(payload['load_count'] == payload['init_count'] == 1, 'Sensor plugin Load/Init count differs')
    return payload


def xyz(model):
    pose = first(model, 'pose')
    position = first(pose, 'position')
    values = [float(first(position, key, '0')) for key in ('x', 'y', 'z')]
    require(all(math.isfinite(value) for value in values), 'Native model position is nonfinite')
    return values


def native_entity(row, name=TARGET):
    native = parsed_model(row['raw_protobuf'])
    require(str(first(native, 'name')).split('::')[-1] == name, 'Wrong native model reply')
    require(int(first(native, 'id')) > 0, 'Native entity ID absent')
    return native


def quiet(session, artifacts, label):
    session.pause(True)
    deadline, index = time.monotonic()+4, 0
    previous = plugin(wire(session, artifacts, label+'-plugin-0', 'snapshot'))
    while True:
        time.sleep(.12)
        current = plugin(wire(session, artifacts, label+'-plugin-'+str(index+1), 'snapshot'))
        if current['callback_count'] == previous['callback_count']:
            state = stats(wire(session, artifacts, label+'-world', 'stats'))
            require(state['paused'], 'Actual world is not paused')
            return state, current
        require(time.monotonic() < deadline, 'Already-due sensor work did not drain while paused')
        previous, index = current, index+1


def move(session, artifacts, label, x):
    before = stats(wire(session, artifacts, label+'-clock-before', 'stats'))
    require(before['paused'], 'Move experiment requires a paused world')
    wire(session, artifacts, label+'-request', 'move', '--name', TARGET, '--x', str(x))
    deadline, attempt = time.monotonic()+5, 0
    while True:
        actual = native_entity(wire(session, artifacts, label+'-native-'+str(attempt), 'entity', '--name', TARGET))
        position = xyz(actual)
        if max(abs(a-b) for a, b in zip(position, (x, 0, .4))) < 1e-7:
            after = stats(wire(session, artifacts, label+'-clock-after', 'stats'))
            require(before['sim_time_ns'] == after['sim_time_ns'] and after['paused'], 'Moving static target advanced paused time')
            return {'actual_entity': actual, 'world': after, 'position_m': position}
        require(time.monotonic() < deadline, 'Actual target move did not complete; request not replayed')
        attempt += 1


def wait_time(session, artifacts, label, target, timeout=8):
    deadline, index = time.monotonic()+timeout, 0
    while True:
        current = stats(wire(session, artifacts, label+'-'+str(index), 'stats'))
        if current['sim_time_ns'] >= target:
            return current
        require(time.monotonic() < deadline, 'Native world did not advance to the bounded observation target')
        index += 1


def watcher(session, artifacts, label):
    path = artifacts/(label+'.jsonl')
    stop = artifacts/(label+'.stop')
    if stop.exists():
        stop.unlink()
    command = shlex.join([str(session.workspace/'build/sensor_observer'), 'watch', '--seconds', '90', '--stop-file', str(stop)])
    process = session._spawn('Independent L28 native observer',
                             ['bash', '--noprofile', '--norc', '-c', command+' > '+shlex.quote(str(path))])
    deadline = time.monotonic()+5
    while True:
        rows, _ = events(path, live=True)
        if any(row.get('source') == 'client_ready' for row in rows) and any(row.get('source') == 'world_stats' for row in rows):
            return process, path, stop
        require(process.poll() is None and time.monotonic() < deadline, 'Native wire observer did not connect')
        time.sleep(.04)


def stop_watcher(process, stop):
    if process.poll() is None:
        stop.write_text('finished independent source-time observations\n', encoding='utf-8')
        try:
            process.wait(timeout=7)
        except BaseException:
            terminate_owned(process)
            raise
    require(process.returncode == 0, 'Native watcher did not complete its real stop/join path')


def scan(row):
    native = parsed_model(row['raw_protobuf'])
    data = first(native, 'scan')
    require(isinstance(data, dict), 'Native LaserScanStamped has no scan')
    ranges = [float(value) for value in data.get('ranges', [])]
    require(len(ranges) == int(first(data, 'count')) == 3, 'Native ray sample count differs')
    low, high = float(first(data, 'range_min')), float(first(data, 'range_max'))
    angle, increment = float(first(data, 'angle_min')), float(first(data, 'angle_step'))
    require(abs(low-.1) < 1e-12 and abs(high-6) < 1e-12 and abs(angle+.1) < 1e-12
            and abs(increment-.1) < 1e-12 and abs(float(first(data, 'angle_max'))-.1) < 1e-12,
            'Actual native range or angular metadata differs')
    sensor_position = xyz({'pose': [first(data, 'world_pose')]})
    require(max(abs(a-b) for a, b in zip(sensor_position, (0, 0, .4))) < 1e-7,
            'Actual native scanner world pose differs from the geometric reference')
    return {'stamp_ns': stamp(native, 'time'), 'ranges': ranges, 'minimum': low, 'maximum': high,
            'angles': [angle+i*increment for i in range(3)], 'sensor_position_m': sensor_position,
            'frame': first(data, 'frame'), 'raw_wire': row}


def finite_ranges(value):
    return [v for v in value['ranges'] if math.isfinite(v) and value['minimum'] <= v < value['maximum']]


def geometry(value, target_x):
    face = target_x-.2
    expected, errors = [], []
    for angle, actual in zip(value['angles'], value['ranges']):
        distance = face/math.cos(angle)
        hit = distance < 6 and abs(face*math.tan(angle)) <= .2
        expected.append(distance if hit else None)
        if hit:
            require(math.isfinite(actual) and abs(actual-distance) < .015, 'Native ray disagrees with independent cube-face geometry')
            errors.append(abs(actual-distance))
        else:
            require(not math.isfinite(actual) or actual >= value['maximum'], 'A geometric miss has an unexpected finite return')
    return {'expected_ranges_m': expected, 'maximum_finite_geometry_error_m': max(errors) if errors else None}


def safe_ranges(values):
    return [value if math.isfinite(value) else ('+inf' if value > 0 else '-inf' if value < 0 else 'nan') for value in values]


def paired(records, low_ns, high_ns):
    scans, custom, duplicates = {}, [], {'scan': 0, 'plugin': 0}
    previous_scan, previous_plugin = None, None
    for row in records:
        if row.get('source') == 'scan':
            value = scan(row)
            if not low_ns < value['stamp_ns'] <= high_ns:
                continue
            if previous_scan is not None:
                require(value['stamp_ns'] >= previous_scan['stamp_ns'], 'Native scan time regressed before reset')
            if value['stamp_ns'] in scans:
                old = scans[value['stamp_ns']]
                require(safe_ranges(old['ranges']) == safe_ranges(value['ranges']), 'Same native timestamp carries different scan values')
                duplicates['scan'] += 1
            else:
                scans[value['stamp_ns']] = value
                previous_scan = value
        elif row.get('source') == 'plugin':
            value = plugin(row)
            t = int(value['source_time_ns'])
            if not low_ns < t <= high_ns:
                continue
            if previous_plugin is not None:
                require(t >= int(previous_plugin['source_time_ns']), 'Custom source stamp regressed before reset')
                if value['callback_count'] == previous_plugin['callback_count']:
                    require(value == previous_plugin, 'Repeated callback identity carries different custom values')
                    duplicates['plugin'] += 1
                    continue
                require(value['callback_count'] == previous_plugin['callback_count']+1, 'Custom callback gap prevents independent EMA reconstruction')
            custom.append(value)
            previous_plugin = value
    require(len(custom) >= 12, 'Too few actual paired SensorPlugin callbacks')
    output = []
    for value in custom:
        t = int(value['source_time_ns'])
        require(t in scans, 'Custom callback has no independently received native scan at its source time')
        output.append((scans[t], value))
    return output, duplicates


def analyze(records, phases, config):
    pairs, duplicates = paired(records, phases[0]['start_ns'], phases[-1]['end_ns'])
    state, seed_checked, checks = None, False, []
    previous_count = None
    for measured, value in pairs:
        t, ranges = measured['stamp_ns'], measured['ranges']
        valid = finite_ranges(measured)
        nearest = min(valid) if valid else None
        if not seed_checked:
            require(nearest is not None and abs(nearest-1.2) < .015, 'First independent filter seed is not the stable baseline geometry')
            # All pre-observation baseline inputs are constant. The first
            # visible output must independently agree with that fixed point.
            state = nearest
            seed_checked = True
            require(abs(value['filtered_m']-state) < 1e-8, 'Initial observed EMA disagrees with independent constant-input fixed point')
        else:
            prior = state
            if nearest is None:
                state = None
            else:
                state = nearest if state is None else config['alpha']*nearest+(1-config['alpha'])*state
            if nearest is not None:
                require(value['previous_filtered_m'] is None if prior is None else
                        abs(value['previous_filtered_m']-prior) < 1e-8,
                        'Custom previous filter state differs from independent recurrence')
            else:
                require(value['previous_filtered_m'] is None, 'Missing echo did not clear previous filter state')
            require(value['filter_initialized_from_current'] == (nearest is not None and prior is None),
                    'Filter reseeding flag differs from native-input state transition')
        require(value['sample_available'] and abs(value['source_time_s']-t/1e9) < 1e-8,
                'Custom sample availability or source-time units differ')
        require(value['valid_count'] == len(valid), 'Custom valid count differs from actual native beams')
        require((value['nearest_m'] is None) == (nearest is None), 'Custom nearest validity differs')
        require(nearest is None or abs(value['nearest_m']-nearest) < 1e-9, 'Custom nearest differs from actual native beams')
        require((value['filtered_m'] is None) == (state is None), 'Custom filter availability differs')
        require(state is None or abs(value['filtered_m']-state) < 1e-8, 'EMA differs from independent native-input recurrence')
        expected_alarm = 'unknown' if state is None else 'near' if state <= config['threshold_m'] else 'clear'
        require(value['alarm_state'] == expected_alarm, 'Custom alarm differs from independently reconstructed filtered distance')
        require(abs(value['alpha']-config['alpha']) < 1e-12 and abs(value['threshold_m']-config['threshold_m']) < 1e-12,
                'Actual cached SensorPlugin parameters differ')
        require(len(value['raw_ranges_m']) == len(ranges) == len(value['raw_kinds']), 'Custom raw array length differs')
        for native_value, mirror, kind in zip(ranges, value['raw_ranges_m'], value['raw_kinds']):
            if math.isfinite(native_value):
                require(mirror is not None and abs(mirror-native_value) < 1e-9 and kind == 'finite', 'Custom raw finite mirror differs')
            else:
                expected_kind = 'nan' if math.isnan(native_value) else 'positive_infinity' if native_value > 0 else 'negative_infinity'
                require(mirror is None and kind == expected_kind, 'Custom nonfinite classification differs')
        checks.append({'source_time_ns': t, 'callback_count': value['callback_count'], 'native_ranges_m': safe_ranges(ranges),
                       'native_valid_count': len(valid), 'native_nearest_m': nearest, 'independent_filtered_m': state,
                       'actual_filtered_m': value['filtered_m'], 'alarm_state': value['alarm_state']})
        previous_count = value['callback_count']
    phase_results = []
    for phase in phases:
        selected = [(measured, value) for measured, value in pairs if phase['start_ns'] < measured['stamp_ns'] <= phase['end_ns']]
        require(len(selected) >= 12, 'Too few source-time samples for '+phase['name'])
        times = [measured['stamp_ns'] for measured, _ in selected]
        require(all(b > a for a, b in zip(times, times[1:])), 'Phase source timestamps are not strictly increasing')
        span = (times[-1]-times[0])/1e9
        rate = (len(times)-1)/span
        require(span >= .55 and 18 <= rate <= 22, 'Actual native sensor cadence differs from the 20Hz observation target')
        geometries = [geometry(measured, phase['target_x']) for measured, _ in selected]
        valid_counts = [len(finite_ranges(measured)) for measured, _ in selected]
        expected_valid = {'baseline': 3, 'near': 3, 'far': 1, 'missing': 0, 'recovered': 3}[phase['name']]
        require(all(count == expected_valid for count in valid_counts), 'Native validity pattern differs for '+phase['name'])
        last_value = selected[-1][1]
        if phase['name'] == 'missing':
            require(last_value['filtered_m'] is None and last_value['alarm_state'] == 'unknown', 'Missing return was treated as safe distance')
        else:
            expected_min = phase['target_x']-.2
            require(abs(last_value['filtered_m']-expected_min) < .012, 'Real filter did not approach actual stable geometry')
        phase_results.append({'name': phase['name'], 'target_x': phase['target_x'], 'messages': len(selected),
                              'source_span_s': span, 'source_rate_hz': rate, 'native_valid_count': expected_valid,
                              'geometry': geometries, 'first_filtered_m': selected[0][1]['filtered_m'],
                              'first_alarm': selected[0][1]['alarm_state'], 'last_filtered_m': last_value['filtered_m'],
                              'last_alarm': last_value['alarm_state'], 'native_move': phase['move']})
    return {'passed': True, 'paired_callbacks': len(pairs), 'duplicates_retained_in_raw': duplicates,
            'independent_recurrence': checks, 'phases': phase_results,
            'seed_policy': 'First constant-baseline output independently checked against actual native closest distance; all later steps reconstructed from native inputs.'}


def mapped_library(session, artifacts, label):
    record = json.loads((session.workspace/'build/sensor_lab.json').read_text(encoding='utf-8'))
    path = (session.workspace/'build/lib/libgazeboteach_l28_sensor.so').resolve()
    require(sha(path) == record['library_sha256'], 'Prepared SensorPlugin ELF differs')
    found = []
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
                    major, minor = [int(part, 16) for part in fields[3].split(':')]
                    require(int(fields[4]) == info.st_ino and (major, minor) == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'Mapped SensorPlugin inode/device differs')
                    lines.append(line)
            if lines:
                target = artifacts/(label+'-plugin.maps')
                target.write_text('\n'.join(lines)+'\n', encoding='utf-8')
                found.append({'pid': pid, 'path': str(path), 'library_sha256': sha(path),
                              'maps': str(target), 'maps_sha256': sha(target)})
        except OSError:
            continue
    require(len(found) == 1, 'Expected one owned gzserver mapping this exact SensorPlugin')
    return found[0]


def main_motion(session, artifacts, label, config):
    quiet(session, artifacts, label+'-initial-quiet')
    process, path, stop = watcher(session, artifacts, label+'-native-signal')
    phases = []
    try:
        for phase, x in [('baseline', 1.4), ('near', .8), ('far', 3.2), ('missing', 8), ('recovered', 1.4)]:
            quiet(session, artifacts, label+'-'+phase+'-before')
            moved = move(session, artifacts, label+'-'+phase+'-move', x)
            start = moved['world']['sim_time_ns']
            session.pause(False)
            wait_time(session, artifacts, label+'-'+phase+'-observe', start+1000000000)
            end, actual = quiet(session, artifacts, label+'-'+phase+'-end')
            phases.append({'name': phase, 'target_x': x, 'move': moved, 'start_ns': start,
                           'end_ns': end['sim_time_ns'], 'actual_plugin_at_end': actual})
        # Receive already-published bytes after the final actual source stamp.
        deadline = time.monotonic()+3
        while True:
            rows, _ = events(path, live=True)
            latest = [int(plugin(row)['source_time_ns']) for row in rows if row.get('source') == 'plugin']
            if latest and max(latest) >= int(phases[-1]['actual_plugin_at_end']['source_time_ns']):
                break
            require(time.monotonic() < deadline, 'Final source-time sensor bytes not received before deadline')
            time.sleep(.04)
    finally:
        stop_watcher(process, stop)
    rows, diagnostics = events(path, live=True)
    result = analyze(rows, phases, config)
    result.update(raw_jsonl=str(path), raw_jsonl_sha256=sha(path), wire_diagnostics=diagnostics,
                  actual_phase_boundaries=phases,
                  observer_end={'returncode': process.poll(), 'stop_file': str(stop),
                                'policy': 'Owned stop-file requested normal observer exit and explicit SDK IO stop/join after all source-time windows; every raw byte is retained.'})
    return result


def rejected_saved_input(session, artifacts, config):
    before, state_before = quiet(session, artifacts, 'saved-input-before')
    path = session.workspace/'parameters.json'
    original = path.read_bytes()
    original_path = artifacts/'saved-input-original.json'
    original_path.write_bytes(original)
    changed = dict(config, threshold_m=config['threshold_m']+.05)
    log = artifacts/'saved-without-reload.log'
    try:
        save(path, changed)
        code = run(session, ['python3', 'lab.py', 'ready'], log)
        require(code != 0, 'Saved but unprepared sensor input was silently accepted')
        message = log.read_text(encoding='utf-8').lower()
        require('reload' in message or 'restart' in message, 'Stale input was rejected without a reload diagnosis')
        after = stats(wire(session, artifacts, 'saved-input-after-world', 'stats'))
        state_after = plugin(wire(session, artifacts, 'saved-input-after-plugin', 'snapshot'))
        require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations']
                and state_before['callback_count'] == state_after['callback_count'], 'Rejected input advanced native simulation or sensor callbacks')
    finally:
        path.write_bytes(original)
    repaired = artifacts/'saved-exact-repair.log'
    require(run(session, ['python3', 'lab.py', 'ready'], repaired) == 0, 'Exact original sensor input did not restore readiness')
    require(path.read_bytes() == original, 'Student parameter bytes were not preserved')
    return {'passed': True, 'before': before, 'after': after, 'negative_log': str(log),
            'original_bytes': str(original_path), 'original_sha256': hashlib.sha256(original).hexdigest(),
            'exact_input_bytes_restored': True}


def pause_and_ticks(session, artifacts):
    before, state_before = quiet(session, artifacts, 'paused-before')
    time.sleep(.25)
    still = stats(wire(session, artifacts, 'paused-still-world', 'stats'))
    state_still = plugin(wire(session, artifacts, 'paused-still-plugin', 'snapshot'))
    require(still['sim_time_ns'] == before['sim_time_ns'] and still['iterations'] == before['iterations']
            and state_still['callback_count'] == state_before['callback_count'], 'Quiescent paused world or sensor continued advancing')
    session.step(100)
    after, state_after = quiet(session, artifacts, 'stepped-after')
    require(after['paused'] and after['sim_time_ns']-before['sim_time_ns'] == 100000000
            and after['iterations']-before['iterations'] == 100, 'Single native 100-step publication did not advance exactly .1s/100 ticks')
    require(state_after['callback_count'] > state_before['callback_count'], 'Real tick advancement did not produce any actual sensor callback')
    require(int(state_after['source_time_ns']) <= after['sim_time_ns'], 'Sensor claims a future source measurement')
    return {'passed': True, 'before': before, 'paused_after': still, 'after': after,
            'plugin_before': state_before, 'plugin_after': state_after,
            'observed_sensor_callback_delta': state_after['callback_count']-state_before['callback_count'],
            'note': '100 physical ticks are exact; asynchronous 20Hz sensor callback count is measured, not equated to two callbacks.'}


def rewind(session, artifacts, config):
    before, state_before = quiet(session, artifacts, 'rewind-before')
    require(before['sim_time_ns'] > 1000000000, 'Clock rewind test needs a nonzero actual entry clock')
    process, path, stop = watcher(session, artifacts, 'rewind-native-signal')
    try:
        wire(session, artifacts, 'rewind-single-control', 'control', '--reset-all')
        deadline, index = time.monotonic()+5, 0
        while True:
            reset_world = stats(wire(session, artifacts, 'rewind-world-'+str(index), 'stats'))
            if reset_world['sim_time_ns'] == 0:
                break
            require(time.monotonic() < deadline, 'Actual full-world clock reset was not observed; command not replayed')
            index += 1
        require(reset_world['paused'] and reset_world['iterations'] == 0, 'Full reset did not pause and rewind native iterations')
        session.pause(False)
        wait_time(session, artifacts, 'rewind-new-samples', 900000000)
        after, state_after = quiet(session, artifacts, 'rewind-after')
        require(state_after['epoch'] == state_before['epoch']+1 and state_after['rewind_count'] == state_before['rewind_count']+1,
                'Actual decreasing sample source time did not establish a new filter epoch')
        require(state_after['load_count'] == state_before['load_count'] == 1
                and state_after['init_count'] == state_before['init_count'] == 1
                and state_after['reset_count'] == state_before['reset_count'] == 0,
                'Stock SensorManager clock reset was mistaken for an automatic SensorPlugin Reset callback')
        time.sleep(.15)
    finally:
        stop_watcher(process, stop)
    rows, diagnostics = events(path, live=True)
    pairs, duplicates = paired(rows, -1, after['sim_time_ns'])
    fresh = [(raw, custom) for raw, custom in pairs if custom['epoch'] == state_after['epoch']]
    require(len(fresh) >= 12, 'Too few real new-epoch native sensor samples')
    first_raw, first_custom = fresh[0]
    expected = min(finite_ranges(first_raw))
    require(first_custom['previous_filtered_m'] is None and first_custom['filter_initialized_from_current']
            and abs(first_custom['filtered_m']-expected) < 1e-9,
            'First measured new epoch did not reseed from its actual native scan')
    require(all(abs(min(finite_ranges(raw))-1.2) < .015 for raw, _ in fresh), 'Full reset did not restore actual baseline target geometry')
    return {'passed': True, 'before': before, 'after_native_reset': reset_world, 'after': after,
            'plugin_before': state_before, 'plugin_after': state_after,
            'first_new_epoch_native_ranges_m': safe_ranges(first_raw['ranges']), 'first_new_epoch_plugin': first_custom,
            'new_epoch_pairs': len(fresh), 'raw_jsonl': str(path), 'raw_jsonl_sha256': sha(path),
            'duplicates_retained_in_raw': duplicates, 'wire_diagnostics': diagnostics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    output = args.report.expanduser().resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(value for value in lessons if value.key == 'classic11-noetic:28')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': False, 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l28-') as temporary:
        try:
            for name, config in [('baseline', {'threshold_m': .9, 'alpha': .35}),
                                 ('threshold_only', {'threshold_m': 1.3, 'alpha': .35}),
                                 ('alpha_only', {'threshold_m': .9, 'alpha': .7})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=False, terminals=False)
                record = json.loads((workspace/'build/sensor_lab.json').read_text(encoding='utf-8'))
                initial = plugin(wire(session, artifacts, name+'-initial-plugin', 'snapshot'))
                require(initial['source_sha256'] == source['sensor_plugin.cc'] == record['source_sha256']['sensor_plugin.cc'],
                        'Actually loaded SensorPlugin source differs')
                require(initial['config_sha256'] == sha(workspace/'parameters.json') == record['config_sha256'],
                        'Actually loaded cached sensor input differs')
                row = {'name': name, 'passed': True, 'parameters': config, 'initial_plugin': initial,
                       'actual_sensor_plugin_mapping': mapped_library(session, artifacts, name),
                       'native_signal_checks': main_motion(session, artifacts, name, config)}
                if name == 'baseline':
                    row['saved_input_guard'] = rejected_saved_input(session, artifacts, config)
                    row['pause_and_100_ticks'] = pause_and_ticks(session, artifacts)
                    row['full_reset_and_source_clock_rewind'] = rewind(session, artifacts, config)
                row['prepared_input_sha256'] = hashes(workspace)
                save(artifacts/(name+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/sensor_lab.json'), str(artifacts/(name+'-sensor_lab.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(name+'-generated-world.sdf')))
                result['scenarios'].append(row)
                session.stop('L28 independent sensor case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            base, threshold, alpha = result['scenarios']
            base_phases = {phase['name']: phase for phase in base['native_signal_checks']['phases']}
            threshold_phases = {phase['name']: phase for phase in threshold['native_signal_checks']['phases']}
            alpha_phases = {phase['name']: phase for phase in alpha['native_signal_checks']['phases']}
            require(base_phases['baseline']['last_alarm'] == 'clear' and threshold_phases['baseline']['last_alarm'] == 'near',
                    'Changing only threshold did not change the real baseline alarm')
            require(abs(base_phases['near']['first_filtered_m']-.99) < .015
                    and abs(alpha_phases['near']['first_filtered_m']-.78) < .015,
                    'Changing only alpha did not change the real first near-step filter response')
            require(base_phases['near']['first_alarm'] == 'clear' and alpha_phases['near']['first_alarm'] == 'near',
                    'Different actual filter response did not affect the first near-step alarm')
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop('L28 formal acceptance cleanup')
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
            save(output, result)
    print(json.dumps({key: value for key, value in result.items() if key not in ('scenarios', 'source_sha256')},
                     ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
