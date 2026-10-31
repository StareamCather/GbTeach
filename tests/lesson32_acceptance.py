#!/usr/bin/env python3
"""Independent Classic11 native state recording and real gzserver playback.

Fresh private sessions vary only recording duration or native encoding. Original
WorldStatistics, PosesStamped, Model, Physics and LogStatus bytes are retained.
The original closed native XML is decoded independently before LogPlay can open
or repair it. A second private gzserver -u -p process must load the same immutable
log and expose actual native playback bounds, time and entity poses at bounded
frame checkpoints. Frame indices come from the file, never elapsed wall time.
"""
import argparse
import base64
import bz2
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
import uuid
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model

WORLD, MODEL, STEP_NS = 'lesson32', 'falling_cube', 1000000


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
            value = json.loads(line)
        except ValueError:
            if live and index == len(lines)-1 and not raw.endswith('\n'):
                partial = line
            else:
                noise.append(line)
            continue
        require(isinstance(value, dict), 'Native wire result is not an object')
        require(value.get('source') not in ('client_error', 'decode_error'), 'Native observer error: '+repr(value))
        rows.append(value)
    return rows, {'non_json_lines': noise, 'trailing_partial_line': partial}


def spawn_output(session, arguments, path, label):
    command = shlex.join([str(v) for v in arguments])+' > '+shlex.quote(str(path))+' 2>&1'
    return session._spawn(label, ['bash', '--noprofile', '--norc', '-c', command])


def run(session, arguments, path, timeout=20):
    process = spawn_output(session, arguments, path, 'Independent L32 native query')
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments, observer=None):
    path = artifacts/(label+'.jsonl')
    binary = observer or session.workspace/'build/log_observer'
    require(run(session, [binary, mode, *arguments], path) == 0, 'Native command failed: '+str(path))
    rows, diagnostics = events(path)
    source = {'stats': 'world_stats', 'entity': 'entity', 'world': 'world', 'physics': 'physics',
              'control': 'control_sent', 'log-control': 'log_control_sent', 'log-status': 'log_status',
              'playback-control': 'playback_control_sent'}[mode]
    values = [v for v in rows if v.get('source') == source]
    require(len(values) == 1, 'Expected one native '+source+' result: '+str(path))
    result = values[0]
    result['_wire_artifact'], result['_wire_sha256'], result['_wire_diagnostics'] = str(path), sha(path), diagnostics
    return result


def timestamp(native, field):
    value = first(native, field)
    require(isinstance(value, dict), 'Missing native timestamp: '+field)
    return int(first(value, 'sec', '0'))*1000000000+int(first(value, 'nsec', '0'))


def stats(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native pause state absent')
    result = {'sim_time_ns': timestamp(native, 'sim_time'), 'iterations': int(first(native, 'iterations')),
              'paused': first(native, 'paused') == 'true', 'raw_wire': row}
    playback = first(native, 'log_playback_stats')
    if playback is not None:
        result['playback_bounds'] = {'start_time_ns': timestamp(playback, 'start_time'),
                                     'end_time_ns': timestamp(playback, 'end_time')}
    return result


def position(native):
    vector = first(first(native, 'pose'), 'position')
    require(isinstance(vector, dict), 'Native model position is absent')
    xyz = [float(first(vector, k, '0')) for k in ('x', 'y', 'z')]
    require(all(math.isfinite(v) for v in xyz), 'Nonfinite native position')
    return xyz


def entity(session, artifacts, label, name=MODEL, observer=None):
    row = wire(session, artifacts, label, 'entity', '--name', name, observer=observer)
    native = parsed_model(row['raw_protobuf'])
    require(str(first(native, 'name')).split('::')[-1] == name and int(first(native, 'id')) > 0,
            'Native model identity differs from requested entity')
    if name == MODEL:
        links = native.get('link', [])
        require(len(links) == 1 and str(first(links[0], 'name')).split('::')[-1] == 'body',
                'Effective canonical-link oracle requires the actual single body link')
        require(max(abs(v) for v in position(links[0])) < 1e-10, 'Actual body link has a nonidentity relative position')
        orientation = first(first(links[0], 'pose'), 'orientation')
        require(max(abs(float(first(orientation, axis, '0'))) for axis in ('x', 'y', 'z')) < 1e-10
                and abs(float(first(orientation, 'w', '1'))-1) < 1e-10,
                'Actual body link has a nonidentity relative orientation')
    return {'position_m': position(native), 'model_id': int(first(native, 'id')), 'native_model': native, 'raw_wire': row}


def box_size(native):
    links = native.get('link', [])
    require(len(links) == 1 and len(links[0].get('collision', [])) == 1, 'Native collision topology differs')
    value = first(first(first(links[0]['collision'][0], 'geometry'), 'box'), 'size')
    return [float(first(value, k)) for k in ('x', 'y', 'z')]


def recorder_path(row):
    native = parsed_model(row['raw_protobuf'])
    return first(first(native, 'log_file', {}), 'full_path', '')


def native_scene(session, artifacts, label, observer=None):
    row = wire(session, artifacts, label+'-scene', 'world', observer=observer)
    native = parsed_model(row['raw_protobuf'])
    require({str(first(m, 'name')).split('::')[-1] for m in native.get('model', [])} == {'floor', MODEL},
            'Native scene has unexpected models')
    floor = entity(session, artifacts, label+'-floor', 'floor', observer)
    cube = entity(session, artifacts, label+'-cube', observer=observer)
    require(first(floor['native_model'], 'is_static') == 'true' and abs(floor['position_m'][2]+.1) < 1e-10,
            'Native floor is not static with its top at z=0')
    require(max(abs(a-b) for a, b in zip(box_size(floor['native_model']), (6, 5, .2))) < 1e-10,
            'Native floor collision differs')
    require(max(abs(v-.4) for v in box_size(cube['native_model'])) < 1e-10, 'Native cube collision differs')
    return {'scene_wire': row, 'floor': floor, 'cube': cube}


def watcher(session, artifacts, label):
    # A reused report directory can retain old stop markers and old handshake
    # bytes. Give this observer fresh paths without deleting earlier evidence.
    identity = label+'-'+uuid.uuid4().hex
    path, stop = artifacts/(identity+'.jsonl'), artifacts/(identity+'.stop')
    require(not path.exists() and not stop.exists(), 'Native watcher artifact identity already exists')
    process = spawn_output(session, [session.workspace/'build/log_observer', 'watch', '--seconds', '120',
                                   '--stop-file', str(stop)], path, 'Independent L32 original native wire')
    deadline = time.monotonic()+8
    try:
        while True:
            rows, _ = events(path, live=True)
            if any(v.get('source') == 'client_ready' for v in rows) and any(v.get('source') == 'world_stats' for v in rows):
                require(process.poll() is None, 'Native watcher exited during its actual readiness handshake')
                return process, path, stop
            require(process.poll() is None and time.monotonic() < deadline, 'Native watcher did not connect')
            time.sleep(.04)
    except BaseException:
        stop_watcher(process, stop)
        raise


def stop_watcher(process, stop):
    if process.poll() is None:
        stop.write_text('Independent L32 observation complete\n', encoding='utf-8')
        try:
            process.wait(timeout=8)
        except BaseException:
            terminate_owned(process)
            raise
    require(process.returncode == 0, 'Native watcher did not complete normal transport/IOManager cleanup')


def log_time(text):
    fields = str(text).split()
    require(len(fields) == 2, 'Native SDF timestamp must contain seconds and nanoseconds')
    return int(fields[0])*1000000000+int(fields[1])


def decode_closed_log(path, artifacts, label, encoding):
    """Independent stdlib decoder; never invokes the repairing native reader."""
    raw = Path(path).read_bytes()
    require(raw.rstrip().endswith(b'</gazebo_log>'), 'Native recorder has not flushed its closing footer')
    root = ET.fromstring(raw)
    require(root.tag == 'gazebo_log', 'Recording is not a native gazebo_log')
    header = root.find('header')
    require(header is not None, 'Native raw log has no header')
    metadata = {key: header.findtext(key) for key in ('log_version', 'gazebo_version', 'rand_seed')}
    require(all(metadata.values()) and metadata['gazebo_version'].startswith('11.'), 'Native log header is incomplete/wrong major')
    chunks, frames, states, held, held_links, source_times = [], [], [], {}, {}, []
    for chunk_index, chunk in enumerate(root.findall('chunk')):
        require(chunk.get('encoding') == encoding, 'Native actual chunk encoding differs from selected encoding')
        text = chunk.text or ''
        if encoding == 'txt':
            decoded = text.encode('utf-8')
        else:
            packed = base64.b64decode(''.join(text.split()), validate=True)
            decoded = zlib.decompress(packed) if encoding == 'zlib' else bz2.decompress(packed)
        decoded = decoded.rstrip(b'\0')
        require(b'\0' not in decoded, 'Native decoded log has an embedded NUL')
        path_decoded = artifacts/(label+'-chunk-'+str(chunk_index)+'.xml')
        path_decoded.write_bytes(decoded)
        documents = ET.fromstring(b'<frames>'+decoded+b'</frames>')
        chunks.append({'index': chunk_index, 'encoding': encoding, 'decoded_xml': str(path_decoded),
                       'decoded_sha256': sha(path_decoded), 'frames': len(documents)})
        for document in documents:
            require(document.tag == 'sdf', 'Native decoded chunk contains a non-SDF frame')
            index = len(frames)
            document.tail = None
            frames.append(document)
            source_times.extend(log_time(v.text) for v in document.findall('.//sim_time'))
            state = document.find('state')
            if state is None:
                require(index == 0 and document.find('world') is not None, 'Unexpected non-state frame after initial full world')
                world = document.find('world')
                require(world.get('name') == WORLD and not list(world.iter('plugin')), 'Log embeds wrong world or a custom replay plugin')
                require({m.get('name') for m in world.findall('model')} == {'floor', MODEL}, 'Native full world model set differs')
                cube = world.find("model[@name='falling_cube']")
                links = cube.findall('link')
                require(len(links) == 1 and links[0].get('name') == 'body'
                        and cube.get('canonical_link', 'body') == 'body', 'Native log has a different canonical-link topology')
                relative = [float(v) for v in links[0].findtext('pose', '0 0 0 0 0 0').split()]
                require(len(relative) == 6 and max(abs(v) for v in relative) < 1e-12,
                        'Native log canonical body initial relative pose is not identity')
                for model in world.findall('model'):
                    pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
                    held[model.get('name')] = pose[:3]
                require(abs(held[MODEL][2]-2) < 1e-10, 'Native full-world initial drop height differs')
                continue
            require(state.get('world_name') == WORLD, 'Native state names another world')
            for model in state.findall('model'):
                pose = model.find('pose')
                if pose is not None:
                    values = [float(v) for v in pose.text.split()]
                    require(len(values) == 6 and all(math.isfinite(v) for v in values), 'Malformed native state pose')
                    held[model.get('name')] = values[:3]
                if model.get('name') == MODEL:
                    link = model.find("link[@name='body']/pose")
                    require(link is not None, 'Native ModelState has no canonical body world pose')
                    values = [float(v) for v in link.text.split()]
                    require(len(values) == 6 and all(math.isfinite(v) for v in values), 'Malformed native LinkState world pose')
                    held_links[MODEL] = values
            require(MODEL in held, 'Native states lack the cube pose')
            require(MODEL in held_links, 'Native states lack the effective canonical body world pose')
            states.append({'frame_index': index, 'sim_time_ns': log_time(state.findtext('sim_time')),
                           'iterations': int(state.findtext('iterations')), 'position_m': list(held[MODEL]),
                           'canonical_link_pose_xyz_rpy': list(held_links[MODEL]),
                           'canonical_link_position_m': list(held_links[MODEL][:3]),
                           'effective_model_position_m': list(held_links[MODEL][:3])})
    require(len(chunks) >= 1 and len(states) >= 5, 'Too few actual native state frames')
    require(all(b['sim_time_ns'] >= a['sim_time_ns'] for a, b in zip(states, states[1:])), 'Native log source times regress')
    require(all(b['iterations'] >= a['iterations'] for a, b in zip(states, states[1:])), 'Native log source iterations regress')
    require(states[-1]['sim_time_ns'] > states[0]['sim_time_ns'] and states[-1]['position_m'][2] < 1.95,
            'Native state content does not establish falling motion')
    return {'raw_log': str(path), 'raw_log_sha256': sha(path), 'raw_log_size': len(raw), 'header': metadata,
            'encoding': encoding, 'chunks': chunks, 'frame_count': len(frames), 'states': states,
            'effective_player_pose_rule': 'Single canonical body with identity initial relative pose: Model.SetState applies model pose, then Link.SetState applies body world pose, which updates the parent model. Effective native model XYZ equals recorded canonical body world XYZ.',
            'start_time_ns': source_times[0], 'end_time_ns': source_times[-1]}, frames


def native_reader(session, artifacts, label, log, decoded, frames):
    path = artifacts/(label+'-native-logplay.jsonl')
    require(run(session, [session.workspace/'build/log_observer', 'log-read', '--file', log], path) == 0,
            'Installed SDK LogPlay could not read a previously closed native log')
    rows, diagnostics = events(path)
    actual_frames = [v for v in rows if v.get('source') == 'log_frame']
    metadata = [v for v in rows if v.get('source') == 'log_metadata']
    require(len(actual_frames) == len(frames) and len(metadata) == 1, 'Native SDK and independent decoder disagree on frame count')
    for index, row in enumerate(actual_frames):
        require(row['frame_index'] == index, 'Native LogPlay frame order has a gap')
        require(ET.tostring(ET.fromstring(row['raw_sdf'])) == ET.tostring(frames[index]),
                'Native LogPlay and independent decoder disagree on original SDF frame '+str(index))
    value = metadata[0]
    require(all(str(value[k]) == str(decoded['header'][k]) for k in ('log_version', 'gazebo_version', 'rand_seed')),
            'Installed SDK metadata differs from the original raw log header')
    require(value['encoding'] == decoded['encoding'] and value['chunk_count'] == len(decoded['chunks'])
            and value['frame_count'] == decoded['frame_count'], 'Native SDK encoding/chunk/frame count differs')
    require(value['start_time_ns'] == decoded['start_time_ns'] and value['end_time_ns'] == decoded['end_time_ns'],
            'Native SDK source-time bounds differ from original log content')
    require(sha(log) == decoded['raw_log_sha256'], 'SDK log reader changed the original native file')
    return {'passed': True, 'metadata': value, 'raw_wire': str(path), 'raw_wire_sha256': sha(path), 'diagnostics': diagnostics}


def process_proof(session, artifacts, label, log=None):
    matches = []
    installed = Path(shutil.which('gzserver', path=session.env.get('PATH'))).resolve()
    for pid in live_token_processes(session.token):
        directory = Path('/proc')/str(pid)
        try:
            exe = (directory/'exe').resolve()
            if not os.path.samefile(str(exe), str(installed)):
                continue
            argv = (directory/'cmdline').read_bytes().decode().strip('\0').split('\0')
            require(exe.read_bytes()[:4] == b'\x7fELF', 'Native gzserver process is not an actual ELF')
            require('-u' in argv and '-s' not in argv, 'Actual server is not paused or loaded an injected process plugin')
            if log is not None:
                require('-p' in argv and Path(argv[argv.index('-p')+1]).resolve() == Path(log).resolve(),
                        'Actual native process was not launched with this exact state log')
            else:
                require('-p' not in argv, 'Original recording process is already a log player')
            environ = dict(v.split('=', 1) for v in (directory/'environ').read_bytes().decode().split('\0') if '=' in v)
            require(environ.get('GAZEBOTEACH_SESSION_TOKEN') == session.token
                    and environ.get('GAZEBO_MASTER_URI') == session.env['GAZEBO_MASTER_URI']
                    and environ.get('IGN_PARTITION') == session.env['IGN_PARTITION'], 'Native process escaped private environment')
            maps = artifacts/(label+'-'+str(pid)+'.maps')
            maps.write_bytes((directory/'maps').read_bytes())
            matches.append({'pid': pid, 'executable': str(exe), 'executable_sha256': sha(exe), 'argv': argv,
                            'maps': str(maps), 'maps_sha256': sha(maps), 'private_master': environ['GAZEBO_MASTER_URI'],
                            'private_partition': environ['IGN_PARTITION'], 'session_token': session.token})
        except (OSError, UnicodeError):
            continue
    require(len(matches) == 1, 'Expected exactly one owned actual native gzserver process')
    return matches[0]


def compare_pose(actual, expected, label, tolerance=2e-6):
    error = max(abs(a-b) for a, b in zip(actual, expected))
    require(error <= tolerance, label+' differs from independent native log pose: '+str(error))
    return error


def serialized_original_pose(actual, significant_digits=3):
    """Native model/link poses round decimal4, then stream precision3/4 defaultfloat.

    This is deliberately separate from replay-vs-log comparisons: a player must
    apply the stored pose, while the original double must reproduce serialization.
    https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ModelState.hh
    """
    result = []
    for value in actual:
        decimal4 = math.copysign(math.floor(abs(value)*10000+.5)/10000, value)
        result.append(float(format(decimal4, '.'+str(significant_digits)+'g')))
    return result


def serialization_sdk(artifacts):
    headers = []
    for filename, digits in [('ModelState.hh', 3), ('LinkState.hh', 4)]:
        header = Path('/usr/include/gazebo-11/gazebo/physics')/filename
        text = header.read_text(encoding='utf-8')
        require('_out.unsetf(std::ios_base::floatfield)' in text and 'std::setprecision('+str(digits)+')' in text
                and all('precision(_state.pose.Pos().'+axis+'(), 4)' in text for axis in ('X', 'Y', 'Z')),
                'Installed native '+filename+' serialization differs from the exact independent oracle')
        preserved = artifacts/('installed-'+filename)
        shutil.copy2(str(header), str(preserved))
        headers.append({'installed_header': str(header), 'installed_header_sha256': sha(header),
                        'preserved_header': str(preserved), 'significant_digits': digits})
    return {'headers': headers, 'source_rule': 'unsetf(floatfield); precision(component,4); defaultfloat significant3 model / significant4 link.',
            'original_comparison': 'Independently reproduce both recorded model and canonical body world coordinate serialization within1e-9 m.',
            'replay_comparison': 'Compare actual native player model to the effective canonical body world pose within2e-6 m; no original-value quantization allowance.'}


def compare_original(actual, expected, label, significant_digits=3):
    quantized = serialized_original_pose(actual, significant_digits)
    error = compare_pose(quantized, expected, label+' after actual ModelState serialization', 1e-9)
    return {'native_original_position_m': actual, 'expected_serialized_position_m': quantized, 'significant_digits': significant_digits,
            'logged_position_m': expected, 'serialized_position_error_m': error,
            'original_to_log_quantization_m': max(abs(a-b) for a, b in zip(actual, expected))}


def original_wire(rows, decoded, final, final_world):
    by_time = {}
    for value in decoded['states']:
        by_time.setdefault(value['sim_time_ns'], []).append(value)
    samples = []
    for row in rows:
        if row.get('source') != 'pose':
            continue
        native = parsed_model(row['raw_protobuf'])
        for pose in native.get('pose', []):
            if str(first(pose, 'name')).split('::')[-1] != MODEL:
                continue
            require(int(first(pose, 'id')) == final['model_id'], 'Original pose stream changed the cube identity')
            ns = timestamp(native, 'time')
            if ns not in by_time:
                continue
            xyz = position({'pose': [pose]})
            quantized = serialized_original_pose(xyz)
            errors = [max(abs(a-b) for a, b in zip(quantized, s['position_m'])) for s in by_time[ns]]
            require(min(errors) <= 1e-9, 'Original native pose stream serialization and native log disagree at source time '+str(ns))
            link_quantized = serialized_original_pose(xyz, 4)
            link_errors = [max(abs(a-b) for a, b in zip(link_quantized, s['canonical_link_position_m'])) for s in by_time[ns]]
            require(min(link_errors) <= 1e-9, 'Original native canonical body world serialization and log disagree at source time '+str(ns))
            joint_error = min(max(a, b) for a, b in zip(errors, link_errors))
            require(joint_error <= 1e-9, 'No same-time native state matches both model and canonical body serialization')
            samples.append({'sim_time_ns': ns, 'position_m': xyz, 'expected_serialized_position_m': quantized,
                            'expected_serialized_body_world_position_m': link_quantized,
                            'serialized_log_pose_error_m': min(errors), 'serialized_log_body_pose_error_m': min(link_errors),
                            'same_state_joint_serialization_error_m': joint_error})
    unique = {v['sim_time_ns']: v for v in samples}
    require(len(unique) >= 3 and max(unique)-min(unique) >= 50000000,
            'Too few independent original native trajectory times match the recorded state log')
    require(any(v['position_m'][2] > final['position_m'][2]+.03 for v in samples), 'Original wire does not show falling motion')
    clocks = [stats(row) for row in rows if row.get('source') == 'world_stats']
    completed = [value for value in clocks if value['paused'] and value['sim_time_ns'] == final_world['sim_time_ns']
                 and value['iterations'] == final_world['iterations']]
    require(completed, 'Continuous independent WorldStatistics did not witness the actual requested physical endpoint')
    log_status = [v for v in rows if v.get('source') == 'log_status']
    require(log_status, 'Continuous native observer did not retain recorder status')
    paths = [recorder_path(v) for v in log_status]
    require(any(paths) and paths[-1] == '', 'Native status did not independently show active path then final async cleanup')
    return {'passed': True, 'pose_samples_in_arrival_order': samples, 'native_status_transitions': log_status,
            'full_path_transitions': paths, 'raw_packets_retained': len(rows),
            'continuous_native_endpoint_world_statistics': completed[-1]}


def endpoint_equivalence(final_world, body, decoded):
    """Change-triggered logging can stop before a later stationary source clock.

    Preserve both real timestamps. A retained state is usable only when BOTH
    native model and canonical body serializations of the independently measured
    final source pose equal the last actual recorded state. No frame is invented.
    """
    last = decoded['states'][-1]
    original_time, recorded_time = final_world['sim_time_ns'], last['sim_time_ns']
    require(recorded_time == decoded['end_time_ns'] and recorded_time <= original_time
            and last['iterations'] <= final_world['iterations'], 'Native log endpoint is after the actual completed source clock')
    require(original_time-recorded_time == (final_world['iterations']-last['iterations'])*STEP_NS,
            'Native source/log endpoint time and iteration lag disagree')
    serialization = {
        'model': compare_original(body['position_m'], last['position_m'], 'Original final native model'),
        'canonical_body_world': compare_original(body['position_m'], last['canonical_link_position_m'],
                                                 'Original final native canonical body world', 4)}
    return {'passed': True, 'original_sim_time_ns': original_time, 'recorded_sim_time_ns': recorded_time,
            'original_iterations': final_world['iterations'], 'recorded_iterations': last['iterations'],
            'match_kind': 'exact' if recorded_time == original_time else 'retained_state',
            'retained_interval_ns': original_time-recorded_time, 'original_to_recorded_serialization': serialization,
            'meaning': 'Native replay ends at the stored timestamp. A later paused original clock is state-equivalent only at both native serialization precisions.'}


def replay_case(original, replay, artifacts, label, log, decoded):
    observer = original.workspace/'build/log_observer'
    replay.world_name = WORLD
    require(replay.env['GAZEBO_MASTER_URI'] != original.env['GAZEBO_MASTER_URI']
            and replay.env['IGN_PARTITION'] != original.env['IGN_PARTITION'], 'Original and playback transport domains overlap')
    replay.server = replay._spawn('Independent native gzserver state-log player', ['gzserver', '--verbose', '-u', '-p', str(log)])
    replay._wait_world(timeout=30)
    proof = process_proof(replay, artifacts, label+'-player', log)
    start = stats(wire(replay, artifacts, label+'-playback-initial-world', 'stats', observer=observer))
    require(start['paused'] and start.get('playback_bounds') == {
        'start_time_ns': decoded['start_time_ns'], 'end_time_ns': decoded['end_time_ns']},
        'Native server has no actual playback statistics matching the native log bounds')
    scene = native_scene(replay, artifacts, label+'-playback-initial', observer)
    require(abs(scene['cube']['position_m'][2]-2) < 1e-10, 'Actual player did not initialize from full native world')
    initial_ns = start['sim_time_ns']
    time.sleep(.16)
    quiet = stats(wire(replay, artifacts, label+'-playback-paused-quiet', 'stats', observer=observer))
    require(quiet['paused'] and quiet['sim_time_ns'] == initial_ns and quiet['iterations'] == start['iterations'],
            'Actual player advances while paused')
    states = decoded['states']
    indices = sorted({0, len(states)//4, len(states)//2, 3*len(states)//4, len(states)-1})
    previous_frame, checkpoints = 0, []
    for ordinal in indices:
        expected = states[ordinal]
        count = expected['frame_index']-previous_frame
        require(1 <= count <= 1000, 'Playback frame delta is outside finite native control bounds')
        control = wire(replay, artifacts, label+'-playback-'+str(ordinal)+'-single-control', 'control', '--steps', str(count), observer=observer)
        native = parsed_model(control['raw_protobuf'])
        require(first(native, 'pause') == 'true' and int(first(native, 'multi_step')) == count,
                'Playback one-shot control bytes differ from the requested native frame count')
        deadline, polls = time.monotonic()+8, []
        while True:
            actual = stats(wire(replay, artifacts, label+'-playback-'+str(ordinal)+'-clock-'+str(len(polls)), 'stats', observer=observer))
            polls.append(actual)
            if actual['sim_time_ns'] >= expected['sim_time_ns'] and actual['iterations'] >= expected['iterations']:
                break
            require(time.monotonic() < deadline, 'Native player did not finish one bounded state jump; control was not resent')
        require(actual['paused'] and actual['sim_time_ns'] == expected['sim_time_ns'] and actual['iterations'] == expected['iterations'],
                'Actual native playback time/iterations do not match the requested log frame')
        body = entity(replay, artifacts, label+'-playback-'+str(ordinal)+'-cube', observer=observer)
        error = compare_pose(body['position_m'], expected['effective_model_position_m'], 'Actual replay entity after canonical body state')
        checkpoints.append({'expected_log_state': expected, 'actual_native_world': actual, 'actual_native_cube': body,
                            'single_publication': control, 'completion_polls': polls, 'pose_max_error_m': error})
        previous_frame = expected['frame_index']
    require(len({v['actual_native_world']['sim_time_ns'] for v in checkpoints}) >= 4,
            'Native player checkpoints do not establish a trajectory')
    require(sha(log) == decoded['raw_log_sha256'], 'Actual native playback changed the original state log')
    return {'passed': True, 'actual_native_player_process': proof, 'initial_native_world': start,
            'initial_native_scene': scene, 'quiet_paused_world': quiet, 'checkpoints': checkpoints,
            'algorithm': 'One initial full world is consumed by Server.Load; bounded multi_step advances actual native SDF frames. Model.SetState applies recorded model pose, then the canonical body world state updates its parent model. Verified identity initial relative pose makes effective native model XYZ equal stored body world XYZ. Actual iterations/time/effective pose are checked against the file.'}


def record_case(session, artifacts, label, config):
    before = stats(wire(session, artifacts, label+'-initial-world', 'stats'))
    require(before['paused'] and before['sim_time_ns'] == before['iterations'] == 0 and 'playback_bounds' not in before,
            'Fresh original world is not a paused physics simulation at zero')
    scene = native_scene(session, artifacts, label+'-original')
    require(abs(scene['cube']['position_m'][2]-2) < 1e-10, 'Original cube starts at wrong height')
    physics = wire(session, artifacts, label+'-physics', 'physics')
    native = parsed_model(physics['raw_protobuf'])
    require(abs(float(first(native, 'max_step_size'))-.001) < 1e-12 and abs(float(first(first(native, 'gravity'), 'z'))+9.8) < 1e-10,
            'Original real physics timestep or gravity differs')
    proof = process_proof(session, artifacts, label+'-original')
    process, path, stop = watcher(session, artifacts, label+'-original-wire')
    try:
        output = artifacts/(label+'-course-record.log')
        require(run(session, ['python3', 'lab.py', 'record'], output, timeout=100) == 0, 'Course native recording failed: '+str(output))
        final = stats(wire(session, artifacts, label+'-recorded-final-world', 'stats'))
        body = entity(session, artifacts, label+'-recorded-final-cube')
        status = wire(session, artifacts, label+'-closed-recorder-status', 'log-status')
        require(recorder_path(status) == '',
                'Native recorder has not completed asynchronous cleanup')
        time.sleep(.2)
    finally:
        stop_watcher(process, stop)
    rows, diagnostics = events(path)
    require(final['paused'] and final['iterations'] == config['record_steps']
            and final['sim_time_ns'] == config['record_steps']*STEP_NS, 'Course did not record exactly the requested paused physical ticks')
    require(body['model_id'] == scene['cube']['model_id'], 'Original recording replaced the actual cube')
    logs = list((session.workspace/'build').rglob('state.log'))
    require(len(logs) == 1, 'Expected one actual native state.log in the fresh personal build')
    log = logs[0]
    initial_digest = sha(log)
    decoded, frames = decode_closed_log(log, artifacts, label, config['encoding'])
    time.sleep(.2)
    require(sha(log) == initial_digest, 'Native log bytes changed after claimed stop/flush')
    final_match = endpoint_equivalence(final, body, decoded)
    trajectory = original_wire(rows, decoded, body, final)
    native_read = native_reader(session, artifacts, label, log, decoded, frames)
    copied = artifacts/(label+'-state.log')
    shutil.copy2(str(log), str(copied))
    require(sha(copied) == initial_digest, 'Preserved native state log differs from original bytes')
    decoded['preserved_raw_log'] = str(copied)
    inspection = artifacts/(label+'-course-inspect.log')
    require(run(session, ['python3', 'lab.py', 'inspect'], inspection, timeout=30) == 0, 'Course native log inspection failed')
    require(sha(log) == initial_digest, 'Course inspection changed native state log bytes')
    return {'passed': True, 'parameters': config, 'initial_native_world': before, 'initial_native_scene': scene,
            'actual_original_process': proof, 'native_physics': physics, 'final_native_world': final, 'final_native_cube': body,
            'final_native_recorder_status': status, 'native_log': decoded, 'installed_sdk_reader': native_read,
            'original_final_to_native_log_endpoint': final_match,
            'original_wire_analysis': trajectory, 'raw_jsonl': str(path), 'raw_jsonl_sha256': sha(path),
            'wire_diagnostics': diagnostics, 'course_record_output': str(output), 'course_inspect_output': str(inspection),
            'watcher_normal_exit': process.returncode, 'watcher_stop_file': str(stop)}, log


def preserve_workspace_diagnostics(session, artifacts, label):
    """Retain bounded course query journals before fresh Storage is removed."""
    build = session.workspace/'build'
    paths = sorted(set(build.glob('checkpoint-*.json'))
                   | set(build.glob('playback-*/diagnostics.json'))
                   | set(build.glob('playback-*/ownership.json')))
    require(len(paths) <= 256, 'Course diagnostic evidence exceeds bounded 256-file preservation')
    total, preserved = 0, []
    for path in paths:
        require(path.is_file() and not path.is_symlink()
                and session.workspace.resolve() in path.resolve().parents,
                'Course diagnostic evidence is not a regular owned workspace file')
        size = path.stat().st_size
        total += size
        require(size <= 16*1024*1024 and total <= 64*1024*1024,
                'Course diagnostic evidence exceeds bounded file/total bytes')
        relative = path.relative_to(build)
        target = artifacts/(label+'-course-diagnostics')/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(path), str(target))
        preserved.append({'scenario': label, 'source': str(path), 'relative_build_path': relative.as_posix(),
                          'preserved': str(target), 'sha256': sha(target), 'size_bytes': size})
    return preserved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(v for v in lessons if v.id == '32' and v.profile == 'classic11-noetic')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': False, 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l32-') as temporary:
        try:
            result['native_serialization_oracle'] = serialization_sdk(artifacts)
            for label, config in [('baseline', {'record_steps': 300, 'encoding': 'zlib'}),
                                  ('duration_only', {'record_steps': 800, 'encoding': 'zlib'}),
                                  ('encoding_only', {'record_steps': 300, 'encoding': 'txt'})]:
                result['active_case'] = label
                storage = Storage(Path(temporary)/label)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                prepared = json.loads((workspace/'build/log_lab.json').read_text(encoding='utf-8'))
                require(prepared['parameters'] == config and prepared['config_sha256'] == sha(workspace/'parameters.json'),
                        'Prepared configuration differs from the fresh requested variant')
                require(all(source[p] == value == sha(workspace/p) for p, value in prepared['source_sha256'].items()),
                        'Prepared native observer/course source differs from original templates')
                require(prepared['observer_sha256'] == sha(workspace/'build/log_observer') and prepared['world_sha256'] == sha(workspace/'world.sdf'),
                        'Prepared actual native observer or world changed')
                row, log = record_case(session, artifacts, label, config)
                playback = LessonSession(ROOT, lesson, storage=Storage(Path(temporary)/(label+'-player')))
                sessions.append((label+'-player', playback))
                row['actual_native_playback'] = replay_case(session, playback, artifacts, label, log, row['native_log'])
                playback.stop(reason='Independent L32 native playback complete')
                after = stats(wire(session, artifacts, label+'-original-after-private-playback', 'stats'))
                after_body = entity(session, artifacts, label+'-original-after-private-playback-cube')
                require(after['paused'] and after['sim_time_ns'] == row['final_native_world']['sim_time_ns']
                        and after['iterations'] == row['final_native_world']['iterations']
                        and after_body['position_m'] == row['final_native_cube']['position_m'], 'Private native player mutated the original world')
                require(sha(log) == row['native_log']['raw_log_sha256'], 'Native source log changed during playback')
                row['name'], row['prepared_input'] = label, prepared
                row['original_isolation_after_playback'] = {'world': after, 'cube': after_body}
                save(artifacts/(label+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/log_lab.json'), str(artifacts/(label+'-prepared.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(label+'-world.sdf')))
                result['scenarios'].append(row)
                session.stop(reason='Independent L32 original recording complete')
                print(json.dumps({'scenario': label, 'passed': True}), flush=True)
            baseline, duration, encoding = result['scenarios']
            baseline_states = baseline['native_log']['states']
            by_time = {v['sim_time_ns']: v for v in duration['native_log']['states']}
            encoding_by_time = {v['sim_time_ns']: v for v in encoding['native_log']['states']}
            overlap = []
            for value in baseline_states:
                ns = value['sim_time_ns']
                if ns in by_time and ns in encoding_by_time:
                    a = compare_pose(by_time[ns]['position_m'], value['position_m'], 'Duration-only overlapping native state')
                    b = compare_pose(encoding_by_time[ns]['position_m'], value['position_m'], 'Encoding-only native state')
                    c = compare_pose(by_time[ns]['canonical_link_position_m'], value['canonical_link_position_m'], 'Duration-only overlapping native canonical body state')
                    d = compare_pose(encoding_by_time[ns]['canonical_link_position_m'], value['canonical_link_position_m'], 'Encoding-only native canonical body state')
                    overlap.append({'sim_time_ns': ns, 'duration_pose_error_m': a, 'encoding_pose_error_m': b,
                                    'duration_canonical_body_pose_error_m': c, 'encoding_canonical_body_pose_error_m': d})
            require(len(overlap) >= 10, 'Fresh variants have too few shared independently recorded source times')
            require(duration['final_native_world']['sim_time_ns'] == 800000000
                    and duration['native_log']['end_time_ns'] <= 800000000
                    and duration['native_log']['end_time_ns'] > baseline['native_log']['end_time_ns'] == 300000000,
                    'Duration-only change did not extend the actual source run and recorded moving trajectory')
            require(.195 <= duration['final_native_cube']['position_m'][2] <= .205, 'Longer native recording did not reach actual floor contact')
            require(encoding['native_log']['encoding'] == 'txt' and baseline['native_log']['encoding'] == 'zlib'
                    and encoding['native_log']['raw_log_sha256'] != baseline['native_log']['raw_log_sha256'],
                    'Encoding-only variant did not change native encoding and bytes')
            result['cross_case_results'] = {'passed': True, 'shared_native_state_times': overlap,
                'long_recording_contact_height_m': duration['final_native_cube']['position_m'][2],
                'long_run_source_final_sim_time_ns': duration['final_native_world']['sim_time_ns'],
                'long_run_native_log_end_time_ns': duration['native_log']['end_time_ns'],
                'long_run_source_to_log_endpoint': duration['original_final_to_native_log_endpoint'],
                'same_motion_under_changed_encoding': True}
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            result['native_log_preservation'] = []
            result['course_diagnostic_preservation'] = []
            for label, session in reversed(sessions):
                # Preserve the bytes that caused a failure before server shutdown
                # gets any opportunity to flush another footer or pending buffer.
                try:
                    for index, path in enumerate(sorted((session.workspace/'build').rglob('state.log'))):
                        target = artifacts/(label+'-before-cleanup-'+str(index)+'-state.log')
                        shutil.copy2(str(path), str(target))
                        result['native_log_preservation'].append({'scenario': label, 'source': str(path),
                            'preserved': str(target), 'sha256': sha(target),
                            'closing_footer_present': target.read_bytes().rstrip().endswith(b'</gazebo_log>')})
                    for filename in ('recording.json', 'log_lab.json'):
                        path = session.workspace/'build'/filename
                        if path.is_file():
                            shutil.copy2(str(path), str(artifacts/(label+'-cleanup-'+filename)))
                    result['course_diagnostic_preservation'].extend(preserve_workspace_diagnostics(session, artifacts, label))
                except Exception as error:
                    result['cleanup_errors'].append(label+' evidence preservation: '+repr(error))
                try:
                    session.stop(reason='Independent L32 formal acceptance cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/(label+'-runtime.log')))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/(label+'-session.json')))
                except Exception as error:
                    result['cleanup_errors'].append(label+': '+repr(error))
            result['leftovers'] = {session.token: live_token_processes(session.token) for _, session in sessions}
            result['source_templates_unchanged'] = hashes(lesson.folder) == source
            if result['cleanup_errors'] or any(result['leftovers'].values()) or not result['source_templates_unchanged']:
                result['passed'] = False
            result['elapsed_seconds'] = round(time.monotonic()-started, 3)
            save(report_path, result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('scenarios', 'source_sha256')}, ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
