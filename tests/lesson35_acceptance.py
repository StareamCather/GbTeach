#!/usr/bin/env python3
"""Independent acceptance of actual Classic11 fault diagnosis and repair.

Opaque input paths carry four independently checked XML/native faults. At every
real paused endpoint a synchronous acceptance hook issues its own native SDK
queries, decodes original protobuf DebugString fields and reads the live server
identity/maps. The hook changes no input, control, ticks or simulator state.
Repairs must change the defective source feature, preserve the original input
and learner files, and correct the corresponding measured physical behavior in
a fresh isolated server. A healthy paused preview is queried before and after.
"""
import argparse
import difflib
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.lesson05_acceptance import first, parsed_model
from tests.lesson32_acceptance import hashes, save, sha

WORLD, MODEL = 'lesson35', 'cube_000'
TOKEN_KEY = 'GAZEBOTEACH_L35_SESSION'
CASES = ('malformed-sdf', 'missing-plugin', 'zero-gravity', 'missing-collision')
SDK_BASE = 'https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/'


def require(value, message):
    if not value:
        raise AssertionError(message)


def close(actual, expected, tolerance, message):
    require(math.isfinite(float(actual)) and abs(float(actual)-expected) <= tolerance,
            message+': observed='+str(actual)+', expected='+str(expected))


def process_info(pid):
    path = Path('/proc')/str(pid)
    try:
        if path.stat().st_uid != os.getuid():
            return None
        fields = (path/'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        environment = dict(v.split('=', 1) for v in (path/'environ').read_bytes().decode().split('\0') if '=' in v)
        return {'pid': int(pid), 'uid': path.stat().st_uid, 'start_ticks': fields[19],
                'exe': str((path/'exe').resolve()),
                'argv': [v.decode(errors='replace') for v in (path/'cmdline').read_bytes().split(b'\0') if v],
                'env': environment}
    except (OSError, ValueError, IndexError):
        return None


def token_processes(token):
    found = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            info = process_info(path.name)
            if info and token in (info['env'].get(TOKEN_KEY), info['env'].get('GAZEBOTEACH_SESSION_TOKEN')):
                found.append({k: info[k] for k in ('pid', 'uid', 'start_ticks', 'exe', 'argv')})
    return found


def native_identity(environment, artifacts, label, token, input_path=None):
    installed = Path(shutil.which('gzserver', path=environment.get('PATH'))).resolve()
    candidates = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        info = process_info(path.name)
        if not info or info['env'].get('GAZEBO_MASTER_URI') != environment['GAZEBO_MASTER_URI']:
            continue
        if token not in (info['env'].get(TOKEN_KEY), info['env'].get('GAZEBOTEACH_SESSION_TOKEN')):
            continue
        try:
            if os.path.samefile(info['exe'], str(installed)):
                candidates.append(info)
        except OSError:
            pass
    require(len(candidates) == 1, 'Exactly one actual installed gzserver must own this private master')
    info = candidates[0]
    require(Path(info['exe']).read_bytes()[:4] == b'\x7fELF', 'Actual gzserver is not native ELF')
    require('-u' in info['argv'] and '-p' not in info['argv'] and '-s' not in info['argv'],
            'Trial must use paused physical gzserver without playback/system plugins')
    if input_path is not None:
        require(any(Path(value).resolve() == Path(input_path).resolve() for value in info['argv'] if value.endswith('.sdf')),
                'Actual gzserver input differs from owned trial input')
    maps = artifacts/(label+'-'+str(info['pid'])+'.maps')
    maps.write_bytes(Path('/proc', str(info['pid']), 'maps').read_bytes())
    libraries = sorted(set(line.split()[-1] for line in maps.read_text().splitlines()
                           if '/libgazebo_' in line and '.so' in line))
    require(any('libgazebo_physics.so' in p for p in libraries)
            and any('libgazebo_transport.so' in p for p in libraries), 'Native physics/transport DSOs absent')
    plugin_maps = sorted(set(line.split()[-1] for line in maps.read_text().splitlines() if 'libfault_force.so' in line))
    return {'pid': info['pid'], 'uid': info['uid'], 'start_ticks': info['start_ticks'],
            'argv': info['argv'], 'exe': info['exe'], 'exe_sha256': sha(info['exe']),
            'maps': str(maps), 'maps_sha256': sha(maps),
            'native_libraries': {p: sha(p) for p in libraries},
            'force_plugin_libraries': {p: sha(p) for p in plugin_maps},
            'token': token, 'master_uri': environment['GAZEBO_MASTER_URI'],
            'framework_token': info['env'].get('GAZEBOTEACH_SESSION_TOKEN'),
            'ign_partition': environment.get('IGN_PARTITION'), 'gz_partition': environment.get('GZ_PARTITION'),
            'home': environment.get('HOME'), 'observed_monotonic_ns': time.monotonic_ns()}


def scalar(native, key, default=0):
    return float(first(native, key, default))


def xyz(native):
    point = first(first(native, 'pose'), 'position')
    return [scalar(point, axis) for axis in ('x', 'y', 'z')]


def stats(native):
    value = first(native, 'sim_time')
    return {'sim_time_ns': int(first(value, 'sec', 0))*1000000000+int(first(value, 'nsec', 0)),
            'iterations': int(first(native, 'iterations', 0)), 'paused': first(native, 'paused') == 'true'}


class Evidence:
    def __init__(self, artifacts, preview_env, preview_token):
        self.artifacts, self.preview_env, self.preview_token = artifacts, preview_env, preview_token
        self.trials, self.tokens, self.counter = {}, set(), 0

    def query(self, binary, environment, label, mode, *arguments):
        self.counter += 1
        path = self.artifacts/('%03d-%s-%s.json' % (self.counter, label, mode))
        command = [str(binary), mode]+list(arguments)
        start = time.monotonic_ns()
        output = subprocess.run(command, env=environment, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, timeout=15)
        rows = []
        for line in output.stdout.splitlines():
            try:
                value = json.loads(line)
            except ValueError:
                continue
            if isinstance(value, dict):
                rows.append(value)
        record = {'command': command, 'started_monotonic_ns': start,
                  'finished_monotonic_ns': time.monotonic_ns(), 'returncode': output.returncode,
                  'stdout': output.stdout, 'stderr': output.stderr, 'events': rows,
                  'binary_sha256': sha(binary)}
        save(path, record)
        require(output.returncode == 0, 'Independent native query failed: '+str(path))
        require(not any(r.get('source') in ('client_error', 'decode_error') for r in rows),
                'Independent native query returned error: '+str(path))
        chosen = [r for r in rows if r.get('source') == ('world_stats' if mode == 'stats' else mode)]
        require(len(chosen) == 1 and chosen[0].get('raw_protobuf'), 'One original native protobuf required: '+str(path))
        row = chosen[0]
        encoded = bytes.fromhex(row.get('serialized_protobuf_hex', ''))
        require(encoded, 'Original serialized native protobuf bytes are absent')
        if mode != 'stats':
            require(bytes.fromhex(row.get('response_protobuf_hex', '')), 'Native response envelope bytes are absent')
        native = parsed_model(row['raw_protobuf'])
        require(native, 'Native protobuf decoder produced no fields')
        return {'artifact': str(path), 'native': native, 'data': row.get('data'),
                'raw_protobuf': row['raw_protobuf'], 'receive_monotonic_ns': row.get('receive_monotonic_ns')}

    def endpoint(self, phase, lab, trial):
        token = trial.token
        self.tokens.add(token)
        record = self.trials.setdefault(token, {'phases': {}, 'directory': str(trial.root)})
        require(phase not in record['phases'], 'Duplicate real paused checkpoint hook')
        require(trial.env['GAZEBO_MASTER_URI'] != self.preview_env['GAZEBO_MASTER_URI'],
                'Fault trial shared preview master')
        require(trial.env.get('GAZEBOTEACH_SESSION_TOKEN') == self.preview_token,
                'Direct API trial did not inherit the owned parent session token')
        for key in ('IGN_PARTITION', 'GZ_PARTITION', 'HOME'):
            require(trial.env.get(key) and trial.env[key] != self.preview_env.get(key),
                    'Fault trial reused preview '+key)
        binary = Path(lab.binary)
        require(binary.read_bytes()[:4] == b'\x7fELF', 'Query helper must be actual compiled ELF')
        before = self.query(binary, trial.env, token+'-'+phase, 'stats')
        cube = self.query(binary, trial.env, token+'-'+phase, 'entity', '--name', MODEL)
        after = self.query(binary, trial.env, token+'-'+phase, 'stats')
        a, b = stats(before['native']), stats(after['native'])
        require(a == b and a['paused'], 'Independent query did not bracket one coherent paused state')
        require(a['sim_time_ns'] == {'initial': 0, 'freefall': 200000000, 'final': 1000000000}[phase]
                and a['iterations']*1000000 == a['sim_time_ns'], 'Real finite tick endpoint differs')
        model = cube['native']
        require(first(model, 'name') == MODEL and first(model, 'is_static', 'false') == 'false',
                'Actual dynamic cube absent')
        links = model.get('link', [])
        require(len(links) == 1, 'Actual cube must have exactly one physical link')
        close(scalar(first(links[0], 'inertial'), 'mass'), 1., 1e-10, 'Actual cube mass differs')
        if phase == 'initial':
            record['process'] = native_identity(trial.env, self.artifacts, token, token, trial.world)
            record['physics'] = self.query(binary, trial.env, token, 'physics')
            record['floor'] = self.query(binary, trial.env, token+'-floor', 'entity', '--name', 'floor')
            record['input_path'], record['input_sha256'] = str(trial.world), sha(trial.world)
        else:
            identity = process_info(record['process']['pid'])
            require(identity and identity['start_ticks'] == record['process']['start_ticks']
                    and identity['env'].get(TOKEN_KEY) == token, 'Physical trial process changed between endpoints')
            require(record['input_sha256'] == sha(trial.world), 'Trial input changed during physical steps')
        record['phases'][phase] = {'stats': a, 'position_m': xyz(model), 'model': model,
                                   'before': before, 'entity': cube, 'after': after}


def shape_size(model):
    link = model['link'][0]
    collision = link.get('collision', [])
    if not collision:
        return None
    require(len(collision) == 1, 'Unexpected actual collision topology')
    geometry = first(collision[0], 'geometry')
    box = first(geometry, 'box')
    require(box is not None, 'Expected actual box collision geometry')
    size = first(box, 'size')
    return [scalar(size, a) for a in ('x', 'y', 'z')]


def physical_check(record, expected):
    phases = record['phases']
    require(set(phases) == {'initial', 'freefall', 'final'}, 'Missing actual physical endpoints')
    initial, falling, final = [phases[key] for key in ('initial', 'freefall', 'final')]
    require(max(abs(a-b) for a, b in zip(initial['position_m'], (0, 0, 2))) < 1e-9,
            'Actual initial pose differs')
    ids = [first(phases[p]['model'], 'id') for p in ('initial', 'freefall', 'final')]
    require(ids[0] and len(set(ids)) == 1, 'Actual cube was replaced during trial')
    physics = record['physics']['native']
    close(scalar(physics, 'max_step_size'), .001, 1e-12, 'Actual dt differs')
    close(scalar(physics, 'real_time_update_rate'), 1000., 1e-12, 'Actual private trial rate cap differs')
    gravity = first(physics, 'gravity')
    actual_gravity = [scalar(gravity, axis) for axis in ('x', 'y', 'z')]
    expected_gravity = [0, 0, 0 if expected == 'zero-gravity' else -9.8]
    require(max(abs(a-b) for a, b in zip(actual_gravity, expected_gravity)) < 1e-10,
            'Actual native Physics gravity differs')
    collision = shape_size(initial['model'])
    require((collision is None) == (expected == 'missing-collision'), 'Actual native collision presence differs')
    if collision is not None:
        require(max(abs(value-.2) for value in collision) < 1e-10, 'Actual cube collision size differs')
    floor = record['floor']['native']
    require(first(floor, 'is_static') == 'true' and shape_size(floor), 'Actual static physical floor absent')
    close(xyz(floor)[2]+shape_size(floor)[2]/2, 0., 1e-10, 'Actual physical floor top differs')
    plugin = record['process']['force_plugin_libraries']
    require(bool(plugin) == (expected != 'missing-plugin'), 'Actual force-plugin native mapping differs')
    expected_x = 0. if expected == 'missing-plugin' else .0201
    expected_z = 2. if expected == 'zero-gravity' else 1.80302
    close(falling['position_m'][0], expected_x, 3e-6, 'Actual horizontal acceleration differs')
    close(falling['position_m'][2], expected_z, 3e-6, 'Actual vertical freefall differs')
    close(falling['position_m'][1], 0., 1e-8, 'Unexpected actual lateral motion')
    final_z = 2. if expected == 'zero-gravity' else (-2.9049 if expected == 'missing-collision' else .1)
    close(final['position_m'][2], final_z, .015 if final_z == .1 else 5e-6,
          'Actual final support/fall outcome differs')
    if expected == 'missing-plugin':
        close(final['position_m'][0], 0., 1e-6, 'Missing plugin unexpectedly moved the cube horizontally')
    elif expected in ('zero-gravity', 'missing-collision'):
        close(final['position_m'][0], .5005, 5e-6, 'Actual continued horizontal force integration differs')
    else:
        require(final['position_m'][0] > .18, 'Actual healthy force produced no sustained horizontal motion')
    return {'passed': True, 'observed_fault': expected, 'token': record['process']['token'],
            'model_id': ids[0], 'positions_m': {p: phases[p]['position_m'] for p in phases},
            'native_gravity_m_s2': actual_gravity, 'actual_collision_size_m': collision,
            'force_plugin_mapped': bool(plugin), 'live_native_process': record['process']}


def xml_feature(path):
    root = ET.parse(str(path)).getroot()
    world = root.find('world')
    model = world.find("model[@name='cube_000']")
    require(world is not None and model is not None, 'Expected standalone teaching world/cube')
    link = model.find('link')
    gravity = [float(v) for v in world.findtext('gravity', '').split()]
    plugin = model.find('plugin')
    return {'root': root, 'world': world, 'model': model, 'link': link, 'gravity': gravity,
            'plugin': plugin, 'collision': link.find('collision')}


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(sorted((canonical(child) for child in element), key=repr)))


def repair_check(bad, fixed, case, healthy, original_sha):
    require(sha(bad) == original_sha, 'Repair overwrote the original defective learner input')
    require(fixed.is_file() and fixed.resolve() != bad.resolve() and sha(fixed) != original_sha,
            'Repair did not create distinct corrected source bytes')
    feature = xml_feature(fixed)
    close(float(feature['link'].findtext('inertial/mass')), 1., 1e-12, 'Repair changed cube mass')
    require(feature['gravity'] == [0., 0., -9.8] and feature['collision'] is not None,
            'Corrected XML lacks healthy gravity/collision')
    require(feature['plugin'] is not None and feature['plugin'].get('filename') == 'libfault_force.so',
            'Corrected XML lacks actual private native force plugin')
    require(canonical(feature['root']) == canonical(xml_feature(healthy)['root']),
            'Repair changed unrelated healthy XML semantics')
    if case != 'malformed-sdf':
        previous = xml_feature(bad)
        if case == 'zero-gravity':
            require(previous['gravity'] == [0., 0., 0.], 'Injected source did not contain zero gravity')
        elif case == 'missing-collision':
            require(previous['collision'] is None, 'Injected source did not lack collision')
        else:
            require(previous['plugin'] is not None and previous['plugin'].get('filename') != 'libfault_force.so',
                    'Injected source did not reference a different missing plugin')
    else:
        try:
            ET.parse(str(bad))
        except ET.ParseError:
            pass
        else:
            raise AssertionError('Injected malformed SDF is valid XML')
    diff = ''.join(difflib.unified_diff(bad.read_text().splitlines(True), fixed.read_text().splitlines(True),
                                        fromfile=str(bad), tofile=str(fixed)))
    diff_path = fixed.with_suffix('.source.diff')
    diff_path.write_text(diff, encoding='utf-8')
    return {'bad_input': str(bad), 'bad_sha256': original_sha, 'fixed_input': str(fixed),
            'fixed_sha256': sha(fixed), 'source_diff': str(diff_path), 'original_preserved': True,
            'corrected_xml_matches_healthy_semantics': True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    output = args.report.expanduser().resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(value for value in lessons if value.key == 'classic11-noetic:35')
    source, started, session, evidence = hashes(lesson.folder), time.monotonic(), None, None
    result = {'passed': False, 'lesson': lesson.key, 'scenarios': [], 'source_sha256': source,
              'acceptance_source_sha256': sha(Path(__file__)), 'test_algorithm': __doc__,
              'graphical_checks_performed': False, 'cleanup_errors': [],
              'official_sdk_sources': [SDK_BASE+p for p in ('gazebo/physics/World.cc', 'gazebo/physics/Link.cc',
                  'gazebo/msgs/model.proto', 'gazebo/msgs/physics.proto', 'gazebo/msgs/world_stats.proto')]}
    previous_directory = Path.cwd()
    previous_token = os.environ.get('GAZEBOTEACH_SESSION_TOKEN')
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l35-independent-') as temporary:
        try:
            storage = Storage(Path(temporary)/'preview')
            workspace = storage.prepare(lesson)
            session = LessonSession(ROOT, lesson, storage=storage)
            session.start(gui=False, terminals=False)
            os.chdir(str(workspace)); sys.path.insert(0, str(workspace))
            clinic = importlib.import_module('clinic')
            lab_module = importlib.import_module('lab')
            config = importlib.import_module('config')
            lab = lab_module.Lab(session.env)
            evidence = Evidence(artifacts, session.env, session.token)
            # Match the public CLI environment while retaining a synchronous
            # hook; every native child inherits the normal parent owner token.
            os.environ['GAZEBOTEACH_SESSION_TOKEN'] = session.token
            # The remainder intentionally follows the public clinic functions;
            # schemas are asserted at the call site when source is frozen.
            run_checks(clinic, lab, config, session, evidence, artifacts, result)
            result['passed'] = True
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
            result['traceback'] = traceback.format_exc()
        finally:
            os.chdir(str(previous_directory))
            if previous_token is None:
                os.environ.pop('GAZEBOTEACH_SESSION_TOKEN', None)
            else:
                os.environ['GAZEBOTEACH_SESSION_TOKEN'] = previous_token
            if evidence is not None:
                result['independent_native_evidence'] = evidence.trials
            if session is not None:
                try:
                    copy = artifacts/'course-workspace'
                    shutil.copytree(str(session.workspace), str(copy), ignore=shutil.ignore_patterns('__pycache__'), dirs_exist_ok=True)
                    result['preserved_workspace'] = str(copy)
                except Exception as error:
                    result['cleanup_errors'].append('Evidence preservation: '+repr(error))
                try:
                    session.stop(reason='Independent L35 real fault/repair acceptance cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/'runtime.log'))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/'session.json'))
                except Exception as error:
                    result['cleanup_errors'].append('Owned preview cleanup: '+repr(error))
                tokens = {session.token} | (evidence.tokens if evidence is not None else set())
                result['leftovers'] = {token: token_processes(token) for token in sorted(tokens)}
            else:
                result['leftovers'] = {}
            result['source_templates_unchanged'] = source == hashes(lesson.folder)
            result['elapsed_seconds'] = round(time.monotonic()-started, 3)
            if result['cleanup_errors'] or any(result['leftovers'].values()) or not result['source_templates_unchanged']:
                result['passed'] = False
            save(output, result)
    print(json.dumps({k: value for k, value in result.items() if k not in
                     ('scenarios', 'source_sha256', 'independent_native_evidence', 'prepared_input')},
                     ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


def run_checks(clinic, lab, config, session, evidence, artifacts, result):
    workspace = session.workspace
    record = json.loads((workspace/'build/clinic_lab.json').read_text())
    require(record == config.prepared(workspace), 'Prepared build identity differs')
    require(record['gazebo_version'] == '11.15.1', 'Acceptance is validated on exact Classic11.15.1')
    require(all(result['source_sha256'][name] == digest == sha(workspace/name)
                for name, digest in record['source_sha256'].items()), 'Prepared sources differ from original templates')
    require(record['world_sha256'] == result['source_sha256']['world.sdf'] == sha(workspace/'world.sdf'),
            'Prepared healthy world source differs')
    require(record['observer_sha256'] == sha(lab.binary)
            and record['plugin_sha256'] == sha(workspace/'build/libfault_force.so'), 'Compiled product bytes differ')
    require(Path(lab.binary).read_bytes()[:4] == b'\x7fELF'
            and (workspace/'build/libfault_force.so').read_bytes()[:4] == b'\x7fELF', 'Actual native build products absent')
    require(all(Path(module.__file__).resolve().parent == workspace.resolve() for module in (clinic, config)),
            'Acceptance imported a module outside the private course workspace')
    original_guard = (ROOT/'courses/classic11_noetic/34-performance-batch/observer.cc').read_text()
    current_observer = (workspace/'observer.cc').read_text()
    def guard(text):
        return text[text.index('class MainThreadIOOwner {'):text.index('int main(')]
    require(guard(original_guard) == guard(current_observer), 'Standalone exact-SDK IOManager guard differs from L34')
    result['prepared_input'] = record
    result['standalone_io_guard_exactly_preserved'] = True
    healthy = artifacts/('input-'+uuid.uuid4().hex+'.sdf')
    healthy.write_bytes((workspace/'world.sdf').read_bytes())
    healthy_sha = sha(healthy)
    learner = workspace/'learner-notes.txt'
    learner.write_text('Independent learner note '+uuid.uuid4().hex+'\n', encoding='utf-8')
    learner_sha = sha(learner)

    def preview(label):
        binary, env = lab.binary, session.env
        before = evidence.query(binary, env, label, 'stats')
        model = evidence.query(binary, env, label, 'entity', '--name', MODEL)
        world = evidence.query(binary, env, label, 'world')
        physics = evidence.query(binary, env, label, 'physics')
        after = evidence.query(binary, env, label, 'stats')
        a, b = stats(before['native']), stats(after['native'])
        require(a == b and a == {'sim_time_ns': 0, 'iterations': 0, 'paused': True},
                'Actual independent preview must stay paused at zero')
        require(first(world['native'], 'name') == WORLD, 'Actual preview world differs')
        require(max(abs(a-b) for a, b in zip(xyz(model['native']), (0, 0, 2))) < 1e-9,
                'Actual default preview cube pose differs')
        require(shape_size(model['native']) == [.2, .2, .2], 'Actual default preview collision absent')
        gravity = first(physics['native'], 'gravity')
        close(scalar(gravity, 'z'), -9.8, 1e-10, 'Actual healthy preview gravity differs')
        close(scalar(physics['native'], 'real_time_update_rate'), 1000., 1e-10, 'Actual healthy preview rate differs')
        identity = native_identity(env, artifacts, label, session.token)
        require(set(identity['force_plugin_libraries'].values()) == {record['plugin_sha256']},
                'Actual healthy preview lacks the compiled force ELF')
        return {'stats': a, 'model': model['native'], 'world': world['native'], 'physics': physics['native'],
                'actual_process': identity}

    def observe(path, directory, expected_code, expected_case):
        before_tokens = set(evidence.trials)
        start = time.monotonic_ns()
        diagnosis = clinic.diagnose(path, directory, checkpoint_hook=evidence.endpoint)
        finish = time.monotonic_ns()
        save(directory/'acceptance-returned-diagnosis.json', diagnosis)
        observation = diagnosis['observation']
        require(diagnosis['diagnosis']['code'] == expected_code and diagnosis['diagnosis']['evidence'],
                'Evidence-based diagnosis differs on opaque renamed input: '+str(path))
        require(diagnosis['input_sha256'] == sha(path) and observation['input_preserved'],
                'Diagnosis did not preserve the exact input bytes')
        require(observation['completed'] and observation['ownership']['cleaned']
                and not observation['ownership']['remaining'], 'Actual owned trial failed or left descendants')
        token = observation['ownership']['token']
        evidence.tokens.add(token)
        require(not token_processes(token), 'Native trial ownership token survived its context')
        require(Path(observation['directory']).resolve() == directory.resolve(), 'Trial output escaped selected private directory')
        actual_input = directory/'world.sdf'
        require(sha(actual_input) == sha(path), 'Actual native input bytes differ from diagnosed input')
        parser = observation['parser']
        require(parser['tool_identity'] == record['parser_identity']
                and parser['command'] == [record['parser_identity']['path'], 'sdf', '-k', str(actual_input)],
                'Native parser command/identity differs')
        if expected_case == 'malformed-sdf':
            # This exact native gz sdf CLI returns 0 even for malformed XML.
            # Its actual parser diagnostics, success marker and independent
            # XML parser establish rejection; exit status alone cannot do so.
            native_output = parser['stdout']+'\n'+parser['stderr']
            require(not parser['valid'] and not observation['xml_valid']
                    and 'Error parsing XML' in native_output and 'SDF parsing the xml failed' in native_output
                    and 'Check complete' not in native_output,
                    'Malformed input lacks actual native parser rejection diagnostics')
            require(set(evidence.trials) == before_tokens and not observation['ownership']['processes'],
                    'Rejected malformed XML unexpectedly started a physical server')
            physical = {'passed': True, 'actual_native_parser_rejected': True,
                        'parser_log': str(directory/'parser.log'), 'actual_parser_returncode': parser['returncode'],
                        'physical_server_started': False}
        else:
            require(parser['valid'] and parser['returncode'] == 0 and observation['xml_valid'],
                    'Valid XML was not accepted by actual native parser')
            require('Check complete' in parser['stdout'] and 'SDF parsing the xml failed' not in parser['stderr'],
                    'Valid input lacks actual native parser success marker')
            require(set(evidence.trials)-before_tokens == {token}, 'Expected exactly one new actual physical server')
            native = evidence.trials[token]
            require(start <= native['process']['observed_monotonic_ns'] <= finish, 'Live process proof escaped actual call interval')
            physical = physical_check(native, expected_case)
            require(all(value == record['plugin_sha256'] for value in native['process']['force_plugin_libraries'].values()),
                    'Actual mapped plugin differs from the compiled source product')
            require(native['input_sha256'] == sha(path), 'Live server proof observed different input bytes')
            require(observation['plugin_maps']['loaded_expected'] == bool(native['process']['force_plugin_libraries']),
                    'Course plugin mapping claim differs from independent /proc reading')
            for phase in ('initial', 'freefall', 'final'):
                observed = native['phases'][phase]
                reported = observation[phase]
                require(observed['stats']['sim_time_ns'] == reported['sim_time_ns']
                        and observed['stats']['iterations'] == reported['iterations'], 'Reported native endpoint differs')
                require(max(abs(a-b) for a, b in zip(observed['position_m'], reported['positions_m'][MODEL])) < 1e-12,
                        'Reported position differs from independently queried native Model')
            rows = [json.loads(line) for line in Path(observation['native_events_file']).read_text().splitlines() if line.strip()]
            controls = []
            readiness_failures = []
            for row in rows:
                require(start <= row['started_monotonic_ns'] < row['finished_monotonic_ns'] <= finish,
                        'Native query journal timing leaves actual diagnosis call')
                if row['returncode'] != 0:
                    require(row['mode'] in ('stats', 'world', 'entity', 'physics')
                            and row['finished_monotonic_ns'] < native['process']['observed_monotonic_ns'],
                            'Only initial read-only readiness queries may fail before a proven native endpoint')
                    readiness_failures.append({'mode': row['mode'], 'returncode': row['returncode'],
                                               'finished_monotonic_ns': row['finished_monotonic_ns']})
                if row['mode'] == 'control':
                    values = [v for v in row['events'] if v.get('source') == 'control_sent']
                    require(len(values) == 1, 'Native control journal lacks exactly one actual request')
                    require(bytes.fromhex(values[0]['serialized_protobuf_hex']), 'Serialized control request absent')
                    controls.append(parsed_model(values[0]['raw_protobuf']))
            require(len(controls) == 2, 'One-shot native finite controls must be published exactly twice')
            for control, count in zip(controls, (200, 800)):
                require(first(control, 'pause') == 'true' and int(first(control, 'multi_step')) == count
                        and 'reset' not in control, 'Actual native control sequence differs')
            physical['actual_native_control_counts'] = [200, 800]
            physical['initial_readiness_failures'] = readiness_failures
            if expected_case == 'missing-plugin':
                log = (directory/'gzserver.out').read_text(errors='replace')
                bad_filename = xml_feature(path)['plugin'].get('filename')
                require(bad_filename in log and ('Failed to load plugin' in log or 'cannot open shared object file' in log),
                        'Missing-plugin diagnosis lacks actual native loader-error log')
        return diagnosis, physical

    before_preview = preview('before-clinic')
    healthy_report, healthy_physics = observe(healthy, artifacts/'healthy-control', 'HEALTHY', 'healthy')
    result['healthy_control'] = healthy_physics
    codes = dict(zip(CASES, ('MALFORMED_SDF', 'PLUGIN_LOAD_FAILURE', 'GRAVITY_DISABLED', 'COLLISION_MISSING')))
    for case in CASES:
        # Neither input basename nor output directory reveals the expected case.
        bad = artifacts/('input-'+uuid.uuid4().hex+'.sdf')
        fixed = artifacts/('input-'+uuid.uuid4().hex+'.sdf')
        bad.write_bytes(clinic.inject(healthy.read_bytes(), case))
        bad_sha = sha(bad)
        diagnosis, bad_physics = observe(bad, artifacts/('trial-'+uuid.uuid4().hex), codes[case], case)
        change = clinic.repair(bad, fixed, diagnosis)
        repair_source = repair_check(bad, fixed, case, healthy, bad_sha)
        require(change['source_preserved'] and change['output_sha256'] == sha(fixed),
                'Public repair report differs from actual new source bytes')
        repaired_report, good_physics = observe(fixed, artifacts/('trial-'+uuid.uuid4().hex), 'HEALTHY', 'healthy')
        require(sha(bad) == bad_sha and sha(healthy) == healthy_sha
                and sha(workspace/'world.sdf') == healthy_sha and sha(learner) == learner_sha,
                'Clinic changed bad input, original healthy reference, preview source or learner notes')
        result['scenarios'].append({'case': case, 'passed': True, 'diagnosis_code': codes[case],
            'opaque_input': str(bad), 'bad_actual_evidence': bad_physics, 'repair': change,
            'repair_source': repair_source, 'repaired_actual_evidence': good_physics,
            'before_report': diagnosis['report_file'], 'after_report': repaired_report['report_file']})
        print(json.dumps({'scenario': case, 'passed': True}), flush=True)
    require(len(evidence.trials) == 8 and len(evidence.tokens) == 9,
            'Expected eight separate physical servers and one native-parser-only case')
    for key in ('token', 'ign_partition', 'gz_partition', 'home'):
        values = [row['process'][key] for row in evidence.trials.values()]
        require(len(set(values)) == len(values), 'Fresh isolated trial reused '+key)
    after_preview = preview('after-clinic')
    require(all(before_preview[key] == after_preview[key] for key in ('stats', 'model', 'world', 'physics'))
            and before_preview['actual_process']['pid'] == after_preview['actual_process']['pid']
            and before_preview['actual_process']['start_ticks'] == after_preview['actual_process']['start_ticks'],
            'Fault diagnosis/repair changed or replaced the paused preview world')
    result['preview_isolation'] = {'passed': True, 'before': before_preview, 'after': after_preview}
    result['learner_files_preserved'] = {'healthy_sha256': healthy_sha, 'learner_note_sha256': learner_sha,
                                        'passed': True}
    result['actual_physical_server_count'] = len(evidence.trials)
    result['case_identification_from_input_path'] = False


if __name__ == '__main__':
    raise SystemExit(main())
