#!/usr/bin/env python3
"""Independent L29 real-window pixels and native physics isolation checks.

QScreen captures only this session's actual gzclient X11 window. Colour masks
and connected components analyse those original pixels; the plugin's capture
command, API colour readback and fabricated images are never visual oracles.
Native entity protobuf and actual library mappings independently distinguish
client appearance from server collision/pose. No expected image is generated.
"""
import argparse
from collections import deque
import hashlib
import json
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
from tests.ui_probe import gazebo_window_id

WORLD, BEACON, REFERENCE = 'lesson29', 'beacon', 'reference'
STRIDE = 2


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


def run(session, argv, output, timeout=18):
    command = shlex.join([str(v) for v in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L29 native command', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    code = run(session, [session.workspace/'build/visual_observer', mode, *arguments], path)
    require(code == 0, 'Actual native command failed: '+str(path))
    rows, diagnostics = [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except ValueError:
            diagnostics.append(line)
            continue
        require(isinstance(value, dict) and value.get('source') not in ('client_error', 'decode_error'),
                'Native wire failed: '+repr(value))
        rows.append(value)
    expected = {'snapshot': 'snapshot', 'command': 'snapshot', 'entity': 'entity',
                'world': 'world', 'stats': 'world_stats', 'control': 'control_sent'}[mode]
    values = [row for row in rows if row.get('source') == expected]
    require(len(values) == 1, 'Expected one '+expected+' result from '+str(path))
    result = values[0]
    result['_wire_artifact'], result['_wire_sha256'] = str(path), sha(path)
    result['_wire_non_json_lines'] = diagnostics
    return result


def plugin(row):
    require(row.get('request_id') == row.get('response_id') and row.get('payload', {}).get('query_nonce'),
            'Visual RPC is missing matching ID/nonce')
    value = row['payload']
    require(value.get('world') == WORLD, 'Visual reply belongs to another world')
    require(value.get('load_count') == 1 and value.get('init_count', 0) >= 1,
            'Actual VisualPlugin Load/Init has not completed')
    return value


def stats(row):
    native = parsed_model(row['raw_protobuf'])
    clock = first(native, 'sim_time')
    require(isinstance(clock, dict), 'Actual native simulation clock absent')
    return {'sim_time_ns': int(first(clock, 'sec', '0'))*1000000000+int(first(clock, 'nsec', '0')),
            'iterations': int(first(native, 'iterations')), 'paused': first(native, 'paused') == 'true',
            'raw_wire': row}


def entities(session, artifacts, label):
    result = {}
    for name in (BEACON, REFERENCE):
        row = wire(session, artifacts, label+'-'+name, 'entity', '--name', name)
        native = parsed_model(row['raw_protobuf'])
        require(first(native, 'name') == name, 'Native entity name differs')
        result[name] = native
    return result


def physical_box(model):
    links = model.get('link', [])
    require(len(links) == 1, 'Beacon native body link count differs')
    collisions = links[0].get('collision', [])
    require(len(collisions) == 1, 'Beacon collision count differs')
    geometry = first(collisions[0], 'geometry')
    box = first(geometry, 'box')
    size = first(box, 'size')
    require(isinstance(size, dict), 'Beacon has no actual native box collision')
    dimensions = [float(first(size, axis)) for axis in ('x', 'y', 'z')]
    require(all(abs(v-1) < 1e-12 for v in dimensions), 'Actual server collision is no longer the 1m cube')
    return dimensions


def mapped_library(session, artifacts, label):
    library = (session.workspace/'build/lib/libgazeboteach_l29_visual.so').resolve()
    info = library.stat()
    found, server_hits = [], []
    for pid in live_token_processes(session.token):
        entry = Path('/proc')/str(pid)
        try:
            role = (entry/'comm').read_text().strip()
            if role not in ('gzserver', 'gzclient'):
                continue
            matches = []
            for line in (entry/'maps').read_text().splitlines():
                parts = line.split(None, 5)
                if len(parts) == 6 and Path(parts[5]).resolve() == library:
                    major, minor = [int(v, 16) for v in parts[3].split(':')]
                    require(int(parts[4]) == info.st_ino and (major, minor) == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'Mapped VisualPlugin inode/device differs from prepared ELF')
                    matches.append(line)
            if matches:
                target = artifacts/(label+'-'+role+'-'+str(pid)+'.maps')
                target.write_text('\n'.join(matches)+'\n', encoding='utf-8')
                item = {'pid': pid, 'process': role, 'path': str(library), 'library_sha256': sha(library),
                        'maps': str(target), 'maps_sha256': sha(target)}
                (found if role == 'gzclient' else server_hits).append(item)
        except OSError:
            continue
    require(len(found) == 1 and not server_hits, 'VisualPlugin must map in exactly one owned gzclient and no gzserver')
    return {'client_mapping': found[0], 'server_mappings': server_hits}


def pump(application, seconds=.12):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        application.processEvents()
        time.sleep(.015)


def largest_component(points, columns, stride, x0, y0):
    remaining, largest = set(points), []
    while remaining:
        seed = remaining.pop()
        queue, component = deque([seed]), [seed]
        while queue:
            point = queue.popleft()
            x, y = point % columns, point // columns
            for dx, dy in ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)):
                nx, ny = x+dx, y+dy
                if nx < 0 or nx >= columns or ny < 0:
                    continue
                other = ny*columns+nx
                if other in remaining:
                    remaining.remove(other)
                    queue.append(other)
                    component.append(other)
        if len(component) > len(largest):
            largest = component
    if not largest:
        return {'sampled_pixels': 0, 'estimated_pixels': 0, 'bbox_xyxy': None}
    xs, ys = [p % columns for p in largest], [p // columns for p in largest]
    return {'sampled_pixels': len(largest), 'estimated_pixels': len(largest)*stride*stride,
            'bbox_xyxy': [x0+min(xs)*stride, y0+min(ys)*stride,
                          x0+(max(xs)+1)*stride, y0+(max(ys)+1)*stride]}


def image_analysis(image):
    # RGB bytes come from Qt's actual screenshot; predicates never create pixels.
    from PyQt5 import QtGui
    image = image.convertToFormat(QtGui.QImage.Format_RGB888)
    width, height = image.width(), image.height()
    raw = image.constBits().asstring(image.byteCount())
    line = image.bytesPerLine()
    x0, x1, y0, y1 = int(width*.12), int(width*.985), int(height*.08), int(height*.92)
    columns = len(range(x0, x1, STRIDE))
    masks = {name: set() for name in ('red', 'blue', 'green', 'yellow')}
    for iy, y in enumerate(range(y0, y1, STRIDE)):
        offset = y*line
        for ix, x in enumerate(range(x0, x1, STRIDE)):
            start = offset+x*3
            red, green, blue = raw[start], raw[start+1], raw[start+2]
            index = iy*columns+ix
            if red > 90 and red > green*1.6 and red > blue*1.6:
                masks['red'].add(index)
            elif blue > 90 and blue > red*1.5 and blue > green*1.3:
                masks['blue'].add(index)
            elif green > 80 and green > red*1.6 and green > blue*1.3:
                masks['green'].add(index)
            elif red > 90 and green > 75 and blue < min(red, green)*.65 and abs(red-green) < max(red, green)*.65:
                masks['yellow'].add(index)
    return {'width': width, 'height': height, 'viewport_crop_xyxy': [x0, y0, x1, y1],
            'sampling_stride_pixels': STRIDE,
            'components': {name: largest_component(values, columns, STRIDE, x0, y0) for name, values in masks.items()},
            'method': 'Largest 8-connected saturated-colour region in original QScreen RGB bytes; cropped UI chrome and isolated axes are not target measurements.'}


def capture(application, session, artifacts, label, colour):
    from PyQt5 import QtGui
    identity, candidates = gazebo_window_id(session)
    wrapper = QtGui.QWindow.fromWinId(identity)
    wrapper.showNormal()
    wrapper.raise_()
    wrapper.requestActivate()
    pump(application, .25)
    deadline, attempts = time.monotonic()+10, []
    while True:
        require(session.gui.poll() is None, 'Owned Gazebo client exited before pixel capture')
        pixmap = application.primaryScreen().grabWindow(identity)
        require(not pixmap.isNull() and pixmap.width() > 300 and pixmap.height() > 200, 'Actual X11 screenshot is empty')
        target = artifacts/(label+'-{:02d}.png'.format(len(attempts)))
        require(pixmap.save(str(target), 'PNG'), 'Could not save original native-window PNG')
        analysis = image_analysis(pixmap.toImage())
        chosen, reference = analysis['components'][colour], analysis['components']['yellow']
        attempts.append({'png': str(target), 'sha256': sha(target), 'analysis': analysis})
        if chosen['estimated_pixels'] >= 800 and reference['estimated_pixels'] >= 160:
            break
        require(time.monotonic() < deadline, 'Expected real '+colour+' beacon/yellow reference not visible: '+str(target))
        pump(application, .15)
    return {'passed': True, 'colour': colour, 'original_png': str(target), 'original_png_sha256': sha(target),
            'xid': hex(identity), 'window_candidates': candidates, 'attempts': attempts,
            'target_to_reference_area_ratio': chosen['estimated_pixels']/reference['estimated_pixels'],
            'reference_fraction_of_image': reference['estimated_pixels']/(analysis['width']*analysis['height']),
            'target_component': chosen, 'reference_component': reference,
            'image_dimensions': [analysis['width'], analysis['height']]}


def command(session, artifacts, label, action, *arguments):
    accepted = plugin(wire(session, artifacts, label+'-queued', 'command', '--action', action, *arguments))
    sequence = accepted.get('command_seq', accepted.get('queued_command_seq'))
    require(isinstance(sequence, int), 'Visual command has no queued sequence identifier')
    deadline, number = time.monotonic()+8, 0
    while True:
        actual = plugin(wire(session, artifacts, label+'-applied-'+str(number), 'snapshot'))
        if actual.get('applied_command_seq', -1) >= sequence:
            return {'accepted': accepted, 'applied': actual, 'sequence': sequence}
        require(time.monotonic() < deadline, 'Queued visual command did not execute in render thread; request not replayed')
        number += 1


def reject_saved_input(session, artifacts, config):
    original = (session.workspace/'parameters.json').read_bytes()
    before = stats(wire(session, artifacts, 'saved-before-world', 'stats'))
    source = entities(session, artifacts, 'saved-before-native')
    log = artifacts/'saved-unprepared.log'
    try:
        # A real supported changed field exercises the source/config guard,
        # rather than merely rejecting an unknown key in the JSON schema.
        changed = dict(config, visual_scale=1.1)
        save(session.workspace/'parameters.json', changed)
        status = run(session, ['python3', 'lab.py', 'ready-client'], log)
        require(status != 0, 'Unprepared visual-scale input silently accepted')
        text = log.read_text(encoding='utf-8').lower()
        require('reload' in text or 'restart' in text, 'Stale visual input lacks a reload diagnosis')
        after = stats(wire(session, artifacts, 'saved-after-world', 'stats'))
        require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations'], 'Rejected stale visual input advanced physics')
        require(entities(session, artifacts, 'saved-after-native') == source, 'Rejected input changed native physics')
    finally:
        (session.workspace/'parameters.json').write_bytes(original)
    repaired = artifacts/'saved-exact-repair.log'
    require(run(session, ['python3', 'lab.py', 'ready-client'], repaired) == 0, 'Exact original input did not restore visual readiness')
    return {'passed': True, 'before': before, 'after': after, 'negative_log': str(log),
            'original_sha256': hashlib.sha256(original).hexdigest(), 'exact_bytes_restored': (session.workspace/'parameters.json').read_bytes() == original}


def baseline_controls(application, session, artifacts, config):
    before = stats(wire(session, artifacts, 'paused-controls-before', 'stats'))
    physical_before = entities(session, artifacts, 'paused-controls-native-before')
    images, commands = {}, {}
    for colour in ('blue', 'green'):
        commands[colour] = command(session, artifacts, 'colour-'+colour, 'color', '--color', colour)
        images[colour] = capture(application, session, artifacts, 'colour-'+colour, colour)
    commands['reset'] = command(session, artifacts, 'visual-reset', 'reset')
    images['reset'] = capture(application, session, artifacts, 'visual-reset', config['color'])
    after = stats(wire(session, artifacts, 'paused-controls-after', 'stats'))
    require(before['paused'] and after['paused'] and before['sim_time_ns'] == after['sim_time_ns']
            and before['iterations'] == after['iterations'], 'Client visual changes advanced paused server physics')
    require(entities(session, artifacts, 'paused-controls-native-after') == physical_before, 'Visual appearance command changed server entity/collision data')
    require(commands['reset']['applied']['render_count'] > commands['blue']['applied']['render_count'], 'Actual render loop did not continue while physics paused')
    lifecycle = [(item['applied']['load_count'], item['applied']['init_count']) for item in commands.values()]
    require(len(set(lifecycle)) == 1, 'Local appearance commands reran plugin Load/Init')
    old_gui, old_client = session.gui.pid, commands['reset']['applied']['client_id']
    terminate_owned(session.gui)
    require(session.gui.poll() is not None, 'Only-owned GUI close did not complete')
    require(session.server.poll() is None, 'Closing own GUI also stopped server')
    session.open_gui()
    reopened = plugin(wire(session, artifacts, 'reopened-client', 'snapshot'))
    require(session.gui.pid != old_gui and reopened['client_id'] != old_client, 'Reopen did not create a fresh client/plugin instance')
    require((reopened['load_count'], reopened['init_count']) == lifecycle[0],
            'Fresh reopened client initialization differs from the original loader path')
    image = capture(application, session, artifacts, 'reopened-initial-appearance', config['color'])
    require(entities(session, artifacts, 'reopened-native') == physical_before, 'Reopening client changed server physics entities')
    final = stats(wire(session, artifacts, 'reopened-world', 'stats'))
    require(final['paused'] and final['sim_time_ns'] == before['sim_time_ns'], 'Reopen changed paused world time')
    mapping = mapped_library(session, artifacts, 'reopened')
    return {'passed': True, 'world_before': before, 'world_after': after, 'commands': commands, 'actual_pixel_images': images,
            'native_entities_unchanged': True, 'old_client_pid': old_gui, 'old_client_id': old_client,
            'reopened_plugin': reopened, 'reopened_actual_pixels': image, 'reopened_mapping': mapping,
            'world_after_reopen': final}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('QT_QPA_PLATFORM', '').lower() in ('offscreen', 'minimal'):
        raise RuntimeError('L29 pixels require a real graphical session')
    from PyQt5 import QtWidgets
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    require(application.primaryScreen() is not None and os.environ.get('DISPLAY'), 'Actual X11 display is unavailable')
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(item for item in lessons if item.id == '29' and item.profile == 'classic11-noetic')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': True, 'qt_platform': application.platformName(), 'test_algorithm': __doc__}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l29-') as temporary:
        try:
            for name, config in [('baseline', {'color': 'red', 'visual_scale': 1.0}),
                                 ('color_only', {'color': 'blue', 'visual_scale': 1.0}),
                                 ('scale_only', {'color': 'red', 'visual_scale': 1.4})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=True, terminals=False)
                session.pause(True)
                initial = plugin(wire(session, artifacts, name+'-plugin', 'snapshot'))
                require(initial['source_sha256'] == source['visual_plugin.cc'], 'Actually loaded visual source differs')
                require(initial['camera_initialized'] and initial['appearance_applied'], 'Client camera/first appearance was not initialized/read back')
                require(initial['configured_color'] == config['color'] and
                        abs(initial['configured_visual_scale']-config['visual_scale']) < 1e-12,
                        'Loaded client parameters differ from the prepared personal exercise')
                physical = entities(session, artifacts, name+'-native-before')
                collision = physical_box(physical[BEACON])
                mapping = mapped_library(session, artifacts, name)
                image = capture(application, session, artifacts, name+'-actual-initial', config['color'])
                require(entities(session, artifacts, name+'-native-after-pixels') == physical, 'Capturing rendered appearance modified server physics')
                row = {'name': name, 'passed': True, 'parameters': config, 'initial_plugin': initial,
                       'actual_visual_plugin_mapping': mapping, 'native_entities': physical,
                       'actual_server_collision_size_m': collision, 'actual_pixel_image': image}
                if name == 'baseline':
                    row['saved_input_guard'] = reject_saved_input(session, artifacts, config)
                    row['paused_render_controls_and_reopen'] = baseline_controls(application, session, artifacts, config)
                row['prepared_input_sha256'] = hashes(workspace)
                save(artifacts/(name+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/visual_lab.json'), str(artifacts/(name+'-visual_lab.json')))
                result['scenarios'].append(row)
                session.stop(reason='L29 independent rendered case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            baseline, blue, enlarged = result['scenarios']
            require(len({(row['initial_plugin']['load_count'], row['initial_plugin']['init_count'])
                         for row in result['scenarios']}) == 1, 'Fresh clients used inconsistent plugin lifecycle paths')
            base_ratio = baseline['actual_pixel_image']['target_to_reference_area_ratio']
            blue_ratio = blue['actual_pixel_image']['target_to_reference_area_ratio']
            enlarged_ratio = enlarged['actual_pixel_image']['target_to_reference_area_ratio']
            require(.85 < blue_ratio/base_ratio < 1.15, 'Colour-only change altered actual projected geometry beyond raster tolerance')
            require(1.5 < enlarged_ratio/base_ratio < 3.3, 'Visual scale1.4 did not enlarge actual rendered target area while leaving server collision unchanged')
            reference_fractions = [row['actual_pixel_image']['reference_fraction_of_image'] for row in result['scenarios']]
            require(max(reference_fractions)/min(reference_fractions) < 1.15, 'Reference image size drift suggests the camera/window changed during scale comparison')
            result['cross_case_actual_pixels'] = {'baseline_area_ratio': base_ratio, 'blue_area_ratio': blue_ratio,
                'enlarged_area_ratio': enlarged_ratio, 'blue_over_baseline': blue_ratio/base_ratio,
                'enlarged_over_baseline': enlarged_ratio/base_ratio, 'reference_image_fractions': reference_fractions,
                'note': 'All areas measured from actual native-window pixels, normalized by the unchanged yellow reference. Server collision remains 1m in every case.'}
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason='L29 formal graphical cleanup')
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
                     ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
