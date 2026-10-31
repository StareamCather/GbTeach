#!/usr/bin/env python3
"""L30 actual X11 button input, Qt presentation and independent native physics.

Only XTest pointer/button events activate the panel controls in this test. The
panel's RPC supplies GUI-thread-cached rectangles and text, never a substitute
click. Native WorldStatistics and entity protobuf independently prove pause,
one-shot physical stepping and a falling cube. QScreen retains original pixels.
"""
import argparse
import ctypes
import ctypes.util
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

WORLD, MODEL, STEP_NS = 'lesson30', 'falling_cube', 1000000


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


def run(session, argv, output, timeout=16):
    command = shlex.join([str(v) for v in argv])+' > '+shlex.quote(str(output))+' 2>&1'
    process = session._spawn('Independent L30 native query', ['bash', '--noprofile', '--norc', '-c', command])
    try:
        return process.wait(timeout=timeout)
    except BaseException:
        terminate_owned(process)
        raise


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'.jsonl')
    require(run(session, [session.workspace/'build/panel_observer', mode, *arguments], path) == 0,
            'Actual native query failed: '+str(path))
    rows, noise = [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except ValueError:
            noise.append(line)
            continue
        require(isinstance(row, dict) and row.get('source') not in ('client_error', 'decode_error'),
                'Native reader error: '+repr(row))
        rows.append(row)
    expected = {'snapshot': 'snapshot', 'stats': 'world_stats', 'entity': 'entity', 'world': 'world'}[mode]
    values = [v for v in rows if v.get('source') == expected]
    require(len(values) == 1, 'Expected one '+expected+' native result: '+str(path))
    result = values[0]
    result['_wire_artifact'], result['_wire_sha256'], result['_non_json_lines'] = str(path), sha(path), noise
    return result


def stats(row):
    native = parsed_model(row['raw_protobuf'])
    stamp = first(native, 'sim_time')
    require(isinstance(stamp, dict) and first(native, 'paused') in ('true', 'false'), 'Missing native clock or pause')
    return {'sim_time_ns': int(first(stamp, 'sec', '0'))*1000000000+int(first(stamp, 'nsec', '0')),
            'iterations': int(first(native, 'iterations')), 'paused': first(native, 'paused') == 'true',
            'raw_wire': row}


def panel(row):
    require(row.get('request_id') == row.get('response_id') and row.get('payload', {}).get('query_nonce'),
            'Panel query lacks matching native request ID/nonce')
    value = row['payload']
    require(value['world'] == WORLD and value['load_count'] == 1, 'Wrong GUI plugin or incomplete Load')
    require(value.get('init_hook_supported') is False and value.get('reset_hook_supported') is False
            and value['init_count'] == 0 and value['reset_count'] == 0,
            'GUIPlugin has no Init/Reset API here; zero compatibility placeholders must not imply real hooks')
    require(value['stats_received'], 'Qt panel has not received actual world statistics')
    return value


def entity(session, artifacts, label):
    row = wire(session, artifacts, label, 'entity', '--name', MODEL)
    model = parsed_model(row['raw_protobuf'])
    require(first(model, 'name') == MODEL, 'Native reader returned another model')
    return model


def position(model):
    pose = first(model, 'pose')
    value = first(pose, 'position')
    return [float(first(value, k, '0')) for k in ('x', 'y', 'z')]


def collision_size(model):
    links = model.get('link', [])
    require(len(links) == 1 and len(links[0].get('collision', [])) == 1, 'Falling cube native topology changed')
    shape = first(first(first(links[0]['collision'][0], 'geometry'), 'box'), 'size')
    result = [float(first(shape, axis)) for axis in ('x', 'y', 'z')]
    require(all(abs(x-.4) < 1e-12 for x in result), 'Actual cube collision is not .4m')
    return result


def mapped_library(session, artifacts, label):
    library = (session.workspace/'build/lib/libgazeboteach_l30_gui.so').resolve()
    info = library.stat()
    found, servers = [], []
    for pid in live_token_processes(session.token):
        p = Path('/proc')/str(pid)
        try:
            kind = (p/'comm').read_text().strip()
            if kind not in ('gzserver', 'gzclient'):
                continue
            matches = []
            for line in (p/'maps').read_text().splitlines():
                parts = line.split(None, 5)
                if len(parts) == 6 and Path(parts[5]).resolve() == library:
                    device = tuple(int(v, 16) for v in parts[3].split(':'))
                    require(int(parts[4]) == info.st_ino and device == (os.major(info.st_dev), os.minor(info.st_dev)),
                            'Actual mapped GUI plugin inode/device differs')
                    matches.append(line)
            if matches:
                path = artifacts/(label+'-'+kind+'-'+str(pid)+'.maps')
                path.write_text('\n'.join(matches)+'\n', encoding='utf-8')
                value = {'pid': pid, 'path': str(library), 'library_sha256': sha(library),
                         'maps': str(path), 'maps_sha256': sha(path)}
                (found if kind == 'gzclient' else servers).append(value)
        except OSError:
            continue
    require(len(found) == 1 and not servers, 'GUIPlugin must map in exactly one owned client and not server')
    return {'client': found[0], 'server_mappings': servers}


def pump(application, seconds=.1):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        application.processEvents()
        time.sleep(.015)


class XInput:
    """Actual X11 input, restricted to the tracked topmost gzclient window."""
    def __init__(self):
        self.x = ctypes.CDLL(ctypes.util.find_library('X11') or 'libX11.so.6')
        self.t = ctypes.CDLL(ctypes.util.find_library('Xtst') or 'libXtst.so.6')
        display, window, integer, uinteger = ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_uint
        pw, pi, pu = ctypes.POINTER(window), ctypes.POINTER(integer), ctypes.POINTER(uinteger)
        self.x.XOpenDisplay.argtypes, self.x.XOpenDisplay.restype = [ctypes.c_char_p], display
        self.x.XDefaultRootWindow.argtypes, self.x.XDefaultRootWindow.restype = [display], window
        self.x.XDefaultScreen.argtypes, self.x.XDefaultScreen.restype = [display], integer
        self.x.XCloseDisplay.argtypes = [display]
        self.x.XFlush.argtypes = [display]
        self.x.XSync.argtypes = [display, integer]
        self.x.XGetGeometry.argtypes = [display, window, pw, pi, pi, pu, pu, pu, pu]
        self.x.XGetGeometry.restype = integer
        self.x.XTranslateCoordinates.argtypes = [display, window, window, integer, integer, pi, pi, pw]
        self.x.XTranslateCoordinates.restype = integer
        self.x.XQueryPointer.argtypes = [display, window, pw, pw, pi, pi, pi, pi, pu]
        self.x.XQueryPointer.restype = integer
        self.t.XTestQueryExtension.argtypes = [display, pi, pi, pi, pi]
        self.t.XTestQueryExtension.restype = integer
        self.t.XTestFakeMotionEvent.argtypes = [display, integer, integer, integer, ctypes.c_ulong]
        self.t.XTestFakeMotionEvent.restype = integer
        self.t.XTestFakeButtonEvent.argtypes = [display, uinteger, integer, ctypes.c_ulong]
        self.t.XTestFakeButtonEvent.restype = integer
        self.display = self.x.XOpenDisplay(None)
        require(self.display, 'Cannot open actual X11 display')
        self.root, self.screen = self.x.XDefaultRootWindow(self.display), self.x.XDefaultScreen(self.display)
        values = [ctypes.c_int() for _ in range(4)]
        require(self.t.XTestQueryExtension(self.display, *(ctypes.byref(v) for v in values)), 'XTest extension unavailable')

    def geometry(self, identity):
        root, x, y, width, height, border, depth = ctypes.c_ulong(), ctypes.c_int(), ctypes.c_int(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
        require(self.x.XGetGeometry(self.display, identity, ctypes.byref(root), ctypes.byref(x), ctypes.byref(y),
                                   ctypes.byref(width), ctypes.byref(height), ctypes.byref(border), ctypes.byref(depth)),
                'Cannot get owned window geometry')
        rx, ry, child = ctypes.c_int(), ctypes.c_int(), ctypes.c_ulong()
        require(self.x.XTranslateCoordinates(self.display, identity, self.root, 0, 0,
                                            ctypes.byref(rx), ctypes.byref(ry), ctypes.byref(child)), 'Cannot translate owned window origin')
        return {'x': rx.value, 'y': ry.value, 'width': width.value, 'height': height.value}

    def pointer(self, identity):
        root, child, rx, ry, x, y, state = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_int(), ctypes.c_int(), ctypes.c_int(), ctypes.c_int(), ctypes.c_uint()
        require(self.x.XQueryPointer(self.display, identity, ctypes.byref(root), ctypes.byref(child), ctypes.byref(rx),
                                    ctypes.byref(ry), ctypes.byref(x), ctypes.byref(y), ctypes.byref(state)), 'X11 pointer query failed')
        return {'root_x': rx.value, 'root_y': ry.value, 'child': child.value, 'button_mask': state.value}

    def owner_chain(self):
        chain, current = [], self.root
        for _ in range(24):
            chain.append(current)
            child = self.pointer(current)['child']
            if not child:
                break
            current = child
        return chain

    def click(self, identity, rectangle):
        require(rectangle.get('enabled'), 'Refusing to click a disabled panel button')
        bounds = self.geometry(identity)
        x = int(rectangle['x'])+int(rectangle['width'])//2
        y = int(rectangle['y'])+int(rectangle['height'])//2
        require(rectangle['width'] >= 15 and rectangle['height'] >= 12, 'Panel button rectangle is not credible')
        require(bounds['x'] <= x < bounds['x']+bounds['width'] and bounds['y'] <= y < bounds['y']+bounds['height'],
                'Panel button centre is outside this owned gzclient')
        original = self.pointer(self.root)
        require(not original['button_mask'] & (1 | 4 | 8 | 64 | 128 | 256 | 512 | 1024),
                'A shortcut modifier or mouse button is held; do not interfere')
        require(self.t.XTestFakeMotionEvent(self.display, self.screen, x, y, 0), 'XTest motion rejected')
        self.x.XSync(self.display, 0)
        chain = self.owner_chain()
        require(identity in chain, 'Another window obscures the intended owned button; no click sent')
        require(self.t.XTestFakeButtonEvent(self.display, 1, 1, 0), 'XTest press rejected')
        require(self.t.XTestFakeButtonEvent(self.display, 1, 0, 0), 'XTest release rejected')
        self.x.XSync(self.display, 0)
        self.t.XTestFakeMotionEvent(self.display, self.screen, original['root_x'], original['root_y'], 0)
        self.x.XFlush(self.display)
        return {'method': 'XTest actual Button1 press/release; no panel command RPC', 'press_count': 1,
                'release_count': 1, 'screen_position': [x, y], 'owned_xid': hex(identity),
                'topmost_owner_chain_before_press': [hex(v) for v in chain], 'owned_geometry': bounds,
                'cached_qt_button_rectangle': rectangle}

    def close(self):
        if self.display:
            self.x.XCloseDisplay(self.display)
            self.display = None


def activate(application, session):
    from PyQt5 import QtGui
    require(session.gui is not None and session.gui.poll() is None, 'Owned GUI is not running')
    identity, candidates = gazebo_window_id(session)
    wrapped = QtGui.QWindow.fromWinId(identity)
    wrapped.showNormal()
    wrapped.raise_()
    wrapped.requestActivate()
    pump(application, .2)
    return identity, wrapped, candidates


def read_panel(session, artifacts, label):
    return panel(wire(session, artifacts, label, 'snapshot'))


def wait_panel(application, session, artifacts, label, predicate, timeout=8):
    deadline, index = time.monotonic()+timeout, 0
    while True:
        current = read_panel(session, artifacts, label+'-'+str(index))
        require(not current.get('last_error'), 'Actual Qt panel reported: '+str(current.get('last_error')))
        if predicate(current):
            return current
        require(time.monotonic() < deadline, 'Qt panel completion not observed: '+label)
        pump(application, .04)
        index += 1


def screenshot(application, session, input_device, artifacts, label):
    identity, wrapper, candidates = activate(application, session)
    state = wait_panel(application, session, artifacts, label+'-panel', lambda s: not s['busy'])
    bounds = input_device.geometry(identity)
    picture = application.primaryScreen().grabWindow(identity)
    require(not picture.isNull() and picture.width() > 400 and picture.height() > 300, 'QScreen returned no real GUI image')
    path = artifacts/(label+'.png')
    require(picture.save(str(path), 'PNG'), 'Could not preserve actual GUI pixels')
    from PyQt5.QtGui import QImage
    image = picture.toImage().convertToFormat(QImage.Format_RGB888)
    raw = image.bits().asstring(image.byteCount())
    row_stride = image.bytesPerLine()
    visible = {}
    for name, rect in state['buttons'].items():
        x0, y0 = int(rect['x'])-bounds['x'], int(rect['y'])-bounds['y']
        x1, y1 = x0+int(rect['width']), y0+int(rect['height'])
        require(0 <= x0 < x1 <= image.width() and 0 <= y0 < y1 <= image.height(), 'Qt button is outside its actual captured window')
        colours = set()
        for y in range(y0, y1, 2):
            for x in range(x0, x1, 2):
                index = y*row_stride+x*3
                colours.add(raw[index:index+3])
        require(len(colours) >= 3, 'Button rectangle has no visible rendered content: '+name)
        visible[name] = {'window_rectangle': [x0, y0, x1, y1], 'sampled_distinct_rgb_values': len(colours)}
    return {'passed': True, 'original_png': str(path), 'original_png_sha256': sha(path), 'owned_xid': hex(identity),
            'window_candidates': candidates, 'image_dimensions': [image.width(), image.height()], 'visible_button_pixels': visible,
            'cached_gui_text': {k: state[k] for k in ('label_time', 'label_state', 'label_iterations')},
            'note': 'Raw QScreen window image; button regions checked for visible content, not OCR or synthesized text.'}


def mouse_action(application, session, input_device, artifacts, label, action):
    identity, wrapper, candidates = activate(application, session)
    before = wait_panel(application, session, artifacts, label+'-ready',
                        lambda s: not s['busy'] and s['buttons'][action]['enabled'])
    clicked = input_device.click(identity, before['buttons'][action])
    after = wait_panel(application, session, artifacts, label+'-completed',
                       lambda s: not s['busy'] and s['completed_controls'] == before['completed_controls']+1
                       and s.get('last_completed_action') == action)
    require(after['click_counts'][action] == before['click_counts'][action]+1, 'Actual Qt clicked signal count is not one')
    require(after['published_controls'] == before['published_controls']+1, 'One native button click did not produce exactly one publication')
    return {'passed': True, 'before': before, 'after': after, 'actual_mouse_event': clicked}


def paused_labels(application, session, artifacts, label, config, native):
    expected = 't = '+format(native['sim_time_ns']/1e9, '.'+str(config['display_decimals'])+'f')+' s'
    current = wait_panel(application, session, artifacts, label,
                         lambda s: s['paused'] and int(s['sim_time_ns']) == native['sim_time_ns']
                         and int(s['iterations']) == native['iterations'] and s['label_time'] == expected)
    require(current['label_state'] == 'paused' and current['label_iterations'] == 'iterations = '+str(native['iterations']),
            'GUI-thread QLabel text does not agree with independent paused native statistics')
    return {'passed': True, 'expected_from_native_clock': expected, 'actual_cached_label_text': current['label_time'],
            'panel': current}


def reject_saved_input(session, artifacts, config):
    original = (session.workspace/'parameters.json').read_bytes()
    before = stats(wire(session, artifacts, 'saved-before-world', 'stats'))
    model = entity(session, artifacts, 'saved-before-cube')
    log = artifacts/'saved-without-reload.log'
    try:
        save(session.workspace/'parameters.json', dict(config, step_count=150))
        require(run(session, ['python3', 'lab.py', 'ready-panel'], log) != 0, 'Stale step count silently applied')
        require(any(x in log.read_text(encoding='utf-8').lower() for x in ('reload', 'restart')), 'Stale input lacks a reload diagnosis')
        after = stats(wire(session, artifacts, 'saved-after-world', 'stats'))
        require(before['sim_time_ns'] == after['sim_time_ns'] and before['iterations'] == after['iterations'] and after['paused'],
                'Rejected stale input advanced paused physics')
        require(entity(session, artifacts, 'saved-after-cube') == model, 'Rejected configuration changed actual cube')
    finally:
        (session.workspace/'parameters.json').write_bytes(original)
    repair = artifacts/'saved-exact-repair.log'
    require(run(session, ['python3', 'lab.py', 'ready-panel'], repair) == 0, 'Exact bytes did not restore readiness')
    return {'passed': True, 'original_sha256': hashlib.sha256(original).hexdigest(), 'negative_log': str(log),
            'exact_bytes_restored': (session.workspace/'parameters.json').read_bytes() == original, 'before': before, 'after': after}


def case(application, session, input_device, artifacts, name, config, extended):
    initial = stats(wire(session, artifacts, name+'-initial-world', 'stats'))
    require(initial['paused'] and initial['sim_time_ns'] == 0 and initial['iterations'] == 0, 'World did not start actually paused at time zero')
    body_before = entity(session, artifacts, name+'-initial-cube')
    collision_size(body_before)
    require(abs(position(body_before)[2]-2) < 1e-8, 'Fresh actual falling cube is not at z=2m')
    initial_panel = read_panel(session, artifacts, name+'-initial-panel')
    initial_text = paused_labels(application, session, artifacts, name+'-initial-labels', config, initial)
    image = screenshot(application, session, input_device, artifacts, name+'-actual-panel')
    click = mouse_action(application, session, input_device, artifacts, name+'-step', 'step')
    stepped = stats(wire(session, artifacts, name+'-stepped-world', 'stats'))
    count = config['step_count']
    require(stepped['paused'] and stepped['iterations']-initial['iterations'] == count
            and stepped['sim_time_ns']-initial['sim_time_ns'] == count*STEP_NS,
            'Actual clicked step did not produce exactly the configured native ticks; input was not replayed')
    body_after = entity(session, artifacts, name+'-stepped-cube')
    elapsed = count*.001
    predicted_z = 2-.5*9.8*elapsed*elapsed
    actual_z = position(body_after)[2]
    require(abs(actual_z-predicted_z) < .012 and actual_z < 2-.02, 'Actual cube did not fall by the expected native step duration')
    require(first(body_after, 'id') == first(body_before, 'id'), 'Step replaced the real model')
    collision_size(body_after)
    text = paused_labels(application, session, artifacts, name+'-stepped-labels', config, stepped)
    output = {'name': name, 'passed': True, 'parameters': config, 'initial_panel': initial_panel,
              'actual_pixel_image': image, 'initial_native_world': initial, 'initial_native_cube': body_before,
              'initial_label_check': initial_text, 'actual_mouse_step': click, 'stepped_native_world': stepped,
              'stepped_native_cube': body_after, 'native_fall_prediction_m': predicted_z, 'native_actual_height_m': actual_z,
              'native_height_residual_m': actual_z-predicted_z, 'stepped_label_check': text}
    if extended:
        output['saved_input_guard'] = reject_saved_input(session, artifacts, config)
        resumed = mouse_action(application, session, input_device, artifacts, name+'-resume', 'resume')
        running = stats(wire(session, artifacts, name+'-running-world', 'stats'))
        require(not running['paused'] and running['sim_time_ns'] > stepped['sim_time_ns'], 'Actual resume click did not resume the world')
        paused = mouse_action(application, session, input_device, artifacts, name+'-pause', 'pause')
        final = stats(wire(session, artifacts, name+'-paused-world', 'stats'))
        require(final['paused'] and final['sim_time_ns'] > running['sim_time_ns'], 'Actual pause button did not pause an advancing world')
        body_final = entity(session, artifacts, name+'-post-run-cube')
        require(.18 <= position(body_final)[2] < actual_z, 'Real resumed cube did not move down toward its support')
        collision_size(body_final)
        final_text = paused_labels(application, session, artifacts, name+'-post-run-labels', config, final)
        before_reopen = read_panel(session, artifacts, name+'-before-reopen-panel')
        old_pid = session.gui.pid
        terminate_owned(session.gui)
        require(session.gui.poll() is not None and session.server.poll() is None, 'Closing own GUI also stopped server')
        session.open_gui()
        reopened = wait_panel(application, session, artifacts, name+'-reopened-panel', lambda s: s['stats_received'] and not s['busy'])
        require(session.gui.pid != old_pid and reopened['client_id'] != before_reopen['client_id'], 'Reopen did not create a fresh GUI instance')
        require((reopened['load_count'], reopened['init_count']) == (initial_panel['load_count'], initial_panel['init_count']),
                'GUI loader Load/unsupported-hook placeholders differ on reopen')
        after_reopen = stats(wire(session, artifacts, name+'-reopened-world', 'stats'))
        require(after_reopen['paused'] and after_reopen['sim_time_ns'] == final['sim_time_ns']
                and after_reopen['iterations'] == final['iterations'], 'Reopening panel advanced paused physics')
        require(entity(session, artifacts, name+'-reopened-cube') == body_final, 'GUI reopen changed the paused model')
        output['resume_pause_and_reopen'] = {'passed': True, 'resume_mouse': resumed, 'running_world': running,
            'pause_mouse': paused, 'paused_world': final, 'native_cube_after_run': body_final, 'paused_label_check': final_text,
            'old_gui_pid': old_pid, 'old_panel': before_reopen, 'reopened_panel': reopened,
            'reopened_native_world': after_reopen, 'reopened_actual_pixels': screenshot(application, session, input_device, artifacts, name+'-reopened-actual-panel'),
            'reopened_mapping': mapped_library(session, artifacts, name+'-reopened')}
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    require(os.environ.get('QT_QPA_PLATFORM', '').lower() not in ('offscreen', 'minimal') and os.environ.get('DISPLAY'),
            'L30 requires the real X11 desktop')
    from PyQt5 import QtWidgets
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    require(application.platformName() == 'xcb', 'Actual native X11 backend is required for XTest')
    report_path = args.report.expanduser().resolve()
    artifacts = report_path.with_name(report_path.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog error: '+repr(errors))
    lesson = next(v for v in lessons if v.id == '30' and v.profile == 'classic11-noetic')
    source, started, sessions = hashes(lesson.folder), time.monotonic(), []
    result = {'passed': False, 'lesson': lesson.key, 'source_sha256': source, 'scenarios': [],
              'graphical_checks_performed': True, 'qt_platform': application.platformName(), 'test_algorithm': __doc__}
    input_device = None
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l30-') as temporary:
        try:
            input_device = XInput()
            for name, config in [('baseline', {'step_count': 100, 'display_decimals': 3}),
                                 ('steps_only', {'step_count': 200, 'display_decimals': 3}),
                                 ('decimals_only', {'step_count': 100, 'display_decimals': 1})]:
                storage = Storage(Path(temporary)/name)
                workspace = storage.prepare(lesson)
                save(workspace/'parameters.json', config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                result['active_case'] = name
                session.start(gui=True, terminals=False)
                initial = read_panel(session, artifacts, name+'-compiled-panel')
                record = json.loads((workspace/'build/gui_lab.json').read_text(encoding='utf-8'))
                require(initial['source_sha256'] in record['source_sha256'].values(), 'Actual GUI plugin source differs from prepared source hashes')
                require(initial['config_sha256'] == record['config_sha256'] and initial['step_count'] == config['step_count']
                        and initial['display_decimals'] == config['display_decimals'], 'Actual panel input differs from fresh configuration')
                mapping = mapped_library(session, artifacts, name)
                row = case(application, session, input_device, artifacts, name, config, name == 'baseline')
                row['actual_gui_plugin_mapping'], row['prepared_input_sha256'] = mapping, hashes(workspace)
                save(artifacts/(name+'-analysis.json'), row)
                shutil.copy2(str(workspace/'build/gui_lab.json'), str(artifacts/(name+'-gui_lab.json')))
                shutil.copy2(str(workspace/'world.sdf'), str(artifacts/(name+'-generated-world.sdf')))
                result['scenarios'].append(row)
                session.stop(reason='L30 independent actual mouse/physics case complete')
                print(json.dumps({'scenario': name, 'passed': True}), flush=True)
            base, longer, precision = result['scenarios']
            require(longer['native_actual_height_m'] < base['native_actual_height_m']-.1, 'Doubling actual ticks did not increase physical fall')
            require(abs(precision['native_actual_height_m']-base['native_actual_height_m']) < 1e-7,
                    'Changing only GUI text precision altered physical stepping')
            require(base['stepped_label_check']['actual_cached_label_text'] != precision['stepped_label_check']['actual_cached_label_text'],
                    'Display precision did not change the actual QLabel text')
            result['cross_case_results'] = {'baseline_height_m': base['native_actual_height_m'], 'steps200_height_m': longer['native_actual_height_m'],
                'decimals_only_height_m': precision['native_actual_height_m'], 'baseline_label': base['stepped_label_check']['actual_cached_label_text'],
                'decimals_only_label': precision['stepped_label_check']['actual_cached_label_text']}
            result['passed'], result['active_case'] = True, None
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
        finally:
            result['cleanup_errors'] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason='L30 formal X11 cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/(name+'-runtime.log')))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/(name+'-session.json')))
                except Exception as error:
                    result['cleanup_errors'].append(name+': '+repr(error))
            if input_device:
                input_device.close()
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
