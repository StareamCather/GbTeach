"""Exercise both modern Gazebo runtimes with a dedicated simulator fixture."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.backends import get_backend
from gazeboteach.catalog import lesson_variants
from gazeboteach.modern import MODERN_PROFILES, PROFILE
from gazeboteach.runtime import terminate_owned, _token_processes
from gazeboteach.storage import Storage

_qt_capture_application = None


def image_metrics(image):
    """Reject blank GL drawables; this is a gate, not proof of correct 3D content."""
    colors, luminance = [], []
    for y in range(0, image.height(), max(1, image.height()//32)):
        for x in range(0, image.width(), max(1, image.width()//48)):
            pixel = image.pixelColor(x, y)
            rgb = (pixel.red(), pixel.green(), pixel.blue())
            colors.append(rgb)
            luminance.append((299*rgb[0] + 587*rgb[1] + 114*rgb[2])//1000)
    if not colors:
        return {'nontrivial_pixels': False, 'sample_count': 0}
    counts = Counter(colors)
    nonblack = sum(value > 12 for value in luminance)/len(luminance)
    minority = 1-max(counts.values())/len(colors)
    spread = max(luminance)-min(luminance)
    return {'nontrivial_pixels': len(counts) >= 8 and spread >= 24 and nonblack >= .03 and minority >= .01,
            'sample_count': len(colors), 'distinct_sampled_colors': len(counts),
            'luma_range': spread, 'nonblack_fraction': round(nonblack, 5),
            'non_dominant_fraction': round(minority, 5)}


def capture_owned_gui(session, target):
    """Capture the real mapped X11 GUI, identified within its owned process group."""
    from PyQt5 import QtWidgets
    global _qt_capture_application
    _qt_capture_application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    application = _qt_capture_application
    deadline = time.monotonic()+12
    last_blank = None
    while time.monotonic() < deadline:
        owned = {item['pid'] for item in _token_processes(session.token)
                 if item['pgid'] == session.gui.pid}
        listing = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'],
                                         env=session.env, text=True, timeout=3)
        for xid in re.findall(r'0x[0-9a-fA-F]+', listing):
            properties = subprocess.check_output(['xprop', '-id', xid, '_NET_WM_PID', '_NET_WM_NAME'],
                                                env=session.env, text=True, errors='replace', timeout=3)
            found = re.search(r'_NET_WM_PID\(CARDINAL\)\s*=\s*(\d+)', properties)
            if not found or int(found.group(1)) not in owned:
                continue
            application.processEvents()
            screenshot = application.primaryScreen().grabWindow(int(xid, 16))
            if screenshot.width() < 300 or screenshot.height() < 200:
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            metrics = image_metrics(screenshot.toImage())
            if not metrics['nontrivial_pixels']:
                rejected = target.with_name(target.stem+'-rejected-blank.png')
                screenshot.save(str(rejected), 'PNG')
                last_blank = {'window_id': xid, 'pid': int(found.group(1)),
                              'screenshot': str(rejected), 'image_metrics': metrics}
                continue
            assert screenshot.save(str(target), 'PNG'), '实际GUI截图保存失败'
            return {'window_id': xid, 'pid': int(found.group(1)), 'width': screenshot.width(),
                    'height': screenshot.height(), 'screenshot': str(target), 'properties': properties.strip(),
                    'image_metrics': metrics, 'manual_scene_review_required': True}
        time.sleep(.2)
    if last_blank:
        raise RuntimeError('Gazebo GUI窗口已出现但像素仍为空白/黑屏，未计图形通过：'+json.dumps(last_blank, ensure_ascii=False))
    raise RuntimeError('未发现本会话真实映射的Gazebo Sim GUI窗口，进程存活不能替代图形验收。')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--profile', choices=MODERN_PROFILES, default=PROFILE)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--terminals', action='store_true')
    parser.add_argument('--output', required=True)
    arguments = parser.parse_args()
    folder = ROOT / 'tests/fixtures/modern_diagnostic'
    manifest = json.loads((folder/'lesson.json').read_text(encoding='utf-8'))
    lessons = {lesson.profile: lesson for lesson in lesson_variants(folder, manifest)}
    lesson = lessons[arguments.profile]
    directory = Path(tempfile.mkdtemp(prefix='gazeboteach-modern-framework-'))
    storage = Storage(directory/'first')
    session = other = None
    checks = []
    started = time.monotonic()
    report = {'ok': False, 'scope': 'Modern runner diagnostic fixture',
              'profile': lesson.profile, 'data_dir': str(directory), 'checks': checks}
    def passed(name, **details):
        checks.append({'name': name, 'passed': True, **details})
    def nanoseconds(stats):
        return stats['sim_sec'] * 1_000_000_000 + stats['sim_nsec']
    try:
        session = get_backend('native').create_session(ROOT, lesson, storage=storage)
        session.start(gui=arguments.gui, terminals=arguments.terminals)
        assert session.running
        passed('real_server_ready', stats=session._stats(), backend=session.native_control_diagnosis,
               token=session.token, session=str(session.session_dir))
        assert session.verify()['passed'] is True
        passed('real_gravity_landing')
        session.pause(True)
        before = session._stats()
        session.step()
        after = session._stats()
        assert after['iterations'] == before['iterations'] + 1 and after['paused']
        assert nanoseconds(after) - nanoseconds(before) == 1_000_000
        passed('one_step_exact', before=before, after=after)
        session.step(10)
        multiple = session._stats()
        assert multiple['iterations'] == after['iterations'] + 10 and multiple['paused']
        assert nanoseconds(multiple) - nanoseconds(after) == 10_000_000
        passed('ten_steps_exact', before=after, after=multiple)
        try:
            session.reset_models()
        except RuntimeError as error:
            assert '不支持' in str(error)
            passed('unsupported_model_only_reset_is_explicit', message=str(error))
        else:
            raise AssertionError('Gazebo Sim model_only reset was falsely accepted')
        session.pause(False)
        before = session._stats(); time.sleep(.3); after = session._stats()
        assert after['sim_time'] > before['sim_time']
        passed('resume_real_time_progression', before=before, after=after)
        if arguments.gui:
            evidence = Path(arguments.output).resolve()
            captured = capture_owned_gui(session, evidence.with_name(evidence.stem + '-gui-initial.png'))
            passed('real_owned_gui_window_initial', **captured)
            first = session.gui.pid
            session.open_gui()
            assert session.gui.pid == first
            terminate_owned(session.gui)
            session.open_gui()
            assert session.gui.pid != first and session.running
            passed('gui_reopen_preserves_server', old_pid=first, new_pid=session.gui.pid)
            captured = capture_owned_gui(session, evidence.with_name(evidence.stem + '-gui-reopened.png'))
            passed('real_owned_gui_window_reopened', **captured)
        if arguments.terminals:
            assert session.health()['terminal_running'] == 2
            passed('real_isolated_log_and_interactive_ptys')
        other = get_backend('native').create_session(ROOT, lesson, storage=Storage(directory/'other'))
        other.start(gui=False, terminals=False)
        partitions = [item.env[item.partition_key] for item in (session, other)]
        domains = [item.env['ROS_DOMAIN_ID'] for item in (session, other)]
        assert partitions[0] != partitions[1] and domains[0] != domains[1]
        passed('two_sessions_unique_partitions_and_dds_domains', domains=domains, partitions=partitions)
        session.pause(True)
        paused = session._stats()
        running = other._stats()
        time.sleep(.2)
        other_after = other._stats()
        assert session._stats()['iterations'] == paused['iterations']
        assert not other_after['paused'] and other_after['iterations'] > running['iterations']
        passed('pausing_first_preserves_other_real_world', paused=paused, other_after=other_after)
        session.stop()
        assert not _token_processes(session.token) and session.domain_lease.handle is None
        assert other.running
        before = other._stats();time.sleep(.2);after = other._stats()
        assert after['iterations'] > before['iterations']
        passed('stopping_first_preserves_other_real_world', other_token=other.token)
        other.stop()
        assert not _token_processes(other.token) and other.domain_lease.handle is None
        passed('all_test_owned_processes_cleaned')
        report['ok'] = True
    except Exception as error:
        report['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        for owned in (session, other):
            if owned:
                try:
                    owned.stop(reason='现代运行器验收结束')
                except Exception as error:
                    report.update(ok=False, cleanup_error=str(error))
        report['elapsed_seconds'] = round(time.monotonic()-started, 3)
        target = Path(arguments.output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
