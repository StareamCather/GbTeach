"""Failure-path regressions for course state restoration and owned teardown."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
COURSES = ROOT/'courses/classic11_noetic'


def load(name, path, config=None):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    if config is None:
        spec.loader.exec_module(module)
    else:
        with patch.dict(sys.modules, {'config': types.SimpleNamespace(**config)}):
            spec.loader.exec_module(module)
    return module


class SensorVerifyRestoration(unittest.TestCase):
    def test_pause_is_restored_when_post_demo_step_fails(self):
        course = load('sensor_lifecycle', COURSES/'28-sensor-plugin/lab.py',
                      {'parameters': None, 'prepared': None})
        lab = course.Lab.__new__(course.Lab)
        lab.stats = Mock(return_value={'paused': False, 'sim_time': {'sec': 1}})
        lab.demo = Mock(return_value={'passed': True, 'restored_target': {'verified': True}})
        lab.pause = Mock(side_effect=lambda paused: {'paused': paused})
        lab.snapshot = Mock(return_value={'source_time_ns': '1000000000'})
        lab.step = Mock(side_effect=RuntimeError('injected finite step failure'))
        with patch.object(course.time, 'sleep'), self.assertRaisesRegex(RuntimeError, 'finite step failure'):
            lab.verify()
        self.assertEqual([call.args[0] for call in lab.pause.call_args_list], [True, False])
        lab.step.assert_called_once_with(100)

    def test_demo_failure_still_restores_entry_pause(self):
        course = load('sensor_lifecycle_demo', COURSES/'28-sensor-plugin/lab.py',
                      {'parameters': None, 'prepared': None})
        lab = course.Lab.__new__(course.Lab)
        lab.stats = Mock(return_value={'paused': True})
        lab.demo = Mock(side_effect=RuntimeError('injected observation failure'))
        lab.pause = Mock(return_value={'paused': True})
        with self.assertRaisesRegex(RuntimeError, 'observation failure'):
            lab.verify()
        lab.pause.assert_called_once_with(True)


class PlaybackTeardown(unittest.TestCase):
    def make(self, root):
        course = load('playback_lifecycle', COURSES/'32-record-replay/lab.py',
                      {'DT_NS': 1000000, 'WORLD': 'lesson32', 'parameters': None,
                       'prepared': None, 'sha': None})
        # The lesson runs on Linux; these isolated policy tests also run on the
        # Windows development host without signalling any real process.
        linux_kill = patch.object(course.signal, 'SIGKILL', 9, create=True)
        linux_kill.start()
        self.addCleanup(linux_kill.stop)
        session = course.OwnedPlayback.__new__(course.OwnedPlayback)
        session.root = root
        session.token = 'private-token'
        session.streams = []
        session.previous_signal = None
        identities = {
            10: {'pid': 10, 'start_ticks': '100', 'token': session.token, 'exe': '/usr/bin/gzserver'},
            20: {'pid': 20, 'start_ticks': '200', 'token': session.token, 'exe': '/usr/bin/gzclient'},
        }
        session.record = {'processes': [dict(value, label='gzserver' if pid == 10 else 'gzclient')
                                        for pid, value in identities.items()]}
        session.processes = [Mock(pid=pid) for pid in identities]
        for process in session.processes:
            process.poll.return_value = 0
            process.wait.return_value = 0
        session.owned = lambda: [dict(value) for value in identities.values() if value['token'] == session.token]
        session.identity = lambda pid: identities.get(pid)
        return course, session, identities

    def test_gui_exits_before_server_is_signalled_even_when_server_pid_is_lower(self):
        with tempfile.TemporaryDirectory() as directory:
            course, session, identities = self.make(Path(directory))
            signals = []
            def kill(pid, sig):
                signals.append((pid, sig))
                identities.pop(pid)
            with patch.object(course.os, 'kill', side_effect=kill), patch.object(course.signal, 'signal'):
                session.__exit__()
            self.assertEqual(signals, [(20, signal.SIGTERM), (10, signal.SIGTERM)])
            self.assertTrue(session.record['cleaned'])
            self.assertEqual([event['role'] for event in session.record['cleanup_events']], ['gui', 'server'])

    def test_pid_reuse_between_discovery_and_signal_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            course, session, identities = self.make(Path(directory))
            def identity(pid):
                value = identities.get(pid)
                if pid == 20 and value:
                    value.update(start_ticks='999', token='unrelated-user-process')
                return value
            session.identity = identity
            signals = []
            def kill(pid, sig):
                signals.append(pid)
                identities.pop(pid)
            with patch.object(course.os, 'kill', side_effect=kill), patch.object(course.signal, 'signal'):
                session.__exit__()
            self.assertEqual(signals, [10])
            self.assertIn(20, identities)

    def test_surviving_gui_prevents_server_teardown(self):
        with tempfile.TemporaryDirectory() as directory:
            course, session, identities = self.make(Path(directory))
            clock = iter(range(1000))
            for process in session.processes:
                process.wait.side_effect = subprocess.TimeoutExpired('owned playback', 1)
            with patch.object(course.os, 'kill') as kill, patch.object(course.signal, 'signal'), \
                    patch.object(course.time, 'sleep'), patch.object(course.time, 'monotonic', side_effect=lambda: next(clock)):
                with self.assertRaisesRegex(RuntimeError, 'survived bounded cleanup'):
                    session.__exit__()
            self.assertEqual([call.args[0] for call in kill.call_args_list], [20, 20])
            self.assertFalse(session.record['cleaned'])


class PersistentControllerPause(unittest.TestCase):
    def test_copied_support_imports_app_from_source_root_with_unrelated_cwd(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            copied = workspace/'profile/course/support/pause.py'
            copied.parent.mkdir(parents=True)
            copied.write_bytes((COURSES/'_control_support/pause.py').read_bytes())
            # -I removes the repo cwd/PYTHONPATH from the import path. A missing
            # SDK toolchain keeps this an import regression without a simulator.
            code = ('import runpy,sys,pathlib; '
                    'assert "gazeboteach" not in sys.modules; '
                    'support=runpy.run_path(sys.argv[1]); '
                    'assert support["get_control"]("fixture",required=False) is None; '
                    'import gazeboteach; '
                    'assert pathlib.Path(gazeboteach.__file__).resolve().parent.parent==pathlib.Path(sys.argv[2]).resolve()')
            env = dict(os.environ, GAZEBOTEACH_SOURCE_ROOT=str(ROOT), PATH='', PYTHONPATH=str(copied.parent))
            process = subprocess.run([sys.executable, '-I', '-c', code, str(copied), str(ROOT)],
                                     cwd=str(workspace), env=env, text=True, capture_output=True, timeout=10)
            self.assertEqual(process.returncode, 0, process.stdout+process.stderr)
            self.assertIn('explicit legacy CLI fallback', process.stderr)

    def test_many_calls_reuse_one_helper_and_failure_never_creates_another(self):
        pause = load('controller_pause_lifecycle', COURSES/'_control_support/pause.py')
        control = Mock()
        control.status.return_value = {'stats': {'paused': False}}
        control.pause.return_value = {'stats': {'paused': True}}
        control.metadata.return_value = {'pid': 123, 'closed': False}
        with tempfile.TemporaryDirectory() as directory, patch.object(pause.Path, 'cwd', return_value=Path(directory)), \
                patch('gazeboteach.native_control.try_create', return_value=(control, {'supported': True})) as create:
            self.assertFalse(pause.read_pause())
            self.assertFalse(pause.read_pause())
            self.assertTrue(pause.set_pause(True)['confirmed'])
            control.pause.side_effect = RuntimeError('unknown mutation outcome')
            with self.assertRaisesRegex(RuntimeError, 'unknown mutation outcome'):
                pause.set_pause(True)
            self.assertFalse(pause.read_pause())
            create.assert_called_once()
            pause.close_controls()
            control.close.assert_called_once()
        # Avoid this test module's atexit callback writing into its removed tempdir.
        pause._controls.clear()


if __name__ == '__main__':
    unittest.main()
