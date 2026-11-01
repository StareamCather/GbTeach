"""Exercise asynchronous pose application and the L01 preparation boundary."""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


COURSES = Path(__file__).resolve().parents[1]/'courses/classic11_noetic'


def load_lesson(folder):
    path = COURSES/folder/'verify.py'
    spec = importlib.util.spec_from_file_location('timing_'+folder.replace('-', '_'), path)
    module = importlib.util.module_from_spec(spec)
    # The editor course owns its geometry module; do not reuse another course's.
    modules = {}
    if folder == '05-gui-editors':
        geometry_spec = importlib.util.spec_from_file_location('geometry', path.with_name('geometry.py'))
        geometry = importlib.util.module_from_spec(geometry_spec)
        geometry_spec.loader.exec_module(geometry)
        modules['geometry'] = geometry
    with mock.patch.object(sys, 'path', list(sys.path)), mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class BasicTimingTests(unittest.TestCase):
    def test_first_world_preparation_is_outside_original_eight_second_experiment(self):
        lesson = load_lesson('01-first-world')
        clock, deadlines = Clock(), []
        reads, pause_reads = [0], []

        def actual_pose(world):
            deadlines.append(lesson.DEADLINE)
            reads[0] += 1
            if reads[0] == 1:
                clock.sleep(6.)  # Cold preparation must not consume the 8 s.
            else:
                clock.sleep(.14)
            # After the single lift, a stale pose, wrong xy at the correct z,
            # and wrong yaw must all be rejected before the actual target.
            if reads[0] == 4:
                return [.1, 0., 2., 0., 0., 0.]
            if reads[0] == 5:
                return [0., 0., 2., 0., 0., .2]
            z = 2. if reads[0] == 6 else .25
            return [0., 0., z, 0., 0., 0.]

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(lesson, 'time', clock))
            stack.enter_context(mock.patch.object(lesson, 'signal', object()))
            stack.enter_context(mock.patch.object(lesson, 'configuration', return_value=('unit', .25, .5)))
            stack.enter_context(mock.patch.object(lesson, 'pose', side_effect=actual_pose))
            stack.enter_context(mock.patch.object(lesson, 'paused_state', side_effect=[False, True, False]))
            pauses = stack.enter_context(mock.patch.object(lesson, 'set_paused',
                                                          side_effect=lambda *args: pause_reads.append(reads[0])))
            writes = stack.enter_context(mock.patch.object(lesson, 'run'))
            output = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            lesson.verify()
        self.assertEqual(deadlines[0], 20.)
        self.assertTrue(all(deadline == 14. for deadline in deadlines[1:]))
        self.assertEqual(writes.call_count, 1)
        self.assertEqual([call.args[1] for call in pauses.call_args_list], [True, False, False])
        self.assertEqual(pause_reads, [1, 6, 9])
        self.assertIn('连续稳定样本=3', output.getvalue())
        self.assertIn('准备耗时：6.000秒', output.getvalue())
        self.assertIn('物理验收耗时', output.getvalue())

    def test_first_world_lift_mismatch_exhausts_original_budget_without_resend_or_release(self):
        for actual in ([0., 0., .25, 0., 0., 0.],
                       [.1, 0., 2., 0., 0., 0.], [0., 0., 2., 0., 0., .2]):
            with self.subTest(actual=actual):
                lesson, clock, deadlines = load_lesson('01-first-world'), Clock(), []

                def actual_pose(world):
                    deadlines.append(lesson.DEADLINE)
                    return [0., 0., .25, 0., 0., 0.] if len(deadlines) == 1 else actual

                with mock.patch.object(lesson, 'time', clock), \
                     mock.patch.object(lesson, 'configuration', return_value=('unit', .25, .5)), \
                     mock.patch.object(lesson, 'prepare_pose_reader'), \
                     mock.patch.object(lesson, 'pose', side_effect=actual_pose) as reads, \
                     mock.patch.object(lesson, 'paused_state', side_effect=[False, True, False]), \
                     mock.patch.object(lesson, 'set_paused') as pauses, \
                     mock.patch.object(lesson, 'run') as writes, contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, '目标位置/四元数=.*最后实测='):
                        lesson.verify()
                self.assertEqual(writes.call_count, 1)
                self.assertGreater(reads.call_count, 1)
                self.assertTrue(all(deadline == 8. for deadline in deadlines))
                self.assertLessEqual(clock.now, 8.)
                # Only initial pause and finally's pause-state restoration ran;
                # the physical release was never reached after a failed lift.
                self.assertEqual([call.args[1] for call in pauses.call_args_list], [True, False])

    @unittest.skipUnless(hasattr(signal, 'setitimer'), 'actual SIGALRM boundary requires Linux/POSIX')
    def test_preparation_alarm_interrupts_and_reaps_blocked_child_and_restores_previous_alarm(self):
        lesson = load_lesson('01-first-world')
        original_handler = signal.getsignal(signal.SIGALRM)
        original_timer = signal.getitimer(signal.ITIMER_REAL)
        started = time.monotonic()
        real_timer = signal.setitimer
        timer_calls = []

        def accelerated_timer(which, duration, interval=0):
            timer_calls.append((duration, interval))
            # Exercise the actual OS alarm and subprocess.run exception cleanup
            # without making this regression test block for twenty seconds.
            return real_timer(which, .5 if duration == 20. else duration, interval)

        def previous_handler(signum, frame):
            raise AssertionError('the previous alarm should not fire in this test')

        try:
            signal.signal(signal.SIGALRM, previous_handler)
            real_timer(signal.ITIMER_REAL, 6.)
            with tempfile.TemporaryDirectory() as directory:
                pid_file = Path(directory)/'blocked-child.pid'

                def blocked_pose(*args):
                    code = 'import os,pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)'
                    subprocess.run([sys.executable, '-c', code, str(pid_file)], check=True, timeout=120)

                with mock.patch.object(lesson, 'native_model_pose', side_effect=blocked_pose) as reader, \
                     mock.patch.object(lesson, 'run') as fallback, \
                     mock.patch.object(signal, 'setitimer', side_effect=accelerated_timer), \
                     contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, '20秒预算'):
                        lesson.prepare_pose_reader('unit')
                self.assertEqual(reader.call_count, 1)
                fallback.assert_not_called()
                self.assertTrue(pid_file.is_file())
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(pid_file.read_text()), 0)
            self.assertLess(time.monotonic()-started, 3.)
            self.assertEqual(timer_calls[0], (20., 0))
            self.assertIs(signal.getsignal(signal.SIGALRM), previous_handler)
            remaining, interval = signal.getitimer(signal.ITIMER_REAL)
            self.assertGreater(remaining, 3.)
            self.assertLess(remaining, 6.)
            self.assertEqual(interval, 0.)
        finally:
            real_timer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, original_handler)
            if original_timer[0] > 0:
                real_timer(signal.ITIMER_REAL, max(.001, original_timer[0]-(time.monotonic()-started)), original_timer[1])

    def test_rig_single_move_and_restore_wait_through_old_position_and_old_rotation(self):
        lesson = load_lesson('03-pose-frames')
        clock = Clock()
        original = [2., 1., .2, 0., 0., 0.]
        marker = [2., 2., .8, 0., 0., 0.]
        raw = {'rig': original, 'sensor_marker': marker, 'mount_axes': [0.]*6}
        expected = {name: lesson.transform(value) for name, value in raw.items()}
        expected.update({'rig::body': expected['rig'], 'sensor_marker::body': expected['sensor_marker']})
        writes, reads = [], []
        pending = []

        def command(*args):
            target = [float(args[args.index(key)+1]) for key in ('-x', '-y', '-z', '-R', '-P', '-Y')]
            writes.append(target)
            wrong_rotation = list(target)
            wrong_rotation[5] += .2
            pending[:] = [list(raw['rig']), wrong_rotation, target]

        def actual(world, name):
            clock.sleep(.02)
            reads.append((name, lesson.DEADLINE))
            if name == 'rig' and pending:
                raw[name] = pending.pop(0)
            return list(raw[name]), lesson.transform(raw[name])

        with mock.patch.object(lesson, 'time', clock), \
             mock.patch.object(lesson, 'graph_configuration', return_value=('unit', expected)), \
             mock.patch.object(lesson, 'live_model', side_effect=actual), \
             mock.patch.object(lesson, 'live_link_relative', return_value=lesson.IDENTITY), \
             mock.patch.object(lesson, 'run', side_effect=command), contextlib.redirect_stdout(io.StringIO()):
            lesson.verify()
        self.assertEqual(writes, [[2.35, 1., .2, 0., 0., 0.], original])
        self.assertEqual(raw['rig'], original)
        self.assertEqual(sum(name == 'rig' for name, _ in reads), 7)
        self.assertTrue(all(deadline == 12. for _, deadline in reads[:-3]))

    def test_probe_single_lift_confirms_before_physics_and_restores_with_read_lag(self):
        lesson = load_lesson('05-gui-editors')
        clock = Clock()
        packages = {'room_example': {'analysis': {'width': .8, 'height': 1.4, 'wall_thickness': .12}},
                    'table_example': {'placement': [0.]*6,
                                      'analysis': {'top': {'pose': [0., 0., .8, 0., 0., 0.]}, 'surface_z': .85}}}
        original = [.2, .1, .95, 0., 0., .2]
        current, pending, writes, pauses = list(original), [], [], []
        phase = ['initial']
        phase_reads = {'moving': 0, 'restoring': 0, 'falling': 0}

        def command(*args):
            target = [float(args[args.index(key)+1]) for key in ('-x', '-y', '-z', '-R', '-P', '-Y')]
            writes.append(target)
            wrong_rotation = list(target)
            wrong_rotation[5] += .2
            pending[:] = [list(current), wrong_rotation, target]
            phase[0] = 'moving' if len(writes) == 1 else 'restoring'

        def actual(world, name):
            clock.sleep(.13)
            if phase[0] in phase_reads:
                phase_reads[phase[0]] += 1
            if pending:
                current[:] = pending.pop(0)
            elif phase[0] == 'falling':
                current[:] = [0., 0., .9, 0., 0., 0.]
            return list(current)

        def pause(world, value):
            pauses.append(value)
            if not value and len(writes) == 1:
                self.assertFalse(pending, 'release must follow full pose confirmation')
                phase[0] = 'falling'

        with mock.patch.object(lesson, 'time', clock), \
             mock.patch.object(lesson, 'configuration', return_value=('unit', packages, [.1]*3)), \
             mock.patch.object(lesson, 'live_geometry'), mock.patch.object(lesson, 'contains_xy', return_value=True), \
             mock.patch.object(lesson, 'live_pose', side_effect=actual), \
             mock.patch.object(lesson, 'paused_state', side_effect=[False, True, True, False]), \
             mock.patch.object(lesson, 'pause', side_effect=pause), \
             mock.patch.object(lesson, 'run', side_effect=command), contextlib.redirect_stdout(io.StringIO()):
            lesson.verify()
        self.assertEqual(len(writes), 2)
        self.assertEqual(writes[1], original)
        self.assertEqual(current, original)
        self.assertEqual(phase_reads, {'moving': 3, 'restoring': 3, 'falling': 3})
        self.assertEqual(pauses, [True, False, True, False])

    def test_deadline_exhaustion_keeps_xyz_rotation_evidence_and_never_resends_move(self):
        for folder in ('03-pose-frames', '05-gui-editors'):
            for actual in ([1., 0., 2., 0., 0., 0.], [0., 0., 2., 0., 0., .2]):
                with self.subTest(folder=folder, actual=actual):
                    lesson, clock = load_lesson(folder), Clock()
                    lesson.DEADLINE = .5
                    target = [0., 0., 2., 0., 0., 0.]
                    read_name = 'live_model' if folder.startswith('03') else 'live_pose'
                    answer = (actual, lesson.transform(actual)) if folder.startswith('03') else actual
                    with mock.patch.object(lesson, 'time', clock), \
                         mock.patch.object(lesson, read_name, return_value=answer) as reads, \
                         mock.patch.object(lesson, 'run') as writes:
                        if folder.startswith('03'):
                            lesson.move_rig('unit', target)
                            confirm = lambda: lesson.confirm_rig_pose('unit', target, 'unit move')
                        else:
                            lesson.move('unit', 'table_probe', target)
                            confirm = lambda: lesson.confirm_probe_pose('unit', target, 'unit move')
                        with self.assertRaisesRegex(RuntimeError, '目标位置/四元数=.*最后实测='):
                            confirm()
                    self.assertEqual(writes.call_count, 1)
                    self.assertGreater(reads.call_count, 1)
                    self.assertEqual(lesson.DEADLINE, .5)
                    self.assertLessEqual(clock.now, .5)


if __name__ == '__main__':
    unittest.main()
