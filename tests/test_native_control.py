"""Inject the persistent helper's actual Popen/stdio boundary, without an SDK.

These tests exercise the real reader, request UUIDs, bounded waits, poisoning,
EOF cleanup and runtime dispatch. Responses deliberately include delayed and
foreign observations; no test starts Gazebo or predicts its physical dynamics.
Model tests verify wire correlation, validation and preservation; actual native
pose acquisition and the installed SDK's Euler conversion require live tests.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from gazeboteach import native_control as native
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage


def stats(seq=1, iterations=40, paused=True, sim_time=2.):
    return {'seq': seq, 'iterations': iterations, 'paused': paused, 'sim_time': sim_time}


def model_reply(request, before=None, after=None):
    """A native entity_info response, including its distinct acquisition fences."""
    before = stats(seq=2) if before is None else before
    after = stats(seq=3) if after is None else after
    return {'id': request['id'], 'ok': True, 'stats': dict(after),
        'model_name': request['name'], 'request_id': 876543210, 'response_id': 876543210,
        'response_type': 'gazebo.msgs.Model',
        'pose': {'position': {'x': 1.25, 'y': -.5, 'z': 3.},
                 'orientation': {'x': 0., 'y': 0., 'z': math.sqrt(.5), 'w': math.sqrt(.5)}},
        'rpy': [0., 0., math.pi/2],
        'pose_stats_before': dict(before), 'pose_stats_after': dict(after)}


class LinePipe:
    def __init__(self):
        self.lines, self.closed, self.finished = queue.Queue(), False, False

    def feed(self, value):
        self.lines.put(value if isinstance(value, str) else json.dumps(value)+'\n')

    def finish(self):
        if not self.finished:
            self.finished = True
            self.lines.put(None)

    def __iter__(self):
        return self

    def __next__(self):
        value = self.lines.get(timeout=5)
        if value is None:
            raise StopIteration
        return value

    def close(self):
        self.closed = True
        self.finish()


class CommandPipe:
    def __init__(self, owner):
        self.owner, self.closed, self.buffer, self.close_calls = owner, False, '', 0

    def write(self, text):
        if self.closed:
            raise BrokenPipeError('test stdin is closed')
        self.buffer += text
        while '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            request = json.loads(line)
            self.owner.requests.append(request)
            answer = self.owner.respond(request)
            if answer is not None:
                self.owner.stdout.feed(answer)
        return len(text)

    def flush(self):
        pass

    def close(self):
        self.closed = True
        self.close_calls += 1


class WireProcess:
    """A process double, not a fake NativeControl or a patched internal RPC."""
    pid = 765432

    def __init__(self, respond=None, ready=None, wait_plan=(), joined=True):
        self.stdout, self.returncode, self.requests = LinePipe(), None, []
        self.stdin = CommandPipe(self)
        self.respond = respond or self.default_response
        self.wait_plan, self.joined = deque(wait_plan), joined
        self.wait_calls, self.terminate_calls, self.kill_calls = [], 0, 0
        self.stdout.feed({'ready': True, 'stats': stats()} if ready is None else ready)

    def default_response(self, request):
        return {'id': request['id'], 'ok': True,
                'stats': stats(seq=len(self.requests)+1, paused=request.get('paused', True))}

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.wait_calls.append(timeout)
        if self.wait_plan:
            action = self.wait_plan.popleft()
            if action == 'timeout':
                raise subprocess.TimeoutExpired('owned-native-control', timeout)
        if self.returncode is None:
            self.returncode = 0
        self.stdout.feed({'closed': True, 'io_guard_joined': self.joined})
        self.stdout.finish()
        return self.returncode

    def terminate(self):
        self.terminate_calls += 1
        self.returncode = -15

    def kill(self):
        self.kill_calls += 1
        self.returncode = -9


class NativeControlTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.executable = self.root/'native_control'
        self.executable.write_bytes(b'unit protocol executable identity')
        self.env = {'GAZEBOTEACH_SESSION_TOKEN': 'native-control-unit-token', 'PATH': 'unit-path'}
        # Only Linux identity observations are injected. Real pathlib, real
        # queues, real reader thread and the public protocol remain in use.
        fields = ['0']*20
        fields[0], fields[1], fields[19] = 'S', str(os.getpid()), '123456'
        proc = mock.Mock()
        proc.__truediv__ = mock.Mock(return_value=SimpleNamespace(read_text=lambda: '765432 (native) '+' '.join(fields)))
        proc.stat.return_value = SimpleNamespace(st_uid=1000)
        proc_root = mock.Mock()
        proc_root.__truediv__ = mock.Mock(return_value=proc)
        real_path = Path
        def path_factory(*parts):
            return proc_root if parts == ('/proc',) else real_path(*parts)
        for patcher in (mock.patch.object(native, 'Path', side_effect=path_factory),
                        mock.patch.object(native.os, 'getpgid', return_value=WireProcess.pid, create=True),
                        mock.patch.object(native.os, 'getsid', return_value=WireProcess.pid, create=True)):
            patcher.start()
            self.addCleanup(patcher.stop)

    def controller(self, process=None):
        process = process or WireProcess()
        patcher = mock.patch.object(native.subprocess, 'Popen', return_value=process)
        spawn = patcher.start()
        self.addCleanup(patcher.stop)
        control = native.NativeControl(self.executable, 'fixture_world', self.env, self.root, {'cache_key': 'unit'})
        self.addCleanup(control.close)
        return control, process, spawn

    def assert_poisoned_without_replay(self, control, process, spawn, action):
        with self.assertRaises(RuntimeError):
            action()
        sent = list(process.requests)
        self.assertIsNotNone(control.metadata()['poisoned'])
        with self.assertRaises(RuntimeError):
            control.reset_models(timeout=.3)
        self.assertEqual(process.requests, sent)
        self.assertEqual(spawn.call_count, 1)

    def test_owned_stdio_process_and_primary_environment_are_preserved(self):
        control, process, spawn = self.controller()
        arguments, options = spawn.call_args
        self.assertEqual(arguments[0], [str(self.executable.resolve()), 'fixture_world'])
        self.assertTrue(options['start_new_session'])
        self.assertIs(options['stdin'], subprocess.PIPE)
        self.assertIs(options['stdout'], subprocess.PIPE)
        self.assertTrue(options['text'])
        self.assertEqual(options['env']['GAZEBOTEACH_SESSION_TOKEN'], self.env['GAZEBOTEACH_SESSION_TOKEN'])
        self.assertEqual(options['env']['GAZEBOTEACH_IO_DIAGNOSTICS'], '1')
        self.assertNotIn('GAZEBOTEACH_IO_DIAGNOSTICS', self.env)
        metadata = control.metadata()
        self.assertEqual(metadata['process']['pid'], process.pid)
        self.assertEqual(metadata['process']['pgid'], process.pid)
        self.assertEqual(metadata['process']['sid'], process.pid)
        self.assertEqual(metadata['process']['start_time'], '123456')
        self.assertEqual(metadata['build']['cache_key'], 'unit')

    def test_paused_world_can_supply_fresh_statistics_without_advancing_time(self):
        control, process, _ = self.controller()
        response = control.status(timeout=.3)
        self.assertEqual(response['stats']['sim_time'], 2.)
        self.assertEqual(response['stats']['iterations'], 40)
        self.assertEqual(response['stats']['seq'], 2)
        self.assertEqual(process.requests[0]['action'], 'status')

    def test_each_request_has_one_distinct_uuid_and_explicit_wall_deadline(self):
        control, process, spawn = self.controller()
        control.status(timeout=.3)
        control.pause(False, timeout=.4)
        control.reset_models(timeout=.5)
        self.assertEqual([row['action'] for row in process.requests], ['status', 'pause', 'reset_models'])
        self.assertEqual(len({row['id'] for row in process.requests}), 3)
        self.assertTrue(all(row['id'].isalnum() and len(row['id']) == 32 for row in process.requests))
        self.assertEqual([row['timeout_ms'] for row in process.requests], [100, 200, 300])
        self.assertEqual(spawn.call_count, 1)

    def test_malformed_ready_closes_owned_process_without_commands_or_restart(self):
        for ready in ('not JSON\n', '[]\n', {'ready': False}, {'ready': True, 'stats': stats(seq=True)}):
            with self.subTest(ready=ready):
                process = WireProcess(ready=ready)
                with mock.patch.object(native.subprocess, 'Popen', return_value=process) as spawn:
                    with self.assertRaises(RuntimeError):
                        native.NativeControl(self.executable, 'fixture', self.env, self.root)
                    self.assertEqual(spawn.call_count, 1)
                    self.assertTrue(process.stdin.closed)
                    self.assertEqual(process.requests, [])
                    self.assertEqual(process.terminate_calls, 0)

    def test_late_previous_uuid_cannot_acknowledge_a_new_mutation(self):
        first = []
        def reply(request):
            if not first:
                first.append(request['id'])
            return {'id': first[0], 'ok': True, 'stats': stats(seq=2)}
        control, process, spawn = self.controller(WireProcess(respond=reply))
        control.status(timeout=.3)
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.pause(False, timeout=.3))
        self.assertEqual([row['action'] for row in process.requests], ['status', 'pause'])

    def test_malformed_response_or_wrong_uuid_poison_the_one_connection(self):
        for response in ('not JSON\n', '[]\n', {'id': 'foreign', 'ok': True, 'stats': stats(seq=2)}):
            with self.subTest(response=response):
                control, process, spawn = self.controller(WireProcess(respond=lambda request, response=response: response))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.pause(False, timeout=.3))

    def test_unknown_mutation_timeout_is_not_retried_and_cannot_restart(self):
        control, process, spawn = self.controller(WireProcess(respond=lambda request: None))
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.pause(False, timeout=.3))
        self.assertEqual(len(process.requests), 1)
        self.assertEqual(process.requests[0]['action'], 'pause')

    def test_process_exit_and_unexpected_eof_are_visible_without_restart(self):
        for exited in (True, False):
            with self.subTest(exited=exited):
                control, process, spawn = self.controller()
                if exited:
                    process.returncode = -6
                else:
                    process.respond = lambda request: process.stdout.finish()
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.status(timeout=.3))
                self.assertEqual(len(process.requests), 0 if exited else 1)

    def test_pause_requires_a_fresh_actual_flag_not_only_matching_uuid(self):
        for observed in (stats(seq=1, paused=True), stats(seq=2, paused=False)):
            with self.subTest(observed=observed):
                process = WireProcess(respond=lambda request, observed=observed: {'id': request['id'], 'ok': True, 'stats': observed})
                control, process, spawn = self.controller(process)
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.pause(True, timeout=.3))

    def test_malformed_native_statistics_are_not_accepted_as_fresh(self):
        for changes in ({'seq': True}, {'iterations': 41.0}, {'iterations': -1}, {'paused': 1}, {'sim_time': float('nan')}):
            with self.subTest(changes=changes):
                observed = dict(stats(seq=2), **changes)
                process = WireProcess(respond=lambda request, observed=observed: {'id': request['id'], 'ok': True, 'stats': observed})
                control, process, spawn = self.controller(process)
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.status(timeout=.3))

    def test_exact_step_uses_actual_native_baseline_not_an_earlier_python_sample(self):
        process = WireProcess(respond=lambda request: {'id': request['id'], 'ok': True, 'baseline_iterations': 100,
            'requested_steps': request['count'], 'stats': stats(seq=2, iterations=103)})
        control, process, _ = self.controller(process)
        self.assertEqual(control.step(3, timeout=.3)['stats']['iterations'], 103)
        self.assertEqual(len(process.requests), 1)

    def test_foreign_steps_unpaused_result_and_malformed_counts_fail_without_resend(self):
        variants = ({'iterations': 44}, {'iterations': 42}, {'paused': False},
                    {'baseline_iterations': -1, 'iterations': 2}, {'requested_steps': True, 'iterations': 41, 'count': 1})
        for changes in variants:
            with self.subTest(changes=changes):
                def reply(request, changes=changes):
                    return {'id': request['id'], 'ok': True, 'baseline_iterations': changes.get('baseline_iterations', 40),
                        'requested_steps': changes.get('requested_steps', request['count']),
                        'stats': stats(seq=2, iterations=changes.get('iterations', 43), paused=changes.get('paused', True))}
                control, process, spawn = self.controller(WireProcess(respond=reply))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.step(changes.get('count', 3), timeout=.3))

    def test_native_refusal_keeps_original_diagnosis_and_never_replays_step(self):
        process = WireProcess(respond=lambda request: {'id': request['id'], 'ok': False, 'error': 'foreign iteration overshoot'})
        control, process, spawn = self.controller(process)
        with self.assertRaisesRegex(RuntimeError, 'foreign iteration overshoot'):
            control.step(3, timeout=.3)
        self.assertEqual(control.metadata()['stats']['iterations'], 40)
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.step(3, timeout=.3))
        self.assertEqual(len(process.requests), 1)

    def test_invalid_inputs_never_cross_the_process_boundary(self):
        control, process, _ = self.controller()
        for count in (True, 0, -1, 1.5, 1000001):
            with self.subTest(count=count), self.assertRaises(ValueError):
                control.step(count)
        for paused in (0, 1, 'true', None):
            with self.subTest(paused=paused), self.assertRaises(ValueError):
                control.pause(paused)
        for timeout in (True, .29, 21, float('inf'), '3'):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                control.status(timeout)
        self.assertEqual(process.requests, [])

    def test_normal_eof_drains_sdk_join_proof_and_close_is_idempotent(self):
        control, process, spawn = self.controller()
        cleanup = control.close()
        self.assertTrue(cleanup['eof_requested'])
        self.assertTrue(cleanup['io_guard_joined'])
        self.assertFalse(cleanup['forced'])
        self.assertEqual(cleanup['returncode'], 0)
        self.assertEqual(process.stdin.close_calls, 1)
        self.assertIs(control.close(), cleanup)
        self.assertEqual(process.terminate_calls, 0)
        self.assertEqual(process.kill_calls, 0)
        with self.assertRaises(RuntimeError):
            control.pause(False, timeout=.3)
        self.assertEqual(process.requests, [])
        self.assertEqual(spawn.call_count, 1)

    def test_missing_join_proof_is_reported_and_forced_cleanup_is_bounded(self):
        control, process, _ = self.controller(WireProcess(joined=False))
        self.assertFalse(control.close()['io_guard_joined'])
        control, process, _ = self.controller(WireProcess(wait_plan=('timeout', 'timeout', 'done')))
        cleanup = control.close()
        self.assertTrue(cleanup['forced'])
        self.assertEqual(process.wait_calls, [3, 1, 1])
        self.assertEqual(process.terminate_calls, 1)
        self.assertEqual(process.kill_calls, 1)
        self.assertEqual(cleanup['returncode'], -9)

    def test_model_pose_preserves_native_pose_euler_and_non_atomic_stats_brackets(self):
        before = stats(seq=2, iterations=40, paused=False, sim_time=2.)
        after = stats(seq=3, iterations=47, paused=False, sim_time=2.007)
        control, process, spawn = self.controller(WireProcess(respond=lambda request: model_reply(request, before, after)))
        value = control.model_pose('crate', timeout=.5)
        self.assertEqual(process.requests[0]['action'], 'model_pose')
        self.assertEqual(process.requests[0]['name'], 'crate')
        self.assertEqual(process.requests[0]['timeout_ms'], 300)
        self.assertEqual(value['pose']['position'], {'x': 1.25, 'y': -.5, 'z': 3.})
        self.assertEqual(value['rpy'], [0., 0., math.pi/2])
        self.assertNotEqual(value['rpy'][2], value['pose']['orientation']['z'])
        self.assertEqual(value['pose_stats_before'], before)
        self.assertEqual(value['pose_stats_after'], after)
        self.assertEqual(value['stats'], after)
        self.assertEqual(control.metadata()['stats'], after)
        self.assertEqual(spawn.call_count, 1)

    def test_model_pose_reuses_one_process_for_literal_scoped_and_utf8_names(self):
        names = ('crate', 'fixture_world::crate', 'a box', '\u6a21'*170, 'x'*512)
        replies = []
        def reply(request):
            index = len(replies)
            value = model_reply(request, stats(seq=2+2*index), stats(seq=3+2*index))
            if index == 0:
                value['model_name'] = 'fixture_world::crate'
            value['request_id'] += index
            value['response_id'] += index
            replies.append(value)
            return value
        control, process, spawn = self.controller(WireProcess(respond=reply))
        for name in names:
            self.assertEqual(control.model_pose(name, timeout=.3)['model_name'], replies[-1]['model_name'])
        self.assertEqual([request['name'] for request in process.requests], list(names))
        self.assertTrue(all(request['action'] == 'model_pose' for request in process.requests))
        self.assertEqual(len({request['id'] for request in process.requests}), len(names))
        self.assertEqual(spawn.call_count, 1)

    def test_invalid_model_names_and_timeouts_never_cross_the_process_boundary(self):
        control, process, _ = self.controller(WireProcess(respond=model_reply))
        names = (None, True, 3, b'crate', '', 'x'*513, '\u6a21'*171, 'crate\0', 'crate\n', '\x1fcrate', 'crate\x7f', '\ud800')
        for name in names:
            with self.subTest(name=repr(name)), self.assertRaises(ValueError):
                control.model_pose(name, timeout=.3)
        for timeout in (True, .29, 21, float('nan'), '3'):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                control.model_pose('crate', timeout=timeout)
        self.assertEqual(process.requests, [])
        self.assertIsNone(control.metadata()['poisoned'])
        self.assertEqual(control.model_pose('crate', timeout=.3)['model_name'], 'crate')

    def test_model_pose_cannot_accept_a_late_previous_outer_uuid(self):
        old_uuid = []
        def reply(request):
            value = model_reply(request, stats(seq=2+len(old_uuid)*2), stats(seq=3+len(old_uuid)*2))
            if old_uuid:
                value['id'] = old_uuid[0]
            else:
                old_uuid.append(request['id'])
            return value
        control, process, spawn = self.controller(WireProcess(respond=reply))
        control.model_pose('crate', timeout=.3)
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))
        self.assertEqual([request['action'] for request in process.requests], ['model_pose', 'model_pose'])

    def test_model_pose_rejects_foreign_model_type_or_native_response_identity(self):
        variants = ({'model_name': 'other_crate'}, {'model_name': 'world::crate_extra'},
            {'model_name': None}, {'response_type': 'gazebo.msgs.Pose'}, {'response_type': None},
            {'request_id': 0, 'response_id': 0}, {'request_id': -1, 'response_id': -1},
            {'request_id': True, 'response_id': True}, {'request_id': 12.0, 'response_id': 12.0},
            {'request_id': 0x80000000, 'response_id': 0x80000000},
            {'request_id': 12, 'response_id': True}, {'request_id': 12, 'response_id': '12'},
            {'request_id': 12, 'response_id': 0x80000000},
            {'request_id': 12, 'response_id': 13})
        for changes in variants:
            with self.subTest(changes=changes):
                def reply(request, changes=changes):
                    return dict(model_reply(request), **changes)
                control, process, spawn = self.controller(WireProcess(respond=reply))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))
                self.assertEqual(len(process.requests), 1)

    def test_model_pose_accepts_both_positive_signed_int32_identity_boundaries(self):
        for native_id in (1, 0x7fffffff):
            with self.subTest(native_id=native_id):
                def reply(request):
                    value = model_reply(request)
                    value['request_id'] = value['response_id'] = native_id
                    return value
                control, process, _ = self.controller(WireProcess(respond=reply))
                self.assertEqual(control.model_pose('crate', timeout=.3)['response_id'], native_id)
                self.assertEqual(len(process.requests), 1)

    def test_model_pose_schema_components_and_euler_angles_must_be_finite(self):
        variants = (('pose', None), ('pose', {'position': {}, 'orientation': {}}),
            ('position', {'x': 1., 'y': 2.}), ('position', {'x': 1., 'y': 2., 'z': 3., 'extra': 4.}),
            ('position', {'x': True, 'y': 2., 'z': 3.}), ('position', {'x': '1', 'y': 2., 'z': 3.}),
            ('position', {'x': float('nan'), 'y': 2., 'z': 3.}),
            ('orientation', {'x': 0., 'y': 0., 'z': float('inf'), 'w': 1.}),
            ('orientation', {'x': 0., 'y': 0., 'z': 0., 'w': True}),
            ('orientation', {'x': 0., 'y': 0., 'z': 0.}),
            ('rpy', None), ('rpy', [0., 0.]), ('rpy', [0., 0., 0., 0.]),
            ('rpy', [0., False, 0.]), ('rpy', [0., 0., float('-inf')]))
        for field, changed in variants:
            with self.subTest(field=field, changed=changed):
                def reply(request, field=field, changed=changed):
                    value = model_reply(request)
                    if field in ('position', 'orientation'):
                        value['pose'][field] = changed
                    else:
                        value[field] = changed
                    return value
                control, process, spawn = self.controller(WireProcess(respond=reply))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))

    def test_model_pose_requires_a_unit_quaternion_without_silently_normalizing(self):
        for orientation in ({'x': 0., 'y': 0., 'z': 0., 'w': 0.},
                {'x': 0., 'y': 0., 'z': 0., 'w': 2.}, {'x': .01, 'y': 0., 'z': 0., 'w': 1.}):
            with self.subTest(orientation=orientation):
                def reply(request, orientation=orientation):
                    value = model_reply(request)
                    value['pose']['orientation'] = orientation
                    return value
                control, process, spawn = self.controller(WireProcess(respond=reply))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))

    def test_model_pose_keeps_quaternion_sign_and_native_pitch_singularity_convention(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                orientation = {'x': 0., 'y': sign*math.sqrt(.5), 'z': 0., 'w': sign*math.sqrt(.5)}
                def reply(request):
                    value = model_reply(request)
                    value['pose']['orientation'] = orientation
                    value['rpy'] = [0., math.pi/2, 0.]
                    return value
                control, _, _ = self.controller(WireProcess(respond=reply))
                value = control.model_pose('crate', timeout=.3)
                self.assertEqual(value['pose']['orientation'], orientation)
                self.assertEqual(value['rpy'], [0., math.pi/2, 0.])

    def test_model_pose_requires_distinct_ordered_native_stats_and_matching_final_fence(self):
        variants = ({'pose_stats_before': None}, {'pose_stats_after': None},
            {'pose_stats_before': stats(seq=True)}, {'pose_stats_before': stats(seq=3)},
            {'pose_stats_before': stats(seq=4)}, {'pose_stats_before': stats(seq=2, sim_time=float('nan'))},
            {'pose_stats_after': stats(seq=3, iterations=41)}, {'pose_stats_after': stats(seq=4)})
        for changes in variants:
            with self.subTest(changes=changes):
                def reply(request, changes=changes):
                    return dict(model_reply(request), **changes)
                control, process, spawn = self.controller(WireProcess(respond=reply))
                self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))

    def test_model_pose_may_reuse_the_previous_stats_sample_as_its_left_fence(self):
        before, after = stats(seq=1), stats(seq=2)
        control, process, _ = self.controller(WireProcess(respond=lambda request: model_reply(request, before, after)))
        value = control.model_pose('crate', timeout=.3)
        self.assertEqual(value['pose_stats_before']['seq'], 1)
        self.assertEqual(value['pose_stats_after']['seq'], 2)
        self.assertEqual(len(process.requests), 1)

    def test_model_pose_rejects_a_left_fence_older_than_prior_confirmed_statistics(self):
        def reply(request):
            if request['action'] == 'status':
                return {'id': request['id'], 'ok': True, 'stats': stats(seq=2)}
            return model_reply(request, stats(seq=1), stats(seq=3))
        control, process, spawn = self.controller(WireProcess(respond=reply))
        control.status(timeout=.3)
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))
        self.assertEqual([request['action'] for request in process.requests], ['status', 'model_pose'])

    def test_model_pose_keeps_seq_ordered_brackets_when_an_external_clock_reset_occurs(self):
        before = stats(seq=2, iterations=2400, sim_time=2.4)
        after = stats(seq=3, iterations=0, sim_time=0.)
        control, process, _ = self.controller(WireProcess(respond=lambda request: model_reply(request, before, after)))
        value = control.model_pose('crate', timeout=.3)
        self.assertGreater(value['pose_stats_after']['seq'], value['pose_stats_before']['seq'])
        self.assertLess(value['pose_stats_after']['sim_time'], value['pose_stats_before']['sim_time'])
        self.assertEqual(len(process.requests), 1)

    def test_model_pose_timeout_is_one_wire_query_and_forbids_later_mutation(self):
        control, process, spawn = self.controller(WireProcess(respond=lambda request: None))
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.model_pose('crate', timeout=.3))
        self.assertEqual([request['action'] for request in process.requests], ['model_pose'])

    def test_model_pose_native_refusal_keeps_read_correlation_diagnosis(self):
        process = WireProcess(respond=lambda request: {'id': request['id'], 'ok': False,
            'error': 'entity_info request correlation/type failed'})
        control, process, spawn = self.controller(process)
        with self.assertRaisesRegex(RuntimeError, 'entity_info request correlation/type failed'):
            control.model_pose('crate', timeout=.3)
        self.assert_poisoned_without_replay(control, process, spawn, lambda: control.pause(False, timeout=.3))
        self.assertEqual([request['action'] for request in process.requests], ['model_pose'])

    def runtime_session(self, control):
        source = self.root/'lesson-source'
        source.mkdir(exist_ok=True)
        (source/'world.sdf').write_text("<sdf version='1.6'><world name='fixture_world'/></sdf>")
        (source/'lesson.md').write_text('# Native dispatch fixture\n')
        lesson = Lesson(source, {'id': '01', 'title': 'Native dispatch fixture', 'requires_ros': False})
        with mock.patch('gazeboteach.runtime.lesson_environment', return_value={
                'GAZEBO_MASTER_URI': 'http://127.0.0.1:11111', 'IGN_PARTITION': 'native-unit'}):
            session = LessonSession(self.root, lesson, storage=Storage(self.root/'runtime-data'))
        session.server = mock.Mock()
        session.server.poll.return_value = None
        session.state, session.world_name, session.native_control = '运行中', 'fixture_world', control
        session.command = mock.Mock(side_effect=AssertionError('SDK backend must not invoke legacy gz CLI'))
        return session

    def test_runtime_uses_one_sdk_helper_for_pause_step_reset_and_stats_without_gz_cli(self):
        observed = stats()
        def reply(request):
            baseline = observed['iterations']
            observed['seq'] += 1
            if request['action'] == 'pause':
                observed['paused'] = request['paused']
            if request['action'] == 'step':
                observed['iterations'] += request['count']
            response = {'id': request['id'], 'ok': True, 'stats': dict(observed)}
            if request['action'] == 'step':
                response.update(baseline_iterations=baseline, requested_steps=request['count'])
            return response
        control, process, spawn = self.controller(WireProcess(respond=reply))
        session = self.runtime_session(control)
        session.pause(False)
        session.step(3)
        session.reset_models()
        self.assertTrue(session._read_pause_state(timeout=.3))
        self.assertEqual([request['action'] for request in process.requests], ['pause', 'pause', 'step', 'reset_models', 'status'])
        self.assertEqual([row['event'] for row in session.storage.records()], ['resumed', 'paused', 'stepped', 'reset_models'])
        self.assertEqual(session.storage.records()[2]['details']['native_result']['stats']['iterations'], 43)
        session.command.assert_not_called()
        self.assertEqual(spawn.call_count, 1)

    def test_runtime_failed_pause_does_not_commit_state_or_fall_back_to_cli(self):
        process = WireProcess(respond=lambda request: {'id': request['id'], 'ok': True, 'stats': stats(seq=2, paused=False)})
        control, process, _ = self.controller(process)
        session = self.runtime_session(control)
        with self.assertRaises(RuntimeError):
            session.pause(True)
        self.assertEqual(session.state, '运行中')
        self.assertFalse(session.paused)
        self.assertEqual(session.storage.records(), [])
        session.command.assert_not_called()

    def test_runtime_foreign_step_never_records_step_success_or_retries_via_cli(self):
        def reply(request):
            if request['action'] == 'pause':
                return {'id': request['id'], 'ok': True, 'stats': stats(seq=2)}
            return {'id': request['id'], 'ok': True, 'baseline_iterations': 40,
                    'requested_steps': request['count'], 'stats': stats(seq=3, iterations=44)}
        control, process, _ = self.controller(WireProcess(respond=reply))
        session = self.runtime_session(control)
        with self.assertRaises(RuntimeError):
            session.step(3)
        self.assertEqual([row['event'] for row in session.storage.records()], ['paused'])
        self.assertEqual([row['action'] for row in process.requests], ['pause', 'step'])
        session.command.assert_not_called()


class NativeControlOptionalBackendTests(unittest.TestCase):
    def test_unsupported_host_or_missing_toolchain_is_an_explicit_optional_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for platform in ('nt', 'posix'):
                with self.subTest(platform=platform), mock.patch.object(native, 'os', SimpleNamespace(name=platform)), \
                        mock.patch.object(native.shutil, 'which', return_value=None), mock.patch.object(native.subprocess, 'Popen') as spawn:
                    helper, diagnosis = native.try_create(root, {}, 'default', root)
                    self.assertIsNone(helper)
                    self.assertEqual(diagnosis['backend'], 'legacy-cli')
                    self.assertFalse(diagnosis['supported'])
                    self.assertTrue(diagnosis['reason'])
                    spawn.assert_not_called()

    def test_supported_backend_startup_failure_cannot_silently_fall_back(self):
        with mock.patch.object(native, 'prepare', side_effect=RuntimeError('exact SDK compile failed')), \
                mock.patch.object(native, 'NativeControl') as create:
            with self.assertRaisesRegex(RuntimeError, 'exact SDK compile failed'):
                native.try_create('.', {}, 'default', '.')
            create.assert_not_called()


if __name__ == '__main__':
    unittest.main()
