"""Exercise L23's real Client read-connection boundary without ROS/Gazebo."""
import importlib.util
import os
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT/'courses/classic11_noetic/23-urdf-xacro/lab.py'
WORLD, JOINT = '/gazebo/get_world_properties', '/gazebo/get_joint_properties'


class ReadProxy:
    def __init__(self, answer):
        self.answer, self.calls, self.close_calls = answer, [], 0

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.answer(*args, **kwargs)

    def close(self):
        self.close_calls += 1


class Lesson23ReadServicesTests(unittest.TestCase):
    def setUp(self):
        config = ModuleType('config')
        config.JOINTS = ('lesson23_shoulder', 'lesson23_elbow')
        config.PACKAGE, config.edit, config.prepared = 'unit-package', mock.Mock(), mock.Mock()
        spec = importlib.util.spec_from_file_location('lesson23_read_service_unit', LAB)
        self.lab = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'config': config}):
            spec.loader.exec_module(self.lab)
        self.subscription = mock.Mock()
        self.common = ModuleType('common')
        self.common.call = mock.Mock(return_value=SimpleNamespace(source='original common.call'))
        self.proxies, self.order = {}, []
        self.answers = {WORLD: lambda **kwargs: SimpleNamespace(sim_time=2.),
                        JOINT: lambda **kwargs: SimpleNamespace(position=[.1], rate=[.2], type=0, damping=[])}
        self.rospy = ModuleType('rospy')
        self.rospy.core = SimpleNamespace(is_initialized=lambda: True)
        self.rospy.Subscriber = mock.Mock(return_value=self.subscription)
        self.rospy.wait_for_service = mock.Mock()
        self.rospy.ServiceProxy = mock.Mock(side_effect=self.make_proxy)
        std_msg = ModuleType('std_msgs.msg')
        std_msg.String = type('String', (), {})
        self.world_type, self.joint_type = type('GetWorldProperties', (), {}), type('GetJointProperties', (), {})
        gazebo_srv = ModuleType('gazebo_msgs.srv')
        gazebo_srv.GetWorldProperties, gazebo_srv.GetJointProperties = self.world_type, self.joint_type
        modules = {'rospy': self.rospy, 'common': self.common, 'std_msgs': ModuleType('std_msgs'),
                   'std_msgs.msg': std_msg, 'gazebo_msgs': ModuleType('gazebo_msgs'), 'gazebo_msgs.srv': gazebo_srv}
        patcher = mock.patch.dict(sys.modules, modules)
        patcher.start()
        self.addCleanup(patcher.stop)
        original_path = list(sys.path)
        self.addCleanup(lambda: sys.path.__setitem__(slice(None), original_path))
        self.client = self.lab.Client({}, {})
        self.addCleanup(self.client.close)

    def make_proxy(self, name, service, persistent=False):
        self.assertTrue(persistent)
        def answer(*args, **kwargs):
            self.order.append((name, kwargs))
            return self.answers[name](*args, **kwargs)
        proxy = ReadProxy(answer)
        self.proxies[name] = proxy
        return proxy

    def test_samples_reuse_two_connections_and_keep_all_four_actual_read_fences(self):
        for _ in range(4):
            sample = self.client.sample()
            self.assertEqual(sample['world_time_before_ns'], 2000000000)
            self.assertEqual(sample['world_time_after_ns'], 2000000000)
            self.assertEqual(set(sample['joint_readback']), {'lesson23_shoulder', 'lesson23_elbow'})
        self.assertEqual([name for name, _ in self.order], [WORLD, JOINT, JOINT, WORLD]*4)
        self.assertEqual(self.rospy.ServiceProxy.call_count, 2)
        self.assertEqual(self.rospy.wait_for_service.call_count, 2)
        self.assertEqual(len(self.proxies[WORLD].calls), 8)
        self.assertEqual(len(self.proxies[JOINT].calls), 8)
        self.assertTrue(all(call.kwargs['persistent'] is True for call in self.rospy.ServiceProxy.call_args_list))
        self.common.call.assert_not_called()
        self.client.close()
        self.client.close()
        self.assertEqual([proxy.close_calls for proxy in self.proxies.values()], [1, 1])
        with self.assertRaisesRegex(RuntimeError, 'client closed'):
            self.client.world_time_ns()
        self.assertEqual(self.rospy.ServiceProxy.call_count, 2)

    def test_mutations_and_other_reads_keep_original_call_path_without_reuse(self):
        for name in ('/gazebo/apply_joint_effort', '/gazebo/pause_physics', '/gazebo/set_link_state', '/gazebo/get_link_state'):
            service, payload = object(), object()
            value = self.client.call(name, service, payload, example=3)
            self.assertEqual(value.source, 'original common.call')
            self.common.call.assert_called_with(name, service, payload, example=3)
        self.assertEqual(self.common.call.call_count, 4)
        self.rospy.ServiceProxy.assert_not_called()
        self.rospy.wait_for_service.assert_not_called()

    def test_read_failure_closes_cached_connections_and_is_not_resent(self):
        for refusal in (False, True):
            with self.subTest(refusal=refusal):
                # A fresh Client is needed after a failed read; the failed
                # client's cache is deliberately not reopened automatically.
                client = self.lab.Client({}, {})
                self.addCleanup(client.close)
                client.world_time_ns()
                def fail(**kwargs):
                    if refusal:
                        return SimpleNamespace(success=False, status_message='actual read refused')
                    raise RuntimeError('actual read transport failed')
                self.answers[JOINT] = fail
                with self.assertRaisesRegex(RuntimeError, 'actual read'):
                    client.joint('lesson23_elbow')
                created, calls = self.rospy.ServiceProxy.call_count, len(self.order)
                for read in (client.world_time_ns, lambda: client.joint('lesson23_elbow')):
                    with self.assertRaises(RuntimeError):
                        read()
                self.assertEqual(self.rospy.ServiceProxy.call_count, created)
                self.assertEqual(len(self.order), calls)
                self.assertEqual(len(self.proxies[JOINT].calls), 1)
                self.assertTrue(all(proxy.close_calls >= 1 for proxy in self.proxies.values()))
                # Restoration writes retain their independent original path.
                client.call('/gazebo/set_model_configuration', object(), example='restore')
                self.common.call.assert_called_with('/gazebo/set_model_configuration', mock.ANY, example='restore')

    def test_timed_out_read_is_disconnected_and_cannot_be_reused_while_worker_lives(self):
        entered, release = threading.Event(), threading.Event()
        def stalled(**kwargs):
            entered.set()
            release.wait(2)
            return SimpleNamespace(position=[.1], rate=[.2], type=0, damping=[])
        self.answers[JOINT] = stalled
        self.client._read_timeout = .05
        try:
            with self.assertRaisesRegex(RuntimeError, 'read timeout'):
                self.client.joint('lesson23_elbow')
            self.assertTrue(entered.is_set())
            proxy = self.proxies[JOINT]
            self.assertGreaterEqual(proxy.close_calls, 1)
            with self.assertRaises(RuntimeError):
                self.client.joint('lesson23_elbow')
            self.assertEqual(len(proxy.calls), 1)
            self.assertEqual(self.rospy.ServiceProxy.call_count, 1)
            self.client.close()
        finally:
            release.set()
        self.assertTrue(self.client._read_lock.acquire(timeout=1))
        self.client._read_lock.release()
        self.assertEqual(self.client._read_proxies, {})
        self.assertGreaterEqual(proxy.close_calls, 2)
        with self.assertRaises(RuntimeError):
            self.client.world_time_ns()
        self.assertEqual(self.rospy.ServiceProxy.call_count, 1)

    def test_timeout_during_service_discovery_cannot_create_a_late_connection(self):
        release = threading.Event()
        self.rospy.wait_for_service.side_effect = lambda *args, **kwargs: release.wait(2)
        self.client._read_timeout = .05
        try:
            with self.assertRaisesRegex(RuntimeError, 'read timeout'):
                self.client.world_time_ns()
        finally:
            release.set()
        self.assertTrue(self.client._read_lock.acquire(timeout=1))
        self.client._read_lock.release()
        self.rospy.ServiceProxy.assert_not_called()
        with self.assertRaises(RuntimeError):
            self.client.world_time_ns()
        self.assertEqual(self.rospy.wait_for_service.call_count, 1)

    def test_concurrent_read_callers_are_serialized_on_the_one_cached_proxy(self):
        entered, release, lock = threading.Event(), threading.Event(), threading.Lock()
        active, maximum, failures = [0], [0], []
        def measured(**kwargs):
            with lock:
                active[0] += 1
                maximum[0] = max(maximum[0], active[0])
            entered.set()
            release.wait(1)
            with lock:
                active[0] -= 1
            return SimpleNamespace(position=[.1], rate=[.2], type=0, damping=[])
        self.answers[JOINT] = measured
        self.client._read_timeout = 1.
        def read():
            try:
                self.client.joint('lesson23_elbow')
            except Exception as error:
                failures.append(error)
        first, second = threading.Thread(target=read), threading.Thread(target=read)
        first.start()
        self.assertTrue(entered.wait(1))
        second.start()
        time.sleep(.02)
        self.assertEqual(maximum[0], 1)
        release.set()
        first.join(1)
        second.join(1)
        self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(failures, [])
        self.assertEqual(maximum[0], 1)
        self.assertEqual(self.rospy.ServiceProxy.call_count, 1)
        self.assertEqual(len(self.proxies[JOINT].calls), 2)

    def test_disconnect_errors_do_not_skip_other_connections_or_subscription(self):
        self.client.world_time_ns()
        self.client.joint('lesson23_elbow')
        def broken_close():
            raise RuntimeError('injected disconnect failure')
        self.proxies[WORLD].close = broken_close
        self.client.close()
        self.assertEqual(self.proxies[JOINT].close_calls, 1)
        self.subscription.unregister.assert_called_once()
        self.assertEqual(self.client._read_close_errors, ['injected disconnect failure'])


if __name__ == '__main__':
    unittest.main()
