"""A42 deadline semantics and ROS argv checks; real graph/physics needs smoke42."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest import mock


SOURCE=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/42-ros1-graph/instrument.py'


def load():
    spec=importlib.util.spec_from_file_location('gazeboteach_a42_test_instrument',str(SOURCE))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


class ROS1GraphBoundaries(unittest.TestCase):
    def test_blocked_worker_cannot_block_the_caller_indefinitely(self):
        module=load();release=threading.Event();entered=threading.Event();finished=threading.Event()
        def blocked():
            entered.set();release.wait();finished.set();return 'late'
        start=time.monotonic()
        try:
            with self.assertRaisesRegex(RuntimeError,'remote outcome is unknown.*no automatic replay'):
                module.bounded(blocked,.04,'deliberately blocked RPC')
            self.assertTrue(entered.is_set())
            self.assertLess(time.monotonic()-start,1)
            self.assertFalse(finished.is_set(),'Caller timeout must not falsely claim the underlying call was cancelled')
        finally:
            release.set();self.assertTrue(finished.wait(1))

    def test_unknown_mutation_is_neither_cancelled_nor_automatically_replayed(self):
        module=load();release=threading.Event();finished=threading.Event();sent=[];applied=[]
        def mutation():
            sent.append('one request');release.wait();applied.append('one eventual effect');finished.set()
        try:
            with self.assertRaisesRegex(RuntimeError,'outcome is unknown'):
                module.bounded(mutation,.025,'finite ApplyJointEffort test double')
            self.assertEqual(sent,['one request']);self.assertEqual(applied,[])
            release.set();self.assertTrue(finished.wait(1))
            self.assertEqual(applied,['one eventual effect']);self.assertEqual(len(sent),1)
        finally:release.set()

    def test_roslaunch_arguments_are_filtered_only_for_argparse(self):
        module=load()
        raw=['lab.py','state','__name:=instrument','__log:=/tmp/owned.log','__ns:=/workshop42','/workshop42/state:=/other/state']
        fake=types.SimpleNamespace(myargv=lambda:['lab.py','state'])
        with mock.patch.dict(sys.modules,{'rospy':fake}),mock.patch.object(sys,'argv',list(raw)),\
             mock.patch.object(module,'initialize',return_value=(fake,'/workshop42')) as initialize,\
             mock.patch.object(module,'joint',return_value={'actual_joint':.1}),contextlib.redirect_stdout(io.StringIO()):
            module.main()
            self.assertEqual(sys.argv,raw)
            initialize.assert_called_once_with('operator')

    def test_bounded_worker_preserves_service_failure_without_retries(self):
        module=load();calls=[]
        def refused():calls.append(1);raise RuntimeError('server rejected one request')
        with self.assertRaisesRegex(RuntimeError,'server rejected one request'):
            module.bounded(refused,.2)
        self.assertEqual(calls,[1])


if __name__=='__main__':unittest.main()
