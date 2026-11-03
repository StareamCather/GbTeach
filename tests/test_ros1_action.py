"""Control-protocol boundaries; these fixtures do not simulate Gazebo physics."""
import importlib.util
import os
import runpy
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest import mock


SOURCE=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/43-ros1-service-action/instrument.py'


def load():
    spec=importlib.util.spec_from_file_location('gazeboteach_a43_test_instrument',str(SOURCE))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class WallClock:
    def __init__(self):self.value=100.
    def __call__(self):return self.value
    def sleep(self,seconds):self.value+=seconds


class ActionBoundaries(unittest.TestCase):
    def setUp(self):self.module=load()

    def test_roslaunch_entry_locates_personal_files_from_script_not_ros_home(self):
        entry=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/43-ros1-service-action/ros_ws/src/gazeboteach_workshop43/scripts/node.py'
        before=Path.cwd();paths=list(sys.path);observed=[]
        fake=types.ModuleType('instrument');fake.main=lambda:observed.append((Path.cwd(),sys.path[0]))
        try:
            with mock.patch.dict(sys.modules,{'instrument':fake}):
                runpy.run_path(str(entry),run_name='__main__')
            expected=entry.resolve().parents[4]
            self.assertEqual(observed,[(expected,str(expected))])
        finally:
            os.chdir(str(before));sys.path[:]=paths

    def test_noetic_force_queue_clear_uses_bare_name_instead_of_scoped_lookup(self):
        package=types.ModuleType('gazebo_msgs');package.__path__=[]
        services=types.ModuleType('gazebo_msgs.srv');services.JointRequest=object
        with mock.patch.dict(sys.modules,{'gazebo_msgs':package,'gazebo_msgs.srv':services}), \
             mock.patch.object(self.module,'call') as call:
            self.module.clear('fake-rospy',_deadline=123)
        call.assert_called_once_with('fake-rospy','/gazebo/clear_joint_forces',object,'slide',_deadline=123)

    def exercise(self,observations,preempt_after_feedback=False,timeout=1):
        clock=WallClock();events=[];rows=list(observations);index=[0];cancelled=[False]
        def observe(deadline):
            value=rows[min(index[0],len(rows)-1)];index[0]+=1
            events.append(('read',value['position_m'],value['velocity_m_s']));return dict(value)
        def clear(deadline):events.append(('clear',))
        def apply(value,deadline):events.append(('apply',value))
        def feedback(row,error,elapsed):
            events.append(('feedback',row['position_m']))
            if preempt_after_feedback and abs(row['position_m'])>.025:cancelled[0]=True
        goal=types.SimpleNamespace(target_m=.35,timeout_wall_s=timeout)
        result=self.module.control_task(goal,observe,clear,apply,
            lambda target,x,v:max(-10,min(10,18*(target-x)-8*v)),lambda:cancelled[0],feedback,
            clock=clock,sleep=clock.sleep)
        return result,events

    def test_success_requires_actual_position_and_low_velocity_then_clear(self):
        outcome,events=self.exercise([{'position_m':0,'velocity_m_s':0},
            {'position_m':.349,'velocity_m_s':.3},  # Close in position but too fast.
            {'position_m':.349,'velocity_m_s':.02}])
        self.assertEqual(outcome['state'],'succeeded');self.assertTrue(outcome['clear_confirmed'])
        self.assertEqual(len([e for e in events if e[0]=='apply']),2)
        self.assertEqual(events[-2],('clear',),'Force must be cleared before the terminal actual read/ack')
        self.assertEqual(events[-1],('read',.349,.02))

    def test_cancel_after_motion_clears_force_without_fabricating_zero_velocity(self):
        outcome,events=self.exercise([{'position_m':0,'velocity_m_s':0},
            {'position_m':.04,'velocity_m_s':.31}],True)
        self.assertEqual(outcome['state'],'preempted');self.assertTrue(outcome['clear_confirmed'])
        self.assertEqual(outcome['final']['velocity_m_s'],.31)
        self.assertEqual(len([e for e in events if e[0]=='apply']),1)
        self.assertEqual(events[-2][0],'clear')

    def test_nonadvancing_simulation_cannot_prevent_wall_clock_abort(self):
        outcome,events=self.exercise([{'position_m':0,'velocity_m_s':0}],timeout=.5)
        self.assertEqual(outcome['state'],'aborted');self.assertTrue(outcome['clear_confirmed'])
        self.assertGreaterEqual(outcome['elapsed_wall_s'],.5)
        self.assertLess(outcome['elapsed_wall_s'],.51)
        self.assertFalse(outcome['poisoned'],'Expected task timeout differs from an uncertain RPC timeout')

    def test_invalid_goals_never_enter_observer_or_actuator(self):
        forbidden=mock.Mock(side_effect=AssertionError('Invalid input reached native control'))
        for target,timeout in [(2,8),(-2,8),(float('nan'),8),(float('inf'),8),(.2,0),(.2,float('nan'))]:
            goal=types.SimpleNamespace(target_m=target,timeout_wall_s=timeout)
            outcome=self.module.control_task(goal,forbidden,forbidden,forbidden,forbidden,forbidden,forbidden)
            self.assertEqual(outcome['state'],'aborted')
        forbidden.assert_not_called()

    def test_uncertain_clear_is_not_replayed_as_automatic_cleanup(self):
        calls=[]
        def unclear(deadline):calls.append('clear');raise self.module.WallDeadline('/gazebo/clear_joint_forces')
        outcome=self.module.control_task(types.SimpleNamespace(target_m=.35,timeout_wall_s=1),
            lambda deadline:{'position_m':0,'velocity_m_s':0},unclear,
            mock.Mock(side_effect=AssertionError('No force can follow an uncertain clear')),
            lambda *args:1,lambda:False,lambda *args:None)
        self.assertEqual(calls,['clear']);self.assertTrue(outcome['poisoned']);self.assertFalse(outcome['clear_confirmed'])

    def test_uncertain_effort_gets_one_distinct_safety_clear_without_effort_replay(self):
        calls=[]
        def apply(value,deadline):calls.append('effort');raise self.module.WallDeadline('/gazebo/apply_joint_effort')
        outcome=self.module.control_task(types.SimpleNamespace(target_m=.35,timeout_wall_s=1),
            lambda deadline:{'position_m':0,'velocity_m_s':0},lambda deadline:calls.append('clear'),apply,
            lambda *args:1,lambda:False,lambda *args:None)
        self.assertEqual(calls,['clear','effort','clear']);self.assertTrue(outcome['poisoned'])
        self.assertEqual(outcome['state'],'aborted')

    def test_action_discovery_has_a_caller_wall_bound_with_paused_ros_time(self):
        release=threading.Event();entered=threading.Event();finished=threading.Event()
        class FakeClient:
            def wait_for_server(self,duration):
                entered.set();release.wait();finished.set();return True
        fake_action=types.SimpleNamespace(SimpleActionClient=lambda *args:FakeClient())
        package=types.ModuleType('gazeboteach_workshop43');package.__path__=[]
        messages=types.ModuleType('gazeboteach_workshop43.msg');messages.MoveSlideAction=object
        original=self.module.bounded
        def short_bound(function,seconds=6,description='test'):
            return original(function,.04,description)
        started=time.monotonic()
        try:
            with mock.patch.dict(sys.modules,{'actionlib':fake_action,'gazeboteach_workshop43':package,
                                             'gazeboteach_workshop43.msg':messages}),\
                 mock.patch.object(self.module,'bounded',side_effect=short_bound):
                with self.assertRaisesRegex(self.module.WallDeadline,'Action server discovery'):
                    self.module.MoveClient(types.SimpleNamespace(Duration=lambda value:value),'/workshop43')
            self.assertTrue(entered.is_set());self.assertLess(time.monotonic()-started,1)
            self.assertFalse(finished.is_set())
        finally:release.set();self.assertTrue(finished.wait(1))

    def test_ros_remaps_are_preserved_outside_argparse(self):
        raw=['lab.py','state','__name:=instrument','__ns:=/workshop43','/workshop43/move:=/private/move']
        fake=types.SimpleNamespace(myargv=lambda:['lab.py','state'])
        with mock.patch.dict(sys.modules,{'rospy':fake}),mock.patch.object(sys,'argv',list(raw)),\
             mock.patch.object(self.module,'initialize',return_value=(fake,'/workshop43')),\
             mock.patch.object(self.module,'joint',return_value={'actual_joint':.1}),mock.patch('builtins.print'):
            self.module.main();self.assertEqual(sys.argv,raw)


if __name__=='__main__':unittest.main()
