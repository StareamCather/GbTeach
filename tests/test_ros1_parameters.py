"""A44 parameter/actuator boundary checks; real F/2F physics is acceptance44."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import time
import types
import unittest
from unittest import mock


SOURCE=Path(__file__).resolve().parents[1]/'courses/classic11_noetic/44-ros1-launch-parameters/instrument.py'


def load():
    spec=importlib.util.spec_from_file_location('gazeboteach_a44_test_instrument',str(SOURCE))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


class ROSParameters(unittest.TestCase):
    def setUp(self):
        self.module=load();self.prefix='/experiment44/instrument'
        self.params={self.prefix+'/pulse_scale':.5,self.prefix+'/pulse_force':1.,
            self.prefix+'/scope_example':'private_scope','/experiment44/scope_example':'group_scope'}
        def resolve(name):
            if name.startswith('~'):return self.prefix+'/'+name[1:]
            return name if name.startswith('/') else '/experiment44/'+name
        self.rospy=types.SimpleNamespace(get_name=lambda:self.prefix,resolve_name=resolve,
            get_param=lambda name,default=None:self.params.get(name,default),
            set_param=lambda name,value:self.params.__setitem__(name,value))
        package=types.ModuleType('std_srvs');package.__path__=[]
        services=types.ModuleType('std_srvs.srv')
        class Response:
            def __init__(self,success,message):self.success=success;self.message=message
        services.TriggerResponse=Response
        self.messages={'std_srvs':package,'std_srvs.srv':services}

    def test_yaml_boolean_string_nonfinite_and_range_are_not_force_numbers(self):
        for bad in (True,False,'2.0',None,float('nan'),float('inf'),.1,3.1):
            with self.assertRaises(ValueError):self.module.validate_force(bad)
        for valid in (.2,1,2.,3):self.assertEqual(self.module.validate_force(valid),float(valid))

    def test_startup_scale_is_cached_while_live_force_changes_actual_commands(self):
        node=self.module.ParameterNode(self.rospy,'/experiment44')
        self.params[self.prefix+'/pulse_scale']=1.25
        observation={'position_m':0,'velocity_m_s':0}
        with mock.patch.dict(sys.modules,self.messages),mock.patch.object(self.module,'joint',return_value=observation),\
             mock.patch.object(self.module,'clear'),mock.patch.object(self.module,'effort') as effort,\
             mock.patch.object(self.module,'world_time',side_effect=[100.,100.001,101.,101.001]):
            first=node.pulse(None);self.params[self.prefix+'/pulse_force']=2.;second=node.pulse(None)
        self.assertTrue(first.success and second.success)
        self.assertEqual([args.args[1] for args in effort.call_args_list],[.5,1.])
        first_json=json.loads(first.message);second_json=json.loads(second.message)
        self.assertEqual(first_json['server_scale']['value'],1.25)
        self.assertEqual(first_json['active_scale'],.5);self.assertEqual(second_json['requested_force'],2)
        self.assertEqual(first_json['force_parameter'],self.prefix+'/pulse_force')

    def test_invalid_parameter_never_reaches_native_read_clear_or_apply_and_recovers(self):
        node=self.module.ParameterNode(self.rospy,'/experiment44')
        with mock.patch.dict(sys.modules,self.messages),mock.patch.object(self.module,'joint') as joint,\
             mock.patch.object(self.module,'clear') as clear,mock.patch.object(self.module,'effort') as effort:
            for bad in (True,'2.0',float('nan'),.1,3.1):
                self.params[self.prefix+'/pulse_force']=bad
                self.assertFalse(node.pulse(None).success)
            joint.assert_not_called();clear.assert_not_called();effort.assert_not_called()
            self.assertFalse(node.fatal.is_set(),'Typed rejection must not kill the parameter node')
        self.params[self.prefix+'/pulse_force']=1.
        with mock.patch.dict(sys.modules,self.messages),\
             mock.patch.object(self.module,'joint',return_value={'position_m':0,'velocity_m_s':0}),\
             mock.patch.object(self.module,'clear'),mock.patch.object(self.module,'effort'),\
             mock.patch.object(self.module,'world_time',side_effect=[100.,100.001]):
            self.assertTrue(node.pulse(None).success)

    def test_future_start_missed_ack_is_unknown_and_never_automatically_rescheduled(self):
        node=self.module.ParameterNode(self.rospy,'/experiment44')
        with mock.patch.dict(sys.modules,self.messages),\
             mock.patch.object(self.module,'joint',return_value={'position_m':0,'velocity_m_s':0}),\
             mock.patch.object(self.module,'clear'),mock.patch.object(self.module,'effort') as effort,\
             mock.patch.object(self.module,'world_time',side_effect=[100.,100.2]):
            response=node.pulse(None)
        self.assertFalse(response.success);self.assertTrue(node.fatal.is_set())
        self.assertEqual(effort.call_count,1)
        detail=json.loads(response.message)
        self.assertFalse(detail['automatic_reschedule']);self.assertEqual(detail['planned_start_sim_s'],100.08)
        self.assertEqual(detail['native_schedule_ack_s'],100.2)

    def test_invalid_parameter_status_stays_strict_json(self):
        for value in (True,'2.0',float('nan'),float('inf'),None):
            text=json.dumps(self.module.typed(value),allow_nan=False)
            self.assertEqual(json.loads(text)['repr'],repr(value))

    def test_analytic_time_kernel_has_consistent_derivative_and_pulse_boundary(self):
        eps=1e-6
        for t in (.05,.15,.3,.45):
            x,v=self.module.unit_response(t)
            plus=self.module.unit_response(t+eps)[0];minus=self.module.unit_response(t-eps)[0]
            self.assertAlmostEqual((plus-minus)/(2*eps),v,places=7)
        before=self.module.unit_response(.2-eps);after=self.module.unit_response(.2+eps)
        self.assertLess(abs(before[0]-after[0]),1e-6);self.assertLess(abs(before[1]-after[1]),2e-6)

    def test_parameter_xmlrpc_wait_has_a_wall_bound_without_retry(self):
        entered=threading.Event();release=threading.Event();finished=threading.Event();calls=[]
        def blocked(name,default):
            calls.append(name);entered.set();release.wait();finished.set();return 1
        original=self.module.bounded
        try:
            with mock.patch.object(self.module,'bounded',side_effect=lambda function,seconds,description:original(function,.035,description)):
                with self.assertRaises(self.module.WallDeadline):
                    self.module.get_param(types.SimpleNamespace(get_param=blocked),'/owned/force')
            self.assertTrue(entered.is_set());self.assertEqual(calls,['/owned/force']);self.assertFalse(finished.is_set())
        finally:release.set();self.assertTrue(finished.wait(1))

    def test_uncertain_clear_is_not_replayed_by_verifier_final_cleanup(self):
        with mock.patch.object(self.module,'clear') as clear:
            with self.assertRaisesRegex(RuntimeError,'automatic cleanup replay.*skipped'):
                self.module.final_clear(self.rospy,self.module.WallDeadline('/gazebo/clear_joint_forces'))
            clear.assert_not_called()
            self.module.final_clear(self.rospy,ValueError('Bad typed parameter'))
            clear.assert_called_once_with(self.rospy)


if __name__=='__main__':unittest.main()
