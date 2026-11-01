#!/usr/bin/env python3
"""Finite wall-clock CLI for the persistent arm controller."""
import argparse
import json
from pathlib import Path
import time
from config import identity,prepared
from mission import bounded


class Client:
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        self.parameters,self.record=prepared();self.rospy=rospy;self.state=None;self.configuration=None
        rospy.init_node('lesson37_cli',anonymous=True,disable_signals=True)
        self.subs=[rospy.Subscriber('/lesson37/state',String,lambda m:setattr(self,'state',json.loads(m.data)),queue_size=10),
                   rospy.Subscriber('/lesson37/configuration',String,lambda m:setattr(self,'configuration',json.loads(m.data)),queue_size=1)]

    def call(self,path,kind,**kwargs):
        def invoke():
            self.rospy.wait_for_service(path,timeout=3);proxy=self.rospy.ServiceProxy(path,kind)
            try:result=proxy(**kwargs)
            finally:proxy.close()
            if hasattr(result,'success') and not result.success:
                raise RuntimeError(path+': '+getattr(result,'message',getattr(result,'status_message','refused')))
            return result
        return bounded(invoke,9)

    def wait(self,predicate,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate():return self.state
            time.sleep(.025)
        raise RuntimeError('Finite wall deadline waiting for arm mission')

    def pause(self,value):
        from std_srvs.srv import Empty
        from gazebo_msgs.srv import GetPhysicsProperties
        self.call('/gazebo/pause_physics' if value else '/gazebo/unpause_physics',Empty)
        end=time.monotonic()+5
        while time.monotonic()<end:
            if self.call('/gazebo/get_physics_properties',GetPhysicsProperties).pause==value:return
            time.sleep(.025)
        raise RuntimeError('Actual physics pause state differs')

    def ready(self):
        from gazebo_msgs.srv import GetJointProperties,GetModelProperties,GetLinkState
        self.wait(lambda:self.state is not None and self.configuration is not None,15)
        if self.configuration['identity']!=identity(self.record):raise RuntimeError('Loaded controller identity differs')
        if self.rospy.get_param('/arm_push/tf_prefix')!='':raise RuntimeError('Empty explicit FT tf_prefix required')
        model=self.call('/gazebo/get_model_properties',GetModelProperties,model_name='push_arm')
        for name in ('shoulder_joint','elbow_joint'):
            joint=self.call('/gazebo/get_joint_properties',GetJointProperties,joint_name='push_arm::'+name)
            if len(joint.position)!=1:raise RuntimeError('Actual actuated joint missing: '+name)
        if model.is_static:raise RuntimeError('Arm must be dynamic with world-fixed base')
        tip=self.call('/gazebo/get_link_state',GetLinkState,link_name='push_arm::pusher',reference_frame='world')
        return {'ready':True,'state':self.state,'actual_tip':[tip.link_state.pose.position.x,tip.link_state.pose.position.y,tip.link_state.pose.position.z],
                'note':'Readiness works at paused t=0; bootstrap acquires actual FT/contact feedback'}

    def bootstrap(self):
        self.ready();start=self.state['sim_time'];self.pause(False)
        try:self.wait(lambda:self.state['sim_time']>=start+.6 and self.state.get('force') is not None and self.state.get('truth') is not None,15)
        finally:self.pause(True)
        return self.state

    def await_result(self,run_id):
        state=self.wait(lambda:self.state['run_id']==run_id and self.state['phase'] in ('SUCCEEDED','FAILED'),45)
        path=Path(state['directory'])/'result.json'
        self.wait(lambda:path.is_file(),8)
        # Do not pause at the first terminal state notification: let the real
        # hold or zero-torque update execute and advance another .25 second.
        initial=self.state['sim_time']
        self.wait(lambda:self.state['sim_time']>=initial+.25,8)
        return {'state':self.state,'result':str(path)}


def main():
    from std_srvs.srv import Trigger,SetBool,Empty
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['ready','bootstrap','snapshot','start','run','reset','enable-arm','pause'])
    parser.add_argument('--enabled',choices=['true','false'],default='true');parser.add_argument('--world',action='store_true')
    args=parser.parse_args();client=Client()
    if args.action=='ready':result=client.ready()
    elif args.action=='bootstrap':result=client.bootstrap()
    elif args.action=='snapshot':result=client.wait(lambda:client.state is not None)
    elif args.action=='pause':client.pause(True);result=client.state
    elif args.action=='enable-arm':result=client.call('/lesson37/enable_arm',SetBool,data=args.enabled=='true').message
    elif args.action=='reset':
        client.pause(True)
        # Stock Noetic FT retains its last publication time across a negative
        # clock jump. Reset all physical models while preserving simulation time.
        if args.world:client.call('/gazebo/reset_world',Empty)
        result=client.call('/lesson37/reset',Trigger).message
    else:
        client.ready();client.pause(False)
        try:
            initial=client.state['sim_time']
            client.wait(lambda:client.state['sim_time']>=initial+.5 and client.state['force'] is not None,15)
            run_id=client.call('/lesson37/start',Trigger).message
            result={'started':True,'run_id':run_id}
            if args.action=='run':
                result=client.await_result(run_id)
                if result['state']['phase']!='SUCCEEDED':raise RuntimeError(json.dumps(result))
        finally:
            if args.action=='run':client.pause(True)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
