#!/usr/bin/env python3
"""Bounded wall-clock CLI for the persistent physical mission controller."""
import argparse
import json
from pathlib import Path
import threading
import time
from config import identity, prepared
from mission import bounded


class Client:
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        self.parameters,self.record=prepared();self.rospy=rospy;self.state=None;self.configuration=None
        rospy.init_node('lesson36_cli',anonymous=True,disable_signals=True)
        self.subs=[rospy.Subscriber('/lesson36/state',String,lambda m:setattr(self,'state',json.loads(m.data)),queue_size=10),
                   rospy.Subscriber('/lesson36/configuration',String,lambda m:setattr(self,'configuration',json.loads(m.data)),queue_size=1)]

    def call(self,path,kind,**kwargs):
        def invoke():
            self.rospy.wait_for_service(path,timeout=3);proxy=self.rospy.ServiceProxy(path,kind)
            try: result=proxy(**kwargs)
            finally: proxy.close()
            if hasattr(result,'success') and not result.success:
                raise RuntimeError(path+': '+getattr(result,'message',getattr(result,'status_message','refused')))
            return result
        return bounded(invoke,9)

    def wait(self,predicate,seconds=10):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if predicate(): return self.state
            time.sleep(.03)
        raise RuntimeError('Finite wall deadline waiting for mission state')

    def pause(self,value):
        from std_srvs.srv import Empty
        from gazebo_msgs.srv import GetPhysicsProperties
        self.call('/gazebo/pause_physics' if value else '/gazebo/unpause_physics',Empty)
        end=time.monotonic()+5
        while time.monotonic()<end:
            state=self.call('/gazebo/get_physics_properties',GetPhysicsProperties)
            if state.pause==value: return
            time.sleep(.03)
        raise RuntimeError('Actual physics pause state differs')

    def ready(self):
        from gazebo_msgs.srv import GetJointProperties,GetModelProperties
        self.wait(lambda:self.state is not None and self.configuration is not None,15)
        if self.configuration['identity']!=identity(self.record): raise RuntimeError('Loaded controller configuration differs')
        if self.rospy.get_param('/warehouse/tf_prefix')!='': raise RuntimeError('Empty explicit tf_prefix required')
        model=self.call('/gazebo/get_model_properties',GetModelProperties,model_name='warehouse_bot')
        joint=self.call('/gazebo/get_joint_properties',GetJointProperties,joint_name='warehouse_door::slide_joint')
        if model.is_static or not {'warehouse_left_joint','warehouse_right_joint'}.issubset(model.joint_names):
            raise RuntimeError('Actual dynamic wheels missing')
        return {'ready':True,'state':self.state,'door_actual_position':list(joint.position),
                'note':'Readiness does not advance physics; bootstrap acquires physical scan and odometry'}

    def bootstrap(self):
        self.ready();start=self.state['sim_time'];self.pause(False)
        try: self.wait(lambda:self.state and self.state['sim_time']>=start+.6 and self.state['x'] is not None and self.state['front_m'] is not None,15)
        finally: self.pause(True)
        return self.state


def main():
    from std_srvs.srv import Trigger,SetBool,Empty
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['ready','bootstrap','snapshot','start','run','reset','enable-door','pause'])
    parser.add_argument('--enabled',choices=['true','false'],default='true');parser.add_argument('--world',action='store_true')
    args=parser.parse_args();client=Client()
    if args.action=='ready': result=client.ready()
    elif args.action=='bootstrap': result=client.bootstrap()
    elif args.action=='snapshot': result=client.wait(lambda:client.state is not None)
    elif args.action=='pause': client.pause(True);result=client.state
    elif args.action=='enable-door':
        result=client.call('/lesson36/enable_door',SetBool,data=args.enabled=='true').message
    elif args.action=='reset':
        client.pause(True)
        if args.world: client.call('/gazebo/reset_simulation',Empty)
        result=client.call('/lesson36/reset',Trigger).message
    else:
        client.ready();client.pause(False)
        try:
            # A paused reset deliberately invalidates caches. Acquire new real
            # feedback before accepting start, without faking a sensor sample.
            start=client.state['sim_time'];client.wait(lambda:client.state['sim_time']>=start+.4 and client.state['x'] is not None,15)
            run_id=client.call('/lesson36/start',Trigger).message
            result={'started':True,'run_id':run_id}
            if args.action=='run':
                state=client.wait(lambda:client.state['run_id']==run_id and client.state['phase'] in ('SUCCEEDED','FAILED'),100)
                result={'state':state,'result':str(Path(state['directory'])/'result.json')}
                if state['phase']!='SUCCEEDED': raise RuntimeError(json.dumps(result))
        finally:
            if args.action=='run': client.pause(True)
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__': main()
