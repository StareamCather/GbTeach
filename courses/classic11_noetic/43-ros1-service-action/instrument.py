#!/usr/bin/env python3
"""A43 only: one finite Trigger request and a custom real-physics MoveSlide action.

Caller wall deadlines do not imply that a transmitted RPC was cancelled.
Uncertain mutations fail without resending the original request.
"""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid


JOINT='workshop::slide'
PACKAGE='gazeboteach_workshop43'


class WallDeadline(RuntimeError):
    def __init__(self,description,request_sent=True):
        self.operation=description;self.request_sent=request_sent
        detail='remote outcome is unknown, no automatic replay' if request_sent else 'request not sent'
        super().__init__(description+' exceeded a monotonic wall deadline; '+detail)


def require(condition,message):
    if not condition:raise RuntimeError(message)


def bounded(function,seconds=6,description='ROS operation'):
    require(math.isfinite(seconds) and seconds>0,'Need a positive finite caller wall budget')
    completed=threading.Event();outcome={};deadline=time.monotonic()+seconds
    def invoke():
        try:outcome['value']=function()
        except BaseException as error:outcome['error']=error
        finally:completed.set()
    threading.Thread(target=invoke,daemon=True).start()
    if not completed.wait(max(0,deadline-time.monotonic())):raise WallDeadline(description)
    if 'error' in outcome:raise outcome['error']
    return outcome['value']


def wait_for(predicate,seconds=8,description='received ROS evidence'):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        value=predicate()
        if value:return value
        time.sleep(min(.02,max(0,deadline-time.monotonic())))
    raise WallDeadline(description,request_sent=False)


def initialize(role):
    import rospy
    if not rospy.core.is_initialized():rospy.init_node('workshop_43_'+role,anonymous=True)
    return rospy,rospy.get_namespace().rstrip('/') or '/workshop43'


def call(rospy,name,kind,*args,_deadline=None,**kwargs):
    budget=6 if _deadline is None else min(6,_deadline-time.monotonic())
    if budget<=0:raise WallDeadline(name,request_sent=False)
    def invoke():
        rospy.wait_for_service(name,timeout=min(4,budget))
        proxy=rospy.ServiceProxy(name,kind)
        try:
            response=proxy(*args,**kwargs)
            require(not hasattr(response,'success') or response.success,
                    name+': '+(getattr(response,'status_message','') or getattr(response,'message','') or 'request rejected'))
            return response
        finally:proxy.close()
    return bounded(invoke,budget,name)


def joint(rospy,_deadline=None):
    from gazebo_msgs.srv import GetJointProperties
    before=time.monotonic_ns();clock_before=rospy.Time.now().to_sec()
    response=call(rospy,'/gazebo/get_joint_properties',GetJointProperties,JOINT,_deadline=_deadline)
    require(response.position and response.rate,'Actual joint state is empty')
    x,v=float(response.position[0]),float(response.rate[0])
    require(math.isfinite(x) and math.isfinite(v),'Actual joint state is not finite')
    return {'position_m':x,'velocity_m_s':v,'receipt_clock_before_s':clock_before,
        'receipt_clock_after_s':rospy.Time.now().to_sec(),'host_monotonic_before_ns':before,
        'host_monotonic_after_ns':time.monotonic_ns(),
        'time_basis':'Local clock readbacks bracket the actual GetJointProperties call; no native acquisition stamp or atomic sample claim'}


def effort(rospy,value,duration=.06,_deadline=None):
    from gazebo_msgs.srv import ApplyJointEffort
    require(math.isfinite(value) and abs(value)<=10,'A43 force must be finite within [-10,10] N')
    require(math.isfinite(duration) and 0<duration<=.3,'A43 effort duration must be within (0,.3] simulation seconds')
    return call(rospy,'/gazebo/apply_joint_effort',ApplyJointEffort,JOINT,value,
                rospy.Time(0),rospy.Duration(duration),_deadline=_deadline)


def clear(rospy,_deadline=None):
    from gazebo_msgs.srv import JointRequest
    # Noetic clearJointForces compares Joint::GetName(), unlike effort/readback
    # lookup which accepts a scoped name. This isolated world has one "slide".
    return call(rospy,'/gazebo/clear_joint_forces',JointRequest,JOINT.rsplit('::',1)[-1],_deadline=_deadline)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_controller(folder,mode):
    require(mode in ('reference_demo','student'),'controller_mode must be reference_demo or student')
    path=Path(folder)/('student.py' if mode=='student' else 'reference.py')
    spec=importlib.util.spec_from_file_location('a43_controller_'+uuid.uuid4().hex,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    require(callable(getattr(module,'controller',None)),'Controller file has no controller(target, position, velocity)')
    return module.controller,path


def check_student(folder=None):
    function,path=load_controller(folder or Path.cwd(),'student')
    for target,x,v in ((.35,0,0),(-.2,.3,.4),(.8,-.8,-2),(0,0,2),(0,.1,0)):
        actual=float(function(target,x,v));expected=max(-10,min(10,18*(target-x)-8*v))
        require(math.isfinite(actual) and abs(actual-expected)<1e-9,'Personal PD formula or signed saturation does not match the stated task')
    return {'algebra_checks_passed':True,'controller_sha256':sha(path),'assignment_pass':False,
        'reason':'These formula checks do not replace actual physical Action acceptance'}


def validate_goal(goal):
    if not math.isfinite(goal.target_m) or abs(goal.target_m)>1:return 'target_m must be finite within [-1,1] m'
    if not math.isfinite(goal.timeout_wall_s) or not .5<=goal.timeout_wall_s<=12:return 'timeout_wall_s must be finite within [.5,12] seconds'
    return None


def control_task(goal,observe,clear_force,apply_force,controller,preempt,feedback,
                 shutdown=lambda:False,clock=time.monotonic,sleep=time.sleep):
    """Testable physical control state machine; observer/actuator are real RPCs.

    One loop: read actual x/v -> feedback -> arrival/preemption check -> clear
    previous finite effort -> apply one bounded effort. Terminal acknowledgement
    is produced only after final force cleanup; it never means velocity is zero.
    """
    problem=validate_goal(goal)
    if problem:return {'state':'aborted','message':problem,'final':None,'clear_confirmed':True,'iterations':0,'poisoned':False}
    started=clock();deadline=started+goal.timeout_wall_s
    last=None;iterations=0;state='aborted';reason='Wall-clock task deadline exceeded';error=None
    try:
        while not shutdown():
            if preempt():state='preempted';reason='Own goal cancelled or preempted';break
            if clock()>=deadline:break
            last=observe(deadline)
            feedback(last,goal.target_m-last['position_m'],clock()-started)
            if preempt():state='preempted';reason='Own goal cancelled after actual feedback';break
            if abs(goal.target_m-last['position_m'])<.025 and abs(last['velocity_m_s'])<.07:
                state='succeeded';reason='Actual position and velocity satisfy arrival bounds';break
            command=float(controller(goal.target_m,last['position_m'],last['velocity_m_s']))
            require(math.isfinite(command) and abs(command)<=10,'Controller must return a finite, already limited force within [-10,10] N')
            clear_force(deadline)
            if preempt():state='preempted';reason='Own goal cancelled before the next finite effort';break
            apply_force(command,deadline);iterations+=1
            sleep(min(.03,max(0,deadline-clock())))
        else:reason='Action server is shutting down'
    except Exception as failure:
        error=failure;reason=type(failure).__name__+': '+str(failure);state='aborted'
    # A timed-out clear itself is NOT automatically resent. Other failures may
    # be followed by one distinct safety clear; no effort command is replayed.
    clear_confirmed=False
    if not (isinstance(error,WallDeadline) and 'clear_joint_forces' in error.operation):
        try:clear_force(clock()+1.5);clear_confirmed=True
        except Exception as failure:
            error=failure;reason+='; final force cleanup failed: '+str(failure);state='aborted'
    if clear_confirmed:
        try:last=observe(clock()+1)
        except Exception as failure:
            error=failure;reason+='; final actual readback failed: '+str(failure);state='aborted'
    if state=='succeeded' and preempt():state='preempted';reason='Cancel observed before terminal acknowledgement'
    if state=='succeeded' and (last is None or abs(goal.target_m-last['position_m'])>=.025 or abs(last['velocity_m_s'])>=.07):
        state='aborted';reason='Actual state moved outside arrival bounds before terminal acknowledgement'
    return {'state':state,'message':reason,'final':last,'clear_confirmed':clear_confirmed,
        'iterations':iterations,'elapsed_wall_s':clock()-started,
        'poisoned':isinstance(error,WallDeadline),'unknown_remote_outcome':str(error) if isinstance(error,WallDeadline) else None}


class SlideServer:
    def __init__(self,rospy,namespace):
        import actionlib
        from gazeboteach_workshop43.msg import MoveSlideAction,MoveSlideFeedback,MoveSlideResult
        self.rospy,self.ns=rospy,namespace
        self.Result,self.Feedback=MoveSlideResult,MoveSlideFeedback
        self.mode=rospy.get_param(namespace+'/controller_mode','reference_demo')
        self.controller,self.controller_path=load_controller(Path.cwd(),self.mode)
        self.controller_sha=sha(self.controller_path)
        self.actuation_lock=threading.RLock();self.fatal=threading.Event();self.fatal_message=None
        self.last=joint(rospy);self.events_lock=threading.Lock()
        self.server=actionlib.SimpleActionServer(namespace+'/move',MoveSlideAction,
            execute_cb=self.execute,auto_start=False)
        self.server.start()

    def event(self,record):
        directory=Path('build');directory.mkdir(exist_ok=True)
        with self.events_lock:
            with (directory/'action-events.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(record,allow_nan=False)+'\n')

    def execute(self,goal):
        result=self.Result();result.reached=False;result.final_position_m=self.last['position_m']
        problem=validate_goal(goal)
        if problem:
            result.message=problem;self.server.set_aborted(result,problem)
            self.event({'event':'invalid_goal_rejected','target_repr':repr(goal.target_m),
                'timeout_repr':repr(goal.timeout_wall_s),'reason':problem,'native_actuation_sent':False})
            return
        if self.fatal.is_set():
            result.message='Native RPC outcome uncertain; restart this owned experiment'
            self.server.set_aborted(result,result.message);return
        if not self.actuation_lock.acquire(blocking=False):
            result.message='A finite pulse is currently using this owned actuator'
            self.server.set_aborted(result,result.message);return
        try:
            def publish(observation,error,elapsed):
                self.last=observation
                feedback=self.Feedback()
                feedback.x_m,feedback.v_m_s=observation['position_m'],observation['velocity_m_s']
                feedback.error_m,feedback.elapsed_wall_s=error,elapsed
                self.server.publish_feedback(feedback)
            outcome=control_task(goal,
                observe=lambda deadline:joint(self.rospy,_deadline=deadline),
                clear_force=lambda deadline:clear(self.rospy,_deadline=deadline),
                apply_force=lambda value,deadline:effort(self.rospy,value,.06,_deadline=deadline),
                controller=self.controller,preempt=self.server.is_preempt_requested,
                feedback=publish,shutdown=self.rospy.is_shutdown)
            if outcome['final'] is not None:self.last=outcome['final']
            result.reached=outcome['state']=='succeeded'
            result.final_position_m=self.last['position_m']
            detail={'reason':outcome['message'],'clear_confirmed':outcome['clear_confirmed'],
                'final_velocity_m_s':self.last['velocity_m_s'],'elapsed_wall_s':outcome.get('elapsed_wall_s'),
                'controller_mode':self.mode,'controller_sha256':self.controller_sha,
                'unknown_remote_outcome':outcome.get('unknown_remote_outcome'),
                'cancel_semantics':'Pending finite effort cleared before acknowledgement; velocity is not overwritten'}
            result.message=json.dumps(detail,allow_nan=False)
            self.event({'event':'terminal','target_m':goal.target_m,'timeout_wall_s':goal.timeout_wall_s,
                        'controller_mode':self.mode,'outcome':outcome})
            if outcome['state']=='succeeded':self.server.set_succeeded(result,outcome['message'])
            elif outcome['state']=='preempted':self.server.set_preempted(result,outcome['message'])
            else:self.server.set_aborted(result,outcome['message'])
            if outcome['poisoned']:
                self.fatal_message=outcome['message'];self.fatal.set()
        finally:self.actuation_lock.release()

    def pulse(self,request):
        from std_srvs.srv import TriggerResponse
        if not self.actuation_lock.acquire(blocking=False):
            return TriggerResponse(False,'Action active: a service pulse cannot overwrite its force command')
        try:
            require(not self.fatal.is_set(),'Native outcome uncertain; restart this experiment')
            before=joint(self.rospy)
            magnitude=abs(float(self.rospy.get_param(self.ns+'/pulse_force',2.0)))
            require(.2<=magnitude<=5 and math.isfinite(magnitude),'Pulse magnitude must be finite within [.2,5] N')
            direction=1 if before['position_m']<=0 else -1
            clear(self.rospy);effort(self.rospy,direction*magnitude,.2)
            return TriggerResponse(True,json.dumps({'scheduled_once':True,'force_N':direction*magnitude,
                'duration_sim_s':.2,'before_joint':before,'task_reached':False},allow_nan=False))
        except WallDeadline as error:
            self.fatal_message=str(error);self.fatal.set();return TriggerResponse(False,str(error))
        except Exception as error:return TriggerResponse(False,type(error).__name__+': '+str(error))
        finally:self.actuation_lock.release()


def daemon():
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from gazeboteach_workshop43.msg import MoveSlideAction
    rospy,ns=initialize('instrument');control=SlideServer(rospy,ns)
    state_publisher=rospy.Publisher(ns+'/state',JointState,queue_size=8)
    ready_publisher=rospy.Publisher(ns+'/ready',String,queue_size=1,latch=True)
    pulse_service=rospy.Service(ns+'/pulse',Trigger,control.pulse)
    action_source=Path('ros_ws/src')/PACKAGE/'action/MoveSlide.action'
    status={'node':rospy.get_name(),'namespace':ns,'session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),
        'instrument_sha256':sha(__file__),'controller_mode':control.mode,
        'controller_sha256':control.controller_sha,'controller_file':str(control.controller_path),
        'action_definition_sha256':sha(action_source),'action_datatype':MoveSlideAction._type,
        'action_md5':MoveSlideAction._md5sum,'published_monotonic_ns':time.monotonic_ns(),
        'first_actual_joint':control.last,'ready_publications':1}
    ready_publisher.publish(String(data=json.dumps(status,allow_nan=False)))
    sequence=0
    try:
        while not rospy.is_shutdown():
            if control.fatal.is_set():
                time.sleep(.2)  # Let the already published terminal result leave its queue.
                raise RuntimeError('A43 native outcome uncertain, instrument stops: '+str(control.fatal_message))
            observation=joint(rospy);control.last=observation
            message=JointState();message.header.seq=sequence
            message.header.stamp=rospy.Time.now();message.header.frame_id='workshop_slide_zero'
            message.name=['slide'];message.position=[observation['position_m']]
            message.velocity=[observation['velocity_m_s']]
            # Effort is not a native measurement in GetJointProperties.
            message.effort=[]
            state_publisher.publish(message);sequence+=1;time.sleep(.04)
    finally:
        # Do not race the Action execute thread's terminal cleanup. If it owns
        # the actuator it sees rospy shutdown and performs its own one cleanup.
        if control.actuation_lock.acquire(blocking=False):
            try:
                if not control.fatal.is_set():clear(rospy)
            except Exception as error:rospy.logwarn('A43 final force cleanup failed: %s',error)
            finally:control.actuation_lock.release()
        pulse_service.shutdown('A43 instrument ended')
        state_publisher.unregister();ready_publisher.unregister()


class Observations:
    def __init__(self):self.lock=threading.Lock();self.status=[];self.states=deque(maxlen=600)
    def ready(self,message):
        try:row={'value':json.loads(message.data),'connection':dict(getattr(message,'_connection_header',{}) or {})}
        except Exception as error:row={'error':str(error)}
        with self.lock:self.status.append(row)
    def state(self,message):
        if message.name!=['slide'] or len(message.position)!=1 or len(message.velocity)!=1:return
        row={'x_m':float(message.position[0]),'v_m_s':float(message.velocity[0]),
            'stamp_s':message.header.stamp.to_sec(),'receipt_monotonic_ns':time.monotonic_ns(),
            'callerid':(getattr(message,'_connection_header',{}) or {}).get('callerid')}
        with self.lock:self.states.append(row)
    def snapshot(self):
        with self.lock:return list(self.status),list(self.states)


def subscribe(rospy,ns):
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    records=Observations()
    subscriptions=[rospy.Subscriber(ns+'/ready',String,records.ready,queue_size=1),
                   rospy.Subscriber(ns+'/state',JointState,records.state,queue_size=30)]
    return records,subscriptions


def ready():
    rospy,ns=initialize('ready');records,subscriptions=subscribe(rospy,ns)
    try:
        wait_for(lambda:records.snapshot()[0] and records.snapshot()[1],description='A43 typed action instrument readiness')
        status=records.snapshot()[0][-1];require('error' not in status,'Malformed A43 readiness status')
        require(status['connection'].get('latching')=='1','Ready publisher is not latched')
        return {'ready':True,'status':status}
    finally:
        for subscription in subscriptions:subscription.unregister()


class MoveClient:
    def __init__(self,rospy,ns):
        import actionlib
        from gazeboteach_workshop43.msg import MoveSlideAction
        self.rospy=rospy;self.name=ns+'/move';self.lock=threading.Lock();self.feedback=[]
        self.client=actionlib.SimpleActionClient(self.name,MoveSlideAction)
        self.active=False;self.cancel_sent=False
        # actionlib's ROS Duration wait can otherwise depend on paused /clock.
        found=bounded(lambda:self.client.wait_for_server(rospy.Duration(4)),5,'Action server discovery')
        require(found,'MoveSlide server was not discovered')
    def receive(self,message):
        row={'x_m':message.x_m,'v_m_s':message.v_m_s,'error_m':message.error_m,
             'elapsed_wall_s':message.elapsed_wall_s,'receipt_monotonic_ns':time.monotonic_ns()}
        require(all(math.isfinite(row[k]) for k in ('x_m','v_m_s','error_m','elapsed_wall_s')),'Action feedback is not finite')
        with self.lock:self.feedback.append(row)
    def rows(self):
        with self.lock:return list(self.feedback)
    def send(self,target,timeout):
        from gazeboteach_workshop43.msg import MoveSlideGoal
        require(not self.active,'This client still owns an unacknowledged goal')
        with self.lock:self.feedback=[]
        goal=MoveSlideGoal();goal.target_m=target;goal.timeout_wall_s=timeout
        self.active=True;self.cancel_sent=False
        # Exactly one goal submission. An uncertain return is never resubmitted.
        bounded(lambda:self.client.send_goal(goal,feedback_cb=self.receive),2,'MoveSlide goal submission')
    def cancel(self):
        if self.active and not self.cancel_sent:
            self.cancel_sent=True
            bounded(self.client.cancel_goal,2,'Cancel own MoveSlide goal')
    def terminal(self):
        from actionlib_msgs.msg import GoalStatus
        terminal=(GoalStatus.SUCCEEDED,GoalStatus.PREEMPTED,GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.RECALLED,GoalStatus.LOST)
        return self.client.get_state() in terminal and self.client.get_result() is not None
    def finish(self,seconds):
        wait_for(self.terminal,seconds,'a terminal MoveSlide status/result using wall time')
        value=self.client.get_result();self.active=False
        return {'state':self.client.get_state(),'reached':value.reached,
                'final_position_m':value.final_position_m,'message':value.message,'feedback':self.rows()}
    def close(self):
        if self.active:
            self.cancel()
            try:self.finish(3)
            except Exception:pass
        self.client.stop_tracking_goal()


def reset_owned(rospy,records):
    from gazebo_msgs.srv import GetWorldProperties
    from std_srvs.srv import Empty
    before=call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time
    clear(rospy);call(rospy,'/gazebo/reset_world',Empty);after_reset_wall=time.monotonic_ns()
    wait_for(lambda:any(row['receipt_monotonic_ns']>after_reset_wall and abs(row['x_m'])<.01 and abs(row['v_m_s'])<.01
                        for row in records.snapshot()[1]),6,'fresh actual zero position/velocity after owned model reset')
    value=joint(rospy);after=call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time
    require(abs(value['position_m'])<.01 and abs(value['velocity_m_s'])<.01,'Actual joint did not reset')
    require(after>=before,'Model-only reset rewound the actual simulation clock')
    return {'service':'/gazebo/reset_world','native_world_time_before_s':before,
            'native_world_time_after_s':after,'after_joint':value,
            'scope':'Owned isolated course models/velocities only; clock and personal files preserved'}


def write_report(result):
    folder=Path('build');folder.mkdir(exist_ok=True);temporary=folder/('.verification-'+uuid.uuid4().hex+'.json')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(str(temporary),str(folder/'verification.json'))


def verify():
    from actionlib_msgs.msg import GoalStatus
    rospy,ns=initialize('verify');records,subscriptions=subscribe(rospy,ns);client=None;started=time.monotonic()
    result={'passed':False,'assignment_pass':False,'lesson':43,'checks':{},
        'trusted_verifier_sha256':sha(__file__),'world_sha256':sha('world.sdf'),
        'scope':'Custom MoveSlide Action on actual Gazebo joint, arrival/feedback/cancel/invalid inputs; no trajectory tracking claim'}
    try:
        wait_for(lambda:records.snapshot()[0] and len(records.snapshot()[1])>=6,8,'ready and actual JointState feedback')
        statuses,states=records.snapshot();identity=statuses[-1]['value']
        require(identity['instrument_sha256']==sha('instrument.py'),'Personal instrument changed; restart the experiment')
        require(identity['session_token']==os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'Readiness belongs to another session')
        mode=identity['controller_mode'];_,controller_path=load_controller(Path.cwd(),mode)
        require(identity['controller_sha256']==sha(controller_path),'Controller changed since loading; restart the experiment')
        require(all(row['callerid']==identity['node'] for row in states),'State publisher differs from the declared instrument')
        require(states[-1]['stamp_s']>states[0]['stamp_s'],'Actual simulation clock is not advancing')
        if mode=='student':result['checks']['student_algebra']=check_student()
        result['controller_mode']=mode;result['controller_sha256']=identity['controller_sha256']
        build=json.loads(Path('build/action-build.json').read_text(encoding='utf-8'))
        require(identity['action_md5']==build['action_md5'],'Loaded action interface differs from the prepared generated messages')
        require(identity['action_definition_sha256']==build['action_definition_sha256'],'Action source changed after message generation')
        result['checks']['generated_interface']=build
        client=MoveClient(rospy,ns)
        result['checks']['arrival_reset']=reset_owned(rospy,records)
        client.send(.35,8);arrival=client.finish(11)
        actual=joint(rospy)
        require(arrival['state']==GoalStatus.SUCCEEDED and arrival['reached'],'Action did not report actual arrival: '+str(arrival))
        require(arrival['feedback'] and abs(actual['position_m']-.35)<.05 and abs(actual['velocity_m_s'])<.10,
                'Actual arrival or feedback disagrees with the Action result')
        require(all(abs(row['error_m']-(.35-row['x_m']))<1e-9 for row in arrival['feedback']),'Action feedback error does not match its actual joint state')
        result['checks']['arrival']={'action':arrival,'independent_joint':actual}
        result['checks']['cancel_reset']=reset_owned(rospy,records)
        client.send(-.7,8)
        wait_for(lambda:any(abs(row['x_m'])>.025 for row in client.rows()),5,'actual Action feedback and motion before cancellation')
        cancel_before=joint(rospy);client.cancel();cancelled=client.finish(4);cancel_after=joint(rospy)
        require(cancelled['state']==GoalStatus.PREEMPTED and not cancelled['reached'],'Own goal did not acknowledge cancellation')
        details=json.loads(cancelled['message']);require(details['clear_confirmed'],'Cancel was acknowledged before pending effort cleanup')
        result['checks']['cancel']={'action':cancelled,'before_joint':cancel_before,'after_joint':cancel_after,
            'semantics':'Pending force cleared; finite remaining velocity is allowed and recorded'}
        result['checks']['invalid_reset']=reset_owned(rospy,records)
        failures=[]
        for target,timeout in ((2.,8),(-2.,8),(float('nan'),8),(float('inf'),8),(.2,0),(.2,float('nan'))):
            before=joint(rospy);client.send(target,timeout);outcome=client.finish(3);after=joint(rospy)
            require(outcome['state']==GoalStatus.ABORTED and not outcome['reached'] and not outcome['feedback'],
                    'Invalid input was not rejected before control execution')
            require(abs(after['position_m']-before['position_m'])<.01,'Invalid input caused physical motion')
            failures.append({'target_repr':repr(target),'timeout_repr':repr(timeout),'action':outcome,
                             'actual_before':before,'actual_after':after})
        result['checks']['invalid_goals']=failures
        result['passed']=True;result['assignment_pass']=mode=='student'
        return result
    except BaseException as error:
        result['error']=type(error).__name__+': '+str(error);raise
    finally:
        if client:
            try:client.close()
            except Exception as error:result['client_cleanup_error']=str(error)
        for subscription in subscriptions:subscription.unregister()
        result['elapsed_wall_s']=time.monotonic()-started;write_report(result)


def action_command(rospy,ns,target,timeout,cancel_after=None):
    client=MoveClient(rospy,ns);started=time.monotonic()
    try:
        client.send(target,timeout)
        if cancel_after is not None:
            require(math.isfinite(cancel_after) and cancel_after>0,'cancel-after must be positive finite wall seconds')
            wait_for(lambda:client.terminal() or time.monotonic()-started>=cancel_after,
                     cancel_after+1,'own Action cancellation time')
            if not client.terminal():client.cancel()
        result=client.finish(max(3,min(16,timeout+3)))
        result['independent_joint']=joint(rospy);return result
    finally:client.close()


def main():
    import rospy
    parser=argparse.ArgumentParser(description='A43 finite service vs custom position Action with actual feedback')
    parser.add_argument('mode',choices=('daemon','ready','state','pulse','action','verify','student-check'),nargs='?',default='verify')
    parser.add_argument('--target',type=float,default=.35);parser.add_argument('--timeout',type=float,default=8)
    parser.add_argument('--cancel-after',type=float)
    arguments=parser.parse_args(rospy.myargv()[1:])
    if arguments.mode=='daemon':daemon();return
    if arguments.mode=='student-check':result=check_student()
    elif arguments.mode=='verify':result=verify()
    elif arguments.mode=='ready':result=ready()
    else:
        rospy,ns=initialize('operator')
        if arguments.mode=='state':result=joint(rospy)
        elif arguments.mode=='action':result=action_command(rospy,ns,arguments.target,arguments.timeout,arguments.cancel_after)
        else:
            from std_srvs.srv import Trigger
            response=call(rospy,ns+'/pulse',Trigger);result=json.loads(response.message)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()
