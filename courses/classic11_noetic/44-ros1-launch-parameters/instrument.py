#!/usr/bin/env python3
"""A44 only: launch scope, typed parameters, cached startup scale and live finite pulse."""
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
PACKAGE='gazeboteach_workshop44'


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
    if not rospy.core.is_initialized():rospy.init_node('workshop_44_'+role,anonymous=True)
    return rospy,rospy.get_namespace().rstrip('/') or '/workshop44'


def call(rospy,name,kind,*args,_deadline=None,_check_success=True,**kwargs):
    budget=6 if _deadline is None else min(6,_deadline-time.monotonic())
    if budget<=0:raise WallDeadline(name,request_sent=False)
    def invoke():
        rospy.wait_for_service(name,timeout=min(4,budget))
        proxy=rospy.ServiceProxy(name,kind)
        try:
            response=proxy(*args,**kwargs)
            require(not _check_success or not hasattr(response,'success') or response.success,
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


def effort(rospy,value,duration=.2,start_time=None,_deadline=None):
    from gazebo_msgs.srv import ApplyJointEffort
    require(math.isfinite(value) and abs(value)<=5,'A44 effective force must be finite within [-5,5] N')
    require(math.isfinite(duration) and 0<duration<=.3,'A44 effort duration must be within (0,.3] simulation seconds')
    return call(rospy,'/gazebo/apply_joint_effort',ApplyJointEffort,JOINT,value,
                (rospy.Time.from_sec(start_time) if start_time is not None else rospy.Time(0)),rospy.Duration(duration),_deadline=_deadline)


def clear(rospy,_deadline=None):
    from gazebo_msgs.srv import JointRequest
    # Noetic clearJointForces compares Joint::GetName(), unlike effort/readback
    # lookup which accepts a scoped name. This isolated world has one "slide".
    return call(rospy,'/gazebo/clear_joint_forces',JointRequest,JOINT.rsplit('::',1)[-1],_deadline=_deadline)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_force(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .2<=value<=3:
        raise ValueError('pulse_force must be a real finite number within [.2,3] N; bool/string are not accepted')
    return float(value)


def validate_scale(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .5<=value<=1.5:
        raise ValueError('Startup pulse_scale must be a real finite number within [.5,1.5]')
    return float(value)


def typed(value):
    finite_number=isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)
    return {'type':type(value).__name__,'repr':repr(value),
            'value':value if isinstance(value,(str,bool)) or value is None or finite_number else None}


def get_param(rospy,name,default=None):
    return bounded(lambda:rospy.get_param(name,default),6,'XMLRPC get_param '+name)


def set_param(rospy,name,value):
    return bounded(lambda:rospy.set_param(name,value),6,'XMLRPC set_param '+name)


def world_time(rospy):
    from gazebo_msgs.srv import GetWorldProperties
    return float(call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time)


def final_clear(rospy,previous_failure=None):
    if isinstance(previous_failure,WallDeadline) and 'clear_joint_forces' in previous_failure.operation:
        raise RuntimeError('Previous clear outcome is uncertain; automatic cleanup replay is deliberately skipped')
    return clear(rospy)


class ParameterNode:
    def __init__(self,rospy,namespace):
        self.rospy,self.namespace=rospy,namespace
        self.private_prefix=rospy.get_name()
        self.force_name=rospy.resolve_name('~pulse_force')
        self.scale_name=rospy.resolve_name('~pulse_scale')
        self.relative_scope_name=rospy.resolve_name('scope_example')
        self.private_scope_name=rospy.resolve_name('~scope_example')
        initial_scale=get_param(rospy,self.scale_name)
        self.active_scale=validate_scale(initial_scale)
        self.scale_source={'parameter':self.scale_name,'read_policy':'read once during node startup, cached in this process',
                           'type':type(initial_scale).__name__}
        self.lock=threading.Lock();self.fatal=threading.Event();self.fatal_message=None

    def snapshot(self):
        return {'node':self.rospy.get_name(),'namespace':self.namespace,'private_prefix':self.private_prefix,
            'active_scale':self.active_scale,'startup_scale_source':self.scale_source,
            'force_parameter':self.force_name,'scale_parameter':self.scale_name,
            'requested_force':typed(get_param(self.rospy,self.force_name)),
            'server_scale':typed(get_param(self.rospy,self.scale_name)),
            'relative_scope':{'name':self.relative_scope_name,'value':get_param(self.rospy,self.relative_scope_name)},
            'private_scope':{'name':self.private_scope_name,'value':get_param(self.rospy,self.private_scope_name)},
            'actual_joint':joint(self.rospy),'dynamic_reconfigure':False,
            'force_read_policy':'rospy.get_param is explicitly called for every Trigger request'}

    def pulse(self,request):
        from std_srvs.srv import TriggerResponse
        if not self.lock.acquire(blocking=False):return TriggerResponse(False,'Another finite pulse request is being scheduled')
        try:
            require(not self.fatal.is_set(),'A native/XMLRPC outcome is uncertain; restart this isolated experiment')
            # Validate FIRST. A rejected bool/string/NaN/range never clears or
            # applies force and does not terminate the instrument process.
            requested=validate_force(get_param(self.rospy,self.force_name))
            effective=requested*self.active_scale
            require(math.isfinite(effective) and 0<effective<=5,'Effective pulse force exceeds the lesson actuator limit')
            current_server_scale=typed(get_param(self.rospy,self.scale_name))
            before=joint(self.rospy);clear(self.rospy)
            native_before=world_time(self.rospy);start=native_before+.08
            effort(self.rospy,effective,.2,start_time=start)
            ack=world_time(self.rospy)
            if ack>=start:
                self.fatal_message=json.dumps({'error':'Future-start timing outcome uncertain; Gazebo may have clamped a late request',
                    'planned_start_sim_s':start,'native_schedule_before_s':native_before,'native_schedule_ack_s':ack,
                    'effective_force_N':effective,'automatic_reschedule':False},allow_nan=False)
                self.fatal.set()
                return TriggerResponse(False,self.fatal_message)
            return TriggerResponse(True,json.dumps({'scheduled_once':True,'requested_force':requested,
                'active_scale':self.active_scale,'effective_force_N':effective,'force_parameter':self.force_name,
                'scale_parameter':self.scale_name,'server_scale':current_server_scale,'joint_before':before,
                'scheduled_start_sim_s':start,'duration_sim_s':.2,'native_schedule_before_s':native_before,
                'native_schedule_ack_s':ack,'task_finished':False,
                'parameter_policy':'live requested force times startup-cached scale'},allow_nan=False))
        except (ValueError,TypeError) as error:return TriggerResponse(False,type(error).__name__+': '+str(error))
        except WallDeadline as error:
            self.fatal_message=str(error);self.fatal.set();return TriggerResponse(False,str(error))
        except Exception as error:return TriggerResponse(False,type(error).__name__+': '+str(error))
        finally:self.lock.release()


def daemon():
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rospy,ns=initialize('instrument');node=ParameterNode(rospy,ns)
    publisher=rospy.Publisher(ns+'/state',String,queue_size=8)
    latch=rospy.Publisher(ns+'/ready',String,queue_size=1,latch=True)
    service=rospy.Service(ns+'/pulse',Trigger,node.pulse)
    status=node.snapshot();status.update(instrument_sha256=sha(__file__),world_sha256=sha('world.sdf'),
        session_token=os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),published_monotonic_ns=time.monotonic_ns())
    latch.publish(String(data=json.dumps(status,allow_nan=False)))
    try:
        while not rospy.is_shutdown():
            require(not node.fatal.is_set(),'A44 stops after uncertain RPC: '+str(node.fatal_message))
            publisher.publish(String(data=json.dumps(node.snapshot(),allow_nan=False)))
            time.sleep(.05)
    finally:
        if node.lock.acquire(blocking=False):
            try:
                if not node.fatal.is_set():clear(rospy)
            except Exception as error:rospy.logwarn('A44 force cleanup failed: %s',error)
            finally:node.lock.release()
        service.shutdown('A44 instrument ended');publisher.unregister();latch.unregister()


class Received:
    def __init__(self):self.lock=threading.Lock();self.status=[];self.states=deque(maxlen=200)
    def callback(self,kind,message):
        try:row={'value':json.loads(message.data),'receipt_monotonic_ns':time.monotonic_ns(),
                 'callerid':(getattr(message,'_connection_header',{}) or {}).get('callerid')}
        except Exception as error:row={'error':str(error)}
        with self.lock:(self.status if kind=='ready' else self.states).append(row)
    def snapshot(self):
        with self.lock:return list(self.status),list(self.states)


def subscribe(rospy,ns):
    from std_msgs.msg import String
    received=Received()
    subscriptions=[rospy.Subscriber(ns+'/'+kind,String,lambda message,k=kind:received.callback(k,message),queue_size=10)
                   for kind in ('ready','state')]
    return received,subscriptions


def ready():
    rospy,ns=initialize('ready');received,subscriptions=subscribe(rospy,ns)
    try:
        wait_for(lambda:received.snapshot()[0] and received.snapshot()[1],8,'A44 latched startup and current parameter status')
        status=received.snapshot()[0][-1];require('error' not in status,'Malformed A44 startup status')
        return {'ready':True,'namespace':ns,'active_scale':status['value']['active_scale'],'status':status}
    finally:
        for subscription in subscriptions:subscription.unregister()


def unit_response(elapsed,mass=1.,damping=.8,duration=.2):
    """Analytic unit-force response, used ONLY for comparison/time normalization.

    Measurements always come from native Gazebo services. This function never
    creates or publishes a measured position/velocity.
    """
    require(math.isfinite(elapsed) and elapsed>=0,'Need nonnegative finite elapsed simulation time')
    active=min(elapsed,duration);decay=damping/mass
    v=(1-math.exp(-decay*active))/damping
    x=(active-(1-math.exp(-decay*active))/decay)/damping
    tail=max(0,elapsed-duration)
    return x+v*(1-math.exp(-decay*tail))/decay,v*math.exp(-decay*tail)


def reset_owned(rospy,received):
    from std_srvs.srv import Empty
    before=world_time(rospy);clear(rospy);call(rospy,'/gazebo/reset_world',Empty)
    wall=time.monotonic_ns()
    wait_for(lambda:any(row.get('receipt_monotonic_ns',0)>wall and 'value' in row
        and abs(row['value']['actual_joint']['position_m'])<.01
        and abs(row['value']['actual_joint']['velocity_m_s'])<.01 for row in received.snapshot()[1]),
        6,'fresh zero joint state after owned model reset')
    after=joint(rospy);time_after=world_time(rospy)
    require(abs(after['position_m'])<.01 and abs(after['velocity_m_s'])<.01,'Actual owned joint did not reset')
    require(time_after>=before,'Owned model reset rewound simulation time')
    return {'before_native_world_s':before,'after_native_world_s':time_after,'after_actual_joint':after}


def sample_trial(rospy,ns,received,force_name,requested,scale):
    from std_srvs.srv import Trigger
    reset=reset_owned(rospy,received);set_param(rospy,force_name,requested)
    response=call(rospy,ns+'/pulse',Trigger);scheduled=json.loads(response.message)
    require(scheduled['requested_force']==requested and scheduled['active_scale']==scale,'Pulse did not use the expected live force/cached scale')
    require(abs(scheduled['effective_force_N']-requested*scale)<1e-12,'Effective force report differs from the actual scheduling inputs')
    start=scheduled['scheduled_start_sim_s'];deadline=time.monotonic()+5
    while world_time(rospy)<start+.45:
        if time.monotonic()>=deadline:raise WallDeadline('Actual simulation time reaching the A44 measurement window',False)
        time.sleep(.015)
    t_before=world_time(rospy);actual=joint(rospy);t_after=world_time(rospy)
    elapsed=(t_before+t_after)/2-start
    require(t_after-t_before<.04 and .44<=elapsed<=.65,'Native timing bracket is too broad or missed the measurement window')
    kx,kv=unit_response(elapsed);common_x,common_v=unit_response(.45)
    predicted_x=requested*scale*kx;predicted_v=requested*scale*kv
    require(abs(actual['position_m']-predicted_x)<max(.004,.12*predicted_x),'Actual displacement disagrees with the scheduled force and measured sim time')
    require(abs(actual['velocity_m_s']-predicted_v)<max(.008,.12*predicted_v),'Actual velocity disagrees with the scheduled force and measured sim time')
    return {'requested_force_N':requested,'active_scale':scale,'scheduled':scheduled,'reset':reset,
        'actual_joint':actual,'native_world_before_s':t_before,'native_world_after_s':t_after,
        'elapsed_sim_bracket_s':[t_before-start,t_after-start],'elapsed_sim_midpoint_s':elapsed,
        'analytic_expected_x_m':predicted_x,'analytic_expected_v_m_s':predicted_v,
        'common_elapsed_sim_s':.45,
        'measured_x_normalized_to_common_time_m':actual['position_m']*common_x/kx,
        'measured_v_normalized_to_common_time_m_s':actual['velocity_m_s']*common_v/kv,
        'normalization_note':'Raw native readings are retained; only time differences are corrected using the known 1 kg/.8 linear-damping response, not synthesized from the requested force'}


def write_report(result):
    folder=Path('build');folder.mkdir(exist_ok=True);temporary=folder/('.verification-'+uuid.uuid4().hex+'.json')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(str(temporary),str(folder/'verification.json'))


def verify():
    from std_srvs.srv import Trigger
    from gazebo_msgs.srv import GetLinkProperties
    import xml.etree.ElementTree as ET
    rospy,ns=initialize('verify');received,subscriptions=subscribe(rospy,ns);started=time.monotonic()
    original={};failure=None;result={'passed':False,'assignment_pass':False,'lesson':44,'namespace':ns,'checks':{},
        'trusted_verifier_sha256':sha(__file__),'world_sha256':sha('world.sdf')}
    try:
        wait_for(lambda:received.snapshot()[0] and len(received.snapshot()[1])>=3,8,'A44 startup and live parameter status')
        statuses,states=received.snapshot();identity=statuses[-1]['value']
        require(identity['instrument_sha256']==sha('instrument.py'),'Personal instrument changed since startup; restart this experiment')
        require(identity['world_sha256']==sha('world.sdf'),'Personal world changed since server startup; restart this experiment')
        require(identity['session_token']==os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'A44 startup belongs to another session')
        require(identity['namespace']==ns,'Instrument/CLI namespace mismatch')
        scale=validate_scale(identity['active_scale']);result['active_scale']=scale
        result['startup_scale_source']=identity['startup_scale_source']
        require(identity['relative_scope']['name']!=identity['private_scope']['name'],'Relative and private names unexpectedly resolve identically')
        require(identity['relative_scope']['value']=='group_scope' and identity['private_scope']['value']=='private_scope','Launch group/private YAML scope values are wrong')
        body=call(rospy,'/gazebo/get_link_properties',GetLinkProperties,'workshop::body')
        require(abs(body.mass-1)<1e-9 and not body.gravity_mode,'Actual mass/gravity differs from the stated linear teaching plant')
        world=ET.parse('world.sdf');damping=float(world.find("world/model[@name='workshop']/joint[@name='slide']/axis/dynamics/damping").text)
        require(abs(damping-.8)<1e-12,'The time normalization requires the loaded personal SDF damping .8')
        result['checks']['launch_scope']={'relative':identity['relative_scope'],'private':identity['private_scope'],
            'cached_scale':scale,'actual_mass_kg':body.mass,'sdf_damping':damping,
            'damping_scope':'SDF configuration .8; GetJointProperties damping is empty in the Noetic ROS API, not falsely treated as a measured field'}
        force_name,scale_name=identity['force_parameter'],identity['scale_parameter']
        original={force_name:get_param(rospy,force_name),scale_name:get_param(rospy,scale_name)}
        alternate=.5 if abs(scale-.5)>.1 else 1.25
        set_param(rospy,scale_name,alternate)
        require(get_param(rospy,scale_name)==alternate,'Parameter server did not acknowledge the new scale value')
        first=sample_trial(rospy,ns,received,force_name,1.,scale)
        second=sample_trial(rospy,ns,received,force_name,2.,scale)
        require(first['scheduled']['server_scale']['value']==alternate and first['scheduled']['active_scale']==scale,
                'The comparison did not retain cached startup scale after a parameter-server update')
        ratio_x=second['measured_x_normalized_to_common_time_m']/first['measured_x_normalized_to_common_time_m']
        ratio_v=second['measured_v_normalized_to_common_time_m_s']/first['measured_v_normalized_to_common_time_m_s']
        require(1.8<ratio_x<2.2 and 1.8<ratio_v<2.2,'Actual paired displacement/velocity do not scale with the live requested force')
        result['checks']['paired_force_trials']={'F':first,'2F':second,'measured_time_normalized_position_ratio':ratio_x,
            'measured_time_normalized_velocity_ratio':ratio_v,'common_elapsed_sim_s':.45,
            'server_scale_after_update':alternate,'actual_node_cached_scale':scale,
            'conclusion':'Parameter server update is visible but not automatically applied to the startup-cached scale; live force is explicitly re-read per request'}
        result['checks']['invalid_reset']=reset_owned(rospy,received)
        invalid=[]
        for value in (True,'2.0',float('nan'),float('inf'),.1,3.1):
            set_param(rospy,force_name,value);before=joint(rospy)
            # A typed rejection is expected, not a transport failure.
            response=call(rospy,ns+'/pulse',Trigger,_check_success=False)
            require(not response.success,'Invalid parameter was scheduled as physical force')
            time.sleep(.1);after=joint(rospy)
            require(abs(after['position_m']-before['position_m'])<.002 and abs(after['velocity_m_s'])<.01,
                    'Rejected parameter caused actual physical motion')
            marker=time.monotonic_ns()
            wait_for(lambda:any(row.get('receipt_monotonic_ns',0)>marker for row in received.snapshot()[1]),4,
                     'instrument remains alive after invalid parameter rejection')
            invalid.append({'parameter':typed(value),'rejection':response.message,'actual_before':before,'actual_after':after})
        result['checks']['invalid_parameter_rejections']=invalid
        set_param(rospy,force_name,1.)
        recovery=sample_trial(rospy,ns,received,force_name,1.,scale)
        result['checks']['recovery_after_parameter_correction']=recovery
        result['passed']=True;return result
    except BaseException as error:failure=error;result['error']=type(error).__name__+': '+str(error);raise
    finally:
        restoration={}
        for name,value in original.items():
            try:set_param(rospy,name,value);restoration[name]={'restored':True,'value':typed(get_param(rospy,name))}
            except Exception as error:restoration[name]={'restored':False,'error':str(error)};result['passed']=False
        result['parameter_restoration']=restoration
        try:final_clear(rospy,failure)
        except Exception as error:result['force_cleanup_error']=str(error);result['passed']=False
        for subscription in subscriptions:subscription.unregister()
        result['elapsed_wall_s']=time.monotonic()-started;write_report(result)
        if any(not item['restored'] for item in restoration.values()) or result.get('force_cleanup_error'):
            raise RuntimeError('A44 cleanup/restoration failed; see build/verification.json')


def check_student():
    path=Path.cwd()/'student.py';spec=importlib.util.spec_from_file_location('a44_student_'+uuid.uuid4().hex,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for value in (.2,1,2.,3):require(module.validate_force(value)==float(value),'Personal valid-parameter conversion is wrong')
    for value in (True,False,'2.0',float('nan'),float('inf'),.1,3.1):
        try:module.validate_force(value)
        except ValueError:continue
        raise RuntimeError('Personal validator accepted an invalid force: '+repr(value))
    return {'algebra_checks_passed':True,'assignment_pass':False,'student_sha256':sha(path),
        'scope':'Independent local parameter-validation exercise; the default live experiment does not execute or grade the student function'}


def main():
    import rospy
    parser=argparse.ArgumentParser(description='A44: private launch scope, startup cache and explicit live parameter reads')
    parser.add_argument('mode',choices=('daemon','ready','state','pulse','verify','student-check'),default='verify',nargs='?')
    arguments=parser.parse_args(rospy.myargv()[1:])
    if arguments.mode=='daemon':daemon();return
    if arguments.mode=='ready':result=ready()
    elif arguments.mode=='verify':result=verify()
    elif arguments.mode=='student-check':result=check_student()
    else:
        rospy,ns=initialize('operator')
        if arguments.mode=='pulse':
            from std_srvs.srv import Trigger
            result=json.loads(call(rospy,ns+'/pulse',Trigger).message)
        else:
            received,subscriptions=subscribe(rospy,ns)
            try:wait_for(lambda:received.snapshot()[1],4,'current A44 state');result=received.snapshot()[1][-1]['value']
            finally:
                for subscription in subscriptions:subscription.unregister()
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()


