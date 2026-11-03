#!/usr/bin/env python3
"""A42 only: real joint observations, typed pub/sub, one latched status, finite effort.

This instrument has no action, TF, bag or later-course implementation. The
service caller has a wall-clock budget; a timeout is an unknown remote outcome,
not a claim that ROS cancelled an already transmitted request.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid


LESSON=42
JOINT='workshop::slide'


def require(condition,message):
    if not condition: raise RuntimeError(message)


def bounded(function,seconds=6,description='ROS operation'):
    """Bound the caller even when a ROS XMLRPC/TCPROS read blocks internally.

    rospy can make its own sockets blocking, so socket.setdefaulttimeout alone
    is not a valid RPC deadline. A daemon worker performs exactly one call. On
    timeout the caller fails and never retries the request automatically.
    """
    completed=threading.Event(); outcome={}
    deadline=time.monotonic()+seconds
    def invoke():
        try: outcome['value']=function()
        except BaseException as error: outcome['error']=error
        finally: completed.set()
    threading.Thread(target=invoke,daemon=True).start()
    remaining=max(0,deadline-time.monotonic())
    if not completed.wait(remaining):
        raise RuntimeError(description+' exceeded a monotonic wall deadline; remote outcome is unknown, no automatic replay')
    if 'error' in outcome: raise outcome['error']
    return outcome['value']


def wait_for(predicate,seconds=8,description='live ROS evidence'):
    """Predicate only reads local received evidence, never performs another RPC."""
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        result=predicate()
        if result:return result
        time.sleep(min(.02,max(0,deadline-time.monotonic())))
    raise RuntimeError('Monotonic wall deadline waiting for '+description)


def initialize(role):
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node('workshop_42_'+role,anonymous=True)
    namespace=rospy.get_namespace().rstrip('/') or '/workshop42'
    return rospy,namespace


def call(rospy,name,kind,*args,**kwargs):
    def invoke():
        # The overall bounded worker includes discovery and response handling.
        rospy.wait_for_service(name,timeout=4)
        proxy=rospy.ServiceProxy(name,kind)
        try:
            response=proxy(*args,**kwargs)
            require(not hasattr(response,'success') or response.success,
                    name+': '+(getattr(response,'status_message','') or getattr(response,'message','') or 'request rejected'))
            return response
        finally:proxy.close()
    return bounded(invoke,6,'One service request '+name)


def joint(rospy):
    from gazebo_msgs.srv import GetJointProperties
    before_wall=time.monotonic_ns(); before_clock=rospy.Time.now().to_sec()
    value=call(rospy,'/gazebo/get_joint_properties',GetJointProperties,JOINT)
    after_clock=rospy.Time.now().to_sec(); after_wall=time.monotonic_ns()
    require(value.position and value.rate,'Actual Gazebo joint state is empty')
    x,v=float(value.position[0]),float(value.rate[0])
    require(math.isfinite(x) and math.isfinite(v),'Actual joint state is not finite')
    return {'position_m':x,'velocity_m_s':v,'receipt_clock_before_s':before_clock,
            'receipt_clock_after_s':after_clock,'host_monotonic_before_ns':before_wall,
            'host_monotonic_after_ns':after_wall,
            'time_basis':'Two local /clock readbacks bracketing a synchronous GetJointProperties call; no native acquisition stamp or atomic snapshot claim'}


def effort(rospy,value,duration=.2):
    from gazebo_msgs.srv import ApplyJointEffort
    require(math.isfinite(value) and .2<=abs(value)<=5,'A42 pulse magnitude must be within [.2,5] N')
    require(math.isfinite(duration) and 0<duration<=.3,'A42 pulse duration must be within (0,.3] simulation seconds')
    return call(rospy,'/gazebo/apply_joint_effort',ApplyJointEffort,
                JOINT,value,rospy.Time(0),rospy.Duration(duration))


def clear(rospy):
    from gazebo_msgs.srv import JointRequest
    # Noetic clears jobs by Joint::GetName(), unlike GetJoint/ApplyJointEffort
    # lookup. This isolated world has exactly one joint named slide.
    return call(rospy,'/gazebo/clear_joint_forces',JointRequest,'slide')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def daemon():
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,TriggerResponse
    rospy,ns=initialize('instrument')
    state_publisher=rospy.Publisher(ns+'/state',PoseStamped,queue_size=8)
    ready_publisher=rospy.Publisher(ns+'/ready',String,queue_size=1,latch=True)
    pulse_lock=threading.Lock()

    def pulse(request):
        try:
            with pulse_lock:
                observation=joint(rospy)
                magnitude=abs(float(rospy.get_param(ns+'/pulse_force',2.0)))
                direction=1 if observation['position_m']<=0 else -1
                clear(rospy)
                effort(rospy,direction*magnitude,.2)
                return TriggerResponse(True,json.dumps({'force_N':direction*magnitude,
                    'duration_sim_s':.2,'before_joint':observation,
                    'note':'Finite effort scheduled once; clearing force does not erase velocity'},allow_nan=False))
        except Exception as error:
            return TriggerResponse(False,type(error).__name__+': '+str(error))

    pulse_service=rospy.Service(ns+'/pulse',Trigger,pulse)
    first=joint(rospy)
    readiness={'node':rospy.get_name(),'namespace':ns,'session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),
        'instrument_sha256':sha(__file__),'published_monotonic_ns':time.monotonic_ns(),
        'joint_source':'/gazebo/get_joint_properties: '+JOINT,
        'state_topic':rospy.resolve_name(ns+'/state'),'pulse_service':rospy.resolve_name(ns+'/pulse'),
        'first_actual_joint':first,'ready_publications':1}
    ready_publisher.publish(String(data=json.dumps(readiness,allow_nan=False)))
    sequence=0
    try:
        while not rospy.is_shutdown():
            observation=joint(rospy)
            sample=PoseStamped()
            sample.header.seq=sequence
            # This is the observation node's current /clock readback after RPC,
            # not a timestamp supplied by GetJointProperties (it has none).
            sample.header.stamp=rospy.Time.now()
            sample.header.frame_id='workshop_slide_zero'
            sample.pose.position.x=observation['position_m']
            sample.pose.orientation.w=1
            state_publisher.publish(sample)
            sequence+=1
            # Wall sleep remains finite with paused /clock; the sample period
            # also includes service latency and is not a 25 Hz guarantee.
            time.sleep(.04)
    finally:
        try:clear(rospy)
        except Exception as error:rospy.logwarn('A42 finite-effort cleanup failed: %s',error)
        pulse_service.shutdown('A42 instrument ended')
        state_publisher.unregister();ready_publisher.unregister()


class Received:
    def __init__(self):self.lock=threading.Lock();self.status=[];self.samples=deque(maxlen=300)
    def ready(self,message):
        try:
            row={'value':json.loads(message.data),'connection':dict(getattr(message,'_connection_header',{}) or {}),
                 'receipt_monotonic_ns':time.monotonic_ns()}
        except Exception as error:
            row={'error':str(error)}
        with self.lock:self.status.append(row)
    def state(self,message):
        row={'seq':message.header.seq,'stamp_s':message.header.stamp.to_sec(),
             'frame':message.header.frame_id,'x_m':message.pose.position.x,
             'connection':dict(getattr(message,'_connection_header',{}) or {}),
             'receipt_monotonic_ns':time.monotonic_ns()}
        with self.lock:self.samples.append(row)
    def snapshot(self):
        with self.lock:return list(self.status),list(self.samples)


def ready():
    from std_msgs.msg import String
    rospy,ns=initialize('ready'); received=Received()
    subscription=rospy.Subscriber(ns+'/ready',String,received.ready,queue_size=1)
    try:
        wait_for(lambda:received.snapshot()[0],description='one latched A42 status')
        row=received.snapshot()[0][-1]
        require('error' not in row,'Malformed A42 readiness status: '+str(row))
        require(row['connection'].get('latching')=='1','Ready connection does not advertise latching')
        return {'ready':True,'status':row}
    finally:subscription.unregister()


def save_verification(result):
    destination=Path.cwd()/'build';destination.mkdir(exist_ok=True)
    temporary=destination/('.verification-'+uuid.uuid4().hex+'.json')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(str(temporary),str(destination/'verification.json'))


def verify():
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import rosgraph
    rospy,ns=initialize('verify'); received=Received(); subscriptions=[]
    created=time.monotonic_ns()
    result={'passed':False,'assignment_pass':False,'lesson':LESSON,
            'scope':'Actual ROS Master graph, TCPROS publisher/subscriber handshakes, one late latched readiness message, finite effort and independent physical joint feedback',
            'trusted_verifier_sha256':sha(__file__),'world_sha256':sha(Path.cwd()/'world.sdf'),'checks':{}}
    try:
        subscriptions=[rospy.Subscriber(ns+'/ready',String,received.ready,queue_size=1),
                       rospy.Subscriber(ns+'/state',PoseStamped,received.state,queue_size=30)]
        wait_for(lambda:len(received.snapshot()[0])>=1 and len(received.snapshot()[1])>=6,
                 description='late status and six actual state messages')
        statuses,samples=received.snapshot(); status=statuses[-1]
        require('error' not in status,'Malformed ready payload: '+str(status))
        identity=status['value'];publisher=identity['node']
        require(status['connection'].get('latching')=='1','Actual TCPROS ready publisher did not set latch')
        require(status['connection'].get('callerid')==publisher,'Ready caller differs from declared node')
        require(identity.get('published_monotonic_ns',created)>=0 and identity['published_monotonic_ns']<created,
                'This check requires a subscriber created after the one-time readiness publication')
        require(identity.get('ready_publications')==1,'Ready status must describe one publication')
        require(identity.get('session_token')==os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'Ready status belongs to another session')
        require(identity.get('instrument_sha256')==sha(Path.cwd()/'instrument.py'),
                'Personal instrument.py changed since the daemon loaded it; restart this experiment before acceptance')
        require(all(row['connection'].get('callerid')==publisher and row['connection'].get('type')=='geometry_msgs/PoseStamped' for row in samples),
                'State messages did not arrive from the expected typed TCPROS publisher')
        require(samples[-1]['stamp_s']>samples[0]['stamp_s'],'No advancing simulation clock in received observations')
        master=rosgraph.Master(rospy.get_name())
        publishers,subscribers,services=bounded(master.getSystemState,6,'ROS Master system state')
        types=dict(bounded(master.getTopicTypes,6,'ROS Master topic types'))
        state_name=rospy.resolve_name(ns+'/state');pulse_name=rospy.resolve_name(ns+'/pulse')
        published=dict(publishers);subscribed=dict(subscribers);provided=dict(services)
        require(types.get(state_name)=='geometry_msgs/PoseStamped','Wrong actual graph type')
        require(publisher in published.get(state_name,[]),'Declared instrument is not an actual graph publisher')
        require(rospy.get_name() in subscribed.get(state_name,[]),'Verifier is not registered as an actual state subscriber')
        require(publisher in provided.get(pulse_name,[]),'The instrument does not own the actual pulse service')
        result['checks'].update(graph_type=types[state_name],graph_publishers=published[state_name],
            graph_subscribers=subscribed[state_name],graph_pulse_servers=provided[pulse_name],
            observing_subscriber=rospy.get_name(),late_latched_status=status,state_connections=[r['connection'] for r in samples[:2]])
        # Acceptance deliberately resets this isolated course's models/velocities
        # once, so repeated checks cannot accumulate pulses at a rail end.
        # /reset_world preserves the simulation clock, unlike /reset_simulation.
        from std_srvs.srv import Empty
        from gazebo_msgs.srv import GetWorldProperties
        before_reset=joint(rospy)
        clock_before_reset=call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time
        clear(rospy)
        call(rospy,'/gazebo/reset_world',Empty)
        reset_receipt_wall=time.monotonic_ns()
        wait_for(lambda:any(row['receipt_monotonic_ns']>reset_receipt_wall and abs(row['x_m'])<.01
                            for row in received.snapshot()[1]),seconds=6,
                 description='fresh state messages confirming the owned model returned to its initial joint position')
        before=joint(rospy)
        clock_after_reset=call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time
        require(abs(before['position_m'])<.01 and abs(before['velocity_m_s'])<.01,
                'Actual reset did not restore the owned joint position and velocity')
        require(clock_after_reset>=clock_before_reset,'Model-only reset unexpectedly rewound simulation time')
        result['checks']['acceptance_reset']={'service':'/gazebo/reset_world','before':before_reset,
            'after':before,'native_world_time_before_s':clock_before_reset,
            'native_world_time_after_s':clock_after_reset,
            'scope':'Owned isolated course world models and velocities reset once; simulation clock and personal files preserved'}
        response=call(rospy,ns+'/pulse',Trigger)
        scheduled=json.loads(response.message)
        direction=1 if scheduled['force_N']>0 else -1
        pulse_start=scheduled['before_joint']['position_m'];pulse_wall=time.monotonic_ns()
        wait_for(lambda:any(row['receipt_monotonic_ns']>pulse_wall and direction*(row['x_m']-pulse_start)>.025
                            for row in received.snapshot()[1]),seconds=8,
                 description='new TCPROS state messages showing the scheduled physical displacement')
        after=joint(rospy)
        require(direction*(after['position_m']-pulse_start)>.025,'Independent GetJointProperties read did not confirm motion')
        statuses,samples=received.snapshot()
        require(all(math.isfinite(row['x_m']) and math.isfinite(row['stamp_s']) for row in samples),'Nonfinite received observation')
        latest=samples[-1]
        latest_age_s=(time.monotonic_ns()-latest['receipt_monotonic_ns'])/1e9
        coordinate_error_m=abs(latest['x_m']-after['position_m'])
        require(latest_age_s<.25 and coordinate_error_m<.05,
                'Fresh topic position disagrees with the independent joint readback (asynchronous tolerance .05 m)')
        result['checks'].update(live_samples=len(samples),sim_time_advances=True,
            before_joint=before,pulse=scheduled,after_joint=after,
            signed_native_displacement_m=direction*(after['position_m']-pulse_start),
            minimum_native_displacement_m=.025,
            message_joint_consistency={'topic_x_m':latest['x_m'],'native_x_m':after['position_m'],
                'absolute_difference_m':coordinate_error_m,'allowed_async_difference_m':.05,'topic_receipt_age_s':latest_age_s},
            received_trace=[{'seq':r['seq'],'stamp_s':r['stamp_s'],'x_m':r['x_m'],'receipt_monotonic_ns':r['receipt_monotonic_ns']} for r in samples],
            stamp_semantics='Observation-node /clock readback after its joint query; not native acquisition time, no atomic sample/clock snapshot claim',
            student_exercise='student.py is untouched and not automatically graded by the reference instrument')
        result['passed']=True
        return result
    except BaseException as error:
        result['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        for subscription in subscriptions:subscription.unregister()
        result['elapsed_wall_s']=(time.monotonic_ns()-created)/1e9
        save_verification(result)


def main():
    import rospy
    parser=argparse.ArgumentParser(description='A42: real ROS graph, typed state and one finite physical pulse')
    parser.add_argument('mode',choices=('daemon','ready','verify','pulse','state'),nargs='?',default='verify')
    # Only argparse gets the filtered argv. rospy still sees roslaunch's __name,
    # __log, __ns and user remaps in the original process command line.
    mode=parser.parse_args(rospy.myargv()[1:]).mode
    if mode=='daemon':daemon();return
    if mode=='ready':result=ready()
    elif mode=='verify':result=verify()
    else:
        rospy,ns=initialize('operator')
        if mode=='state':result=joint(rospy)
        else:
            from std_srvs.srv import Trigger
            response=call(rospy,ns+'/pulse',Trigger)
            result={'scheduled':True,'pulse':json.loads(response.message)}
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()
