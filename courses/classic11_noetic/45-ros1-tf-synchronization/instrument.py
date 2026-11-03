#!/usr/bin/env python3
"""A45 only: actual link/TF geometry and explicitly controlled timestamp matching."""
import argparse
from collections import deque
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import threading
import time
import uuid
import xml.etree.ElementTree as ET


JOINT='workshop::slide'
PACKAGE='gazeboteach_workshop45'


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
    if not rospy.core.is_initialized():rospy.init_node('workshop_45_'+role,anonymous=True)
    return rospy,rospy.get_namespace().rstrip('/') or '/workshop45'


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
    require(math.isfinite(value) and abs(value)<=5,'A45 effective force must be finite within [-5,5] N')
    require(math.isfinite(duration) and 0<duration<=.3,'A45 effort duration must be within (0,.3] simulation seconds')
    return call(rospy,'/gazebo/apply_joint_effort',ApplyJointEffort,JOINT,value,
                (rospy.Time.from_sec(start_time) if start_time is not None else rospy.Time(0)),rospy.Duration(duration),_deadline=_deadline)


def clear(rospy,_deadline=None):
    from gazebo_msgs.srv import JointRequest
    # Noetic clearJointForces compares Joint::GetName(), unlike effort/readback
    # lookup which accepts a scoped name. This isolated world has one "slide".
    return call(rospy,'/gazebo/clear_joint_forces',JointRequest,JOINT.rsplit('::',1)[-1],_deadline=_deadline)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


SYNC_PERIOD=.06


def quaternion(rpy):
    r,p,y=(value/2 for value in rpy)
    return [math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
        math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
        math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y),
        math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)]


def rotation(q):
    norm=math.sqrt(sum(v*v for v in q));require(norm>1e-12,'Zero quaternion is not a rotation')
    x,y,z,w=[v/norm for v in q]
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def transform_point(R,t,p):return [sum(R[i][j]*p[j] for j in range(3))+t[i] for i in range(3)]
def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))


def angle_between(a,b):
    dot=abs(sum(x*y for x,y in zip(a,b)))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    return 2*math.acos(min(1,max(0,dot)))


def read_mount(path='world.sdf'):
    document=ET.parse(str(path));model=document.find("world/model[@name='workshop']")
    require(model is not None,'Keep the real workshop model identity')
    def pose(element):
        values=[float(v) for v in ((element.text if element is not None else None) or '0 0 0 0 0 0').split()]
        require(len(values)==6 and all(math.isfinite(v) for v in values),'SDF pose needs six finite numbers')
        return values
    base=pose(model.find('pose'));body=pose(model.find("link[@name='body']/pose"))
    require(all(abs(v)<1e-12 for v in base[3:]+body[3:]),'This one-axis rig keeps the rail/body base axes aligned with world')
    rail=[base[i]+body[i] for i in range(3)]+[0,0,0]
    probe=model.find("link[@name='probe']/pose")
    require(probe is not None and probe.get('relative_to')=='body','Probe pose must explicitly be relative_to body')
    mount=pose(probe)
    require(model.find("joint[@name='probe_mount']/parent").text=='body' and
            model.find("joint[@name='probe_mount']/child").text=='probe','Probe must have its actual fixed mounting joint')
    return {'rail_pose':rail,'probe_pose_relative_body':mount,'probe_link':'workshop::probe',
            'local_point':[.2,0,0],'world_sha256':sha(path)}


def configuration(rospy):
    values={}
    for key,default,low,high in (('delay_sim_s',.03,0,.12),('offset_sim_s',.012,0,.03),('slop_s',.02,.001,.02)):
        value=bounded(lambda k=key,d=default:rospy.get_param('~'+k,d),6,'XMLRPC get_param ~'+key)
        require(isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and low<=value<=high,
                key+' outside the finite teaching range '+str((low,high)))
        values[key]=float(value)
    values['sync_period_sim_s']=SYNC_PERIOD
    # Neighbor transactions cannot accidentally satisfy ATS: .06-.03 > .02.
    require(SYNC_PERIOD-values['offset_sim_s']>values['slop_s'],'Configured stamps could alias a neighboring transaction')
    return values


class Cadence:
    """Sim-time cadence based on completed observation stamp; never catch up."""
    def __init__(self):self.next_due_ns=None;self.last_ns=None
    def due(self,stamp_ns):
        if stamp_ns<=0:return False
        if self.last_ns is not None and stamp_ns<self.last_ns:self.next_due_ns=None
        if self.last_ns==stamp_ns:return False
        self.last_ns=stamp_ns
        if self.next_due_ns is not None and stamp_ns<self.next_due_ns:return False
        self.next_due_ns=stamp_ns+round(SYNC_PERIOD*1e9)
        return True


def tf_message(rospy,parent,child,translation,rotation_xyzw,stamp):
    from geometry_msgs.msg import TransformStamped
    message=TransformStamped();message.header.stamp=stamp;message.header.frame_id=parent;message.child_frame_id=child
    message.transform.translation.x,message.transform.translation.y,message.transform.translation.z=translation
    message.transform.rotation.x,message.transform.rotation.y,message.transform.rotation.z,message.transform.rotation.w=rotation_xyzw
    return message


def world_time(rospy):
    from gazebo_msgs.srv import GetWorldProperties
    return float(call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time)


def final_clear(rospy,previous_failure=None):
    if isinstance(previous_failure,WallDeadline) and 'clear_joint_forces' in previous_failure.operation:
        raise RuntimeError('Previous clear outcome is uncertain; automatic cleanup replay is deliberately skipped')
    return clear(rospy)


def native_probe(rospy):
    from gazebo_msgs.srv import GetLinkState
    before=world_time(rospy);value=call(rospy,'/gazebo/get_link_state',GetLinkState,'workshop::probe','world')
    after=world_time(rospy);pose=value.link_state.pose
    return {'position':[pose.position.x,pose.position.y,pose.position.z],
        'orientation_xyzw':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],
        'native_world_time_bracket_s':[before,after],'source':'GetLinkState workshop::probe relative to world; no native acquisition stamp'}


def daemon():
    import tf2_ros
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,TriggerResponse
    rospy,ns=initialize('instrument');config=configuration(rospy);mount=read_mount();pulse_lock=threading.Lock();uncertain=threading.Event()
    dynamic=tf2_ros.TransformBroadcaster();static=tf2_ros.StaticTransformBroadcaster()
    now=rospy.Time.now()
    # /tf_static latch retains the last TFMessage; publish both edges together.
    static.sendTransform([tf_message(rospy,'world','rail',mount['rail_pose'][:3],[0,0,0,1],now),
        tf_message(rospy,'cart','probe',mount['probe_pose_relative_body'][:3],quaternion(mount['probe_pose_relative_body'][3:]),now)])
    pose_pub=rospy.Publisher(ns+'/pose',PoseStamped,queue_size=30)
    companion_pub=rospy.Publisher(ns+'/companion',TwistStamped,queue_size=30)
    shadow_pub=rospy.Publisher(ns+'/companion_offset',TwistStamped,queue_size=30)
    trace_pub=rospy.Publisher(ns+'/publication_trace',String,queue_size=30)
    status_pub=rospy.Publisher(ns+'/ready',String,queue_size=1,latch=True)
    def pulse(request):
        if not pulse_lock.acquire(blocking=False):return TriggerResponse(False,'Another pulse is being scheduled')
        try:
            observation=joint(rospy);direction=1 if observation['position_m']<=0 else -1
            clear(rospy);effort(rospy,2*direction,.2)
            return TriggerResponse(True,json.dumps({'scheduled_once':True,'force_N':2*direction,'duration_sim_s':.2,'before_joint':observation}))
        except WallDeadline as error:
            uncertain.set();return TriggerResponse(False,type(error).__name__+': '+str(error))
        except Exception as error:return TriggerResponse(False,type(error).__name__+': '+str(error))
        finally:pulse_lock.release()
    pulse_service=rospy.Service(ns+'/pulse',Trigger,pulse)
    first=joint(rospy)
    status={'node':rospy.get_name(),'namespace':ns,'configuration':config,'mount':mount,
        'instrument_sha256':sha(__file__),'session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),
        'first_actual_joint':first,'stamp_origin':'One observation-node clock read after one real joint query, shared by two teaching channels; not independent native sensor stamps',
        'shadow_stamp_origin':'Deliberate added offset_sim_s, not a physical sampling timestamp or clock correction'}
    status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
    cadence=Cadence();pending=deque();last_tf_ns=None;sequence=0
    try:
        while not rospy.is_shutdown():
            require(not uncertain.is_set(),'A45 pulse RPC outcome uncertain; stop without automatic replay')
            observation=joint(rospy);stamp=rospy.Time.now();stamp_ns=stamp.to_nsec()
            # A new use_sim_time process may not have received its first
            # /clock yet. Zero is not evidence of a real acquisition time.
            if stamp_ns<=0:time.sleep(.02);continue
            if last_tf_ns is not None and stamp_ns<last_tf_ns:pending.clear();cadence=Cadence()
            if stamp_ns!=last_tf_ns:
                dynamic.sendTransform(tf_message(rospy,'rail','cart',[observation['position_m'],0,0],[0,0,0,1],stamp))
                last_tf_ns=stamp_ns
            if cadence.due(stamp_ns):
                pose=PoseStamped();pose.header.seq=sequence;pose.header.stamp=stamp;pose.header.frame_id='rail'
                pose.pose.position.x=observation['position_m'];pose.pose.orientation.w=1
                companion=TwistStamped();companion.header=copy.deepcopy(pose.header);companion.twist.linear.x=observation['velocity_m_s']
                shadow=copy.deepcopy(companion);shadow.header.stamp=stamp+rospy.Duration.from_sec(config['offset_sim_s'])
                pose_publish_ns=rospy.Time.now().to_nsec();pose_pub.publish(pose)
                pending.append((stamp_ns+round(config['delay_sim_s']*1e9),companion,shadow,
                                sequence,stamp_ns,pose_publish_ns,pose.header.seq,pose.header.stamp.to_nsec()));sequence+=1
                require(len(pending)<200,'Delayed publication queue overflow; no silent data fabrication')
            while pending and pending[0][0]<=stamp_ns:
                _,companion,shadow,source_seq,query_stamp_ns,pose_publish_ns,pose_publisher_seq,pose_stamp_ns=pending.popleft()
                companion_publish_ns=rospy.Time.now().to_nsec()
                companion_pub.publish(companion)
                shadow_publish_ns=rospy.Time.now().to_nsec();shadow_pub.publish(shadow)
                # rospy serializes Header.seq using each Publisher's own
                # counter; it is NOT the source observation identity.
                trace_pub.publish(String(data=json.dumps({'source_observation_seq':source_seq,'query_stamp_ns':query_stamp_ns,
                    'pose_publisher_seq':pose_publisher_seq,'pose_stamp_ns':pose_stamp_ns,
                    'companion_publisher_seq':companion.header.seq,'companion_stamp_ns':companion.header.stamp.to_nsec(),
                    'shadow_publisher_seq':shadow.header.seq,'shadow_stamp_ns':shadow.header.stamp.to_nsec(),
                    'pose_publish_call_sim_ns':pose_publish_ns,'companion_publish_call_sim_ns':companion_publish_ns,
                    'shadow_publish_call_sim_ns':shadow_publish_ns,
                    'requested_delay_sim_s':config['delay_sim_s'],
                    'scope':'Source-node simulator time at Publisher.publish call sites; not wire transmission or subscriber receipt time'},allow_nan=False)))
            time.sleep(.02)
    finally:
        if pulse_lock.acquire(blocking=False):
            try:
                if not uncertain.is_set():clear(rospy)
            except Exception as error:rospy.logwarn('A45 force cleanup failed: %s',error)
            finally:pulse_lock.release()
        pulse_service.shutdown('A45 instrument ended')
        for publisher in (pose_pub,companion_pub,shadow_pub,trace_pub,status_pub):publisher.unregister()


def lookup(rospy,buffer,parent='world',child='probe',seconds=4,min_stamp=None):
    """Only immediate TF reads are retried; the wall clock owns the wait budget."""
    deadline=time.monotonic()+seconds;last_error='No transform yet'
    while time.monotonic()<deadline:
        try:
            value=buffer.lookup_transform(parent,child,rospy.Time(0),rospy.Duration(0))
            if min_stamp is None or value.header.stamp.to_sec()>=min_stamp:return value
            last_error='Latest common transform time predates the requested observation'
        except Exception as error:last_error=str(error)
        time.sleep(.01)
    raise WallDeadline('Immediate TF lookup: '+last_error,False)


def tf_dict(value):
    t,q=value.transform.translation,value.transform.rotation
    return {'position':[t.x,t.y,t.z],'orientation_xyzw':[q.x,q.y,q.z,q.w],
        'stamp_s':value.header.stamp.to_sec(),'parent':value.header.frame_id,'child':value.child_frame_id,
        'time_semantics':'lookup Time(0) returns the latest common time of this chain, not necessarily the current simulator time'}


def compare_tf(rospy,buffer,min_stamp=None):
    transform=tf_dict(lookup(rospy,buffer,min_stamp=min_stamp));native=native_probe(rospy)
    position_error=distance(transform['position'],native['position'])
    orientation_error=angle_between(transform['orientation_xyzw'],native['orientation_xyzw'])
    point=[.2,0,0]
    transformed=transform_point(rotation(transform['orientation_xyzw']),transform['position'],point)
    native_point=transform_point(rotation(native['orientation_xyzw']),native['position'],point)
    require(position_error<.04 and orientation_error<.03 and distance(transformed,native_point)<.04,
            'Actual link pose/rotated local point differs from the composed TF chain')
    return {'tf':transform,'independent_native_link':native,'position_error_m':position_error,
        'orientation_error_rad':orientation_error,'local_point':point,'tf_transformed_point':transformed,
        'native_transformed_point':native_point,'point_error_m':distance(transformed,native_point),
        'comparison_scope':'Asynchronous latest-common-time TF versus independent GetLinkState bracket, with explicit tolerances; no atomic snapshot claim'}


class SyncProbe:
    def __init__(self,rospy,ns,config):
        import message_filters
        from geometry_msgs.msg import PoseStamped,TwistStamped
        from std_msgs.msg import String
        self.lock=threading.Lock();self.raw={'pose':[],'companion':[],'companion_offset':[]}
        self.traces=[]
        self.pairs={'baseline_exact':[],'offset_exact':[],'offset_approx':[]}
        self.subscriptions=[message_filters.Subscriber(ns+'/'+name,kind,queue_size=100)
            for name,kind in (('pose',PoseStamped),('companion',TwistStamped),('companion_offset',TwistStamped))]
        for name,subscription in zip(self.raw,self.subscriptions):subscription.registerCallback(lambda message,n=name:self.record_raw(n,message))
        self.filters=[message_filters.TimeSynchronizer(self.subscriptions[:2],100),
            message_filters.TimeSynchronizer([self.subscriptions[0],self.subscriptions[2]],100),
            message_filters.ApproximateTimeSynchronizer([self.subscriptions[0],self.subscriptions[2]],100,config['slop_s'])]
        for name,filter_ in zip(self.pairs,self.filters):filter_.registerCallback(lambda a,b,n=name:self.record_pair(n,a,b))
        self.trace_subscription=rospy.Subscriber(ns+'/publication_trace',String,self.record_trace,queue_size=100)
    def record_raw(self,name,message):
        row={'seq':message.header.seq,'stamp_ns':message.header.stamp.to_nsec(),
            'publisher_seq':message.header.seq,
            'receipt_monotonic_ns':time.monotonic_ns(),'frame':message.header.frame_id}
        if name=='pose':row['actual_joint_position_m']=message.pose.position.x
        else:row['actual_joint_velocity_m_s']=message.twist.linear.x
        with self.lock:self.raw[name].append(row)
    def record_pair(self,name,a,b):
        row={'pose_seq':a.header.seq,'companion_seq':b.header.seq,'pose_stamp_ns':a.header.stamp.to_nsec(),
            'companion_stamp_ns':b.header.stamp.to_nsec(),'stamp_delta_s':abs(a.header.stamp.to_nsec()-b.header.stamp.to_nsec())/1e9,
            'callback_receipt_monotonic_ns':time.monotonic_ns()}
        with self.lock:self.pairs[name].append(row)
    def snapshot(self):
        with self.lock:return ({key:list(value) for key,value in self.raw.items()},{key:list(value) for key,value in self.pairs.items()})
    def record_trace(self,message):
        row=json.loads(message.data)
        with self.lock:self.traces.append(row)
    def trace_snapshot(self):
        with self.lock:return list(self.traces)
    def close(self):
        for subscription in self.subscriptions:subscription.sub.unregister()
        self.trace_subscription.unregister()


def annotate_sources(raw,pairs,traces):
    """Map actual wire headers to an independent observation identity.

    rospy overwrites Header.seq with per-Publisher counters, which can differ
    across streams after different connection times. Preserve those counters;
    never rewrite them merely to make a pairing look like the same transaction.
    """
    packets={}
    for row in traces:
        source=row['source_observation_seq']
        for kind,prefix in (('pose','pose'),('companion','companion'),('companion_offset','shadow')):
            key=(kind,row[prefix+'_stamp_ns'],row[prefix+'_publisher_seq'])
            require(key not in packets or packets[key]==source,'One published packet maps to multiple source observations')
            packets[key]=source
    for kind,rows in raw.items():
        for row in rows:row['source_observation_seq']=packets.get((kind,row['stamp_ns'],row['seq']))
    counts={}
    for kind,rows in pairs.items():
        companion_kind='companion' if kind=='baseline_exact' else 'companion_offset'
        mapped=0
        for row in rows:
            a=packets.get(('pose',row['pose_stamp_ns'],row['pose_seq']))
            b=packets.get((companion_kind,row['companion_stamp_ns'],row['companion_seq']))
            row['pose_source_observation_seq']=a;row['companion_source_observation_seq']=b
            if a is None or b is None:continue
            require(a==b,'Matcher borrowed a neighboring source observation transaction')
            mapped+=1
        counts[kind]={'source_verified':mapped,'unmapped_capture_boundary_pairs':len(rows)-mapped}
    return counts


def delivery_metrics(raw,traces,config):
    """Keep simulator publication evidence and wall reception evidence separate."""
    joined=[];lags=[];delay_ns=round(config['delay_sim_s']*1e9)
    pose_packets={(row['seq'],row['stamp_ns']):row for row in raw['pose']}
    companion_packets={(row['seq'],row['stamp_ns']):row for row in raw['companion']}
    for row in traces:
        pose=pose_packets.get((row['pose_publisher_seq'],row['pose_stamp_ns']))
        companion=companion_packets.get((row['companion_publisher_seq'],row['companion_stamp_ns']))
        if pose is None or companion is None:continue
        require(row['query_stamp_ns']==pose['stamp_ns']==companion['stamp_ns'],
                'Publication trace does not identify the actual same-stamp observation pair')
        require(row['pose_publish_call_sim_ns']>=row['query_stamp_ns'],
                'Pose publication call predates its observed source stamp')
        actual_delay_ns=row['companion_publish_call_sim_ns']-row['query_stamp_ns']
        require(actual_delay_ns>=delay_ns,'Configured simulated delay was not actually observed at the companion publish call')
        require(row['companion_publish_call_sim_ns']>=row['pose_publish_call_sim_ns'],
                'Companion publish call preceded its pose channel')
        lags.append((companion['receipt_monotonic_ns']-pose['receipt_monotonic_ns'])/1e9)
        joined.append({**row,'actual_source_stamp_to_companion_publish_sim_s':actual_delay_ns/1e9})
    require(len(joined)>=8,'Not enough actual same-sequence publication trace and receipt evidence')
    median=statistics.median(lags)
    if delay_ns>0:require(median>0,'Positive delayed publication did not produce a positive median companion wall-delivery lag')
    return {'source_publication_traces':joined,'same_sequence_receive_wall_lags_s':lags,
        'median_receive_wall_lag_s':median,
        'scope':'Simulator-time publish-call delay and monotonic wall receipt lag are separate measurements; neither is a native independent-sensor sampling offset'}


def status_subscription(rospy,ns):
    from std_msgs.msg import String
    rows=[]
    subscription=rospy.Subscriber(ns+'/ready',String,lambda message:rows.append(json.loads(message.data)),queue_size=1)
    return rows,subscription


def ready():
    rospy,ns=initialize('ready');rows,subscription=status_subscription(rospy,ns)
    try:wait_for(lambda:rows,8,'A45 latched instrument status');return {'ready':True,'status':rows[-1]}
    finally:subscription.unregister()


def reset_owned(rospy):
    from std_srvs.srv import Empty
    before=world_time(rospy);clear(rospy);call(rospy,'/gazebo/reset_world',Empty)
    actual=joint(rospy);after=world_time(rospy)
    require(abs(actual['position_m'])<.01 and abs(actual['velocity_m_s'])<.01,'Owned joint did not reset')
    require(after>=before,'Model reset rewound simulation time')
    return {'before_native_world_s':before,'after_native_world_s':after,'actual_joint':actual}


def write_report(result):
    directory=Path('build');directory.mkdir(exist_ok=True);temporary=directory/('.verification-'+uuid.uuid4().hex+'.json')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(str(temporary),str(directory/'verification.json'))


def verify():
    import tf2_ros
    from std_srvs.srv import Trigger
    rospy,ns=initialize('verify');rows,subscription=status_subscription(rospy,ns)
    probe=None;listener=None;failure=None;started=time.monotonic()
    result={'passed':False,'assignment_pass':False,'lesson':45,'namespace':ns,
        'trusted_verifier_sha256':sha(__file__),'checks':{}}
    try:
        wait_for(lambda:rows,8,'A45 source/configuration identity');identity=rows[-1]
        require(identity['instrument_sha256']==sha('instrument.py'),'Personal instrument changed; restart the experiment')
        require(identity['mount']['world_sha256']==sha('world.sdf'),'Personal SDF changed since load; restart the experiment')
        require(identity['session_token']==os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'A45 status belongs to another session')
        config=identity['configuration'];result['configuration']=config;result['mount']=identity['mount']
        buffer=tf2_ros.Buffer(cache_time=rospy.Duration(5),debug=False);listener=tf2_ros.TransformListener(buffer)
        probe=SyncProbe(rospy,ns,config)
        result['checks']['reset']=reset_owned(rospy)
        before=compare_tf(rospy,buffer,min_stamp=result['checks']['reset']['after_native_world_s'])
        response=call(rospy,ns+'/pulse',Trigger);scheduled=json.loads(response.message)
        wait_for(lambda:any(abs(row['actual_joint_position_m'])>.035 for row in probe.snapshot()[0]['pose']),5,
                 'actual moving joint observations after the finite pulse')
        actual_after=joint(rospy);require(abs(actual_after['position_m'])>.03,'Native joint did not move')
        after=compare_tf(rospy,buffer,min_stamp=scheduled['before_joint']['receipt_clock_after_s'])
        require(abs(after['tf']['position'][0]-before['tf']['position'][0])>.025,'Composed TF did not follow the actual slider motion')
        result['checks']['physical_tf_motion']={'before':before,'after':after,'pulse':scheduled,'independent_joint_after':actual_after}
        # Capture enough actual streams for both a positive and a deliberate
        # negative matcher result. All waits inspect received data only.
        wait_for(lambda:len(probe.snapshot()[1]['baseline_exact'])>=10 and len(probe.snapshot()[0]['companion_offset'])>=10
                 and len(probe.trace_snapshot())>=10,
                 8,'ten baseline pairs and delayed shadow observations')
        raw,pairs=probe.snapshot()
        offset_ns=round(config['offset_sim_s']*1e9);slop_ns=round(config['slop_s']*1e9)
        expected_exact=offset_ns==0;expected_approx=offset_ns<slop_ns
        source_counts=annotate_sources(raw,pairs,probe.trace_snapshot())
        require(source_counts['baseline_exact']['source_verified']>=8,'Insufficient baseline pairs with explicit source-observation identity')
        require(all(row['stamp_delta_s']==0 for row in pairs['baseline_exact']),'Baseline Exact matcher paired different stamps')
        if expected_exact:require(len(pairs['offset_exact'])>=8,'Zero-offset Exact should pair')
        else:require(not pairs['offset_exact'],'Nonzero injected offset unexpectedly matched Exact stamps')
        if expected_approx:
            require(len(pairs['offset_approx'])>=8,'Injected offset below strict slop should pair approximately')
            require(source_counts['offset_approx']['source_verified']>=8,'Insufficient ATS pairs with explicit common source identity')
            require(all(row['stamp_delta_s']<config['slop_s'] for row in pairs['offset_approx']),'ATS pair violates strict slop')
        else:require(not pairs['offset_approx'],'Offset at/above strict slop should be an expected mismatch')
        pose_stamps=[row['stamp_ns'] for row in raw['pose']]
        require(all(b-a>=round(SYNC_PERIOD*1e9)-1 for a,b in zip(pose_stamps,pose_stamps[1:])),
                'Actual observation stamps violate the anti-alias cadence')
        pose_by_seq={row['source_observation_seq']:row for row in raw['pose'] if row['source_observation_seq'] is not None}
        baseline_by_seq={row['source_observation_seq']:row for row in raw['companion'] if row['source_observation_seq'] is not None}
        deltas=[];lags=[]
        for shadow in raw['companion_offset']:
            source=shadow['source_observation_seq']
            original=pose_by_seq.get(source);baseline=baseline_by_seq.get(source)
            if original:
                require(shadow['stamp_ns']-original['stamp_ns']==offset_ns,'Raw shadow timestamp perturbation differs from configuration')
                deltas.append((shadow['stamp_ns']-original['stamp_ns'])/1e9)
            if original and baseline:
                require(baseline['stamp_ns']==original['stamp_ns'],'Publication delay changed the original companion stamp')
                lags.append((baseline['receipt_monotonic_ns']-original['receipt_monotonic_ns'])/1e9)
        require(len(deltas)>=8 and len(lags)>=8,'Not enough raw same-transaction evidence')
        publication_evidence=delivery_metrics(raw,probe.trace_snapshot(),config)
        result['match_statistics']={key:len(value) for key,value in pairs.items()}
        result['match_statistics'].update(expected_offset_exact=expected_exact,expected_offset_approx=expected_approx,
            offset_approx_outcome='expected_match' if expected_approx else 'expected_mismatch',
            same_transaction_seq_verified=True,
            same_transaction_identity='Independent source_observation_seq mapped through (topic kind, raw stamp, per-Publisher Header.seq); raw Header.seq need not agree across topics',
            source_identity_pair_counts=source_counts,strict_slop=True)
        result['checks']['timestamp_matching']={'raw':raw,'pairs':pairs,'injected_stamp_deltas_s':deltas,
            'companion_receive_wall_lags_s':lags,'sample_origin':identity['stamp_origin'],
            'shadow_origin':identity['shadow_stamp_origin'],
            'publication_delay_evidence':publication_evidence,
            'delay_note':'Receive wall lag includes the controlled simulated-time publication delay and transport/callback scheduling; it is not an independent sensor sampling offset'}
        result['passed']=True;return result
    except BaseException as error:failure=error;result['error']=type(error).__name__+': '+str(error);raise
    finally:
        if probe:probe.close()
        if listener:listener.unregister()
        subscription.unregister()
        try:
            final_clear(rospy,failure)
        except Exception as error:result['force_cleanup_error']=str(error);result['passed']=False
        result['elapsed_wall_s']=time.monotonic()-started;write_report(result)
        if result.get('force_cleanup_error'):raise RuntimeError('A45 cleanup failed; verification report retained')


def check_student():
    path=Path.cwd()/'student.py';spec=importlib.util.spec_from_file_location('a45_student_'+uuid.uuid4().hex,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    R=rotation(quaternion([0,0,math.pi/2]));t=[.1,.15,.75];point=[.2,0,0]
    require(distance(module.transform_point(R,t,point),[.1,.35,.75])<1e-9,'Personal point transform omitted or mis-signed R*p+t')
    require(abs(module.stamp_delta(1.,1.012)-.012)<1e-12 and abs(module.stamp_delta(1.012,1.)-.012)<1e-12,
            'Personal stamp difference must be absolute seconds')
    return {'algebra_checks_passed':True,'assignment_pass':False,'student_sha256':sha(path),
        'scope':'Independent local formula exercise; reference physics/synchronizer acceptance does not fill or grade student.py'}


def main():
    import rospy
    parser=argparse.ArgumentParser(description='A45 actual link/TF geometry and deliberately controlled timestamp matching')
    parser.add_argument('mode',choices=('daemon','ready','state','pulse','verify','student-check'),default='verify',nargs='?')
    mode=parser.parse_args(rospy.myargv()[1:]).mode
    if mode=='daemon':daemon();return
    if mode=='ready':result=ready()
    elif mode=='verify':result=verify()
    elif mode=='student-check':result=check_student()
    else:
        rospy,ns=initialize('operator')
        if mode=='pulse':
            from std_srvs.srv import Trigger
            result=json.loads(call(rospy,ns+'/pulse',Trigger).message)
        else:
            import tf2_ros
            rows,subscription=status_subscription(rospy,ns);buffer=tf2_ros.Buffer(cache_time=rospy.Duration(5),debug=False)
            listener=tf2_ros.TransformListener(buffer)
            try:wait_for(lambda:rows,5,'A45 configuration');result={'status':rows[-1],'geometry':compare_tf(rospy,buffer)}
            finally:listener.unregister();subscription.unregister()
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()


