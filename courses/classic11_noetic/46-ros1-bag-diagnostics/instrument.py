#!/usr/bin/env python3
"""A46 only: true rosbag CLI record/play, isolated recorded clock and stale-data diagnosis."""
import argparse
from collections import deque
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid


JOINT='workshop::slide'
PACKAGE='gazeboteach_workshop46'


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
    if not rospy.core.is_initialized():rospy.init_node('workshop_46_'+role,anonymous=True)
    return rospy,rospy.get_namespace().rstrip('/') or '/workshop46'


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
    require(math.isfinite(value) and abs(value)<=5,'A46 effective force must be finite within [-5,5] N')
    require(math.isfinite(duration) and 0<duration<=.3,'A46 effort duration must be within (0,.3] simulation seconds')
    return call(rospy,'/gazebo/apply_joint_effort',ApplyJointEffort,JOINT,value,
                (rospy.Time.from_sec(start_time) if start_time is not None else rospy.Time(0)),rospy.Duration(duration),_deadline=_deadline)


def clear(rospy,_deadline=None):
    from gazebo_msgs.srv import JointRequest
    # Noetic clearJointForces compares Joint::GetName(), unlike effort/readback
    # lookup which accepts a scoped name. This isolated world has one "slide".
    return call(rospy,'/gazebo/clear_joint_forces',JointRequest,JOINT.rsplit('::',1)[-1],_deadline=_deadline)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def get_param(rospy,name,default=None):return bounded(lambda:rospy.get_param(name,default),6,'XMLRPC get_param '+name)
def set_param(rospy,name,value):return bounded(lambda:rospy.set_param(name,value),6,'XMLRPC set_param '+name)


def world_time(rospy):
    from gazebo_msgs.srv import GetWorldProperties
    return float(call(rospy,'/gazebo/get_world_properties',GetWorldProperties).sim_time)


def final_clear(rospy,failure=None):
    if isinstance(failure,WallDeadline) and 'clear_joint_forces' in failure.operation:
        raise RuntimeError('Unknown clear outcome; automatic cleanup replay deliberately skipped')
    return clear(rospy)


def health_level(age,warn,error):
    require(math.isfinite(age) and age>=0 and 0<warn<error,'Invalid freshness thresholds or age')
    return 2 if age>=error else 1 if age>=warn else 0


def observer():
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,TriggerResponse
    rospy,ns=initialize('observer');uncertain=threading.Event();lock=threading.Lock()
    publisher=rospy.Publisher(ns+'/state',JointState,queue_size=30)
    status=rospy.Publisher(ns+'/observer_ready',String,queue_size=1,latch=True)
    emit_name=rospy.resolve_name('~emit_state')
    def pulse(request):
        if not lock.acquire(blocking=False):return TriggerResponse(False,'Another finite pulse is being scheduled')
        try:
            before=joint(rospy);direction=1 if before['position_m']<=0 else -1
            clear(rospy);effort(rospy,2*direction,.2)
            return TriggerResponse(True,json.dumps({'scheduled_once':True,'force_N':2*direction,'duration_sim_s':.2,'before_joint':before}))
        except WallDeadline as error:uncertain.set();return TriggerResponse(False,str(error))
        except Exception as error:return TriggerResponse(False,str(error))
        finally:lock.release()
    service=rospy.Service(ns+'/pulse',Trigger,pulse)
    status.publish(String(data=json.dumps({'node':rospy.get_name(),'namespace':ns,'emit_parameter':emit_name,
        'state_topic':ns+'/state','instrument_sha256':sha(__file__),'world_sha256':sha('world.sdf'),
        'session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'initial_actual_joint':joint(rospy)})))
    try:
        while not rospy.is_shutdown():
            require(not uncertain.is_set(),'Observer stops after an uncertain pulse RPC; no automatic replay')
            emit=get_param(rospy,emit_name,True);require(type(emit) is bool,'emit_state must be a YAML/ROS boolean')
            if emit:
                value=joint(rospy);stamp=rospy.Time.now()
                if stamp.to_nsec()>0:
                    message=JointState();message.header.stamp=stamp;message.header.frame_id='workshop_slide_zero'
                    message.name=['slide'];message.position=[value['position_m']];message.velocity=[value['velocity_m_s']]
                    message.effort=[];publisher.publish(message)
            time.sleep(.05)
    finally:
        if lock.acquire(blocking=False):
            try:
                if not uncertain.is_set():clear(rospy)
            except Exception as error:rospy.logwarn('A46 observer force cleanup failed: %s',error)
            finally:lock.release()
        service.shutdown('A46 observer ended');publisher.unregister();status.unregister()


def monitor():
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    rospy,ns=initialize('diagnostic_monitor')
    warn=float(get_param(rospy,'~warn_age_wall_s',.25));error=float(get_param(rospy,'~error_age_wall_s',.7))
    require(math.isfinite(warn) and math.isfinite(error) and 0<warn<error<3,'Invalid startup wall-age thresholds')
    received={'last':None,'count':0,'source_stamp_ns':None};lock=threading.Lock();started=time.monotonic()
    def state(message):
        if message.name!=['slide'] or len(message.position)!=1 or len(message.velocity)!=1:return
        if not all(math.isfinite(value) for value in message.position+message.velocity):return
        with lock:received.update(last=time.monotonic(),count=received['count']+1,source_stamp_ns=message.header.stamp.to_nsec())
    subscription=rospy.Subscriber(ns+'/state',JointState,state,queue_size=100)
    publisher=rospy.Publisher(ns+'/diagnostics',DiagnosticArray,queue_size=30)
    ready_pub=rospy.Publisher(ns+'/monitor_ready',String,queue_size=1,latch=True)
    ready_pub.publish(String(data=json.dumps({'node':rospy.get_name(),'namespace':ns,
        'warn_age_wall_s':warn,'error_age_wall_s':error,'instrument_sha256':sha(__file__),
        'session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'basis':'Monotonic wall time since this subscriber received a valid state message'})))
    try:
        while not rospy.is_shutdown():
            with lock:snapshot=dict(received)
            age=max(0,time.monotonic()-(snapshot['last'] if snapshot['last'] is not None else started))
            level=health_level(age,warn,error)
            if snapshot['last'] is None:level=max(1,level)
            status=DiagnosticStatus();status.name='GazeboTeach actual state-stream freshness';status.hardware_id='simulated-workshop46'
            status.level=level;status.message=('OK: fresh state','WARN: state stream stale','ERROR: state stream stale')[level]
            status.values=[KeyValue('state_topic',ns+'/state'),KeyValue('age_wall_s',str(age)),
                KeyValue('received_count',str(snapshot['count'])),KeyValue('source_header_stamp_ns',str(snapshot['source_stamp_ns'])),
                KeyValue('warn_age_wall_s',str(warn)),KeyValue('error_age_wall_s',str(error))]
            message=DiagnosticArray();message.header.stamp=rospy.Time.now();message.status=[status];publisher.publish(message)
            time.sleep(.05)
    finally:subscription.unregister();publisher.unregister();ready_pub.unregister()


def topic_types(ns):
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray
    from rosgraph_msgs.msg import Clock
    return {ns+'/state':JointState,ns+'/diagnostics':DiagnosticArray,'/clock':Clock}


def payload(message):
    buffer=io.BytesIO();message.serialize(buffer);return hashlib.sha256(buffer.getvalue()).hexdigest()


def stamp(message):
    if hasattr(message,'header'):return {'kind':'message.header.stamp','ns':message.header.stamp.to_nsec()}
    if hasattr(message,'clock'):return {'kind':'recorded Clock.clock value','ns':message.clock.to_nsec()}
    return {'kind':'none','ns':None}


class LiveEvidence:
    def __init__(self,rospy,ns):
        from std_msgs.msg import String
        self.lock=threading.Lock();self.ready={};self.states=[];self.diagnostics=[];self.live_clock=[]
        self.subscriptions=[]
        for role in ('observer','monitor'):
            self.subscriptions.append(rospy.Subscriber(ns+'/'+role+'_ready',String,lambda message,r=role:self.on_ready(r,message),queue_size=1))
        kinds=topic_types(ns)
        self.subscriptions.extend([rospy.Subscriber(ns+'/state',kinds[ns+'/state'],self.state,queue_size=100),
            rospy.Subscriber(ns+'/diagnostics',kinds[ns+'/diagnostics'],self.diag,queue_size=100),
            rospy.Subscriber('/clock',kinds['/clock'],self.clock,queue_size=10000,buff_size=8*1024*1024)])
    def on_ready(self,role,message):
        with self.lock:self.ready[role]=json.loads(message.data)
    def state(self,message):
        if message.position and message.velocity:
            with self.lock:self.states.append({'x_m':message.position[0],'v_m_s':message.velocity[0],
                'source_stamp_ns':message.header.stamp.to_nsec(),'receipt_monotonic_ns':time.monotonic_ns()})
    def diag(self,message):
        for status in message.status:
            with self.lock:self.diagnostics.append({'level':status.level,'message':status.message,
                'values':{value.key:value.value for value in status.values},'source_stamp_ns':message.header.stamp.to_nsec(),
                'receipt_monotonic_ns':time.monotonic_ns()})
    def clock(self,message):
        with self.lock:self.live_clock.append({'clock_ns':message.clock.to_nsec(),'receipt_monotonic_ns':time.monotonic_ns(),
            'callerid':(getattr(message,'_connection_header',{}) or {}).get('callerid')})
    def snapshot(self):
        with self.lock:return dict(self.ready),list(self.states),list(self.diagnostics),list(self.live_clock)
    def close(self):
        for subscription in self.subscriptions:subscription.unregister()


def owned_group_members(pgid,token):
    result=[]
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():continue
        try:
            if directory.stat().st_uid!=os.getuid():continue
            text=(directory/'stat').read_text();fields=text[text.rfind(')')+2:].split()
            if int(fields[2])!=pgid or fields[0]=='Z':continue
            marker=('GAZEBOTEACH_BAG_TOKEN='+token).encode()
            require(marker in (directory/'environ').read_bytes().split(b'\0'),'Refuse to signal a group containing an unowned process')
            result.append({'pid':int(directory.name),'pgid':pgid,'start_ticks':fields[19]})
        except (FileNotFoundError,ProcessLookupError):continue
    return result


class BagProcess:
    """One CLI and its C++ child, both proven by a unique inherited ownership tag."""
    def __init__(self,command,log):
        require(os.name=='posix' and os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'Run inside the owned Linux lesson session')
        self.token=uuid.uuid4().hex;self.stream=Path(log).open('wb');self.command=list(command)
        environment=dict(os.environ);environment['GAZEBOTEACH_BAG_TOKEN']=self.token
        try:self.process=subprocess.Popen(command,cwd=str(Path.cwd()),env=environment,stdout=self.stream,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:self.stream.close();raise
        self.pgid=self.process.pid;self.closed=False;self.report={'command':list(command),'pid':self.process.pid,'pgid':self.pgid,'ownership_token':self.token}
    def members(self):return owned_group_members(self.pgid,self.token)
    def stop(self):
        if self.closed:return self.report
        stages=[]
        try:
            for number,grace in ((signal.SIGINT,5),(signal.SIGTERM,2),(signal.SIGKILL,1)):
                members=self.members()
                if not members:break
                try:os.killpg(self.pgid,number)
                except ProcessLookupError:break
                deadline=time.monotonic()+grace
                while time.monotonic()<deadline and self.members():time.sleep(.04)
                stages.append({'signal':number,'members_before':members,'remaining':self.members()})
            try:self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:pass
            self.report.update(stop_stages=stages,returncode=self.process.poll(),remaining_owned_group=self.members())
            require(not self.report['remaining_owned_group'],'Owned rosbag descendants survived bounded cleanup')
            return self.report
        finally:self.closed=True;self.stream.close()
    def finish(self,seconds):
        self.process.wait(timeout=seconds)
        require(self.process.returncode==0,'Real rosbag CLI exited with code '+str(self.process.returncode))
        return self.stop()


def record_command(path,topics,node):
    return ['rosbag','record','--buffsize','16','--chunksize','128','-O',str(path)]+list(topics)+['__name:='+node]


def playback_command(path,topics,mapping,rate,node):
    require(set(mapping)==set(topics) and '/clock' in mapping,'All recorded topics, especially /clock, must be explicitly remapped')
    require(all(value!='/clock' and value.startswith('/') for value in mapping.values()),'Playback may not publish to live /clock')
    require(all(value not in topics for value in mapping.values()) and len(set(mapping.values()))==len(mapping),
            'Every replay output must be distinct and isolated from the live recorded topics')
    require(isinstance(rate,(int,float)) and not isinstance(rate,bool) and math.isfinite(rate) and .25<=rate<=4,
            'Replay rate must be finite within [.25,4] for bounded teaching runs')
    # --topics greedily consumes trailing non-options. Put bag/remaps BEFORE it.
    # The real recorded Clock messages are played into recorded_clock; no --clock.
    # Noetic Player also advertises a relative clock even when --clock is off.
    # Its own namespace keeps that advertisement separate from the absolute
    # bag /clock remap, so its queue=1 cannot be reused for recorded Clock data.
    internal_namespace=mapping['/clock'].rsplit('/',1)[0]+'/player_internal'
    return ['rosbag','play','--rate',str(rate),'--queue','10000','--delay','.2','--wait-for-subscribers',
        str(path)]+[key+':='+mapping[key] for key in topics]+['__name:='+node,'__ns:='+internal_namespace,'--topics']+list(topics)


def audit_bag(path,topics):
    path=Path(path);require(path.is_file() and not Path(str(path)+'.active').exists(),'A complete .bag is required; .active is not success')
    import rosbag
    records={topic:[] for topic in topics};positions=[]
    with rosbag.Bag(str(path),'r') as bag:
        require(bag.get_message_count()>0,'Real bag is empty')
        for topic,message,bag_time in bag.read_messages(topics=list(topics)):
            row={'sha256':payload(message),'message_stamp':stamp(message),'bag_record_time_ns':bag_time.to_nsec()}
            records[topic].append(row)
            if hasattr(message,'position') and message.position:positions.append(message.position[0])
    require(all(len(rows)>=15 for rows in records.values()),'Not enough actual recorded messages on every selected topic')
    require(positions and max(positions)-min(positions)>.03,'Recorded real slider state did not demonstrate physical motion')
    for topic,rows in records.items():require(all(a['bag_record_time_ns']<=b['bag_record_time_ns'] for a,b in zip(rows,rows[1:])),topic+' bag ordering is invalid')
    return {'records':records,'counts':{topic:len(rows) for topic,rows in records.items()},
        'recorded_position_range_m':[min(positions),max(positions)],'bag_sha256':sha(path),
        'scope':'Real rosbag.Bag read after CLI shutdown; bag record times and message stamps retained separately without assuming one always follows the other'}


def master_publishers(rospy):
    import rosgraph
    state=bounded(rosgraph.Master(rospy.get_name()).getSystemState,3,'Read actual ROS Master publisher graph')
    return dict(state[0])


def await_recorder(rospy,topics,node,seconds=8):
    import rosgraph
    master=rosgraph.Master(rospy.get_name());deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        state=bounded(master.getSystemState,min(2,deadline-time.monotonic()),'Read recorder subscriptions')
        subscribers=dict(state[1])
        if all(any(name.endswith('/'+node) for name in subscribers.get(topic,[])) for topic in topics):return subscribers
        time.sleep(.05)
    raise WallDeadline('Actual rosbag recorder subscription readiness',False)


def reset_owned(rospy):
    from std_srvs.srv import Empty
    clear(rospy);before=world_time(rospy);call(rospy,'/gazebo/reset_world',Empty)
    state=joint(rospy);after=world_time(rospy)
    require(abs(state['position_m'])<.01 and abs(state['velocity_m_s'])<.01,'Owned slider did not reset')
    require(after>=before,'Model reset rewound actual simulation time')
    return {'actual_joint':state,'native_world_before_s':before,'native_world_after_s':after}


def record_motion(rospy,ns,evidence,directory,hold_wall_s=2.6):
    """Real CLI recording entry, including a bounded start and complete cleanup.

    The directory must be new. Paused /clock may make recorder readiness fail;
    that is a bounded failure, never acceptance of an empty/.active bag.
    """
    from std_srvs.srv import Trigger
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    path=directory/'motion.bag';topics=list(topic_types(ns));name='gazeboteach_a46_record_'+uuid.uuid4().hex
    reset=reset_owned(rospy);process=BagProcess(record_command(path,topics,name),directory/'record.log')
    report={'path':str(path),'topics':topics,'reset':reset}
    try:
        report['actual_recorder_subscriptions']=await_recorder(rospy,topics,name)
        report['pulse']=json.loads(call(rospy,ns+'/pulse',Trigger).message)
        # Recording duration is a wall budget. No ROS --duration dependency.
        end=time.monotonic()+hold_wall_s
        while time.monotonic()<end:
            require(process.process.poll() is None,'Recorder exited before the wall-bounded observation window')
            time.sleep(min(.04,max(0,end-time.monotonic())))
    finally:report['process_close']=process.stop()
    require(path.is_file() and not Path(str(path)+'.active').exists(),'Recorder did not close a complete bag')
    report['audit']=audit_bag(path,topics);report['actual_joint_after']=joint(rospy)
    return report


class ReplayEvidence:
    def __init__(self,rospy,topics,mapping):
        self.lock=threading.Lock();self.rows={topic:[] for topic in topics}
        self.subscriptions=[rospy.Subscriber(mapping[topic],kind,
            lambda message,t=topic:self.receive(t,message),queue_size=10000,buff_size=8*1024*1024)
            for topic,kind in topics.items()]
    def receive(self,topic,message):
        row={'sha256':payload(message),'message_stamp':stamp(message),
            'receipt_monotonic_ns':time.monotonic_ns(),
            'callerid':(getattr(message,'_connection_header',{}) or {}).get('callerid')}
        with self.lock:self.rows[topic].append(row)
    def snapshot(self):
        with self.lock:return {key:list(value) for key,value in self.rows.items()}
    def close(self):
        for subscription in self.subscriptions:subscription.unregister()


def replay_once(rospy,ns,bag,expected,rate,evidence):
    kinds=topic_types(ns);topics=list(kinds);token=uuid.uuid4().hex
    prefix=ns+'/replay_'+token
    mapping={ns+'/state':prefix+'/state',ns+'/diagnostics':prefix+'/diagnostics','/clock':prefix+'/recorded_clock'}
    observer=ReplayEvidence(rospy,kinds,mapping)
    name='gazeboteach_a46_play_'+token;command=playback_command(bag,topics,mapping,rate,name)
    process=BagProcess(command,Path(bag).parent/('play-'+str(rate)+'-'+token+'.log'))
    internal_clock=prefix+'/player_internal/clock'
    report={'rate':rate,'topic_remaps':mapping,'command':command,'bag_path':str(Path(bag).resolve()),
        'player_internal_namespace':prefix+'/player_internal','unused_player_clock_topic':internal_clock,
        'recorded_clock_topic':mapping['/clock']}
    try:
        before=world_time(rospy);wall_before=time.monotonic_ns()
        wait_for(lambda:observer.snapshot()[ns+'/state'],8,'first real isolated rosbag replay state')
        publishers=master_publishers(rospy)
        require(not any(name in node for node in publishers.get('/clock',[])),'Bag player advertised on live /clock')
        report['live_clock_publishers_during_play']=publishers.get('/clock',[])
        report['internal_clock_publishers_during_play']=publishers.get(internal_clock,[])
        report['recorded_clock_publishers_during_play']=publishers.get(mapping['/clock'],[])
        state_expected=expected[ns+'/state'];span=(state_expected[-1]['bag_record_time_ns']-state_expected[0]['bag_record_time_ns'])/1e9
        report['process_close']=process.finish(span/rate+8)
        wait_for(lambda:all(len(observer.snapshot()[topic])>=len(expected[topic]) for topic in topics),3,'all recorded replay payloads')
        rows=observer.snapshot()
        for topic in topics:
            require(len(rows[topic])==len(expected[topic]),'Replay message count differs: '+topic)
            require([row['sha256'] for row in rows[topic]]==[row['sha256'] for row in expected[topic]],
                    'Replay changed serialized payload or per-topic order: '+topic)
            require([row['message_stamp'] for row in rows[topic]]==[row['message_stamp'] for row in expected[topic]],
                    'Replay changed the original message stamp: '+topic)
        states=rows[ns+'/state'];wall_span=(states[-1]['receipt_monotonic_ns']-states[0]['receipt_monotonic_ns'])/1e9
        require(abs(wall_span-span/rate)<max(.18,.2*span/rate),'Real replay delivery span differs from requested rate')
        clocks=[row for row in evidence.snapshot()[3] if row['receipt_monotonic_ns']>=wall_before]
        require(clocks and all(a['clock_ns']<=b['clock_ns'] for a,b in zip(clocks,clocks[1:])),
                'Live /clock moved backwards during isolated historical-clock replay')
        require(all(name not in str(row['callerid']) for row in clocks),'Live clock received a message from the bag player')
        after=world_time(rospy);require(after>before,'Actual native world did not continue advancing during bag replay')
        report.update(rows=rows,bag_record_time_span_s=span,wall_delivery_span_s=wall_span,
            native_world_before_s=before,native_world_after_s=after,
            live_clock_receive_count=len(clocks),live_clock_first=clocks[0],live_clock_last=clocks[-1],
            clock_policy='Actual recorded Clock messages use the absolute recorded_clock remap; the disabled player relative clock advertisement has a separate internal namespace; no --clock; live clock remains native')
        return report
    except BaseException as error:
        # Preserve the actual failure snapshot before unregistering subscribers;
        # this does not extend a replay deadline or relax any comparison.
        report['error']=type(error).__name__+': '+str(error)
        report['failure_snapshot_monotonic_ns']=time.monotonic_ns()
        report['expected_counts']={topic:len(rows) for topic,rows in expected.items()}
        report['player_returncode_before_cleanup']=process.process.poll()
        try:
            report['process_close']=process.report if process.closed else process.stop()
        except Exception as cleanup_error:
            report['process_close']=process.report
            report['process_cleanup_error']=type(cleanup_error).__name__+': '+str(cleanup_error)
        report['rows']=observer.snapshot()
        report['received_counts']={topic:len(rows) for topic,rows in report['rows'].items()}
        destination=Path(bag).parent/('replay-'+str(rate)+'-failure.json')
        destination.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        raise
    finally:
        if not process.closed:report['process_close']=process.stop()
        observer.close()


def write_report(result):
    folder=Path('build');folder.mkdir(exist_ok=True);temporary=folder/('.verification-'+uuid.uuid4().hex+'.json')
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(str(temporary),str(folder/'verification.json'))


def verify():
    rospy,ns=initialize('verify');evidence=LiveEvidence(rospy,ns);failure=None;original_emit=None;emit_name=None;started=time.monotonic()
    result={'passed':False,'assignment_pass':False,'lesson':46,'namespace':ns,'checks':{},'trusted_verifier_sha256':sha(__file__)}
    try:
        wait_for(lambda:len(evidence.snapshot()[0])==2,8,'two separate real observer/monitor nodes')
        identity=evidence.snapshot()[0]
        for row in identity.values():
            require(row['instrument_sha256']==sha('instrument.py'),'Personal instrument changed; restart this experiment')
            require(row['session_token']==os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'A46 node belongs to another owned session')
        require(identity['observer']['world_sha256']==sha('world.sdf'),'Personal SDF changed since simulator startup')
        emit_name=identity['observer']['emit_parameter'];original_emit=get_param(rospy,emit_name)
        require(type(original_emit) is bool,'Initial emit_state must be a boolean')
        set_param(rospy,emit_name,True)
        wait_for(lambda:evidence.snapshot()[1] and any(row['level']==0 for row in evidence.snapshot()[2]),5,
                 'real state publication and fresh-data OK after enabling the observer')
        directory=Path('build/bags')/uuid.uuid4().hex
        record=record_motion(rospy,ns,evidence,directory)
        bag=Path(record['path']);bag_sha=record['audit']['bag_sha256']
        result.update(bag_path=str(bag),bag_sha256=bag_sha);result['checks']['real_recording']=record
        replays=[replay_once(rospy,ns,bag,record['audit']['records'],rate,evidence) for rate in (1.,2.)]
        require(sha(bag)==bag_sha,'Replay changed the original bag bytes')
        ratio=replays[0]['wall_delivery_span_s']/replays[1]['wall_delivery_span_s']
        require(1.6<ratio<2.5,'Measured 1x/2x state receipt spans did not show the expected speed change')
        result['checks']['real_cli_replays']={'runs':replays,'measured_1x_to_2x_wall_span_ratio':ratio,
            'comparison':'First-to-last matched state receipt, excluding process startup and subscriber-wait delay'}
        marker=time.monotonic_ns();set_param(rospy,emit_name,False)
        wait_for(lambda:any(row['receipt_monotonic_ns']>marker and row['level']==2 for row in evidence.snapshot()[2]),4,
                 'actual state-message loss reaching ERROR by wall freshness')
        stale=[row for row in evidence.snapshot()[2] if row['receipt_monotonic_ns']>marker]
        require(any(row['level']==1 for row in stale),'Staleness did not pass through the WARN interval')
        require(float(stale[-1]['values']['age_wall_s'])>=identity['monitor']['error_age_wall_s'],
                'ERROR does not correspond to the actual elapsed receive age')
        restored_marker=time.monotonic_ns();set_param(rospy,emit_name,True)
        wait_for(lambda:any(row['receipt_monotonic_ns']>restored_marker and row['level']==0 for row in evidence.snapshot()[2]),4,
                 'a real new state message restoring fresh-data OK')
        fresh=[row for row in evidence.snapshot()[1] if row['receipt_monotonic_ns']>restored_marker]
        require(fresh,'Health recovered without a real new state message')
        result['checks']['actual_stale_stream_diagnosis']={'emit_parameter':emit_name,'thresholds':identity['monitor'],
            'dropout_diagnostics':stale,'recovery_new_state':fresh[-1],
            'recovery_diagnostics':[row for row in evidence.snapshot()[2] if row['receipt_monotonic_ns']>restored_marker],
            'scope':'Observer actually stopped its topic; a separate subscriber monitored monotonic receipt age, never a manually forced diagnostic level'}
        result['passed']=True;return result
    except BaseException as error:failure=error;result['error']=type(error).__name__+': '+str(error);raise
    finally:
        if emit_name is not None:
            try:
                set_param(rospy,emit_name,original_emit);result['emit_parameter_restored']=get_param(rospy,emit_name)==original_emit
                require(result['emit_parameter_restored'],'emit_state parameter readback differs from the original value')
            except Exception as error:result['restoration_error']=str(error);result['passed']=False
        try:final_clear(rospy,failure)
        except Exception as error:result['force_cleanup_error']=str(error);result['passed']=False
        evidence.close();result['elapsed_wall_s']=time.monotonic()-started;write_report(result)
        if result.get('restoration_error') or result.get('force_cleanup_error'):
            raise RuntimeError('A46 cleanup/restoration failed; report retained')


def ready():
    rospy,ns=initialize('ready');evidence=LiveEvidence(rospy,ns)
    try:
        wait_for(lambda:len(evidence.snapshot()[0])==2 and evidence.snapshot()[1] and evidence.snapshot()[2],8,'observer and separate diagnostic monitor')
        return {'ready':True,'nodes':evidence.snapshot()[0]}
    finally:evidence.close()


def check_student():
    path=Path('student.py');spec=importlib.util.spec_from_file_location('a46_student_'+uuid.uuid4().hex,str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    allowed=('/workshop46/state','/workshop46/diagnostics','/clock')
    for topic,expected in (('/clock',True),('/workshop46/state',True),('/clock_extra',False),('/other',False)):
        require(module.is_selected_topic(topic,allowed) is expected,'Topic selection must be exact whitelist membership')
    for age,expected in ((0,0),(.249,0),(.25,1),(.699,1),(.7,2),(1,2)):
        require(module.health_level(age,.25,.7)==expected,'Incorrect freshness boundary')
    return {'algebra_checks_passed':True,'assignment_pass':False,'student_sha256':sha(path),
        'scope':'Independent local exercise, not automatically substituted into the reference experiment'}


def main():
    import rospy
    parser=argparse.ArgumentParser(description='A46 true rosbag CLI and actual stale-stream diagnostics')
    parser.add_argument('mode',choices=('observer','monitor','ready','state','pulse','verify','student-check'),default='verify',nargs='?')
    mode=parser.parse_args(rospy.myargv()[1:]).mode
    if mode=='observer':observer();return
    if mode=='monitor':monitor();return
    if mode=='ready':result=ready()
    elif mode=='verify':result=verify()
    elif mode=='student-check':result=check_student()
    else:
        rospy,ns=initialize('operator')
        if mode=='pulse':
            from std_srvs.srv import Trigger
            result=json.loads(call(rospy,ns+'/pulse',Trigger).message)
        else:
            evidence=LiveEvidence(rospy,ns)
            try:wait_for(lambda:evidence.snapshot()[1] and evidence.snapshot()[2],5,'actual state and health');result={'state':evidence.snapshot()[1][-1],'diagnostic':evidence.snapshot()[2][-1]}
            finally:evidence.close()
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),flush=True)


if __name__=='__main__':main()


