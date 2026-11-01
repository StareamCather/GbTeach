#!/usr/bin/env python3
"""Bounded physical 2R pushing mission; measurements come from Gazebo and ROS FT."""
import base64
import csv
import io
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid
from config import identity, prepared


def bounded(action, seconds=4):
    done,result=threading.Event(),{}
    def invoke():
        try:result['value']=action()
        except Exception as error:result['error']=str(error)
        finally:done.set()
    threading.Thread(target=invoke,daemon=True).start()
    if not done.wait(seconds):raise RuntimeError('Wall deadline on Gazebo/ROS service')
    if 'error' in result:raise RuntimeError(result['error'])
    return result.get('value')


class Mission:
    terminal=('SUCCEEDED','FAILED')
    active=('SETTLE','PUSH','HOLD','STOPPING')
    fields=('sim_time','phase','desired_x_m','desired_y_m','tip_x_m','tip_y_m','tip_z_m',
            'error_m','block_x_m','block_y_m','block_speed_mps','q1_rad','q2_rad','dq1_rps',
            'dq2_rps','tau1_nm','tau2_nm','force_stamp_ns','force_x_n','force_y_n','force_z_n',
            'horizontal_force_n','contacts')

    def __init__(self):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import WrenchStamped
        from gazebo_msgs.msg import ContactsState
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from std_srvs.srv import Trigger,SetBool
        self.root=Path.cwd().resolve();self.parameters,self.record=prepared()
        if not os.environ.get('GAZEBOTEACH_SESSION_TOKEN'):raise RuntimeError('Private session required')
        rospy.init_node('lesson37_controller',disable_signals=True)
        self.rospy=rospy;self.lock=threading.RLock();self.control_lock=threading.RLock();self.stop_event=threading.Event()
        self.sim=0.;self.phase='IDLE';self.reason='initial idle';self.phase_since=0.
        self.start_sim=None;self.start_wall=None;self.truth=None;self.force=None;self.native_state=None
        self.native_process=None;self.native_identity=None;self.native_reader=None;self.native_error_log=None
        self.native_queue=queue.Queue()
        self.desired={'x':.4,'y':0.};self.stable_since=None;self.closed=False
        self.last_sample_wall=0.;self.last_state_wall=0.;self.last_feedback_wall=time.monotonic()
        self.new_run()
        self.state_pub=rospy.Publisher('/lesson37/state',String,queue_size=1,latch=True)
        self.config_pub=rospy.Publisher('/lesson37/configuration',String,queue_size=1,latch=True)
        self.tf=tf2_ros.TransformBroadcaster()
        self.config_pub.publish(String(data=json.dumps({'identity':identity(self.record),'pid':os.getpid(),
            'node':rospy.get_name(),'joint_offsets_rad':[-math.acos(4/7),2*math.acos(4/7)],
            'link_lengths_m':[.35,.35],'tool_goal_world_m':[.64,0,.1],
            'block_displacement_goal_m':.1,'force_frame':'pusher',
            'tf_source':'actual /gazebo/get_link_state pusher pose in world'})))
        self.subs=[rospy.Subscriber('/clock',Clock,self.clock,queue_size=100),
                   rospy.Subscriber('/arm_push/wrench',WrenchStamped,self.on_force,queue_size=300),
                   rospy.Subscriber('/arm_push/contacts',ContactsState,self.on_contacts,queue_size=100)]
        self.services=[rospy.Service('/lesson37/start',Trigger,self.start),
                       rospy.Service('/lesson37/reset',Trigger,self.reset),
                       rospy.Service('/lesson37/enable_arm',SetBool,self.enable)]
        for action,key in [('cap','torque_limit_nm'),('duration','push_duration_s')]:
            ack=self.native(action,self.parameters[key])
            if ack[key]!=self.parameters[key]:raise RuntimeError('Native configuration not applied: '+key)
        self.publish_state()
        print(json.dumps({'controller_ready':True,'pid':os.getpid(),'run_id':self.run_id}),flush=True)

    def new_run(self):
        # Closing a run is serialized with sensor callbacks that own raw bytes.
        with self.lock:self._new_run()

    def _new_run(self):
        for name in ('raw_file','event_file','measurement_file'):
            handle=getattr(self,name,None)
            if handle:handle.close()
        self.run_id=uuid.uuid4().hex;self.directory=self.root/'build/runs'/self.run_id
        self.directory.mkdir(parents=True)
        self.raw_file=(self.directory/'raw_ros.jsonl').open('w',buffering=1)
        self.event_file=(self.directory/'events.jsonl').open('w',buffering=1)
        self.measurement_file=(self.directory/'measurements.csv').open('w',newline='',buffering=1)
        self.measurements=csv.DictWriter(self.measurement_file,fieldnames=self.fields);self.measurements.writeheader()
        self.events=[];self.rows=[];self.contact_count=0;self.peak_horizontal_force=0.;self.finished=False
        self.initial_block=None;self.last_contacts=0
        self.stop_confirmation={'requested':False,'acknowledged':False,'applied_zero':False}

    def event(self,source,data):
        row={'source':source,'receive_monotonic_ns':time.monotonic_ns(),'sim_time':self.sim,'data':data}
        self.event_file.write(json.dumps(row,allow_nan=False)+'\n')
        if source=='transition':self.events.append(row)

    def raw(self,source,message):
        buf=io.BytesIO();message.serialize(buf)
        self.raw_file.write(json.dumps({'source':source,'receive_monotonic_ns':time.monotonic_ns(),
            'stamp_ns':message.header.stamp.to_nsec() if hasattr(message,'header') else None,
            'serialized_ros_base64':base64.b64encode(buf.getvalue()).decode('ascii')})+'\n')

    def clock(self,message):
        with self.lock:
            now=message.clock.to_sec()
            if now<self.sim-1e-6:
                self.force=None;self.truth=None;self.native_state=None
                self.event('clock_reset',{'previous':self.sim,'now':now})
                if self.phase in self.active:self.transition('FAILED','Simulation reset during active mission')
            self.sim=now

    def on_force(self,message):
        with self.lock:
            self.raw('wrench',message);f=message.wrench.force;t=message.wrench.torque
            self.force={'stamp_ns':message.header.stamp.to_nsec(),'frame':message.header.frame_id,
                        'x':f.x,'y':f.y,'z':f.z,'tx':t.x,'ty':t.y,'tz':t.z,'horizontal':math.hypot(f.x,f.y)}
            self.last_feedback_wall=time.monotonic()
            if self.phase in self.active:self.peak_horizontal_force=max(self.peak_horizontal_force,self.force['horizontal'])

    def on_contacts(self,message):
        with self.lock:
            self.raw('contacts',message);self.last_contacts=len(message.states)
            if message.states and self.phase in self.active:
                self.contact_count+=len(message.states)
                self.event('contact',{'pairs':[[s.collision1_name,s.collision2_name] for s in message.states]})

    def native_line(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            try:line=self.native_queue.get(timeout=min(.25,max(.001,deadline-time.monotonic())))
            except queue.Empty:
                if self.native_process.poll() is not None:raise RuntimeError('Owned native stream exited; automatic restart is forbidden')
                continue
            if line is None:raise RuntimeError('Owned native stream closed; see build/native-client.stderr.log')
            value=json.loads(line)
            if 'error' in value:raise RuntimeError('Native stream: '+value['error'])
            return value
        raise RuntimeError('Wall deadline waiting for native stream response; command was not retried')

    def ensure_native(self):
        if self.native_process is not None:
            if self.native_process.poll() is not None:raise RuntimeError('Owned native stream exited; automatic restart is forbidden')
            return
        self.native_error_log=(self.root/'build/native-client.stderr.log').open('a',buffering=1)
        self.native_process=subprocess.Popen([str(self.root/'build/arm_client'),'stream','0'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.native_error_log,text=True,bufsize=1,start_new_session=True)
        pid=self.native_process.pid
        proc=Path('/proc/{}'.format(pid));stat=proc.joinpath('stat').read_text().rsplit(') ',1)[1].split()
        self.native_identity={'pid':pid,'start_time':stat[19],'ppid':int(stat[1]),
            'pgid':os.getpgid(pid),'sid':os.getsid(pid),'uid':proc.stat().st_uid,
            'session_token':os.environ['GAZEBOTEACH_SESSION_TOKEN'],
            'mode':'one persistent stdin JSON / native UUID ACK stream; separate owned process session for EOF cleanup'}
        def receive():
            try:
                for line in self.native_process.stdout:self.native_queue.put(line)
            finally:self.native_queue.put(None)
        self.native_reader=threading.Thread(target=receive,daemon=True);self.native_reader.start()
        ready=self.native_line(10)
        if ready.get('stream_ready') is not True or not isinstance(ready.get('status'),dict):
            raise RuntimeError('Native stream did not complete its fresh read-only response handshake')
        self.native_state=ready['status'];self.event('native_stream_ready',{'process':self.native_identity,'ack':self.native_state})

    def native(self,action,value=0):
        self.ensure_native()
        request_id=uuid.uuid4().hex
        command={'action':action,'value':int(value),'id':request_id}
        # No mutating request is retried. Losing this ACK is an explicit error,
        # and the same owned native process is never replaced automatically.
        self.native_process.stdin.write(json.dumps(command)+'\n');self.native_process.stdin.flush()
        answer=self.native_line(5)
        if answer.get('id')!=request_id:raise RuntimeError('Native stream response identity differs')
        self.native_state=answer
        self.event('native_command',{'action':action,'request_id':request_id,'process':self.native_identity,'ack':answer})
        return answer

    def close_native(self):
        process=self.native_process
        if process is None:return
        try:
            if process.stdin and not process.stdin.closed:
                try:process.stdin.close()
                except OSError as error:self.event('native_stream_close_error',{'operation':'stdin EOF','error':str(error)})
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
            if self.native_reader:self.native_reader.join(timeout=1)
        finally:
            for handle in (process.stdout,self.native_error_log):
                if handle:
                    try:handle.close()
                    except OSError as error:self.event('native_stream_close_error',{'operation':'close log pipe','error':str(error)})

    def best_effort_stop(self):
        if self.stop_confirmation['requested']:return
        self.stop_confirmation.update({'requested':True,'requested_monotonic_ns':time.monotonic_ns()})
        try:
            self.native('stop');self.stop_confirmation['acknowledged']=True
        except Exception as error:
            self.stop_confirmation['error']=str(error)
            self.event('stop_unconfirmed',{'error':str(error),'note':'Original failure retained; no claim that zero torque executed.'})

    def call(self,path,kind,**kwargs):
        def invoke():
            self.rospy.wait_for_service(path,timeout=2);proxy=self.rospy.ServiceProxy(path,kind)
            try:result=proxy(**kwargs)
            finally:proxy.close()
            if hasattr(result,'success') and not result.success:
                raise RuntimeError(path+': '+getattr(result,'status_message',getattr(result,'message','refused')))
            return result
        return bounded(invoke)

    def sample_truth(self):
        from gazebo_msgs.srv import GetJointProperties,GetModelState,GetLinkState,GetWorldProperties
        from geometry_msgs.msg import TransformStamped
        begin=time.monotonic_ns()
        before=self.call('/gazebo/get_world_properties',GetWorldProperties)
        j1=self.call('/gazebo/get_joint_properties',GetJointProperties,joint_name='push_arm::shoulder_joint')
        j2=self.call('/gazebo/get_joint_properties',GetJointProperties,joint_name='push_arm::elbow_joint')
        p=self.call('/gazebo/get_link_state',GetLinkState,link_name='push_arm::pusher',reference_frame='world').link_state
        b=self.call('/gazebo/get_model_state',GetModelState,model_name='push_block',relative_entity_name='world')
        after=self.call('/gazebo/get_world_properties',GetWorldProperties)
        sample_sim=(before.sim_time+after.sim_time)/2
        if not j1.position or not j2.position:raise RuntimeError('Actual joint position unavailable')
        self.truth={'sample_begin_monotonic_ns':begin,'sample_end_monotonic_ns':time.monotonic_ns(),
                    'world_time_before_s':before.sim_time,'world_time_after_s':after.sim_time,
                    'sim_time':sample_sim,'time_basis':'Midpoint of sequential native world query brackets; queried states are not atomic.',
                    'tip_x':p.pose.position.x,'tip_y':p.pose.position.y,'tip_z':p.pose.position.z,
                    'tip_speed':math.hypot(p.twist.linear.x,p.twist.linear.y),
                    'block_x':b.pose.position.x,'block_y':b.pose.position.y,
                    'block_speed':math.hypot(b.twist.linear.x,b.twist.linear.y),
                    'q1':j1.position[0],'q2':j2.position[0],
                    'dq1':j1.rate[0] if j1.rate else None,'dq2':j2.rate[0] if j2.rate else None}
        if self.start_sim is not None:
            u=max(0.,min(1.,(sample_sim-self.start_sim-.8)/self.parameters['push_duration_s']))
            self.desired={'x':.4+.24*u*u*(3-2*u),'y':0.}
        self.truth['error']=math.hypot(self.truth['tip_x']-self.desired['x'],self.truth['tip_y'])
        self.event('ground_truth',self.truth)
        tf=TransformStamped();tf.header.frame_id='world';tf.header.stamp=self.rospy.Time.from_sec(sample_sim)
        tf.child_frame_id='pusher';tf.transform.translation.x=p.pose.position.x
        tf.transform.translation.y=p.pose.position.y;tf.transform.translation.z=p.pose.position.z
        tf.transform.rotation=p.pose.orientation;self.tf.sendTransform(tf)
        return self.truth

    def publish_state(self):
        from std_msgs.msg import String
        value={'phase':self.phase,'reason':self.reason,'run_id':self.run_id,'sim_time':self.sim,
               'phase_since':self.phase_since,'start_sim':self.start_sim,'desired':self.desired,
               'truth':self.truth,'force':self.force,'native':self.native_state,
               'native_client':self.native_identity,'stop_confirmation':self.stop_confirmation,
               'contact_count':self.contact_count,'directory':str(self.directory),'parameters':self.parameters}
        self.state_pub.publish(String(data=json.dumps(value,allow_nan=False)))

    def transition(self,phase,reason):
        previous=self.phase;self.phase=phase;self.phase_since=self.sim;self.reason=reason;self.stable_since=None
        self.event('transition',{'from':previous,'to':phase,'reason':reason})
        if phase=='FAILED':self.best_effort_stop()
        self.publish_state()

    def fresh(self):
        return self.force is not None and self.force['frame']=='pusher' and -.001<=self.sim-self.force['stamp_ns']/1e9<=.2

    def start(self,request):
        from std_srvs.srv import TriggerResponse
        with self.control_lock:
            try:
                prepared()
                if self.phase!='IDLE':raise RuntimeError('Explicit reset required before start')
                if not self.fresh():raise RuntimeError('Fresh real FT sample required; bootstrap physics first')
                self.desired={'x':.4,'y':0.};self.sample_truth()
                if self.truth['error']>.015 or abs(self.truth['block_x']-.51)>.02:
                    raise RuntimeError('Physical start geometry differs; explicit reset --world required')
                self.initial_block=self.truth['block_x'];self.start_wall=time.monotonic()
                self.native('start');self.start_sim=None
                self.transition('SETTLE','Native torque trajectory accepted; waiting actual simulation tick')
                return TriggerResponse(True,self.run_id)
            except Exception as error:return TriggerResponse(False,str(error))

    def reset(self,request):
        from std_srvs.srv import TriggerResponse
        with self.control_lock:
            try:
                if self.phase in self.active:self.transition('FAILED','Operator explicit reset')
                if not self.finished:self.finish()
                self.native('hold');self.native('enable',1)
                self.new_run();self.phase='IDLE';self.reason='Explicit controller reset; use --world to restore geometry'
                self.start_sim=None;self.start_wall=None;self.force=None;self.truth=None
                self.desired={'x':.4,'y':0.};self.phase_since=self.sim;self.stable_since=None
                self.publish_state();return TriggerResponse(True,self.run_id)
            except Exception as error:return TriggerResponse(False,str(error))

    def enable(self,request):
        from std_srvs.srv import SetBoolResponse
        with self.control_lock:
            try:return SetBoolResponse(True,json.dumps(self.native('enable',request.data)))
            except Exception as error:return SetBoolResponse(False,str(error))

    def record_measurement(self):
        if not self.truth or not self.force:return
        t,f,n=self.truth,self.force,self.native_state or {}
        row=dict(zip(self.fields,(t['sim_time'],self.phase,self.desired['x'],self.desired['y'],t['tip_x'],t['tip_y'],t['tip_z'],
            t['error'],t['block_x'],t['block_y'],t['block_speed'],t['q1'],t['q2'],t['dq1'],t['dq2'],
            n.get('tau1',0),n.get('tau2',0),f['stamp_ns'],f['x'],f['y'],f['z'],f['horizontal'],self.last_contacts)))
        self.measurements.writerow(row);self.rows.append(row)

    def good_hold(self):
        t=self.truth
        return bool(t and t['error']<.025 and t['block_x']-self.initial_block>.10 and abs(t['block_y'])<.08
                    and t['tip_speed']<.025 and t['block_speed']<.025
                    and abs(t['dq1'] or 0)<.08 and abs(t['dq2'] or 0)<.08
                    and self.contact_count>0 and self.peak_horizontal_force>.12)

    def tick(self):
        # Serialize mission actions with start/reset, separately from the cache
        # lock: /clock and FT callbacks stay responsive during service/native IO.
        with self.control_lock:
            now=time.monotonic()
            if now-self.last_sample_wall>=.12 and not self.finished:
                ack=self.native('status')
                if self.phase in self.active and self.start_sim is None and ack['start_sim']>=0:self.start_sim=ack['start_sim']
                self.sample_truth();self.record_measurement();self.last_sample_wall=time.monotonic()
            if self.phase in self.active:
                if now-self.start_wall>40:self.transition('FAILED','40 second wall deadline')
                elif now-self.last_feedback_wall>3:self.transition('FAILED','3 second wall FT deadline (paused/stalled)')
                elif self.start_sim is not None:
                    elapsed=self.sim-self.start_sim
                    if elapsed>self.parameters['push_duration_s']+7:self.transition('FAILED','Bounded physical goal timeout')
                    elif self.phase=='SETTLE' and elapsed>=.8:self.transition('PUSH','Actual native time reached physical push trajectory')
                    elif self.phase=='PUSH' and elapsed>=.8+self.parameters['push_duration_s']:
                        self.native('hold');self.transition('HOLD','Final IK reference reached; verify actual tool and block motion')
                    elif self.phase in ('HOLD','STOPPING'):
                        if self.good_hold() and self.fresh():
                            if self.stable_since is None:self.stable_since=self.sim
                            if self.sim-self.stable_since>=.4:
                                if self.phase=='HOLD':self.transition('STOPPING','Actual tool/block velocities and tracking error stable; PD hold stays active')
                                else:self.transition('SUCCEEDED','Actual final tip, block displacement, contact force and stationary hold verified')
                        else:self.stable_since=None
                        if self.phase=='HOLD' and self.sim-self.phase_since>3:
                            self.transition('FAILED','Tool tracking/block displacement/contact timeout; actuation disabled or insufficient')
            if self.phase in self.terminal and not self.finished:
                # A failed zero command must actually execute before pause. A
                # successful hold remains PD controlled and may need torque.
                if self.phase=='FAILED':
                    try:
                        ack=self.native('status')
                        self.stop_confirmation['applied_zero']=abs(ack['tau1'])<=1e-9 and abs(ack['tau2'])<=1e-9
                    except Exception as error:
                        self.stop_confirmation['error']=str(error)
                    elapsed=(time.monotonic_ns()-self.stop_confirmation['requested_monotonic_ns'])/1e9
                    if not self.stop_confirmation['applied_zero'] and elapsed<2:return
                try:self.sample_truth();self.record_measurement()
                except Exception as error:self.event('final_measurement_unavailable',{'error':str(error)})
                self.finish()
            if time.monotonic()-self.last_state_wall>=.1:self.publish_state();self.last_state_wall=time.monotonic()

    def finish(self):
        self.measurement_file.flush();self.raw_file.flush();self.event_file.flush()
        self.make_svg()
        result={'run_id':self.run_id,'phase':self.phase,'reason':self.reason,'identity':identity(self.record),
                'start_sim':self.start_sim,'end_sim':self.sim,'truth':self.truth,'desired':self.desired,
                'initial_block_x':self.initial_block,'force':self.force,'native':self.native_state,
                'native_client':self.native_identity,'stop_confirmation':self.stop_confirmation,
                'contact_count':self.contact_count,'peak_horizontal_force_n':self.peak_horizontal_force,
                'measurement_count':len(self.rows),'transitions':self.events,'directory':str(self.directory),
                'hold_semantics':'success: fixed final joint references with bounded PD torque; failure: request SetForce zero, consult stop_confirmation for actual confirmation'}
        temporary=self.directory/'result.tmp';temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');temporary.replace(self.directory/'result.json')
        (self.root/'build/latest_run.json').write_text(json.dumps({'run_id':self.run_id,'directory':str(self.directory)})+'\n')
        self.finished=True

    def make_svg(self):
        rows=self.rows
        svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1080" height="850" viewBox="0 0 1080 850">',
             '<rect width="1080" height="850" fill="#f5f7fb"/>',
             '<g font-family="sans-serif" font-size="18" fill="#17243b">',
             '<text x="40" y="30">Measured physical 2R push: actual tip / error / horizontal wrist force / block</text>']
        if rows:
            first=float(rows[0]['sim_time']);span=max(.1,float(rows[-1]['sim_time'])-first)
            for panel,(keys,title,lo,hi) in enumerate([
                (('tip_x_m','desired_x_m','block_x_m'),'World x: actual tool (blue), desired tool (green), block (red)',.35,.82),
                (('tip_y_m','block_y_m'),'World y: actual tool (blue), block (green)',-.08,.08),
                (('error_m',),'Actual tool tracking error [m]',0,max(.03,max(float(r['error_m']) for r in rows)*1.1)),
                (('horizontal_force_n',),'Real horizontal wrist force [N] (Fz in CSV; full wrench in raw ROS)',0,max(1.,max(float(r['horizontal_force_n']) for r in rows)*1.1))]):
                y=65+panel*190;svg.append('<text x="45" y="{}">{}</text>'.format(y,title))
                svg.append('<path d="M 70 {} V {} H 1010" fill="none" stroke="#a4adbb"/>'.format(y+15,y+145))
                for key,color in zip(keys,('#2460aa','#20925f','#c65042')):
                    points=' '.join('{:.2f},{:.2f}'.format(70+940*(float(r['sim_time'])-first)/span,y+145-120*(float(r[key])-lo)/(hi-lo)) for r in rows)
                    svg.append('<polyline points="{}" fill="none" stroke="{}" stroke-width="2"/>'.format(points,color))
                svg.append('<text x="70" y="{}" font-size="13">{:.3g} to {:.3g}; time {:.2f}–{:.2f} s</text>'.format(y+168,lo,hi,first,first+span))
        svg.append('</g></svg>');(self.directory/'measurements.svg').write_text('\n'.join(svg)+'\n')

    def run(self):
        def stop(*_):self.stop_event.set()
        signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
        try:
            while not self.rospy.is_shutdown() and not self.stop_event.is_set():
                try:self.tick()
                except Exception as error:
                    with self.control_lock:
                        self.transition('FAILED','Controller exception: '+str(error))
                        if not self.finished:self.finish()
                self.stop_event.wait(.025)
        finally:
            try:
                with self.control_lock:
                    self.best_effort_stop()
                    if not self.finished:
                        if self.phase in self.active:self.transition('FAILED','Controller shutdown')
                        self.finish()
            finally:self.close_native()


if __name__=='__main__':Mission().run()
