#!/usr/bin/env python3
"""Physical sensor-gated FSM and measured scan projection; not SLAM/navigation."""
import base64
import csv
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
from collections import deque
from config import identity, prepared


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def bounded(action, seconds=4):
    done, result = threading.Event(), {}
    def invoke():
        try: result['value'] = action()
        except Exception as error: result['error'] = str(error)
        finally: done.set()
    threading.Thread(target=invoke, daemon=True).start()
    if not done.wait(seconds): raise RuntimeError('Wall deadline on Gazebo/ROS service')
    if 'error' in result: raise RuntimeError(result['error'])
    return result.get('value')


class Mission:
    terminal = ('SUCCEEDED', 'FAILED')
    active = ('SCAN', 'APPROACH', 'WAIT_DOOR', 'PASS', 'GOAL', 'STOPPING')
    fields = ('scan_stamp_ns','beam_index','range_m','angle_rad','odom_stamp_ns',
              'robot_x_m','robot_y_m','robot_yaw_rad','x_m','y_m')

    def __init__(self):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan, JointState
        from gazebo_msgs.msg import ContactsState
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from std_srvs.srv import Trigger, SetBool
        self.root=Path.cwd().resolve(); self.parameters,self.record=prepared()
        if not os.environ.get('GAZEBOTEACH_SESSION_TOKEN'): raise RuntimeError('Private session required')
        rospy.init_node('lesson36_controller',disable_signals=True)
        self.rospy=rospy; self.lock=threading.RLock(); self.stop_event=threading.Event()
        self.sim=0.; self.last_sim=0.; self.phase='IDLE'; self.reason='initial idle'
        self.odom=None; self.scan=None; self.odoms=deque(maxlen=400); self.door=None; self.gt=None
        self.front=None; self.bumper_count=0; self.last_feedback=time.monotonic(); self.clear_since=None
        self.start_sim=None; self.start_wall=None; self.phase_since=0.; self.commanded=(0.,0.)
        self.last_sample_wall=0.; self.last_state_wall=0.; self.closed=False; self.finished=False
        self.new_run()
        self.cmd=rospy.Publisher('/warehouse/cmd_vel',Twist,queue_size=1)
        self.state_pub=rospy.Publisher('/lesson36/state',String,queue_size=1,latch=True)
        self.config_pub=rospy.Publisher('/lesson36/configuration',String,queue_size=1,latch=True)
        self.static=tf2_ros.StaticTransformBroadcaster()
        transforms=[]
        for parent,child,xyz in [('world','warehouse/odom',(0,0,0)),
                                 ('warehouse/base_footprint','warehouse/laser',(.15,0,.2202))]:
            item=TransformStamped(); item.header.frame_id=parent; item.child_frame_id=child
            item.transform.translation.x,item.transform.translation.y,item.transform.translation.z=xyz
            item.transform.rotation.w=1; transforms.append(item)
        self.static.sendTransform(transforms)
        self.config_pub.publish(String(data=json.dumps({'identity':identity(self.record),'pid':os.getpid(),
            'node':rospy.get_name(),'scan_projection':'actual LaserScan + preceding actual world odometry; not SLAM',
            'sensor_offset_m':[.15,0,.2202]})))
        self.subs=[rospy.Subscriber('/clock',Clock,self.clock,queue_size=100),
            rospy.Subscriber('/warehouse/cmd_vel',Twist,lambda m:self.raw('cmd_vel',m),queue_size=100),
            rospy.Subscriber('/warehouse/odom',Odometry,self.on_odom,queue_size=300),
            rospy.Subscriber('/warehouse/scan',LaserScan,self.on_scan,queue_size=100),
            rospy.Subscriber('/warehouse/bumper',ContactsState,self.on_bumper,queue_size=100),
            rospy.Subscriber('/warehouse/joint_states',JointState,lambda m:self.raw('joint_states',m),queue_size=100)]
        self.services=[rospy.Service('/lesson36/start',Trigger,self.start),
            rospy.Service('/lesson36/reset',Trigger,self.reset),rospy.Service('/lesson36/enable_door',SetBool,self.enable)]
        cap_ack=self.native('cap',self.parameters['door_force_n'])
        if cap_ack.get('force_cap')!=self.parameters['door_force_n']:
            raise RuntimeError('Loaded native door force cap differs from prepared parameters')
        self.publish_state()
        print(json.dumps({'controller_ready':True,'pid':os.getpid(),'run_id':self.run_id}),flush=True)

    def new_run(self):
        for name in ('raw_file','event_file','point_file'):
            handle=getattr(self,name,None)
            if handle: handle.close()
        self.run_id=uuid.uuid4().hex; self.directory=self.root/'build/runs'/self.run_id
        self.directory.mkdir(parents=True)
        self.raw_file=(self.directory/'raw_ros.jsonl').open('w',buffering=1)
        self.event_file=(self.directory/'events.jsonl').open('w',buffering=1)
        self.point_file=(self.directory/'points.csv').open('w',newline='',buffering=1)
        self.points=csv.DictWriter(self.point_file,fieldnames=self.fields);self.points.writeheader()
        self.point_count=0; self.projection_rejected=0; self.events=[]
        self.bumper_count=0;self.finished=False

    def event(self,source,data):
        row={'source':source,'receive_monotonic_ns':time.monotonic_ns(),'sim_time':self.sim,'data':data}
        self.event_file.write(json.dumps(row,allow_nan=False)+'\n')
        if source=='transition': self.events.append(row)

    def raw(self,source,message):
        with self.lock:
            buf=io.BytesIO();message.serialize(buf)
            self.raw_file.write(json.dumps({'source':source,'receive_monotonic_ns':time.monotonic_ns(),
                'stamp_ns':message.header.stamp.to_nsec() if hasattr(message,'header') else None,
                'serialized_ros_base64':base64.b64encode(buf.getvalue()).decode('ascii')})+'\n')

    def clock(self,message):
        with self.lock:
            now=message.clock.to_sec()
            if now<self.sim-1e-6:
                self.odom=None;self.scan=None;self.odoms.clear();self.front=None;self.door=None;self.gt=None
                self.event('clock_reset',{'previous':self.sim,'now':now})
                if self.phase in self.active: self.transition('FAILED','Simulation clock reset during mission')
            self.sim=now

    def on_odom(self,message):
        with self.lock:
            self.raw('odom',message)
            p=message.pose.pose.position
            value={'stamp_ns':message.header.stamp.to_nsec(),'x':p.x,'y':p.y,'yaw':yaw(message.pose.pose.orientation),
                   'v':message.twist.twist.linear.x,'w':message.twist.twist.angular.z,
                   'parent':message.header.frame_id,'child':message.child_frame_id}
            self.odom=value;self.odoms.append(value);self.last_feedback=time.monotonic()

    def on_scan(self,message):
        with self.lock:
            self.raw('scan',message);self.scan=message
            values=[r for i,r in enumerate(message.ranges) if abs(message.angle_min+i*message.angle_increment)<.08 and math.isfinite(r) and message.range_min<=r<=message.range_max]
            self.front=min(values) if values else None
            if self.finished: return
            stamp=message.header.stamp.to_nsec()
            # A preceding sample avoids using future poses and exposes the exact
            # measurement chosen, so an independent reader can reproduce it.
            odom=next((p for p in reversed(self.odoms) if p['stamp_ns']<=stamp),None)
            if not odom or stamp-odom['stamp_ns']>120000000:
                self.projection_rejected+=1;return
            for index in range(0,len(message.ranges),6):
                distance=message.ranges[index]
                if not math.isfinite(distance) or not message.range_min<=distance<=message.range_max: continue
                theta=message.angle_min+index*message.angle_increment
                lx=.15+distance*math.cos(theta);ly=distance*math.sin(theta);a=odom['yaw']
                x=odom['x']+math.cos(a)*lx-math.sin(a)*ly
                y=odom['y']+math.sin(a)*lx+math.cos(a)*ly
                self.points.writerow(dict(zip(self.fields,(stamp,index,distance,theta,odom['stamp_ns'],odom['x'],odom['y'],a,x,y))))
                self.point_count+=1

    def on_bumper(self,message):
        with self.lock:
            self.raw('bumper',message)
            if message.states:
                self.bumper_count+=len(message.states)
                self.event('contact',{'pairs':[[s.collision1_name,s.collision2_name] for s in message.states]})
                if self.phase in self.active: self.transition('FAILED','Body contact detected')

    def command(self,v=0.,w=0.):
        from geometry_msgs.msg import Twist
        item=Twist();item.linear.x=v;item.angular.z=w;self.cmd.publish(item);self.commanded=(v,w)

    def native(self,action,value):
        result=subprocess.run([str(self.root/'build/door_client'),action,str(int(value))],text=True,capture_output=True,timeout=7)
        if result.returncode: raise RuntimeError('Native door command failed: '+result.stdout+result.stderr)
        data=json.loads(result.stdout.strip().splitlines()[-1]);self.event('door_command',{'action':action,'value':int(value),'ack':data})
        return data

    def call(self,path,kind,**kwargs):
        def invoke():
            self.rospy.wait_for_service(path,timeout=2)
            client=self.rospy.ServiceProxy(path,kind)
            try: result=client(**kwargs)
            finally: client.close()
            if hasattr(result,'success') and not result.success: raise RuntimeError(path+': '+result.status_message)
            return result
        return bounded(invoke)

    def sample_truth(self):
        from gazebo_msgs.srv import GetJointProperties,GetModelState
        j=self.call('/gazebo/get_joint_properties',GetJointProperties,joint_name='warehouse_door::slide_joint')
        m=self.call('/gazebo/get_model_state',GetModelState,model_name='warehouse_bot',relative_entity_name='world')
        d=self.call('/gazebo/get_model_state',GetModelState,model_name='warehouse_door',relative_entity_name='world')
        self.door={'q':j.position[0],'velocity':j.rate[0] if j.rate else None,'success':j.success}
        self.gt={'x':m.pose.position.x,'y':m.pose.position.y,'yaw':yaw(m.pose.orientation),
                 'v':math.hypot(m.twist.linear.x,m.twist.linear.y),'w':m.twist.angular.z,
                 'door_model_x':d.pose.position.x,'door_model_y':d.pose.position.y}
        self.event('ground_truth',{'door_joint':self.door,'robot':self.gt})

    def publish_state(self):
        from std_msgs.msg import String
        value={'phase':self.phase,'reason':self.reason,'run_id':self.run_id,'sim_time':self.sim,
               'phase_since':self.phase_since,'start_sim':self.start_sim,'door_q':self.door['q'] if self.door else None,
               'front_m':self.front,'x':self.odom['x'] if self.odom else None,'y':self.odom['y'] if self.odom else None,
               'yaw':self.odom['yaw'] if self.odom else None,'command':list(self.commanded),
               'point_count':self.point_count,'directory':str(self.directory),'parameters':self.parameters}
        self.state_pub.publish(String(data=json.dumps(value,allow_nan=False)))

    def transition(self,phase,reason):
        previous=self.phase;self.phase=phase;self.phase_since=self.sim;self.reason=reason;self.clear_since=None
        self.event('transition',{'from':previous,'to':phase,'reason':reason})
        if phase in self.terminal: self.command()
        self.publish_state()

    def start(self,request):
        from std_srvs.srv import TriggerResponse
        with self.lock:
            try:
                prepared()
                if self.phase!='IDLE': raise RuntimeError('Explicit /lesson36/reset required before start')
                if not self.odom or not self.scan or not -.001<=self.sim-self.scan.header.stamp.to_sec()<=.3 or not -.001<=self.sim-self.odom['stamp_ns']/1e9<=.15:
                    raise RuntimeError('Fresh scan and odometry required; run bootstrap while idle')
                if self.odom['x']>=-.40: raise RuntimeError('Start must be before door; explicit world reset needed for a completed mission')
                self.sample_truth()
                if self.door['q']>.08: raise RuntimeError('Door must physically close before start')
                self.start_sim=self.sim;self.start_wall=time.monotonic();self.closed=False
                self.transition('SCAN','Start accepted with actual closed door and fresh feedback')
                return TriggerResponse(True,self.run_id)
            except Exception as error: return TriggerResponse(False,str(error))

    def reset(self,request):
        from std_srvs.srv import TriggerResponse
        with self.lock:
            try:
                self.command();self.native('open',False);self.native('enable',True)
                if self.phase in self.active: self.transition('FAILED','Operator explicit reset')
                if not self.finished: self.finish()
                self.new_run();self.phase='IDLE';self.reason='Explicit reset; door close/enabled, robot pose preserved'
                self.phase_since=self.sim;self.start_sim=None;self.start_wall=None;self.odom=None;self.scan=None
                self.front=None;self.odoms.clear();self.gt=None;self.door=None;self.last_feedback=time.monotonic()
                self.publish_state();return TriggerResponse(True,self.run_id)
            except Exception as error: return TriggerResponse(False,str(error))

    def enable(self,request):
        from std_srvs.srv import SetBoolResponse
        with self.lock:
            try:
                result=self.native('enable',request.data)
                return SetBoolResponse(True,json.dumps(result))
            except Exception as error: return SetBoolResponse(False,str(error))

    def drive(self,target):
        a=self.odom['yaw'];x=self.odom['x'];y=self.odom['y'];error=target-x
        desired=math.atan2(-1.0*y,max(.35,error))
        w=max(-.6,min(.6,2.5*wrap(desired-a)))
        v=max(-.05,min(self.parameters['linear_speed_mps'],.9*error))
        if abs(wrap(desired-a))>.5: v=0.
        self.command(v,w)

    def tick(self):
        with self.lock:
            now=time.monotonic()
            if self.phase in self.active:
                if now-self.start_wall>90: self.transition('FAILED','90 second wall deadline')
                elif now-self.last_feedback>3: self.transition('FAILED','3 second wall feedback deadline (paused/stalled)')
                elif self.sim-self.start_sim>40: self.transition('FAILED','40 second simulation deadline')
                if now-self.last_sample_wall>=.08:
                    self.sample_truth();self.last_sample_wall=now
                fresh=self.odom and self.scan and -.001<=self.sim-self.odom['stamp_ns']/1e9<.15 and -.001<=self.sim-self.scan.header.stamp.to_sec()<.3
                if not fresh: self.command()
                elif self.odom['parent']!='warehouse/odom' or self.odom['child']!='warehouse/base_footprint' or self.scan.header.frame_id!='warehouse/laser':
                    self.transition('FAILED','Unexpected feedback frame identity')
                elif self.phase=='SCAN':
                    self.command()
                    if self.sim-self.phase_since>=.5:
                        if self.front is None or not .35<self.front<2.: self.transition('FAILED','Closed door not visible in actual front ray')
                        else: self.transition('APPROACH','Actual closed-door scan acquired')
                elif self.phase=='APPROACH':
                    if self.front is None: self.transition('FAILED','No finite front ray')
                    elif self.front<.65 or self.odom['x']>-.85:
                        self.command();self.transition('WAIT_DOOR','Actual laser approached closed door');self.native('open',True)
                    else: self.drive(-.75)
                elif self.phase=='WAIT_DOOR':
                    self.command()
                    if self.sim-self.phase_since>6: self.transition('FAILED','Door clearance timeout; zero command before door')
                    elif self.door and self.door['q']>1.10 and self.front is not None and self.front>1.2:
                        if self.clear_since is None: self.clear_since=self.sim
                        if self.sim-self.clear_since>=.3: self.transition('PASS','Measured door joint and actual front laser both clear')
                    else: self.clear_since=None
                elif self.phase=='PASS':
                    if self.front is None or self.front<.30: self.transition('FAILED','Obstacle while crossing doorway')
                    else:
                        self.drive(2.2)
                        if self.odom['x']>.65:
                            self.native('open',False);self.closed=True;self.transition('GOAL','Actual robot rear cleared door; close requested')
                elif self.phase=='GOAL':
                    self.drive(2.2)
                    if abs(2.2-self.odom['x'])<.05: self.transition('STOPPING','Inside measured goal position band')
                elif self.phase=='STOPPING':
                    self.command()
                    good=self.gt and abs(self.gt['x']-2.2)<.06 and abs(self.gt['y'])<.10 and self.gt['v']<.028 and abs(self.gt['w'])<.08 and self.door and self.door['q']<.05
                    if good:
                        if self.clear_since is None: self.clear_since=self.sim
                        if self.sim-self.clear_since>=.5: self.transition('SUCCEEDED','Ground truth goal and stopped feedback stable .5 simulation second; door closed')
                    else: self.clear_since=None
            if self.phase not in self.active: self.command()
            if self.phase in self.terminal and not self.finished:
                self.sample_truth();self.finish()
            if now-self.last_state_wall>=.1: self.publish_state();self.last_state_wall=now

    def finish(self):
        self.point_file.flush();self.raw_file.flush();self.event_file.flush()
        svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="-2.9 -2.25 6.5 4.5" width="1040" height="720">',
             '<rect x="-2.9" y="-2.25" width="6.5" height="4.5" fill="#f5f7fa"/>',
             '<g fill="#235d9f" fill-opacity=".20">']
        # The SVG contains ONLY measured projected points, no SDF geometry.
        with (self.directory/'points.csv').open() as stream:
            for row in csv.DictReader(stream):
                svg.append('<circle cx="{}" cy="{}" r=".009"/>'.format(row['x_m'],-float(row['y_m'])))
        svg+=['</g></svg>'];(self.directory/'map.svg').write_text('\n'.join(svg)+'\n')
        result={'run_id':self.run_id,'phase':self.phase,'reason':self.reason,'identity':identity(self.record),
                'start_sim':self.start_sim,'end_sim':self.sim,'ground_truth':self.gt,'door_joint':self.door,
                'odom':self.odom,'front_m':self.front,'point_count':self.point_count,
                'projection_rejected':self.projection_rejected,'bumper_contacts':self.bumper_count,
                'command':list(self.commanded),'transitions':self.events,'directory':str(self.directory),
                'projection':'actual scan + preceding actual odom <=120ms; sensor x=.15; not SLAM'}
        temporary=self.directory/'result.tmp';temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');temporary.replace(self.directory/'result.json')
        (self.root/'build/latest_run.json').write_text(json.dumps({'run_id':self.run_id,'directory':str(self.directory)})+'\n')
        self.finished=True

    def run(self):
        def stop(*_): self.stop_event.set()
        signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
        try:
            while not self.rospy.is_shutdown() and not self.stop_event.is_set():
                try: self.tick()
                except Exception as error:
                    with self.lock:
                        self.transition('FAILED','Controller exception: '+str(error))
                        if not self.finished: self.finish()
                self.stop_event.wait(.025)
        finally:
            for _ in range(4): self.command();time.sleep(.02)
            with self.lock:
                if not self.finished:
                    if self.phase in self.active: self.transition('FAILED','Controller shutdown')
                    self.finish()


if __name__=='__main__': Mission().run()
