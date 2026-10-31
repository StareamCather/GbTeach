#!/usr/bin/env python3
"""Actual RaySensor messages, transparent filtering, and bounded scene controls."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from config import parameters,prepared

def require(condition,message):
    if not condition:raise RuntimeError(message)

def decode(text):
    rows=[]
    for line in text.splitlines():
        try:value=json.loads(line)
        except ValueError:continue
        if isinstance(value,dict):rows.append(value)
    return rows

def ns(value):return int(value.get('sec',0))*1000000000+int(value.get('nsec',0))

def paired_analysis(events,settings):
    scans={};pairs=[];previous=None;gaps=[]
    for row in events:
        require(row.get('source') not in ('client_error','decode_error'),'Native wire reported an error')
        if row.get('source')=='scan':
            t=ns(row['data']['time']);scans.setdefault(t,[]).append(row)
    for row in events:
        if row.get('source')!='plugin':continue
        value=row['payload'];t=int(value['source_time_ns'])
        if not value['sample_available'] or t not in scans:continue
        raw=scans[t][-1];ranges=[float(v) for v in raw['data']['scan']['ranges']]
        valid=[v for v in ranges if math.isfinite(v) and .1<=v<6]
        require(value['valid_count']==len(valid),'Custom valid count differs from the original native scan')
        require(value['raw_kinds']==['nan' if math.isnan(v) else 'positive_infinity' if v==math.inf else 'negative_infinity' if v==-math.inf else 'finite' for v in ranges],
                'Custom classification lost a native nonfinite range')
        for native,reported in zip(ranges,value['raw_ranges_m']):
            require((not math.isfinite(native) and reported is None) or (math.isfinite(native) and abs(native-reported)<1e-9),'Custom raw range differs from native message')
        if previous and value['callback_count']==previous['callback_count']+1 and value['epoch']==previous['epoch']:
            # Invalid input clears the filter before publishing this callback.
            # Only a current valid sample may carry a previous valid EMA;
            # recovery after an invalid sample must start from a fresh seed.
            if valid:
                expected_old=previous['filtered_m'] if previous['valid_count'] else None
                require(value['previous_filtered_m']==expected_old,'EMA continuity differs from previous actual callback')
        elif previous:gaps.append([previous['callback_count'],value['callback_count']])
        if valid:
            nearest=min(valid);old=value['previous_filtered_m']
            expected=nearest if old is None else settings['alpha']*nearest+(1-settings['alpha'])*old
            require(abs(value['nearest_m']-nearest)<1e-9 and abs(value['filtered_m']-expected)<1e-9,'Actual custom EMA recurrence differs')
            expected_alarm='near' if expected<=settings['threshold_m'] else 'clear'
            require(value['alarm_state']==expected_alarm,'Threshold decision differs from actual filtered distance')
        else:
            require(value['nearest_m'] is None and value['previous_filtered_m'] is None and value['filtered_m'] is None and value['alarm_state']=='unknown','No return must clear EMA and remain unknown')
        pairs.append({'source_time_ns':t,'native_scan':raw,'plugin':row});previous=value
    require(len(pairs)>=30,'Too few time-paired native/custom measurements')
    return {'paired_samples':len(pairs),'callback_gaps':gaps,'pairs':pairs,
            'note':'Native LaserScanStamped supplies physical ranges; custom state is checked against them without changing source timestamps.'}

class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve();self.binary=self.workspace/'build/sensor_observer'

    def client(self,mode,*args,timeout=8):
        p=subprocess.run([str(self.binary),mode]+list(args),text=True,capture_output=True,timeout=timeout)
        rows=decode(p.stdout)
        require(p.returncode==0,'Actual native client failed: '+p.stdout[-1800:]+' '+p.stderr[-1200:])
        require(rows,'Native client returned no actual result')
        return rows[-1]
    def stats(self):return self.client('stats')['data']
    def snapshot(self):return self.client('snapshot')['payload']
    def entity(self):return self.client('entity','--name','target_box')['data']

    def ready(self):
        self.settings,self.record=prepared();s=self.snapshot()
        require(s['world']=='lesson28' and s['sensor_scoped_name']=='lesson28::scanner::link::proximity','Wrong actual world/sensor')
        require(s['load_count']==s['init_count']==1,'Actual SensorPlugin Load/Init not exactly once')
        require(s['source_sha256']==self.record['source_sha256']['sensor_plugin.cc'] and s['config_sha256']==self.record['config_sha256'],'Actually loaded source/config differ')
        require(abs(s['threshold_m']-self.settings['threshold_m'])<1e-12 and abs(s['alpha']-self.settings['alpha'])<1e-12,'Actually cached plugin parameters differ; reload')
        return {'ready':True,'state':s,'native_target':self.entity(),'world_stats':self.stats(),'library_sha256':self.record['library_sha256']}

    def pause(self,value):
        if bool(self.stats().get('paused',False))!=value:self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+5
        while True:
            actual=self.stats()
            if bool(actual.get('paused',False))==value:return actual
            require(time.monotonic()<deadline,'Actual pause did not complete; no command replayed')

    def move(self,x=None,pose=None):
        self.ready();self.pause(True)
        if pose is None:
            require(type(x) in (int,float) and math.isfinite(x) and .5<=x<=9,'Target centre x must be [.5,9] metres')
            pose={'position':{'x':x,'y':0,'z':.4},'orientation':{'x':0,'y':0,'z':0,'w':1}}
        request=self.client('move','--name','target_box','--pose-json',json.dumps(pose,separators=(',',':')))
        deadline=time.monotonic()+5
        while True:
            entity=self.entity();actual=entity['pose']
            if all(abs(float(actual[part].get(key,0))-float(value))<1e-7 for part in ('position','orientation') for key,value in pose[part].items()):break
            require(time.monotonic()<deadline,'Actual target pose was not applied; move not replayed')
        return {'request':request,'native_target':entity,'world_stats':self.stats(),'note':'Real model pose changed while paused. Resume to obtain fresh sensor measurements.'}

    def step(self,count=100):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 physics ticks')
        require(self.stats().get('paused',False),'Pause before stepping')
        before=self.stats();start=ns(before['sim_time']);self.client('control','--steps',str(count));deadline=time.monotonic()+5
        while True:
            after=self.stats();delta=ns(after['sim_time'])-start
            if delta>=count*1000000:break
            require(time.monotonic()<deadline,'Actual ticks not completed; pulse not replayed')
        require(delta==count*1000000 and after['paused'],'Actual tick count/time/pause differ')
        return {'passed':True,'before':before,'after':after,'delta_sim_time_ns':delta,'sensor_state':self.snapshot(),
                'note':'Sensor callbacks occur at their own 20Hz schedule, not once per 1kHz physics tick.'}

    def watch(self,seconds=2):
        require(math.isfinite(seconds) and .1<=seconds<=12,'Observe .1..12 wall seconds')
        path=self.workspace/'build'/('watch-'+uuid.uuid4().hex+'.jsonl')
        with path.open('w',encoding='utf-8') as stream:
            p=subprocess.run([str(self.binary),'watch','--seconds',str(seconds)],stdout=stream,stderr=subprocess.PIPE,text=True,timeout=seconds+8)
        require(p.returncode==0,'Actual watcher failed: '+p.stderr)
        return {'raw_events_file':str(path),'events':decode(path.read_text(encoding='utf-8'))}

    def demo(self):
        self.ready();entry=self.stats();original=self.entity()['pose'];pause=bool(entry.get('paused',False));process=None;result={}
        identity=uuid.uuid4().hex;path=self.workspace/'build'/('demo-'+identity+'.jsonl');stop=self.workspace/'build'/('stop-'+identity)
        try:
            self.pause(True)
            with path.open('w',encoding='utf-8') as stream:
                process=subprocess.Popen([str(self.binary),'watch','--seconds','75','--stop-file',str(stop)],stdout=stream,stderr=subprocess.PIPE,text=True)
                deadline=time.monotonic()+5
                while 'client_ready' not in path.read_text(encoding='utf-8'):
                    require(process.poll() is None and time.monotonic()<deadline,'Native watcher not connected');time.sleep(.02)
                phases=[]
                for name,x,expected_valid in [('home',1.4,3),('near',.8,3),('far',3.2,1),('noecho',8,0),('recovery',1.4,3)]:
                    moved=self.move(x);before=self.stats();start=ns(before['sim_time']);self.pause(False);deadline=time.monotonic()+10
                    while True:
                        current=self.stats()
                        if ns(current['sim_time'])>=start+1000000000:break
                        require(time.monotonic()<deadline,'Actual sensor phase did not advance')
                    after=self.pause(True)
                    phases.append({'name':name,'target_centre_x_m':x,'expected_valid_beams':expected_valid,'native_target':moved['native_target'],
                                   'start_source_ns':start,'end_source_ns':ns(after['sim_time']),'world_before':before,'world_after':after})
                stop.touch();_,stderr=process.communicate(timeout=8)
                require(process.returncode==0,'Native watcher did not exit normally: '+stderr)
            events=decode(path.read_text(encoding='utf-8'));analysis=paired_analysis(events,self.settings)
            for phase in phases:
                rows=[p for p in analysis['pairs'] if phase['start_source_ns']+100000000<int(p['source_time_ns'])<=phase['end_source_ns']]
                require(len(rows)>=8,'Too few actual paired samples in phase '+phase['name'])
                tail=rows[-5:]
                require(all(p['plugin']['payload']['valid_count']==phase['expected_valid_beams'] for p in tail),'Actual native beam geometry differs in '+phase['name'])
                if phase['expected_valid_beams']:
                    expected=phase['target_centre_x_m']-.2
                    require(all(abs(p['plugin']['payload']['nearest_m']-expected)<.015 for p in tail),'Actual nearest distance differs from target face')
                else:require(all(p['plugin']['payload']['alarm_state']=='unknown' for p in tail),'No echo was incorrectly labelled safe')
                phase.update(paired_samples=len(rows),tail=[p['plugin']['payload'] for p in tail])
            result={'passed':True,'phases':phases,'analysis':analysis,'raw_events_file':str(path),'events':events,
                    'policy':'Only the target pose and entry pause are restored. Sensor EMA/counters and simulated clock are real history, not rewound.'}
        finally:
            stop.touch()
            if process and process.poll() is None:
                try:process.communicate(timeout=8)
                except subprocess.TimeoutExpired:process.terminate();process.communicate(timeout=3)
            self.pause(True);result['restored_target']=self.move(pose=original);result['final_world']=self.pause(pause)
        return result

    def reset(self):
        self.ready();entry=self.stats();pause=bool(entry.get('paused',False));self.pause(True);before=self.snapshot();clock=self.stats();result={}
        try:
            self.client('control','--reset-all');deadline=time.monotonic()+5
            while True:
                after=self.stats()
                if ns(after['sim_time'])<=1000000:break
                require(time.monotonic()<deadline,'Actual full reset clock rewind not observed; request not replayed')
            self.pause(False);deadline=time.monotonic()+7
            while True:
                current=self.snapshot()
                if current['rewind_count']==before['rewind_count']+1:break
                require(time.monotonic()<deadline,'No fresh sensor callback detected source-time rewind')
            require(current['reset_count']==before['reset_count'] and current['load_count']==current['init_count']==1,'Unexpected SensorPlugin lifecycle on sampling-clock reset')
            result={'passed':True,'before':before,'after':current,'world_before':clock,'world_after_reset':after,
                    'note':'Full reset rewinds world/sampling clocks. This SDK does not call SensorPlugin.Reset; first regressed measurement starts a new EMA epoch.'}
        finally:result['final_world']=self.pause(pause)
        return result

    def verify(self):
        result=self.demo();self.pause(True);before=self.stats();s=self.snapshot();time.sleep(.2);after=self.stats();t=self.snapshot()
        require(ns(before['sim_time'])==ns(after['sim_time']),'Paused world advanced')
        # A queued sensor callback may complete just after pause. Sample time
        # cannot exceed the frozen world; do not pretend asynchronous work vanishes.
        require(int(t['source_time_ns'])<=ns(after['sim_time']),'Sensor measurement came from the future')
        result['pause_observation']={'world_before':before,'world_after':after,'sensor_before':s,'sensor_after':t}
        result['steps']=self.step(100)
        result['final_world']=self.pause(bool(result['final_world'].get('paused',False)))
        return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('ready','status','predict','demo','watch','home','near','far','noecho','pause','resume','step','reset','verify'))
    p.add_argument('--seconds',type=float,default=2);p.add_argument('--steps',type=int,default=100);a=p.parse_args()
    if a.action=='predict':
        v=parameters();result={'parameters':v,'ema_rule':'first valid y=x; then y=alpha*x+(1-alpha)*previous; no valid => clear and unknown',
            'scan_rate_hz':20,'physics_rate_hz':1000,'home_nearest_m':1.2,'near_nearest_m':.6,'far_nearest_m':3,
            'home_valid_beams':3,'far_valid_beams':1,'decision':'filtered_m <= threshold_m => near; missing valid measurement => unknown'}
    else:
        lab=Lab()
        if a.action in ('ready','status'):result=lab.ready()
        elif a.action=='watch':result=lab.watch(a.seconds)
        elif a.action=='step':result=lab.step(a.steps)
        elif a.action in ('pause','resume'):result={'world_stats':lab.pause(a.action=='pause')}
        elif a.action in ('home','near','far','noecho'):result=lab.move({'home':1.4,'near':.8,'far':3.2,'noecho':8}[a.action])
        else:result=getattr(lab,a.action)()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))

if __name__=='__main__':
    try:main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
