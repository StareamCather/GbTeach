#!/usr/bin/env python3
"""Real SystemPlugin lifecycle, native physics state, and finite world controls."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from config import DT, INITIAL_Z, parameters, prepared


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


def z_of(model):return float(model['pose']['position'].get('z',0))


def predicted_z(gravity,integrations):
    """Labelled ODE free-fall estimate, never substituted for measurements."""
    return INITIAL_Z-gravity*DT*DT*integrations*(integrations+1)/2


class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve()
        self.binary=self.workspace/'build/system_observer'

    def client(self,mode,*args,timeout=8):
        argv=[str(self.binary),mode]+list(args)
        context='mode='+mode+', argv='+json.dumps(argv,ensure_ascii=False)
        process=subprocess.run(argv,text=True,capture_output=True,timeout=timeout)
        rows=decode(process.stdout)
        require(process.returncode==0,'Actual native client failed ('+context+', returncode='+str(process.returncode)+'): '+process.stdout[-1800:]+' '+process.stderr[-1200:])
        require(rows and all(row.get('source') not in ('client_error','decode_error') for row in rows),'Native client returned no usable actual result ('+context+')')
        return rows[-1]

    def stats(self):return self.client('stats')['data']
    def snapshot(self):return self.client('snapshot')['payload']
    def entity(self):return self.client('entity','--name','falling_cube')['data']
    def physics(self):return self.client('physics')['data']

    def ready(self):
        self.settings,self.record=prepared();state=self.snapshot();stats=self.stats();physics=self.physics()
        require(state['world']=='lesson31' and state['plugin_type']=='SystemPlugin' and state['scope']=='gzserver_process','Wrong real SystemPlugin/world')
        require(state['load_count']==state['init_count']==state['world_created_count']==1,'Actual Load/Init/worldCreated not exactly once')
        require(state['source_sha256']==self.record['source_sha256']['system_plugin.cc'] and state['config_sha256']==self.record['config_sha256'],'Actually loaded source/config differ; reload')
        require(state['gravity_m_s2']==self.settings['gravity_m_s2'] and state['record_every_steps']==self.settings['record_every_steps'],'Loaded parameters differ from the saved personal configuration')
        require(not state['last_error'],'Real plugin ownership/error: '+state['last_error'])
        hooks=state['lifecycle']
        require([row['hook'] for row in hooks[:3]]==['load','init','world_created'],'Real lifecycle order differs')
        require(not hooks[0]['world_available'] and not hooks[1]['world_available'] and hooks[2]['world_available'],'World binding was reported before worldCreated')
        if not state['updates']:
            require(not state['gravity_applied'] and state['last_prephysics'] is None,'Paused startup must not invent an update or gravity application')
            require(abs(float(physics['gravity']['z'])+9.8)<1e-9,'Initial native gravity differs from fixed SDF baseline')
        elif state['gravity_applied']:
            require(abs(float(physics['gravity']['z'])+self.settings['gravity_m_s2'])<1e-9,'Native physics gravity differs from plugin application')
        return {'ready':True,'state':state,'world_stats':stats,'native_physics':physics,'native_cube':self.entity(),
                'library_sha256':self.record['library_sha256'],
                'note':'Load/Init do not bind a world. An initially paused server has not run its first WorldUpdateBegin; gravity_applied remains false until real stepping.'}

    def pause(self,value):
        if bool(self.stats().get('paused',False))!=value:self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+5
        while True:
            actual=self.stats()
            if bool(actual.get('paused',False))==value:return actual
            require(time.monotonic()<deadline,'Actual pause did not complete; no command replayed')

    def step(self,count=100):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 real physics ticks')
        require(self.stats().get('paused',False),'Pause before stepping')
        before=self.stats();plugin_before=self.snapshot();start=ns(before['sim_time'])
        sent=self.client('control','--steps',str(count));deadline=time.monotonic()+6
        while True:
            after=self.stats();current=self.snapshot();delta=ns(after['sim_time'])-start
            if delta>=count*1000000 and current['updates']>=plugin_before['updates']+count:break
            require(time.monotonic()<deadline,'Actual ticks not completed; non-idempotent pulse not replayed')
        require(delta==count*1000000 and after['paused'],'Actual step time/pause differ')
        require(current['updates']==plugin_before['updates']+count and current['epoch']==plugin_before['epoch'],'Actual WorldUpdateBegin count/epoch differs')
        require(current['epoch_updates']==plugin_before['epoch_updates']+count,'Actual epoch update count differs')
        stride=self.settings['record_every_steps']
        expected=current['epoch_updates']//stride-plugin_before['epoch_updates']//stride
        require(current['record_count']-plugin_before['record_count']==expected,'Real record stride count differs')
        require(current['gravity_applied'],'First real callback did not apply gravity')
        physics=self.physics();cube=self.entity()
        require(abs(float(physics['gravity']['z'])+self.settings['gravity_m_s2'])<1e-9,'Independent native physics_info gravity differs')
        require(current['last_prephysics']['sim_time_ns']==ns(after['sim_time']),'Last prephysics callback source time differs from completed world step')
        return {'passed':True,'request':sent,'before':before,'after':after,'delta_sim_time_ns':delta,
                'plugin_before':plugin_before,'state':current,'native_physics':physics,'native_cube':cube,
                'note':'World time/iterations advance before WorldUpdateBegin. Its pose and velocity precede the integration for that labelled tick; native entity after stepping includes it.'}

    def run(self,seconds=.3):
        require(math.isfinite(seconds) and .001<=seconds<=1,'Run .001..1 simulated seconds')
        count=round(seconds/DT)
        require(abs(seconds-count*DT)<1e-9,'Use an integer number of 1ms physics ticks')
        self.ready();self.pause(True)
        result=self.step(count);result['requested_sim_seconds']=seconds
        return result

    def watch(self,seconds=2):
        require(math.isfinite(seconds) and .1<=seconds<=12,'Observe .1..12 wall seconds')
        path=self.workspace/'build'/('watch-'+uuid.uuid4().hex+'.jsonl')
        with path.open('w',encoding='utf-8') as stream:
            process=subprocess.run([str(self.binary),'watch','--seconds',str(seconds)],stdout=stream,stderr=subprocess.PIPE,text=True,timeout=seconds+8)
        require(process.returncode==0,'Actual watcher failed: '+process.stderr)
        return {'raw_events_file':str(path),'events':decode(path.read_text(encoding='utf-8')),
                'note':'Watching does not advance a paused world. Use another terminal for finite steps; output retains actual native pose/stats and custom callback measurements.'}

    def reset(self):
        self.ready();self.pause(True);before=self.snapshot();clock=self.stats();sent=self.client('control','--reset-all')
        deadline=time.monotonic()+6
        while True:
            actual=self.stats();cube=self.entity();after=self.snapshot()
            if ns(actual['sim_time'])==0 and abs(z_of(cube)-INITIAL_Z)<1e-7:break
            require(time.monotonic()<deadline,'Actual full reset not observed; non-idempotent reset not replayed')
        require(after['reset_count']==before['reset_count']==0,'SDK unexpectedly dispatched SystemPlugin.Reset')
        if before['last_prephysics'] is not None:
            require(after['world_reset_event_count']==before['world_reset_event_count']+1 and after['epoch']==before['epoch']+1,'Owned real clock rewind/reset event was not observed')
            require(after['epoch_updates']==after['epoch_record_count']==0 and not after['gravity_applied'],'Reset did not begin a fresh measured epoch')
        return {'passed':True,'request':sent,'before':before,'after':after,'world_before':clock,'world_after':actual,'native_cube':cube,
                'note':'World::Reset calls world/model plugins, not process g_plugins. This SystemPlugin.Reset count stays zero. A confirmed owned-world clock rewind is handled via the separate worldReset event.'}

    def verify(self):
        initial=self.ready();entry_pause=bool(initial['world_stats'].get('paused',False));self.pause(True)
        result={'passed':True,'initial':initial};process=None;identity=uuid.uuid4().hex
        path=self.workspace/'build'/('verify-'+identity+'.jsonl');stop=self.workspace/'build'/('stop-'+identity)
        try:
            if not initial['state']['updates']:
                first=self.step(1);result['first_step']=first
                require(first['state']['last_prephysics']['z_m']==INITIAL_Z,'First prephysics callback did not observe the original cube')
                require(abs(z_of(first['native_cube'])-predicted_z(self.settings['gravity_m_s2'],1))<2e-6,'First integration did not use plugin gravity')
            result['reset_before_fall']=self.reset();phases=[];total=0
            with path.open('w',encoding='utf-8') as stream:
                process=subprocess.Popen([str(self.binary),'watch','--seconds','90','--stop-file',str(stop)],stdout=stream,stderr=subprocess.PIPE,text=True)
                deadline=time.monotonic()+5
                while 'client_ready' not in path.read_text(encoding='utf-8'):
                    require(process.poll() is None and time.monotonic()<deadline,'Native watcher did not connect');time.sleep(.02)
                # Last 199->200 step independently pairs the previous native
                # postphysics pose with the next prephysics callback reading.
                for count in (1,9,90,99,1):
                    phase=self.step(count);total+=count;state=phase['state'];g=self.settings['gravity_m_s2']
                    estimate=predicted_z(g,total);pre_estimate=predicted_z(g,total-1)
                    require(abs(z_of(phase['native_cube'])-estimate)<3e-6,'Native falling body differs from labelled ODE free-fall estimate')
                    require(abs(state['last_prephysics']['z_m']-pre_estimate)<3e-6,'Actual prephysics sample was confused with postphysics pose')
                    require(abs(state['last_prephysics']['vz_m_s']+g*DT*(total-1))<3e-6,'Actual prephysics velocity differs from integration history')
                    phase['prediction']={'label':'ODE semi-implicit free-fall estimate, separate from all measured fields','integrations':total,
                                         'postphysics_z_m':estimate,'prephysics_z_m':pre_estimate}
                    phases.append(phase)
                require(abs(phases[-1]['state']['last_prephysics']['z_m']-z_of(phases[-2]['native_cube']))<1e-12,'199-step native pose differs from 200th prephysics callback')
                state=phases[-1]['state'];stride=self.settings['record_every_steps'];samples=state['recent_records']
                expected_count=200//stride
                require(state['epoch_record_count']==expected_count,'Real cumulative epoch record count differs')
                require(len(samples)==min(expected_count,128),'Bounded real record window has the wrong length')
                require([s['epoch_updates'] for s in samples]==list(range(stride,201,stride))[-128:],'Record window was not based on the last real stride callbacks')
                for sample in samples:
                    n=sample['epoch_updates']
                    require(sample['sim_time_ns']==n*1000000,'Prephysics source timestamp was shifted to match its pose')
                    require(abs(sample['z_m']-predicted_z(self.settings['gravity_m_s2'],n-1))<3e-6,'Recorded actual pose differs from its prephysics context')
                before=self.stats();s=self.snapshot();time.sleep(.15);after=self.stats();t=self.snapshot()
                require(ns(before['sim_time'])==ns(after['sim_time']) and s['updates']==t['updates'] and s['record_count']==t['record_count'],'Paused world falsely accumulated callbacks or samples')
                stop.touch();_,stderr=process.communicate(timeout=8)
                require(process.returncode==0,'Native watcher failed normal cleanup: '+stderr)
            events=decode(path.read_text(encoding='utf-8'))
            require(all(e.get('source') not in ('client_error','decode_error') for e in events),'Raw wire capture contained client/decode errors')
            require(any(e.get('source')=='plugin' for e in events) and any(e.get('source')=='pose' for e in events),'Missing real custom/native wire evidence')
            result.update(phases=phases,samples=samples,pause_observation={'world_before':before,'world_after':after,'state_before':s,'state_after':t},
                          raw_events_file=str(path),events=events,reset_after_fall=self.reset(),
                          policy='Only entry pause is restored. Physics history is real; verification performs explicit full world resets, never replaces measurements with predictions.')
        finally:
            stop.touch()
            if process and process.poll() is None:
                try:process.communicate(timeout=8)
                except subprocess.TimeoutExpired:process.terminate();process.communicate(timeout=3)
            result['final_world']=self.pause(entry_pause)
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('ready','status','predict','run','watch','pause','resume','step','reset','verify'))
    parser.add_argument('--seconds',type=float);parser.add_argument('--steps',type=int,default=100);args=parser.parse_args()
    if args.action=='predict':
        settings=parameters();g=settings['gravity_m_s2']
        result={'label':'Predictions only; no native measurements','parameters':settings,'physics_dt_s':DT,
                'rule':'ODE semi-implicit free fall: v_n=-g*n*dt; z_n=2-g*dt^2*n*(n+1)/2, before floor contact',
                'step200_postphysics_z_m':predicted_z(g,200),'callback200_prephysics_z_m':predicted_z(g,199),
                'callback200_source_time_ns':200000000,'records_in_200_updates':200//settings['record_every_steps'],
                'clock':'World increments time/iterations before prephysics callback, then integrates physics.'}
    else:
        lab=Lab()
        if args.action in ('ready','status'):result=lab.ready()
        elif args.action in ('pause','resume'):result={'world_stats':lab.pause(args.action=='pause')}
        elif args.action=='step':result=lab.step(args.steps)
        elif args.action=='watch':result=lab.watch(2 if args.seconds is None else args.seconds)
        elif args.action=='run':result=lab.run(.3 if args.seconds is None else args.seconds)
        else:result=getattr(lab,args.action)()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try:main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
