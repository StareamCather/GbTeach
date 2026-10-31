#!/usr/bin/env python3
"""Native factory observations, asynchronous ownership and explicit cleanup."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from config import GRAVITY, GROUND_TOP, BLOCK_SIDE, parameters, prepared


def require(value,message):
    if not value:raise RuntimeError(message)


def source_time(value):
    return int(value.get('sec',0))+int(value.get('nsec',0))*1e-9


def decode_lines(text):
    rows=[]
    for line in text.splitlines():
        try:value=json.loads(line)
        except ValueError:continue
        if isinstance(value,dict):rows.append(value)
    return rows


def native_stability(events, identities):
    """Require a genuine source-time rest window for every actual model ID."""
    result=[];target=GROUND_TOP+BLOCK_SIDE/2
    for identity in identities:
        samples=[];duplicates=0
        for event in events:
            if event.get('source')!='pose':continue
            stamp=source_time(event['data']['time'])
            for pose in event['data'].get('pose',[]):
                if int(pose.get('id',-1))!=identity:continue
                if samples and stamp<=samples[-1]['time_s']:
                    duplicates+=1;continue
                samples.append({'time_s':stamp,'z':float(pose['position']['z'])})
        if len(samples)<8:return None
        tail=[s for s in samples if s['time_s']>=samples[-1]['time_s']-.25]
        if len(tail)<8 or tail[-1]['time_s']-tail[0]['time_s']<.2:return None
        rates=[(b['z']-a['z'])/(b['time_s']-a['time_s']) for a,b in zip(tail,tail[1:])]
        spread=max(s['z'] for s in tail)-min(s['z'] for s in tail)
        speed=max(abs(v) for v in rates)
        if spread>.003 or speed>.03 or any(abs(s['z']-target)>.008 for s in tail):return None
        result.append({'id':identity,'source_window_s':[tail[0]['time_s'],tail[-1]['time_s']],
                       'native_samples':tail,'z_range_m':spread,'maximum_fd_vertical_speed_m_s':speed,
                       'duplicates_retained_in_raw':duplicates})
    return result


class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve()
        self.binary=self.workspace/'build/world_observer'

    def client(self,mode,*args,timeout=8):
        process=subprocess.run([str(self.binary),mode]+list(args),text=True,capture_output=True,timeout=timeout)
        rows=decode_lines(process.stdout)
        require(process.returncode==0,'Actual native client failed: '+process.stdout[-1800:]+' '+process.stderr[-1000:])
        require(rows,'Native client returned no actual data')
        return rows[-1]

    def snapshot(self):return self.client('snapshot')['payload']
    def stats(self):return self.client('stats')['data']
    def scene(self):return self.client('world')['data']
    def entity(self,name):return self.client('entity','--name',name)['data']
    def command(self,action):return self.client('command','--action',action)['payload']

    def pause(self,value):
        before=self.stats()
        if bool(before.get('paused',False))!=value:self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+5
        while True:
            current=self.stats()
            if bool(current.get('paused',False))==value:return current
            require(time.monotonic()<deadline,'Native pause did not complete; no control replayed')

    def ready(self):
        self.settings,self.record=prepared();state=self.snapshot()
        require(state['world']=='lesson27','Wrong native world')
        require(state['load_count']==state['init_count']==1,'Actual WorldPlugin Load/Init not exactly once')
        require(state['source_sha256']==self.record['source_sha256']['world_plugin.cc'],'Actually loaded plugin source differs')
        require(state['config_sha256']==self.record['config_sha256'],'Actual cached SDF inputs differ; reload')
        require(state['session_prefix']==self.record['session_prefix'],'Wrong private factory instance')
        require(state['count']==self.settings['count'] and abs(state['drop_height']-self.settings['drop_height'])<1e-12,'Stale actual plugin inputs')
        require(not state['last_error'],'WorldPlugin error: '+state['last_error'])
        return {'ready':True,'state':state,'world_stats':self.stats(),'native_scene':self.scene(),
                'native_sentinel':self.entity('external_sentinel'),'library_sha256':self.record['library_sha256']}

    def step(self,count=100):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 physics ticks')
        require(bool(self.stats().get('paused',False)),'Pause before stepping')
        before=self.snapshot();initial=self.stats();start=source_time(initial['sim_time'])
        self.client('control','--steps',str(count));deadline=time.monotonic()+6
        while True:
            after=self.stats();delta=source_time(after['sim_time'])-start
            if delta>=count*.001-1e-8:break
            require(time.monotonic()<deadline,'Native ticks not completed; pulse not replayed')
        state=self.snapshot()
        require(abs(delta-count*.001)<1e-8 and after['paused'],'Actual step delta or pause differs')
        require(state['updates']-before['updates']==count,'WorldPlugin update count differs from native ticks')
        return {'passed':True,'before':before,'after':state,'world_before':initial,'world_after':after,'delta_sim_time_s':delta}

    def wait_phase(self,phase,seconds=12):
        deadline=time.monotonic()+seconds
        while True:
            state=self.snapshot()
            require(not state['last_error'],'Actual WorldPlugin error: '+state['last_error'])
            if state['phase']==phase:return state
            require(time.monotonic()<deadline,'Actual phase '+phase+' not observed; request not replayed')
            time.sleep(.04)

    def clear(self):
        self.ready();entry=self.stats();pause=bool(entry.get('paused',False));before=self.snapshot()
        sentinel_before=self.entity('external_sentinel');result={}
        try:
            accepted=self.command('clear');require(accepted['accepted'],'Clear rejected')
            self.pause(False);after=self.wait_phase('idle')
            self.pause(True);scene=self.scene()
            names={m['name'] for m in scene.get('model',[])}
            require(not any(m['name'] in names for m in before['models']),'Owned native entity survived clear')
            require(not any(n.startswith(self.record['session_prefix']) for n in names),'Late private factory entity survived clear')
            sentinel_after=self.entity('external_sentinel')
            require(sentinel_before==sentinel_after,'External sentinel changed during scoped clear')
            result={'passed':True,'before':before,'after':after,'accepted':accepted,'native_scene':scene,
                    'sentinel_before':sentinel_before,'sentinel_after':sentinel_after,'entry_world':entry,
                    'policy':'Only registered private IDs are deleted after pending factories appear; clock and external dynamics are not rewound.'}
        finally:result['final_world']=self.pause(pause)
        return result

    def watch(self,seconds):
        require(math.isfinite(seconds) and .1<=seconds<=12,'Watch .1..12 wall seconds')
        path=self.workspace/'build'/('watch-'+uuid.uuid4().hex+'.jsonl')
        with path.open('w',encoding='utf-8') as stream:
            p=subprocess.run([str(self.binary),'watch','--seconds',str(seconds)],stdout=stream,stderr=subprocess.PIPE,text=True,timeout=seconds+7)
        require(p.returncode==0,'Native observer failed: '+p.stderr)
        return {'raw_events_file':str(path),'events':decode_lines(path.read_text(encoding='utf-8'))}

    def batch(self):
        self.ready();entry=self.stats();pause=bool(entry.get('paused',False));before=self.snapshot()
        require(before['phase']=='idle' and not before['models'],'Clear previous batch before starting another')
        path=self.workspace/'build'/('batch-'+uuid.uuid4().hex+'.jsonl');process=None;result={}
        try:
            self.pause(True)
            with path.open('w',encoding='utf-8') as stream:
                process=subprocess.Popen([str(self.binary),'watch','--seconds','8'],stdout=stream,stderr=subprocess.PIPE,text=True)
                deadline=time.monotonic()+4
                while 'client_ready' not in path.read_text(encoding='utf-8'):
                    require(process.poll() is None and time.monotonic()<deadline,'Native watcher not ready')
                    time.sleep(.03)
                accepted=self.command('batch');require(accepted['accepted'],'Batch rejected')
                self.pause(False);after=self.wait_phase('complete');deadline=time.monotonic()+12
                target=GROUND_TOP+BLOCK_SIDE/2
                identities=[]
                entities=[self.entity(item['name']) for item in after['models']]
                for item,entity in zip(after['models'],entities):
                    require(int(entity['id'])==item['id'],'Actual native factory identity differs')
                    identities.append(int(entity['id']))
                while True:
                    stability=native_stability(decode_lines(path.read_text(encoding='utf-8')),identities)
                    if stability:break
                    require(process.poll() is None and time.monotonic()<deadline,'No actual native stable landing window')
                    time.sleep(.04)
                self.pause(True);after=self.snapshot();entities=[self.entity(item['name']) for item in after['models']]
                require(all(abs(float(item['pose']['position']['z'])-target)<.008 for item in entities),
                        'Native paused final centres differ from the observed landing window')
                _,error=process.communicate(timeout=10)
                require(process.returncode==0,'Native watcher failed: '+error)
            events=decode_lines(path.read_text(encoding='utf-8'));falls=[]
            for item in after['models']:
                samples=[]
                for e in events:
                    if e.get('source')!='pose':continue
                    for pose in e['data'].get('pose',[]):
                        if int(pose.get('id',-1))==item['id']:
                            samples.append({'time_s':source_time(e['data']['time']),'z':float(pose['position']['z'])})
                require(len(samples)>=8,'Too few independent native model poses')
                require(max(s['z'] for s in samples)-min(s['z'] for s in samples)>.3,'No actual native falling displacement')
                falls.append({'name':item['name'],'id':item['id'],'native_samples':samples,'drop_observed_m':max(s['z'] for s in samples)-min(s['z'] for s in samples)})
            result={'passed':True,'before':before,'after':after,'accepted':accepted,'entities':entities,'native_falls':falls,
                    'native_stability':stability,
                    'events':events,'raw_events_file':str(path),'entry_world':entry,
                    'policy':'Batch remains visible; original pause is restored. Use clear to reclaim registered IDs, reload for a pristine world.'}
        finally:
            if process and process.poll() is None:process.terminate();process.communicate(timeout=3)
            result['final_world']=self.pause(pause)
        return result

    def model_reset(self):
        self.pause(True);before=self.snapshot();native_before=self.stats()
        self.client('control','--reset-models');deadline=time.monotonic()+5
        while True:
            native_after=self.stats()
            if int(native_after.get('iterations',0))>int(native_before.get('iterations',0)):break
            require(time.monotonic()<deadline,'Native model-reset management tick not observed; request not replayed')
        after=self.snapshot()
        require(after['reset_count']==before['reset_count'],'Model-only reset unexpectedly called WorldPlugin::Reset')
        delta=source_time(native_after['sim_time'])-source_time(native_before['sim_time'])
        require(abs(delta-.001)<1e-8,'Model-only reset did not preserve clock apart from its native management tick')
        entities=[]
        for item in before['models']:
            if not item['appeared'] or item['removed']:continue
            entity=self.entity(item['name'])
            require(int(entity['id'])==item['id'],'Model reset entity ID changed')
            require(abs(float(entity['pose']['position']['z'])-self.record['factory_expected_drop_height'])<1e-7,
                    'Actual model-only reset did not restore the spawned initial model height')
            entities.append(entity)
        return {'before':before,'after':after,'world_before':native_before,'world_after':native_after,
                'native_entities_after':entities,'reset_clock_delta_s':delta,
                'requested_drop_height':self.settings['drop_height'],
                'factory_expected_drop_height':self.record['factory_expected_drop_height'],
                'native_scene':self.scene(),'note':'Model entities return to their initial model poses; WorldPlugin::Reset is not invoked. Remains paused.'}

    def reset(self):
        self.ready();entry=self.stats();pause=bool(entry.get('paused',False));self.pause(True)
        before=self.snapshot();native_before=self.stats();result={}
        try:
            self.client('control','--reset-all');deadline=time.monotonic()+5
            while True:
                after=self.snapshot()
                if after['reset_count']==before['reset_count']+1:break
                require(time.monotonic()<deadline,'Full reset hook not observed; reset not replayed')
            native_after=self.stats()
            require(after['load_count']==before['load_count'] and after['init_count']==before['init_count'],'Reset reran Load/Init')
            require(source_time(native_after['sim_time'])<=.001+1e-8,'Full reset did not return native clock to zero')
            self.pause(False);clean=self.wait_phase('idle');self.pause(True)
            scene=self.scene();names=[m['name'] for m in scene.get('model',[])]
            require(not any(n.startswith(self.record['session_prefix']) for n in names),'Private entities survived full-reset cleanup')
            result={'passed':True,'before':before,'after':after,'cleaned':clean,'world_before':native_before,'world_after_reset':native_after,
                    'native_scene':scene,'note':'Full world reset rewinds clock and calls WorldPlugin::Reset; deferred cleanup requires real updates and changes external dynamics.'}
        finally:result['final_world']=self.pause(pause)
        return result

    def verify(self):
        pause=bool(self.stats().get('paused',False));result={}
        try:
            self.clear();motion=self.batch();self.pause(True);before=self.snapshot();clock=self.stats();time.sleep(.15)
            after=self.snapshot();require(before['updates']==after['updates'] and self.stats()['sim_time']==clock['sim_time'],'Paused world advanced')
            stepped=self.step(100);model=self.model_reset();full=self.reset()
            result={'passed':True,'motion':motion,'steps':stepped,'model_only_reset':model,'full_reset':full,
                    'note':'Actual batches/physics/lifecycle checked; ends with own models removed, entry pause restored. Clock and external dynamics are not snapshots.'}
        finally:
            result['cleanup']=self.clear();result['final_world']=self.pause(pause)
        return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('ready','status','predict','batch','clear','watch','pause','resume','step','reset','model-reset','verify'))
    p.add_argument('--seconds',type=float,default=2);p.add_argument('--steps',type=int,default=100);a=p.parse_args()
    if a.action=='predict':
        values=parameters();factory_height=float(format(values['drop_height'],'.6g'))
        result={'parameters':values,'requested_drop_height_m':values['drop_height'],
                'sdk9_factory_expected_drop_height_m':factory_height,
                'ground_top_m':GROUND_TOP,'settled_centre_z_m':GROUND_TOP+BLOCK_SIDE/2,
                'ideal_requested_free_fall_seconds':math.sqrt(2*(values['drop_height']-GROUND_TOP-BLOCK_SIDE/2)/GRAVITY),
                'free_fall_seconds':math.sqrt(2*(factory_height-GROUND_TOP-BLOCK_SIDE/2)/GRAVITY),
                'request_interval_s':.35,'note':'Actual insertion is asynchronous; use native first samples, not request time, for physical comparisons.'}
    else:
        lab=Lab()
        if a.action in ('ready','status'):result=lab.ready()
        elif a.action=='watch':result=lab.watch(a.seconds)
        elif a.action=='step':result=lab.step(a.steps)
        elif a.action in ('pause','resume'):result={'world_stats':lab.pause(a.action=='pause')}
        else:result=getattr(lab,a.action.replace('-','_'))()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try:main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
