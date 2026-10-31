#!/usr/bin/env python3
"""Real offline trials with owned processes and explicitly inclusive wall timing."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import time
import uuid
from config import DT_NS, WORLD, prepared, sha, world_bytes


def require(condition, message):
    if not condition: raise RuntimeError(message)


def decode(text):
    rows=[]
    for line in text.splitlines():
        try: value=json.loads(line)
        except ValueError: continue
        if isinstance(value,dict): rows.append(value)
    return rows


def ns(value): return int(value.get('sec',0))*1000000000+int(value.get('nsec',0))


def write_json(path, value):
    temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def positions(scene):
    return {m['name']:[float(m['pose']['position'].get(axis,0)) for axis in ('x','y','z')]
            for m in scene.get('model',[]) if m['name'].startswith('cube_')}


class OwnedTrial:
    """Fresh master/partition/home; signal only this UID+token+starttime."""
    ENV_KEY='GAZEBOTEACH_L34_SESSION'

    def __init__(self, root, world, seed):
        self.token=uuid.uuid4().hex; self.root=root; root.mkdir(); (root/'home').mkdir()
        self.world=root/'world.sdf'; self.world.write_bytes(world)
        self.processes=[]; self.streams=[]; self.seed=seed; self.previous_signal=None
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0)); self.port=listener.getsockname()[1]
        self.env=os.environ.copy(); self.env.update({
            self.ENV_KEY:self.token,'GAZEBO_MASTER_URI':'http://127.0.0.1:'+str(self.port),
            'GAZEBO_IP':'127.0.0.1','IGN_PARTITION':'gazeboteach-l34-'+self.token,
            'GZ_PARTITION':'gazeboteach-l34-'+self.token,'HOME':str(root/'home'),
            'GAZEBO_MODEL_DATABASE_URI':'','GAZEBO_PLUGIN_PATH':'','GAZEBO_MODEL_PATH':''})
        self.record={'token':self.token,'uid':os.getuid(),'master_uri':self.env['GAZEBO_MASTER_URI'],
                     'ign_partition':self.env['IGN_PARTITION'],'gz_partition':self.env['GZ_PARTITION'],
                     'world':str(self.world),'input_sha256':sha(self.world),'processes':[],'cleaned':False}
        write_json(root/'ownership.json',self.record)

    @classmethod
    def identity(cls,pid):
        try:
            path=Path('/proc')/str(pid)
            if path.stat().st_uid!=os.getuid(): return None
            environment=dict(item.split(b'=',1) for item in (path/'environ').read_bytes().split(b'\0') if b'=' in item)
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            return {'pid':pid,'uid':path.stat().st_uid,'start_ticks':stat[19],
                    'token':environment.get(cls.ENV_KEY.encode(),b'').decode(),
                    'exe':os.readlink(str(path/'exe')),
                    'cmdline':(path/'cmdline').read_bytes().replace(b'\0',b' ').decode('utf-8','replace')}
        except (OSError,ValueError): return None

    def owned(self):
        result=[]
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                value=self.identity(int(path.name))
                if value and value['token']==self.token: result.append(value)
        return result

    def spawn(self, command, label):
        stream=(self.root/(label+'.out')).open('w',encoding='utf-8'); self.streams.append(stream)
        process=subprocess.Popen(command,cwd=str(self.root),env=self.env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append(process); identity=self.identity(process.pid)
        require(identity and identity['token']==self.token,'Spawned trial identity not established')
        self.record['processes'].append(dict(identity,label=label,command=command))
        write_json(self.root/'ownership.json',self.record); return process

    def __enter__(self):
        def interrupted(signum,frame): raise KeyboardInterrupt('Bounded batch interrupted by signal '+str(signum))
        self.previous_signal=signal.signal(signal.SIGTERM,interrupted)
        try:
            self.server_spawn_monotonic_ns=time.monotonic_ns()
            self.server=self.spawn(['gzserver','--verbose','-u','--seed',str(self.seed),str(self.world)],'gzserver')
            return self
        except BaseException:
            self.__exit__(None,None,None); raise

    def __exit__(self,*unused):
        started=time.monotonic_ns()
        try:
            for sig,limit in ((signal.SIGTERM,5),(signal.SIGKILL,2)):
                for value in self.owned():
                    again=self.identity(value['pid'])
                    if again and again['token']==self.token and again['uid']==value['uid'] and again['start_ticks']==value['start_ticks']:
                        try: os.kill(value['pid'],sig)
                        except ProcessLookupError: pass
                deadline=time.monotonic()+limit
                while self.owned() and time.monotonic()<deadline:
                    for process in self.processes: process.poll()
                    time.sleep(.03)
            for process in self.processes:
                try: process.wait(timeout=1)
                except subprocess.TimeoutExpired: pass
            self.record['remaining']=self.owned(); self.record['cleaned']=not self.record['remaining']
            self.record['owned_at_cleanup']=[{'pid':p.pid,'returncode':p.poll()} for p in self.processes]
            self.record['cleanup_wall_s']=(time.monotonic_ns()-started)/1e9
            write_json(self.root/'ownership.json',self.record)
            require(self.record['cleaned'],'Owned trial descendants survived cleanup')
        finally:
            for stream in self.streams: stream.close()
            if self.previous_signal is not None: signal.signal(signal.SIGTERM,self.previous_signal)


class Lab:
    def __init__(self, env=None, event_file=None):
        self.workspace=Path.cwd().resolve(); self.settings,self.record=prepared(self.workspace)
        self.binary=self.workspace/'build/batch_observer'; self.env=env; self.event_file=event_file

    def client(self,mode,*args,timeout=8):
        started=time.monotonic_ns()
        process=subprocess.run([str(self.binary),mode]+list(args),env=self.env,text=True,capture_output=True,timeout=timeout)
        rows=decode(process.stdout)
        if self.event_file:
            with self.event_file.open('a',encoding='utf-8') as stream:
                stream.write(json.dumps({'mode':mode,'args':list(args),'started_monotonic_ns':started,
                    'finished_monotonic_ns':time.monotonic_ns(),'returncode':process.returncode,
                    'stdout':process.stdout,'stderr':process.stderr,'events':rows},allow_nan=False)+'\n')
        require(process.returncode==0,'Native client failed: '+process.stdout[-1500:]+' '+process.stderr[-1000:])
        require(rows and not any(r.get('source') in ('client_error','decode_error') for r in rows),'No valid actual native result')
        return rows[-1]

    def stats(self): return self.client('stats')['data']
    def physics(self): return self.client('physics')['data']
    def entity(self,name='cube_000'): return self.client('entity','--name',name)['data']

    def checkpoint(self,dt_ns=DT_NS):
        attempts=[]; deadline=time.monotonic()+12
        while True:
            before=self.stats(); world=self.client('world')['data']; after=self.stats()
            first=(ns(before['sim_time']),int(before.get('iterations',0)))
            last=(ns(after['sim_time']),int(after.get('iterations',0)))
            stable=first==last and before.get('paused') and after.get('paused')
            attempts.append({'before':list(first),'after':list(last),'stable':bool(stable)})
            if stable:
                require(world.get('name')==WORLD,'Wrong actual world')
                require(last[0]==last[1]*dt_ns,'Actual time/iterations inconsistent with fixed dt')
                return {'sim_time_ns':last[0],'iterations':last[1],'world_stats':after,
                        'native_world':world,'positions_m':positions(world),'consistency_attempts':attempts}
            require(time.monotonic()<deadline,'No coherent paused native checkpoint; control not replayed')

    def ready(self):
        physics=self.physics(); world=self.client('world')['data']; stats=self.stats()
        require(world.get('name')==WORLD and set(positions(world))=={'cube_%03d'%i for i in range(8)},'Actual preview grid differs')
        require(abs(float(physics['max_step_size'])-.001)<1e-12 and abs(float(physics['real_time_update_rate'])-1000)<1e-9,'Preview physics differs')
        return {'ready':True,'parameters':self.settings,'native_world':world,'native_physics':physics,'world_stats':stats,
                'note':'Preview has 8 cubes, 1ms dt, 1000Hz cap; batch owns separate servers and never modifies this world.'}

    def pause(self,value=True):
        if bool(self.stats().get('paused',False))!=value: self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+6
        while True:
            value_after=self.stats()
            if bool(value_after.get('paused',False))==value: return value_after
            require(time.monotonic()<deadline,'Pause incomplete; command not replayed')

    def step(self,count=100,dt_ns=DT_NS):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 actual ticks')
        before=self.checkpoint(dt_ns); start=time.monotonic_ns()
        sent=self.client('control','--steps',str(count)); deadline=time.monotonic()+8
        target=before['sim_time_ns']+count*dt_ns
        while True:
            actual=self.stats()
            if ns(actual['sim_time'])>=target: break
            require(time.monotonic()<deadline,'Finite ticks incomplete; one-shot pulse not repeated')
        after=self.checkpoint(dt_ns)
        require(after['sim_time_ns']==target and after['iterations']-before['iterations']==count,'Actual step delta differs')
        return {'passed':True,'request':sent,'before':before,'after':after,'steps':count,
                'control_start_monotonic_ns':start,'endpoint_monotonic_ns':time.monotonic_ns()}

    def reset(self):
        self.pause(True); sent=self.client('control','--reset-all'); deadline=time.monotonic()+12
        while True:
            after=self.checkpoint()
            if after['sim_time_ns']==0 and all(abs(p[2]-2)<1e-7 for p in after['positions_m'].values()):
                return {'passed':True,'request':sent,'after':after}
            require(time.monotonic()<deadline,'Full reset incomplete; command not replayed')

    def trial(self,batch_root,count,dt,repeat):
        name='n%d-dt%d-r%d'%(count,round(dt*1e6),repeat); root=batch_root/name
        trial=OwnedTrial(root,world_bytes(self.workspace/'world.sdf',count,dt,0),self.settings['seed'])
        row={'trial':name,'count':count,'dt_s':dt,'repeat':repeat,'seed':self.settings['seed'],
             'input_sha256':sha(trial.world),'directory':str(root),'passed':False}
        try:
            with trial:
                child=Lab(trial.env,root/'native-events.jsonl'); dt_ns=round(dt*1e9)
                deadline=time.monotonic()+18; readiness_errors=[]
                while True:
                    require(trial.server.poll() is None,'Actual gzserver exited before readiness')
                    try:
                        initial=child.checkpoint(dt_ns); physics=child.physics(); break
                    except (RuntimeError,subprocess.TimeoutExpired) as error:
                        readiness_errors.append(str(error))
                        require(time.monotonic()<deadline,'Native readiness failed: '+str(error))
                ready_ns=time.monotonic_ns()
                require(initial['sim_time_ns']==initial['iterations']==0,'Fresh child did not start paused at zero')
                names={'cube_%03d'%i for i in range(count)}
                require(set(initial['positions_m'])==names and len(initial['native_world'].get('model',[]))==count+1,'Native model count differs')
                require(all(abs(p[2]-2)<1e-9 for p in initial['positions_m'].values()),'Native initial cube heights differ')
                require(abs(float(physics['max_step_size'])-dt)<1e-12 and float(physics['real_time_update_rate'])==0,'Native physics inputs differ')
                first=child.step(round(.2/dt),dt_ns); second=child.step(round(.8/dt),dt_ns)
                final_ns=second['endpoint_monotonic_ns']; final=second['after']; falling=first['after']
                expected_z=2-9.8*dt*dt*round(.2/dt)*(round(.2/dt)+1)/2
                freefall_error=max(abs(p[2]-expected_z) for p in falling['positions_m'].values())
                landed_error=max(abs(p[2]-.1) for p in final['positions_m'].values())
                drift=max(math.hypot(p[0]-initial['positions_m'][n][0],p[1]-initial['positions_m'][n][1]) for n,p in final['positions_m'].items())
                require(set(falling['positions_m'])==set(final['positions_m'])==names,'Actual cubes disappeared')
                require(falling['sim_time_ns']==200000000 and final['sim_time_ns']==1000000000,'Actual physical endpoints differ')
                require(freefall_error<3e-6,'Native freefall differs from labelled semi-implicit ODE estimate')
                require(landed_error<.015 and drift<.01,'Actual cubes did not land on the floor')
                e2e=(final_ns-trial.server_spawn_monotonic_ns)/1e9
                row.update(passed=True,initial=initial,freefall=falling,final=final,native_physics=physics,
                    requests=[first['request'],second['request']],readiness_errors=readiness_errors,
                    prediction={'label':'ODE semi-implicit freefall prediction, not a measurement','freefall_z_m':expected_z},
                    freefall_max_error_m=freefall_error,landed_max_error_m=landed_error,xy_max_drift_m=drift,
                    server_spawn_monotonic_ns=trial.server_spawn_monotonic_ns,ready_monotonic_ns=ready_ns,
                    control_start_monotonic_ns=first['control_start_monotonic_ns'],endpoint_monotonic_ns=final_ns,
                    startup_readiness_wall_s=(ready_ns-trial.server_spawn_monotonic_ns)/1e9,
                    measured_control_wall_s=(final_ns-first['control_start_monotonic_ns'])/1e9,
                    end_to_end_wall_s=e2e,end_to_end_sim_throughput=1/e2e,
                    sim_seconds=1.0,iterations=final['iterations'])
        finally:
            row['ownership']=trial.record; row['cleanup_wall_s']=trial.record.get('cleanup_wall_s',0)
            write_json(root/'result.json',row)
        return row

    def batch(self):
        before_sha=sha(self.workspace/'world.sdf'); identity=uuid.uuid4().hex
        root=self.workspace/'build/batches'/identity; root.mkdir(parents=True)
        result={'passed':False,'batch_id':identity,'parameters':self.settings,'provenance':self.record,'trials':[],
                'report_file':str(root/'report.json'),'csv_file':str(root/'trials.csv'),
                'measurement':{'clock':'Python time.monotonic_ns in parent, actual measured wall time',
                    'end_to_end_wall_s':'before Popen(gzserver) to final coherent paused native world/stats endpoint; includes startup, readiness, helper process startup, native controls/queries and pauses between phases; excludes build and cleanup',
                    'startup_readiness_wall_s':'before server Popen to initial coherent zero-time checkpoint and actual physics_info returned',
                    'measured_control_wall_s':'before first control helper to final coherent endpoint; includes helpers, queries and interphase pause; not pure physics',
                    'end_to_end_sim_throughput':'1 simulated second / end_to_end_wall_s; application batch throughput, NOT engine RTF or CPU time',
                    'cleanup_wall_s':'owned child shutdown and confirmed cleanup after endpoint',
                    'native_real_time':'raw WorldStatistics only; may stay zero during paused multistep; never used as divisor',
                    'rate':'batch real_time_update_rate=0 disables throttling; finite ticks still require real computation',
                    'ordering':'no hardware performance ranking is asserted; repeated timing varies',
                    'rng':'--seed seeds Gazebo native RNG; scene layout is deterministic XML arithmetic, no Python random module'}}
        started=time.monotonic_ns()
        try:
            for count in self.settings['counts']:
                for dt in self.settings['step_sizes_s']:
                    for repeat in range(1,self.settings['repeats']+1):
                        result['trials'].append(self.trial(root,count,dt,repeat))
                        write_json(root/'report.json',result)
            comparisons=[]; summaries=[]
            for count in self.settings['counts']:
                for dt in self.settings['step_sizes_s']:
                    rows=[r for r in result['trials'] if r['count']==count and r['dt_s']==dt]
                    baseline=rows[0]; maximum=0
                    require(len({r['input_sha256'] for r in rows})==1,'Repeated SDF inputs differ')
                    for row in rows[1:]:
                        for phase in ('freefall','final'):
                            maximum=max(maximum,max(abs(p[a]-baseline[phase]['positions_m'][n][a]) for n,p in row[phase]['positions_m'].items() for a in range(3)))
                    require(maximum<1e-5,'Identical-input physical outcomes differ beyond 10um tolerance')
                    comparisons.append({'count':count,'dt_s':dt,'repeats':len(rows),'input_sha256':baseline['input_sha256'],
                                        'max_xyz_difference_m':maximum,'tolerance_m':1e-5,'comparison':'same-machine same-input sampled physical states; not cross-machine bitwise determinism'})
                    timings=[r['end_to_end_wall_s'] for r in rows]
                    summaries.append({'count':count,'dt_s':dt,'trials':len(rows),'wall_min_s':min(timings),'wall_median_s':statistics.median(timings),'wall_max_s':max(timings)})
            fields=['trial','count','dt_s','repeat','seed','input_sha256','sim_seconds','iterations','startup_readiness_wall_s',
                    'measured_control_wall_s','end_to_end_wall_s','end_to_end_sim_throughput','cleanup_wall_s','freefall_max_error_m','landed_max_error_m','xy_max_drift_m','passed']
            with (root/'trials.csv').open('w',newline='',encoding='utf-8') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore'); writer.writeheader(); writer.writerows(result['trials'])
            require(sha(self.workspace/'world.sdf')==before_sha,'Batch modified preview input')
            result.update(passed=True,repeat_comparisons=comparisons,timing_summary=summaries,preview_world_sha256=before_sha)
        except BaseException as error:
            result['error']=type(error).__name__+': '+str(error); raise
        finally:
            result['total_wall_s']=(time.monotonic_ns()-started)/1e9; write_json(root/'report.json',result)
        return result

    def verify(self):
        preview=self.ready(); result=self.batch(); result['preview_before']=preview
        result['preview_after']=self.ready(); return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('ready','status','pause','resume','step','reset','batch','verify'))
    parser.add_argument('--steps',type=int,default=100); args=parser.parse_args(); lab=Lab()
    if args.action in ('ready','status'): result=lab.ready()
    elif args.action in ('pause','resume'): result={'world_stats':lab.pause(args.action=='pause')}
    elif args.action=='step': result=lab.step(args.steps)
    else: result=getattr(lab,args.action)()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try: main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False)); raise SystemExit(1)
