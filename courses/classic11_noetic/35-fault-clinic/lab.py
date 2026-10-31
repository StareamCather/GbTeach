#!/usr/bin/env python3
"""Real native checkpoints and finite controls, with isolated owned processes."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid
from config import DT_NS, WORLD, prepared, sha


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
    ENV_KEY='GAZEBOTEACH_L35_SESSION'

    def __init__(self, root, world, seed=3501):
        self.token=uuid.uuid4().hex; self.root=Path(root).resolve(); root=self.root; root.mkdir(exist_ok=True); (root/'home').mkdir()
        self.world=root/'world.sdf'; self.world.write_bytes(world)
        self.processes=[]; self.streams=[]; self.seed=seed; self.previous_signal=None
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0)); self.port=listener.getsockname()[1]
        self.record_build=prepared(); self.env=os.environ.copy(); self.env.update({
            self.ENV_KEY:self.token,'GAZEBO_MASTER_URI':'http://127.0.0.1:'+str(self.port),
            'GAZEBO_IP':'127.0.0.1','IGN_PARTITION':'gazeboteach-l35-'+self.token,
            'GZ_PARTITION':'gazeboteach-l35-'+self.token,'HOME':str(root/'home'),
            'GAZEBO_MODEL_DATABASE_URI':'','GAZEBO_PLUGIN_PATH':str(Path.cwd().resolve()/'build'),'GAZEBO_MODEL_PATH':''})
        self.record={'token':self.token,'uid':os.getuid(),'master_uri':self.env['GAZEBO_MASTER_URI'],
                     'ign_partition':self.env['IGN_PARTITION'],'gz_partition':self.env['GZ_PARTITION'],
                     'world':str(self.world),'input_sha256':sha(self.world),'inherited_session_token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN'),'server_identity':self.record_build['server_identity'],'processes':[],'cleaned':False}
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
        def interrupted(signum,frame): raise KeyboardInterrupt('Bounded clinic interrupted by signal '+str(signum))
        self.previous_signal=signal.signal(signal.SIGTERM,interrupted)
        try:
            self.server_spawn_monotonic_ns=time.monotonic_ns()
            self.server=self.spawn([self.record_build['server_identity']['path'],'--verbose','-u','--seed',str(self.seed),str(self.world)],'gzserver')
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
        self.workspace=Path.cwd().resolve(); self.record=prepared(self.workspace)
        self.binary=self.workspace/'build/clinic_observer'; self.env=env; self.event_file=event_file

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
        require(world.get('name')==WORLD and set(positions(world))=={'cube_000'},'Healthy preview model differs')
        require(abs(float(physics['max_step_size'])-.001)<1e-12,'Preview dt differs')
        return {'ready':True,'native_world':world,'native_physics':physics,'world_stats':stats,
                'note':'GUI preview is independent; clinic creates isolated fresh paused worlds.'}

    def pause(self,value=True):
        if bool(self.stats().get('paused',False))!=value:
            self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+6
        while True:
            actual=self.stats()
            if bool(actual.get('paused',False))==value: return actual
            require(time.monotonic()<deadline,'Pause incomplete; command not repeated')

    def step(self,count=100,dt_ns=DT_NS):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 actual ticks')
        before=self.checkpoint(dt_ns)
        sent=self.client('control','--steps',str(count)); deadline=time.monotonic()+8
        target=before['sim_time_ns']+count*dt_ns
        while True:
            actual=self.stats()
            if ns(actual['sim_time'])>=target: break
            require(time.monotonic()<deadline,'Finite ticks incomplete; command not repeated')
        after=self.checkpoint(dt_ns)
        require(after['sim_time_ns']==target and after['iterations']-before['iterations']==count,'Actual step delta differs')
        return {'request':sent,'before':before,'after':after,'steps':count}

    def reset(self):
        self.pause(True); sent=self.client('control','--reset-all'); deadline=time.monotonic()+12
        while True:
            actual=self.checkpoint()
            if actual['sim_time_ns']==0 and abs(actual['positions_m']['cube_000'][2]-2)<1e-7:
                return {'request':sent,'after':actual}
            require(time.monotonic()<deadline,'Reset incomplete; command not repeated')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('ready','status','pause','resume','step','reset'))
    parser.add_argument('--steps',type=int,default=100); args=parser.parse_args(); lab=Lab()
    if args.action in ('ready','status'): result=lab.ready()
    elif args.action in ('pause','resume'): result={'world_stats':lab.pause(args.action=='pause')}
    elif args.action=='step': result=lab.step(args.steps)
    else: result=lab.reset()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__': main()
