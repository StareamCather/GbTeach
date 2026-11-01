#!/usr/bin/env python3
"""Native Classic state recording, immutable inspection, isolated real playback."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET
from config import DT_NS, WORLD, parameters, prepared, sha


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


def state_ns(text):
    words=text.split();require(len(words)==2,'Native state time needs seconds and nanoseconds')
    return int(words[0])*1000000000+int(words[1])


def position(entity):return [float(entity['pose']['position'].get(axis,0)) for axis in ('x','y','z')]


def serialized_position(entity,significant_digits=3):
    """SDK11.15.1: decimal4 rounding, defaultfloat model3/link4 significant digits.

    This is a labelled serialization prediction, never a native measurement.
    C++ round's half-away-from-zero differs from Python's bankers' round.
    """
    return [float(format(math.copysign(math.floor(abs(value)*10000+.5)/10000,value),'.'+str(significant_digits)+'g'))
            for value in position(entity)]


def write_json(path,value):
    temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


class OwnedPlayback:
    """Only signals processes carrying this unguessable session token and UID.

    Every child inherits the token. We validate /proc starttime again before
    signalling each PID, so a dead leader or PID reuse cannot target user apps.
    """
    def __init__(self,root,log_path,gui):
        self.token=uuid.uuid4().hex;self.root=root/'build'/('playback-'+self.token)
        self.root.mkdir();(self.root/'home').mkdir();self.processes=[];self.streams=[]
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0));self.port=listener.getsockname()[1]
        self.env=os.environ.copy();self.env.update({
            'GAZEBOTEACH_L32_SESSION':self.token,'GAZEBO_MASTER_URI':'http://127.0.0.1:'+str(self.port),
            'GAZEBO_IP':'127.0.0.1','IGN_PARTITION':'gazeboteach-l32-'+self.token,
            'GZ_PARTITION':'gazeboteach-l32-'+self.token,'HOME':str(self.root/'home'),
            'GAZEBO_MODEL_DATABASE_URI':'','GAZEBO_PLUGIN_PATH':'','GAZEBO_MODEL_PATH':''})
        self.log_path=log_path;self.gui=gui;self.previous_signal=None
        self.record={'token':self.token,'master_uri':self.env['GAZEBO_MASTER_URI'],
                     'ign_partition':self.env['IGN_PARTITION'],'native_log':str(log_path),'processes':[],'cleaned':False}

    @staticmethod
    def identity(pid):
        try:
            path=Path('/proc')/str(pid)
            if path.stat().st_uid!=os.getuid():return None
            environment=dict(item.split(b'=',1) for item in (path/'environ').read_bytes().split(b'\0') if b'=' in item)
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            return {'pid':pid,'start_ticks':stat[19],'token':environment.get(b'GAZEBOTEACH_L32_SESSION',b'').decode(),
                    'exe':os.readlink(str(path/'exe')),'cmdline':(path/'cmdline').read_bytes().replace(b'\0',b' ').decode('utf-8','replace')}
        except (OSError,ValueError):return None

    def owned(self):
        values=[]
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                value=self.identity(int(path.name))
                if value and value['token']==self.token:values.append(value)
        return values

    def spawn(self,command,label):
        stream=(self.root/(label+'.out')).open('w',encoding='utf-8');self.streams.append(stream)
        process=subprocess.Popen(command,cwd=str(self.root),env=self.env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append(process);identity=self.identity(process.pid)
        require(identity and identity['token']==self.token,'Spawned playback identity was not established')
        self.record['processes'].append(dict(identity,label=label,command=command))
        write_json(self.root/'ownership.json',self.record);return process

    def __enter__(self):
        def interrupted(signum,frame):raise KeyboardInterrupt('Bounded playback interrupted by signal '+str(signum))
        self.previous_signal=signal.signal(signal.SIGTERM,interrupted)
        try:
            self.server=self.spawn(['gzserver','--verbose','-u','-p',str(self.log_path)],'gzserver')
            return self
        except BaseException:
            self.__exit__(None,None,None);raise

    def role(self,value):
        # Recorded labels require the original PID AND start tick. Descendants
        # still need the private token/UID checked by owned()/identity().
        for original in self.record['processes']:
            if value['pid']==original['pid'] and value['start_ticks']==original['start_ticks']:
                if original['label']=='gzclient':return 'gui'
                if original['label']=='gzserver':return 'server'
        executable=Path(value['exe']).name
        if executable.startswith('gzclient'):return 'gui'
        if executable.startswith('gzserver'):return 'server'
        return 'client'

    def __exit__(self,*unused):
        # All GUI observation functions run synchronously inside the with block;
        # none can issue a read/seek after exit starts. Defer repeat signals while
        # this bounded cleanup drains GUI clients before their master/server.
        handlers={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGTERM,signal.SIGINT)}
        self.record['cleanup_events']=[]
        try:
            for role in ('gui','client','server'):
                for sig,limit in ((signal.SIGTERM,5),(signal.SIGKILL,2)):
                    values=[v for v in self.owned() if self.role(v)==role]
                    for value in values:
                        again=self.identity(value['pid'])
                        if again and again['token']==self.token and again['start_ticks']==value['start_ticks']:
                            try:
                                os.kill(value['pid'],sig)
                                self.record['cleanup_events'].append({'role':role,'pid':value['pid'],
                                    'start_ticks':value['start_ticks'],'signal':int(sig),'monotonic_ns':time.monotonic_ns()})
                            except ProcessLookupError:pass
                    deadline=time.monotonic()+limit
                    while any(self.role(v)==role for v in self.owned()) and time.monotonic()<deadline:
                        for process in self.processes:process.poll()
                        time.sleep(.04)
                # Do not tear down the server underneath a surviving GUI.
                if any(self.role(v)==role for v in self.owned()):break
            for process in self.processes:
                try:process.wait(timeout=1)
                except subprocess.TimeoutExpired:pass
            remaining=self.owned();self.record['remaining']=remaining;self.record['cleaned']=not remaining
            self.record['cleanup_order']='gui, client, server; each role fully exits before the next is signalled'
            self.record['owned_at_cleanup']=[{'pid':p.pid,'returncode':p.poll()} for p in self.processes]
            write_json(self.root/'ownership.json',self.record)
            require(not remaining,'Owned native playback descendants survived bounded cleanup')
        finally:
            for stream in self.streams:stream.close()
            for sig,handler in handlers.items():signal.signal(sig,handler)
            if self.previous_signal is not None:signal.signal(signal.SIGTERM,self.previous_signal)


class Lab:
    def __init__(self):
        self.settings,self.record=prepared();self.workspace=Path.cwd().resolve();self.binary=self.workspace/'build/log_observer'

    def client(self,mode,*args,env=None,timeout=8,all_rows=False):
        process=subprocess.run([str(self.binary),mode]+list(args),env=env,text=True,capture_output=True,timeout=timeout)
        rows=decode(process.stdout)
        require(process.returncode==0,'Native client failed: '+process.stdout[-1600:]+' '+process.stderr[-1000:])
        require(rows and not any(row.get('source') in ('client_error','decode_error') for row in rows),'Actual native client returned no valid result')
        return rows if all_rows else rows[-1]

    def stats(self,env=None):return self.client('stats',env=env)['data']
    def entity(self,env=None):return self.client('entity','--name','falling_cube',env=env)['data']
    def status(self):return self.client('log-status')

    def checkpoint(self,env=None,context='checkpoint'):
        # A full reset or seek may land between these independent native
        # queries. Resample READS until a coherent paused interval is observed;
        # never replay its preceding non-idempotent control publication.
        path=self.workspace/'build'/('checkpoint-'+uuid.uuid4().hex+'.json')
        attempts=[];deadline=time.monotonic()+6
        diagnostic={'passed':False,'context':context,'master_uri':(env or os.environ).get('GAZEBO_MASTER_URI',''),
                    'attempts':attempts}
        write_json(path,diagnostic)
        def query(mode,*args):
            remaining=deadline-time.monotonic()
            require(remaining>.1,'Read-only consistency deadline expired')
            native_timeout=max(.1,min(3,remaining-.2))
            return self.client(mode,*args,'--timeout',str(native_timeout),env=env,timeout=min(8,remaining))['data']
        def summary(value):
            return {'sim_time_ns':ns(value['sim_time']),'iterations':int(value.get('iterations',0)),
                    'paused':bool(value.get('paused',False))}
        try:
            while True:
                before=query('stats');diagnostic['last_before']=before;write_json(path,diagnostic)
                entity=query('entity','--name','falling_cube');diagnostic['last_native_entity']=entity;write_json(path,diagnostic)
                after=query('stats');diagnostic['last_after']=after
                first=summary(before);last=summary(after)
                stable=(first['paused'] and last['paused'] and first['sim_time_ns']==last['sim_time_ns']
                        and first['iterations']==last['iterations'])
                attempts.append({'attempt':len(attempts)+1,'before':first,'after':last,
                                 'status':'stable' if stable else 'pending_transition'})
                diagnostic['passed']=stable;write_json(path,diagnostic)
                if stable:
                    return {'sim_time_ns':last['sim_time_ns'],'world_stats':after,'native_entity':entity,
                            'position_m':position(entity),'consistency_attempts':attempts,'diagnostics_file':str(path)}
                require(time.monotonic()<deadline,'No coherent paused native checkpoint before the deadline')
                time.sleep(.03)
        except Exception as error:
            diagnostic['error']=type(error).__name__+': '+str(error);write_json(path,diagnostic)
            raise RuntimeError(context+': '+str(error)+'; checkpoint diagnostics '+str(path)) from error

    def ready(self):
        self.settings,self.record=prepared();world=self.client('world')['data'];stats=self.stats()
        require(world.get('name')==WORLD,'Wrong native world identity')
        names={model['name'] for model in world.get('model',[])}
        require({'falling_cube','floor'}<=names,'Missing native falling scene')
        return {'ready':True,'parameters':self.settings,'world_stats':stats,'native_entity':self.entity(),
                'log_status':self.status(),'observer_sha256':self.record['observer_sha256']}

    def pause(self,value=True):
        if bool(self.stats().get('paused',False))!=value:self.client('control','--pause','true' if value else 'false')
        deadline=time.monotonic()+5
        while True:
            state=self.stats()
            if bool(state.get('paused',False))==value:return state
            require(time.monotonic()<deadline,'Native pause did not complete; command not repeated')

    def step(self,count=100):
        require(type(count) is int and 1<=count<=1000,'Use 1..1000 actual ticks')
        before=self.checkpoint(context='step/before/'+str(count));sent=self.client('control','--steps',str(count));deadline=time.monotonic()+6
        target=before['sim_time_ns']+count*DT_NS
        while True:
            actual=self.stats()
            if ns(actual['sim_time'])>=target:break
            require(time.monotonic()<deadline,'Native finite steps not completed; non-idempotent pulse not repeated')
        require(ns(actual['sim_time'])==target and actual.get('paused'),'Unexpected native finite step completion')
        return {'passed':True,'request':sent,'before':before,'after':self.checkpoint(context='step/after/'+str(count)),'steps':count}

    def reset(self):
        self.pause(True);sent=self.client('control','--reset-all');deadline=time.monotonic()+6
        while True:
            actual=self.checkpoint(context='full_reset/poll')
            if actual['sim_time_ns']==0 and abs(actual['position_m'][2]-2)<1e-7:return {'request':sent,'after':actual}
            require(time.monotonic()<deadline,'Native full reset not observed; command not repeated')

    def inspect(self,path=None):
        if path is None:path=Path(json.loads((self.workspace/'build/recording.json').read_text())['log_path'])
        path=Path(path).resolve();require(path.is_file() and path.stat().st_size<=16*1024*1024,'Use an existing bounded native log')
        original=sha(path);container=ET.parse(str(path)).getroot()
        require(container.tag=='gazebo_log' and path.read_bytes().rstrip().endswith(b'</gazebo_log>'),'Refuse incomplete native XML before LogPlay can repair it')
        rows=self.client('log-read','--file',str(path),timeout=15,all_rows=True)
        require(sha(path)==original,'Native log bytes changed during inspection')
        metadata=rows[-1];require(metadata.get('source')=='log_metadata','Missing actual LogPlay metadata')
        states=[];initial_worlds=[]
        for row in rows[:-1]:
            require(row.get('source')=='log_frame','Unexpected native log decoder output')
            element=ET.fromstring(row['raw_sdf'].rstrip('\x00'));world=element.find('world')
            if world is not None:initial_worlds.append({'frame_index':row['frame_index'],'world':world.get('name')})
            state=element.find('state')
            if state is None and world is not None:state=world.find('state')
            if state is None:continue
            model=state.find("model[@name='falling_cube']")
            if model is None or model.find('pose') is None:continue
            pose=[float(item) for item in model.findtext('pose').split()]
            require(len(pose)==6 and all(math.isfinite(v) for v in pose),'Malformed measured native model pose')
            body=model.find("link[@name='body']")
            require(body is not None and body.find('pose') is not None,'Native state omitted its canonical body link')
            link_pose=[float(item) for item in body.findtext('pose').split()]
            require(len(link_pose)==6 and all(math.isfinite(v) for v in link_pose),'Malformed measured native canonical-link world pose')
            states.append({'frame_index':row['frame_index'],'sim_time_ns':state_ns(state.findtext('sim_time')),
                           'iterations':int(state.findtext('iterations','0')),'pose_xyz_rpy':pose,'position_m':pose[:3],
                           'canonical_link_pose_xyz_rpy':link_pose,'canonical_link_position_m':link_pose[:3],
                           'effective_model_position_m':link_pose[:3]})
        require(initial_worlds and initial_worlds[0]['world']==WORLD and len(states)>3,'Native log needs initial world and changing falling states')
        times=[s['sim_time_ns'] for s in states];require(times==sorted(times),'Native log state times rewind unexpectedly')
        return {'log_path':str(path),'log_sha256':original,'file_bytes':path.stat().st_size,'closed_xml':True,
                'metadata':metadata,'initial_worlds':initial_worlds,'states':states,'raw_frames':rows[:-1]}

    def record_native(self):
        self.ready();self.pause(True)
        previous=self.status();require(not previous['data'].get('log_file',{}).get('full_path'),'A native recorder is already active; stop its owner first')
        self.reset();identity=uuid.uuid4().hex;base=self.workspace/'build/recordings'/identity;base.mkdir(parents=True)
        require(not any(base.iterdir()),'Recording directory must be new')
        checkpoints=[self.checkpoint(context='record/initial')];started=False;statuses=[];stop=None;closed=None
        try:
            # Claim before publication: even an uncertain client result receives
            # one bounded cleanup stop. Never resend start or finite steps.
            started=True
            start=self.client('log-control','--action','start','--path',str(base),'--encoding',self.settings['encoding'])
            deadline=time.monotonic()+6
            while True:
                actual=self.status();statuses.append(actual)
                if actual['data'].get('log_file',{}).get('full_path'):break
                require(time.monotonic()<deadline,'Native recorder did not expose an active log path')
            total=self.settings['record_steps'];last=0
            for target in sorted({max(1,total//4),total//2,3*total//4,total}):
                self.step(target-last);last=target;checkpoints.append(self.checkpoint(context='record/original/'+str(target)))
            # Native writer is asynchronous. While paused its worker gets time
            # to drain the final physics sample; no physics tick is invented.
            time.sleep(.25)
            stop=self.client('log-control','--action','stop');started=False
            deadline=time.monotonic()+8;last_signature=None;stable_since=None
            while True:
                actual=self.status();statuses.append(actual);files=list(base.rglob('state.log'))
                if not actual['data'].get('log_file',{}).get('full_path') and len(files)==1:
                    path=files[0];data=path.read_bytes()
                    if data.rstrip().endswith(b'</gazebo_log>'):
                        try:ET.fromstring(data)
                        except ET.ParseError:pass
                        else:
                            signature=(len(data),sha(path))
                            if signature==last_signature:
                                if time.monotonic()-stable_since>=.2:
                                    closed={'status':actual,'size_bytes':len(data),'sha256':signature[1],'stable_seconds':time.monotonic()-stable_since};break
                            else:last_signature=signature;stable_since=time.monotonic()
                require(time.monotonic()<deadline,'Native stop has no stable closed complete log; refusing replay')
                time.sleep(.08)
            inspected=self.inspect(path);states=inspected['states'];matches=[]
            for checkpoint_index,checkpoint in enumerate(checkpoints[1:],1):
                # Native LogWorker suppresses unchanged poses. A settled body
                # can reach original time T without a state stamped T. Only
                # the latest actual record S<=T may stand for that checkpoint,
                # and only if BOTH saved representations equal the original's
                # serialization. Keep the two clocks explicit; never invent T.
                candidates=[s for s in states if s['sim_time_ns']<=checkpoint['sim_time_ns']]
                require(candidates,'No native state at/before original checkpoint: '+str(checkpoint['sim_time_ns']))
                logged=candidates[-1];error=max(abs(a-b) for a,b in zip(logged['position_m'],checkpoint['position_m']))
                prediction=serialized_position(checkpoint['native_entity'])
                serialization_error=max(abs(a-b) for a,b in zip(logged['position_m'],prediction))
                require(serialization_error<=1e-9,'Native logged position differs from the installed ModelState serialization of the original')
                link_prediction=serialized_position(checkpoint['native_entity'],4)
                link_error=max(abs(a-b) for a,b in zip(logged['canonical_link_position_m'],link_prediction))
                require(link_error<=1e-9,'Native canonical-link world position differs from the installed LinkState serialization of the original')
                original_time=checkpoint['sim_time_ns'];recorded_time=logged['sim_time_ns']
                matches.append({'sim_time_ns':recorded_time,'original_sim_time_ns':original_time,
                                'recorded_sim_time_ns':recorded_time,'original_checkpoint_index':checkpoint_index,
                                'match_kind':'exact' if recorded_time==original_time else 'retained_state',
                                'retained_interval_ns':original_time-recorded_time,
                                'max_position_error_m':error,'logged':logged,
                                'serialization_prediction_m':prediction,'serialization_error_m':serialization_error,
                                'link_serialization_prediction_m':link_prediction,'link_serialization_error_m':link_error})
            require(checkpoints[-1]['sim_time_ns']==total*DT_NS,'Original world did not reach the requested bounded duration')
            require(states[-1]['sim_time_ns']<=total*DT_NS and inspected['metadata']['end_time_ns']<=total*DT_NS,
                    'Native log extends beyond the measured original recording interval')
            require(matches[-1]['recorded_sim_time_ns']==states[-1]['sim_time_ns'],'Final original checkpoint was not compared to the last actual native state')
            result={'passed':True,'parameters':self.settings,'recording_id':identity,'log_path':str(path),'log_sha256':inspected['log_sha256'],
                    'preparation':self.record,
                    'file_bytes':inspected['file_bytes'],'metadata':inspected['metadata'],'original_checkpoints':checkpoints,
                    'original_log_matches':matches,'start_request':start,'stop_request':stop,'log_status_observations':statuses,
                    'stop_completion':closed,'native_state_count':len(states),'frame_count':len(inspected['raw_frames']),
                    'original_final_sim_time_ns':checkpoints[-1]['sim_time_ns'],
                    'last_recorded_sim_time_ns':states[-1]['sim_time_ns'],
                    'manifest_path':str(self.workspace/'build/recording.json'),
                    'policy':'Original world remains paused at recorded end. Full reset is explicit; no model pose setter or control replay is used.'}
            write_json(base/'manifest.json',result);write_json(self.workspace/'build/recording.json',result);return result
        finally:
            if started:self.client('log-control','--action','stop')
            self.pause(True)

    def playback(self,gui=False,seconds=6):
        require(math.isfinite(seconds) and .5<=seconds<=20,'GUI hold duration must be .5..20 seconds')
        if gui:require(bool(os.environ.get('DISPLAY')),'--gui requires an actual display')
        manifest=json.loads((self.workspace/'build/recording.json').read_text(encoding='utf-8'))
        stable=('parameters','config_sha256','source_sha256','world_sha256','gazebo_version','observer_sha256','cache_key')
        require(all(manifest['preparation'][key]==self.record[key] for key in stable),'Recording belongs to different prepared inputs; record again after reload')
        inspection=self.inspect(manifest['log_path']);require(inspection['log_sha256']==manifest['log_sha256'],'Recorded native log fingerprint changed')
        live_before=self.checkpoint(context='playback/live_before');comparisons=[];session=OwnedPlayback(self.workspace,Path(manifest['log_path']),gui)
        with session:
            deadline=time.monotonic()+15;last_error=''
            while True:
                require(session.server.poll() is None,'Native playback server exited: '+(session.root/'gzserver.out').read_text()[-2000:])
                try:
                    stats=self.stats(session.env)
                    if stats.get('paused'):
                        require('log_playback_stats' in stats,'Private server has no native log playback statistics')
                        break
                except RuntimeError as error:last_error=str(error)
                require(time.monotonic()<deadline,'Private native playback world not ready: '+last_error)
                time.sleep(.1)
            identity=session.identity(session.server.pid)
            installed=shutil.which('gzserver')
            require(identity and installed and os.path.samefile(identity['exe'],installed) and ' -p ' in identity['cmdline'],
                    'Playback is not the resolved installed native gzserver -p process')
            with open(identity['exe'],'rb') as binary:require(binary.read(4)==b'\x7fELF','Playback executable is not a native ELF')
            diagnostics={'passed':False,'native_process':identity,'log_path':manifest['log_path'],
                         'log_sha256':manifest['log_sha256'],'comparisons':comparisons}
            write_json(session.root/'diagnostics.json',diagnostics)
            client=None;liveness={'enabled':gui,'checks':0}
            diagnostics['gui_liveness']=liveness
            def check_gui_alive(stage):
                liveness.update(checks=liveness['checks']+1,last_stage=stage,
                                server_returncode=session.server.poll(),
                                client_returncode=client.poll() if client is not None else None)
                if liveness['server_returncode'] is not None or (client is not None and liveness['client_returncode'] is not None):
                    liveness['failed']=True;write_json(session.root/'diagnostics.json',diagnostics)
                    server_tail=(session.root/'gzserver.out').read_text()[-1200:]
                    gui_tail=(session.root/'gzclient.out').read_text()[-1200:] if client is not None else ''
                    raise RuntimeError('Owned native GUI playback process exited during '+stage+': '+
                                       server_tail+' '+gui_tail+'; see '+str(session.root/'diagnostics.json'))
            def gui_hold(duration,stage):
                deadline=time.monotonic()+duration
                while True:
                    check_gui_alive(stage);remaining=deadline-time.monotonic()
                    if remaining<=0:break
                    time.sleep(min(.1,remaining))
            if gui:
                client=session.spawn(['gzclient','--verbose'],'gzclient');gui_hold(3,'GUI startup')
            # Seek is native LogPlaybackControl, not a pose setter. Gazebo seeks
            # to a frame at/before the requested time, then applies that frame.
            for matched in manifest['original_log_matches']:
                original=manifest['original_checkpoints'][matched['original_checkpoint_index']]
                target=matched['recorded_sim_time_ns'];logged=[s for s in inspection['states'] if s['sim_time_ns']==target][-1]
                if gui:check_gui_alive('before seek '+str(target))
                sent=self.client('playback-control','--seek-ns',str(target),env=session.env)
                deadline=time.monotonic()+6
                while True:
                    actual=self.checkpoint(session.env,context='playback/seek/'+str(target))
                    if gui:check_gui_alive('after seek checkpoint '+str(target))
                    diagnostics['last_native_playback']=actual;write_json(session.root/'diagnostics.json',diagnostics)
                    if actual['sim_time_ns']==target:break
                    require(time.monotonic()<deadline,'Native seek did not apply exact recorded timestamp')
                # Model.SetState applies model pose THEN link states. Setting
                # the canonical link recomputes its parent model world pose.
                # prepare requires this sole body's initial relative pose=I,
                # so the effective model position equals logged body WORLD xyz.
                log_error=max(abs(a-b) for a,b in zip(actual['position_m'],logged['effective_model_position_m']))
                original_error=max(abs(a-b) for a,b in zip(actual['position_m'],original['position_m']))
                prediction=serialized_position(original['native_entity'],4)
                serialization_error=max(abs(a-b) for a,b in zip(actual['position_m'],prediction))
                comparisons.append({'sim_time_ns':target,'recorded_sim_time_ns':target,
                                    'original_sim_time_ns':original['sim_time_ns'],'match_kind':matched['match_kind'],
                                    'retained_interval_ns':matched['retained_interval_ns'],
                                    'request':sent,'native_playback':actual,'logged':logged,
                                    'original':original,'log_position_error_m':log_error,'original_position_error_m':original_error,
                                    'serialization_prediction_m':prediction,'serialization_error_m':serialization_error})
                write_json(session.root/'diagnostics.json',diagnostics)
                require(log_error<=1e-8 and serialization_error<=1e-8,
                        'Actual native playback position differs from canonical-link log/serialized original; see '+str(session.root/'diagnostics.json'))
                if gui:gui_hold(.5,'checkpoint display '+str(target))
            if gui:gui_hold(seconds,'final display')
            require(sha(manifest['log_path'])==manifest['log_sha256'],'Native playback changed source log')
            result={'passed':True,'native_command':['gzserver','--verbose','-u','-p',manifest['log_path']],
                    'native_process':identity,'session_directory':str(session.root),'ownership_file':str(session.root/'ownership.json'),
                    'private_master_uri':session.env['GAZEBO_MASTER_URI'],'ign_partition':session.env['IGN_PARTITION'],
                    'gui_requested':gui,'comparisons':comparisons,'log_sha256':manifest['log_sha256'],
                    'diagnostics_file':str(session.root/'diagnostics.json'),'gui_liveness':liveness}
            if gui:check_gui_alive('before owned cleanup')
            diagnostics['passed']=True;write_json(session.root/'diagnostics.json',diagnostics)
        live_after=self.checkpoint(context='playback/live_after');require(live_before['sim_time_ns']==live_after['sim_time_ns'] and live_before['position_m']==live_after['position_m'],'Private playback changed the original paused world')
        result.update(ownership=json.loads((session.root/'ownership.json').read_text()),live_before=live_before,live_after=live_after)
        write_json(session.root/'result.json',result);return result

    def verify(self):
        original=self.record_native();replay=self.playback()
        return {'passed':True,'recording':original,'playback':replay,
                'limits':'Native state/time/position acceptance; does not reproduce command topics or verify sensor pixels. GUI is optional and explicitly requested.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=['ready','pause','resume','reset','step','record','inspect','playback','verify'])
    parser.add_argument('--steps',type=int,default=100);parser.add_argument('--file',type=Path)
    parser.add_argument('--gui',action='store_true');parser.add_argument('--seconds',type=float,default=6);args=parser.parse_args();lab=Lab()
    if args.command=='ready':result=lab.ready()
    elif args.command=='pause':result=lab.pause(True)
    elif args.command=='resume':result=lab.pause(False)
    elif args.command=='reset':result=lab.reset()
    elif args.command=='step':result=lab.step(args.steps)
    elif args.command=='record':result=lab.record_native()
    elif args.command=='inspect':result=lab.inspect(args.file)
    elif args.command=='playback':result=lab.playback(args.gui,args.seconds)
    else:result=lab.verify()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try:main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
