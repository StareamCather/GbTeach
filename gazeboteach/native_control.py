"""Optional exact-Classic11 persistent control, isolated from the simulator host."""
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shlex
import shutil
import subprocess
import threading
import time
import uuid


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _output(command, env, timeout=20):
    return subprocess.check_output(command, env=env, text=True, stderr=subprocess.PIPE, timeout=timeout).strip()


def prepare(root, env):
    """Missing SDKs are optional; errors with the supported SDK remain visible."""
    source=Path(root).resolve()/'resources/native_control.cc'
    fallback={'backend':'legacy-cli','supported':False}
    if os.name!='posix':return None,dict(fallback,reason='Persistent Classic control requires the validated Linux SDK')
    paths={name:shutil.which(name,path=env.get('PATH')) for name in ('g++','pkg-config','gzserver')}
    if not all(paths.values()):return None,dict(fallback,reason='Optional g++/pkg-config/gzserver toolchain is absent')
    try:version=_output([paths['pkg-config'],'--modversion','gazebo'],env)
    except (OSError,subprocess.SubprocessError):return None,dict(fallback,reason='Optional Gazebo development SDK is absent')
    exact=Path('/usr/bin/gzserver-11.15.1')
    if version!='11.15.1' or not exact.is_file() or not os.path.samefile(exact,paths['gzserver']):
        return None,dict(fallback,reason='Persistent control is validated only with exact SDK and gzserver 11.15.1',sdk_version=version)
    import fcntl
    cflags=shlex.split(_output([paths['pkg-config'],'--cflags','gazebo'],env))
    libs=shlex.split(_output([paths['pkg-config'],'--libs','gazebo'],env))+['-ldl']
    compiler=Path(paths['g++']).resolve()
    dependencies=_output([str(compiler),'-std=c++17','-M',str(source)]+cflags,env)
    headers=shlex.split(dependencies.replace('\\\n',' ').split(':',1)[1])
    inputs={str(Path(name).resolve()):_sha(name) for name in headers}
    inputs[str(compiler)]=_sha(compiler);inputs[str(exact.resolve())]=_sha(exact)
    directories=[Path(flag[2:]) for flag in libs if flag.startswith('-L')]
    directories += [Path('/usr/lib')/_output([str(compiler),'-print-multiarch'],env),Path('/usr/lib')]
    for flag in libs:
        if not flag.startswith('-l'):continue
        filename='lib'+flag[2:]+'.so'
        library=next((directory/filename for directory in directories if (directory/filename).is_file()),None)
        if library is None:
            candidate=Path(_output([str(compiler),'-print-file-name='+filename],env))
            if candidate.is_file():library=candidate
        if library is None:raise RuntimeError('Cannot fingerprint native control library '+flag)
        library=library.resolve();inputs[str(library)]=_sha(library)
        linked=_output(['ldd',str(library)],env)
        for word in linked.replace('=>',' ').split():
            if word.startswith('/') and Path(word).is_file():
                path=Path(word).resolve();inputs[str(path)]=_sha(path)
    identity={'sdk_version':version,'source_sha256':_sha(source),'inputs_sha256':inputs,
              'compiler':_output([str(compiler),'--version'],env).splitlines()[0],'cflags':cflags,'libs':libs}
    key=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    cache=Path(env.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'gazeboteach/native-control'/key
    cache.mkdir(parents=True,exist_ok=True);target=cache/'native_control';record=cache/'build.json'
    with (cache/'compile.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        previous=json.loads(record.read_text()) if record.is_file() else {}
        if not target.is_file() or _sha(target)!=previous.get('executable_sha256'):
            temporary=cache/('compile-'+uuid.uuid4().hex)
            command=[str(compiler),'-std=c++17','-O2','-Wno-deprecated-declarations']+cflags+[str(source),'-o',str(temporary)]+libs
            try:
                result=subprocess.run(command,env=env,text=True,capture_output=True,timeout=120)
                if result.returncode:raise RuntimeError('Native control SDK build failed:\n'+result.stdout+result.stderr)
                os.replace(str(temporary),str(target))
            finally:
                if temporary.exists():temporary.unlink()
            value={'cache_key':key,'identity':identity,'executable_sha256':_sha(target),'compiler_command':command}
            temporary=cache/('record-'+uuid.uuid4().hex+'.json');temporary.write_text(json.dumps(value,indent=2)+'\n');temporary.replace(record)
    return target,{'backend':'classic11-sdk-persistent','supported':True,'cache_key':key,'record_path':str(record),
                   'source_sha256':identity['source_sha256'],'executable_sha256':_sha(target),'sdk_version':version}


class NativeControl:
    """One owned process, serialized finite RPCs, never restart unknown outcomes."""
    def __init__(self, executable, world, env, session_dir, build_identity=None):
        self.lock=threading.RLock();self.queue=queue.Queue();self.closed=False;self.poisoned=None
        self.build_identity=dict(build_identity or {});self.last_stats=None;self.cleanup=None
        self.executable=Path(executable).resolve();self.env=dict(env);self.world=world
        self.stderr_path=Path(session_dir)/'native-control.stderr.log'
        self.stderr=self.stderr_path.open('a',buffering=1)
        child_env=dict(env);child_env['GAZEBOTEACH_IO_DIAGNOSTICS']='1'
        self.process=subprocess.Popen([str(self.executable),world],env=child_env,stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=self.stderr,text=True,bufsize=1,start_new_session=True)
        proc=Path('/proc')/str(self.process.pid);fields=(proc/'stat').read_text().rsplit(') ',1)[1].split()
        self.identity={'pid':self.process.pid,'ppid':int(fields[1]),'start_time':fields[19],
            'pgid':os.getpgid(self.process.pid),'sid':os.getsid(self.process.pid),'uid':proc.stat().st_uid,
            'token':env.get('GAZEBOTEACH_SESSION_TOKEN'),'executable':str(self.executable),
            'executable_sha256':_sha(self.executable),'stderr_path':str(self.stderr_path)}
        def receive():
            try:
                for line in self.process.stdout:self.queue.put(line)
            finally:self.queue.put(None)
        self.reader=threading.Thread(target=receive,daemon=True);self.reader.start()
        try:
            ready=self._line(12)
            if ready.get('ready') is not True:raise RuntimeError('Native control readiness failed: '+str(ready))
            self.last_stats=self._stats(ready.get('stats'))
        except Exception:
            self.close();raise

    @staticmethod
    def _stats(value):
        if not isinstance(value,dict) or type(value.get('paused')) is not bool:
            raise RuntimeError('Native WorldStatistics paused identity is malformed')
        if any(type(value.get(key)) is not int or value[key]<0 for key in ('seq','iterations')):
            raise RuntimeError('Native WorldStatistics sequence/iterations are malformed')
        if type(value.get('sim_time')) not in (int,float) or not math.isfinite(value['sim_time']) or value['sim_time']<0:
            raise RuntimeError('Native WorldStatistics simulation time is malformed')
        return value

    def _line(self,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try:line=self.queue.get(timeout=min(.2,max(.001,deadline-time.monotonic())))
            except queue.Empty:
                if self.process.poll() is not None:raise RuntimeError('Owned native control exited; it will not restart automatically')
                continue
            if line is None:raise RuntimeError('Owned native control stream closed')
            try:value=json.loads(line)
            except (ValueError,TypeError) as error:raise RuntimeError('Malformed native control JSON') from error
            if not isinstance(value,dict):raise RuntimeError('Native control response must be a JSON object')
            return value
        raise RuntimeError('Native control wall timeout; unknown mutation outcome is not retried')

    def _rpc(self,action,timeout=3,**fields):
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or not .3<=timeout<=20:
            raise ValueError('RPC timeout must be finite, between .3 and 20 seconds')
        with self.lock:
            if self.closed or self.poisoned:raise RuntimeError('Native control unavailable: '+str(self.poisoned or 'closed'))
            if self.process.poll() is not None:
                self.poisoned='owned process exited';raise RuntimeError(self.poisoned)
            request_id=uuid.uuid4().hex
            request=dict(fields,id=request_id,action=action,timeout_ms=max(100,int(timeout*1000)-200))
            try:
                self.process.stdin.write(json.dumps(request)+'\n');self.process.stdin.flush()
                response=self._line(timeout)
                if response.get('id')!=request_id:raise RuntimeError('Native response UUID differs; outcome is unknown')
                if response.get('ok') is not True:raise RuntimeError('Native control refused/unconfirmed: '+str(response.get('error')))
                stats=self._stats(response.get('stats'))
                if self.last_stats and stats['seq']<=self.last_stats['seq']:
                    raise RuntimeError('Native control response reused a stale statistics sequence')
                self.last_stats=stats
                return response
            except Exception as error:
                self.poisoned=str(error);raise

    def status(self,timeout=3):return self._rpc('status',timeout)

    def model_pose(self,name,timeout=3):
        """Read actual Model.pose with one correlated native entity_info request."""
        if not isinstance(name,str) or not name or len(name.encode('utf-8'))>512 or any(ord(c)<32 or ord(c)==127 for c in name):
            raise ValueError('Model pose needs a nonempty bounded model name')
        with self.lock:
            previous_seq=self.last_stats['seq'] if self.last_stats else 0
            value=self._rpc('model_pose',timeout,name=name)
        try:
            actual=value.get('model_name')
            if not isinstance(actual,str) or (actual!=name and actual.rsplit('::',1)[-1]!=name):
                raise RuntimeError('Native Model pose name differs')
            request=value.get('request_id');response=value.get('response_id')
            if type(request) is not int or not 1<=request<=0x7fffffff or type(response) is not int or request!=response or value.get('response_type')!='gazebo.msgs.Model':
                raise RuntimeError('Native Model response ID/type differs')
            pose=value.get('pose')
            if not isinstance(pose,dict) or set(pose)!= {'position','orientation'}:
                raise RuntimeError('Native Model pose schema is malformed')
            for field,keys in (('position',{'x','y','z'}),('orientation',{'x','y','z','w'})):
                components=pose[field]
                if not isinstance(components,dict) or set(components)!=keys or any(type(v) not in (int,float) or not math.isfinite(v) for v in components.values()):
                    raise RuntimeError('Native Model pose components are malformed')
            if abs(sum(v*v for v in pose['orientation'].values())-1)>1e-6:
                raise RuntimeError('Native Model pose quaternion is not unit length')
            rpy=value.get('rpy')
            if not isinstance(rpy,list) or len(rpy)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in rpy):
                raise RuntimeError('Native Model pose Euler angles are malformed')
            before=self._stats(value.get('pose_stats_before'));after=self._stats(value.get('pose_stats_after'))
            if before['seq']<previous_seq or before['seq']>=after['seq'] or after!=value['stats']:
                raise RuntimeError('Native Model pose statistics bracket is malformed')
        except Exception as error:
            self.poisoned=str(error);raise
        return value

    def pause(self,paused,timeout=4):
        if type(paused) is not bool:raise ValueError('Pause requires explicit bool')
        value=self._rpc('pause',timeout,paused=paused)
        if value['stats']['paused']!=paused:
            self.poisoned='Actual pause response differs';raise RuntimeError(self.poisoned)
        return value

    def step(self,count,timeout=5):
        if type(count) is not int or not 1<=count<=1000000:raise ValueError('Step count must be 1..1000000')
        value=self._rpc('step',timeout,count=count)
        baseline=value.get('baseline_iterations')
        if type(baseline) is not int or baseline<0 or type(value.get('requested_steps')) is not int or value['requested_steps']!=count or not value['stats']['paused'] or value['stats']['iterations']!=baseline+count:
            self.poisoned='Exact native iteration increment was not confirmed';raise RuntimeError(self.poisoned)
        return value

    def reset_models(self,timeout=4):return self._rpc('reset_models',timeout)

    def metadata(self):
        return {'backend':'classic11-sdk-persistent','process':dict(self.identity),'build':dict(self.build_identity),
                'world':self.world,'stats':self.last_stats,'poisoned':self.poisoned,'closed':self.closed,'cleanup':self.cleanup}

    def close(self):
        with self.lock:
            if self.closed:return self.cleanup
            self.closed=True;value={'eof_requested':False,'forced':False,'io_guard_joined':False}
            try:
                try:self.process.stdin.close();value['eof_requested']=True
                except OSError as error:value['stdin_error']=str(error)
                try:self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    value['forced']=True;self.process.terminate()
                    try:self.process.wait(timeout=1)
                    except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=1)
                self.reader.join(timeout=.5)
                while not self.queue.empty():
                    line=self.queue.get_nowait()
                    if line:
                        try:
                            result=json.loads(line)
                            if result.get('closed') is True and result.get('io_guard_joined') is True:value['io_guard_joined']=True
                        except (ValueError,AttributeError):pass
                value['returncode']=self.process.returncode
            except Exception as error:value['error']=str(error)
            finally:
                if not self.reader.is_alive():self.process.stdout.close()
                self.stderr.close();self.cleanup=value
            return value


def try_create(root,env,world,session_dir):
    executable,diagnosis=prepare(root,env)
    if executable is None:return None,diagnosis
    return NativeControl(executable,world,env,session_dir,diagnosis),diagnosis
