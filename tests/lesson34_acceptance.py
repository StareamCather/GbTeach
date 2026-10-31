#!/usr/bin/env python3
"""Independent live acceptance of L34's real headless batch experiments.

A paused preview is observed before and after the public batch command. A
separate native transport subscriber attaches to every actual child gzserver
while it exists, preserving original WorldStatistics / PosesStamped messages.
Its process identity, executable, mapped SDK libraries, private transport and
input SDF are read from /proc independently of the course's result files.
All eight factorial trials are decoded again from their native query journals.
Geometry, real freefall, landing, dt, iterations, repeated input/outcome and CSV
rows are checked. Wall-clock values are recomputed from recorded boundaries and
bounded by an independent parent monotonic interval; they include startup and
queries, and are never presented as engine RTF or isolated physics CPU time.
The actual batch CLI runs through an acceptance worker that gates only the
first step of each child until native zero-time observer readiness is proven.
Every original step is called once with unchanged arguments. Inclusive wall
timing includes this measured gate; it is not an uninstrumented CLI benchmark.
"""
import argparse
import base64
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import runpy
import sys
import tempfile
import threading
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model
from tests.lesson32_acceptance import events, hashes, save, sha

WORLD = 'lesson34'
CUSTOM_TOKEN = 'GAZEBOTEACH_L34_SESSION'
CONFIG = {'counts': [8, 24], 'step_sizes_s': [.001, .002], 'repeats': 2, 'seed': 12345}
SDK_BASE = 'https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/'

# This wire-only addition is compiled into an acceptance artifact, never into
# the lesson. It keeps the already-connected watcher node alive through every
# endpoint: no new helper process or transport discovery is needed at 1 second.
ACTIVE_SCENE_CPP = r'''
class AcceptanceSceneWatch {
 public:
  gazebo::transport::PublisherPtr publisher;
  struct Pending { std::string phase; long long simNs, triggerNs; std::string stats, request; };
  std::mutex mutex;
  std::map<int, Pending> pending;
  std::map<long long, int> sent;
  int nextId = 0x50000000 | static_cast<int>(Now() & 0x0fffffff);
  std::string latestStats;
  long long latestSimNs=-1;

  static std::string Base64(const std::string &bytes) {
    static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string result; unsigned int value=0; int bits=-6;
    for(unsigned char c:bytes) {
      value=(value<<8)|c; bits+=8;
      while(bits>=0) { result+=alphabet[(value>>bits)&63]; bits-=6; }
    }
    if(bits>-6)result+=alphabet[((value<<8)>>(bits+8))&63];
    while(result.size()%4)result+='=';
    return result;
  }
  void Stats(ConstWorldStatisticsPtr &message) {
    const long long callbackReceiveNs=Now();
    Print("{\"source\":\"independent_native_stats\",\"receive_monotonic_ns\":"+
      std::to_string(callbackReceiveNs)+",\"raw_protobuf\":"+Q(message->DebugString())+"}");
    const long long sim=message->sim_time().sec()*1000000000LL+message->sim_time().nsec();
    {
      std::lock_guard<std::mutex> lock(mutex);
      latestSimNs=sim;latestStats=message->DebugString();
    }
    if(!message->paused())return;
    std::string phase;
    if(sim==0)phase="initial";
    else if(sim==200000000LL)phase="freefall";
    else if(sim==1000000000LL)phase="final";
    else return;
    msgs::Request query;
    Pending request;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if(sent.count(sim))return;
      const int id=nextId++;
      query.set_id(id);query.set_request("scene_info");query.set_data("acceptance_"+phase);
      request={phase,sim,Now(),message->DebugString(),query.DebugString()};
      pending.emplace(id,request);sent.emplace(sim,id);
    }
    Print("{\"source\":\"independent_scene_request\",\"phase\":"+Q(phase)+
      ",\"trigger_native_sim_time_ns\":"+std::to_string(sim)+
      ",\"trigger_receive_monotonic_ns\":"+std::to_string(request.triggerNs)+
      ",\"request_id\":"+std::to_string(query.id())+
      ",\"raw_protobuf\":"+Q(query.DebugString())+
      ",\"serialized_request_base64\":"+Q(Base64(query.SerializeAsString()))+"}");
    // Nonblocking publication only. A transport callback never waits for the
    // matching response, shuts transport down, or joins a thread.
    publisher->Publish(query,false);
  }
  void Response(ConstResponsePtr &message) {
    Pending request;
    std::string responseStats;
    long long responseSimNs=-1;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto found=pending.find(message->id());
      if(found==pending.end() || message->request()!="scene_info")return;
      request=found->second;pending.erase(found);
      responseStats=latestStats;responseSimNs=latestSimNs;
    }
    msgs::Scene scene;
    if(message->response()!="success" || message->type()!="gazebo.msgs.Scene" ||
       !scene.ParseFromString(message->serialized_data())) {
      Print("{\"source\":\"client_error\",\"error\":\"Independent connected scene request rejected or native bytes invalid\"}");
      return;
    }
    Print("{\"source\":\"independent_scene\",\"phase\":"+Q(request.phase)+
      ",\"trigger_native_sim_time_ns\":"+std::to_string(request.simNs)+
      ",\"trigger_receive_monotonic_ns\":"+std::to_string(request.triggerNs)+
      ",\"receive_monotonic_ns\":"+std::to_string(Now())+
      ",\"request_id\":"+std::to_string(message->id())+
      ",\"response_id\":"+std::to_string(message->id())+
      ",\"latest_native_stats_at_response_time_ns\":"+std::to_string(responseSimNs)+
      ",\"latest_native_stats_at_response_raw_protobuf\":"+Q(responseStats)+
      ",\"trigger_world_stats_raw_protobuf\":"+Q(request.stats)+
      ",\"query_raw_protobuf\":"+Q(request.request)+
      ",\"response_raw_protobuf\":"+Q(message->DebugString())+
      ",\"native_scene_payload_base64\":"+Q(Base64(message->serialized_data()))+
      ",\"data\":"+Json(scene)+",\"raw_protobuf\":"+Q(scene.DebugString())+"}");
  }
};

'''


def build_active_observer(session, artifacts, prepared):
    """Precisely extend the frozen wire client; leave its SDK guard untouched."""
    base = session.workspace/'observer.cc'
    require(sha(base) == prepared['source_sha256']['observer.cc'], 'Frozen observer source hash differs')
    source = base.read_text(encoding='utf-8')
    require(source.count('receiver.id = static_cast<int>((Now() & 0x3fffffff)+1);') == 1,
            'Normal helper request-ID range no longer proves disjoint acceptance IDs')
    generated = source
    additions = [
        ('#include <vector>\n', '#include <vector>\n#include <functional>\n'),
        ('  int id = 0; std::string requestName; msgs::Response response; msgs::WorldStatistics statistics;\n',
         '  int id = 0; std::string requestName; msgs::Response response; msgs::WorldStatistics statistics;\n'
         '  std::function<void(ConstWorldStatisticsPtr &)> acceptanceStatsHook;\n'
         '  std::function<void(ConstResponsePtr &)> acceptanceResponseHook;\n'),
        ('  void Stats(ConstWorldStatisticsPtr &m) {\n',
         '  void Stats(ConstWorldStatisticsPtr &m) {\n    if(acceptanceStatsHook)acceptanceStatsHook(m);\n'),
        ('  void Response(ConstResponsePtr &m) {\n',
         '  void Response(ConstResponsePtr &m) {\n    if(acceptanceResponseHook)acceptanceResponseHook(m);\n'),
        ('// Compatibility for this exact Classic11 SDK, isolated from teaching plugins.\n',
         ACTIVE_SCENE_CPP+'// Compatibility for this exact Classic11 SDK, isolated from teaching plugins.\n'),
        ('  Receiver receiver;\n', '  Receiver receiver;\n  std::unique_ptr<AcceptanceSceneWatch> acceptanceSceneWatch;\n'),
        ('      receiver.watching = true;\n      auto poses = node->Subscribe("~/pose/info", &Receiver::Poses, &receiver);\n',
         '      acceptanceSceneWatch.reset(new AcceptanceSceneWatch());\n'
         '      acceptanceSceneWatch->publisher=node->Advertise<msgs::Request>("~/request");\n'
         '      publishers.push_back(acceptanceSceneWatch->publisher);\n'
         '      receiver.acceptanceResponseHook=[&](ConstResponsePtr &m){acceptanceSceneWatch->Response(m);};\n'
         '      subscriptions.push_back(node->Subscribe("~/response", &Receiver::Response, &receiver, true));\n'
         '      if(!acceptanceSceneWatch->publisher->WaitForConnection(common::Time(3,0)))\n'
         '        throw std::runtime_error("Independent watcher scene request publisher did not connect");\n'
         '      receiver.acceptanceStatsHook=[&](ConstWorldStatisticsPtr &m){acceptanceSceneWatch->Stats(m);};\n'
         '      Print("{\\\"source\\\":\\\"scene_request_publisher_connected\\\",\\\"receive_monotonic_ns\\\":"+std::to_string(Now())+"}");\n'
         '      receiver.watching = true;\n      auto poses = node->Subscribe("~/pose/info", &Receiver::Poses, &receiver);\n'),
    ]
    for old, new in additions:
        require(generated.count(old) == 1, 'Frozen observer replacement anchor count differs: '+repr(old[:80]))
        generated = generated.replace(old, new, 1)
    marker = 'class MainThreadIOOwner {'
    require(source[source.index(marker):source.index('int main(')] ==
            generated[generated.index(marker):generated.index('int main(')],
            'Acceptance generation changed the standalone main-thread IOManager guard')
    cpp = artifacts/('acceptance-active-observer-'+uuid.uuid4().hex+'.cc')
    cpp.write_text(generated, encoding='utf-8')
    binary = cpp.with_suffix('')
    identity = prepared['build_identity']
    compiler = installed_binary(session, 'g++')
    command = [str(compiler)]+identity['options']+identity['cflags']+[str(cpp), '-o', str(binary)]+identity['libs']
    log = cpp.with_suffix('.compile.log')
    process = spawn_output(session, command, log, 'Independent L34 connected wire observer compilation')
    try:
        require(process.wait(timeout=100) == 0, 'Independent native wire observer compilation failed: '+str(log))
    except BaseException:
        terminate_owned(process)
        raise
    require(binary.is_file() and binary.read_bytes()[:4] == b'\x7fELF', 'Generated independent observer is not actual ELF')
    provenance = {'base_source': str(base), 'base_source_sha256': sha(base), 'generated_source': str(cpp),
                  'generated_source_sha256': sha(cpp), 'compiler': str(compiler), 'compiler_sha256': sha(compiler),
                  'compiler_command': command, 'compiler_version': prepared['compiler'], 'libs': identity['libs'],
                  'gazebo_version': prepared['gazebo_version'], 'sdk_inputs': identity['sdk_inputs'],
                  'native_executable': str(binary), 'native_executable_sha256': sha(binary),
                  'compile_log': str(log), 'replacement_anchor_count': len(additions),
                  'acceptance_request_id_range': [0x50000000, 0x60000001],
                  'normal_helper_request_id_maximum': 0x40000000,
                  'main_thread_io_guard_exactly_unchanged': True}
    save(artifacts/'acceptance-active-observer-provenance.json', provenance)
    return binary, provenance


def require(value, message):
    if not value:
        raise AssertionError(message)


def atomic_json(path, value):
    temporary = path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    save(temporary, value)
    temporary.replace(path)


def batch_worker(report, nonce):
    """Instrument first-step readiness only, then run the actual frozen CLI."""
    sys.path.insert(0, str(Path.cwd()))
    from lab import Lab
    original = Lab.step
    original_text = inspect.getsource(original)
    original_sha = hashlib.sha256(original_text.encode('utf-8')).hexdigest()
    original_file = Path(inspect.getsourcefile(original)).resolve()
    entered = set()

    def gated_step(self, *args, **kwargs):
        token = self.env.get(CUSTOM_TOKEN) if self.env else None
        if token and token not in entered:
            entered.add(token)
            root = self.event_file.parent.resolve()
            require(Path.cwd().resolve()/'build/batches' in root.parents, 'Gate escaped actual batch root')
            start = time.monotonic_ns()
            record = {'nonce': nonce, 'token': token, 'root': str(root), 'entered_monotonic_ns': start,
                      'original_step_source_sha256': original_sha, 'original_lab_source_sha256': sha(original_file),
                      'original_step_args': list(args), 'original_step_kwargs': kwargs,
                      'deadline_monotonic_ns': start+15000000000, 'released': False}
            try:
                dt_ns = kwargs.get('dt_ns', args[1] if len(args) > 1 else 1000000)
                checkpoint = self.checkpoint(dt_ns)
                physics = self.physics()
                require(checkpoint['sim_time_ns'] == checkpoint['iterations'] == 0,
                        'Independent readiness gate requires actual paused zero-time native checkpoint')
                record.update(native_zero_checkpoint=checkpoint, native_physics=physics,
                              ready_monotonic_ns=time.monotonic_ns())
                atomic_json(root/('acceptance-ready-'+nonce+'.json'), record)
                release_path = root/('acceptance-release-'+nonce+'.json')
                while not release_path.is_file():
                    require(time.monotonic_ns() < record['deadline_monotonic_ns'],
                            'Independent observer did not establish actual zero-time readiness within 15 seconds')
                    time.sleep(.02)
                release = json.loads(release_path.read_text())
                require(release['nonce'] == nonce and release['token'] == token and release['root'] == str(root),
                        'Independent readiness release ownership/nonce differs')
                record.update(released=True, release=release, released_monotonic_ns=time.monotonic_ns())
                record['readiness_gate_wall_s'] = (record['released_monotonic_ns']-start)/1e9
            except BaseException as error:
                record['error'] = type(error).__name__+': '+str(error)
                raise
            finally:
                atomic_json(root/('acceptance-gate-'+nonce+'.json'), record)
        # Every original call is made exactly once with its exact original
        # arguments; no native control, reset, pose, physics or result is mocked.
        return original(self, *args, **kwargs)

    Lab.step = gated_step
    sys.argv = ['batch.py', 'run', '--report', str(report)]
    runpy.run_path(str(Path.cwd()/'batch.py'), run_name='__main__')
    return 0


def close(actual, expected, tolerance, message):
    require(math.isfinite(float(actual)) and abs(actual-expected) <= tolerance,
            message+': actual='+str(actual)+', expected='+str(expected))


def stamp(native, field):
    value = first(native, field)
    require(isinstance(value, dict), 'Missing actual native timestamp: '+field)
    return int(first(value, 'sec', 0))*1000000000+int(first(value, 'nsec', 0))


def position(model):
    # msgs.Model embeds msgs.Pose; PosesStamped.pose entries are Pose itself.
    vector = first(first(model, 'pose', model), 'position')
    require(isinstance(vector, dict), 'Native model position absent')
    xyz = [float(first(vector, axis, 0)) for axis in ('x', 'y', 'z')]
    require(all(math.isfinite(v) for v in xyz), 'Nonfinite actual native position')
    return xyz


def name(native):
    return str(first(native, 'name')).split('::')[-1]


def native_stats(row):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'paused') in ('true', 'false'), 'Native pause state missing')
    return {'sim_time_ns': stamp(native, 'sim_time'), 'real_time_ns': stamp(native, 'real_time'),
            'iterations': int(first(native, 'iterations')), 'paused': first(native, 'paused') == 'true'}


def box_size(model):
    links = model.get('link', [])
    require(len(links) == 1 and len(links[0].get('collision', [])) == 1,
            'Actual single-body collision topology differs')
    shape = first(first(first(links[0]['collision'][0], 'geometry'), 'box'), 'size')
    return [float(first(shape, axis)) for axis in ('x', 'y', 'z')]


def scene_check(row, count, phase, dt, ids=None):
    native = parsed_model(row['raw_protobuf'])
    require(first(native, 'name') == WORLD, 'Native scene has wrong world name')
    models = {name(m): m for m in native.get('model', [])}
    cubes = {'cube_%03d' % i for i in range(count)}
    require(set(models) == cubes | {'floor'}, 'Actual native model count/names differ')
    actual_ids = {n: int(first(m, 'id')) for n, m in models.items()}
    require(len(set(actual_ids.values())) == count+1 and min(actual_ids.values()) > 0,
            'Native model identities are absent or duplicated')
    if ids is not None:
        require(actual_ids == ids, 'A physical model was replaced during the trial')
    floor = models['floor']
    require(first(floor, 'is_static') == 'true', 'Actual floor is not static')
    require(max(abs(a-b) for a, b in zip(box_size(floor), (6, 5, .2))) < 1e-10,
            'Actual floor collision geometry differs')
    close(position(floor)[2], -.1, 1e-10, 'Actual floor top must be z=0')
    points, errors = {}, []
    target_time = {'initial': 0., 'freefall': .2, 'final': 1.}[phase]
    n = round(target_time/dt)
    # Explicit semi-implicit ODE estimate, kept separate from measured poses.
    target_z = 2.-9.8*dt*dt*n*(n+1)/2 if phase != 'final' else .1
    for index in range(count):
        key = 'cube_%03d' % index
        model, xyz = models[key], position(models[key])
        require(first(model, 'is_static', 'false') == 'false', 'Actual cube is static')
        require(max(abs(v-.2) for v in box_size(model)) < 1e-10, 'Actual cube edge differs from .2 m')
        link = model['link'][0]
        require(name(link) == 'body' and max(abs(v) for v in position(link)) < 1e-10,
                'Actual canonical body relative origin differs')
        orientation = first(first(link, 'pose'), 'orientation')
        require(max(abs(float(first(orientation, a, 0))) for a in ('x', 'y', 'z')) < 1e-10
                and abs(float(first(orientation, 'w', 1))-1) < 1e-10,
                'Actual canonical body relative rotation differs')
        close(float(first(first(link, 'inertial'), 'mass')), 1., 1e-10, 'Actual cube mass differs')
        expected_xy = ((index % 6-2.5)*.5, (index//6-1.5)*.5)
        require(max(abs(a-b) for a, b in zip(xyz[:2], expected_xy)) < (.01 if phase == 'final' else 1e-8),
                'Actual cube grid position/drift differs')
        error = abs(xyz[2]-target_z)
        require(error < (.015 if phase == 'final' else 3e-6),
                'Actual '+phase+' height differs from independent physical oracle: '+key+' '+str(error))
        points[key] = xyz
        errors.append(error)
    return {'model_ids': actual_ids, 'positions_m': points, 'actual_cube_count': count,
            'height_oracle_m': target_z, 'max_height_error_m': max(errors)}


def process_info(pid):
    """Read only this UID's live native process; PID alone is never ownership."""
    path = Path('/proc')/str(pid)
    if path.stat().st_uid != os.getuid():
        return None
    fields = (path/'stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] == 'Z':
        return None
    env = dict(v.split('=', 1) for v in (path/'environ').read_bytes().decode().split('\0') if '=' in v)
    return {'pid': int(pid), 'start_ticks': fields[19], 'uid': path.stat().st_uid,
            'exe': str((path/'exe').resolve()),
            'argv': [v.decode(errors='replace') for v in (path/'cmdline').read_bytes().split(b'\0') if v],
            'env': env}


def installed_binary(session, executable):
    found = shutil.which(executable, path=session.env.get('PATH'))
    require(found is not None, 'Actual installed native executable absent: '+executable)
    return Path(found).resolve()


def actual_process(info, artifacts, label, installed):
    native = Path(info['exe'])
    require(native.read_bytes()[:4] == b'\x7fELF' and os.path.samefile(str(native), str(installed)),
            'Actual process is not the exact installed native gzserver ELF')
    require('-u' in info['argv'] and '-p' not in info['argv'] and '-s' not in info['argv'],
            'Actual experiment must be paused normal physics without injected process plugin')
    maps = artifacts/(label+'-'+str(info['pid'])+'.maps')
    maps.write_bytes(Path('/proc', str(info['pid']), 'maps').read_bytes())
    libraries = sorted(set(line.split()[-1] for line in maps.read_text().splitlines()
                           if '/libgazebo_' in line and '.so' in line))
    require(any('libgazebo_physics.so' in p for p in libraries)
            and any('libgazebo_transport.so' in p for p in libraries), 'Native physics/transport DSOs not mapped')
    return {'pid': info['pid'], 'start_ticks': info['start_ticks'], 'uid': info['uid'],
            'argv': info['argv'], 'executable': info['exe'], 'executable_sha256': sha(native),
            'maps': str(maps), 'maps_sha256': sha(maps), 'native_libraries': {p: sha(p) for p in libraries},
            'framework_token': info['env'].get('GAZEBOTEACH_SESSION_TOKEN'),
            'token': info['env'].get(CUSTOM_TOKEN), 'master_uri': info['env'].get('GAZEBO_MASTER_URI'),
            'ign_partition': info['env'].get('IGN_PARTITION'), 'gz_partition': info['env'].get('GZ_PARTITION'),
            'home': info['env'].get('HOME'), 'first_observed_monotonic_ns': time.monotonic_ns()}


def spawn_output(session, argv, path, label, environment=None):
    command = shlex.join([str(v) for v in argv])+' > '+shlex.quote(str(path))+' 2>&1'
    if environment is None:
        return session._spawn(label, ['bash', '--noprofile', '--norc', '-c', command])
    # The framework serializes spawning and records our token/starttime. Give a
    # subscriber the child's transport, but never its cleanup ownership token.
    with session.lock:
        original = session.env
        try:
            session.env = environment
            return session._spawn(label, ['bash', '--noprofile', '--norc', '-c', command])
        finally:
            session.env = original


def wire(session, artifacts, label, mode, *arguments):
    path = artifacts/(label+'-'+uuid.uuid4().hex+'.jsonl')
    process = spawn_output(session, [session.workspace/'build/batch_observer', mode, *arguments], path,
                           'Independent L34 native '+mode)
    try:
        require(process.wait(timeout=18) == 0, 'Independent native query failed: '+str(path))
    except BaseException:
        terminate_owned(process)
        raise
    rows, diagnostics = events(path)
    source = {'stats': 'world_stats', 'world': 'world', 'entity': 'entity', 'physics': 'physics'}[mode]
    chosen = [r for r in rows if r.get('source') == source]
    require(len(chosen) == 1, 'Expected one actual native '+source)
    return dict(chosen[0], artifact=str(path), diagnostics=diagnostics)


class LiveTrials:
    """Actual native readiness, bounded observer reconnect, then unchanged steps."""
    def __init__(self, session, artifacts, observer, nonce):
        self.session, self.artifacts = session, artifacts
        self.observer, self.nonce = observer, nonce
        self.environments = {}
        self.server_executable = installed_binary(session, 'gzserver')
        self.client_executable = installed_binary(session, 'gzclient')
        self.stop, self.rows, self.errors, self.watchers = threading.Event(), {}, [], []
        self.thread = threading.Thread(target=self.monitor, daemon=True)

    def start(self):
        self.thread.start()

    def monitor(self):
        while not self.stop.is_set():
            try:
                self.poll()
            except Exception as error:
                self.errors.append(type(error).__name__+': '+str(error))
            self.stop.wait(.03)

    def start_watcher(self, token):
        proof = self.rows[token]
        identity = token+'-'+uuid.uuid4().hex
        path = self.artifacts/('native-live-'+identity+'.jsonl')
        stop = self.artifacts/('native-live-'+identity+'.stop')
        require(not path.exists() and not stop.exists(), 'Independent watcher reused artifact paths')
        environment = dict(self.environments[token])
        environment[CUSTOM_TOKEN] = 'acceptance-'+uuid.uuid4().hex
        process = spawn_output(self.session, [self.observer, 'watch', '--seconds', '120', '--stop-file', stop],
                               path, 'Independent L34 real readiness observer', environment)
        attempt = {'id': identity, 'wire_file': str(path), 'stop_file': str(stop),
                   'observer_custom_token': environment[CUSTOM_TOKEN], 'started_monotonic_ns': time.monotonic_ns(),
                   'successful': False}
        proof['watch_attempts'].append(attempt)
        proof['current_attempt'] = identity
        self.watchers.append((token, process, path, stop, attempt))

    def release_if_ready(self, token, observed, attempt):
        proof = self.rows[token]
        if proof.get('handshake_live'):
            return
        scenes = [r for r in observed if r.get('source') == 'independent_scene' and r.get('phase') == 'initial']
        if not (any(r.get('source') == 'client_ready' for r in observed) and
                any(r.get('source') == 'scene_request_publisher_connected' for r in observed) and len(scenes) == 1):
            return
        scene = scenes[0]
        subsequent = [r for r in observed if r.get('source') == 'independent_native_stats'
                      and r['receive_monotonic_ns'] > scene['receive_monotonic_ns']]
        if not subsequent:
            return
        require(scene['trigger_native_sim_time_ns'] == scene['latest_native_stats_at_response_time_ns'] == 0,
                'Initial independent native scene has wrong actual phase')
        actual = native_stats(subsequent[0])
        require(actual['paused'] and actual['sim_time_ns'] == actual['iterations'] == 0,
                'Actual native observer did not handshake while held paused at zero')
        ready = proof['readiness_marker']
        count = len(ready['native_zero_checkpoint']['positions_m'])
        dt = float(ready['native_physics']['max_step_size'])
        checked = scene_check(scene, count, 'initial', dt)
        require(checked['positions_m'] == ready['native_zero_checkpoint']['positions_m'],
                'Actual independent initial scene and real course readiness checkpoint disagree')
        identity = process_info(proof['pid'])
        require(identity and identity['start_ticks'] == proof['start_ticks']
                and identity['env'].get(CUSTOM_TOKEN) == token, 'Native PID identity changed before readiness release')
        now = time.monotonic_ns()
        require(now < ready['deadline_monotonic_ns'], 'Real observer handshake exceeded bounded readiness deadline')
        release = {'nonce': self.nonce, 'token': token, 'root': str(Path(proof['world']).parent),
                   'released_monotonic_ns': now, 'watch_attempt': attempt['id'],
                   'actual_initial_model_ids': checked['model_ids'], 'actual_stats': actual}
        atomic_json(Path(proof['world']).parent/('acceptance-release-'+self.nonce+'.json'), release)
        attempt['successful'] = True
        proof.update(handshake_live=True, handshake_observed_monotonic_ns=now,
                     wire_file=attempt['wire_file'], stop_file=attempt['stop_file'],
                     observer_custom_token=attempt['observer_custom_token'], readiness_release=release)

    def poll(self):
        active = set()
        root = (self.session.workspace/'build/batches').resolve()
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():
                continue
            try:
                info = process_info(int(directory.name))
                if not info or not info['env'].get(CUSTOM_TOKEN):
                    continue
                if info['env'].get('GAZEBOTEACH_SESSION_TOKEN') != self.session.token:
                    continue
                token = info['env'][CUSTOM_TOKEN]
                if os.path.samefile(info['exe'], str(self.client_executable)):
                    raise AssertionError('An actual batch child created gzclient')
                if not os.path.samefile(info['exe'], str(self.server_executable)):
                    continue
                world = Path(info['argv'][-1]).resolve()
                require(root in world.parents, 'Actual child world escaped its batch root')
                active.add(token)
                if token not in self.rows:
                    marker = world.parent/('acceptance-ready-'+self.nonce+'.json')
                    if not marker.is_file():
                        continue
                    ready = json.loads(marker.read_text())
                    require(ready['nonce'] == self.nonce and ready['token'] == token
                            and ready['root'] == str(world.parent), 'Actual course readiness marker ownership differs')
                    require(ready['original_lab_source_sha256'] == sha(self.session.workspace/'lab.py'),
                            'Gate did not wrap the actual frozen course Lab source')
                    require(ready['native_zero_checkpoint']['sim_time_ns'] == ready['native_zero_checkpoint']['iterations'] == 0,
                            'Actual course readiness marker is not a native zero-time checkpoint')
                    maps = Path('/proc', str(info['pid']), 'maps').read_text()
                    # /proc/exe changes at exec, before the dynamic loader has
                    # necessarily mapped the actual SDK. Poll for readiness.
                    if 'libgazebo_physics.so' not in maps or 'libgazebo_transport.so' not in maps:
                        continue
                    proof = actual_process(info, self.artifacts, 'trial-'+token, self.server_executable)
                    proof.update(world=str(world), world_sha256=sha(world),
                                 observer=str(self.observer), observer_sha256=sha(self.observer),
                                 readiness_marker=ready, watch_attempts=[])
                    self.environments[token] = dict(info['env'])
                    self.rows[token] = proof
                    self.start_watcher(token)
                self.rows[token]['last_observed_monotonic_ns'] = time.monotonic_ns()
            except (OSError, UnicodeError, ValueError, IndexError):
                continue
        for token, process, path, stop, attempt in list(self.watchers):
            proof = self.rows[token]
            if token in active and proof['current_attempt'] == attempt['id']:
                # Preserve failed native connections as attempts. Only a real
                # complete zero-time response/stats handshake releases physics.
                raw = path.read_text() if path.is_file() else ''
                observed = []
                for line in raw.splitlines():
                    try:
                        row = json.loads(line)
                        if isinstance(row, dict):
                            observed.append(row)
                    except ValueError:
                        pass
                if not proof.get('handshake_live'):
                    error_rows = [r for r in observed if r.get('source') in ('client_error', 'decode_error')]
                    # Once a stop has been requested, a late queued message
                    # cannot turn that retiring attempt into the live observer.
                    if not attempt.get('handshake_timeout') and not error_rows and process.poll() is None:
                        self.release_if_ready(token, observed, attempt)
                    if not proof.get('handshake_live'):
                        now = time.monotonic_ns()
                        if error_rows or process.poll() is not None or now-attempt['started_monotonic_ns'] > 3000000000:
                            if not stop.exists():
                                stop.write_text('Read-only observer handshake retry; physics remains held at zero\n')
                            attempt.update(handshake_timeout=True, native_errors=error_rows)
                            if process.poll() is not None:
                                attempt['returncode'] = process.returncode
                                require(now < proof['readiness_marker']['deadline_monotonic_ns']
                                        and len(proof['watch_attempts']) < 4, 'Bounded actual observer readiness attempts exhausted')
                                self.start_watcher(token)
            if token not in active:
                proof.setdefault('native_exit_observed_monotonic_ns', time.monotonic_ns())
                if not stop.exists():
                    stop.write_text('Independent native trial observation complete\n', encoding='utf-8')

    def finish(self):
        self.stop.set()
        self.thread.join(timeout=10)
        require(not self.thread.is_alive(), 'Independent native monitor did not join')
        self.poll()
        for token, process, path, stop, attempt in self.watchers:
            if process.poll() is None:
                stop.write_text('Independent L34 observation complete\n', encoding='utf-8')
            try:
                process.wait(timeout=10)
            except BaseException:
                terminate_owned(process)
                raise
            attempt.update(returncode=process.returncode, wire_sha256=sha(path),
                           finished_monotonic_ns=time.monotonic_ns())
            if attempt['successful']:
                self.rows[token]['observer_returncode'] = process.returncode
                self.rows[token]['wire_sha256'] = sha(path)
        save(self.artifacts/'independent-live-processes.json', {'trials': self.rows, 'errors': self.errors})


def preview(session, artifacts, label):
    start = native_stats(wire(session, artifacts, label+'-before', 'stats'))
    world = wire(session, artifacts, label+'-scene', 'world')
    body = wire(session, artifacts, label+'-entity', 'entity', '--name', 'cube_000')
    end = native_stats(wire(session, artifacts, label+'-after', 'stats'))
    require(start == end and end['paused'] and end['sim_time_ns'] == end['iterations'] == 0,
            'Preview was not a stable paused zero-time world')
    checked = scene_check(world, 8, 'initial', .001)
    close(max(abs(a-b) for a, b in zip(position(parsed_model(body['raw_protobuf'])), checked['positions_m']['cube_000'])),
          0., 1e-10, 'Independent native scene/entity disagree')
    physics = wire(session, artifacts, label+'-physics', 'physics')
    native = parsed_model(physics['raw_protobuf'])
    close(float(first(native, 'max_step_size')), .001, 1e-12, 'Preview timestep differs')
    close(float(first(native, 'real_time_update_rate')), 1000., 1e-9, 'Preview rate cap differs')
    info = process_info(session.server.pid)
    require(info and info['env'].get('GAZEBOTEACH_SESSION_TOKEN') == session.token, 'Preview ownership differs')
    proof = actual_process(info, artifacts, label+'-preview', installed_binary(session, 'gzserver'))
    return {'stats': end, 'scene': checked, 'native_scene': world, 'native_entity': body,
            'native_physics': physics, 'actual_process': proof}


def journal(path):
    rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]
    require(rows, 'Actual native query journal missing')
    for row in rows:
        require(type(row['started_monotonic_ns']) is int and type(row['finished_monotonic_ns']) is int
                and row['finished_monotonic_ns'] > row['started_monotonic_ns'], 'Native helper wall interval invalid')
        decoded = []
        for line in row['stdout'].splitlines():
            try:
                value = json.loads(line)
                if isinstance(value, dict):
                    decoded.append(value)
            except ValueError:
                pass
        require(decoded == row['events'], 'Native journal event mirror differs from original helper stdout')
    return rows


def live_check(proof, trial, ids):
    require(proof.get('handshake_live') and proof['observer_returncode'] == 0,
            'No live readiness+statistics handshake/normal cleanup for this actual trial')
    rows, diagnostics = events(proof['wire_file'])
    stats_rows = [native_stats(row) for row in rows if row.get('source') == 'world_stats']
    dt_ns = round(trial['dt_s']*1e9)
    require(stats_rows and all(row['paused'] and row['sim_time_ns'] == row['iterations']*dt_ns for row in stats_rows),
            'Independently observed native clock/iterations/pause differ')
    require({0, 200000000, 1000000000} <= {r['sim_time_ns'] for r in stats_rows},
            'Independent native subscriber missed an actual physical endpoint')
    require(all(b['iterations'] >= a['iterations'] for a, b in zip(stats_rows, stats_rows[1:])),
            'Actual trial clock moved backwards')
    by_time = {}
    for row in rows:
        if row.get('source') != 'pose':
            continue
        native = parsed_model(row['raw_protobuf'])
        ns = stamp(native, 'time')
        observed = {name(p): p for p in native.get('pose', []) if name(p) in ids and name(p).startswith('cube_')}
        if observed:
            by_time.setdefault(ns, []).append(observed)
    require(any(v.get('source') == 'scene_request_publisher_connected' for v in rows),
            'Independent active watcher never connected its native request publisher')
    endpoints, used_ids = [], set()
    # pose/info publishes dirty model poses, so a settled cube need not emit a
    # fresh pose message at exactly 1s. Query the actual native scene at each
    # paused endpoint instead of assigning a later timestamp to old pose bytes.
    for phase, ns in (('initial', 0), ('freefall', 200000000), ('final', 1000000000)):
        expected = trial[phase]['positions_m']
        requests = [v for v in rows if v.get('source') == 'independent_scene_request' and v.get('phase') == phase]
        scenes = [v for v in rows if v.get('source') == 'independent_scene' and v.get('phase') == phase]
        require(len(requests) == len(scenes) == 1, 'Missing unique independent actual '+phase+' request/response')
        request, scene = requests[0], scenes[0]
        request_id = request['request_id']
        require(0x50000000 <= request_id < 0x60000002 and request_id not in used_ids
                and request_id == scene['request_id'] == scene['response_id'],
                'Independent native scene request/response identity differs/collides')
        used_ids.add(request_id)
        require(request['trigger_native_sim_time_ns'] == scene['trigger_native_sim_time_ns'] == ns,
                'Independent request phase differs from actual native clock trigger')
        query = parsed_model(scene['query_raw_protobuf'])
        # DebugString serialized_data contains C octal escapes, unlike the
        # JSON-compatible printable header strings. Keep original payload bytes
        # separately and decode only the native envelope's four header fields.
        response_lines = scene['response_raw_protobuf'].splitlines()
        require(sum(line.startswith('serialized_data:') for line in response_lines) == 1,
                'Native Scene response envelope lacks one original payload')
        response = parsed_model('\n'.join(line for line in response_lines if not line.startswith('serialized_data:')))
        require(int(first(query, 'id')) == int(first(response, 'id')) == request_id
                and first(query, 'request') == first(response, 'request') == 'scene_info'
                and first(response, 'response') == 'success' and first(response, 'type') == 'gazebo.msgs.Scene',
                'Independent native response is not the matching successful Scene request')
        trigger = native_stats({'raw_protobuf': scene['trigger_world_stats_raw_protobuf']})
        latest = native_stats({'raw_protobuf': scene['latest_native_stats_at_response_raw_protobuf']})
        following = [v for v in rows if v.get('source') == 'independent_native_stats'
                     and v['receive_monotonic_ns'] > scene['receive_monotonic_ns']]
        require(following, 'Missing fresh native statistics after independent '+phase+' scene response')
        after = native_stats(following[0])
        require(trigger['paused'] and latest['paused'] and after['paused']
                and trigger['sim_time_ns'] == latest['sim_time_ns'] == after['sim_time_ns'] == ns
                and trigger['iterations'] == latest['iterations'] == after['iterations'] == round(ns/dt_ns)
                and scene['latest_native_stats_at_response_time_ns'] == ns,
                'Independent scene response crossed its actual paused native phase')
        payload = base64.b64decode(scene['native_scene_payload_base64'], validate=True)
        require(payload, 'Native Scene original Response payload bytes absent')
        checked = scene_check(scene, trial['count'], phase, trial['dt_s'], ids)
        error = max(abs(a-b) for key, p in checked['positions_m'].items() for a, b in zip(p, expected[key]))
        require(error < 1e-8, 'Independent actual scene query and course native scene disagree at '+phase)
        endpoints.append({'phase': phase, 'trigger_native_time_ns': ns, 'max_pose_error_m': error,
                          'request_id': request_id, 'trigger_stats': trigger, 'stats_at_response': latest,
                          'fresh_stats_after_response': after, 'response_receive_monotonic_ns': scene['receive_monotonic_ns'],
                          'native_scene_payload_sha256': hashlib.sha256(payload).hexdigest()})
    falling_frames = []
    for ns, candidates in by_time.items():
        if not 0 < ns <= 200000000:
            continue
        ticks = round(ns/dt_ns)
        expected_z = 2.-9.8*trial['dt_s']**2*ticks*(ticks+1)/2
        for poses in candidates:
            if set(poses) != set(trial['initial']['positions_m']):
                continue
            require(all(int(first(v, 'id')) == ids[k] for k, v in poses.items()), 'Actual native pose stream model ID differs')
            error = max(abs(position(p)[2]-expected_z) for p in poses.values())
            require(error < 3e-6, 'Independently observed moving native pose differs from real freefall')
            falling_frames.append({'native_sim_time_ns': ns, 'actual_cube_count': len(poses), 'max_freefall_error_m': error})
    require(falling_frames, 'Independent native pose stream contains no actual falling frame')
    return {'passed': True, 'statistics_count': len(stats_rows), 'pose_source_times': sorted(by_time),
            'native_endpoints': endpoints, 'independent_falling_pose_frames': falling_frames,
            'wire_diagnostics': diagnostics, 'actual_process': proof}


def trial_check(trial, proof, parent_start, parent_end):
    count, dt = trial['count'], trial['dt_s']
    root = Path(trial['directory'])
    gate_files = list(root.glob('acceptance-gate-*.json'))
    require(len(gate_files) == 1, 'Actual trial has no unique instrumented first-step gate record')
    gate = json.loads(gate_files[0].read_text())
    require(gate['released'] and gate['nonce'] == proof['readiness_marker']['nonce']
            and gate['token'] == proof['token'] and gate['root'] == str(root)
            and gate['original_step_args'] == [round(.2/dt), round(dt*1e9)] and not gate['original_step_kwargs']
            and gate['entered_monotonic_ns'] <= gate['ready_monotonic_ns'] < gate['released_monotonic_ns']
            <= gate['deadline_monotonic_ns'], 'Instrumented first-step gate ownership, arguments or bounded release differs')
    close(gate['readiness_gate_wall_s'], (gate['released_monotonic_ns']-gate['entered_monotonic_ns'])/1e9,
          1e-12, 'Instrumented actual readiness gap measurement differs')
    require(count in CONFIG['counts'] and dt in CONFIG['step_sizes_s'] and trial['repeat'] in (1, 2)
            and trial['seed'] == CONFIG['seed'] and trial['passed'], 'Unexpected/failed actual trial inputs')
    require(trial == json.loads((root/'result.json').read_text()), 'Batch and per-trial result differ')
    require(sha(root/'world.sdf') == trial['input_sha256'] == proof['world_sha256'], 'Actual child input SDF bytes differ')
    xml = ET.parse(str(root/'world.sdf'))
    require(not list(xml.iter('plugin')) and not list(xml.iter('uri')), 'Actual physical world injects plugins/online assets')
    ownership = trial['ownership']
    require(ownership == json.loads((root/'ownership.json').read_text()) and ownership['cleaned'] and not ownership['remaining'],
            'Actual trial ownership cleanup record differs/is incomplete')
    for key in ('token', 'master_uri', 'ign_partition', 'gz_partition'):
        require(ownership[key] == proof[key], 'Independent actual process and ownership differ: '+key)
    require(proof['home'] == str(root/'home') and proof['token'] != proof['observer_custom_token'],
            'Trial HOME is shared or independent observer shares trial cleanup ownership')
    servers = [p for p in ownership['processes'] if p['label'] == 'gzserver']
    require(len(servers) == 1 and servers[0]['pid'] == proof['pid'] and servers[0]['start_ticks'] == proof['start_ticks'],
            'Recorded native server PID/starttime differs from independent live identity')
    require('--seed' in proof['argv'] and int(proof['argv'][proof['argv'].index('--seed')+1]) == CONFIG['seed'],
            'Actual gzserver did not receive the recorded native RNG seed')
    wrappers = journal(root/'native-events.jsonl')
    successful = [r for r in wrappers if r['returncode'] == 0]
    native_rows = [event for r in successful for event in r['events']]
    physics_rows = [r for r in native_rows if r.get('source') == 'physics']
    require(physics_rows, 'No actual native Physics response')
    physics = parsed_model(physics_rows[-1]['raw_protobuf'])
    close(float(first(physics, 'max_step_size')), dt, 1e-12, 'Native trial timestep differs')
    close(float(first(physics, 'real_time_update_rate')), 0., 1e-12, 'Native trial rate cap is enabled')
    gravity = first(physics, 'gravity')
    require(max(abs(float(first(gravity, k, 0))-v) for k, v in zip(('x', 'y', 'z'), (0, 0, -9.8))) < 1e-10,
            'Actual native gravity differs')
    phase_for_time = {0: 'initial', 200000000: 'freefall', 1000000000: 'final'}
    checked, ids = {}, None
    # Native helper journal sequence brackets each scene by two native stats
    # queries. Reconstruct that coherence independently of checkpoint mirrors.
    for index, wrapper in enumerate(successful):
        if wrapper['mode'] != 'world' or index == 0 or index+1 >= len(successful):
            continue
        before, after = successful[index-1], successful[index+1]
        if before['mode'] != 'stats' or after['mode'] != 'stats':
            continue
        a = native_stats(next(r for r in before['events'] if r.get('source') == 'world_stats'))
        b = native_stats(next(r for r in after['events'] if r.get('source') == 'world_stats'))
        require(a == b and a['paused'] and a['sim_time_ns'] == a['iterations']*round(dt*1e9),
                'Actual native scene is not bracketed by a coherent paused clock')
        phase = phase_for_time.get(a['sim_time_ns'])
        require(phase is not None, 'Actual native checkpoint has unexpected physical duration')
        event = next(r for r in wrapper['events'] if r.get('source') == 'world')
        value = scene_check(event, count, phase, dt, ids)
        ids = value['model_ids']
        require(trial[phase]['positions_m'] == value['positions_m'] and trial[phase]['sim_time_ns'] == a['sim_time_ns']
                and trial[phase]['iterations'] == a['iterations'], 'Course checkpoint mirror differs from actual native messages')
        checked[phase] = value
    require(set(checked) == {'initial', 'freefall', 'final'}, 'Native journals do not prove all three physical phases')
    controls = [r for r in native_rows if r.get('source') == 'control_sent']
    require(len(controls) == 2, 'Trial did not issue exactly two finite physical control publications')
    for row, seconds in zip(controls, (.2, .8)):
        value = parsed_model(row['raw_protobuf'])
        require(first(value, 'pause') == 'true' and int(first(value, 'multi_step')) == round(seconds/dt)
                and not value.get('reset'), 'Actual finite control request differs')
    t0, ready, control, end = [trial[k] for k in ('server_spawn_monotonic_ns', 'ready_monotonic_ns',
                                                'control_start_monotonic_ns', 'endpoint_monotonic_ns')]
    require(all(type(v) is int for v in (t0, ready, control, end))
            and parent_start <= t0 < ready <= control < end <= parent_end, 'Reported timing boundaries leave independent parent interval')
    require(ready <= gate['entered_monotonic_ns'] <= gate['ready_monotonic_ns']
            <= gate['released_monotonic_ns'] <= control
            and gate['readiness_gate_wall_s'] <= (control-ready)/1e9,
            'Instrumented readiness gap leaves inclusive end-to-end interval or overlaps original control measurement')
    require(t0 <= proof['first_observed_monotonic_ns'] < end
            and end <= proof['native_exit_observed_monotonic_ns'], 'Reported physical interval leaves independently observed native process lifetime')
    for key, expected in (('startup_readiness_wall_s', (ready-t0)/1e9),
                          ('measured_control_wall_s', (end-control)/1e9),
                          ('end_to_end_wall_s', (end-t0)/1e9),
                          ('end_to_end_sim_throughput', 1e9/(end-t0))):
        require(float(trial[key]) > 0, 'Wall timing/throughput must be positive')
        close(trial[key], expected, 1e-12, 'Incorrect actual measured timing: '+key)
    require(trial['cleanup_wall_s'] > 0 and all(t0 <= r['started_monotonic_ns'] < r['finished_monotonic_ns'] <= end
            for r in wrappers), 'Actual helper wall overhead is not contained in measured inclusive interval')
    close(trial['sim_seconds'], 1., 1e-12, 'Fixed simulation duration differs')
    require(trial['iterations'] == round(1/dt), 'dt-only change did not change actual physical tick count')
    return {'passed': True, 'trial': trial['trial'], 'parameters': {'count': count, 'dt_s': dt, 'repeat': trial['repeat']},
            'input_sha256': trial['input_sha256'], 'physical_phases': checked,
            'live_native_evidence': live_check(proof, trial, ids), 'native_query_count': len(wrappers),
            'instrumented_readiness_gate': gate,
            'end_to_end_wall_s': trial['end_to_end_wall_s'], 'end_to_end_sim_throughput': trial['end_to_end_sim_throughput'],
            'native_real_time_used_as_divisor': False}


def preserve(session, artifacts):
    """Preserve partial/error evidence before temporary Storage disappears."""
    targets = []
    for filename in ('parameters.json', 'world.sdf', 'build/batch_lab.json'):
        source = session.workspace/filename
        if source.is_file():
            target = artifacts/'course-workspace'/filename
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(source), str(target))
            targets.append({'source': str(source), 'preserved': str(target), 'sha256': sha(target)})
    total = 0
    for source in sorted((session.workspace/'build/batches').rglob('*')):
        if not source.is_file() or source.is_symlink() or source.suffix not in ('.json', '.jsonl', '.csv', '.sdf', '.out', '.log'):
            continue
        require(session.workspace.resolve() in source.resolve().parents, 'Diagnostic path escaped owned workspace')
        total += source.stat().st_size
        require(total <= 256*1024*1024 and len(targets) < 8192, 'Diagnostic preservation exceeded bounded bytes/files')
        target = artifacts/'course-workspace'/source.relative_to(session.workspace)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(source), str(target))
        targets.append({'source': str(source), 'preserved': str(target), 'sha256': sha(target)})
    save(artifacts/'preservation-manifest.json', targets)
    return {'manifest': str(artifacts/'preservation-manifest.json'), 'files': len(targets), 'trial_bytes': total}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--batch-worker', action='store_true')
    parser.add_argument('--gate-nonce')
    args = parser.parse_args()
    if args.batch_worker:
        require(args.gate_nonce and len(args.gate_nonce) == 32, 'Instrumented CLI worker requires a fresh gate nonce')
        return batch_worker(args.report.resolve(), args.gate_nonce)
    output = args.report.expanduser().resolve()
    artifacts = output.with_name(output.stem+'-artifacts')
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    require(not errors, 'Catalog errors: '+repr(errors))
    lesson = next(v for v in lessons if v.key == 'classic11-noetic:34')
    source, started, session, monitor = hashes(lesson.folder), time.monotonic(), None, None
    result = {'passed': False, 'lesson': lesson.key, 'scenarios': [], 'source_sha256': source,
              'acceptance_source_sha256': sha(Path(__file__)),
              'test_algorithm': __doc__, 'graphical_checks_performed': False,
              'official_sdk_sources': [SDK_BASE+p for p in ('gazebo/physics/World.cc', 'gazebo/physics/Model.cc',
                  'gazebo/physics/Link.cc', 'gazebo/msgs/world_stats.proto')],
              'measurement_environment': 'Instrumented actual batch CLI: first-step readiness gates and independent native subscriber/request clients add measured wall overhead. End-to-end time includes this gap, native helper queries, a paused headless preview and uncontrolled desktop/background load. Bare CLI benchmark timing and isolated engine speed/load ordering are not claimed.'}
    with tempfile.TemporaryDirectory(prefix='gazeboteach-l34-independent-') as temporary:
        try:
            storage = Storage(Path(temporary)/'preview')
            workspace = storage.prepare(lesson)
            save(workspace/'parameters.json', CONFIG)
            session = LessonSession(ROOT, lesson, storage=storage)
            session.start(gui=False, terminals=False)
            prepared = json.loads((workspace/'build/batch_lab.json').read_text())
            require(prepared['parameters'] == CONFIG and prepared['config_sha256'] == sha(workspace/'parameters.json'),
                    'Prepared batch parameters differ')
            require(all(source[p] == digest == sha(workspace/p) for p, digest in prepared['source_sha256'].items()),
                    'Prepared native/course source differs from original templates')
            require(prepared['observer_sha256'] == sha(workspace/'build/batch_observer')
                    and prepared['world_sha256'] == sha(workspace/'world.sdf'), 'Prepared observer/world hash differs')
            binary = workspace/'build/batch_observer'
            require(binary.read_bytes()[:4] == b'\x7fELF', 'Native observer is not actual compiled ELF')
            result['prepared_input'] = prepared
            before = preview(session, artifacts, 'before-batch')
            active_observer, observer_provenance = build_active_observer(session, artifacts, prepared)
            result['independent_active_observer'] = observer_provenance
            gate_nonce = uuid.uuid4().hex
            monitor = LiveTrials(session, artifacts, active_observer, gate_nonce)
            monitor.start()
            batch_output = artifacts/'public-batch-report.json'
            batch_log = artifacts/'public-batch-command.log'
            start = time.monotonic_ns()
            process = spawn_output(session, ['python3', Path(__file__).resolve(), '--batch-worker',
                                   '--gate-nonce', gate_nonce, '--report', batch_output], batch_log,
                                   'Actual L34 batch CLI with explicit first-step observer readiness instrumentation')
            try:
                status = process.wait(timeout=480)
            except BaseException:
                terminate_owned(process)
                raise
            end = time.monotonic_ns()
            monitor.finish()
            require(status == 0 and batch_output.is_file(), 'Actual public batch failed: '+str(batch_log))
            raw = json.loads(batch_output.read_text())
            require(raw['passed'] and raw['parameters'] == CONFIG and raw['provenance'] == prepared,
                    'Public batch report/provenance differs or actual batch failed')
            require(not monitor.errors, 'Independent live monitor errors: '+repr(monitor.errors))
            require(len(raw['trials']) == len(monitor.rows) == 8, 'Actual eight-trial native process count differs')
            actual = {(r['count'], r['dt_s'], r['repeat']) for r in raw['trials']}
            require(actual == {(n, dt, repeat) for n in (8, 24) for dt in (.001, .002) for repeat in (1, 2)},
                    'Actual batch factorial grid differs')
            require(len({r['ownership']['token'] for r in raw['trials']}) == 8
                    and len({r['ownership']['ign_partition'] for r in raw['trials']}) == 8
                    and len({r['ownership']['gz_partition'] for r in raw['trials']}) == 8,
                    'Fresh trial tokens/partitions were reused')
            for trial in raw['trials']:
                token = trial['ownership']['token']
                require(token in monitor.rows, 'Actual trial has no independent live process proof')
                require(trial['ownership']['master_uri'] != session.env['GAZEBO_MASTER_URI'], 'Batch trial shared preview master')
                checked = trial_check(trial, monitor.rows[token], start, end)
                result['scenarios'].append(checked)
                print(json.dumps({'scenario': trial['trial'], 'passed': True}), flush=True)
            csv_rows = list(csv.DictReader(Path(raw['csv_file']).open(encoding='utf-8', newline='')))
            require(len(csv_rows) == 8, 'Actual CSV does not contain eight trials')
            for csv_row, trial in zip(csv_rows, raw['trials']):
                for key, value in csv_row.items():
                    expected = trial[key]
                    require(value == str(expected), 'CSV scalar differs from corresponding measured trial: '+key)
            repeated = []
            for n in (8, 24):
                for dt in (.001, .002):
                    rows = [r for r in raw['trials'] if r['count'] == n and r['dt_s'] == dt]
                    require(len(rows) == 2 and rows[0]['input_sha256'] == rows[1]['input_sha256'], 'Repeated exact input bytes differ')
                    maximum = max(abs(p[a]-rows[0][phase]['positions_m'][key][a])
                                  for phase in ('freefall', 'final') for key, p in rows[1][phase]['positions_m'].items() for a in range(3))
                    require(maximum < 1e-5, 'Independent repeated actual physical outcomes exceed 10um tolerance')
                    repeated.append({'count': n, 'dt_s': dt, 'maximum_actual_xyz_difference_m': maximum})
            require(sum(r['end_to_end_wall_s']+r['cleanup_wall_s'] for r in raw['trials']) <= (end-start)/1e9,
                    'Summed serial measured intervals exceed independent parent duration')
            after = preview(session, artifacts, 'after-batch')
            require(before['stats'] == after['stats'] and before['scene'] == after['scene']
                    and before['actual_process']['pid'] == after['actual_process']['pid']
                    and before['actual_process']['start_ticks'] == after['actual_process']['start_ticks'],
                    'Actual batch changed/replaced its paused preview world')
            result.update(passed=True, preview_isolation={'passed': True, 'before': before, 'after': after},
                          repeated_actual_outcomes=repeated, actual_csv_rows=len(csv_rows),
                          independent_public_command_wall_s=(end-start)/1e9,
                          course_total_wall_s=raw['total_wall_s'], measurement=raw['measurement'],
                          instrumented_cli=True, readiness_gate_nonce=gate_nonce,
                          measured_readiness_gate_total_s=sum(r['instrumented_readiness_gate']['readiness_gate_wall_s'] for r in result['scenarios']),
                          live_native_trial_processes=monitor.rows)
        except Exception as error:
            result['error'] = type(error).__name__+': '+str(error)
            result['traceback'] = traceback.format_exc()
        finally:
            result['cleanup_errors'] = []
            if monitor is not None and monitor.thread.is_alive():
                try:
                    monitor.finish()
                except Exception as error:
                    result['cleanup_errors'].append('Independent monitor cleanup: '+repr(error))
            if session is not None:
                try:
                    result['preserved_evidence'] = preserve(session, artifacts)
                    if session.log_path.is_file():
                        shutil.copy2(str(session.log_path), str(artifacts/'before-cleanup-runtime.log'))
                except Exception as error:
                    result['cleanup_errors'].append('Evidence preservation: '+repr(error))
                try:
                    session.stop(reason='Independent L34 formal real batch acceptance cleanup')
                    shutil.copy2(str(session.log_path), str(artifacts/'runtime.log'))
                    shutil.copy2(str(session.session_dir/'session.json'), str(artifacts/'session.json'))
                except Exception as error:
                    result['cleanup_errors'].append('Owned preview/batch cleanup: '+repr(error))
                result['leftovers'] = {session.token: live_token_processes(session.token)}
            else:
                result['leftovers'] = {}
            result['source_templates_unchanged'] = source == hashes(lesson.folder)
            result['elapsed_seconds'] = round(time.monotonic()-started, 3)
            if result['cleanup_errors'] or any(result['leftovers'].values()) or not result['source_templates_unchanged']:
                result['passed'] = False
            save(output, result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('scenarios', 'source_sha256', 'prepared_input',
          'preview_isolation', 'live_native_trial_processes')}, ensure_ascii=False, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
