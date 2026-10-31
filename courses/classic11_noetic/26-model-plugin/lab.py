#!/usr/bin/env python3
"""Native wire observations; model resets are explicit experimental boundaries."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
import uuid
from config import INERTIA, parameters, prepared


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def source_time(value):
    return int(value.get('sec', 0)) + int(value.get('nsec', 0))*1e-9


def decode_lines(text):
    rows = []
    for line in text.splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict): rows.append(value)
    return rows


def native_rotation(rows, settings, epoch):
    """Unwrap actual native link quaternions, then compare interval displacement."""
    samples = []
    repeated_or_backwards = []
    for event in rows:
        if event.get('source') != 'pose': continue
        packet = event['data']
        stamp = source_time(packet['time'])
        for pose in packet.get('pose', []):
            if pose.get('name') not in ('rotor', 'rotor_lab::rotor', 'lesson26::rotor_lab::rotor'): continue
            q = pose['orientation']
            x,y,z,w = (float(q.get(key, default)) for key,default in [('x',0),('y',0),('z',0),('w',1)])
            norm = math.sqrt(x*x+y*y+z*z+w*w)
            require(abs(norm-1) < 1e-5, 'Native orientation is not a unit quaternion')
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            if samples and stamp <= samples[-1]['stamp_s']:
                repeated_or_backwards.append({'stamp_s': stamp, 'yaw_rad': yaw})
                continue
            angle = yaw if not samples else samples[-1]['unwrapped_rad'] + math.atan2(
                math.sin(yaw-samples[-1]['yaw_rad']), math.cos(yaw-samples[-1]['yaw_rad']))
            samples.append({'stamp_s': stamp, 'yaw_rad': yaw, 'unwrapped_rad': angle})
    require(len(samples) >= 30, 'Too few actual native rotor poses; inspect raw pose names/data')
    first,last = samples[0],samples[-1]
    require(last['stamp_s']-first['stamp_s'] > 2, 'Native motion window is too short')
    torque,drag = settings['drive_torque'],settings['drag']
    tau = INERTIA/drag
    def theta(t):
        require(t >= -1e-6, 'Native pose timestamp precedes the real reset epoch')
        return torque/drag*(t-tau*(1-math.exp(-t/tau)))
    expected = theta(last['stamp_s']-epoch)-theta(first['stamp_s']-epoch)
    observed = last['unwrapped_rad']-first['unwrapped_rad']
    require(abs(observed-expected) < .035, 'Native link rotation differs from torque/drag prediction')
    require(observed*torque > 0 and abs(observed) > .4, 'No expected nontrivial actual rotor motion')
    return {'native_pose_count': len(samples), 'first': first, 'last': last,
            'observed_delta_angle_rad': observed, 'analytic_delta_angle_rad': expected,
            'absolute_angle_error_rad': abs(observed-expected),
            'duplicate_or_backwards_pose_samples': repeated_or_backwards,
            'note': 'Native link relative poses in a fixed, unrotated model; raw events retain original source stamps.'}


class Lab:
    def __init__(self):
        self.settings, self.record = prepared()
        self.workspace = Path.cwd().resolve()
        self.binary = self.workspace/'build/model_observer'

    def client(self, mode, *arguments, timeout=7):
        completed = subprocess.run([str(self.binary),mode]+list(arguments), text=True, capture_output=True, timeout=timeout)
        rows = decode_lines(completed.stdout)
        require(completed.returncode == 0, 'Native SDK client failed: '+completed.stdout[-1800:]+' '+completed.stderr[-1000:])
        require(rows, 'Native SDK client produced no actual response')
        return rows[-1]

    def snapshot(self):
        return self.client('snapshot')['payload']

    def stats(self):
        return self.client('stats')['data']

    def pause(self, value):
        before = self.stats()
        if bool(before.get('paused', False)) != bool(value):
            self.client('control','--pause','true' if value else 'false')
        deadline = time.monotonic()+5
        while True:
            current = self.stats()
            if bool(current.get('paused', False)) == bool(value): return current
            require(time.monotonic() < deadline, 'Actual pause state not confirmed; control was not replayed')

    def ready(self):
        self.settings,self.record = prepared()
        state = self.snapshot()
        require(state['world'] == 'lesson26' and state['model'] == 'rotor_lab', 'Wrong private model/world')
        require(state['load_count'] == state['init_count'] == 1, 'Load/Init lifecycle did not finish exactly once')
        require(state['source_sha256'] == self.record['source_sha256']['model_plugin.cc'], 'Actual loaded plugin source hash differs')
        require(state['config_sha256'] == self.record['config_sha256'], 'Actual SDF inputs are stale; reload required')
        for key in ('drive_torque','drag'):
            require(abs(state[key]-self.settings[key]) < 1e-12, 'Actual plugin cached parameter differs: '+key)
        return {'ready': True, 'state': state, 'world_stats': self.stats(),
                'native_entity': self.client('entity'), 'library_sha256': self.record['library_sha256'],
                'note': 'Read-only readiness does not advance a paused world; plugin snapshot retains its actual source capture phase/time.'}

    def reset(self):
        self.pause(True)
        before = self.snapshot()
        native_before = self.stats()
        self.client('control','--reset-models')
        deadline = time.monotonic()+5
        while True:
            after = self.snapshot()
            if after['reset_count'] == before['reset_count']+1: break
            require(time.monotonic() < deadline, 'Native model reset hook was not observed; reset was not replayed')
            time.sleep(.02)
        native_after = self.stats()
        require(after['load_count'] == before['load_count'] and after['init_count'] == before['init_count'],
                'Model reset unexpectedly reran Load or Init')
        require(after['epoch_updates'] == 0 and abs(after['omega_rad_s']) < 1e-9,
                'Actual native reset did not reset epoch/rotor velocity')
        reset_clock_delta = source_time(native_after['sim_time'])-source_time(native_before['sim_time'])
        require(-1e-8 <= reset_clock_delta <= .001+1e-8,
                'Native model reset advanced beyond its one control-dispatch iteration')
        return {'before': before, 'after': after, 'world_before': native_before, 'world_after': native_after,
                'native_entity_after': self.client('entity'), 'paused': True,
                'reset_clock_delta_s': reset_clock_delta,
                'note':'Classic can consume one .001s control-dispatch iteration for reset, returning before normal plugin/physics update.'}

    def step(self, count):
        require(type(count) is int and 1 <= count <= 1000, 'Use 1..1000 native ticks')
        require(bool(self.stats().get('paused', False)), 'Pause the world before stepping')
        before, initial = self.snapshot(), self.stats()
        start = source_time(initial['sim_time'])
        self.client('control','--steps',str(count))
        deadline = time.monotonic()+6
        while True:
            final = self.stats()
            elapsed = source_time(final['sim_time'])-start
            if elapsed >= count*.001-1e-8: break
            require(time.monotonic() < deadline, 'Requested native ticks did not finish; no replay attempted')
        after = self.snapshot()
        require(abs(elapsed-count*.001) < 1e-8 and bool(final.get('paused',False)), 'Unexpected tick count or pause state')
        require(after['updates']-before['updates'] == count and after['reset_count'] == before['reset_count'],
                'Actual ModelPlugin update counter differs from the actual physics tick count')
        return {'passed': True, 'requested_steps': count, 'delta_sim_time_s': elapsed,
                'before': before, 'after': after, 'world_before': initial, 'world_after': final}

    def watch(self, seconds):
        require(math.isfinite(seconds) and .1 <= seconds <= 8, 'Watch .1..8 wall seconds')
        output = self.workspace/'build'/('watch-'+uuid.uuid4().hex+'.jsonl')
        with output.open('w',encoding='utf-8') as stream:
            result = subprocess.run([str(self.binary),'watch','--seconds',str(seconds)],stdout=stream,stderr=subprocess.PIPE,
                                    text=True,timeout=seconds+7)
        require(result.returncode == 0, 'Native observer failed: '+result.stderr)
        return {'raw_events_file': str(output), 'events': decode_lines(output.read_text(encoding='utf-8'))}

    def demo(self, seconds=3):
        require(math.isfinite(seconds) and 2.5 <= seconds <= 6, 'Compare 2.5..6 actual simulation seconds')
        self.ready()
        entry_stats, entry = self.stats(), self.snapshot()
        original_pause = bool(entry_stats.get('paused',False))
        result = {'entry': entry, 'entry_world': entry_stats,
                  'physical_end_policy': 'Model-only reset clears velocity and restarts the plugin epoch while retaining the observed internal angle; only the entry pause state is restored.'}
        process = None
        raw = self.workspace/'build'/('motion-'+uuid.uuid4().hex+'.jsonl')
        try:
            reset = self.reset()
            result['initial_reset'] = reset
            with raw.open('w',encoding='utf-8') as stream:
                process = subprocess.Popen([str(self.binary),'watch','--seconds',str(seconds+3)],stdout=stream,
                                           stderr=subprocess.PIPE,text=True)
                deadline = time.monotonic()+4
                while not raw.is_file() or 'client_ready' not in raw.read_text(encoding='utf-8'):
                    require(process.poll() is None and time.monotonic() < deadline,'Native observer did not become ready')
                    time.sleep(.03)
                epoch = reset['after']['epoch_time_s']
                self.pause(False)
                deadline = time.monotonic()+seconds*5
                while source_time(self.stats()['sim_time'])-epoch < seconds:
                    require(time.monotonic() < deadline, 'Real simulation did not advance before deadline')
                self.pause(True)
                result['end_before_reset'] = self.snapshot()
                result['native_entity_before_reset'] = self.client('entity')
                _, error = process.communicate(timeout=seconds+8)
                require(process.returncode == 0, 'Native observer failed: '+error)
            events = decode_lines(raw.read_text(encoding='utf-8'))
            result['native_rotation'] = native_rotation(events,self.settings,epoch)
            result.update(raw_events_file=str(raw),events=events,passed=True)
        finally:
            if process and process.poll() is None:
                process.terminate()
                process.communicate(timeout=3)
            result['final_reset'] = self.reset()
            result['final_world'] = self.pause(original_pause)
        return result

    def verify(self):
        original_pause = bool(self.stats().get('paused',False))
        motion = self.demo(3)
        try:
            self.pause(True)
            before, stats_before = self.snapshot(), self.stats()
            time.sleep(.2)
            after, stats_after = self.snapshot(), self.stats()
            require(before['updates'] == after['updates'] and stats_before['sim_time'] == stats_after['sim_time'],
                    'Paused world/plugin advanced without a requested tick')
            stepped = self.step(100)
            reset = self.reset()
            return {'passed': True, 'motion': motion, 'pause_observation': {'before':before,'after':after},
                    'steps':stepped, 'reset':reset,
                    'note':'Ends at a native model-only reset boundary: velocity cleared, internal angle retained, entry pause restored. Lifecycle, clock and messages are not rewound.'}
        finally:
            self.pause(original_pause)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('ready','status','predict','watch','pause','resume','reset','step','demo','verify'))
    parser.add_argument('--seconds',type=float,default=3.)
    parser.add_argument('--steps',type=int,default=100)
    args=parser.parse_args()
    if args.action=='predict':
        p=parameters();result={'parameters':p,'inertia_kg_m2':INERTIA,'omega_infinity_rad_s':p['drive_torque']/p['drag'],'time_constant_s':INERTIA/p['drag']}
    else:
        lab=Lab()
        if args.action in ('ready','status'):result=lab.ready()
        elif args.action=='watch':result=lab.watch(args.seconds)
        elif args.action=='step':result=lab.step(args.steps)
        elif args.action=='demo':result=lab.demo(args.seconds)
        elif args.action in ('pause','resume'):result={'world_stats':lab.pause(args.action=='pause')}
        else:result=getattr(lab,args.action)()
    print(json.dumps(result,ensure_ascii=False,allow_nan=False))


if __name__=='__main__':
    try: main()
    except Exception as error:
        print(json.dumps({'passed':False,'error':type(error).__name__+': '+str(error)},ensure_ascii=False));raise SystemExit(1)
