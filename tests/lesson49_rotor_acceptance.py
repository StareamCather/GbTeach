"""Physical four-rotor force/torque and pause-lease probe on a fresh Classic session."""
import argparse
import json
from pathlib import Path
import shlex
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.backends import get_backend
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage

PROBE="""
import json, os
from pathlib import Path
from plant import Plant
tests=[]
with Plant() as p:
    for i,signs in enumerate(((0,-1,1),(1,0,-1),(0,1,1),(-1,0,-1))):
        reset=p.reset(1,0,0)
        force=[2.4525]*4;force[i]+=.75
        r=p.step(force)['transition']
        rates=r['after']['state'][9:12]
        for actual,expected in zip(rates,signs):
            assert expected*actual>.005 if expected else abs(actual)<.005,(i,rates,signs)
        assert r['after']['state'][8]>r['before']['state'][8]
        tests.append({'rotor_index':i,'expected_rate_sign':signs,'transition':r})
    reset=p.reset(1,0,0)
    r=p.step([2.4525]*4,wind=(.6,0))['transition']
    assert r['after']['state'][6]>.01
    tests.append({'wind_x_n':.6,'transition':r})
Path('build/rotor-probe.json').write_text(json.dumps(tests))
with Plant() as p:
    os._exit(0)
"""

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True,type=Path)
    output=parser.parse_args().output
    folder=ROOT/'courses/classic11_noetic/49-quadrotor-flight'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gt-a49-native-'))
    workspace=storage.prepare(lesson)
    session=get_backend('native').create_session(ROOT,lesson,storage=storage)
    endpoint=Path('/tmp/gt-quad-'+session.token+'.sock')
    try:
        session.start(gui=False,terminals=False)
        session.pause(True)
        before=session.native_control.status()['stats']
        session.command('python3 -c '+shlex.quote(PROBE),timeout=20,label='native rotor and EOF probe')
        after=session.native_control.status()['stats']
        assert before['paused'] and after['paused']
        assert after['iterations']-before['iterations']==105
        assert abs(after['sim_time']-before['sim_time']-.105)<1e-6
        time.sleep(.2)
        idle=session.native_control.status()['stats']
        assert idle['iterations']==after['iterations'] and idle['sim_time']==after['sim_time']
        session.pause(False)
        raw='import os; from plant import Plant; p=Plant(); os._exit(0)'
        session.command('python3 -c '+shlex.quote(raw),timeout=8,label='running-world lease EOF probe')
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and session.native_control.status()['stats']['paused']:
            time.sleep(.05)
        assert not session.native_control.status()['stats']['paused']
        report={'ok':True,'before':before,'after':after,'idle':idle,'running_eof_restored':True,
                'trials':json.loads((workspace/'build/rotor-probe.json').read_text(encoding='utf-8')),
                'runtime_log':str(session.log_path)}
    finally:
        session.stop()
        assert not _token_processes(session.token)
        assert not endpoint.exists()
    report['owned_endpoint_and_processes_cleaned']=True
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':
    main()
