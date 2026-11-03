"""Real typed ROS graph, repeated reset/pulse checks and private-edit persistence."""
import argparse
import json
import os
import shlex
from pathlib import Path
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.catalog import Lesson
from gazeboteach.backends import get_backend
from gazeboteach.storage import Storage
from gazeboteach.runtime import _token_processes


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    options=parser.parse_args();target=Path(options.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/42-ros1-graph'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a42-'))
    started=time.monotonic();session=None;previous=os.environ.get('ROS_NAMESPACE')
    report={'lesson':'42','ok':False,'data_dir':str(storage.base),'checks':[]}
    try:
        workspace=storage.prepare(lesson);marker='\n# Private A42 edit must survive initialization and reentry.\n'
        for name in ('instrument.py','student.py'):
            with (workspace/name).open('a',encoding='utf-8') as stream:stream.write(marker)
        os.environ['ROS_NAMESPACE']='/external_robot_must_not_leak'
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        assert not session.env.get('ROS_NAMESPACE'),'External namespace renamed private services'
        session.start(gui=False,terminals=False)
        (workspace/'build').mkdir(exist_ok=True)
        session.pause(True)
        probe='import instrument,json; r,n=instrument.initialize("queue_probe"); before=instrument.joint(r); instrument.effort(r,2,.2); instrument.clear(r); print(json.dumps({"before":before,"scheduled_sim_s":.2,"clear_name":"slide"}))'
        session.command('python3 -c '+shlex.quote(probe)+' > build/clear-before.json',timeout=20)
        session.pause(False);time.sleep(.3)
        session.command('python3 lab.py state > build/clear-after.json',timeout=10)
        before=json.loads((workspace/'build/clear-before.json').read_text())
        after=json.loads((workspace/'build/clear-after.json').read_text())
        assert after['receipt_clock_after_s']-before['before']['receipt_clock_after_s']>.2
        assert abs(after['position_m']-before['before']['position_m'])<.003 and abs(after['velocity_m_s'])<.003
        report['checks'].append({'name':'real_paused_force_queue_clear_prevents_motion_after_resume','passed':True,
                                 'before':before,'after':after})
        for index in range(3):
            assert session.verify()['passed'] is True
            result=json.loads((workspace/'build/verification.json').read_text())
            assert result['passed'] and result['assignment_pass'] is False
            report['checks'].append({'name':'repeated_real_graph_and_pulse_'+str(index+1),'passed':True,'verification':result})
        session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        assert session.verify()['passed'] is True
        for name in ('instrument.py','student.py'):
            assert marker in (workspace/name).read_text(encoding='utf-8'),'Personal edit overwritten: '+name
        report['checks'].append({'name':'reentry_preserves_personal_instrument_and_student','passed':True,
                                 'verification':json.loads((workspace/'build/verification.json').read_text())})
        session.stop();assert not _token_processes(session.token)
        report['checks'].append({'name':'foreign_namespace_isolated_and_owned_processes_cleaned','passed':True})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A42通信专项验收结束')
        if previous is None:os.environ.pop('ROS_NAMESPACE',None)
        else:os.environ['ROS_NAMESPACE']=previous
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
