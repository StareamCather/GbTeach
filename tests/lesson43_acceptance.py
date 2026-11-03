"""Real custom Action generation, control, paused wall timeout and reentry."""
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


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    target=Path(parser.parse_args().output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/43-ros1-service-action'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a43-'))
    started=time.monotonic();session=None
    report={'lesson':'43','ok':False,'data_dir':str(storage.base),'checks':[]}
    def command_json(arguments,name,timeout=18):
        destination='build/'+name+'.json'
        session.command(' '.join(shlex.quote(str(x)) for x in arguments)+' > '+shlex.quote(destination),timeout=timeout,label=name)
        return json.loads((session.workspace/destination).read_text(encoding='utf-8'))
    def verify(name,student=False):
        assert session.verify()['passed'] is True
        record=json.loads((session.workspace/'build/verification.json').read_text(encoding='utf-8'))
        assert record['passed'] and record['assignment_pass'] is student
        report['checks'].append({'name':name,'passed':True,'verification':record})
    def actual_world_time(name):
        source='import instrument,json; from gazebo_msgs.srv import GetWorldProperties; r,n=instrument.initialize("native_time"); print(json.dumps({"sim_time_s":instrument.call(r,"/gazebo/get_world_properties",GetWorldProperties).sim_time}))'
        return command_json(['python3','-c',source],name)
    try:
        workspace=storage.prepare(lesson);marker='\n# A43 acceptance private edit; preserve across initialization.\n'
        for name in ('instrument.py','student.py'):
            with (workspace/name).open('a',encoding='utf-8') as stream:stream.write(marker)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        # A false-success scoped clear leaves the .2s force queued while paused.
        # Only real cancellation of that queue keeps the resumed plant at rest.
        session.pause(True)
        rest=command_json(['python3','lab.py','state'],'clear-before')
        pulse=command_json(['python3','lab.py','pulse'],'clear-scheduled-pulse')
        clearing=command_json(['python3','-c','import instrument,json; r,n=instrument.initialize("clear_probe"); instrument.clear(r); print(json.dumps({"acknowledged":True}))'],'clear-queue')
        session.pause(False);time.sleep(.3)
        settled=command_json(['python3','lab.py','state'],'clear-after-resume')
        assert settled['receipt_clock_after_s']-rest['receipt_clock_after_s']>.2
        assert abs(settled['position_m']-rest['position_m'])<.003 and abs(settled['velocity_m_s'])<.003,settled
        report['checks'].append({'name':'paused_force_queue_clear_has_real_no_motion_effect','passed':True,
            'before':rest,'scheduled_pulse':pulse,'clear':clearing,'after_resume':settled})
        for index in range(2):verify('repeat_real_arrival_feedback_cancel_invalid_'+str(index+1))
        build=json.loads((workspace/'build/action-build.json').read_text(encoding='utf-8'))
        assert build['action_md5']==build['rosmsg_action_md5']
        session.command('rostopic type /workshop43/move/feedback > build/feedback-type.txt',timeout=8)
        assert (workspace/'build/feedback-type.txt').read_text().strip()=='gazeboteach_workshop43/MoveSlideActionFeedback'
        # Actual CLI decoder must load this private generated message class.
        session.command('timeout 8 rostopic echo -n 1 /workshop43/move/status > build/raw-status.yaml',timeout=10)
        assert 'status_list:' in (workspace/'build/raw-status.yaml').read_text()
        session.command('rosmsg show gazeboteach_workshop43/MoveSlideActionFeedback > build/raw-feedback-schema.txt',timeout=8)
        assert 'elapsed_wall_s' in (workspace/'build/raw-feedback-schema.txt').read_text()
        report['checks'].append({'name':'real_catkin_and_raw_cli_types','passed':True,'build':build})
        session.pause(True)
        before=command_json(['python3','lab.py','state'],'paused-before')
        clock_before=actual_world_time('paused-world-time-before')
        paused_started=time.monotonic()
        outcome=command_json(['python3','lab.py','action','--target','.8','--timeout','2'],'paused-action',timeout=12)
        elapsed=time.monotonic()-paused_started
        after=command_json(['python3','lab.py','state'],'paused-after')
        clock_after=actual_world_time('paused-world-time-after')
        assert outcome['state']==4 and not outcome['reached'],outcome
        detail=json.loads(outcome['message'])
        assert detail['clear_confirmed'] and 'deadline' in detail['reason'].lower(),detail
        assert 2<=elapsed<8,elapsed
        assert abs(after['position_m']-before['position_m'])<.001
        # A new node may still read /clock=0 while paused (no new clock message).
        # Query the simulator itself to establish that time actually stood still.
        assert clock_after==clock_before
        report['checks'].append({'name':'new_cli_while_paused_wall_timeout','passed':True,
            'action':outcome,'wall_seconds':elapsed,'before':before,'after':after,
            'native_world_time_before':clock_before,'native_world_time_after':clock_after})
        session.pause(False);session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False);verify('reentry_preserves_instrument_and_student')
        for name in ('instrument.py','student.py'):
            assert marker in (workspace/name).read_text(encoding='utf-8'),'Personal edit overwritten: '+name
        session.stop();assert not _token_processes(session.token)
        # A controlled test fixture fills the temporary exercise only. Delivered
        # student.py remains blank; reference-mode acceptance stays ungraded.
        (workspace/'student.py').write_text((workspace/'reference.py').read_text(encoding='utf-8')+marker,encoding='utf-8')
        config=workspace/'ros_ws/src/gazeboteach_workshop43/config/controller.yaml'
        config.write_text(config.read_text(encoding='utf-8').replace('reference_demo','student'),encoding='utf-8')
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        verify('controlled_temporary_student_fixture_actual_action',student=True)
        session.stop();assert not _token_processes(session.token)
        report['checks'].append({'name':'owned_processes_cleaned','passed':True})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A43 service/action acceptance finished')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
