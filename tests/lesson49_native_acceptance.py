"""Real fixed-step quadrotor flight, independent world statistics and persistence."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import stat
import sys
import tempfile
import time
import traceback

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.backends import get_backend
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    target=Path(parser.parse_args().output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/49-quadrotor-flight'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a49-'));session=None;started=time.monotonic()
    report={'lesson':'49','ok':False,'data_dir':str(storage.base),'checks':[]}
    def statistics():
        assert session.native_control,'This acceptance requires the independent Classic SDK observer'
        return session.native_control.status()['stats']
    def restored(expected):
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            actual=statistics()
            if actual['paused'] is expected:return actual
            time.sleep(.04)
        raise RuntimeError('Native paused state did not return to '+str(expected))
    def endpoint():return Path('/tmp/gt-quad-'+session.token+'.sock')
    def stop_owned():
        socket_path=endpoint();session.stop()
        assert not _token_processes(session.token),'Owned process remained after stop'
        assert not socket_path.exists(),'Owned native plant endpoint was not removed'
    def verify_artifacts(workspace,value):
        file=(workspace/Path(value['trace_log'])).resolve()
        assert workspace.resolve() in file.parents and file.is_file()
        assert digest(file)==value['trace_sha256']
        trials=[json.loads(line) for line in file.read_text(encoding='utf-8').splitlines()]
        assert sorted(t['initial_z'] for t in trials)==[.45,1.55]
        for trial in trials:
            rows=trial['trace']
            assert trial['complete'] and len(rows)==400
            assert abs(rows[0]['before']['state'][2]-trial['initial_z'])<.001
            for i,row in enumerate(rows):
                assert row['after']['iterations']-row['before']['iterations']==20
                assert row['wind_force_n']==([.45,-.25] if 100<=i<175 else [0.,0.])
                f=np.asarray(row['applied_rotors_n'])
                assert f.shape==(4,) and ((0<=f)&(f<=6)).all()
                np.testing.assert_allclose(f,np.clip(row['control']['raw_rotors_n'],0,6),atol=1e-10)
                wrench=[sum(f),.22*(f[1]-f[3]),.22*(f[2]-f[0]),.018*(f[0]-f[1]+f[2]-f[3])]
                np.testing.assert_allclose(wrench,row['control']['allocated_wrench_n_nm'],atol=1e-10)
                assert row['transition']['applied_rotors_n']==list(f)
                assert row['transition']['wind_force_n']==row['wind_force_n']
            tail=np.asarray([row['after']['state'] for row in rows[-75:]])
            rms=[float(np.sqrt(np.mean(v))) for v in ((tail[:,2]-1)**2,np.sum(tail[:,:2]**2,axis=1),tail[:,3:6]**2)]
            np.testing.assert_allclose(rms,[trial[k] for k in ('terminal_z_rms','terminal_xy_rms','terminal_attitude_rms')],atol=1e-10)
            assert all(a<b for a,b in zip(rms,[.09,.14,.09]))
        return {file:digest(file)}
    def verify_steps(before,after,value):
        count=int(value['real_gazebo_transitions']);assert before['paused'] and after['paused']
        assert count==800,'Expected both native flight trials'
        resets=int(value['auxiliary_reset_physics_steps']);assert resets==2
        physical_steps=20*count+resets
        assert after['iterations']-before['iterations']==physical_steps,(before,after,count,resets)
        assert abs((after['sim_time']-before['sim_time'])-.001*physical_steps)<1e-5,(before,after,count,resets)
    try:
        workspace=storage.prepare(lesson);marker='\n# A49 private learner edit must survive reentry.\n'
        with (workspace/'learner.py').open('a',encoding='utf-8') as stream:stream.write(marker)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        socket_stat=endpoint().stat()
        assert stat.S_ISSOCK(socket_stat.st_mode) and stat.S_IMODE(socket_stat.st_mode)==0o600
        running=restored(False)
        # Abrupt exit bypasses Python __exit__/end; the native worker must notice
        # EOF and restore the healthy lease's original running state itself.
        probe='import os; from plant import Plant\nwith Plant() as p: os._exit(0)'
        session.command('python3 -c '+shlex.quote(probe),timeout=12,label='native EOF pause restoration')
        after_eof=restored(False)
        report['checks'].append({'name':'private_endpoint_and_abrupt_client_eof_restore_running','passed':True,
                                 'before':running,'after':after_eof,'mode_octal':'0600'})
        session.pause(True);before=statistics();assert before['paused']
        latest=workspace/'results/flight-report.json'
        if latest.exists():latest.unlink()
        assert session.verify()['passed'] is True,'See physical flight runtime log'
        value=json.loads(latest.read_text(encoding='utf-8'))
        assert value['passed'] and value['assignment_pass'] is False
        after=statistics();verify_steps(before,after,value)
        time.sleep(.2);idle=statistics()
        assert idle['iterations']==after['iterations'] and idle['sim_time']==after['sim_time']
        saved=verify_artifacts(workspace,value)
        report['checks'].append({'name':'actual_training_and_exact_independent_native_world_step_total','passed':True,
                                 'before':before,'after':after,'idle':idle,'verification':value})
        stop_owned()
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False);restored(False)
        assert marker in (workspace/'learner.py').read_text(encoding='utf-8')
        for file,expected in saved.items():assert digest(file)==expected,'Prior training artifact changed: '+str(file)
        # Fill only the isolated temporary test copy, never the delivered exercise.
        (workspace/'learner.py').write_text((workspace/'reference.py').read_text(encoding='utf-8')+marker,encoding='utf-8')
        session.pause(True);before_student=statistics()
        if latest.exists():latest.unlink()
        session.command('python3 lab.py student',timeout=int(lesson.data.get('verify_timeout',150)),
                        label='controlled temporary student fixture native flight')
        student=json.loads(latest.read_text(encoding='utf-8'));assert student['passed'] and student['assignment_pass'] is True
        after_student=statistics();verify_steps(before_student,after_student,student)
        new=verify_artifacts(workspace,student)
        assert not (set(new)&set(saved)),'Student run overwrote reference artifacts'
        for file,expected in saved.items():assert digest(file)==expected
        report['checks'].append({'name':'temporary_student_fixture_native_training_and_checkpoint_audit','passed':True,
                                 'before':before_student,'after':after_student,'verification':student})
        stop_owned()
        report['checks'].append({'name':'reentry_preserved_learner_and_flight_artifacts_then_cleaned_owned_endpoint','passed':True,
                                 'artifact_sha256':{str(path):value for path,value in saved.items()}})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['traceback']=traceback.format_exc()
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:
            try:session.stop(reason='A49 native fixed-step acceptance finished')
            except Exception as error:report['cleanup_error']=str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
