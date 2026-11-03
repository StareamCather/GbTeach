"""Real fixed-step behavior cloning, independent world statistics and persistence."""
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
    folder=ROOT/'courses/classic11_noetic/47-mlp-imitation'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a47-'));session=None;started=time.monotonic()
    report={'lesson':'47','ok':False,'data_dir':str(storage.base),'checks':[]}
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
    def endpoint():return Path('/tmp/gt-learning-'+session.token+'.sock')
    def stop_owned():
        socket_path=endpoint();session.stop()
        assert not _token_processes(session.token),'Owned process remained after stop'
        assert not socket_path.exists(),'Owned native plant endpoint was not removed'
    def verify_artifacts(workspace,value):
        paths={}
        for key in ('data','policy','traces','report'):
            file=Path(value[key]);file=(file if file.is_absolute() else workspace/file).resolve()
            assert workspace.resolve() in file.parents and file.is_file()
            paths[key]=file
        dataset=json.loads(paths['data'].read_text(encoding='utf-8'));rows=dataset['rows']
        assert dataset['complete'] and len(rows)==600
        assert dataset['train_episode_ids']==list(range(8)) and dataset['held_out_episode_ids']==[8,9]
        assert set(row['target_m'] for row in rows)=={0.0}
        assert all(sum(row['episode']==i for row in rows)==60 for i in range(10))
        assert value['training_rows']==480 and value['held_out_episode_rows']==120
        with np.load(str(paths['policy']),allow_pickle=False) as weights:
            def prediction(inputs):
                return (np.tanh(np.asarray(inputs)@weights['W1']+weights['b1'])@weights['W2']+weights['b2']).reshape(-1)
            held=[row for row in rows if row['episode'] in (8,9)]
            held_force=8*prediction([row['features'] for row in held])
            mse=float(np.mean((held_force-np.asarray([row['expert_force_n'] for row in held]))**2))
            assert abs(mse-value['test_mse_after'])<1e-10
            trials=value['physical_trials'];assert sorted(x['target_m'] for x in trials)==[-.3,.3]
            for trial in trials:
                trace=trial['trace'];goal=trial['target_m'];assert len(trace)==170
                expected=[[(goal-row['before']['x_m']),row['before']['v_m_s']/2] for row in trace]
                np.testing.assert_allclose([row['features'] for row in trace],expected,atol=1e-10)
                forces=np.clip(8*prediction(expected),-8,8)
                np.testing.assert_allclose([row['actual_force_n'] for row in trace],forces,atol=1e-9)
                np.testing.assert_allclose([row['transition']['force_n'] for row in trace],forces,atol=1e-9)
                assert all(row['after']['iterations']-row['before']['iterations']==20 for row in trace)
                rms=float(np.sqrt(np.mean([(row['after']['x_m']-goal)**2 for row in trace[-30:]])))
                assert abs(rms-trial['terminal_error_rms_m'])<1e-10
        assert digest(paths['policy'])==value['policy_sha256']
        return {path:digest(path) for path in paths.values()}
    def verify_steps(before,after,value):
        count=int(value['real_gazebo_transitions']);assert before['paused'] and after['paused']
        assert count>=940,'Expert collection plus both unseen closed-loop trials were not executed'
        resets=int(value['auxiliary_reset_physics_steps']);assert resets>=12
        physical_steps=20*count+resets
        assert after['iterations']-before['iterations']==physical_steps,(before,after,count,resets)
        assert abs((after['sim_time']-before['sim_time'])-.001*physical_steps)<1e-5,(before,after,count,resets)
    try:
        workspace=storage.prepare(lesson);marker='\n# A47 private learner edit must survive reentry.\n'
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
        latest=workspace/'results/imitation-report.json'
        if latest.exists():latest.unlink()
        assert session.verify()['passed'] is True,'See physical training runtime log'
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
        fixture='''import numpy as np
def forward(X,W1,b1,W2,b2):
    h=np.tanh(np.atleast_2d(X)@W1+b1)
    return (h@W2+b2).reshape(-1)
def mse_derivative(prediction,target):
    return 2*(prediction-target)
'''
        (workspace/'learner.py').write_text(fixture+marker,encoding='utf-8')
        session.pause(True);before_student=statistics()
        if latest.exists():latest.unlink()
        session.command('python3 lab.py student',timeout=int(lesson.data.get('verify_timeout',150)),
                        label='controlled temporary student fixture native training')
        student=json.loads(latest.read_text(encoding='utf-8'));assert student['passed'] and student['assignment_pass'] is True
        after_student=statistics();verify_steps(before_student,after_student,student)
        new=verify_artifacts(workspace,student)
        assert not (set(new)&set(saved)),'Student run overwrote reference artifacts'
        for file,expected in saved.items():assert digest(file)==expected
        report['checks'].append({'name':'temporary_student_fixture_native_training_and_checkpoint_audit','passed':True,
                                 'before':before_student,'after':after_student,'verification':student})
        stop_owned()
        report['checks'].append({'name':'reentry_preserved_learner_and_training_artifacts_then_cleaned_owned_endpoint','passed':True,
                                 'artifact_sha256':{str(path):value for path,value in saved.items()}})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['traceback']=traceback.format_exc()
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:
            try:session.stop(reason='A47 native fixed-step acceptance finished')
            except Exception as error:report['cleanup_error']=str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
