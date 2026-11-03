"""Real fixed-step REINFORCE, independent world statistics and persistence."""
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
    folder=ROOT/'courses/classic11_noetic/48-mlp-reinforcement'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a48-'));session=None;started=time.monotonic()
    report={'lesson':'48','ok':False,'data_dir':str(storage.base),'checks':[]}
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
    def endpoint():return Path('/tmp/gt-reinforce-'+session.token+'.sock')
    def stop_owned():
        socket_path=endpoint();session.stop()
        assert not _token_processes(session.token),'Owned process remained after stop'
        assert not socket_path.exists(),'Owned native plant endpoint was not removed'
    def verify_artifacts(workspace,value):
        paths={}
        for key in ('trajectory_log','initial_policy','final_policy'):
            file=Path(value[key]);file=(file if file.is_absolute() else workspace/file).resolve()
            assert workspace.resolve() in file.parents and file.is_file()
            paths[key]=file
        episodes=[json.loads(line) for line in paths['trajectory_log'].read_text(encoding='utf-8').splitlines()]
        before=[ep for ep in episodes if ep['phase']=='eval_before']
        after=[ep for ep in episodes if ep['phase']=='eval_after']
        training=[ep for ep in episodes if ep['phase']=='train']
        assert len(before)==len(after)==2 and len(training)==24
        assert sum(len(ep['rows']) for ep in episodes)==1440
        assert {ep['requested_initial_x_m'] for ep in before}=={-.55,.55}
        for x,y in zip(before,after):
            assert x['noise_seed']==y['noise_seed']
            assert x['requested_initial_x_m']==y['requested_initial_x_m']
            assert len(x['rows'])==len(y['rows'])==120
            assert [r['noise_standard_normal'] for r in x['rows']]==[r['noise_standard_normal'] for r in y['rows']]
        for ep in episodes:
            for row in ep['rows']:
                assert row['after']['iterations']-row['before']['iterations']==20
                assert abs(row['raw_action_n']-(row['mean_n']+1.4*row['noise_standard_normal']))<1e-10
                assert abs(row['applied_force_n']-float(8*np.tanh(row['raw_action_n']/8)))<1e-10
                assert abs(row['transition']['force_n']-row['applied_force_n'])<1e-10
                expected_reward=-(row['after']['x_m']**2+.03*row['after']['v_m_s']**2+.001*row['applied_force_n']**2)
                assert abs(row['reward']-expected_reward)<1e-10
        assert digest(paths['trajectory_log'])==value['trajectory_sha256']
        assert digest(paths['initial_policy'])==value['initial_policy_sha256']
        assert digest(paths['final_policy'])==value['final_policy_sha256']
        assert digest(paths['initial_policy'])!=digest(paths['final_policy'])
        return {path:digest(path) for path in paths.values()}
    def verify_steps(before,after,value):
        count=int(value['real_gazebo_transitions']);assert before['paused'] and after['paused']
        assert count==1440,'Expected six training batches and paired policy evaluations'
        resets=int(value['auxiliary_reset_physics_steps']);assert resets==28
        physical_steps=20*count+resets
        assert after['iterations']-before['iterations']==physical_steps,(before,after,count,resets)
        assert abs((after['sim_time']-before['sim_time'])-.001*physical_steps)<1e-5,(before,after,count,resets)
    try:
        workspace=storage.prepare(lesson);marker='\n# A48 private learner edit must survive reentry.\n'
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
        latest=workspace/'results/reinforce-report.json'
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
        (workspace/'learner.py').write_text((workspace/'reference.py').read_text(encoding='utf-8')+marker,encoding='utf-8')
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
            try:session.stop(reason='A48 native fixed-step acceptance finished')
            except Exception as error:report['cleanup_error']=str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
