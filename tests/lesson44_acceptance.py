"""Real launch expansion, runtime parameter effects, namespace variant, reentry."""
import argparse
import json
from pathlib import Path
import shlex
import sys
import tempfile
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.backends import get_backend
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    target=Path(parser.parse_args().output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/44-ros1-launch-parameters'
    data=json.loads((folder/'lesson.json').read_text(encoding='utf-8'));lesson=Lesson(folder,data)
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a44-'));session=None;started=time.monotonic()
    report={'lesson':'44','ok':False,'data_dir':str(storage.base),'checks':[]}
    def result(name):
        value=json.loads((session.workspace/'build/verification.json').read_text(encoding='utf-8'))
        assert value['passed'] and value['assignment_pass'] is False,value.get('error')
        report['checks'].append({'name':name,'passed':True,'verification':value})
        return value
    def clear_old_result():
        path=session.workspace/'build/verification.json'
        if path.exists():path.unlink()
    try:
        workspace=storage.prepare(lesson);marker='\n# A44 private exercise edit must survive initialization and reentry.\n'
        for name in ('instrument.py','student.py'):
            with (workspace/name).open('a',encoding='utf-8') as stream:stream.write(marker)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False);(workspace/'build').mkdir(exist_ok=True)
        session.command('rospack find gazeboteach_workshop44 > build/resolved-package.txt',timeout=8)
        actual=Path((workspace/'build/resolved-package.txt').read_text().strip()).resolve()
        assert actual==(workspace/'ros_ws/src/gazeboteach_workshop44').resolve()
        session.command('roslaunch --nodes gazeboteach_workshop44 lab.launch > build/launch-nodes.txt',timeout=8)
        assert '/workshop44/instrument' in (workspace/'build/launch-nodes.txt').read_text()
        for index in range(2):
            clear_old_result()
            assert session.verify()['passed'] is True,'See runtime log for actual parameter verification failure'
            result('default_repeated_parameter_and_physics_'+str(index+1))
        session.command('rosparam dump build/private-namespace-snapshot.yaml /workshop44',timeout=8)
        assert 'pulse_force' in (workspace/'build/private-namespace-snapshot.yaml').read_text()
        report['checks'].append({'name':'private_package_static_launch_and_real_parameter_dump','passed':True,
            'resolved_package':str(actual),'expanded_nodes':(workspace/'build/launch-nodes.txt').read_text()})
        session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        clear_old_result()
        assert session.verify()['passed'] is True
        result('reentry_preserves_personal_code')
        for name in ('instrument.py','student.py'):
            assert marker in (workspace/name).read_text(encoding='utf-8'),'Personal edit overwritten: '+name
        session.stop();assert not _token_processes(session.token)
        variant=json.loads(json.dumps(data))
        variant['startup'][0]['command']+=' namespace:=experiment44 pulse_scale:=0.5'
        variant['after_startup']=[command+' __ns:=/experiment44' for command in variant['after_startup']]
        alternate=Lesson(folder,variant)
        session=get_backend('native').create_session(ROOT,alternate,
            storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a44-namespace-')))
        session.start(gui=False,terminals=False)
        # Apply namespace only to the client, never to Gazebo's whole environment.
        clear_old_result()
        session.command('/usr/bin/python3 '+shlex.quote(str(folder/'verify.py'))+' __ns:=/experiment44',
                        timeout=int(variant.get('verify_timeout',65)),label='namespace/scale variant verification')
        value=result('actual_namespace_and_half_scale_launch_variant')
        assert value['namespace']=='/experiment44',value
        assert value['active_scale']==.5,value
        session.command('rosnode list > build/live-node-names.txt',timeout=8)
        names=(session.workspace/'build/live-node-names.txt').read_text()
        assert '/experiment44/instrument' in names and '/workshop44/instrument' not in names
        session.stop();assert not _token_processes(session.token)
        report['checks'].append({'name':'all_owned_processes_cleaned','passed':True})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['traceback']=traceback.format_exc()
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:
            try:session.stop(reason='A44 parameter acceptance finished')
            except Exception as error:
                report['cleanup_error']=type(error).__name__+': '+str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
