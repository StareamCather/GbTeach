"""Real TF/point composition, preserved-stamp delay and expected bad-slop case."""
import argparse
import json
from pathlib import Path
import shlex
import sys
import tempfile
import time
import traceback
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.backends import get_backend
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    target=Path(parser.parse_args().output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/45-ros1-tf-synchronization'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a45-'));session=None;started=time.monotonic()
    report={'lesson':'45','ok':False,'data_dir':str(storage.base),'checks':[]}
    def verify(name):
        destination=session.workspace/'build/verification.json'
        if destination.exists():destination.unlink()
        assert session.verify()['passed'] is True,'See native TF and synchronization evidence in runtime log'
        value=json.loads(destination.read_text(encoding='utf-8'))
        assert value['passed'] and value['assignment_pass'] is False,value.get('error')
        report['checks'].append({'name':name,'passed':True,'verification':value})
        return value
    try:
        workspace=storage.prepare(lesson);marker='\n# A45 private edit must survive preparation and reentry.\n'
        for name in ('instrument.py','student.py'):
            with (workspace/name).open('a',encoding='utf-8') as stream:stream.write(marker)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        (workspace/'build').mkdir(exist_ok=True)
        # Exercise the actual Noetic command taught in the lesson, with a
        # bounded lifetime and captured real TF output (not executable discovery).
        echo='''import subprocess
from pathlib import Path
p=subprocess.Popen(['rosrun','tf','tf_echo','world','probe'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
try: output=p.communicate(timeout=3)[0]
except subprocess.TimeoutExpired:
 p.terminate();output=p.communicate(timeout=2)[0]
Path('build/tf-echo.txt').write_text(output)
assert 'Translation' in output and 'Rotation' in output, output
'''
        session.command('python3 -c '+shlex.quote(echo),timeout=9,label='actual Noetic TF echo command')
        report['checks'].append({'name':'actual_rosrun_tf_echo_world_probe','passed':True,
                                 'output':(workspace/'build/tf-echo.txt').read_text()})
        for index in range(2):
            value=verify('default_repeated_native_tf_and_sync_'+str(index+1))
            counts=value['match_statistics']
            assert counts['baseline_exact']>=6 and counts['offset_exact']==0 and counts['offset_approx']>=6,counts
        session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        verify('reentry_preserves_instrument_and_student')
        for name in ('instrument.py','student.py'):
            assert marker in (workspace/name).read_text(encoding='utf-8'),'Personal edit overwritten: '+name
        session.stop();assert not _token_processes(session.token)
        alternate_storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a45-mount-slop-'))
        variant=alternate_storage.prepare(lesson)
        world=ET.parse(str(variant/'world.sdf'))
        pose=world.find("world/model[@name='workshop']/link[@name='probe']/pose")
        assert pose is not None,'Expected actual probe link with explicit mount pose'
        values=[float(value) for value in pose.text.split()]
        values[2]=.10;values[5]=.3;pose.text=' '.join(str(value) for value in values)
        world.write(str(variant/'world.sdf'),encoding='utf-8',xml_declaration=True)
        import yaml  # Already required by ROS1 rosparam/roslaunch on this host.
        config=variant/'ros_ws/src/gazeboteach_workshop45/config/controller.yaml'
        settings=yaml.safe_load(config.read_text(encoding='utf-8'));settings['slop_s']=.004
        config.write_text(yaml.safe_dump(settings,sort_keys=True),encoding='utf-8')
        session=get_backend('native').create_session(ROOT,lesson,storage=alternate_storage)
        session.start(gui=False,terminals=False)
        value=verify('changed_native_probe_mount_and_expected_bad_slop')
        assert value['configuration']['slop_s']==.004
        counts=value['match_statistics']
        assert counts['baseline_exact']>=6 and counts['offset_exact']==0 and counts['offset_approx']==0,counts
        assert counts['expected_offset_approx'] is False,counts
        assert abs(float(ET.parse(str(variant/'world.sdf')).find("world/model[@name='workshop']/link[@name='probe']/pose").text.split()[2])-.10)<1e-12
        session.stop();assert not _token_processes(session.token)
        report['checks'].append({'name':'all_owned_processes_cleaned','passed':True})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['traceback']=traceback.format_exc()
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:
            try:session.stop(reason='A45 TF/synchronization acceptance finished')
            except Exception as error:report['cleanup_error']=str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
