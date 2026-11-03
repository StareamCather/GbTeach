"""Real bag lifecycle/replay, diagnostic recovery, artifact and edit persistence."""
import argparse
import hashlib
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


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    target=Path(parser.parse_args().output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/46-ros1-bag-diagnostics'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a46-'));session=None;started=time.monotonic();saved={}
    report={'lesson':'46','ok':False,'data_dir':str(storage.base),'checks':[]}
    def verify(name):
        destination=session.workspace/'build/verification.json'
        if destination.exists():destination.unlink()
        assert session.verify()['passed'] is True,'See real recorder/player/diagnostics in runtime log'
        value=json.loads(destination.read_text(encoding='utf-8'))
        assert value['passed'] and value['assignment_pass'] is False,value.get('error')
        bag=Path(value['bag_path'])
        if not bag.is_absolute():bag=session.workspace/bag
        bag=bag.resolve()
        assert session.workspace.resolve() in bag.parents,'Recording left this private exercise'
        assert bag.is_file() and bag.suffix=='.bag' and bag.stat().st_size>0
        assert bag not in saved,'A later experiment overwrote an earlier recording name'
        assert not Path(str(bag)+'.active').exists(),'Successful recorder did not finish its file'
        actual=digest(bag);assert value['bag_sha256']==actual
        saved[bag]=actual
        for previous,expected in saved.items():
            assert digest(previous)==expected,'Record/replay modified a previous bag: '+str(previous)
        report['checks'].append({'name':name,'passed':True,'verification':value,
                                 'closed_bag':str(bag),'bag_sha256':actual})
    try:
        workspace=storage.prepare(lesson);marker='\n# A46 private code edit must survive initialization and reentry.\n'
        for name in ('instrument.py','student.py'):
            with (workspace/name).open('a',encoding='utf-8') as stream:stream.write(marker)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        for index in range(2):verify('repeat_real_record_play_and_diagnostic_recovery_'+str(index+1))
        session.pause(True)
        paused='''import json,os,time,uuid
from pathlib import Path
import instrument
r,ns=instrument.initialize('paused_record_probe');e=instrument.LiveEvidence(r,ns)
started=time.monotonic();accepted=False;result={}
try:
 instrument.record_motion(r,ns,e,Path('build/paused-recordings')/uuid.uuid4().hex,hold_wall_s=.3)
 accepted=True
except Exception as error: result['expected_recording_failure']=type(error).__name__+': '+str(error)
finally:
 try: instrument.final_clear(r)
 finally: e.close()
result['elapsed_wall_s']=time.monotonic()-started;result['recording_accepted']=accepted
remaining=[];session_marker=('GAZEBOTEACH_SESSION_TOKEN='+os.environ['GAZEBOTEACH_SESSION_TOKEN']).encode()
for p in Path('/proc').iterdir():
 if not p.name.isdigit(): continue
 try:
  if p.stat().st_uid!=os.getuid(): continue
  env=(p/'environ').read_bytes().split(b'\\0');state=(p/'stat').read_text().split(') ',1)[1].split()[0]
  if state!='Z' and session_marker in env and any(v.startswith(b'GAZEBOTEACH_BAG_TOKEN=') for v in env): remaining.append(int(p.name))
 except (FileNotFoundError,ProcessLookupError,PermissionError): pass
result['remaining_owned_bag_processes']=remaining
Path('build/paused-record-probe.json').write_text(json.dumps(result,indent=2))
assert not accepted and not remaining and result['elapsed_wall_s']<25,result
'''
        try:session.command('python3 -c '+shlex.quote(paused),timeout=28,label='paused recorder wall deadline and child cleanup')
        finally:session.pause(False)
        report['checks'].append({'name':'paused_recorder_bounded_failure_and_cpp_child_cleanup','passed':True,
            'evidence':json.loads((workspace/'build/paused-record-probe.json').read_text(encoding='utf-8'))})
        session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        verify('reentry_preserves_private_code_and_prior_bags')
        for name in ('instrument.py','student.py'):
            assert marker in (workspace/name).read_text(encoding='utf-8'),'Personal edit overwritten: '+name
        session.stop();assert not _token_processes(session.token)
        report['checks'].append({'name':'all_owned_recorder_player_and_ros_processes_cleaned','passed':True,
                                 'distinct_closed_bags':len(saved)})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error);report['traceback']=traceback.format_exc()
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:
            try:session.stop(reason='A46 bag/diagnostic acceptance finished')
            except Exception as error:report['cleanup_error']=str(error);report['ok']=False
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'ok':report['ok'],'elapsed_seconds':report['elapsed_seconds'],
                          'error':report.get('error'),'report':str(target)},ensure_ascii=False))
    return 0 if report['ok'] else 1


if __name__=='__main__':sys.exit(main())
