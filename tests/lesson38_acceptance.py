"""Real supplementary scene38 acceptance; only test-owned practice is edited."""
import argparse
import json
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
from tests.modern_runtime_acceptance import capture_owned_gui


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    options=parser.parse_args();target=Path(options.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/38-detailed-factory'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a38-'))
    started=time.monotonic();session=None
    report={'lesson':'38','ok':False,'data_dir':str(storage.base),'checks':[]}
    def passed(name,**values):report['checks'].append({'name':name,'passed':True,**values})
    try:
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=True,terminals=True)
        assert session.verify()['passed'] is True
        before=json.loads((session.session_dir/'physics.json').read_text())
        assert session.health()['terminal_running']==2
        captured=capture_owned_gui(session,target.parent/'a38-factory-overview.png')
        passed('real_baseline_probes_and_window',actual=before,gui=captured)
        # Follow the documented single-variable exercise in a private practice copy.
        config=json.loads((session.workspace/'parameters.json').read_text())
        config['table_height_m']=1.2
        (session.workspace/'parameters.json').write_text(json.dumps(config),encoding='utf-8')
        session.stop();assert not _token_processes(session.token)
        session=get_backend('native').create_session(ROOT,lesson,storage=storage)
        session.start(gui=False,terminals=False)
        assert session.verify()['passed'] is True
        after=json.loads((session.session_dir/'physics.json').read_text())
        old={p['name']:p['z'] for p in before['probes']}
        new={p['name']:p['z'] for p in after['probes']}
        assert abs(new['probe_table']-old['probe_table']-.2)<.025
        assert abs(new['probe_pallet']-old['probe_pallet'])<.025
        assert abs(new['probe_gap']-old['probe_gap'])<.025
        passed('student_table_height_change_only',before=old,after=new)
        session.stop();assert not _token_processes(session.token)
        passed('all_test_owned_processes_cleaned')
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A38专项验收结束')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
