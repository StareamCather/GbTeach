"""Actual A50 reruns in one isolated lesson and artifact preservation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.backends import get_backend
from gazeboteach.catalog import Lesson
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True,type=Path)
    output=parser.parse_args().output
    folder=ROOT/'courses/classic11_noetic/50-fast-lio'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gt-a50-repeat-'))
    workspace=storage.prepare(lesson)
    session=get_backend('native').create_session(ROOT,lesson,storage=storage)
    saved={}
    reports=[]
    started=time.monotonic()
    try:
        session.start(gui=False,terminals=False)
        for index in range(2):
            assert session.verify()['passed'],str(session.log_path)
            r=json.loads((workspace/'build/verification.json').read_text(encoding='utf-8'))
            assert r['passed'] and not r['inputs']['truth_published']
            assert r['capture']['scans']==140 and r['capture']['truth_samples']==1400
            assert r['capture']['scan_span_min_ns']==r['capture']['scan_span_max_ns']==99500000
            assert r['capture']['points']>400000
            assert r['imu']['count']>2750 and r['imu']['median_dt_ns']==5000000
            assert r['outputs']['odometry_messages']>110 and r['metrics']['matched']>110
            assert r['metrics']['truth_max_displacement_m']>5
            assert r['metrics']['position_rmse_m']<.15 and r['metrics']['orientation_rmse_rad']<.1
            for f,h in saved.items():
                assert digest(f)==h
            path=Path(r['run_directory']).resolve()
            assert workspace.resolve() in path.parents and path.name=='run-'+r['request_id']
            for name in ('native.jsonl','capture-done.json','sensors.bag','report.json','trajectory-comparison.json'):
                f=path/name
                assert f not in saved
                saved[f]=digest(f)
            assert saved[path/'native.jsonl']==r['capture']['sha256']
            assert saved[path/'sensors.bag']==r['bag']['sha256']
            reports.append(r)
        assert reports[1]['capture']['first_scan_ns']>reports[0]['capture']['last_scan_end_ns']
        assert len({r['request_id'] for r in reports})==2
    finally:
        session.stop()
        assert not _token_processes(session.token)
    report={'ok':True,'runs':reports,'artifacts':{str(f):h for f,h in saved.items()},
            'runtime_log':str(session.log_path),'elapsed_seconds':round(time.monotonic()-started,3),
            'owned_processes_cleaned':True}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':
    main()
