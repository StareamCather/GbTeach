"""Fresh public-CLI regression of each selected profile with private learner copies."""
import argparse
import json
import hashlib
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.modern import MODERN_PROFILES
from gazeboteach.runtime import _token_processes
from gazeboteach.storage import Storage

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def fingerprints(root):
    return {f.relative_to(root).as_posix():digest(f) for f in sorted(root.rglob('*'))
            if f.is_file() and not {'build','__pycache__'}.intersection(f.relative_to(root).parts)}

def write(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temporary.replace(path)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--profile',required=True,choices=['classic11-noetic',*MODERN_PROFILES])
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--ids',nargs='*')
    p.add_argument('--all-gui',action='store_true')
    args=p.parse_args()
    target=args.output.resolve()
    target.mkdir(parents=True,exist_ok=False)
    lessons,errors=load_catalog(ROOT)
    assert not errors,errors
    expected=({f'{i:02d}' for i in range(1,54)}|{'091','092'} if args.profile=='classic11-noetic'
              else {'01','11','22','23','24','25','26'})
    available={x.id:x for x in lessons if x.profile==args.profile}
    if args.ids:
        assert set(args.ids)<=set(available)<=expected
    else:
        assert set(available)==expected
    ids=args.ids or [x.id for x in sorted(available.values(),key=lambda x:x.order)]
    source=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    result={'passed':False,'complete':False,'source_commit':source,'profile':args.profile,
            'ids_requested':ids,'course_count':len(expected),'cases':[]}
    started=time.monotonic()
    write(target/'progress.json',result)
    for id in ids:
        begin=time.monotonic()
        lesson=available[id]
        before=fingerprints(lesson.folder)
        case_dir=target/('lesson-'+id)
        case_dir.mkdir()
        storage=Storage(case_dir/'data')
        workspace=storage.prepare(lesson)
        student_hash=None
        if args.profile=='classic11-noetic' and id in ('091','092'):
            (workspace/'learner.py').write_bytes((lesson.folder/'reference_controller.py').read_bytes())
            (workspace/'runtime.json').write_text('{"mode":"student"}\n',encoding='utf-8')
            student_hash=digest(workspace/'learner.py')
        gui=bool(args.all_gui or lesson.data.get('requires_gui'))
        command=[sys.executable,'-m','gazeboteach','--smoke',id,'--profile',args.profile,
                 '--seconds','0','--data-dir',str(storage.base)]
        if gui:command+=['--gui']
        with (case_dir/'stdout.json').open('w',encoding='utf-8') as stdout,(case_dir/'stderr.log').open('w',encoding='utf-8') as stderr:
            child=subprocess.Popen(command,cwd=ROOT,stdout=stdout,stderr=stderr,start_new_session=True)
            try:
                code=child.wait(timeout=520)
            except subprocess.TimeoutExpired:
                child.send_signal(signal.SIGTERM)
                try:child.wait(timeout=18)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=5)
                code=124
        case={'id':id,'gui_requested':gui,'returncode':code,'elapsed_seconds':round(time.monotonic()-begin,3),
              'source_unchanged':fingerprints(lesson.folder)==before,
              'private_learner_preserved':student_hash is None or digest(workspace/'learner.py')==student_hash}
        try:
            raw=(case_dir/'stdout.json').read_text(encoding='utf-8')
            report=json.loads(raw)
            case['verification']=report.get('verification')
            case['error']=report.get('error')
            case['passed']=code==0 and report.get('ok') is True
        except Exception as error:
            case.update(error=str(error),passed=False)
        live={}
        for f in (storage.base/'sessions').glob('*/session.json'):
            token=json.loads(f.read_text(encoding='utf-8')).get('token')
            if token:live[token]=_token_processes(token)
        case['owned_leftovers']=live
        case['passed']=case['passed'] and not any(live.values()) and case['source_unchanged'] and case['private_learner_preserved']
        write(case_dir/'case.json',case)
        result['cases'].append(case)
        result['elapsed_seconds']=round(time.monotonic()-started,3)
        write(target/'progress.json',result)
        print(json.dumps({k:case.get(k) for k in ('id','passed','elapsed_seconds','error')},ensure_ascii=False),flush=True)
    result['complete']=True
    result['passed']=all(case['passed'] for case in result['cases'])
    write(target/'progress.json',result)
    return 0 if result['passed'] else 1

if __name__=='__main__':
    raise SystemExit(main())
