"""Real UV pixels across baseline, a single-variable edit and a legal corner."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from gazeboteach.catalog import Lesson
from gazeboteach.backends import get_backend
from gazeboteach.storage import Storage
from gazeboteach.runtime import _token_processes


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    options=parser.parse_args();target=Path(options.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/39-texture-workshop'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a39-'))
    started=time.monotonic();session=None
    report={'lesson':'39','ok':False,'data_dir':str(storage.base),'checks':[]}
    try:
        workspace=storage.prepare(lesson)
        config=json.loads((workspace/'parameters.json').read_text())
        for name,left,right,alpha in [('baseline',1,4,.45),('right_repeat_only',1,3,.45),
                                      ('legal_corner',2,3,.45),('opacity_only',2,3,.7)]:
            config.update(uv_repeat_left=left,uv_repeat_right=right,glass_alpha=alpha)
            (workspace/'parameters.json').write_text(json.dumps(config),encoding='utf-8')
            session=get_backend('native').create_session(ROOT,lesson,storage=storage)
            session.start(gui=False,terminals=False)
            assert session.verify()['passed'] is True
            result=json.loads((workspace/'build/verification.json').read_text())
            shutil.copy2(str(session.session_dir/'camera.ppm'),str(target.parent/(name+'.ppm')))
            report['checks'].append({'name':name,'passed':True,'verification':result})
            session.stop();assert not _token_processes(session.token)
        before=report['checks'][-2]['verification']['mean_rgb']['glass']
        after=report['checks'][-1]['verification']['mean_rgb']['glass']
        assert after[0]>before[0]+15 and after[2]<before[2]-15,'Opacity did not change actual red/blue blending'
        report['checks'].append({'name':'actual_opacity_response','passed':True,'before_rgb':before,'after_rgb':after})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A39贴图专项验收结束')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
