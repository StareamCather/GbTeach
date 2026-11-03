"""Measure actual lighting response with one changed factor per baseline comparison."""
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
    folder=ROOT/'courses/classic11_noetic/40-lighting-studio'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a40-'))
    started=time.monotonic();session=None
    report={'lesson':'40','ok':False,'data_dir':str(storage.base),'checks':[]}
    try:
        workspace=storage.prepare(lesson)
        baseline=json.loads((workspace/'parameters.json').read_text())
        results={}
        for name,change in [('baseline',{}),('attenuation_only',{'attenuation_quadratic':.12}),
                            ('directional_only',{'directional_fill_intensity':.10})]:
            config=dict(baseline,**change)
            (workspace/'parameters.json').write_text(json.dumps(config),encoding='utf-8')
            session=get_backend('native').create_session(ROOT,lesson,storage=storage)
            session.start(gui=False,terminals=False)
            assert session.verify()['passed'] is True
            result=json.loads((workspace/'build/verification.json').read_text())
            shutil.copy2(str(session.session_dir/'camera.ppm'),str(target.parent/(name+'.ppm')))
            report['checks'].append({'name':name,'passed':True,'verification':result});results[name]=result
            session.stop();assert not _token_processes(session.token)
        before=results['baseline']['graycard_rgb'];attenuated=results['attenuation_only']['graycard_rgb']
        brighter=results['directional_only']['graycard_rgb']
        assert attenuated['red_spot'][0]<before['red_spot'][0]-5,'Red spot attenuation has no measured effect'
        assert attenuated['blue_spot'][2]<before['blue_spot'][2]-5,'Blue spot attenuation has no measured effect'
        assert brighter['red_spot'][1]>before['red_spot'][1]+3,'Neutral directional fill did not brighten green channel'
        assert brighter['blue_spot'][1]>before['blue_spot'][1]+3,'Neutral directional fill did not affect second card'
        report['checks'].append({'name':'real_parameter_effects','passed':True,'baseline_rgb':before,
                                 'attenuated_rgb':attenuated,'directional_rgb':brighter})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A40光照专项验收结束')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
