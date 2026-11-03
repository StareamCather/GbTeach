"""Real prescribed wind across uniform, zero, shear, gust, mass and area cases."""
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


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    options=parser.parse_args();target=Path(options.output).resolve();target.parent.mkdir(parents=True,exist_ok=True)
    folder=ROOT/'courses/classic11_noetic/41-wind-tunnel'
    lesson=Lesson(folder,json.loads((folder/'lesson.json').read_text(encoding='utf-8')))
    storage=Storage(tempfile.mkdtemp(prefix='gazeboteach-a41-'))
    started=time.monotonic();session=None
    report={'lesson':'41','ok':False,'data_dir':str(storage.base),'checks':[]}
    try:
        workspace=storage.prepare(lesson)
        baseline=json.loads((workspace/'parameters.json').read_text())
        results={}
        for name,change in [('baseline',{}),('no_wind',{'base_wind_m_s':0}),
                            ('area_only',{'area_m2':.15}),('mass_only',{'mass_kg':.75}),
                            ('shear_only',{'shear_per_s':.2}),('gust_only',{'gust_m_s':.6})]:
            config=dict(baseline,**change)
            (workspace/'parameters.json').write_text(json.dumps(config),encoding='utf-8')
            session=get_backend('native').create_session(ROOT,lesson,storage=storage)
            session.start(gui=False,terminals=False)
            assert session.verify()['passed'] is True
            result=json.loads((workspace/'build/verification.json').read_text())
            report['checks'].append({'name':name,'passed':True,'verification':result});results[name]=result
            session.stop();assert not _token_processes(session.token)
        def nominal(name):
            return next(row['actual_displacement_m'] for row in results[name]['comparison'] if row['name']=='cart_nominal')
        assert nominal('area_only')>nominal('baseline')+.05
        assert nominal('mass_only')<nominal('baseline')-.05
        assert nominal('gust_only')>nominal('baseline')+.05
        assert abs(nominal('no_wind'))<1e-4
        report['checks'].append({'name':'causal_parameter_response','passed':True,
                                 'nominal_displacement_m':{name:nominal(name) for name in results}})
        report['ok']=True
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        if session:report['runtime_log']=str(session.log_path)
    finally:
        if session:session.stop(reason='A41风场专项验收结束')
        report['elapsed_seconds']=round(time.monotonic()-started,3)
        target.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['ok'] else 1

if __name__=='__main__':sys.exit(main())
