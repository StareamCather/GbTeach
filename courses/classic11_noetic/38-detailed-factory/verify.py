#!/usr/bin/env python3
import json,os,time,sys
from pathlib import Path
from scene import validate

def run():
 config=validate();end=time.monotonic()+20;first=None;actual=None
 while time.monotonic()<end:
  try:
   actual=json.loads((Path(os.environ['GAZEBOTEACH_SESSION_DIR'])/'physics.json').read_text())
   if first is None:first=actual['sequence']
   probes={p['name']:p for p in actual['probes']}
   if actual['sequence']>first and len(probes)==3 and actual['sim_time_s']>1.3 and all(p['speed']<.04 for p in probes.values()):
    checks={name:abs(probes[name]['z']-height)<.025 for name,height in config['physics_expectations_m'].items()}
    return {'passed':all(checks.values()),'checks':checks,'actual':actual,'configuration':config,'visual_mesh_review':'manual: inspect mesh silhouette, assembly detail, normals and visual/collision overlay'}
  except (OSError,ValueError,KeyError):pass
  time.sleep(.1)
 raise RuntimeError('Real geometry probes did not settle/progress: '+str(actual))
if __name__=='__main__':
 try:r=run();print(json.dumps(r,ensure_ascii=False));sys.exit(0 if r['passed'] else 1)
 except Exception as e:print(json.dumps({'passed':False,'error':str(e)}));sys.exit(1)
