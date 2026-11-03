#!/usr/bin/env python3
"""Fresh real render evidence, not success inferred from loading the scene."""
import json,os,time,sys
from pathlib import Path
from scene import validate

def frame():
 root=Path(os.environ['GAZEBOTEACH_SESSION_DIR']); meta=json.loads((root/'camera.json').read_text())
 raw=(root/'camera.ppm').read_bytes(); head,wh,maxv,pixels=raw.split(b'\n',3)
 w,h=map(int,wh.split())
 if head!=b'P6' or maxv!=b'255' or (w,h)!=(640,360) or len(pixels)!=w*h*3:raise ValueError('Invalid actual camera frame')
 checksum=14695981039346656037
 for v in pixels:checksum=((checksum^v)*1099511628211)&((1<<64)-1)
 if str(checksum)!=meta['fnv64']:raise ValueError('Frame changed while reading; bounded retry')
 return meta,pixels

def roi(pixels,x0,x1,y0=140,y1=220):
 vals=[pixels[(y*640+x)*3+c] for y in range(y0,y1) for x in range(x0,x1) for c in range(3)]
 means=[sum(vals[c::3])/len(vals[c::3]) for c in range(3)]
 return means,max(vals)-min(vals)

def run():
 config=validate();end=time.monotonic()+18;first=None;last_error='No camera frame'
 while time.monotonic()<end:
  try:
   m,p=frame()
   if first is None:first=m
   if m['sequence']>=first['sequence']+5 and m['sim_time_s']>first['sim_time_s']:
    return assess(config,m,p)
  except (OSError,ValueError,KeyError) as e:last_error=str(e)
  time.sleep(.15)
 raise RuntimeError('No progressing, internally consistent real camera frames: '+last_error)

def assess(config,meta,pixels):
 red,rr=roi(pixels,95,165);blue,br=roi(pixels,475,545);gray,gr=roi(pixels,285,355)
 checks={'actual_frame_advances':meta['sequence']>=10,'red_light_changes_graycard':red[0]>max(red[1],red[2])*1.5 and red[0]>25,'blue_light_changes_graycard':blue[2]>max(blue[0],blue[1])*1.5 and blue[2]>25,'finite_illuminated_scene':max(pixels)>60 and min(pixels)<50}
 return {'passed':all(checks.values()),'checks':checks,'actual':meta,'graycard_rgb':{'red_spot':red,'blue_spot':blue,'occluded_center':gray},'configuration':config,'image_path':str(Path(os.environ['GAZEBOTEACH_SESSION_DIR'])/'camera.ppm'),'visual_review':'manual: shadow edge, cast_shadows toggle, specular view dependence; auto check does not certify photometric accuracy'}

if __name__=='__main__':
 try:
  result=run()
  Path('build').mkdir(exist_ok=True)
  Path('build/verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
  print(json.dumps(result,ensure_ascii=False,allow_nan=False));sys.exit(0 if result['passed'] else 1)
 except Exception as e:print(json.dumps({'passed':False,'error':type(e).__name__+': '+str(e)},ensure_ascii=False));sys.exit(1)
