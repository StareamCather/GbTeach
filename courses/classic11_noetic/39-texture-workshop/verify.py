#!/usr/bin/env python3
"""Fresh real render evidence, not success inferred from loading the scene."""
import json,math,os,time,sys
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
 # Cards project at x≈131 and x≈320; use their interior midline.
 def transitions(lo,hi):
  vals=[pixels[(180*640+x)*3] for x in range(lo,hi)]
  return sum((a<120)!=(b<120) for a,b in zip(vals,vals[1:]))
 left=transitions(65,195);right=transitions(255,385)
 a,ar=roi(pixels,70,190);b,br=roi(pixels,260,380);glass,gr=roi(pixels,450,565)
 # The immutable course camera has fixed projection: card width = focal*1.6/4.
 # A PNG edge spans four checker cells; a repeat scales their spatial frequency.
 focal=640/(2*math.tan(1.3/2));projected_width=focal*1.6/4
 predictions={}
 for name,repeat,observed in [('left',config['uv_repeat_left'],left),('right',config['uv_repeat_right'],right)]:
  expected=4*repeat*(130-1)/projected_width
  predictions[name]={'observed':observed,'expected_boundary_density_count':expected,'edge_tolerance':2.0,'minimum':max(1,expected-2),'maximum':expected+2}
 red_share=glass[0]/max(1,glass[0]+glass[2])
 checks={'actual_frame_advances':meta['sequence']>=10,'both_cards_have_contrast':ar>100 and br>100,'left_uv_matches_current_repeat':predictions['left']['minimum']<=left<=predictions['left']['maximum'],'right_uv_matches_current_repeat':predictions['right']['minimum']<=right<=predictions['right']['maximum'],'higher_uv_has_more_boundaries':right>left,'transparent_red_and_blue_mix':glass[0]>35 and glass[2]>35,'glass_is_purple_not_white':min(glass[0],glass[2])>glass[1]*2+20,'glass_mix_tracks_current_alpha':abs(red_share-config['glass_alpha'])<.18}
 return {'passed':all(checks.values()),'checks':checks,'actual':meta,'uv_boundary_counts':{'left':left,'right':right},'uv_boundary_predictions':predictions,'projected_card_width_px':projected_width,'glass_red_share':red_share,'mean_rgb':{'left':a,'right':b,'glass':glass},'parameters':config,'image_path':str(Path(os.environ['GAZEBOTEACH_SESSION_DIR'])/'camera.ppm'),'visual_review':'manual: UV seams, oblique filtering, alpha sorting and aliasing'}

if __name__=='__main__':
 try:
  result=run()
  Path('build').mkdir(exist_ok=True)
  Path('build/verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
  print(json.dumps(result,ensure_ascii=False,allow_nan=False));sys.exit(0 if result['passed'] else 1)
 except Exception as e:print(json.dumps({'passed':False,'error':type(e).__name__+': '+str(e)},ensure_ascii=False));sys.exit(1)
