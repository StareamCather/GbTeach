#!/usr/bin/env python3
"""Compare actual native drag dynamics to an independent finite-step prediction."""
import json,math,os,time,sys
from pathlib import Path
from scene import validate

def predict(config,item,t):
 # Same physical equation, independent Python integrator; dt=.001 matches course solver timestep.
 x=v=0.;age=0.;dt=.001
 while age<t-1e-8:
  h=min(dt,t-age);wind=config['base_wind_m_s']+config['shear_per_s']*item['y_m']+config['gust_m_s']*math.sin(2*math.pi*age/config['gust_period_s'])
  if not item['wind_on']:wind=0.
  u=wind-v;force=.5*config['density_kg_m3']*config['drag_coefficient']*item['area_m2']*u*abs(u)
  force=max(-20,min(20,force));v+=force/item['mass_kg']*h;x+=v*h;age+=h
 return x,v

def run():
 config=validate();end=time.monotonic()+22;first=None;actual=None
 while time.monotonic()<end:
  try:
   actual=json.loads((Path(os.environ['GAZEBOTEACH_SESSION_DIR'])/'wind.json').read_text())
   if actual['session_token']!=os.environ['GAZEBOTEACH_SESSION_TOKEN']:raise RuntimeError('Cross-session telemetry rejected')
   if first is None:first=actual['sequence']
   sample=actual['sample_at_one_second']
   if actual['sequence']>first and sample and len(sample['carts'])==4:
    rows=[]
    for item in sample['carts']:
     x,v=predict(config,item,sample['elapsed_s']);rows.append(dict(name=item['name'],predicted_displacement_m=x,actual_displacement_m=item['displacement_m'],predicted_velocity_m_s=v,actual_velocity_m_s=item['velocity_m_s'],pass_dynamics=abs(x-item['displacement_m'])<.025 and abs(v-item['velocity_m_s'])<.025))
    by={i['name']:i for i in sample['carts']}
    expected={'cart_calm':(1,1,False),'cart_nominal':(1,1,True),'cart_wide':(2,1,True),'cart_heavy':(1,2,True)}
    actual_inputs_match=set(by)==set(expected) and all(abs(by[name]['area_m2']-config['area_m2']*am)<1e-6 and abs(by[name]['mass_kg']-config['mass_kg']*mm)<max(1e-6,config['mass_kg']*mm*1e-5) and by[name]['wind_on'] is on for name,(am,mm,on) in expected.items())
    checks={'fresh_actual_updates':actual['sequence']>first,'four_actual_models':len({i['model_id'] for i in sample['carts']})==4,'actual_mass_area_switches_match_parameters':actual_inputs_match,'bounded_equation_matches_physics':all(r['pass_dynamics'] for r in rows),'calm_stays_still':abs(by['cart_calm']['displacement_m'])<1e-4}
    zero_field=config['base_wind_m_s']==0 and config['shear_per_s']==0 and config['gust_m_s']==0
    if zero_field:checks['zero_field_stays_still']=all(abs(i['displacement_m'])<1e-4 and abs(i['velocity_m_s'])<1e-4 for i in sample['carts'])
    elif config['base_wind_m_s']>=2:checks['wind_causes_motion']=by['cart_nominal']['displacement_m']>.1
    if config['shear_per_s']==0 and config['gust_m_s']==0 and config['base_wind_m_s']>0:
     checks['larger_area_moves_further']=by['cart_wide']['displacement_m']>by['cart_nominal']['displacement_m']
     checks['larger_mass_moves_less']=by['cart_heavy']['displacement_m']<by['cart_nominal']['displacement_m']
    return {'passed':all(checks.values()),'checks':checks,'parameters':config,'actual':actual,'comparison':rows,'model_scope':'scalar longitudinal drag in prescribed uniform/shear/gust field; no fluid solver, propeller wake or lift'}
  except (OSError,ValueError,KeyError):pass
  time.sleep(.1)
 raise RuntimeError('No fresh native wind measurement: '+str(actual))
if __name__=='__main__':
 try:
  r=run()
  Path('build').mkdir(exist_ok=True)
  Path('build/verification.json').write_text(json.dumps(r,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
  print(json.dumps(r,ensure_ascii=False,allow_nan=False));sys.exit(0 if r['passed'] else 1)
 except Exception as e:print(json.dumps({'passed':False,'error':type(e).__name__+': '+str(e)},ensure_ascii=False));sys.exit(1)
