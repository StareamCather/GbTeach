import json, math, xml.etree.ElementTree as ET
from pathlib import Path

def params():
 p=json.loads(Path('parameters.json').read_text(encoding='utf-8'))
 return p

def box(name,size,pose,color='0.6 0.65 0.7 1',collision=True,static=True):
 c=f'<collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>' if collision else ''
 return f'<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose><link name="body">{c}<visual name="visual"><geometry><box><size>{size}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'

def header(name,ambient='.25 .25 .25 1'):
 return f'<sdf version="1.6"><world name="{name}"><physics type="ode"><max_step_size>.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics><gravity>0 0 -9.81</gravity><scene><ambient>{ambient}</ambient><background>.1 .12 .15 1</background><shadows>true</shadows></scene>'

def camera():
 return '<model name="measurement_camera"><static>true</static><pose>0 0 1.5 0 0 0</pose><link name="mount"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>15</update_rate><camera><horizontal_fov>1.3</horizontal_fov><image><width>640</width><height>360</height><format>R8G8B8</format></image><clip><near>.05</near><far>20</far></clip></camera><plugin name="actual_frame" filename="libgt_camera.so"/></sensor></link></model>'

def footer():
 return '<gui fullscreen="0"><camera name="user_camera"><pose>-6 -9 9 0 .609 .844</pose><view_controller>orbit</view_controller></camera></gui></world></sdf>'

def write_world(text):
 ET.fromstring(text)
 Path('world.sdf').write_text(text,encoding='utf-8')

def finite(p,name,lo,hi):
 value=p[name]
 if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:raise ValueError(name+' outside documented finite limits')
 return value

def build():
 p=params();rho=finite(p,'density_kg_m3',.8,1.5);cd=finite(p,'drag_coefficient',.5,1.8);speed=finite(p,'base_wind_m_s',0,6);shear=finite(p,'shear_per_s',-.25,.25);gust=finite(p,'gust_m_s',0,.8);period=finite(p,'gust_period_s',1,4);area=finite(p,'area_m2',.05,.2);mass=finite(p,'mass_kg',.3,1)
 s=header('lesson41')+'<light name="sun" type="directional"><direction>-.4 -.2 -1</direction><diffuse>.8 .8 .8 1</diffuse></light>'
 s+=box('wind_floor','18 8 .1','2 0 -.05 0 0 0','.25 .3 .35 1')
 cards=[]
 for name,y,am,mm,on in [('calm',2.1,1,1,False),('nominal',.7,1,1,True),('wide',-.7,2,1,True),('heavy',-2.1,1,2,True)]:
  a=area*am;m=mass*mm
  color={'calm':'.45 .5 .55 1','nominal':'.15 .5 .75 1','wide':'.15 .65 .3 1','heavy':'.8 .2 .15 1'}[name]
  s+=f'<model name="cart_{name}"><pose>-3 {y} .6 0 0 0</pose><link name="body"><gravity>false</gravity><inertial><mass>{m}</mass><inertia><ixx>{m*.01}</ixx><iyy>{m*.01}</iyy><izz>{m*.01}</izz></inertia></inertial><visual name="cart"><geometry><box><size>.45 .35 .2</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual><visual name="sail"><pose>0 0 .28 0 0 0</pose><geometry><box><size>.025 {a**.5} {a**.5}</size></box></geometry><material><ambient>.95 .65 .12 1</ambient><diffuse>.95 .65 .12 1</diffuse></material></visual></link></model>'
  s+=box('rail_'+name,'15 .04 .04',f'2 {y} .45 0 0 0','.7 .7 .7 1',collision=False)
  cards.append(f'<cart><name>cart_{name}</name><area>{a}</area><wind_on>{str(on).lower()}</wind_on></cart>')
 for x in range(-3,10):
  s+=box('tick_'+str(x),'.025 6 .015',f'{x} 0 .015 0 0 0','.6 .6 .6 1',collision=False)
 s+=f'<plugin name="drag_wind" filename="libgt_wind.so"><density>{rho}</density><cd>{cd}</cd><wind>{speed}</wind><shear>{shear}</shear><gust>{gust}</gust><period>{period}</period>'+''.join(cards)+'</plugin>'+footer();write_world(s);return p

def validate():
 p=params()
 for key,lo,hi in [('density_kg_m3',.8,1.5),('drag_coefficient',.5,1.8),('base_wind_m_s',0,6),('shear_per_s',-.25,.25),('gust_m_s',0,.8),('gust_period_s',1,4),('area_m2',.05,.2),('mass_kg',.3,1)]:finite(p,key,lo,hi)
 world=ET.parse('world.sdf').find('world')
 if world is None or world.get('name')!='lesson41':raise ValueError('Keep lesson41 world identity')
 return p
if __name__=='__main__':print(build())
