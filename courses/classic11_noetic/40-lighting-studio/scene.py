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
 return '<gui fullscreen="0"><camera name="user_camera"><pose>-4 -6 5 0 .42 .75</pose><view_controller>orbit</view_controller></camera></gui></world></sdf>'

def write_world(text):
 ET.fromstring(text)
 Path('world.sdf').write_text(text,encoding='utf-8')

def finite(p,name,lo,hi):
 value=p[name]
 if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not lo<=value<=hi:raise ValueError(name+' outside documented finite limits')
 return value

def build():
 p=params();a=finite(p,'ambient',.01,.2);red=finite(p,'red_intensity',.4,1);blue=finite(p,'blue_intensity',.4,1);linear=finite(p,'attenuation_linear',0,.5);quadratic=finite(p,'attenuation_quadratic',0,.2);outer=finite(p,'spot_outer_rad',.5,1.2);fill=finite(p,'directional_fill_intensity',0,1)
 s=header('lesson40',f'{a} {a} {a} 1')
 for name,y in [('red',1.8),('gray',0),('blue',-1.8)]:
  s+=box('graycard_'+name,'.08 1.6 1.6',f'4 {y} 1.5 0 0 0','.8 .8 .8 1')
 # Two actual spot key lights; directional and point fills remain independent.
 for name,y,color in [('red',1.8,f'{red} .01 .01 1'),('blue',-1.8,f'.01 .01 {blue} 1')]:
  s+=f'<light name="{name}_spot" type="spot"><pose>1.2 {y} 1.5 0 0 0</pose><diffuse>{color}</diffuse><specular>{color}</specular><direction>1 0 0</direction><cast_shadows>true</cast_shadows><attenuation><range>10</range><constant>1</constant><linear>{linear}</linear><quadratic>{quadratic}</quadratic></attenuation><spot><inner_angle>.25</inner_angle><outer_angle>{outer}</outer_angle><falloff>1</falloff></spot></light>'
 s+='<light name="white_fill" type="point"><pose>2 .1 3.4 0 0 0</pose><diffuse>.3 .3 .3 1</diffuse><specular>.6 .6 .6 1</specular><attenuation><range>8</range><constant>1</constant><linear>.2</linear><quadratic>.1</quadratic></attenuation><cast_shadows>true</cast_shadows></light>'
 s+=f'<light name="neutral_directional" type="directional"><pose>0 0 8 0 0 0</pose><direction>1 0 -.5</direction><diffuse>{fill} {fill} {fill} 1</diffuse><specular>{fill} {fill} {fill} 1</specular><cast_shadows>true</cast_shadows></light>'
 s+=box('shadow_gobo','.12 .22 1.1','3 0 1.6 0 0 0','.12 .12 .12 1')
 s+=box('studio_floor','8 8 .1','3 0 -.05 0 0 0','.25 .25 .25 1')
 s+=camera()+footer();write_world(s);return p

def validate():
 p=params();finite(p,'ambient',.01,.2);finite(p,'red_intensity',.4,1);finite(p,'blue_intensity',.4,1);finite(p,'attenuation_linear',0,.5);finite(p,'attenuation_quadratic',0,.2);finite(p,'spot_outer_rad',.5,1.2);finite(p,'directional_fill_intensity',0,1)
 w=ET.parse('world.sdf').find('world')
 if w is None or w.get('name')!='lesson40':raise ValueError('Keep lesson40 world')
 return {'parameters':p,'attenuation_at_2_8m':1/(1+p['attenuation_linear']*2.8+p['attenuation_quadratic']*2.8**2),'graycard_diffuse':.8,'lights':[{'name':l.get('name'),'type':l.get('type')} for l in w.findall('light')]}
if __name__=='__main__':print(build())
