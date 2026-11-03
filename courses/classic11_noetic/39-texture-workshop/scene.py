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
 p=params();left=finite(p,'uv_repeat_left',1,2);right=finite(p,'uv_repeat_right',3,6);alpha=finite(p,'glass_alpha',.2,.8)
 s=header('lesson39',ambient='1 1 1 1')
 # UV coordinates lie in mesh; OGRE receives a real texture unit with wrapping.
 for name,y,repeat in [('left',1.8,left),('right',0,right)]:
  mesh=Path('models/texture_cards/meshes')/(name+'.dae');mesh.parent.mkdir(parents=True,exist_ok=True)
  mesh.write_text(quad(repeat),encoding='utf-8')
  s+=f'<model name="card_{name}"><static>true</static><pose>4 {y} 1.5 0 0 0</pose><link name="panel"><visual name="uv_quad"><geometry><mesh><uri>model://texture_cards/meshes/{name}.dae</uri></mesh></geometry><material><lighting>false</lighting><script><uri>model://texture_cards/materials/scripts/cards.material</uri><uri>model://texture_cards/materials/textures</uri><name>GT/Checker</name></script></material></visual></link></model>'
 s+=box('blue_back',' .08 1.6 1.6','4.1 -1.8 1.5 0 0 0','0 .05 1 1')
 # Use one real geometric plane and the installed Classic manual-colour alpha-blend pattern.
 # Lighting-off diffuse alone becomes white; explicit texture-unit colour/alpha operations are required.
 Path('models/texture_cards/meshes/glass.dae').write_text(quad(1),encoding='utf-8')
 glass_script=Path('models/texture_cards/materials/scripts/glass.material')
 glass_script.write_text(glass_material(alpha),encoding='utf-8')
 s+='<model name="glass"><static>true</static><pose>3.95 -1.8 1.5 0 0 0</pose><link name="panel"><visual name="glass"><geometry><mesh><uri>model://texture_cards/meshes/glass.dae</uri></mesh></geometry><material><script><uri>model://texture_cards/materials/scripts/glass.material</uri><name>GT/Glass</name></script></material></visual></link></model>'
 s+=camera()+footer();write_world(s);return p

def glass_material(alpha):
 return ("material GT/Glass\n{\n technique\n {\n  pass\n  {\n   scene_blend alpha_blend\n   depth_write off\n   depth_check on\n   lighting off\n   cull_hardware none\n   cull_software none\n   texture_unit\n   {\n    colour_op_ex source1 src_manual src_current 1 0 0\n    alpha_op_ex source1 src_manual src_current "+format(alpha,'.12g')+"\n   }\n  }\n }\n}\n")

def quad(repeat):
 return f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset><library_geometries><geometry id="card"><mesh><source id="p"><float_array id="pa" count="12">0 -.8 -.8 0 .8 -.8 0 .8 .8 0 -.8 .8</float_array><technique_common><accessor source="#pa" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source><source id="uv"><float_array id="ua" count="8">0 0 {repeat} 0 {repeat} {repeat} 0 {repeat}</float_array><technique_common><accessor source="#ua" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source><vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="2"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="TEXCOORD" source="#uv" offset="1" set="0"/><p>0 0 2 2 1 1 0 0 3 3 2 2</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="s"><node><instance_geometry url="#card"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>'''

def validate():
 p=params();finite(p,'uv_repeat_left',1,2);finite(p,'uv_repeat_right',3,6);finite(p,'glass_alpha',.2,.8)
 w=ET.parse('world.sdf').find('world')
 if w is None or w.get('name')!='lesson39':raise ValueError('Keep lesson39 world')
 for n in ['left','right']:
  if not Path('models/texture_cards/meshes/'+n+'.dae').is_file():raise ValueError('Missing UV mesh')
 if not Path('models/texture_cards/materials/textures/checker.png').is_file():raise ValueError('Missing offline checker PNG')
 return p
if __name__=='__main__':print(build())
