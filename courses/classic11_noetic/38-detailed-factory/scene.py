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
 p=params();table=finite(p,'table_height_m',.65,1.35);pallet=finite(p,'pallet_height_m',.2,.6);width=finite(p,'machine_width_m',1,2);aisle=finite(p,'aisle_width_m',1.4,2.4)
 s=header('lesson38')
 s+='<light name="sun" type="directional"><diffuse>.8 .8 .8 1</diffuse><direction>-.5 .1 -1</direction><cast_shadows>true</cast_shadows></light>'
 s+=box('floor','14 12 .1','0 0 -.05 0 0 0','.3 .34 .38 1')
 # Side walls and a real three-piece rear doorway; the aperture stays collision-free.
 for side,x in [('left',-4),('right',4)]:s+=box('wall_'+side,'.2 8 2.8',f'{x} 1 1.4 0 0 0','.65 .68 .7 1')
 for side,x in [('left',-2.35),('right',2.35)]:s+=box('doorwall_'+side,'3.3 .2 2.8',f'{x} 5 1.4 0 0 0','.65 .68 .7 1')
 s+=box('doorwall_lintel','1.4 .2 .7','0 5 2.45 0 0 0','.65 .68 .7 1')
 # Each rack is .8 m deep: +/- (clearance/2 + half-depth) gives the requested clear aisle.
 for y in [-(aisle/2+.4),aisle/2+.4]:
  for x in [-2.8,0,2.8]:
   s+=box('rack_'+str(x)+'_'+str(y),'1.8 .8 .08',f'{x} {y} 1.1 0 0 0','.65 .4 .15 1')
   for dx in [-.85,.85]:
    for dy in [-.35,.35]:s+=box('post_'+str(x+dx)+'_'+str(y+dy),'.07 .07 2.3',f'{x+dx} {y+dy} 1.15 0 0 0','.2 .3 .4 1')
   for k in range(4):s+=box('crate_'+str(x)+'_'+str(y)+'_'+str(k),'.35 .6 .3',f'{x-.6+k*.4} {y} 1.29 0 0 0','.65 .43 .22 1')
 # Detailed visual mesh uses one simple outer collision proxy; small trim has no contact detail.
 s+=f'<model name="cnc_cell"><static>true</static><pose>2 0 .65 0 0 0</pose><link name="machine"><collision name="proxy"><geometry><box><size>{width} 1.2 1.3</size></box></geometry></collision><visual name="shell_mesh"><geometry><mesh><uri>model://factory_detail/meshes/cell.dae</uri><scale>{width/1.6} 1 1</scale></mesh></geometry></visual></link></model>'
 s+=box('table_surface','1.2 1 .1',f'-2 0 {table-.05} 0 0 0','.25 .55 .65 1')
 for x in [-2.5,-1.5]:
  for y in [-.4,.4]:s+=box('leg_'+str(x)+'_'+str(y),f'.06 .06 {table-.1}',f'{x} {y} {(table-.1)/2} 0 0 0','.2 .25 .3 1')
 # Two pallet strips leave a real gap. The third probe falls through, despite a translucent visual cover.
 for y in [-.32,.32]:s+=box('pallet_strip_'+str(y),f'1.2 .22 {pallet}',f'0 {y} {pallet/2} 0 0 0','.55 .32 .12 1')
 s+=box('pallet_visual_wrap','1.2 .85 .015',f'0 0 {pallet+.01} 0 0 0','.2 .7 .5 .35',collision=False)
 for n,x,y in [('table',-2,0),('pallet',0,.32),('gap',0,0)]:
  s+=f'<model name="probe_{n}"><pose>{x} {y} 2.3 0 0 0</pose><link name="ball"><inertial><mass>.15</mass><inertia><ixx>.000384</ixx><iyy>.000384</iyy><izz>.000384</izz></inertia></inertial><collision name="ball"><geometry><sphere><radius>.08</radius></sphere></geometry><surface><bounce><restitution_coefficient>0</restitution_coefficient></bounce></surface></collision><visual name="ball"><geometry><sphere><radius>.08</radius></sphere></geometry><material><ambient>1 .3 .1 1</ambient><diffuse>1 .3 .1 1</diffuse></material></visual></link></model>'
 s+='<plugin name="real_geometry" filename="libgt_probes.so"/>'+footer();write_world(s)
 return p

def validate():
 p=params();finite(p,'table_height_m',.65,1.35);finite(p,'pallet_height_m',.2,.6);finite(p,'machine_width_m',1,2);finite(p,'aisle_width_m',1.4,2.4)
 world=ET.parse('world.sdf').find('world')
 if world is None or world.get('name')!='lesson38':raise ValueError('Keep lesson38 world identity')
 mesh=Path('models/factory_detail/meshes/cell.dae')
 if not mesh.is_file():raise ValueError('Missing offline detailed mesh')
 rack_y=[float(m.findtext('pose').split()[1]) for m in world.findall('model') if m.get('name','').startswith('rack_')]
 clearance=min(y-.4 for y in rack_y if y>0)-max(y+.4 for y in rack_y if y<0)
 if abs(clearance-p['aisle_width_m'])>1e-8:raise ValueError('Generated aisle clearance differs from parameters; restart to rebuild current inputs')
 ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
 mesh_xml=ET.parse(str(mesh));triangles=sum(int(t.get('count','0')) for t in mesh_xml.findall('.//c:triangles',ns))
 if triangles<=0:raise ValueError('Detailed mesh contains no triangles')
 return {'parameters':p,'model_count':len(world.findall('model')),'measured_aisle_clearance_m':clearance,'rear_door_clear_width_m':1.4,'rear_door_clear_height_m':2.1,'mesh_triangle_count':triangles,'mesh_bytes':mesh.stat().st_size,'physics_expectations_m':{'probe_table':p['table_height_m']+.08,'probe_pallet':p['pallet_height_m']+.08,'probe_gap':.08}}

if __name__=='__main__':print(build())
