#!/usr/bin/env python3
"""Offline closed-box assembly to a single COLLADA mesh; metres, Z-up, UV seams."""
from pathlib import Path
def dae_boxes(boxes):
 pos=[];nor=[];uv=[]
 for cx,cy,cz,sx,sy,sz in boxes:
  vs=[(cx+dx*sx/2,cy+dy*sy/2,cz+dz*sz/2) for dx,dy,dz in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
  for a,b,c,d,n in [(0,3,2,1,(0,0,-1)),(4,5,6,7,(0,0,1)),(0,1,5,4,(0,-1,0)),(3,7,6,2,(0,1,0)),(0,4,7,3,(-1,0,0)),(1,2,6,5,(1,0,0))]:
   for k in [a,b,c,a,c,d]:pos.extend(vs[k]);nor.extend(n)
   uv.extend([0,0,1,0,1,1,0,0,1,1,0,1])
 count=len(pos)//3
 sources=''
 for name,arr,stride,fields in [('p',pos,3,'XYZ'),('n',nor,3,'XYZ'),('uv',uv,2,'ST')]:
  sources+=f'<source id="{name}"><float_array id="{name}_array" count="{len(arr)}">'+ ' '.join(map(str,arr))+f'</float_array><technique_common><accessor source="#{name}_array" count="{len(arr)//stride}" stride="{stride}">'+''.join(f'<param name="{c}" type="float"/>' for c in fields)+'</accessor></technique_common></source>'
 indices=' '.join(f'{i} {i} {i}' for i in range(count))
 return f'''<?xml version="1.0"?><COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1"><asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset><library_effects><effect id="paint"><profile_COMMON><technique sid="common"><phong><diffuse><color>.15 .45 .55 1</color></diffuse><specular><color>.45 .45 .45 1</color></specular><shininess><float>45</float></shininess></phong></technique></profile_COMMON></effect></library_effects><library_materials><material id="steel"><instance_effect url="#paint"/></material></library_materials><library_geometries><geometry id="cell"><mesh>{sources}<vertices id="v"><input semantic="POSITION" source="#p"/></vertices><triangles count="{count//3}" material="paint"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#n" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>{indices}</p></triangles></mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene"><node id="assembly"><instance_geometry url="#cell"><bind_material><technique_common><instance_material symbol="paint" target="#steel"><bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''

def build():
 # Structural enclosure, bed, explicit louvers and bolt heads are visual details.
 boxes=[(0,0,-.55,1.6,1.2,.2),(-.72,0,.1,.16,1.2,1.1),(.72,0,.1,.16,1.2,1.1),(0,.52,.1,1.3,.16,1.1),(0,0,.57,1.6,1.2,.16),(0,0,-.15,1.1,.8,.12)]
 for x in [-.7,.7]:
  for z in [-.3,0,.3]:boxes.append((x,-.62,z,.055,.06,.055))
 for k in range(15):boxes.append((-.55+k*.08,-.615,.4,.035,.025,.13))
 path=Path('models/factory_detail/meshes/cell.dae');path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(dae_boxes(boxes),encoding='utf-8')
 print('Generated',len(boxes),'closed boxes,',len(boxes)*12,'triangles:',path)
if __name__=='__main__':build()
