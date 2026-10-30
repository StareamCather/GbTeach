"""Generate and inspect the lesson's offline annular-cylinder display mesh."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

URI = "model://inertia_ring/meshes/ring.dae"


def write_ring(workspace, outer, inner, length):
    if not 0 < inner < outer or length <= 0:
        raise ValueError("环形网格需要0<内半径<外半径、厚度>0")
    count = 64
    positions = []
    for radius, z in ((inner, -length/2), (outer, -length/2), (inner, length/2), (outer, length/2)):
        for index in range(count):
            angle = 2*math.pi*index/count
            positions.extend((radius*math.cos(angle), radius*math.sin(angle), z))
    triangles = []
    for index in range(count):
        following = (index+1) % count
        ib, ob, it, ot = index, count+index, 2*count+index, 3*count+index
        jb, pb, jt, pt = following, count+following, 2*count+following, 3*count+following
        triangles.extend((it, ot, pt, it, pt, jt, ib, pb, ob, ib, jb, pb,
                          ob, pb, pt, ob, pt, ot, ib, it, jt, ib, jt, jb))
    root = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "created").text = "2026-10-30T00:00:00Z"
    ET.SubElement(asset, "modified").text = "2026-10-30T00:00:00Z"
    ET.SubElement(asset, "unit", name="meter", meter="1")
    ET.SubElement(asset, "up_axis").text = "Z_UP"
    effects = ET.SubElement(root, "library_effects")
    effect = ET.SubElement(effects, "effect", id="ring-effect")
    technique = ET.SubElement(ET.SubElement(effect, "profile_COMMON"), "technique", sid="common")
    diffuse = ET.SubElement(ET.SubElement(technique, "lambert"), "diffuse")
    ET.SubElement(diffuse, "color").text = ".9 .45 .08 1"
    materials = ET.SubElement(root, "library_materials")
    ET.SubElement(ET.SubElement(materials, "material", id="ring-material", name="ring-material"), "instance_effect", url="#ring-effect")
    geometries = ET.SubElement(root, "library_geometries")
    mesh = ET.SubElement(ET.SubElement(geometries, "geometry", id="ring-geometry", name="ring"), "mesh")
    source = ET.SubElement(mesh, "source", id="ring-positions")
    ET.SubElement(source, "float_array", id="ring-positions-array", count=str(len(positions))).text = " ".join("%.12g" % value for value in positions)
    accessor = ET.SubElement(ET.SubElement(source, "technique_common"), "accessor", source="#ring-positions-array", count=str(4*count), stride="3")
    for axis in ("X", "Y", "Z"):
        ET.SubElement(accessor, "param", name=axis, type="float")
    vertices = ET.SubElement(mesh, "vertices", id="ring-vertices")
    ET.SubElement(vertices, "input", semantic="POSITION", source="#ring-positions")
    node = ET.SubElement(mesh, "triangles", count=str(len(triangles)//3), material="ring-material")
    ET.SubElement(node, "input", semantic="VERTEX", source="#ring-vertices", offset="0")
    ET.SubElement(node, "p").text = " ".join(str(value) for value in triangles)
    scenes = ET.SubElement(root, "library_visual_scenes")
    scene = ET.SubElement(scenes, "visual_scene", id="Scene", name="Scene")
    item = ET.SubElement(scene, "node", id="ring", name="ring", type="NODE")
    instance = ET.SubElement(item, "instance_geometry", url="#ring-geometry")
    binding = ET.SubElement(ET.SubElement(instance, "bind_material"), "technique_common")
    ET.SubElement(binding, "instance_material", symbol="ring-material", target="#ring-material")
    ET.SubElement(ET.SubElement(root, "scene"), "instance_visual_scene", url="#Scene")
    target = Path(workspace) / "models/inertia_ring/meshes/ring.dae"
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(str(target), encoding="utf-8", xml_declaration=True)
    return target


def check_ring(workspace, outer, inner, length):
    target = Path(workspace) / "models/inertia_ring/meshes/ring.dae"
    root = ET.parse(str(target)).getroot()
    array = root.find(".//{*}float_array[@id='ring-positions-array']")
    if array is None:
        raise ValueError("本地环形网格缺少生成的顶点；运行lab.py rebuild-ring")
    values = [float(value) for value in (array.text or "").split()]
    if len(values) < 24 or len(values) % 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("本地环形网格顶点无效")
    radii = [math.hypot(values[index], values[index+1]) for index in range(0, len(values), 3)]
    zs = values[2::3]
    if max(abs(min(radii)-inner), abs(max(radii)-outer), abs(max(zs)-length/2), abs(min(zs)+length/2)) > 1e-7:
        raise ValueError("环形网格与当前内/外半径或厚度不一致；更新惯量后运行lab.py rebuild-ring")
