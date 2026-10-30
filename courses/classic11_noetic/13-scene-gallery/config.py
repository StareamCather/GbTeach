"""Validate the personal SDF, numeric PNG and offline resources before control."""
import hashlib
import math
import os
from pathlib import Path
import re
import struct
import zlib
import xml.etree.ElementTree as ET

PNG_URI = "file://assets/heightmap.png"
WALK = Path("/usr/share/gazebo-11/media/models/walk.dae")
WOOD_SCRIPT = Path("/usr/share/gazebo-11/media/materials/scripts/gazebo.material")
WOOD_TEXTURE = Path("/usr/share/gazebo-11/media/materials/textures/wood.jpg")
TERRAIN_DIFFUSE = Path("/usr/share/gazebo-11/media/materials/textures/dirt_diffusespecular.png")
TERRAIN_NORMAL = Path("/usr/share/gazebo-11/media/materials/textures/flat_normal.png")
WALK_SHA = "49af0df3a319d1cb8ca2cebf02dbd00f625e5d5bec820bc5e109925b18b65c6e"
FRACTIONS = (0., 5/12, 1/2, 11/12, 1.)
PROBES = (("probe_nw", -1., 1., 85), ("probe_ne", 1., 1., 170),
          ("probe_sw", -1., -1., 0), ("probe_se", 1., -1., 255),
          ("probe_matte", 3.4, -.9, None), ("probe_emissive", 3.4, .9, None))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def numbers(text, length=None):
    try:
        values = [float(x) for x in text.split()]
    except (ValueError, AttributeError):
        raise ValueError("SDF field must contain finite numeric values")
    if (length is not None and len(values) != length) or not all(math.isfinite(x) for x in values):
        raise ValueError("SDF field has non-finite numbers or wrong vector length")
    return values


def read_png(path):
    b = Path(path).read_bytes()
    if not b.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("heightmap PNG signature is invalid")
    offset, chunks = 8, []
    while offset < len(b):
        if offset+12 > len(b):
            raise ValueError("heightmap PNG chunk is truncated")
        size = struct.unpack(">I", b[offset:offset+4])[0]
        kind = b[offset+4:offset+8]
        end = offset+12+size
        if end > len(b):
            raise ValueError("heightmap PNG chunk is truncated")
        data = b[offset+8:offset+8+size]
        crc = struct.unpack(">I", b[offset+8+size:end])[0]
        if zlib.crc32(kind+data) & 0xffffffff != crc:
            raise ValueError("heightmap PNG CRC mismatch")
        chunks.append((kind, data))
        offset = end
    if [x[0] for x in chunks] != [b"IHDR", b"IDAT", b"IEND"] or chunks[-1][1]:
        raise ValueError("heightmap PNG must have only IHDR/IDAT/IEND, no palette/gamma")
    if struct.unpack(">IIBBBBB", chunks[0][1]) != (65, 65, 8, 0, 0, 0, 0):
        raise ValueError("heightmap PNG must be 65x65, 8-bit grayscale, non-interlaced")
    raw = zlib.decompress(chunks[1][1])
    expected = bytes(x for v in range(65) for x in ([0]+[85 if u < 32 and v < 32 else 170 if v < 32 else 0 if u < 32 else 255 for u in range(65)]))
    if raw != expected:
        raise ValueError("heightmap PNG numeric fixture/filter0 changed: expected gray 85/170/0/255 quadrants")
    return {"width": 65, "height": 65, "bit_depth": 8, "color_type": 0,
            "gray_levels": [0, 85, 170, 255], "sha256": hashlib.sha256(b).hexdigest()}


def _parameters(root):
    hm = root.findall(".//heightmap")
    if len(hm) != 2:
        raise ValueError("world requires exactly one collision and one visual heightmap")
    values = []
    for h in hm:
        if h.findtext("uri") != PNG_URI or numbers(h.findtext("pos"), 3) != [0., 0., 0.]:
            raise ValueError("heightmap URI/pos must match the fixed local numeric fixture")
        size = numbers(h.findtext("size"), 3)
        if size[:2] != [4., 4.] or not .3 <= size[2] <= 1.:
            raise ValueError("height_m must be finite within [.3,1.0], XY remains 4x4")
        if h.findtext("sampling") != "1":
            raise ValueError("heightmap sampling must remain 1")
        values.append(size[2])
    if values[0] != values[1]:
        raise ValueError("visual/collision heightmap size must agree")
    actors = root.findall("./world/actor")
    if {a.get("name") for a in actors} != {"walker", "marker"}:
        raise ValueError("world requires exactly the fixed walker and marker Actors")
    cycles = []
    for a in actors:
        s = a.find("script")
        if s is None or [s.findtext(x) for x in ("loop", "auto_start", "delay_start")] != ["true", "true", "0"]:
            raise ValueError("Actor unsafe script flags: loop/auto_start true, delay_start 0 are mandatory")
        ts = s.findall("trajectory")
        if len(ts) != 1 or ts[0].get("tension") != "1":
            raise ValueError("Actor requires one trajectory with tension=1")
        times = [numbers(w.findtext("time"), 1)[0] for w in ts[0].findall("waypoint")]
        if len(times) != 5 or not 4 <= times[-1] <= 12:
            raise ValueError("actor_cycle_s requires five waypoints and cycle in [4,12]")
        if any(abs(t-times[-1]*f) > 1e-10 for t, f in zip(times, FRACTIONS)):
            raise ValueError("Actor waypoint times must share fixed cycle fractions")
        cycles.append(times[-1])
    if cycles[0] != cycles[1]:
        raise ValueError("walker/marker actor_cycle_s must agree")
    light = root.find("./world/light[@name='daylight']/diffuse")
    rgba = numbers(light.text if light is not None else None, 4)
    if rgba[0] != rgba[1] or rgba[1] != rgba[2] or rgba[3] != 1 or not .1 <= rgba[0] <= 1:
        raise ValueError("daylight diffuse must be neutral RGB within [.1,1] and alpha1")
    for e in root.findall(".//visual/material/*"):
        if e.tag in ("ambient", "diffuse", "specular", "emissive"):
            if any(not 0 <= v <= 1 for v in numbers(e.text, 4)):
                raise ValueError("material "+e.tag+" RGBA must be within [0,1]")
    for l in root.findall("./world/light"):
        for tag in ("diffuse", "specular"):
            if any(not 0 <= v <= 1 for v in numbers(l.findtext(tag), 4)):
                raise ValueError("light "+tag+" RGBA must be within [0,1]")
        if sum(x*x for x in numbers(l.findtext("direction"), 3)) < 1e-8:
            raise ValueError("light direction must be nonzero")
        if l.findtext("cast_shadows") not in ("true", "false"):
            raise ValueError("light cast_shadows must be true or false")
        for tag in ("constant", "linear", "quadratic", "range"):
            v = numbers(l.findtext("attenuation/"+tag), 1)[0]
            if v < 0 or (tag in ("constant", "linear") and v > 1) or (tag == "range" and v <= 0):
                raise ValueError("light attenuation/"+tag+" is out of range")
        if l.get("type") == "spot":
            inner, outer, falloff = [numbers(l.findtext("spot/"+x), 1)[0] for x in ("inner_angle", "outer_angle", "falloff")]
            if not 0 <= inner <= outer <= math.pi or falloff < 0:
                raise ValueError("spot requires 0<=inner_angle<=outer_angle<=pi and nonnegative falloff")
    return {"height_m": values[0], "actor_cycle_s": cycles[0], "daylight": rgba[0]}


def _signature(root):
    masked = set()
    for h in root.findall(".//heightmap"):
        masked.add(id(h.find("size")))
    for a in root.findall("./world/actor"):
        masked.update(id(e) for e in a.findall("./script/trajectory/waypoint/time"))
    for e in root.findall(".//visual/material/*"):
        if e.tag in ("ambient", "diffuse", "specular", "emissive"):
            masked.add(id(e))
    for l in root.findall("./world/light"):
        masked.update(id(e) for e in list(l) if e.tag in ("diffuse", "specular", "direction", "cast_shadows"))
        masked.update(id(e) for e in l.findall("./attenuation/*")+l.findall("./spot/*"))
    def visit(e):
        return (e.tag, tuple(sorted(e.attrib.items())), "<editable>" if id(e) in masked else " ".join((e.text or "").split()), tuple(visit(c) for c in e))
    return visit(root)


def read_world(path, template):
    root = ET.fromstring(Path(path).read_bytes())
    p = _parameters(root)
    if _signature(root) != _signature(ET.fromstring(Path(template).read_bytes())):
        raise ValueError("world fixed topology/geometry/URI/Actor paths changed; restore template or adapt a separate project")
    return p


def resources(workspace):
    png = read_png(Path(workspace)/"assets/heightmap.png")
    for p in (WALK, WOOD_SCRIPT, WOOD_TEXTURE, TERRAIN_DIFFUSE, TERRAIN_NORMAL):
        if not p.is_file():
            raise ValueError("Missing offline resource: "+str(p))
    if digest(WALK) != WALK_SHA or "material Gazebo/Wood" not in WOOD_SCRIPT.read_text():
        raise ValueError("Installed walk/Wood resource differs from the validated Classic11 fixture")
    texture_assets = {}
    for name, path, expected, purpose in (
            ("terrain_diffuse", TERRAIN_DIFFUSE, "bd48f9d847a375a56c3f0637bd35ff08f468ae6d5527503e24b995dde660c54a", "heightmap visual diffuse/specular layer"),
            ("terrain_normal", TERRAIN_NORMAL, "5badb5b8def8bdb9c037932ff4c621a3c14374daa2ef175a35cc2e77a6943714", "flat visual normal map without changing collision height")):
        actual = digest(path)
        if actual != expected:
            raise ValueError("Installed offline terrain texture differs: "+str(path))
        texture_assets[name] = {"path": str(path), "sha256": actual, "package": "gazebo11-common",
                                "license": "Apache-2.0 per package copyright Files:*; system reference only", "purpose": purpose,
                                "upstream": "https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/textures/"+path.name}
    return {"heightmap": png, "walk": {"path": str(WALK), "sha256": digest(WALK),
            "package": "gazebo11-common", "license": "Apache-2.0 (package copyright Files:*)",
            "upstream": "https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/models/walk.dae"},
            "wood_script": {"path": str(WOOD_SCRIPT), "sha256": digest(WOOD_SCRIPT)},
            "wood_texture": {"path": str(WOOD_TEXTURE), "sha256": digest(WOOD_TEXTURE)}, **texture_assets}


def edit_world(path, template, field, value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("edit value must be finite")
    before = Path(path).read_text(encoding="utf-8")
    read_world(path, template)
    if field == "height":
        after, count = re.subn(r"(<heightmap>.*?<size>)[^<]+(</size>)", lambda m: m[1]+"4 4 %.17g" % value+m[2], before, flags=re.S)
        expected_count, parameter = 2, "height_m"
    elif field == "cycle":
        counts = []
        def actor(m):
            i = iter(FRACTIONS)
            result, count = re.subn(r"<time>[^<]+</time>", lambda _: "<time>%.17g</time>" % (value*next(i)), m[0])
            counts.append(count)
            return result
        after, actor_count = re.subn(r"<actor\b.*?</actor>", actor, before, flags=re.S)
        count, expected_count, parameter = sum(counts), 10, "actor_cycle_s"
        if actor_count != 2 or counts != [5, 5]:
            raise ValueError("cycle edit requires exactly two five-waypoint Actor blocks")
    elif field == "daylight":
        pattern = r"(<light\b(?=[^>]*\bname\s*=\s*['\"]daylight['\"])[^>]*>.*?<diffuse>)[^<]+(</diffuse>)"
        after, count = re.subn(pattern, lambda m: m[1]+"%.17g %.17g %.17g 1" % (value, value, value)+m[2], before, flags=re.S)
        expected_count, parameter = 1, "daylight"
    else:
        raise ValueError("unknown editable field")
    root = ET.fromstring(after)
    p = _parameters(root)
    if count != expected_count or abs(p[parameter]-value) > 1e-12:
        raise ValueError("SDF edit did not change exactly the requested field occurrences")
    if _signature(root) != _signature(ET.fromstring(Path(template).read_bytes())):
        raise ValueError("edited world changed fixed topology")
    temporary = Path(path).with_suffix(".edit.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as output:
        output.write(after)
    os.replace(str(temporary), str(path))
    return {"passed": True, "parameters": p, "reload_required": True, "field": field, "value": value, "world_sha256": digest(path)}
