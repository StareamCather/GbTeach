"""Validate the real five-sensor stand and its two single-variable settings."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def official_folder():
    return Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(canonical(child) for child in element))


def read_world(path):
    root = ET.parse(str(path)).getroot()
    field = [float(value) for value in root.findtext("world/magnetic_field", "").split()]
    pose = [float(value) for value in root.findtext("world/model[@name='nav_rig']/pose", "").split()]
    scale = next((scale for scale in (1, 2) if len(field) == 3 and all(math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-14) for a, b in zip(field, [2e-5*scale, 0, -4e-5*scale]))), None)
    if scale is None:
        raise ValueError("Magnetic field must be[20,0,-40]uT times scale1 or2")
    if len(pose) != 6 or pose[:2] != [0, 0] or pose[2] not in (1, 2) or pose[3:] != [0, 0, 0]:
        raise ValueError("Initial nav pose must be0 0 height 0 0 0 with height1 or2m")
    official = ET.parse(str(official_folder()/"world.sdf")).getroot()
    clone = copy.deepcopy(root)
    for path in ("world/magnetic_field", "world/model[@name='nav_rig']/pose"):
        clone.find(path).text = official.findtext(path)
    if canonical(clone) != canonical(official):
        raise ValueError("Fixed sensor stand mismatch: restore factory/topic/noise/spherical/heading/sonar/logical/collision/geometry/rate")
    return {"magnetic_scale": scale, "magnetic_field_tesla": field, "initial_height_m": pose[2], "rate_hz": 10,
            "spherical_reference": {"latitude_deg": 0, "longitude_deg": 0, "elevation_m": 100, "heading_deg": 180},
            "sonar_min_m": 0, "sonar_max_m": 3, "sonar_radius_m": .2}


def prepared():
    world = Path.cwd()/"world.sdf"
    config = read_world(world)
    record = json.loads((Path.cwd()/"build/stand_lab.json").read_text(encoding="utf-8"))
    if digest(world) != record.get("world_sha256") or config != record.get("config"):
        raise ValueError("Saved stand input differs from prepared files; reload before controls or verification")
    return config, record


def edit(field_scale=None, initial_height=None):
    path = Path.cwd()/"world.sdf"
    read_world(path)
    text = path.read_bytes().decode("utf-8")
    def masked(value):
        return re.sub(r"<!--.*?-->", lambda match: " "*len(match.group()), value, flags=re.S)
    if field_scale is not None:
        if field_scale not in (1, 2):
            raise ValueError("field-scale must be1 or2")
        fields = list(re.finditer(r"<magnetic_field\b[^>]*>(.*?)</magnetic_field\s*>", masked(text), re.S))
        if len(fields) != 1:
            raise ValueError("Expected one magnetic_field element")
        left, right = fields[0].span(1)
        inner = text[left:right]
        values = list(re.finditer(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", masked(inner)))
        if len(values) != 3:
            raise ValueError("Expected three magnetic-field components")
        for index, value in ((2, -4e-5*field_scale), (0, 2e-5*field_scale)):
            begin, end = values[index].span()
            inner = inner[:begin]+format(value, ".17g")+inner[end:]
        text = text[:left]+inner+text[right:]
    if initial_height is not None:
        if initial_height not in (1, 2):
            raise ValueError("initial-height must be1 or2m")
        models = list(re.finditer(r"<model\b(?=[^>]*\bname\s*=\s*['\"]nav_rig['\"])[^>]*>.*?</model\s*>", masked(text), re.S))
        if len(models) != 1:
            raise ValueError("Expected one nav_rig model")
        begin, end = models[0].span()
        block = text[begin:end]
        pose = re.search(r"<pose\b[^>]*>(.*?)</pose\s*>", masked(block), re.S)
        values = list(re.finditer(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", pose.group(1)))
        if len(values) != 6:
            raise ValueError("Unexpected initial nav model pose")
        number = values[2]
        left, right = pose.start(1)+number.start(), pose.start(1)+number.end()
        block = block[:left]+str(initial_height)+block[right:]
        text = text[:begin]+block+text[end:]
    temporary = path.with_name(".stand-edit-"+uuid.uuid4().hex+".sdf")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        read_world(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_world(path)
