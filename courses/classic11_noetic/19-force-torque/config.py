"""Validate the actual offset-load fixture, including mass and inertia context."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET

LINK = "./world/model[@name='weighing_rig']/link[@name='load_link']"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def official_folder():
    return Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(canonical(child) for child in element))


def numbers(element):
    return [float(value) for value in element.text.split()]


def read_world(path):
    root = ET.parse(str(path)).getroot()
    link = root.find(LINK)
    if link is None:
        raise ValueError("Missing fixed load_link")
    mass = float(link.findtext("inertial/mass", "nan"))
    pose = numbers(link.find("inertial/pose"))
    if not math.isfinite(mass) or not .5 <= mass <= 3:
        raise ValueError("Load mass must bewithin[.5,3]kg")
    if len(pose) != 6 or not .15 <= pose[0] <= .6 or pose[1:] != [0]*5:
        raise ValueError("Inertial CoM pose must be arm 0 0 0 0 0 with armwithin[.15,.6]m")
    arm = pose[0]
    expected = {"inertial/mass": [mass], "inertial/pose": [arm, 0, 0, 0, 0, 0],
                "collision[@name='payload_collision']/pose": [arm, 0, 0, 0, 0, 0],
                "visual[@name='payload_visual']/pose": [arm, 0, 0, 0, 0, 0],
                "visual[@name='lever_visual']/pose": [arm/2, 0, 0, 0, 0, 0],
                "visual[@name='lever_visual']/geometry/box/size": [arm, .035, .035]}
    for axis in ("ixx", "iyy", "izz"):
        expected["inertial/inertia/"+axis] = [mass*.2**2/6]
    for field, wanted in expected.items():
        values = numbers(link.find(field))
        if len(values) != len(wanted) or any(not math.isfinite(a) or not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12) for a, b in zip(values, wanted)):
            raise ValueError("Mass/CoM/inertia/geometry context mismatch at "+field+"; use lab.py config")
    official = ET.parse(str(official_folder()/"world.sdf")).getroot()
    clone = copy.deepcopy(root)
    for field in expected:
        clone.find(LINK+"/"+field).text = official.findtext(LINK+"/"+field)
    if canonical(clone) != canonical(official):
        raise ValueError("Fixed wrench rig mismatch: restore feedback/frame/direction/noise/gravity/joint/rate and fixed geometry")
    return {"mass_kg": mass, "arm_m": arm, "gravity_m_s2": 9.81, "rate_hz": 50,
            "anchor_world_m": [0, 0, 1.2]}


def prepared():
    world = Path.cwd()/"world.sdf"
    config = read_world(world)
    record = json.loads((Path.cwd()/"build/wrench_lab.json").read_text(encoding="utf-8"))
    if digest(world) != record.get("world_sha256") or config != record.get("config"):
        raise ValueError("Saved wrench fixture differs from prepared input; reload before observation or verification")
    return config, record


def edit(mass=None, arm=None):
    path = Path.cwd()/"world.sdf"
    read_world(path)
    text = path.read_bytes().decode("utf-8")
    def masked(value):
        return re.sub(r"<!--.*?-->", lambda match: " "*len(match.group()), value, flags=re.S)
    def change(scope_tag, name, tag, value, index=0, count=1):
        nonlocal text
        if name is None:
            pattern = r"<"+scope_tag+r"\b[^>]*>.*?</"+scope_tag+r"\s*>"
        else:
            pattern = r"<"+scope_tag+r"\b(?=[^>]*\bname\s*=\s*['\"]"+name+r"['\"])[^>]*>.*?</"+scope_tag+r"\s*>"
        scopes = list(re.finditer(pattern, masked(text), re.S))
        if len(scopes) != 1:
            raise ValueError("Expected one actual scalar-edit scope: "+scope_tag+"/"+str(name))
        start, end = scopes[0].span()
        block = text[start:end]
        fields = list(re.finditer(r"<"+tag+r"\b[^>]*>(.*?)</"+tag+r"\s*>", masked(block), re.S))
        if len(fields) != 1:
            raise ValueError("Expected one actual numeric field: "+tag)
        field = fields[0]
        values = list(re.finditer(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", field.group(1)))
        if len(values) != count:
            raise ValueError("Unexpected numeric layout: "+tag)
        number = values[index]
        left, right = field.start(1)+number.start(), field.start(1)+number.end()
        block = block[:left]+format(value, ".17g")+block[right:]
        text = text[:start]+block+text[end:]
    if mass is not None:
        if not math.isfinite(mass) or not .5 <= mass <= 3:
            raise ValueError("mass must bewithin[.5,3]kg")
        change("inertial", None, "mass", mass)
        for axis in ("ixx", "iyy", "izz"):
            change("inertial", None, axis, mass*.2**2/6)
    if arm is not None:
        if not math.isfinite(arm) or not .15 <= arm <= .6:
            raise ValueError("arm must bewithin[.15,.6]m")
        change("inertial", None, "pose", arm, count=6)
        change("collision", "payload_collision", "pose", arm, count=6)
        change("visual", "payload_visual", "pose", arm, count=6)
        change("visual", "lever_visual", "pose", arm/2, count=6)
        change("visual", "lever_visual", "size", arm, count=3)
    temporary = path.with_name(".wrench-edit-"+uuid.uuid4().hex+".sdf")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        read_world(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_world(path)
