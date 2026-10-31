"""Read the real personal SDF; retain files and constrain the teaching rig."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
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
    puck = root.find("./world/model[@name='puck']")
    if puck is None:
        raise ValueError("Missing fixed puck model")
    mass = float(puck.findtext("link/inertial/mass", "nan"))
    pose = [float(x) for x in puck.findtext("pose", "").split()]
    if not math.isfinite(mass) or not 0.25 <= mass <= 4:
        raise ValueError("puck mass must be within [0.25,4] kg")
    if len(pose) != 6 or any(not math.isfinite(x) for x in pose) or pose[:2] != [0, 0] or pose[3:] != [0, 0, 0] or not 0.6 <= pose[2] <= 3:
        raise ValueError("puck pose must be 0 0 z 0 0 0 with z within [0.6,3] m")
    expected = {"ixx": mass*(0.35**2+0.2**2)/12,
                "iyy": mass*(0.35**2+0.2**2)/12,
                "izz": mass*(0.35**2+0.35**2)/12}
    for name, value in expected.items():
        actual = float(puck.findtext("link/inertial/inertia/"+name, "nan"))
        if not math.isfinite(actual) or not math.isclose(actual, value, rel_tol=1e-9, abs_tol=1e-12):
            raise ValueError("Uniform box inertia mismatch; use lab.py config --mass")
    official = ET.parse(str(official_folder()/"world.sdf")).getroot()
    clone = copy.deepcopy(root)
    clone_puck = clone.find("./world/model[@name='puck']")
    official_puck = official.find("./world/model[@name='puck']")
    for field in ["pose", "link/inertial/mass", "link/inertial/inertia/ixx", "link/inertial/inertia/iyy", "link/inertial/inertia/izz"]:
        clone_puck.find(field).text = official_puck.findtext(field)
    if canonical(clone) != canonical(official):
        raise ValueError("Fixed rig mismatch: restore collision/filter/frame/rate and other fixed fields; only mass/inertia and release Z are editable")
    return {"mass_kg": mass, "release_height_m": pose[2], "gravity_m_s2": 9.81,
            "plate_top_m": 0.2, "puck_half_height_m": 0.1, "sensor_rate_hz": 50}


def prepared(workspace=None):
    workspace = Path(workspace or Path.cwd())
    config = read_world(workspace/"world.sdf")
    record = json.loads((workspace/"build/contact_lab.json").read_text(encoding="utf-8"))
    if digest(workspace/"world.sdf") != record.get("world_sha256") or config != record.get("config"):
        raise ValueError("Saved world.sdf differs from prepared input; reload this lesson before controls or verification")
    return config, record


def edit(mass=None, height=None):
    path = Path.cwd()/"world.sdf"
    read_world(path)
    text = path.read_bytes().decode("utf-8")
    # Mask comments with equal-length blanks so commented examples do not match.
    def masked(value):
        return re.sub(r"<!--.*?-->", lambda item: " "*len(item.group()), value, flags=re.S)
    blocks = list(re.finditer(r"<model\b(?=[^>]*\bname\s*=\s*['\"]puck['\"])[^>]*>.*?</model\s*>", masked(text), re.S))
    if len(blocks) != 1:
        raise ValueError("Expected exactly one actual puck model for scalar edit")
    start, end = blocks[0].span()
    block = text[start:end]

    def scalar(name, value, pose=False):
        nonlocal block
        pattern = r"<"+name+r"\b[^>]*>(.*?)</"+name+r"\s*>"
        matches = list(re.finditer(pattern, masked(block), re.S))
        if len(matches) != 1:
            raise ValueError("Expected one actual puck field: "+name)
        match = matches[0]
        inner_start, inner_end = match.span(1)
        inner = block[inner_start:inner_end]
        numbers = list(re.finditer(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", masked(inner)))
        if len(numbers) != (6 if pose else 1):
            raise ValueError("Unexpected numeric field layout: "+name)
        number = numbers[2 if pose else 0]
        left, right = inner_start+number.start(), inner_start+number.end()
        block = block[:left]+format(value, ".17g")+block[right:]

    if mass is not None:
        if not math.isfinite(mass) or not 0.25 <= mass <= 4:
            raise ValueError("mass must be within [0.25,4] kg")
        scalar("mass", mass)
        for name, value in {"ixx": mass*(0.35**2+0.2**2)/12, "iyy": mass*(0.35**2+0.2**2)/12,
                            "izz": mass*(0.35**2+0.35**2)/12}.items():
            scalar(name, value)
    if height is not None:
        if not math.isfinite(height) or not 0.6 <= height <= 3:
            raise ValueError("release height must be within [0.6,3] m")
        scalar("pose", height, pose=True)
    temporary = path.with_name(".world-contact-edit.sdf")
    try:
        temporary.write_bytes((text[:start]+block+text[end:]).encode("utf-8"))
        read_world(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_world(path)
