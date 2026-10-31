"""Validate the real personal camera SDF; only resolution and HFOV are open."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET

CAMERA = "./world/model[@name='camera_rig']/link/sensor/camera"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def official_folder():
    return Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(),
            tuple(canonical(child) for child in element))


def read_world(path):
    root = ET.parse(str(path)).getroot()
    camera = root.find(CAMERA)
    if camera is None:
        raise ValueError("Missing camera_rig fixed camera")
    width, height = int(camera.findtext("image/width", "0")), int(camera.findtext("image/height", "0"))
    hfov = float(camera.findtext("horizontal_fov", "nan"))
    if width not in (320, 640) or height != width*3//4:
        raise ValueError("Resolution must be320x240 or640x480")
    degrees = next((value for value in (60, 75) if math.isclose(hfov, math.radians(value), abs_tol=1e-10)), None)
    if degrees is None:
        raise ValueError("HFOV must be60 or75 degrees")
    official = ET.parse(str(official_folder()/"world.sdf")).getroot()
    clone = copy.deepcopy(root)
    for field in ("image/width", "image/height", "horizontal_fov"):
        clone.find(CAMERA+"/"+field).text = official.findtext(CAMERA+"/"+field)
    if canonical(clone) != canonical(official):
        raise ValueError("Fixed RGB rig mismatch: restore target geometry, format/encoding, frame, noise, distortion and rate")
    return {"width": width, "height": height, "hfov_deg": degrees, "rate_hz": 20,
            "target_size_m": [.04, .6, .6], "camera_position_m": [0, 0, 1]}


def prepared():
    path = Path.cwd()/"world.sdf"
    config = read_world(path)
    record = json.loads((Path.cwd()/"build/camera_lab.json").read_text(encoding="utf-8"))
    if digest(path) != record.get("world_sha256") or config != record.get("config"):
        raise ValueError("Saved camera input differs from prepared files; reload before controls or verification")
    return config, record


def edit(width=None, fov=None):
    path = Path.cwd()/"world.sdf"
    read_world(path)
    text = path.read_bytes().decode("utf-8")
    def masked(value):
        return re.sub(r"<!--.*?-->", lambda item: " "*len(item.group()), value, flags=re.S)
    changes = {}
    if width is not None:
        if width not in (320, 640):
            raise ValueError("width must be320 or640")
        changes.update(width=width, height=width*3//4)
    if fov is not None:
        if fov not in (60, 75):
            raise ValueError("fov must be60 or75 degrees")
        changes["horizontal_fov"] = math.radians(fov)
    for tag, value in changes.items():
        matches = list(re.finditer(r"<"+tag+r"\b[^>]*>(\s*)([-+0-9.eE]+)(\s*)</"+tag+r"\s*>", masked(text)))
        if len(matches) != 1:
            raise ValueError("Expected one actual numeric camera field: "+tag)
        left, right = matches[0].span(2)
        text = text[:left]+format(value, ".17g")+text[right:]
    temporary = path.with_name(".camera-edit-"+uuid.uuid4().hex+".sdf")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        read_world(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_world(path)
