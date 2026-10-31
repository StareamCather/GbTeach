#!/usr/bin/env python3
"""Small explicit input surface: camera Hz and physics step, applied on reload."""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET


def official_folder():
    return Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(element):
    return element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(), tuple(canonical(child) for child in element)


def read_world(path=None, benchmark=False):
    path = Path(path or Path.cwd()/"world.sdf")
    root = ET.parse(str(path)).getroot()
    base = ET.parse(str(official_folder()/"world.sdf")).getroot()
    fields = ["world/physics/max_step_size", "world/physics/real_time_update_rate",
              "world/model[@name='sync_rig']/link/sensor[@name='camera']/update_rate"]
    values = [float(root.find(field).text) for field in fields]
    step, rate, camera = values
    if step not in (.001, .002) or camera not in (10, 20):
        raise ValueError("Only physics step .001/.002 and camera rate10/20 are supported")
    if not math.isclose(rate, 0 if benchmark else 1/step, abs_tol=1e-9):
        raise ValueError("real_time_update_rate must preserve main RTF cap1, or be0 for the explicit child benchmark")
    for field in fields:
        root.find(field).text = base.find(field).text
    if canonical(root) != canonical(base):
        raise ValueError("Fixed synchronization rig differs: frame/pose/noise/IMU/camera format/metadata plugin/physics geometry must match template")
    return {"camera_rate_hz": camera, "imu_rate_hz": 100, "physics_step_s": step,
            "real_time_update_rate_hz": rate, "image_width": 128, "image_height": 96,
            "main_lockstep": True}


def prepared():
    config = read_world()
    record = json.loads((Path.cwd()/"build/sync_lab.json").read_text(encoding="utf-8"))
    if digest(Path.cwd()/"world.sdf") != record["world_sha256"] or config != record["config"]:
        raise RuntimeError("Saved configuration differs from the loaded world: reload/restart the lesson")
    for name, expected in record["source_sha256"].items():
        if digest(official_folder()/name) != expected:
            raise RuntimeError("Native observer source changed; reload/restart to rebuild")
    for key in ("observer", "metadata_library"):
        if digest(record[key]) != record[key+"_sha256"]:
            raise RuntimeError("Native artifact changed; reload/restart to rebuild")
    return config, record


def replace_scalar(text, tag, value):
    masked = re.sub(r"<!--[\s\S]*?-->", lambda match: " "*len(match.group()), text)
    matches = list(re.finditer(r"<"+tag+r">\s*([-+0-9.eE]+)\s*</"+tag+r">", masked))
    if len(matches) != 1:
        raise ValueError("Expected one explicit scalar: "+tag)
    start, end = matches[0].span(1)
    return text[:start]+format(value, ".17g")+text[end:]


def edit(camera_hz=None, step_ms=None):
    path = Path.cwd()/"world.sdf"
    current = read_world(path)
    text = path.read_bytes().decode("utf-8")
    if camera_hz is not None:
        # This is the only <update_rate> belonging to the camera sensor.
        masked = re.sub(r"<!--[\s\S]*?-->", lambda match: " "*len(match.group()), text)
        match = re.search(r'<sensor\s+name="camera"[\s\S]*?</sensor>', masked)
        if not match:
            raise ValueError("Unique camera sensor block is missing")
        part = replace_scalar(text[match.start():match.end()], "update_rate", camera_hz)
        text = text[:match.start()]+part+text[match.end():]
    if step_ms is not None:
        step = step_ms/1000
        text = replace_scalar(text, "max_step_size", step)
        text = replace_scalar(text, "real_time_update_rate", 1/step)
    temporary = path.with_name("world-"+uuid.uuid4().hex+".tmp")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        result = read_world(temporary)
        os.replace(str(temporary), str(path))
        return {"before": current, "after": result, "reload_required": True}
    finally:
        if temporary.exists():
            temporary.unlink()
