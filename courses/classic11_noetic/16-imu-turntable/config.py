"""Fixed real IMU fixtures and a small, checked experiment parameter file."""
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
    if canonical(ET.parse(str(path)).getroot()) != canonical(ET.parse(str(official_folder()/"world.sdf")).getroot()):
        raise ValueError("Fixed IMU rig mismatch: restore gravity/inertia/noise/frame/offset/rate/reference and fixed geometry")


def read_parameters(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("Duplicate parameter key: "+key)
            result[key] = value
        return result
    value = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=pairs)
    if not isinstance(value, dict) or set(value) != {"speed_rad_s", "tilt_deg"}:
        raise ValueError("Expected only speed_rad_s and tilt_deg parameters")
    for key, lower, upper in (("speed_rad_s", 0.2, 1.5), ("tilt_deg", 10, 60)):
        number = value[key]
        if type(number) not in (int, float) or not math.isfinite(number) or not lower <= number <= upper:
            raise ValueError(key+" must be finite within "+str([lower, upper]))
    return value


def prepared():
    workspace = Path.cwd()
    read_world(workspace/"world.sdf")
    parameters = read_parameters(workspace/"parameters.json")
    record = json.loads((workspace/"build/imu_lab.json").read_text(encoding="utf-8"))
    if digest(workspace/"world.sdf") != record.get("world_sha256") or digest(workspace/"parameters.json") != record.get("parameters_sha256") or parameters != record.get("parameters"):
        raise ValueError("Saved input differs from prepared files; reload before controls or verification")
    return parameters, record


def edit(speed=None, tilt=None):
    path = Path.cwd()/"parameters.json"
    read_parameters(path)
    text = path.read_bytes().decode("utf-8")
    for key, value in (("speed_rad_s", speed), ("tilt_deg", tilt)):
        if value is None:
            continue
        if not math.isfinite(value):
            raise ValueError("Parameter must be finite")
        pattern = r'("'+key+r'"\s*:\s*)([-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?)'
        text, count = re.subn(pattern, lambda match: match.group(1)+format(value, ".17g"), text)
        if count != 1:
            raise ValueError("Expected one numeric field: "+key)
    temporary = path.with_name(".imu-edit-"+uuid.uuid4().hex+".json")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        read_parameters(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_parameters(path)
