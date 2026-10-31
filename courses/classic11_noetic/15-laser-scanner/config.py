"""Keep the personal scene; expose only wall center and matched beam counts."""
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
    wall = root.find("./world/model[@name='target_wall']/pose")
    if wall is None:
        raise ValueError("Missing fixed target_wall pose")
    pose = [float(x) for x in wall.text.split()]
    if len(pose) != 6 or any(not math.isfinite(x) for x in pose) or pose[1:] != [0, 1, 0, 0, 0] or not 2.5 <= pose[0] <= 4:
        raise ValueError("target_wall pose must be distance 0 1 0 0 0, distance within[2.5,4]")
    sensors = root.findall("./world/model[@name='scanner']/link/sensor")
    samples = [int(sensor.findtext("ray/scan/horizontal/samples", "0")) for sensor in sensors]
    if len(samples) != 2 or samples[0] not in (181, 361) or samples[1] != samples[0]:
        raise ValueError("CPU/GPU samples must both be181 or both361")
    official = ET.parse(str(official_folder()/"world.sdf")).getroot()
    clone = copy.deepcopy(root)
    clone.find("./world/model[@name='target_wall']/pose").text = official.findtext("./world/model[@name='target_wall']/pose")
    for sensor, template in zip(clone.findall("./world/model[@name='scanner']/link/sensor"), official.findall("./world/model[@name='scanner']/link/sensor")):
        sensor.find("ray/scan/horizontal/samples").text = template.findtext("ray/scan/horizontal/samples")
    if canonical(clone) != canonical(official):
        raise ValueError("Fixed laser rig mismatch: restore collision/visual/sensor/frame/range/rate/noise; only distance and paired samples are editable")
    return {"wall_distance_m": pose[0], "beam_count": samples[0], "rate_hz": 20,
            "sensor_origin_m": [0, 0, 0.7], "range_min_m": 0.1, "range_max_m": 10}


def prepared():
    config = read_world(Path.cwd()/"world.sdf")
    record = json.loads((Path.cwd()/"build/laser_lab.json").read_text(encoding="utf-8"))
    if digest(Path.cwd()/"world.sdf") != record.get("world_sha256") or config != record.get("config"):
        raise ValueError("Saved world.sdf differs from prepared input; reload before controls or verification")
    return config, record


def edit(distance=None, samples=None):
    path = Path.cwd()/"world.sdf"
    read_world(path)
    text = path.read_bytes().decode("utf-8")
    def masked(value):
        return re.sub(r"<!--.*?-->", lambda match: " "*len(match.group()), value, flags=re.S)
    def update(model, tag, value, count=1, number_index=0, number_count=1):
        nonlocal text
        blocks = list(re.finditer(r"<model\b(?=[^>]*\bname\s*=\s*['\"]"+model+r"['\"])[^>]*>.*?</model\s*>", masked(text), re.S))
        if len(blocks) != 1:
            raise ValueError("Expected one actual model: "+model)
        begin, end = blocks[0].span()
        block = text[begin:end]
        fields = list(re.finditer(r"<"+tag+r"\b[^>]*>(.*?)</"+tag+r"\s*>", masked(block), re.S))
        if len(fields) != count:
            raise ValueError("Unexpected field count: "+tag)
        replacements = []
        for field in fields:
            numbers = list(re.finditer(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", field.group(1)))
            if len(numbers) != number_count:
                raise ValueError("Unexpected numeric field layout: "+tag)
            number = numbers[number_index]
            replacements.append((field.start(1)+number.start(), field.start(1)+number.end()))
        for left, right in reversed(replacements):
            block = block[:left]+format(value, ".17g")+block[right:]
        text = text[:begin]+block+text[end:]
    if distance is not None:
        if not math.isfinite(distance) or not 2.5 <= distance <= 4:
            raise ValueError("distance must be within[2.5,4]m")
        update("target_wall", "pose", distance, number_count=6)
    if samples is not None:
        if samples not in (181, 361):
            raise ValueError("samples must be181 or361")
        update("scanner", "samples", samples, count=2)
    temporary = path.with_name(".laser-edit-"+uuid.uuid4().hex+".sdf")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        read_world(temporary)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()
    return read_world(path)
