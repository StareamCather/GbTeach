#!/usr/bin/env python3
"""Small Xacro input surface; generated artifacts remain actual tool outputs."""
import hashlib
import json
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET

PACKAGE = "ros_ws/src/gazeboteach_l23"
JOINTS = ("lesson23_shoulder", "lesson23_elbow")
SOURCES = ("config.py", "ros_entry.py", "ros_lab.sh", PACKAGE+"/package.xml", PACKAGE+"/launch/arm.launch",
    PACKAGE+"/urdf/arm.urdf.xacro", PACKAGE+"/urdf/beam_macros.xacro", PACKAGE+"/scripts/common.py", PACKAGE+"/scripts/description_info.py")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_settings(path=None):
    data = json.loads(Path(path or Path.cwd()/"arm_settings.json").read_bytes().decode("utf-8"))
    if set(data) != {"length1_m", "preserve_tool"} or data["length1_m"] not in (.6, .8) or type(data["preserve_tool"]) is not bool:
        raise ValueError("Only length1_m .6/.8 and boolean preserve_tool are supported")
    return {**data, "length2_m": .4, "mass1_kg": 1, "mass2_kg": .5, "tool_mass_kg": .02,
        "base_yaw_rad": .2, "joint2_origin_yaw_rad": .15, "base_height_m": .7, "joint1_height_m": .08,
        "physics_step_s": .001, "model_name": "xacro_arm", "namespace": "/lesson23"}


def prepared(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    config = read_settings(workspace/"arm_settings.json")
    record = json.loads((workspace/"build/arm_lab.json").read_text(encoding="utf-8"))
    if config != record["config"] or digest(workspace/"arm_settings.json") != record["settings_sha256"] or digest(workspace/"world.sdf") != record["world_sha256"]:
        raise RuntimeError("Saved model inputs differ from the loaded world: reload/restart required")
    if any(digest(workspace/name) != record["source_sha256"][name] for name in SOURCES):
        raise RuntimeError("Xacro/source package changed; reload/restart to regenerate and respawn")
    for name, key in (("generated_urdf_path", "generated_urdf_sha256"), ("converted_sdf_path", "converted_sdf_sha256")):
        if digest(record[name]) != record[key]:
            raise RuntimeError("Generated URDF/SDF artifact drifted; reload/restart required")
    return config, record


def overlay(workspace):
    workspace = Path(workspace).resolve()
    environment = os.environ.copy()
    path = str(workspace/"ros_ws/src")
    environment["ROS_PACKAGE_PATH"] = os.pathsep.join([path]+[p for p in environment.get("ROS_PACKAGE_PATH", "").split(os.pathsep) if p and p != path])
    home = Path(environment.get("GAZEBOTEACH_SESSION_DIR", workspace/"build"))/"ros-home"
    home.mkdir(parents=True, exist_ok=True)
    environment.update(ROS_HOME=str(home), ROS_LOG_DIR=str(home/"log"), GAZEBOTEACH_WORKSPACE=str(workspace))
    return environment


def model_element(path):
    root = ET.parse(str(path)).getroot()
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        model = root.find("world/model")
    if model is None:
        raise ValueError("Actual converter output contains no SDF model")
    return model


def sdf_properties(path):
    result = {}
    for link in model_element(path).findall("link"):
        inertial = link.find("inertial")
        pose = [float(x) for x in (inertial.findtext("pose") or "0 0 0 0 0 0").split()]
        result[link.attrib["name"]] = {"mass": float(inertial.findtext("mass")), "com_pose": pose,
            "inertia": {name: float(inertial.findtext("inertia/"+name) or 0) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
            "gravity_mode": (link.findtext("gravity") or "true").strip() == "true",
            "collision_names": [item.attrib["name"] for item in link.findall("collision")]}
    return result


def edit(length=None, preserve=None):
    path = Path.cwd()/"arm_settings.json"
    before = read_settings(path)
    text = path.read_bytes().decode("utf-8")
    for field, value in (("length1_m", length), ("preserve_tool", preserve)):
        if value is None:
            continue
        matches = list(re.finditer(r'"'+field+r'"\s*:\s*(true|false|[-+0-9.eE]+)', text))
        if len(matches) != 1:
            raise ValueError("Unique Xacro settings scalar is missing")
        start, end = matches[0].span(1)
        text = text[:start]+json.dumps(value)+text[end:]
    temporary = path.with_name("settings-"+uuid.uuid4().hex+".tmp")
    try:
        temporary.write_bytes(text.encode("utf-8"))
        after = read_settings(temporary)
        os.replace(str(temporary), str(path))
        return {"before": before, "after": after, "reload_required": True}
    finally:
        if temporary.exists():
            temporary.unlink()
