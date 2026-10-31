#!/usr/bin/env python3
"""Three learner inputs and immutable preparation evidence for the actual graph."""
import hashlib
import json
import math
import os
from pathlib import Path

PACKAGE = "ros_ws/src/gazeboteach_l25"
MODEL = "servo_arm"
JOINT = "lesson25_hinge"
NAMESPACE = "/lesson25"
SERVOS = ("position_servo", "soft_servo")
SOURCES = ("config.py", "prepare.py", "ros_entry.py", "ros_lab.sh", PACKAGE+"/package.xml",
           PACKAGE+"/urdf/servo.urdf", PACKAGE+"/launch/servo.launch", PACKAGE+"/scripts/configuration_node.py")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_parameters(path=None):
    value = json.loads(Path(path or Path.cwd()/"parameters.json").read_text(encoding="utf-8"))
    if set(value) != {"p", "d", "target_rad"}:
        raise ValueError("Use exactly p, d, target_rad in parameters.json")
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value.values()):
        raise ValueError("Parameters must be finite numbers, not bools")
    if not (1 <= value["p"] <= 6 and .15 <= value["d"] <= 1.5 and -.5 <= value["target_rad"] <= .5):
        raise ValueError("Use p in [1,6], d in [.15,1.5], target_rad in [-.5,.5]")
    return {key: float(number) for key, number in value.items()}


def overlay(workspace):
    workspace = Path(workspace).resolve()
    environment = os.environ.copy()
    paths = [str(workspace/"ros_ws/src"), str(workspace/"vendor")]
    paths += [p for p in environment.get("ROS_PACKAGE_PATH", "").split(os.pathsep) if p and p not in paths]
    environment["ROS_PACKAGE_PATH"] = os.pathsep.join(paths)
    home = Path(environment.get("GAZEBOTEACH_SESSION_DIR", workspace/"build"))/"ros-home"
    home.mkdir(parents=True, exist_ok=True)
    environment.update(ROS_HOME=str(home), ROS_LOG_DIR=str(home/"log"), GAZEBOTEACH_WORKSPACE=str(workspace))
    return environment


def prepared(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    settings = read_parameters(workspace/"parameters.json")
    record = json.loads((workspace/"build/servo_lab.json").read_text(encoding="utf-8"))
    if settings != record["config"] or digest(workspace/"parameters.json") != record["settings_sha256"]:
        raise RuntimeError("Saved control settings differ from prepared inputs; reload/restart required")
    if digest(workspace/"world.sdf") != record["world_sha256"] or any(
            digest(workspace/name) != record["source_sha256"][name] for name in SOURCES):
        raise RuntimeError("Control source/world changed; reload/restart required")
    for path_key, hash_key in (("generated_urdf_path", "generated_urdf_sha256"),
                               ("controllers_path", "controllers_sha256"),
                               ("vendor_plugin_xml_path", "vendor_plugin_xml_sha256"),
                               ("vendor_library_path", "vendor_library_sha256")):
        if digest(record[path_key]) != record[hash_key]:
            raise RuntimeError("Prepared artifact changed; reload/restart required: "+path_key)
    return settings, record
