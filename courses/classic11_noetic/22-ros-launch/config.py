#!/usr/bin/env python3
"""Explicit lesson settings and fingerprints of the actual private source package."""
import hashlib
import json
import os
from pathlib import Path
import re
import uuid
import xml.etree.ElementTree as ET

PACKAGE = "ros_ws/src/gazeboteach_l22"
SOURCES = ("config.py", "ros_entry.py", "ros_lab.sh", PACKAGE+"/package.xml",
    PACKAGE+"/launch/lab.launch", PACKAGE+"/launch/nodes.launch",
    PACKAGE+"/scripts/common.py", PACKAGE+"/scripts/pulse_driver.py", PACKAGE+"/scripts/state_observer.py")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(element):
    return element.tag, tuple(sorted(element.attrib.items())), (element.text or "").strip(), tuple(canonical(child) for child in element)


def read_settings(path=None):
    data = json.loads(Path(path or Path.cwd()/"launch_settings.json").read_bytes().decode("utf-8"))
    if set(data) != {"force_n", "robot_ns"} or type(data["force_n"]) is not int or data["force_n"] not in (2, 4) or data["robot_ns"] not in ("cart_a", "cart_b"):
        raise ValueError("Only force_n2/4 and robot_ns cart_a/cart_b are supported")
    return {**data, "namespace": "/lesson22/"+data["robot_ns"], "duration_s": .3,
            "lead_s": .1, "mass_kg": 1, "physics_step_s": .001, "model_name": "launch_slider", "body_name": "launch_slider::body"}


def read_world(workspace):
    workspace = Path(workspace)
    official = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent))
    if canonical(ET.parse(str(workspace/"world.sdf")).getroot()) != canonical(ET.parse(str(official/"world.sdf")).getroot()):
        raise ValueError("Fixed world mass/inertia/gravity/damping/physics/body geometry differs from template")


def prepared(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    config = read_settings(workspace/"launch_settings.json")
    read_world(workspace)
    record = json.loads((workspace/"build/launch_lab.json").read_text(encoding="utf-8"))
    if config != record["config"] or digest(workspace/"launch_settings.json") != record["settings_sha256"] or digest(workspace/"world.sdf") != record["world_sha256"]:
        raise RuntimeError("Saved inputs differ from the loaded lesson: reload/restart required")
    if any(digest(workspace/name) != record["source_sha256"][name] for name in SOURCES):
        raise RuntimeError("Source package changed since launch; reload/restart required")
    return config, record


def overlay(workspace, session=None):
    workspace = Path(workspace).resolve()
    environment = os.environ.copy()
    path = str(workspace/"ros_ws/src")
    existing = [item for item in environment.get("ROS_PACKAGE_PATH", "").split(os.pathsep) if item and item != path]
    environment["ROS_PACKAGE_PATH"] = os.pathsep.join([path]+existing)
    home = Path(session or environment.get("GAZEBOTEACH_SESSION_DIR", workspace/"build"))/"ros-home"
    home.mkdir(parents=True, exist_ok=True)
    environment["ROS_HOME"] = str(home)
    environment["ROS_LOG_DIR"] = str(home/"log")
    environment["GAZEBOTEACH_WORKSPACE"] = str(workspace)
    return environment


def launch_argv(config, operation="start", overrides=None):
    values = {"robot_ns": config["robot_ns"], "force_n": str(config["force_n"])}
    values.update(overrides or {})
    flags = {"start": ["--wait"], "nodes": ["--nodes"], "params": ["--dump-params"]}[operation]
    return ["roslaunch"]+flags+["gazeboteach_l22", "lab.launch"]+[key+":="+str(value) for key, value in values.items()]


def edit(force=None, namespace=None):
    path = Path.cwd()/"launch_settings.json"
    before = read_settings(path)
    text = path.read_bytes().decode("utf-8")
    for field, value in (("force_n", force), ("robot_ns", namespace)):
        if value is None:
            continue
        matches = list(re.finditer(r'"'+field+r'"\s*:\s*("[^"]*"|[0-9]+)', text))
        if len(matches) != 1:
            raise ValueError("Unique settings scalar is missing")
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
