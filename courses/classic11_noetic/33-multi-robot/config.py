#!/usr/bin/env python3
"""Small command input surface and immutable loaded-world fingerprint."""
import hashlib
import json
from pathlib import Path

ROBOTS = ("alpha", "beta")
WORLD = "default"
SOURCES = ("config.py", "prepare.py", "bridge.py", "lab.py", "verify.py", "world.sdf")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def parameters(path=None):
    value = json.loads(Path(path or Path.cwd()/"parameters.json").read_text(encoding="utf-8"))
    allowed = {"linear_speed_mps": (.15, .20, .30), "angular_speed_rps": (.40, .60, .90)}
    if set(value) != set(allowed) or any(type(value[key]) not in (int, float) or value[key] not in choices
                                       for key, choices in allowed.items()):
        raise ValueError("Use linear_speed_mps 0.15/0.20/0.30 and angular_speed_rps 0.40/0.60/0.90 only")
    return value


def prepared(root=None):
    root = Path(root or Path.cwd()).resolve()
    record = json.loads((root/"build/multi_robot.json").read_text(encoding="utf-8"))
    value = parameters(root/"parameters.json")
    if value != record["parameters"] or sha(root/"parameters.json") != record["parameters_sha256"]:
        raise RuntimeError("Saved command parameters changed: reload/restart the lesson")
    if any(sha(root/name) != record["source_sha256"][name] for name in SOURCES):
        raise RuntimeError("Loaded lesson source/world differs: reload/restart the lesson")
    return value, record


def identity(record):
    return {key: record[key] for key in ("parameters", "parameters_sha256", "source_sha256", "plugin_sha256")}
