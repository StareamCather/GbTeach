#!/usr/bin/env python3
"""Validate the private source package and executable nodes; no catkin build is needed."""
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from config import PACKAGE, SOURCES, digest, launch_argv, overlay, read_settings, read_world


def main():
    workspace = Path.cwd().resolve()
    config = read_settings()
    read_world(workspace)
    for name in SOURCES:
        path = workspace/name
        if path.suffix in (".sh", ".py") and b"\r" in path.read_bytes():
            raise ValueError("Executable Python/shell source must use Unix LF; preserve edits and save with LF: "+name)
        if path.suffix in (".xml", ".launch"):
            ET.parse(str(path))
    for path in (workspace/PACKAGE/"scripts").glob("*.py"):
        path.chmod(path.stat().st_mode | 0o111)
    environment = overlay(workspace)
    if not environment.get("GAZEBOTEACH_SESSION_TOKEN") or not environment.get("ROS_MASTER_URI"):
        raise RuntimeError("Prepare must run within the existing GazeboTeach private Master session")
    # This executes after the private Master exists, BEFORE gzserver/ros::init.
    subprocess.run(["rosparam", "set", "/use_sim_time", "true"], env=environment, check=True, timeout=8)
    found = subprocess.check_output(["rospack", "find", "gazeboteach_l22"], env=environment, text=True, timeout=8).strip()
    if Path(found).resolve() != (workspace/PACKAGE).resolve():
        raise RuntimeError("rospack resolved a different package instead of the private overlay")
    nodes = subprocess.check_output(launch_argv(config, "nodes"), env=environment, text=True, timeout=10).splitlines()
    expected = {config["namespace"]+"/driver", config["namespace"]+"/observer"}
    if set(line.strip() for line in nodes if line.strip()) != expected:
        raise RuntimeError("Actual roslaunch --nodes names differ from the expected private namespace")
    data = {"config": config, "world_sha256": digest(workspace/"world.sdf"),
        "settings_sha256": digest(workspace/"launch_settings.json"), "source_sha256": {name: digest(workspace/name) for name in SOURCES},
        "package_path": found, "ros_package_path": environment["ROS_PACKAGE_PATH"],
        "launch_argv": launch_argv(config), "resolved_nodes": sorted(expected)}
    target = workspace/"build/launch_lab.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(target))
    print(json.dumps({"prepared": True, **data}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
