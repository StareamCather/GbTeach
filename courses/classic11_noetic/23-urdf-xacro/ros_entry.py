#!/usr/bin/env python3
"""Use the existing private Master; spawn one generated URDF, never another Gazebo."""
import os
from pathlib import Path
import sys
from config import overlay, prepared


def main():
    workspace = Path(__file__).resolve().parent
    config, record = prepared(workspace)
    environment = overlay(workspace)
    operation = sys.argv[1] if len(sys.argv) > 1 else "start"
    if operation not in ("start", "nodes", "params", "find"):
        raise ValueError("Use start/nodes/params/find")
    if operation == "start" and not environment.get("GAZEBOTEACH_SESSION_TOKEN"):
        raise RuntimeError("Start from the GazeboTeach private session")
    arguments = ["rospack", "find", "gazeboteach_l23"] if operation == "find" else ["roslaunch"]+{
        "start": ["--wait"], "nodes": ["--nodes"], "params": ["--dump-params"]}[operation]+[
        "gazeboteach_l23", "arm.launch", "description_file:="+record["generated_urdf_path"]]
    os.chdir(str(workspace))
    os.execvpe(arguments[0], arguments, environment)


if __name__ == "__main__":
    main()
