#!/usr/bin/env python3
"""Check installed camera/array tools, retaining the actual personal scene."""
import json
import os
from pathlib import Path
import subprocess
import numpy
from config import digest, read_world


def main():
    world = Path.cwd()/"world.sdf"
    config = read_world(world)
    package = subprocess.check_output(["rospack", "find", "gazebo_plugins"], text=True, timeout=5).strip()
    plugin = Path("/opt/ros/noetic/lib/libgazebo_ros_camera.so")
    if not plugin.is_file():
        raise RuntimeError("Missing installed Noetic camera plugin")
    result = {"config": config, "world_sha256": digest(world), "plugin": str(plugin),
              "plugin_sha256": digest(plugin), "gazebo_plugins": package, "numpy_version": numpy.__version__}
    path = Path.cwd()/"build/camera_lab.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))
    print(json.dumps({"prepared": True, **result}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
