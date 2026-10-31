#!/usr/bin/env python3
"""Check the actual personal scene and record input; do not regenerate it."""
import json
import os
from pathlib import Path
import subprocess
from config import digest, read_world


def main():
    world = Path.cwd()/"world.sdf"
    config = read_world(world)
    package = subprocess.check_output(["rospack", "find", "gazebo_plugins"], text=True, timeout=5).strip()
    library = Path("/opt/ros/noetic/lib/libgazebo_ros_bumper.so")
    if not library.is_file():
        raise RuntimeError("Missing installed Noetic libgazebo_ros_bumper.so")
    output = Path.cwd()/"build/contact_lab.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    data = {"world_sha256": digest(world), "config": config, "plugin": str(library),
            "plugin_sha256": digest(library), "gazebo_plugins": package}
    temporary = output.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(output))
    print(json.dumps({"prepared": True, **data}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
