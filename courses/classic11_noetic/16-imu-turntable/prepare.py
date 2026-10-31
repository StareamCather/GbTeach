#!/usr/bin/env python3
"""Validate both fixtures and record the actual input/stock plugin fingerprints."""
import json
import os
from pathlib import Path
import subprocess
from config import digest, read_parameters, read_world


def main():
    workspace = Path.cwd()
    read_world(workspace/"world.sdf")
    parameters = read_parameters(workspace/"parameters.json")
    package = subprocess.check_output(["rospack", "find", "gazebo_plugins"], text=True, timeout=5).strip()
    plugin = Path("/opt/ros/noetic/lib/libgazebo_ros_imu_sensor.so")
    if not plugin.is_file():
        raise RuntimeError("Missing installed stock Noetic IMU sensor plugin")
    output = workspace/"build/imu_lab.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    result = {"parameters": parameters, "world_sha256": digest(workspace/"world.sdf"),
              "parameters_sha256": digest(workspace/"parameters.json"), "plugin": str(plugin),
              "plugin_sha256": digest(plugin), "gazebo_plugins": package}
    temporary = output.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(output))
    print(json.dumps({"prepared": True, **result}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
