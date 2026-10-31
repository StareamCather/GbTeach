#!/usr/bin/env python3
"""Validate the installed Noetic plugin and fixed physical world offline."""
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from config import ROBOTS, SOURCES, WORLD, parameters, sha


def main():
    root = Path.cwd().resolve()
    value = parameters()
    if not os.environ.get("GAZEBOTEACH_SESSION_TOKEN") or not os.environ.get("ROS_MASTER_URI"):
        raise RuntimeError("Prepare within the existing GazeboTeach private ROS Master session")
    for name in SOURCES:
        if name.endswith(".py") and b"\r" in (root/name).read_bytes():
            raise ValueError("Executable sources must use Unix LF: "+name)
    world = ET.parse(str(root/"world.sdf")).find("world")
    if world is None or world.get("name") != WORLD:
        raise ValueError("Keep the default Classic11 world")
    if world.find("physics/max_step_size").text != ".001":
        raise ValueError("Keep the 1ms physical integration step")
    for robot in ROBOTS:
        model = world.find("model[@name='"+robot+"']")
        if model is None or model.findtext("static") != "false":
            raise ValueError("Keep the two dynamic differential robots")
        plugins = model.findall("plugin")
        if len(plugins) != 1 or plugins[0].get("filename") != "libgazebo_ros_diff_drive.so":
            raise ValueError("Each robot must use only the installed ROS differential drive plugin")
        expected = {"robotNamespace": "/"+robot, "commandTopic": "cmd_vel", "odometryTopic": "odom",
                    "odometryFrame": robot+"/odom", "robotBaseFrame": robot+"/base_footprint",
                    "leftJoint": robot+"_left_joint", "rightJoint": robot+"_right_joint",
                    "wheelSeparation": ".36", "wheelDiameter": ".2", "odometrySource": "world",
                    "publishTf": "true", "publishOdomTF": "true", "publishWheelTF": "false",
                    "publishWheelJointState": "true", "updateRate": "50", "wheelTorque": "5", "wheelAcceleration": "0"}
        if {node.tag: node.text for node in plugins[0]} != expected:
            raise ValueError("Plugin fields/frames differ from this fixed lesson ABI")
        if {node.get("name") for node in model.findall("joint[@type='revolute']")} != {robot+"_left_joint", robot+"_right_joint"}:
            raise ValueError("Keep the actual left/right wheel joints")
    probe = subprocess.run(["gazebo", "--version"], capture_output=True, text=True, timeout=5)
    version = (probe.stdout+probe.stderr).strip()
    if probe.returncode not in (0, 255) or "version 11." not in version:
        raise RuntimeError("Gazebo Classic11 required")
    package = Path(subprocess.check_output(["rospack", "find", "gazebo_plugins"], text=True, timeout=6).strip())
    plugin = package.parent.parent/"lib/libgazebo_ros_diff_drive.so"
    package_version = ET.parse(str(package/"package.xml")).findtext("version")
    if not plugin.is_file() or package_version != "2.9.3":
        raise RuntimeError("This course requires the locally verified Noetic gazebo_plugins 2.9.3 ABI")
    converted = subprocess.run(["gz", "sdf", "-p", str(root/"world.sdf")], capture_output=True,
                               text=True, check=True, timeout=12)
    ET.fromstring(converted.stdout)
    # GazeboRos searches tf_prefix separately from robotNamespace. Empty local
    # values block accidental inheritance of a global prefix before plugin load.
    for key, setting in (("/use_sim_time", "true"), ("/alpha/tf_prefix", "''"), ("/beta/tf_prefix", "''")):
        subprocess.run(["rosparam", "set", key, setting], check=True, timeout=6)
    build = root/"build"
    build.mkdir(exist_ok=True)
    record = {"parameters": value, "parameters_sha256": sha(root/"parameters.json"),
              "source_sha256": {name: sha(root/name) for name in SOURCES}, "plugin_path": str(plugin),
              "plugin_sha256": sha(plugin), "plugin_package_version": package_version,
              "gazebo_version": version, "converter_stderr": converted.stderr,
              "load_method": "gzserver -u with libgazebo_ros_api_plugin.so; two physical SDF models"}
    temporary = build/"multi_robot.tmp"
    temporary.write_text(json.dumps(record, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    os.replace(str(temporary), str(build/"multi_robot.json"))
    print(json.dumps({"prepared": True, **record}, ensure_ascii=False))


if __name__ == "__main__":
    main()
