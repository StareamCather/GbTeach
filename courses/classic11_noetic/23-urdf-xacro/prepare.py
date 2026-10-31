#!/usr/bin/env python3
"""Run installed Xacro and native URDF-to-SDF conversion, without extra downloads."""
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from config import PACKAGE, SOURCES, digest, overlay, read_settings, sdf_properties


def main():
    workspace = Path.cwd().resolve()
    config = read_settings()
    environment = overlay(workspace)
    if not environment.get("GAZEBOTEACH_SESSION_TOKEN") or not environment.get("ROS_MASTER_URI"):
        raise RuntimeError("Prepare inside the existing private GazeboTeach Master session")
    subprocess.run(["rosparam", "set", "/use_sim_time", "true"], env=environment, check=True, timeout=8)
    for name in SOURCES:
        path = workspace/name
        if path.suffix in (".sh", ".py") and b"\r" in path.read_bytes():
            raise ValueError("Executable source must use Unix LF: "+name)
    for path in (workspace/PACKAGE/"scripts").glob("*.py"):
        path.chmod(path.stat().st_mode | 0o111)
    package = subprocess.check_output(["rospack", "find", "gazeboteach_l23"], env=environment, text=True, timeout=8).strip()
    if Path(package).resolve() != (workspace/PACKAGE).resolve():
        raise RuntimeError("Private ROS package discovery resolved another path")
    build = workspace/"build"
    build.mkdir(exist_ok=True)
    urdf, sdf = build/"arm.urdf", build/"arm.sdf"
    command = ["xacro", str(workspace/PACKAGE/"urdf/arm.urdf.xacro"), "length1:="+str(config["length1_m"]), "preserve_tool:="+str(config["preserve_tool"]).lower()]
    xacro = subprocess.run(command, env=environment, capture_output=True, text=True, check=True, timeout=12)
    urdf.write_bytes(xacro.stdout.encode("utf-8"))
    robot = ET.fromstring(xacro.stdout)
    if robot.tag != "robot" or {link.attrib["name"] for link in robot.findall("link")} != {"world", "base_link", "arm1", "arm2", "tool_link"}:
        raise ValueError("Generated URDF link tree differs")
    parents = {}
    for joint in robot.findall("joint"):
        child = joint.find("child").attrib["link"]
        if child in parents:
            raise ValueError("A URDF child has multiple parent joints")
        parents[child] = joint.find("parent").attrib["link"]
    for child in parents:
        seen, current = set(), child
        while current in parents:
            if current in seen:
                raise ValueError("URDF link tree has a cycle")
            seen.add(current)
            current = parents[current]
        if current != "world":
            raise ValueError("URDF tree must have the special world root")
    converted = subprocess.run(["gz", "sdf", "-p", str(urdf)], env=environment, capture_output=True, text=True, check=True, timeout=12)
    sdf.write_bytes(converted.stdout.encode("utf-8"))
    properties = sdf_properties(sdf)
    expected = {"base_link", "arm1", "arm2"} | ({"tool_link"} if config["preserve_tool"] else set())
    if set(properties) != expected:
        raise ValueError("Actual URDF-to-SDF physical links differ from fixed-joint preservation: "+str(list(properties)))
    record = {"config": config, "settings_sha256": digest(workspace/"arm_settings.json"), "world_sha256": digest(workspace/"world.sdf"),
        "source_sha256": {name: digest(workspace/name) for name in SOURCES}, "generated_urdf_path": str(urdf), "generated_urdf_sha256": digest(urdf),
        "converted_sdf_path": str(sdf), "converted_sdf_sha256": digest(sdf), "sdf_link_properties": properties,
        "xacro_command": command, "xacro_stderr": xacro.stderr, "converter_command": converted.args, "converter_stderr": converted.stderr,
        "parser_evidence": "Actual installed xacro plus gz sdf -p; Python ElementTree structural checks. check_urdf was not run."}
    target = build/"arm_lab.json"
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(target))
    print(json.dumps({"prepared": True, **record}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
