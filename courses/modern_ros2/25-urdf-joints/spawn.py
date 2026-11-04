"""Convert the current URDF with libsdformat and spawn the reviewed SDF model."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import _bootstrap  # Adds the installed course support directory.
from common import FORTRESS, SIM_CLI, SYSTEM_NAMESPACE, run, service


def main():
    urdf = Path("robot.urdf").resolve()
    converted = run([SIM_CLI, "sdf", "-p", str(urdf)], timeout=10)
    root = ET.fromstring(converted[converted.index("<sdf"):].strip())
    model = root.find("model")
    if model is None:
        raise RuntimeError("URDF conversion did not produce an SDF model")
    joints = {row.get("name"): row.get("type") for row in model.findall("joint")}
    links = {row.get("name") for row in model.findall("link")}
    if FORTRESS:
        for plugin in model.findall("plugin"):
            name, filename = plugin.get("name", ""), plugin.get("filename", "")
            if name.startswith("gz::sim::systems::") and filename.startswith("gz-sim-"):
                plugin.set("name", SYSTEM_NAMESPACE + name.removeprefix("gz::sim::systems::"))
                plugin.set("filename", "ignition-gazebo-" + filename.removeprefix("gz-sim-"))
    plugins = {row.get("name") for row in model.findall("plugin")}
    expected_plugins = {SYSTEM_NAMESPACE + name for name in
                        ("JointStatePublisher", "JointPositionController", "PosePublisher")}
    if (not {"base", "arm", "tool"} <= links or joints.get("shoulder") != "revolute"
            or joints.get("anchor") != "fixed" or joints.get("tool_mount") != "fixed"
            or not expected_plugins <= plugins):
        raise RuntimeError(f"Converted SDF lost a required link, joint or plugin: {links}, {joints}, {plugins}")
    output = Path("build/robot.converted.sdf").resolve()
    output.parent.mkdir(exist_ok=True)
    output.write_text(ET.tostring(root, encoding="unicode") + "\n", encoding="utf-8")
    service("/world/default/create", "gz.msgs.EntityFactory",
            "sdf_filename: " + json.dumps(str(output)) + ', name: "urdf_arm", allow_renaming: false')
    print(json.dumps({"urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                      "sdf_path": str(output), "native_joints": joints}), flush=True)


if __name__ == "__main__":
    main()
