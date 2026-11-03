"""Convert the current URDF with libsdformat and spawn the reviewed SDF model."""

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import _bootstrap  # Adds the installed course support directory.
from common import run, service


def main():
    urdf = Path("robot.urdf").resolve()
    converted = run(["gz", "sdf", "-p", str(urdf)], timeout=10)
    root = ET.fromstring(converted[converted.index("<sdf"):].strip())
    model = root.find("model")
    if model is None:
        raise RuntimeError("URDF conversion did not produce an SDF model")
    joints = {row.get("name"): row.get("type") for row in model.findall("joint")}
    links = {row.get("name") for row in model.findall("link")}
    plugins = {row.get("name") for row in model.findall("plugin")}
    expected_plugins = {"gz::sim::systems::JointStatePublisher",
                        "gz::sim::systems::JointPositionController", "gz::sim::systems::PosePublisher"}
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
