#!/usr/bin/env python3
"""Build unchanged, pinned official controllers offline in this personal copy."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET
from config import JOINT, PACKAGE, SOURCES, digest, overlay, read_parameters


def run(arguments, environment, timeout=20):
    result = subprocess.run(arguments, env=environment, text=True, capture_output=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError("Command failed: {}\n{}\n{}".format(arguments, result.stdout, result.stderr))
    return result


def main():
    workspace = Path.cwd().resolve()
    settings = read_parameters()
    build = workspace/"build"
    build.mkdir(exist_ok=True)
    environment = overlay(workspace)
    source_root = Path(os.environ["GAZEBOTEACH_SOURCE_ROOT"])
    upstream = source_root/"resources/third_party/effort_controllers"
    provenance = json.loads((upstream/"UPSTREAM.json").read_text(encoding="utf-8"))
    if any(digest(upstream/name) != expected for name, expected in provenance["unmodified_files"].items()):
        raise RuntimeError("Pinned official effort controller source fingerprint differs")
    vendor = workspace/"vendor/effort_controllers"
    vendor.mkdir(parents=True, exist_ok=True)
    for source in upstream.rglob("*"):
        if source.is_file() and "__pycache__" not in source.parts:
            destination = vendor/source.relative_to(upstream)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(source), str(destination))
    cmake = build/"effort-build"
    configured = run(["cmake", "-S", str(vendor), "-B", str(cmake), "-DCATKIN_ENABLE_TESTING=OFF",
                      "-DPYTHON_EXECUTABLE=/usr/bin/python3", "-DCMAKE_BUILD_TYPE=Release"], environment, 45)
    compiled = run(["cmake", "--build", str(cmake), "--", "-j2"], environment, 75)
    library = cmake/"devel/lib/libeffort_controllers.so"
    if not library.is_file():
        raise RuntimeError("Official effort controller build produced no shared library")
    # Use a unique private library token. A host installation can contain the
    # generic libeffort_controllers.so and pluginlib searches catkin paths first.
    # Only this generated personal XML is derived; pinned resource bytes remain
    # unchanged and its C++ classes are built from the original upstream source.
    plugin_xml = vendor/"effort_controllers_plugins.xml"
    descriptor = ET.parse(str(plugin_xml))
    if descriptor.getroot().tag != "library":
        raise ValueError("Unexpected pinned effort controller plugin descriptor")
    descriptor.getroot().set("path", "libgazeboteach_l25_effort")
    descriptor.write(str(plugin_xml), encoding="utf-8", xml_declaration=True)
    for relative in ("libgazeboteach_l25_effort.so", "lib/libgazeboteach_l25_effort.so"):
        destination = vendor/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(library), str(destination))
    run(["rospack", "profile"], environment, 12)
    found = run(["rospack", "find", "effort_controllers"], environment).stdout.strip()
    if Path(found).resolve() != vendor.resolve():
        raise RuntimeError("Expected the personal pinned effort controller package: "+found)
    plugins = run(["rospack", "plugins", "--attrib=plugin", "controller_interface"], environment).stdout
    if "effort_controllers" not in plugins or str(vendor) not in plugins:
        raise RuntimeError("Official controller export is not discoverable in the private overlay: "+plugins)
    run(["rosparam", "set", "/use_sim_time", "true"], environment)
    for path in (workspace/PACKAGE/"scripts").glob("*.py"):
        path.chmod(path.stat().st_mode | 0o111)
    urdf = workspace/PACKAGE/"urdf/servo.urdf"
    tree = ET.parse(str(urdf)).getroot()
    joint = tree.find("joint[@name='"+JOINT+"']")
    if joint is None or tree.find("transmission/joint/hardwareInterface").text != "hardware_interface/EffortJointInterface":
        raise ValueError("The actual URDF must expose the hinge through EffortJointInterface")
    generated = build/"servo.urdf"
    generated.write_bytes(urdf.read_bytes())
    controllers = build/"controllers.yaml"
    controllers.write_bytes(("""joint_state:
  type: joint_state_controller/JointStateController
  publish_rate: 50
position_servo:
  type: effort_controllers/JointPositionController
  joint: lesson25_hinge
  pid: {p: %.17g, i: 0.0, d: %.17g, i_clamp: 0.0, antiwindup: false}
soft_servo:
  type: effort_controllers/JointPositionController
  joint: lesson25_hinge
  pid: {p: 1.0, i: 0.0, d: 0.35, i_clamp: 0.0, antiwindup: false}
""" % (settings["p"], settings["d"])).encode())
    record = {"config": settings, "settings_sha256": digest(workspace/"parameters.json"),
              "world_sha256": digest(workspace/"world.sdf"),
              "source_sha256": {name: digest(workspace/name) for name in SOURCES},
              "generated_urdf_path": str(generated), "generated_urdf_sha256": digest(generated),
              "controllers_path": str(controllers), "controllers_sha256": digest(controllers),
              "vendor_library_path": str(vendor/"libgazeboteach_l25_effort.so"),
              "vendor_library_sha256": digest(library), "vendor_package_path": str(vendor),
              "vendor_plugin_xml_path": str(plugin_xml), "vendor_plugin_xml_sha256": digest(plugin_xml),
              "derived_plugin_descriptor": {"upstream_library_token": "libeffort_controllers",
                                            "private_library_token": "libgazeboteach_l25_effort",
                                            "change": "Only the personal-copy library path attribute is rewritten."},
              "upstream_commit": provenance["commit"], "plugin_exports": plugins,
              "configure_stdout": configured.stdout, "compile_stdout": compiled.stdout}
    (build/"servo_lab.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"prepared": True, "config": settings, "vendor_library": record["vendor_library_path"],
                      "vendor_library_sha256": record["vendor_library_sha256"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
