#!/usr/bin/env python3
"""Build the pinned FAST-LIO 1 nodes and Livox message bindings in a private Noetic overlay."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

FAST_LIO_SHA = "6e1fa94351316eafac5fcf55c5fcd575f3e89829"
LIVOX_SHA = "3d240d5666129e1a3052e78ee8487a04b08fdda3"
ARCHIVES = {
    "fast-lio1.tar.gz": ("hku-mars/FAST_LIO", FAST_LIO_SHA,
                         "b5e05896a9290592eca705bf2f666e62316c67021432c3b65580726353af2fb6"),
    "livox-ros-driver.tar.gz": ("Livox-SDK/livox_ros_driver", LIVOX_SHA,
                                "e953ffe14cdb45ea4989807924c5c7360f2ecde63e93b40272159928beae0f7b"),
}
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
PATCHES = {
    "generated-targets": "Order both native targets after generated Livox and FAST-LIO message headers.",
    "feature-input-guards": "Check point count, finite fields and ring length before indexing.",
    "ordered-feature-times": "Sort surface features by original per-point time before backend synchronization.",
    "empty-measurement-guards": "Drop empty feature groups and clear stale undistortion output.",
    "scan-end-publication": "Timestamp estimate, TF, path and point clouds at the processed feature scan end.",
    "runtime-log-directory": "Write upstream debug files relative to the node's working directory.",
}
MESSAGE_CMAKE = """cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES CustomPoint.msg CustomMsg.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
"""
MESSAGE_XML = """<?xml version="1.0"?>
<package format="2">
  <name>livox_ros_driver</name>
  <version>0.0.0</version>
  <description>Livox ROS 1 message definitions only, without the hardware driver.</description>
  <maintainer email="dev@livoxtech.com">Livox Technology Company Limited</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <depend>std_msgs</depend>
  <exec_depend>message_runtime</exec_depend>
</package>
"""


def digest(path):
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def archive_path(directory, name, allow_download):
    repo, revision, expected = ARCHIVES[name]
    target = directory / name
    if target.exists():
        if digest(target) != expected:
            raise RuntimeError(f"Archive SHA256 mismatch: {target}")
        return target
    if not allow_download:
        raise RuntimeError(f"Missing {target}. Supply the pinned archive with --archives or use --download.")
    directory.mkdir(parents=True, exist_ok=True)
    url = f"https://codeload.github.com/{repo}/tar.gz/{revision}"
    print(f"Downloading pinned source {url}", flush=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as output:
            temp = Path(output.name)
            with urllib.request.urlopen(url, timeout=30) as response:
                remaining = 64 * 1024 * 1024
                while True:
                    block = response.read(1024 * 1024)
                    if not block:
                        break
                    remaining -= len(block)
                    if remaining < 0:
                        raise RuntimeError(f"Source archive exceeds its size bound: {url}")
                    output.write(block)
        if digest(temp) != expected:
            raise RuntimeError(f"Downloaded archive SHA256 mismatch: {url}")
        temp.replace(target)
    finally:
        if temp is not None and temp.exists():
            temp.unlink()
    return target


def extract_pinned(archive, directory, top_level):
    # Only allow ordinary files/directories under the expected upstream root.
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle.getmembers():
            parts = Path(member.name).parts
            if (not parts or parts[0] != top_level or ".." in parts or
                    not (member.isfile() or member.isdir())):
                raise RuntimeError(f"Unexpected archive entry: {member.name}")
        bundle.extractall(directory)
    return directory / top_level


def replace_exact(text, before, after, count=1):
    if text.count(before) != count:
        raise RuntimeError("Pinned FAST-LIO patch context changed: " + before[:100])
    return text.replace(before, after)


def patch_sources(source):
    cmake = source / "CMakeLists.txt"
    cmake_text = cmake.read_text()
    cmake_text = replace_exact(cmake_text, 'SET(CMAKE_BUILD_TYPE "Debug")',
                               'set(CMAKE_BUILD_TYPE "Release")')
    cmake_text = replace_exact(cmake_text, "find_package(OpenMP QUIET)", "find_package(OpenMP REQUIRED)")
    cmake_text = replace_exact(cmake_text, "add_executable(loam_feat_extract src/feature_extract.cpp)",
        "add_executable(loam_feat_extract src/feature_extract.cpp)\n"
        "add_dependencies(loam_feat_extract ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})")
    cmake_text = replace_exact(cmake_text, "add_dependencies(loam_laserMapping ${catkin_EXPORTED_TARGETS})",
        "add_dependencies(loam_laserMapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})")
    cmake.write_text(cmake_text)

    frontend = source / "src/feature_extract.cpp"
    text = frontend.read_text()
    start = text.index("void horizon_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)\n{")
    end = text.index("void velo16_handler(", start)
    horizon = text[start:end]
    horizon = replace_exact(horizon, "  uint plsize = msg->point_num;",
        "  if (N_SCANS <= 0 || group_size < 3 || msg->point_num != msg->points.size()) return;\n"
        "  uint plsize = msg->point_num;\n"
        "  if (plsize == 0) return;")
    horizon = replace_exact(horizon, "  for(uint i=1; i<plsize; i++)",
        "  for(uint i=0; i<plsize; i++)")
    horizon = replace_exact(horizon, "        && (!IS_VALID",
        "        && std::isfinite(msg->points[i].x) && std::isfinite(msg->points[i].y)\n"
        "        && std::isfinite(msg->points[i].z) && (!IS_VALID")
    horizon = replace_exact(horizon, "      if((std::abs(pl_full[i].x",
        "      if(i==0 || (std::abs(pl_full[i].x")
    horizon = replace_exact(horizon, "  if(pl_buff[0].size() <= 7) {return;}\n\n", "")
    horizon = replace_exact(horizon, "    plsize = pl.size();",
        "    plsize = pl.size();\n    if (plsize <= static_cast<unsigned>(group_size)) continue;")
    horizon = replace_exact(horizon, "  pub_func(pl_surf, pub_surf, msg->header.stamp);",
        "  // Per-ring feature extraction appends in ring order. The backend uses\n"
        "  // the final feature's timestamp as the synchronization boundary.\n"
        "  std::stable_sort(pl_surf.points.begin(), pl_surf.points.end(),\n"
        "      [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });\n"
        "  if (!pl_surf.empty()) pub_func(pl_surf, pub_surf, msg->header.stamp);")
    text = text[:start] + horizon + text[end:]
    text = "#include <algorithm>\n#include <cmath>\n#include <omp.h>\n" + text
    frontend.write_text(text)

    backend = source / "src/laserMapping.cpp"
    text = backend.read_text()
    text = replace_exact(text, "        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();",
        "        if (meas.lidar->empty()) {\n"
        "            lidar_buffer.pop_front();\n"
        "            return false;\n"
        "        }\n"
        "        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();")
    text = replace_exact(text, "            p_imu->Process(Measures, state, feats_undistort);",
        "            feats_undistort->clear();\n"
        "            p_imu->Process(Measures, state, feats_undistort);")
    text = replace_exact(text, "            /******* Publish current frame points in world coordinates:  *******/",
        "            ros::Time processed_stamp;\n"
        "            processed_stamp.fromSec(lidar_end_time);\n"
        "            /******* Publish current frame points in world coordinates:  *******/")
    for field in ("laserCloudFullRes3", "laserCloudMap", "odomAftMapped", "msg_body_pose"):
        text = replace_exact(text, f"{field}.header.stamp = ros::Time::now();",
                             f"{field}.header.stamp = processed_stamp;",
                             count=2 if field == "laserCloudFullRes3" else 1)
    text = replace_exact(text, 'std::cout << "~~~~"<<ROOT_DIR<<" file opened"',
                         'std::cout << "Log directory opened"')
    text = replace_exact(text, 'std::cout << "~~~~"<<ROOT_DIR<<" doesn\'t exist"',
                         'std::cout << "Log directory is unavailable"')
    backend.write_text(text)
    common = source / "include/common_lib.h"
    text = replace_exact(common.read_text(), 'std::string(std::string(ROOT_DIR) + "Log/"+ name)',
                         'std::string("Log/") + name')
    common.write_text(text)


def populate_sources(workspace, archives):
    destination = workspace / "src"
    if destination.is_dir():
        return
    with tempfile.TemporaryDirectory(dir=workspace) as directory:
        stage = Path(directory)
        src = stage / "src"
        src.mkdir()
        algorithm = extract_pinned(archives["fast-lio1.tar.gz"], stage, "FAST_LIO-" + FAST_LIO_SHA)
        patch_sources(algorithm)
        algorithm.rename(src / "fast_lio")
        livox = extract_pinned(archives["livox-ros-driver.tar.gz"], stage, "livox_ros_driver-" + LIVOX_SHA)
        messages = src / "livox_ros_driver"
        (messages / "msg").mkdir(parents=True)
        for name in ("CustomMsg.msg", "CustomPoint.msg"):
            shutil.copyfile(livox / "livox_ros_driver" / "msg" / name, messages / "msg" / name)
        shutil.copyfile(livox / "LICENSE.txt", messages / "LICENSE.txt")
        (messages / "CMakeLists.txt").write_text(MESSAGE_CMAKE)
        (messages / "package.xml").write_text(MESSAGE_XML)
        (src / "CMakeLists.txt").symlink_to(ROS_SETUP.parent / "share/catkin/cmake/toplevel.cmake")
        src.rename(destination)


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
            temp = Path(stream.name)
            json.dump(value, stream, indent=2)
            stream.write("\n")
        temp.replace(path)
    finally:
        if temp is not None and temp.exists():
            temp.unlink()


def build(cache, archives):
    import fcntl

    if not ROS_SETUP.is_file():
        raise RuntimeError(f"ROS Noetic is required at {ROS_SETUP}")
    host = Path("/etc/os-release").read_bytes()
    compiler = subprocess.check_output(["/usr/bin/g++", "--version"], timeout=5)
    key = hashlib.sha256(Path(__file__).read_bytes() + host + compiler + os.uname().machine.encode()).hexdigest()
    workspace = cache / key / "ws"
    workspace.mkdir(parents=True, exist_ok=True)
    with (workspace.parent / "build.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        populate_sources(workspace, archives)
        prefix = workspace / "devel"
        paths = {
            "workspace": workspace,
            "setup_bash": prefix / "setup.bash",
            "python_path": prefix / "lib/python3/dist-packages",
            "feature_extract": prefix / "lib/fast_lio/loam_feat_extract",
            "laser_mapping": prefix / "lib/fast_lio/loam_laserMapping",
        }
        receipt = workspace.parent / "built.json"
        if not receipt.is_file() or not all(path.exists() for path in paths.values()):
            env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": str(Path.home()), "LANG": "C.UTF-8"}
            command = ("source /opt/ros/noetic/setup.bash && "
                       "exec catkin_make -j1 -l1 -DPYTHON_EXECUTABLE=/usr/bin/python3 "
                       "-DPython3_EXECUTABLE=/usr/bin/python3 -DDEPLOY=OFF")
            log = workspace.parent / "build.log"
            print(f"Building isolated A50 Noetic overlay. Progress: {log}", flush=True)
            with log.open("w", encoding="utf-8") as output:
                status = subprocess.run(["/bin/bash", "-c", command], cwd=workspace, env=env,
                                        stdout=output, stderr=subprocess.STDOUT, timeout=900)
            if status.returncode:
                raise RuntimeError(f"FAST-LIO 1 build failed ({status.returncode}); see {log}")
        generated = paths["python_path"] / "livox_ros_driver/msg/_CustomMsg.py"
        if not generated.is_file() or not all(path.exists() for path in paths.values()):
            raise RuntimeError(f"Missing expected built executables or Livox Python bindings in {prefix}")
        value = {name: str(path) for name, path in paths.items()}
        value.update({
            "algorithm_commit": FAST_LIO_SHA,
            "livox_message_commit": LIVOX_SHA,
            "archive_sha256": {name: spec[2] for name, spec in ARCHIVES.items()},
            "patches": PATCHES,
            "feature_extract_sha256": digest(paths["feature_extract"]),
            "laser_mapping_sha256": digest(paths["laser_mapping"]),
            "build_key": key,
        })
        atomic_json(receipt, value)
        return value


def main():
    cache = (Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) /
             "gazeboteach/a50-fast-lio1").resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archives", type=Path, default=cache / "archives")
    parser.add_argument("--download", action="store_true", help="Fetch missing pinned official archives")
    parser.add_argument("--receipt", type=Path, default=Path("build/fast-lio1.json"))
    args = parser.parse_args()
    archives = {name: archive_path(args.archives, name, args.download) for name in ARCHIVES}
    value = build(cache, archives)
    atomic_json(args.receipt, value)
    print(f"A50 FAST-LIO 1 overlay prepared: {args.receipt}", flush=True)


if __name__ == "__main__":
    main()
