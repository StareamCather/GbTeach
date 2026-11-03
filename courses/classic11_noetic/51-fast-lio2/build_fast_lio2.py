#!/usr/bin/env python3
"""Build the pinned FAST-LIO2 nodes and Livox message bindings in a private Noetic overlay."""

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

FAST_LIO_SHA = "7cc4175de6f8ba2edf34bab02a42195b141027e9"
IKD_SHA = "e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"
LIVOX_SHA = "3d240d5666129e1a3052e78ee8487a04b08fdda3"
ARCHIVES = {
    "fast-lio2.tar.gz": ("hku-mars/FAST_LIO", FAST_LIO_SHA,
                         "10fea840f01cb36c5c8922eb0d491864d940976e053ca31430ec2549c76ebd0c"),
    "ikd-tree.tar.gz": ("hku-mars/ikd-Tree", IKD_SHA,
                        "74f6e20d5b8e369bd20fede7bef2735d0e263658ee0b912a1f126dc5f63f7d59"),
    "livox-ros-driver.tar.gz": ("Livox-SDK/livox_ros_driver", LIVOX_SHA,
                                "e953ffe14cdb45ea4989807924c5c7360f2ecde63e93b40272159928beae0f7b"),
}
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
PATCHES = {
    "generated-targets": "Order the mapper after generated Livox and FAST-LIO2 message headers.",
    "velodyne-measurement-times": "Sort ring-grouped feature output, skip invalid timed Velodyne scans, and use the final retained point's actual offset.",
    "runtime-log-directory": "Use run-relative debug/PCD paths and guard unavailable optional debug files.",
    "scan-diagnostics": "Log measured preprocessing and mapping stages and point counts without feeding them back into estimation.",
    "local-map-snapshot": "On request, publish the active ikd-Tree local map periodically from a locked snapshot.",
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
    text = cmake.read_text()
    text = replace_exact(text, 'SET(CMAKE_BUILD_TYPE "Debug")', 'set(CMAKE_BUILD_TYPE "Release")')
    text = replace_exact(text, "find_package(OpenMP QUIET)", "find_package(OpenMP REQUIRED)")
    text = replace_exact(text, 'add_definitions(-DROOT_DIR=\\"${CMAKE_CURRENT_SOURCE_DIR}/\\")',
                         'add_definitions(-DROOT_DIR=\\"./\\")')
    line = "add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)"
    text = replace_exact(text, line, line + "\n"
        "add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})")
    cmake.write_text(text)

    backend = source / "src/laserMapping.cpp"
    text = "#include <algorithm>\n#include <cmath>\n" + backend.read_text()
    text = replace_exact(text, "bool   runtime_pos_log =",
        "bool local_map_en = false;\nbool   runtime_pos_log =")
    text = replace_exact(text, '    nh.param<bool>("publish/path_en",path_en, true);',
        '    nh.param<bool>("publish/local_map_en", local_map_en, false);\n'
        '    nh.param<bool>("publish/path_en",path_en, true);')
    line = "    p_pre->process(msg, ptr);\n    lidar_buffer.push_back(ptr);"
    start = text.index("void standard_pcl_cbk(")
    end = text.index("void livox_pcl_cbk(", start)
    standard = replace_exact(text[start:end], line,
        "    p_pre->process(msg, ptr);\n"
        "    if (lidar_type == VELO16 && p_pre->feature_enabled) {\n"
        "        // The feature branch appends surfaces ring by ring.\n"
        "        std::stable_sort(ptr->points.begin(), ptr->points.end(),\n"
        "            [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });\n"
        "    }\n"
        "    const double preprocess_seconds = omp_get_wtime() - preprocess_start_time;\n"
        '    printf("GT_FAST_LIO2_PRE stamp=%.9f points=%zu seconds=%.9f\\n",\n'
        "           msg->header.stamp.toSec(), ptr->points.size(), preprocess_seconds);\n"
        "    fflush(stdout);\n"
        "    lidar_buffer.push_back(ptr);")
    standard = replace_exact(standard, "        lidar_buffer.clear();",
                             "        lidar_buffer.clear();\n        time_buffer.clear();\n        lidar_pushed = false;")
    text = text[:start] + standard + text[end:]
    line = "        if (meas.lidar->points.size() <= 1) // time too little"
    text = replace_exact(text, line,
        "        if (lidar_type == VELO16)\n"
        "        {\n"
        "            if (meas.lidar->size() <= 1 || !std::isfinite(meas.lidar->back().curvature) ||\n"
        "                meas.lidar->back().curvature <= 0)\n"
        "            {\n"
        "                lidar_buffer.pop_front();\n"
        "                time_buffer.pop_front();\n"
        '                ROS_WARN("Discarding Velodyne scan with no valid retained time extent");\n'
        "                return false;\n"
        "            }\n"
        "            lidar_end_time = meas.lidar_beg_time + meas.lidar->back().curvature / 1000.0;\n"
        "        }\n"
        "        else if (meas.lidar->points.size() <= 1) // time too little")
    text = replace_exact(text, "            p_imu->Process(Measures, kf, feats_undistort);",
        "            feats_undistort->clear();\n"
        "            p_imu->Process(Measures, kf, feats_undistort);")
    text = replace_exact(text, "            // publish_map(pubLaserCloudMap);",
        "            const int valid_map_points = ikdtree.validnum();\n"
        '            printf("GT_FAST_LIO2_SCAN start=%.9f end=%.9f points=%zu downsampled=%d "\n'
        '                   "effective=%d map_points=%d seconds=%.9f\\n",\n'
        "                   Measures.lidar_beg_time, lidar_end_time, feats_undistort->size(),\n"
        "                   feats_down_size, effct_feat_num, valid_map_points, t5 - t0);\n"
        "            fflush(stdout);\n"
        "            static unsigned map_publish_counter = 0;\n"
        "            if (local_map_en && (++map_publish_counter % 10) == 1)\n"
        "            {\n"
        "                ikdtree.snapshot(featsFromMap->points);\n"
        "                featsFromMap->width = featsFromMap->points.size();\n"
        "                featsFromMap->height = 1;\n"
        "                publish_map(pubLaserCloudMap);\n"
        "            }")
    text = replace_exact(text, "                dump_lio_state_to_log(fp);",
                         "                if (fp) dump_lio_state_to_log(fp);")
    text = replace_exact(text, "    fout_out.close();",
                         "    if (fp) fclose(fp);\n    fout_out.close();")
    text = replace_exact(text, '        fp2 = fopen(log_dir.c_str(),"w");',
        '        fp2 = fopen(log_dir.c_str(),"w");\n'
        '        if (!fp2) { ROS_WARN("Could not open optional timing log"); return 0; }')
    backend.write_text(text)

    tree_header = source / "include/ikd-Tree/ikd_Tree.h"
    text = tree_header.read_text()
    declaration = "    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type);"
    tree_header.write_text(replace_exact(text, declaration,
        declaration + "\n    void snapshot(PointVector &storage);"))
    tree_cpp = source / "include/ikd-Tree/ikd_Tree.cpp"
    text = tree_cpp.read_text()
    anchor = "template <typename PointType>\nvoid KD_TREE<PointType>::flatten("
    text = replace_exact(text, anchor,
        "template <typename PointType>\n"
        "void KD_TREE<PointType>::snapshot(PointVector &storage)\n"
        "{\n"
        "    // Hold the outer rebuild lock: flatten's Push_Down can lock working_flag_mutex.\n"
        "    pthread_mutex_lock(&rebuild_ptr_mutex_lock);\n"
        "    try {\n"
        "        storage.clear();\n"
        "        flatten(Root_Node, storage, NOT_RECORD);\n"
        "    } catch (...) {\n"
        "        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);\n"
        "        throw;\n"
        "    }\n"
        "    pthread_mutex_unlock(&rebuild_ptr_mutex_lock);\n"
        "}\n\n" + anchor)
    tree_cpp.write_text(text)


def populate_sources(workspace, archives):
    destination = workspace / "src"
    if destination.is_dir():
        return
    with tempfile.TemporaryDirectory(dir=workspace) as directory:
        stage = Path(directory)
        src = stage / "src"
        src.mkdir()
        algorithm = extract_pinned(archives["fast-lio2.tar.gz"], stage, "FAST_LIO-" + FAST_LIO_SHA)
        ikd_tree = extract_pinned(archives["ikd-tree.tar.gz"], stage, "ikd-Tree-" + IKD_SHA)
        tree_target = algorithm / "include/ikd-Tree"
        if tree_target.exists():
            tree_target.rmdir()  # GitHub archive contains an empty gitlink directory.
        ikd_tree.rename(tree_target)
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
            "laser_mapping": prefix / "lib/fast_lio/fastlio_mapping",
        }
        receipt = workspace.parent / "built.json"
        if not receipt.is_file() or not all(path.exists() for path in paths.values()):
            env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": str(Path.home()), "LANG": "C.UTF-8"}
            command = ("source /opt/ros/noetic/setup.bash && "
                       "exec catkin_make -j1 -l1 -DPYTHON_EXECUTABLE=/usr/bin/python3 "
                       "-DPython3_EXECUTABLE=/usr/bin/python3")
            log = workspace.parent / "build.log"
            print(f"Building isolated A51 Noetic overlay. Progress: {log}", flush=True)
            with log.open("w", encoding="utf-8") as output:
                status = subprocess.run(["/bin/bash", "-c", command], cwd=workspace, env=env,
                                        stdout=output, stderr=subprocess.STDOUT, timeout=900)
            if status.returncode:
                raise RuntimeError(f"FAST-LIO2 build failed ({status.returncode}); see {log}")
        generated = paths["python_path"] / "livox_ros_driver/msg/_CustomMsg.py"
        if not generated.is_file() or not all(path.exists() for path in paths.values()):
            raise RuntimeError(f"Missing expected built executables or Livox Python bindings in {prefix}")
        value = {name: str(path) for name, path in paths.items()}
        value.update({
            "algorithm_commit": FAST_LIO_SHA,
            "ikd_tree_commit": IKD_SHA,
            "livox_message_commit": LIVOX_SHA,
            "archive_sha256": {name: spec[2] for name, spec in ARCHIVES.items()},
            "patches": PATCHES,
            "laser_mapping_sha256": digest(paths["laser_mapping"]),
            "build_key": key,
        })
        atomic_json(receipt, value)
        return value


def main():
    cache = (Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) /
             "gazeboteach/a51-fast-lio2").resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archives", type=Path, default=cache / "archives")
    parser.add_argument("--download", action="store_true", help="Fetch missing pinned official archives")
    parser.add_argument("--receipt", type=Path, default=Path("build/fast-lio2.json"))
    args = parser.parse_args()
    archives = {name: archive_path(args.archives, name, args.download) for name in ARCHIVES}
    value = build(cache, archives)
    atomic_json(args.receipt, value)
    print(f"A51 FAST-LIO2 overlay prepared: {args.receipt}", flush=True)


if __name__ == "__main__":
    main()
