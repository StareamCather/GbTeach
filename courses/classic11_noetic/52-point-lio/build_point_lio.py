#!/usr/bin/env python3
"""Build the pinned Point-LIO nodes and Livox message bindings in a private Noetic overlay."""

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

POINT_LIO_SHA = "1510c3bbf1743f254d83d0b22fbabb5b2d729f6b"
LIVOX_SHA = "3d240d5666129e1a3052e78ee8487a04b08fdda3"
ARCHIVES = {
    "point-lio.tar.gz": ("hku-mars/Point-LIO", POINT_LIO_SHA,
                         "a8b3a427a4942e20db719220e1755eb24ba5fdbc3cee17d98846bee7479d28f2"),
    "livox-ros-driver.tar.gz": ("Livox-SDK/livox_ros_driver", LIVOX_SHA,
                                "e953ffe14cdb45ea4989807924c5c7360f2ecde63e93b40272159928beae0f7b"),
}
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
PATCHES = {
    "generated-targets": "Order the mapper after generated Livox message headers.",
    "timed-velodyne-input-guards": "Check finite point fields, ring bounds, and a positive measured end offset before accepting Velodyne scans.",
    "working-buffer-bounds": "Reject scans outside the fixed solver array bound and resize indexed work vectors before writing.",
    "runtime-log-directory": "Use run-relative debug/PCD paths and guard unavailable optional debug files.",
    "scan-diagnostics": "Count time groups, successful native lidar updates, odometry calls and active IMU saturation masks per processed scan.",
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
    line = ("add_executable(pointlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp "
            "src/parameters.cpp src/preprocess.cpp src/Estimator.cpp)")
    text = replace_exact(text, line, line + "\n"
                         "add_dependencies(pointlio_mapping ${catkin_EXPORTED_TARGETS})")
    cmake.write_text(text)

    preprocess = source / "src/preprocess.cpp"
    text = "#include <cmath>\n" + preprocess.read_text()
    start = text.index("void Preprocess::velodyne_handler(")
    end = text.index("void Preprocess::", start + 1)
    velodyne = text[start:end]
    velodyne = replace_exact(velodyne, "    if (plsize == 0) return;",
        "    // GazeboTeach uses measured rolling-scan time; never infer it from azimuth.\n"
        "    if (plsize == 0 || !std::isfinite(pl_orig.points.back().time) ||\n"
        "        pl_orig.points.back().time <= 0 || N_SCANS <= 0 || point_filter_num <= 0) return;")
    start_fallback = velodyne.index("    /*** These variables only works")
    end_fallback = velodyne.index("    for (int i = 0;", start_fallback)
    velodyne = velodyne[:start_fallback] + velodyne[end_fallback:]
    start_fallback = velodyne.index("      if (!given_offset_time)")
    end_fallback = velodyne.index("      if (i % point_filter_num", start_fallback)
    velodyne = velodyne[:start_fallback] + velodyne[end_fallback:]
    velodyne = replace_exact(velodyne, "      PointType added_pt;",
        "      const auto &sample = pl_orig.points[i];\n"
        "      if (sample.ring >= N_SCANS || !std::isfinite(sample.time) || sample.time < 0 ||\n"
        "          !std::isfinite(sample.x) || !std::isfinite(sample.y) ||\n"
        "          !std::isfinite(sample.z) || !std::isfinite(sample.intensity)) continue;\n"
        "      PointType added_pt;")
    preprocess.write_text(text[:start] + velodyne + text[end:])

    estimator = source / "src/Estimator.cpp"
    text = estimator.read_text()
    text = replace_exact(text, "std::vector<int> time_seq;",
                         "std::vector<int> time_seq;\nunsigned long long gt_imu_axis_masks = 0;")
    for axis in range(6):
        line = f"ekfom_data.satu_check[{axis}] = true;"
        text = replace_exact(text, line, line + "\n\t\t\t++gt_imu_axis_masks;")
    estimator.write_text(text)

    backend = source / "src/laserMapping.cpp"
    text = backend.read_text()
    text = replace_exact(text, "string root_dir = ROOT_DIR;",
                         "string root_dir = ROOT_DIR;\nextern unsigned long long gt_imu_axis_masks;")
    text = replace_exact(text, "            time_seq = time_compressing<int>(feats_down_body);",
        "            if (feats_down_body->empty() ||\n"
        "                feats_down_body->size() > sizeof(point_selected_surf) / sizeof(point_selected_surf[0])) {\n"
        '                ROS_WARN("Skipping scan outside Point-LIO working-array bounds");\n'
        "                continue;\n"
        "            }\n"
        "            time_seq = time_compressing<int>(feats_down_body);")
    for name in ("crossmat_list", "pbody_list"):
        text = replace_exact(text, f"{name}.reserve(feats_down_size);",
                             f"{name}.resize(feats_down_size);")
    text = replace_exact(text, "            crossmat_list.resize(feats_down_size);",
        "            const auto masks_at_start = gt_imu_axis_masks;\n"
        "            unsigned gt_updates = 0, gt_odom_calls = 0;\n"
        "            crossmat_list.resize(feats_down_size);")
    for branch in ("output", "input"):
        start = text.index(f"                    if (!kf_{branch}.update_iterated_dyn_share_modified())")
        end = text.index("                    solve_start = omp_get_wtime();", start)
        text = text[:end] + "                    ++gt_updates;\n" + text[end:]
    line = "publish_odometry(pubOdomAftMapped);"
    # Only increment where the native publish call actually ran.
    text = replace_exact(text, "                        " + line,
                         "                        " + line + "\n                        ++gt_odom_calls;", count=2)
    text = replace_exact(text, "            if (!publish_odometry_without_downsample)\n            {\n                " + line,
                         "            if (!publish_odometry_without_downsample)\n            {\n                " + line +
                         "\n                ++gt_odom_calls;")
    text = replace_exact(text, "            /*** Debug variables Logging ***/",
        '            printf("GT_POINT_LIO_SCAN start=%.9f end=%.9f groups=%zu updates=%u "\n'
        '                   "publish_calls=%u saturated_axes=%llu seconds=%.9f\\n",\n'
        "                   Measures.lidar_beg_time, lidar_end_time, time_seq.size(), gt_updates,\n"
        "                   gt_odom_calls, gt_imu_axis_masks - masks_at_start, t5 - t0);\n"
        "            fflush(stdout);\n\n"
        "            /*** Debug variables Logging ***/")
    text = replace_exact(text, "                dump_lio_state_to_log(fp);",
                         "                if (fp) dump_lio_state_to_log(fp);")
    text = replace_exact(text, "    fout_out.close();", "    if (fp) fclose(fp);\n    fout_out.close();")
    backend.write_text(text)


def populate_sources(workspace, archives):
    destination = workspace / "src"
    if destination.is_dir():
        return
    with tempfile.TemporaryDirectory(dir=workspace) as directory:
        stage = Path(directory)
        src = stage / "src"
        src.mkdir()
        algorithm = extract_pinned(archives["point-lio.tar.gz"], stage, "Point-LIO-" + POINT_LIO_SHA)
        patch_sources(algorithm)
        algorithm.rename(src / "point_lio")
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
            "laser_mapping": prefix / "lib/point_lio/pointlio_mapping",
        }
        receipt = workspace.parent / "built.json"
        if not receipt.is_file() or not all(path.exists() for path in paths.values()):
            env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": str(Path.home()), "LANG": "C.UTF-8"}
            command = ("source /opt/ros/noetic/setup.bash && "
                       "exec catkin_make -j1 -l1 -DPYTHON_EXECUTABLE=/usr/bin/python3 "
                       "-DPython3_EXECUTABLE=/usr/bin/python3")
            log = workspace.parent / "build.log"
            print(f"Building isolated A52 Noetic overlay. Progress: {log}", flush=True)
            with log.open("w", encoding="utf-8") as output:
                status = subprocess.run(["/bin/bash", "-c", command], cwd=workspace, env=env,
                                        stdout=output, stderr=subprocess.STDOUT, timeout=900)
            if status.returncode:
                raise RuntimeError(f"Point-LIO build failed ({status.returncode}); see {log}")
        generated = paths["python_path"] / "livox_ros_driver/msg/_CustomMsg.py"
        if not generated.is_file() or not all(path.exists() for path in paths.values()):
            raise RuntimeError(f"Missing expected built executables or Livox Python bindings in {prefix}")
        value = {name: str(path) for name, path in paths.items()}
        value.update({
            "algorithm_commit": POINT_LIO_SHA,
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
             "gazeboteach/a52-point-lio").resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archives", type=Path, default=cache / "archives")
    parser.add_argument("--download", action="store_true", help="Fetch missing pinned official archives")
    parser.add_argument("--receipt", type=Path, default=Path("build/point-lio.json"))
    args = parser.parse_args()
    archives = {name: archive_path(args.archives, name, args.download) for name in ARCHIVES}
    value = build(cache, archives)
    atomic_json(args.receipt, value)
    print(f"A52 Point-LIO overlay prepared: {args.receipt}", flush=True)


if __name__ == "__main__":
    main()
