#!/usr/bin/env python3
"""Build fixed LIO-SAM and GTSAM sources in a private Noetic overlay."""

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

LIO_SAM_SHA = "0be1fbe6275fb8366d5b800af4fc8c76a885c869"
GTSAM_SHA = "6425000775da29c93719f37dce3f2de38a0064ec"  # Official 4.0.3 tag.
ARCHIVES = {
    "lio-sam.tar.gz": ("TixiaoShan/LIO-SAM", LIO_SAM_SHA,
                       "b10ba8d99a7b9e18c46daf600d6f1056d7a8baf22f77d1c721a5923aa5e4b5b6"),
    "gtsam-4.0.3.tar.gz": ("borglab/gtsam", GTSAM_SHA,
                          "4cc482c70a36ea3c0ed5d90af48ffe732c030faaa4d94f5969a713b2734ee299"),
}
ROS_SETUP = Path("/opt/ros/noetic/setup.bash")
GTSAM_FLAGS = (
    "-DCMAKE_BUILD_TYPE=Release",
    "-DCMAKE_INSTALL_RPATH=$ORIGIN",
    "-DGTSAM_USE_SYSTEM_EIGEN=ON",
    "-DGTSAM_BUILD_TESTS=OFF",
    "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF",
    "-DGTSAM_BUILD_UNSTABLE=OFF",
    "-DGTSAM_BUILD_WRAP=OFF",
    "-DGTSAM_WITH_TBB=OFF",
    "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF",
    "-DGTSAM_BUILD_WITH_CCACHE=OFF",
)


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


def build_gtsam(cache, archive, jobs):
    import fcntl

    host = Path("/etc/os-release").read_bytes()
    compiler = subprocess.check_output(["/usr/bin/g++", "--version"], timeout=5)
    key_bytes = ("\n".join(GTSAM_FLAGS) + GTSAM_SHA + os.uname().machine).encode()
    key = hashlib.sha256(key_bytes + host + compiler).hexdigest()
    directory = cache / "gtsam" / key
    directory.mkdir(parents=True, exist_ok=True)
    prefix = directory / "prefix"
    source = directory / ("gtsam-" + GTSAM_SHA)
    with (directory / "build.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        receipt = directory / "built.json"
        if receipt.is_file():
            config = Path(json.loads(receipt.read_text())["gtsam_config"])
            if config.is_file():
                return config
        if not source.is_dir():
            with tempfile.TemporaryDirectory(dir=directory) as stage:
                extracted = extract_pinned(archive, Path(stage), source.name)
                extracted.rename(source)
        build_dir = directory / "build"
        log = directory / "build.log"
        print(f"Building private GTSAM 4.0.3. Progress: {log}", flush=True)
        with log.open("w", encoding="utf-8") as output:
            configure = ["cmake", "-S", str(source), "-B", str(build_dir),
                         "-DCMAKE_INSTALL_PREFIX=" + str(prefix), *GTSAM_FLAGS]
            install = ["cmake", "--build", str(build_dir), "--target", "install", "--parallel", str(jobs)]
            for command, timeout in ((configure, 120), (install, 3600)):
                status = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, timeout=timeout)
                if status.returncode:
                    raise RuntimeError(f"GTSAM build failed ({status.returncode}); see {log}")
        configs = list(prefix.rglob("GTSAMConfig.cmake"))
        if len(configs) != 1:
            raise RuntimeError(f"Expected one installed GTSAMConfig.cmake in {prefix}")
        atomic_json(receipt, {"gtsam_config": str(configs[0]), "gtsam_prefix": str(prefix),
                              "gtsam_commit": GTSAM_SHA, "gtsam_flags": GTSAM_FLAGS})
        return configs[0]


PATCHES = {
    "noetic-build": "Use C++14, the OpenCV 4 core header, and -O2 -g0 for Noetic.",
    "unused-gtsam-header": "Remove the unused unstable fixed-lag header. The IMU graph uses stable iSAM2.",
    "projection-input-bound": "Skip empty clouds before reading the last point time.",
    "projection-and-feature-diagnostics": "Record actual deskew and extracted feature counts.",
    "finite-scan-to-map": "Skip undefined zero-distance line Jacobians, require full neighbors, and reject invalid pose updates.",
    "map-graph-diagnostics": "Record factors only after iSAM2 accepts the batch and returns optimized estimates.",
    "imu-graph-diagnostics": "Record actual integrated IMU sample count, factor insertion, accepted corrections and graph resets.",
}


def patch_sources(source):
    cmake = source / "CMakeLists.txt"
    text = cmake.read_text()
    text = replace_exact(text, 'set(CMAKE_CXX_FLAGS "-std=c++11")',
                         'set(CMAKE_CXX_FLAGS "-std=c++14")')
    text = replace_exact(text, 'set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")',
                         'set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g0 -pthread")')
    cmake.write_text(text)

    utility = source / "include/utility.h"
    utility.write_text(replace_exact(utility.read_text(), "#include <opencv/cv.h>",
                                     "#include <opencv2/core.hpp>"))

    projection = source / "src/imageProjection.cpp"
    text = "#include <cstdio>\n" + projection.read_text()
    text = replace_exact(text, "        timeScanCur = cloudHeader.stamp.toSec();",
        "        if (laserCloudIn->empty()) {\n"
        '            ROS_WARN("Dropping empty lidar cloud before deskew");\n'
        "            return false;\n"
        "        }\n"
        "        timeScanCur = cloudHeader.stamp.toSec();")
    text = replace_exact(text, "        publishClouds();\n\n        resetParameters();",
        "        publishClouds();\n"
        '        std::printf("GT_LIO_SAM_PROJECTION start=%.9f end=%.9f input=%zu projected=%zu imu=%d deskew=%d\\n",\n'
        "                    timeScanCur, timeScanEnd, laserCloudIn->size(), extractedCloud->size(),\n"
        "                    cloudInfo.imuAvailable ? imuPointerCur + 1 : 0,\n"
        "                    (deskewFlag == 1 && cloudInfo.imuAvailable) ? 1 : 0);\n"
        "        std::fflush(stdout);\n\n"
        "        resetParameters();")
    projection.write_text(text)

    features = source / "src/featureExtraction.cpp"
    text = "#include <cstdio>\n" + features.read_text()
    text = replace_exact(text, "        pubLaserCloudInfo.publish(cloudInfo);",
        "        pubLaserCloudInfo.publish(cloudInfo);\n"
        '        std::printf("GT_LIO_SAM_FEATURE stamp=%.9f corners=%zu surfaces=%zu\\n",\n'
        "                    cloudHeader.stamp.toSec(), cornerCloud->size(), surfaceCloud->size());\n"
        "        std::fflush(stdout);")
    features.write_text(text)

    mapping = source / "src/mapOptmization.cpp"
    text = "#include <cstdio>\n#include <stdexcept>\n" + mapping.read_text()
    text = replace_exact(text, "            updateInitialGuess();",
        "            updateInitialGuess();\n"
        "            for (const float value : transformTobeMapped) {\n"
        "                if (!std::isfinite(value)) {\n"
        '                    ROS_ERROR("GT_LIO_SAM_NUMERIC stage=initial_guess stamp=%.9f", timeLaserInfoCur);\n'
        '                    throw std::runtime_error("LIO-SAM received a nonfinite scan-to-map initial guess");\n'
        "                }\n"
        "            }")
    start = text.index("    void cornerOptimization()")
    end = text.index("    void surfOptimization()", start)
    corner = text[start:end]
    corner = replace_exact(corner, "        #pragma omp parallel for num_threads(numberOfCores)",
        "        unsigned gt_zero_distance_lines = 0, gt_invalid_lines = 0;\n"
        "        #pragma omp parallel for num_threads(numberOfCores) reduction(+:gt_zero_distance_lines,gt_invalid_lines)")
    corner = replace_exact(corner, "            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);",
        "            if (kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis) != 5) continue;")
    corner = replace_exact(corner, "                    float la =",
        "                    // At zero point-to-line distance the derivative of its norm is undefined.\n"
        "                    // A satisfied line supplies no residual to this linearization.\n"
        "                    if (!std::isfinite(a012) || !std::isfinite(l12) || l12 <= 1e-8f) {\n"
        "                        ++gt_invalid_lines;\n"
        "                        continue;\n"
        "                    }\n"
        "                    if (a012 <= 1e-8f) {\n"
        "                        ++gt_zero_distance_lines;\n"
        "                        continue;\n"
        "                    }\n\n"
        "                    float la =")
    corner = replace_exact(corner, "        }\n    }\n\n",
        "        }\n"
        "        if (gt_zero_distance_lines || gt_invalid_lines) {\n"
        '            ROS_INFO("GT_LIO_SAM_ZERO_LINE stamp=%.9f zero_distance=%u invalid_line=%u",\n'
        "                     timeLaserInfoCur, gt_zero_distance_lines, gt_invalid_lines);\n"
        "        }\n"
        "    }\n\n")
    text = text[:start] + corner + text[end:]
    text = replace_exact(text, "            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);",
        "            if (kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis) != 5) continue;")
    text = replace_exact(text, "                pa /= ps; pb /= ps; pc /= ps; pd /= ps;",
        "                if (!std::isfinite(ps) || ps <= 1e-8f) continue;\n"
        "                pa /= ps; pb /= ps; pc /= ps; pd /= ps;")
    text = replace_exact(text, "        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);",
        "        if (!cv::checkRange(matAtA) || !cv::checkRange(matAtB) ||\n"
        "            !cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR) || !cv::checkRange(matX)) {\n"
        '            ROS_ERROR("GT_LIO_SAM_NUMERIC stage=qr stamp=%.9f iter=%d rows=%d",\n'
        "                      timeLaserInfoCur, iterCount, laserCloudSelNum);\n"
        '            throw std::runtime_error("LIO-SAM scan-to-map QR did not yield a finite update");\n'
        "        }")
    text = replace_exact(text, "        transformTobeMapped[0] += matX.at<float>(0, 0);",
        "        for (int i = 0; i < 6; ++i) {\n"
        "            if (!std::isfinite(transformTobeMapped[i] + matX.at<float>(i, 0))) {\n"
        '                ROS_ERROR("GT_LIO_SAM_NUMERIC stage=pose_delta stamp=%.9f iter=%d axis=%d",\n'
        "                          timeLaserInfoCur, iterCount, i);\n"
        '                throw std::runtime_error("LIO-SAM scan-to-map pose delta is nonfinite");\n'
        "            }\n"
        "        }\n"
        "        transformTobeMapped[0] += matX.at<float>(0, 0);")
    text = replace_exact(text, "        // odom factor\n        addOdomFactor();",
        "        const auto gt_key = cloudKeyPoses3D->size();\n"
        "        const auto gt_before_odom = gtSAMgraph.size();\n"
        "        // odom factor\n"
        "        addOdomFactor();\n"
        "        const auto gt_odom = gtSAMgraph.size() - gt_before_odom;")
    text = replace_exact(text, "        // gps factor\n        addGPSFactor();",
        "        const auto gt_before_gps = gtSAMgraph.size();\n"
        "        // gps factor\n"
        "        addGPSFactor();\n"
        "        const auto gt_gps = gtSAMgraph.size() - gt_before_gps;")
    text = replace_exact(text, "        // loop factor\n        addLoopFactor();",
        "        // Keep only diagnostics before the native method drains its accepted queue.\n"
        "        const auto gt_loop_pairs = loopIndexQueue;\n"
        "        const auto gt_loop_measurements = loopPoseQueue;\n"
        "        const auto gt_loop_noises = loopNoiseQueue;\n"
        "        const gtsam::Values gt_pre_loop = gt_loop_pairs.empty() ? gtsam::Values() : isamCurrentEstimate;\n"
        "        const auto gt_before_loop = gtSAMgraph.size();\n"
        "        // loop factor\n"
        "        addLoopFactor();\n"
        "        const auto gt_loop = gtSAMgraph.size() - gt_before_loop;")
    text = replace_exact(text, "        isamCurrentEstimate = isam->calculateEstimate();",
        "        isamCurrentEstimate = isam->calculateEstimate();\n"
        '        std::printf("GT_LIO_SAM_GRAPH stamp=%.9f key=%zu prior=%zu odom=%zu gps=%zu loop=%zu\\n",\n'
        "                    timeLaserInfoCur, gt_key, gt_key == 0 ? gt_odom : size_t(0),\n"
        "                    gt_key == 0 ? size_t(0) : gt_odom, gt_gps, gt_loop);\n"
        "        for (size_t j = 0; j < gt_loop; ++j)\n"
        "        {\n"
        "            const int from = gt_loop_pairs[j].first, to = gt_loop_pairs[j].second;\n"
        "            if (!gt_pre_loop.exists(from) || !gt_pre_loop.exists(to)) continue;\n"
        "            const auto pre_from = gt_pre_loop.at<Pose3>(from);\n"
        "            const auto pre_to = gt_pre_loop.at<Pose3>(to);\n"
        "            const auto post_from = isamCurrentEstimate.at<Pose3>(from);\n"
        "            const auto post_to = isamCurrentEstimate.at<Pose3>(to);\n"
        "            const auto &relative = gt_loop_measurements[j];\n"
        "            const double pre_error = Pose3::Logmap(relative.between(pre_from.between(pre_to))).norm();\n"
        "            const double post_error = Pose3::Logmap(relative.between(post_from.between(post_to))).norm();\n"
        "            const double sigma = gt_loop_noises[j]->sigmas()(0);\n"
        '            std::printf("GT_LIO_SAM_LOOP stamp=%.9f from=%d to=%d from_stamp=%.9f to_stamp=%.9f fitness=%.9g "\n'
        '                        "pre_xyz=%.9g,%.9g,%.9g post_xyz=%.9g,%.9g,%.9g "\n'
        '                        "pre_to_xyz=%.9g,%.9g,%.9g post_to_xyz=%.9g,%.9g,%.9g "\n'
        '                        "pre_residual=%.9g post_residual=%.9g\\n",\n'
        "                        timeLaserInfoCur, from, to, cloudKeyPoses6D->points[from].time,\n"
        "                        cloudKeyPoses6D->points[to].time, sigma * sigma,\n"
        "                        pre_from.x(), pre_from.y(), pre_from.z(), post_from.x(), post_from.y(), post_from.z(),\n"
        "                        pre_to.x(), pre_to.y(), pre_to.z(), post_to.x(), post_to.y(), post_to.z(),\n"
        "                        pre_error, post_error);\n"
        "        }\n"
        "        std::fflush(stdout);")
    mapping.write_text(text)

    imu = source / "src/imuPreintegration.cpp"
    text = "#include <cstdio>\n" + imu.read_text()
    text = replace_exact(text, "#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>\n", "")
    for reason, key_after in (("initial", 0), ("periodic", 100)):
        if reason == "initial":
            marker = "            key = 1;\n            systemInitialized = true;"
        else:
            marker = "            key = 1;\n        }\n\n\n        // 1. integrate imu data and optimize"
        text = replace_exact(text, marker,
            f'            std::printf("GT_LIO_SAM_IMU_RESET stamp=%.9f reason={reason} key=%d\\n",\n'
            f"                        currentCorrectionTime, {key_after});\n"
            "            std::fflush(stdout);\n" + marker)
    text = replace_exact(text, "        // 1. integrate imu data and optimize\n",
        "        // 1. integrate imu data and optimize\n"
        "        unsigned gt_integrated_samples = 0;\n")
    text = replace_exact(text, "                lastImuT_opt = imuTime;",
        "                ++gt_integrated_samples;\n"
        "                lastImuT_opt = imuTime;")
    text = replace_exact(text, "        // Reset the optimization preintegration object.",
        "        const double gt_integrated_dt = imuIntegratorOpt_->deltaTij();\n"
        "        // Reset the optimization preintegration object.")
    text = replace_exact(text, "        if (failureDetection(prevVel_, prevBias_))\n",
        "        const bool gt_failure = failureDetection(prevVel_, prevBias_);\n"
        '        std::printf("GT_LIO_SAM_IMU stamp=%.9f key=%d imu=1 bias=1 pose=1 integrated=%.9f samples=%u accepted=%d\\n",\n'
        "                    currentCorrectionTime, key, gt_integrated_dt, gt_integrated_samples, gt_failure ? 0 : 1);\n"
        "        std::fflush(stdout);\n"
        "        if (gt_failure)\n")
    text = replace_exact(text, "            resetParams();\n            return;",
        '            std::printf("GT_LIO_SAM_IMU_RESET stamp=%.9f reason=failure key=%d\\n",\n'
        "                        currentCorrectionTime, key);\n"
        "            std::fflush(stdout);\n"
        "            resetParams();\n            return;")
    imu.write_text(text)


def populate_sources(workspace, archive):
    destination = workspace / "src"
    if destination.is_dir():
        return
    with tempfile.TemporaryDirectory(dir=workspace) as directory:
        stage = Path(directory)
        src = stage / "src"
        src.mkdir()
        algorithm = extract_pinned(archive, stage, "LIO-SAM-" + LIO_SAM_SHA)
        patch_sources(algorithm)
        algorithm.rename(src / "lio_sam")
        (src / "CMakeLists.txt").symlink_to(ROS_SETUP.parent / "share/catkin/cmake/toplevel.cmake")
        src.rename(destination)


def build_lio_sam(cache, archive, gtsam_config):
    import fcntl

    if not ROS_SETUP.is_file():
        raise RuntimeError(f"ROS Noetic is required at {ROS_SETUP}")
    key = hashlib.sha256(Path(__file__).read_bytes() + str(gtsam_config).encode()).hexdigest()
    workspace = cache / "lio-sam" / key / "ws"
    workspace.mkdir(parents=True, exist_ok=True)
    with (workspace.parent / "build.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        populate_sources(workspace, archive)
        prefix = workspace / "devel"
        bin_dir = prefix / "lib/lio_sam"
        executables = {
            "image_projection": bin_dir / "lio_sam_imageProjection",
            "feature_extraction": bin_dir / "lio_sam_featureExtraction",
            "map_optimization": bin_dir / "lio_sam_mapOptmization",
            "imu_preintegration": bin_dir / "lio_sam_imuPreintegration",
        }
        paths = {"workspace": workspace, "setup_bash": prefix / "setup.bash",
                 "python_path": prefix / "lib/python3/dist-packages", **executables}
        receipt = workspace.parent / "built.json"
        if not receipt.is_file() or not all(path.exists() for path in paths.values()):
            env = {"PATH": "/usr/local/bin:/usr/bin:/bin", "HOME": str(Path.home()), "LANG": "C.UTF-8"}
            command = ("source /opt/ros/noetic/setup.bash && "
                       "exec catkin_make -j1 -l1 -DPYTHON_EXECUTABLE=/usr/bin/python3 "
                       '-DPython3_EXECUTABLE=/usr/bin/python3 "-DGTSAM_DIR=$1"')
            log = workspace.parent / "build.log"
            print(f"Building isolated A53 Noetic overlay. Progress: {log}", flush=True)
            with log.open("w", encoding="utf-8") as output:
                status = subprocess.run(["/bin/bash", "-c", command, "bash", str(gtsam_config.parent)],
                                        cwd=workspace, env=env, stdout=output, stderr=subprocess.STDOUT,
                                        timeout=1200)
            if status.returncode:
                raise RuntimeError(f"LIO-SAM build failed ({status.returncode}); see {log}")
        generated = paths["python_path"] / "lio_sam/msg/_cloud_info.py"
        if not generated.is_file() or not all(path.exists() for path in paths.values()):
            raise RuntimeError(f"Missing expected built executables or LIO-SAM Python bindings in {prefix}")
        value = {name: str(path) for name, path in paths.items()}
        gtsam_prefix = next(parent for parent in gtsam_config.parents if parent.name == "prefix")
        value.update({
            "algorithm_commit": LIO_SAM_SHA, "gtsam_commit": GTSAM_SHA,
            "gtsam_prefix": str(gtsam_prefix), "gtsam_config": str(gtsam_config),
            "gtsam_flags": GTSAM_FLAGS, "archive_sha256": {name: spec[2] for name, spec in ARCHIVES.items()},
            "patches": PATCHES, "build_key": key,
        })
        value.update({name + "_sha256": digest(path) for name, path in executables.items()})
        atomic_json(receipt, value)
        return value


def main():
    cache = (Path(os.environ.get("XDG_CACHE_HOME", str(Path.home() / ".cache"))) /
             "gazeboteach/a53-lio-sam").resolve()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archives", type=Path, default=cache / "archives")
    parser.add_argument("--download", action="store_true", help="Fetch missing pinned official archives")
    parser.add_argument("--gtsam-only", action="store_true", help="Prebuild the pinned dependency")
    parser.add_argument("--jobs", type=int, default=2, choices=range(1, 5))
    parser.add_argument("--receipt", type=Path, default=Path("build/lio-sam.json"))
    args = parser.parse_args()
    archive = archive_path(args.archives, "gtsam-4.0.3.tar.gz", args.download)
    config = build_gtsam(cache, archive, args.jobs)
    if args.gtsam_only:
        print(f"A53 private GTSAM ready: {config}", flush=True)
        return
    source = archive_path(args.archives, "lio-sam.tar.gz", args.download)
    value = build_lio_sam(cache, source, config)
    atomic_json(args.receipt, value)
    print(f"A53 LIO-SAM overlay prepared: {args.receipt}", flush=True)


if __name__ == "__main__":
    main()
