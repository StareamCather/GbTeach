#!/usr/bin/env python3
"""Actual depth/cloud bytes: optical Z, invalid values, layout, and projection.

All scenarios use a real graphical session and fresh private ROS/Gazebo pairs.
Organized clouds are decoded using their declared offsets and strides. The
canonical observer's summary is never used as a substitute for raw measurements.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import actual_pose
from tests.lesson06_acceptance import actual_stats

CAMERA = "./world/model[@name='camera_rig']/link/sensor/camera"
PLUGIN = "./world/model[@name='camera_rig']/link/sensor/plugin"


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def mutate(path, width=None, cutoff=None):
    tree = ET.parse(str(path))
    if width is not None:
        tree.find(CAMERA+"/image/width").text = str(width)
        tree.find(CAMERA+"/image/height").text = str(width*3//4)
    if cutoff is not None:
        tree.find(PLUGIN+"/pointCloudCutoffMax").text = str(cutoff)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def blob(metadata):
    data = Path(metadata["data_path"]).read_bytes()
    require(hashlib.sha256(data).hexdigest() == metadata["data_sha256"]
            and len(data) == metadata["data_length"], "Raw sensor bytes do not match their declared SHA/length")
    return data


def inspect_raw(report, width, cutoff):
    import numpy as np
    height = width*3//4
    focal = width/(2*math.tan(float(format(math.pi/3, ".6g"))/2))
    coordinates = {name: (round(x*width), round(y*height)) for name, x, y in (
        ("foreground", .5, .5), ("wall_a", .7, .5), ("wall_b", .8, .5),
        ("near_probe", .25, .5), ("far_probe", .5, .075), ("no_return", .03, .05))}
    cache, windows = {}, {}
    for name, phase in report["phases"].items():
        rows = [r for r in report["trace"] if r["phase"] == name and r["disposition"] == "accepted"]
        stamps = [r["stamp_ns"] for r in rows]
        require(len(rows) >= 12 and all(b > a for a, b in zip(stamps, stamps[1:])), "Depth windows lack distinct actual triples")
        span = (stamps[-1]-stamps[0])/1e9
        require(span >= 1 and 8 <= (len(rows)-1)/span <= 12, "Actual depth simulation frequency/duration differs")
        wall_front = phase["wall_pose"]["position"][0]-.05
        final_samples = None
        for row in rows:
            depth, cloud, info = row["depth"], row["cloud"], row["camera_info"]
            require(row["stamp_ns"] == depth["stamp_ns"] == info["stamp_ns"] == cloud["stamp_ns"], "Depth/Info/Cloud are not exact-stamp triples")
            require(all(m["width"] == width and m["height"] == height for m in (depth, cloud, info)), "Actual sensor dimensions differ from the input")
            require(depth["encoding"] == "32FC1" and depth["step"] >= width*4, "Depth bytes are not scalar float meters")
            require(depth["frame"] == cloud["frame"] == info["frame"] == "depth_optical_frame", "Unexpected optical-frame label")
            require(not cloud["is_dense"], "Known invalid pixels were advertised as a dense point cloud")
            require(all(math.isfinite(v) for v in info["K"]) and abs(info["K"][0]-focal) < 1e-5
                    and info["K"][2] == (width+1)/2 and info["K"][5] == (height+1)/2,
                    "Actual CameraInfo differs from the known SDK convention")
            key = (depth["data_sha256"], cloud["data_sha256"])
            if key not in cache:
                raw_depth, raw_cloud = blob(depth), blob(cloud)
                require(len(raw_depth) == depth["step"]*height and len(raw_cloud) == cloud["row_step"]*height,
                        "Byte counts do not honor row stride")
                ds = ">" if depth["is_bigendian"] else "<"
                cs = ">" if cloud["is_bigendian"] else "<"
                values = np.ndarray((height, width), dtype=ds+"f4", buffer=raw_depth, strides=(depth["step"], 4))
                fields = {item["name"]: item for item in cloud["fields"]}
                require(all(n in fields and fields[n]["datatype"] == 7 and fields[n]["count"] == 1 for n in ("x", "y", "z", "rgb")),
                        "Actual PointCloud2 fields do not match the native float carriers")
                dtype = np.dtype({"names": ["x", "y", "z"], "formats": [cs+"f4"]*3,
                                  "offsets": [fields[n]["offset"] for n in ("x", "y", "z")],
                                  "itemsize": cloud["point_step"]})
                points = np.ndarray((height, width), dtype=dtype, buffer=raw_cloud,
                                    strides=(cloud["row_step"], cloud["point_step"]))
                valid = np.isfinite(values)
                require(valid.any() and (~valid).any() and np.isnan(values[~valid]).all(), "Expected finite depths and NaN invalid pixels")
                require(bool(np.all(values[valid] > .4)) and bool(np.all(values[valid] < cutoff)),
                        "Actual finite depths violate strict software cutoffs")
                require(np.array_equal(valid, np.isfinite(points["z"]))
                        and np.isnan(points["x"][~valid]).all() and np.isnan(points["y"][~valid]).all(), "Point cloud validity differs from actual depth")
                require(float(np.max(np.abs(values[valid]-points["z"][valid]))) < 1e-6, "Actual point-cloud Z differs from optical depth")
                yy, xx = np.indices((height, width))
                expected_x = values*(xx-(width-1)/2)/focal
                expected_y = values*(yy-(height-1)/2)/focal
                max_error = max(float(np.max(np.abs(points["x"][valid]-expected_x[valid]))),
                                float(np.max(np.abs(points["y"][valid]-expected_y[valid]))))
                require(max_error < 1e-5, "Point cloud does not follow the native helper's pixel-center convention")
                samples = {}
                for label, (u, v) in coordinates.items():
                    z = float(values[v, u])
                    xyz = [float(points[n][v, u]) for n in ("x", "y", "z")]
                    samples[label] = {"pixel": [u, v], "z_m": z if math.isfinite(z) else "nan",
                                      "xyz_m": [a if math.isfinite(a) else "nan" for a in xyz],
                                      "range_m": math.sqrt(sum(a*a for a in xyz)) if math.isfinite(z) else "nan"}
                cache[key] = {"samples": samples, "layout": {"point_step": cloud["point_step"], "row_step": cloud["row_step"], "fields": cloud["fields"]},
                              "helper_max_error_m": max_error, "finite_pixels": int(valid.sum()), "nan_pixels": int((~valid).sum())}
            decoded = cache[key]
            for label, expected in (("foreground", 1.9), ("wall_a", wall_front), ("wall_b", wall_front),
                                    ("near_probe", None), ("far_probe", 4.95 if cutoff == 6 else None), ("no_return", None)):
                measured = decoded["samples"][label]
                if expected is None:
                    require(measured["z_m"] == "nan" and measured["xyz_m"] == ["nan"]*3, "Software cutoff/no-return is not actually NaN")
                else:
                    require(isinstance(measured["z_m"], (int, float)) and abs(measured["z_m"]-expected) < .03,
                            "Actual depth does not match the real surface: "+label)
            require(decoded["samples"]["wall_b"]["range_m"] > decoded["samples"]["wall_b"]["z_m"]+.1,
                    "Off-axis Euclidean range was confused with optical Z")
            final_samples = decoded
        windows[name] = {"triples": len(rows), "span_s": span, "rate_hz": (len(rows)-1)/span,
                         "independent_raw_decoding": final_samples}
    require(set(windows) == {"baseline", "farther", "restored"}, "Missing a real wall-motion phase")
    z0 = windows["baseline"]["independent_raw_decoding"]["samples"]["wall_a"]["z_m"]
    z1 = windows["farther"]["independent_raw_decoding"]["samples"]["wall_a"]["z_m"]
    require(abs(z1-z0-1) < .03, "Actual wall movement did not produce the expected depth change")
    return {"windows": windows, "distinct_raw_depth_cloud_pairs": len(cache)}


def verify_case(session, artifacts, label, width, cutoff, expected=True, diagnosis=None):
    before, wall_before = actual_stats(session), actual_pose(session, "wall")
    world, path = session.workspace/"world.sdf", artifacts/(label+".json")
    original = world.read_bytes()
    answer = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                            cwd=str(session.workspace), env=session.env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=110)
    path.with_suffix(".stdout.log").write_text(answer.stdout, encoding="utf-8")
    require((answer.returncode == 0) is expected, "Unexpected verifier result {}: {}".format(label, answer.stdout[-3000:]))
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "JSON/process results disagree")
    if diagnosis:
        require(diagnosis.lower() in (answer.stdout+json.dumps(observation)).lower(), "Missing specific diagnosis: "+diagnosis)
    checks = inspect_raw(observation, width, cutoff) if expected else None
    after, wall_after = actual_stats(session), actual_pose(session, "wall")
    require(before["paused"] == after["paused"], "Verifier did not restore pause state")
    require(all(abs(a-b) < 1e-6 for a, b in zip(wall_before, wall_after)), "Actual wall pose was not restored")
    require(world.read_bytes() == original, "Verifier rewrote personal input")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Invalid input advanced paused physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected, "raw_report": str(path),
            "independent_raw_checks": checks, "stats_before": before, "stats_after": after,
            "wall_before": wall_before, "wall_after": wall_after,
            "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    candidates = [l for l in lessons if l.key == "classic11-noetic:18"]
    require(len(candidates) == 1, "Expected L18: "+repr(errors))
    lesson = candidates[0]
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "real_gui_each_case": True, "scenarios": []}
    sessions, started = [], time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l18-") as folder:
        try:
            for label, width, cutoff, paused in (("baseline", 320, 6, True), ("resolution_only", 640, 6, False), ("cutoff_only", 320, 4, True)):
                storage = Storage(Path(folder)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":mutate(workspace/"world.sdf", width=width, cutoff=cutoff)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=True, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label, width, cutoff))
                if label == "baseline":
                    session.pause(False)
                    capture = artifacts/"viewer-pseudocolor.png"
                    result = subprocess.run([sys.executable, str(lesson.folder/"lab.py"), "view", "--seconds", "3", "--capture", str(capture)],
                                            cwd=str(workspace), env=session.env, stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, timeout=20)
                    (artifacts/"viewer.stdout.log").write_text(result.stdout, encoding="utf-8")
                    require(result.returncode == 0 and capture.is_file(), "Actual depth viewer failed: "+result.stdout[-1200:])
                    report["live_viewer"] = {"passed": True, "capture": str(capture), "output": result.stdout}
                if label == "cutoff_only":
                    path, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        mutate(path, cutoff=6)
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", width, cutoff, False, "reload"))
                    finally:path.write_bytes(saved)
                    for bad, xpath, value, diagnosis in (
                        ("wrong_frame", PLUGIN+"/frameName", "fake_frame", "frame"),
                        ("wrong_16bit_format", PLUGIN+"/useDepth16UC1Format", "true", "format"),
                        ("wrong_distortion", PLUGIN+"/distortionK1", ".1", "distortion")):
                        try:
                            tree = ET.fromstring(saved);node = tree.find(xpath)
                            require(node is not None, "Missing injection field: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, bad, width, cutoff, False, diagnosis))
                        finally:path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired", width, cutoff))
                session.stop(reason="L18 real graphical scenario completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L18 final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error:report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":sys.exit(main())
