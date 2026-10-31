#!/usr/bin/env python3
"""RGB acceptance evaluates actual rendered bytes and exact-stamp calibration.

Three fresh graphical sessions isolate resolution and FOV changes. Independent
pixel classification, geometry, raw SHA, state restoration, and rejected stale
inputs are checked alongside the canonical course verifier.
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


def require(value, message):
    if not value:
        raise AssertionError(message)


def hashes(folder):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(folder.iterdir()) if p.is_file()}


def mutate(path, width=None, fov=None):
    tree = ET.parse(str(path))
    camera = tree.find(CAMERA)
    require(camera is not None, "Missing fixed camera")
    if width is not None:
        camera.find("image/width").text = str(width)
        camera.find("image/height").text = str(width*3//4)
    if fov is not None:
        camera.find("horizontal_fov").text = repr(math.radians(fov))
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def inspect_raw(observation, width, fov):
    import numpy as np
    height = width*3//4
    # Classic Camera::SetHFOV -> sdformat9 Param::Set serializes the double
    # with a default stringstream (six significant digits). Validate that
    # concrete native getter convention without changing published calibration.
    render_hfov = float(format(math.radians(fov), ".6g"))
    focal = width/(2*math.tan(render_hfov/2))
    cx, cy = (width+1)/2, (height+1)/2
    cache, results = {}, {}
    for name, phase in observation["phases"].items():
        rows = [r for r in observation["trace"] if r["phase"] == name and r["disposition"] == "accepted"]
        stamps = [row["stamp_ns"] for row in rows]
        require(len(rows) >= 12 and all(b > a for a, b in zip(stamps, stamps[1:])),
                "Actual image windows need twelve distinct increasing headers")
        span = (stamps[-1]-stamps[0])/1e9
        require(span >= .5 and 15 <= (len(rows)-1)/span <= 25, "Actual paired image frequency/duration invalid")
        x, y, z = phase["target_pose"]["position"]
        corners = [(cx-focal*(y+dy)/(x+dx), cy-focal*(z+dz-1)/(x+dx))
                   for dx in (-.02, .02) for dy in (-.3, .3) for dz in (-.3, .3)]
        predicted = [min(p[0] for p in corners), min(p[1] for p in corners),
                     max(p[0] for p in corners), max(p[1] for p in corners)]
        counts, boxes = [], []
        for row in rows:
            image, info = row["image"], row["camera_info"]
            require(image["stamp_ns"] == info["stamp_ns"] == row["stamp_ns"], "Image/Info pair timestamps differ")
            require(image["width"] == info["width"] == width and image["height"] == info["height"] == height,
                    "Rendered dimensions/calibration differ from the active input")
            require(image["encoding"] == "rgb8" and image["step"] == width*3 and image["is_bigendian"] == 0,
                    "Actual raw image byte layout differs from declared RGB8")
            require(image["frame"] == info["frame"] == "camera_optical_frame", "Optical-frame label differs")
            k_expected = [focal, 0, cx, 0, focal, cy, 0, 0, 1]
            p_expected = [focal, 0, cx, 0, 0, focal, cy, 0, 0, 0, 1, 0]
            require(len(info["K"]) == 9 and all(math.isfinite(v) for v in info["K"])
                    and max(abs(a-b) for a, b in zip(info["K"], k_expected)) < 1e-5,
                    "Actual K differs from configured focal/principal-point convention")
            require(len(info["P"]) == 12 and all(math.isfinite(v) for v in info["P"])
                    and max(abs(a-b) for a, b in zip(info["P"], p_expected)) < 1e-5,
                    "Actual monocular projection matrix differs")
            require(info["R"] == [1, 0, 0, 0, 1, 0, 0, 0, 1] and info["D"] == [0]*5,
                    "Rectification/distortion matrices differ from the fixed zero-distortion fixture")
            digest = image["data_sha256"]
            if digest not in cache:
                data = Path(image["data_path"]).read_bytes()
                require(hashlib.sha256(data).hexdigest() == digest and len(data) == height*width*3,
                        "Saved raw rendered image hash/size mismatch")
                rgb = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
                # Independent conservative primary-red mask; it uses real bytes,
                # not the verifier's stored projection or pixel measurements.
                yy, xx = np.where((rgb[:, :, 0] > 200) & (rgb[:, :, 1] < 80) & (rgb[:, :, 2] < 80))
                require(len(xx) > 100, "Actual RGB bytes contain no visible red teaching target")
                cache[digest] = {"count": len(xx), "bbox": [int(xx.min()), int(yy.min()), int(xx.max()), int(yy.max())]}
            measurement = cache[digest]
            box = measurement["bbox"]
            require(max(abs((box[i]+box[i+2])/2-(predicted[i]+predicted[i+2])/2) for i in (0, 1)) < 3,
                    "Rendered target position disagrees with independent 8-corner projection")
            require(max(abs((box[i+2]-box[i]+1)-(predicted[i+2]-predicted[i])) for i in (0, 1)) < 3.5,
                    "Rendered target extent disagrees with actual geometry")
            counts.append(measurement["count"])
            boxes.append(box)
        results[name] = {"paired_images": len(rows), "span_s": span, "rate_hz": (len(rows)-1)/span,
                         "mean_red_pixels": sum(counts)/len(counts), "last_actual_bbox": boxes[-1],
                         "independent_projected_bbox": predicted}
    base, far, lateral, restored = (results[key] for key in ("baseline", "farther", "lateral", "restored"))
    require(.45 < far["mean_red_pixels"]/base["mean_red_pixels"] < .68, "Moving the target farther did not reduce its actual image area")
    require(lateral["last_actual_bbox"][0] < base["last_actual_bbox"][0]-20,
            "Positive world Y did not move the rendered target toward negative optical X")
    require(abs(restored["mean_red_pixels"]/base["mean_red_pixels"]-1) < .02, "Restored target image was not recovered")
    return {"windows": results, "distinct_raw_images_checked": len(cache)}


def verify_case(session, artifacts, label, width, fov, expected=True, diagnosis=None):
    before, pose_before = actual_stats(session), actual_pose(session, "target_red")
    world = session.workspace/"world.sdf"
    source = world.read_bytes()
    path = artifacts/(label+".json")
    answer = subprocess.run([sys.executable, str(session.lesson.folder/"verify.py"), "--report", str(path)],
                            cwd=str(session.workspace), env=session.env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=95)
    path.with_suffix(".stdout.log").write_text(answer.stdout, encoding="utf-8")
    require((answer.returncode == 0) is expected, "Unexpected verifier result {}: {}".format(label, answer.stdout[-3500:]))
    observation = json.loads(path.read_text(encoding="utf-8"))
    require(observation.get("passed") is expected, "JSON/process result differs")
    if diagnosis:
        require(diagnosis.lower() in (answer.stdout+json.dumps(observation)).lower(), "Missing diagnosis: "+diagnosis)
    checks = inspect_raw(observation, width, fov) if expected else None
    after, pose_after = actual_stats(session), actual_pose(session, "target_red")
    require(before["paused"] == after["paused"], "Verifier did not restore pause state")
    require(all(abs(a-b) < 1e-6 for a, b in zip(pose_before, pose_after)), "Target pose was not restored")
    require(world.read_bytes() == source, "Verifier modified personal input")
    if not expected and before["paused"]:
        require(before["iterations"] == after["iterations"], "Rejected input still advanced paused physics")
    print(json.dumps({"scenario": label, "passed": True, "expected_acceptance": expected}), flush=True)
    return {"name": label, "passed": True, "expected_acceptance": expected, "raw_report": str(path),
            "independent_raw_checks": checks, "stats_before": before, "stats_after": after,
            "target_before": pose_before, "target_after": pose_after,
            "observation": {k: v for k, v in observation.items() if k != "trace"}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()
    output = Path(args.report).expanduser().resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    candidates = [lesson for lesson in lessons if lesson.key == "classic11-noetic:17"]
    require(len(candidates) == 1, "Expected L17: "+repr(errors))
    lesson = candidates[0]
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source,
              "real_gui_each_case": True, "scenarios": []}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l17-") as folder:
        try:
            for label, width, fov, paused in (("baseline", 320, 60, True), ("resolution_only", 640, 60, False), ("fov_only", 320, 75, True)):
                storage = Storage(Path(folder)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    mutate(workspace/"world.sdf", width=width, fov=fov)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=True, terminals=False)
                session.pause(paused)
                report["scenarios"].append(verify_case(session, artifacts, label, width, fov))
                if label == "baseline":
                    session.pause(False)
                    capture = artifacts/"viewer-native.png"
                    run = subprocess.run([sys.executable, str(lesson.folder/"lab.py"), "view", "--seconds", "3", "--capture", str(capture)],
                                         cwd=str(workspace), env=session.env, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, timeout=20)
                    (artifacts/"viewer.stdout.log").write_text(run.stdout, encoding="utf-8")
                    require(run.returncode == 0 and capture.is_file() and capture.stat().st_size > 100,
                            "Actual finite desktop image viewer failed: "+run.stdout[-1500:])
                    report["live_viewer"] = {"passed": True, "capture": str(capture), "output": run.stdout}
                if label == "fov_only":
                    path, saved = workspace/"world.sdf", (workspace/"world.sdf").read_bytes()
                    try:
                        mutate(path, fov=60)
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", width, fov, False, "reload"))
                    finally:
                        path.write_bytes(saved)
                    for bad, xpath, value, diagnosis in (
                        ("wrong_frame", "./world/model[@name='camera_rig']/link/sensor/plugin/frameName", "fake_frame", "frame"),
                        ("wrong_format", CAMERA+"/image/format", "L8", "format"),
                        ("wrong_distortion", "./world/model[@name='camera_rig']/link/sensor/plugin/distortionK1", ".1", "distortion")):
                        try:
                            tree = ET.fromstring(saved)
                            node = tree.find(xpath)
                            require(node is not None, "Missing fault injection field: "+xpath)
                            node.text = value
                            path.write_bytes(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
                            report["scenarios"].append(verify_case(session, artifacts, bad, width, fov, False, diagnosis))
                        finally:
                            path.write_bytes(saved)
                    report["scenarios"].append(verify_case(session, artifacts, "exact_source_repaired", width, fov))
                session.stop(reason="L17 actual graphical scenario completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L17 acceptance final cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {s.token: live_token_processes(s.token) for _, s in sessions}
            report["source_templates_unchanged"] = hashes(lesson.folder) == source
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
