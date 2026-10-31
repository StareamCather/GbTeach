#!/usr/bin/env python3
"""Retain and verify actual depth/cloud byte buffers during real wall motion."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
import uuid
from config import prepared
from lab import Client, measure, rgb_summary, save_depth_png, state_values


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def retain(meta, payload, suffix, assets):
    digest = hashlib.sha256(payload).hexdigest()
    path = assets/(digest+suffix)
    if not path.exists():
        path.write_bytes(payload)
    return dict(meta, data_path=str(path), data_sha256=digest)


def rotation_error(left, right):
    dot = abs(sum(a*b for a, b in zip(left, right)))
    dot /= math.sqrt(sum(a*a for a in left)*sum(b*b for b in right))
    return 2*math.acos(min(1, max(0, dot)))


def save_rgb(actual, path):
    from PyQt5.QtGui import QImage
    meta = actual["rgb"]
    image = QImage(actual["rgb_data"], meta["width"], meta["height"], meta["step"], QImage.Format_RGB888).copy()
    if not image.save(str(path), "PNG"):
        raise RuntimeError("Could not retain actual auxiliary RGB image")


def experiment(result, assets):
    config, record = prepared()  # Refuse stale/invalid inputs before any ROS/control.
    result.update(config=config, world_sha256=record["world_sha256"], phases={})
    client, original, original_pause = Client(), None, None
    try:
        original_pause = client.physics().pause
        result["restore"] = {"pause_before": original_pause}
        original = client.state()
        result["restore"]["wall_before"] = state_values(original)
        require(client.rospy.get_param("/use_sim_time", False), "ROS simulation clock is not active")
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "Actual depth/info/cloud/RGB publisher connections are required")

        def phase(name, distance):
            client.pause(True)
            pose = client.move(distance)
            epoch = client.rospy.Time.now().to_nsec()
            client.drain()
            client.pause(False)
            accepted, seen, acquired, last_bundle = [], set(), False, None
            deadline = time.monotonic()+22
            last_error = None
            while time.monotonic() < deadline:
                for bundle in client.drain():
                    stamp = bundle["stamp_ns"]
                    if stamp <= epoch or stamp in seen:
                        continue
                    seen.add(stamp)
                    row = {key: value for key, value in bundle.items() if not key.endswith("_data")}
                    for kind, extension in (("depth", ".f32"), ("cloud", ".pc2"), ("rgb", ".rgb")):
                        if kind+"_data" in bundle:
                            row[kind] = retain(bundle[kind], bundle[kind+"_data"], extension, assets)
                    row.update(phase=name, disposition="guard")
                    result["trace"].append(row)
                    if stamp < epoch+200000000:
                        continue
                    try:
                        observed = measure(bundle, config, distance)
                    except RuntimeError as error:
                        last_error = str(error)
                        row["measurement_error"] = last_error
                        if acquired:
                            raise
                        row["disposition"] = "acquiring"
                        continue
                    row.update(disposition="accepted", observations=observed, intrinsics_check=bundle["intrinsics_check"])
                    acquired = True
                    accepted.append(row)
                    last_bundle = bundle
                if len(accepted) >= 12 and accepted[-1]["stamp_ns"]-accepted[0]["stamp_ns"] >= 1000000000:
                    break
                time.sleep(.01)
            else:
                raise RuntimeError("Actual depth/cloud geometric window timed out during "+name+": "+str(last_error))
            client.pause(True)
            stamps = [row["stamp_ns"] for row in accepted]
            require(all(b > a for a, b in zip(stamps, stamps[1:])), "Native depth/cloud triple headers are not increasing")
            span = (stamps[-1]-stamps[0])/1e9
            rate = (len(stamps)-1)/span
            require(8 <= rate <= 12, "Actual exact-stamped depth/cloud frequency differs from native10Hz")
            png = assets/(name+"-depth-pseudocolor.png")
            save_depth_png(last_bundle, config, png)
            actual_rgb = client.latest_rgb()
            require(actual_rgb is not None and actual_rgb["rgb"]["stamp_ns"] >= epoch+200000000,
                    "No fresh actual RGB evidence of near/far target visibility")
            at_pixels = rgb_summary(actual_rgb)
            near, far = at_pixels["near_probe"], at_pixels["far_probe"]
            require(near[1] > 160 and near[1] > 2*max(near[0], near[2]), "Software-near rejection lacks an actually visible green probe")
            require(far[2] > 160 and far[2] > 2*max(far[0], far[1]), "Far-cutoff interpretation lacks an actually visible blue probe")
            rgb_meta = retain(actual_rgb["rgb"], actual_rgb["rgb_data"], ".rgb", assets)
            rgb_png = assets/(name+"-actual-rgb.png")
            save_rgb(actual_rgb, rgb_png)
            return {"wall_pose": pose, "control_epoch_ns": epoch, "message_count": len(accepted), "stamp_ns": stamps,
                    "span_s": span, "rate_hz": rate, "png_path": str(png),
                    "visualization": {"kind": "depth_pseudocolor", "near_m": config["min_depth_m"], "far_m": config["max_depth_m"],
                        "mapping": "R=255(1-t), G=255(1-|2t-1|), B=255t; t=clamp((Z-near)/(far-near),0,1)", "invalid_rgb": [255, 0, 255]},
                    "rgb_evidence": {"image": rgb_meta, "at_sample_pixels": at_pixels, "png_path": str(rgb_png),
                        "note": "Independent current RGB image; cloud packed RGB may use an earlier cached exposure and stock byte packing."},
                    "observations": {"samples": accepted[-1]["observations"]["samples"],
                        "mean_wall_a_z_m": statistics.mean(row["observations"]["samples"]["wall_a"]["depth_m"] for row in accepted),
                        "mean_wall_b_z_m": statistics.mean(row["observations"]["samples"]["wall_b"]["depth_m"] for row in accepted),
                        "mean_foreground_z_m": statistics.mean(row["observations"]["samples"]["foreground"]["depth_m"] for row in accepted),
                        "max_cloud_helper_error_m": max(row["observations"]["cloud_helper_max_error_m"] for row in accepted)}}

        for name, distance in (("baseline", 3), ("farther", 4), ("restored", 3)):
            result["phases"][name] = phase(name, distance)
        phases = result["phases"]
        for sample in ("wall_a", "wall_b"):
            key = "mean_"+sample+"_z_m"
            change = phases["farther"]["observations"][key]-phases["baseline"]["observations"][key]
            require(abs(change-1) < .03, "Actual wall translation did not produce one meter of axial-depth change")
            require(abs(phases["restored"]["observations"][key]-phases["baseline"]["observations"][key]) < .01,
                    "Restored wall did not recover actual depth")
        result["observations"] = {"wall_depth_change_m": phases["farther"]["observations"]["mean_wall_a_z_m"]-phases["baseline"]["observations"]["mean_wall_a_z_m"],
            "invalid_policy": "This stock Openni ROS path maps rejected software-cutoff and no-return depth/XYZ to NaN; original bytes retained.",
            "pointcloud_rgb_policy": "Inspect original field bytes; do not assume same-exposure standard packed true color."}
    finally:
        try:
            if original is not None:
                client.pause(True)
                client.apply(original)
                actual, expected = state_values(client.state()), state_values(original)
                scalar = max(abs(a-b) for key in ("position", "linear", "angular") for a, b in zip(actual[key], expected[key]))
                rotation = rotation_error(actual["orientation"], expected["orientation"])
                result["restore"].update(wall_after=actual, position_twist_error=scalar, rotation_error_rad=rotation)
                require(scalar < 1e-7 and rotation < 1e-6, "Original wall pose/twist restoration failed")
        finally:
            try:
                if original_pause is not None:
                    client.pause(original_pause)
                    result["restore"]["pause_after"] = client.physics().pause
            finally:
                client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    path = (args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("depth-verify-"+uuid.uuid4().hex+".json")).resolve()
    assets = path.with_name(path.stem+"-assets")
    assets.mkdir(parents=True, exist_ok=True)
    result = {"passed": False, "trace": [], "trace_path": str(path), "assets_dir": str(assets)}
    started = time.monotonic()
    try:
        experiment(result, assets)
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-started
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
