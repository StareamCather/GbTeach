#!/usr/bin/env python3
"""Validate real rendered pixels and paired native camera calibration messages."""
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
from lab import Client, check_pair, pixels, projection, save_png, state_values


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def pixel_checks(pair, observed, predicted):
    red, green, blue = (observed[color] for color in ("red", "green", "blue"))
    if red["count"] <= 100 or green["count"] <= 25 or blue["count"] <= 25:
        return False
    left, top, right, bottom = red["bbox"]
    center, extent = [(left+right)/2, (top+bottom)/2], [right-left+1, bottom-top+1]
    if any(abs(actual-expected) > 2.5 for actual, expected in zip(center, predicted["bbox_center"])):
        return False
    if any(abs(actual-expected) > 3 for actual, expected in zip(extent, predicted["bbox_extent"])):
        return False
    if any(abs(actual-expected) > 3 for actual, expected in zip(red["centroid"], predicted["bbox_center"])):
        return False
    info = pair["camera_info"]
    return green["centroid"][0] > info["K"][2]+20 and blue["centroid"][1] < info["K"][5]-20


def rotation_error(left, right):
    dot = abs(sum(a*b for a, b in zip(left, right)))
    dot /= math.sqrt(sum(a*a for a in left)*sum(b*b for b in right))
    return 2*math.acos(min(1, max(0, dot)))


def experiment(result, assets):
    config, record = prepared()  # No ROS or controls before input-fingerprint checks.
    result.update(config=config, world_sha256=record["world_sha256"], phases={})
    client, original, original_pause = Client(), None, None
    try:
        original_pause = client.physics().pause
        result["restore"] = {"pause_before": original_pause}
        original = client.state()
        result["restore"]["target_before"] = state_values(original)
        require(client.rospy.get_param("/use_sim_time", False), "ROS simulation clock is not active")
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "Actual Image and CameraInfo publisher connections are both required")

        def phase(name, x, y):
            client.pause(True)
            pose = client.move(x, y)
            epoch = client.rospy.Time.now().to_nsec()
            client.drain()
            client.pause(False)
            accepted, seen, acquired, last_pair = [], set(), False, None
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                for pair in client.drain():
                    stamp = pair["stamp_ns"]
                    if stamp <= epoch or stamp in seen:
                        continue
                    seen.add(stamp)
                    check_pair(pair, config)
                    observed, predicted = pixels(pair), projection(pair["camera_info"], x, y)
                    payload = pair["data"]
                    digest = hashlib.sha256(payload).hexdigest()
                    data_path = assets/(digest+".rgb")
                    if not data_path.exists():
                        data_path.write_bytes(payload)
                    row = {key: value for key, value in pair.items() if key != "data"}
                    row["image"] = dict(row["image"], data_path=str(data_path), data_sha256=digest)
                    row.update(phase=name, disposition="guard", pixels=observed, projection=predicted)
                    result["trace"].append(row)
                    if stamp < epoch+200000000:
                        continue
                    valid = pixel_checks(pair, observed, predicted)
                    if not acquired and not valid:
                        row["disposition"] = "acquiring"
                        continue
                    require(valid, "Actual rendered pixels disagree with calibrated eight-corner projection during "+name+": "+str({"pixels": observed, "projection": predicted}))
                    row["disposition"] = "accepted"
                    acquired = True
                    accepted.append(row)
                    last_pair = pair
                if len(accepted) >= 12 and accepted[-1]["stamp_ns"]-accepted[0]["stamp_ns"] >= 500000000:
                    break
                time.sleep(.01)
            else:
                raise RuntimeError("Fresh paired actual RGB/projection window timed out during "+name)
            client.pause(True)
            stamps = [row["stamp_ns"] for row in accepted]
            require(all(right > left for left, right in zip(stamps, stamps[1:])), "Paired native camera timestamps are not increasing")
            span = (stamps[-1]-stamps[0])/1e9
            rate = (len(stamps)-1)/span
            require(15 <= rate <= 25, "Actual paired camera frequency differs from20Hz in simulation time")
            png = assets/(name+".png")
            save_png(last_pair, png)
            return {"target_pose": pose, "control_epoch_ns": epoch, "message_count": len(accepted),
                    "stamp_ns": stamps, "span_s": span, "rate_hz": rate, "png_path": str(png),
                    "observations": {"mean_red_count": statistics.mean(row["pixels"]["red"]["count"] for row in accepted),
                        "mean_red_centroid": [statistics.mean(row["pixels"]["red"]["centroid"][axis] for row in accepted) for axis in range(2)],
                        "mean_bbox_center": [statistics.mean((row["pixels"]["red"]["bbox"][axis]+row["pixels"]["red"]["bbox"][axis+2])/2 for row in accepted) for axis in range(2)],
                        "expected_projection": accepted[-1]["projection"]}}

        for name, x, y in (("baseline", 3, 0), ("farther", 4, 0), ("lateral", 3, .5), ("restored", 3, 0)):
            result["phases"][name] = phase(name, x, y)
        baseline = result["phases"]["baseline"]["observations"]
        farther = result["phases"]["farther"]["observations"]
        lateral = result["phases"]["lateral"]["observations"]
        restored = result["phases"]["restored"]["observations"]
        ratio = farther["mean_red_count"]/baseline["mean_red_count"]
        expected_ratio = (2.98/3.98)**2
        require(abs(ratio-expected_ratio) < .07, "Actual target pixel area did not shrink according to range")
        require(lateral["mean_red_centroid"][0] < baseline["mean_red_centroid"][0]-20,
                "World +Y target displacement did not move rendered pixels left")
        require(abs(restored["mean_red_count"]-baseline["mean_red_count"]) <= 2,
                "Restored target did not recover actual baseline pixel area")
        result["observations"] = {"farther_area_ratio": ratio, "expected_front_face_area_ratio": expected_ratio,
            "world_positive_y_pixel_delta_u": lateral["mean_red_centroid"][0]-baseline["mean_red_centroid"][0],
            "note": "Projection uses all eight corners; legacy CameraInfo principal point and raster pixel centers may differ by about one pixel."}
    finally:
        try:
            if original is not None:
                client.pause(True)
                client.apply(original)
                actual, expected = state_values(client.state()), state_values(original)
                scalar = max(abs(a-b) for key in ("position", "linear", "angular") for a, b in zip(actual[key], expected[key]))
                rotation = rotation_error(actual["orientation"], expected["orientation"])
                result["restore"].update(target_after=actual, position_twist_error=scalar, rotation_error_rad=rotation)
                require(scalar < 1e-7 and rotation < 1e-6, "Original target pose/twist restoration failed")
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
    path = args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("rgb-verify-"+uuid.uuid4().hex+".json")
    path = path.resolve()
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
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
