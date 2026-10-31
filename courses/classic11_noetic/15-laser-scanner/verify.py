#!/usr/bin/env python3
"""Move a real wall, read CPU/GPU geometry, and restore the actual input state."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid
from config import prepared
from lab import Client, TOPICS, check_metadata, checks, state_values


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def experiment(result):
    config, record = prepared()  # Input failures must precede ROS initialization/control.
    result.update(config=config, world_sha256=record["world_sha256"], phases={})
    client, original_wall, original_pause = Client(), None, None
    try:
        original_pause = client.physics().pause
        original_wall = client.state()
        result["restore"] = {"wall_before": state_values(original_wall), "pause_before": original_pause}
        require(client.rospy.get_param("/use_sim_time", False), "ROS simulation clock not active")
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(0.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "CPU/GPU scan publisher connection missing")

        def phase(name, distance):
            client.pause(True)
            pose = client.move(distance)
            epoch = client.rospy.Time.now().to_nsec()
            client.drain()
            client.pause(False)
            accepted = {source: [] for source in TOPICS}
            seen = {source: set() for source in TOPICS}
            acquisition_started = {source: False for source in TOPICS}
            deadline = time.monotonic()+14
            while time.monotonic() < deadline:
                for row in client.drain():
                    source, stamp = row["source"], row["stamp_ns"]
                    if stamp <= epoch or stamp in seen[source]:
                        continue
                    seen[source].add(stamp)
                    row["phase"] = name
                    row["disposition"] = "pre_guard"
                    result["trace"].append(row)
                    check_metadata(row, config)
                    if stamp < epoch+200000000:
                        continue
                    evaluations = checks(row, config, distance)
                    valid = all(item["passed"] for item in evaluations)
                    if not valid and not acquisition_started[source]:
                        row["disposition"] = "acquiring"
                        continue
                    require(valid, "Actual "+source+" geometry changed within stable "+name+" window: "+str(evaluations))
                    row["disposition"] = "accepted"
                    acquisition_started[source] = True
                    accepted[source].append(row)
                if all(len(rows) >= 12 and rows[-1]["stamp_ns"]-rows[0]["stamp_ns"] >= 500000000 for rows in accepted.values()):
                    break
                time.sleep(0.02)
            else:
                raise RuntimeError("Fresh CPU/GPU geometric scan window timed out during "+name)
            sensors = {}
            for source, rows in accepted.items():
                stamps = [row["stamp_ns"] for row in rows]
                require(all(right > left for left, right in zip(stamps, stamps[1:])), "Actual scan timestamps are not strictly increasing")
                span = (stamps[-1]-stamps[0])/1e9
                rate = (len(stamps)-1)/span
                require(15 <= rate <= 25, "Actual "+source+" message frequency differs from20Hz in simulation time")
                sensors[source] = {"message_count": len(rows), "span_s": span, "rate_hz": rate,
                                   "stamp_ns": stamps, "checks": checks(rows[-1], config, distance)}
            return {"wall_distance_m": distance, "wall_center_x_m": distance, "wall_front_x_m": distance-0.1,
                    "wall_pose": pose, "control_epoch_ns": epoch, "sensors": sensors}

        for name, distance in (("baseline", config["wall_distance_m"]), ("moved", config["wall_distance_m"]+1),
                               ("restored", config["wall_distance_m"])):
            result["phases"][name] = phase(name, distance)
    finally:
        try:
            if original_wall is not None:
                client.pause(True)
                client.apply(original_wall)
                actual = state_values(client.state())
                expected = state_values(original_wall)
                error = max(abs(left-right) for field in expected for left, right in zip(expected[field], actual[field]))
                result["restore"]["wall_after"] = actual
                result["restore"]["wall_max_error"] = error
                require(error < 1e-7, "Original wall pose/twist restoration failed")
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
    report_path = args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("laser-verify-"+uuid.uuid4().hex+".json")
    result = {"passed": False, "trace": [], "trace_path": str(report_path.resolve())}
    start = time.monotonic()
    try:
        experiment(result)
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-start
    report_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = report_path.with_name(report_path.name+".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(report_path))
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
