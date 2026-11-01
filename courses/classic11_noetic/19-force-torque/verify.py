#!/usr/bin/env python3
"""Observe actual fixed-joint equilibrium and four signed wrench channels."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
import uuid
from config import RIGS, prepared
from lab import CHANNELS, Client, check_properties, check_row, check_truth, expected


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def vector_stat(rows, key, function):
    return [function(row[key][axis] for row in rows) for axis in range(3)]


def experiment(result):
    config, record = prepared()  # Reject invalid/stale input before ROS/control.
    result.update(config=config, world_sha256=record["world_sha256"], channels={}, truth=[], stock_truth=[],
                  native_bridge_source_sha256=record["native_bridge_source_sha256"],
                  native_bridge_library_sha256=record["native_bridge_library_sha256"])
    client, original_pause = Client(), None
    try:
        original_pause = client.physics().pause
        result["restore"] = {"pause_before": original_pause}
        result["rig_properties"] = {name: check_properties(client.properties(name), config) for name in RIGS}
        result["actual_properties"] = result["rig_properties"]["native"]
        result["stock_actual_properties"] = result["rig_properties"]["stock"]
        result["rig_geometry"] = {name: client.geometry(name, config) for name in RIGS}
        gravity = client.physics().gravity
        require(math.sqrt(gravity.x**2+gravity.y**2+(gravity.z+9.81)**2) < 1e-9,
                "Actual world gravity differs from wrench prediction")
        initial_truth = {name: client.truth(name) for name in RIGS}
        for name, value in initial_truth.items():check_truth(value, config, name)
        result["initial_truth"] = initial_truth['native']
        result["stock_initial_truth"] = initial_truth['stock']
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "Four actual wrench publisher connections are required")
        # The world-properties service remains fresh even if this new ROS client
        # has not received /clock while the existing world was already paused.
        epoch = client.sim_time_ns()
        result["control_epoch_ns"] = epoch
        client.drain()
        client.pause(False)
        rows = {source: [] for source in CHANNELS}
        seen = {source: set() for source in CHANNELS}
        deadline, next_truth = time.monotonic()+12, time.monotonic()
        while time.monotonic() < deadline:
            for row in client.drain():
                source, stamp = row["source"], row["stamp_ns"]
                if stamp <= epoch or stamp in seen[source]:
                    continue
                seen[source].add(stamp)
                row.update(phase="steady", disposition="guard")
                result["trace"].append(row)
                if stamp < epoch+200000000:
                    continue
                check_row(row, config)
                row["disposition"] = "accepted"
                rows[source].append(row)
            if time.monotonic() >= next_truth:
                for name, key in (("native", "truth"), ("stock", "stock_truth")):
                    actual = client.truth(name)
                    check_truth(actual, config, name)
                    result[key].append(actual)
                next_truth = time.monotonic()+.045
            if all(len(values) >= 30 and values[-1]["stamp_ns"]-values[0]["stamp_ns"] >= 600000000 for values in rows.values()):
                break
            time.sleep(.01)
        else:
            raise RuntimeError("Four fresh equilibrium wrench windows timed out")
        wanted = expected(config)
        for source, values in rows.items():
            stamps = [row["stamp_ns"] for row in values]
            require(all(right > left for left, right in zip(stamps, stamps[1:])), "Actual wrench headers are not strictly increasing")
            span = (stamps[-1]-stamps[0])/1e9
            rate = (len(stamps)-1)/span
            require(40 <= rate <= 60, "Actual "+source+" rate differs from nominal50Hz")
            intervals = [(right-left)/1e9 for left, right in zip(stamps, stamps[1:])]
            result["channels"][source] = {"message_count": len(values), "stamp_ns": stamps,
                "span_s": span, "rate_hz": rate, "expected": wanted[source],
                **{key+"_"+stat: vector_stat(values, key, fn) for key in ("force", "torque")
                   for stat, fn in (("min", min), ("max", max), ("median", statistics.median))},
                "mean_force": vector_stat(values, "force", statistics.mean),
                "mean_torque": vector_stat(values, "torque", statistics.mean),
                "dt_min_s": min(intervals), "dt_max_s": max(intervals)}
        overlap_start = max(values[0]["stamp_ns"] for values in rows.values())
        overlap_end = min(values[-1]["stamp_ns"] for values in rows.values())
        require(overlap_end-overlap_start >= 500000000, "Independent wrench windows lack a common equilibrium interval")
        actual_in_window = [item for item in result["truth"] if overlap_start <= (item["clock_before_ns"]+item["clock_after_ns"])/2 <= overlap_end]
        require(len(actual_in_window) >= 3, "Need three actual link-state queries during the shared equilibrium interval")
        stock_in_window = [item for item in result["stock_truth"] if overlap_start <= (item["clock_before_ns"]+item["clock_after_ns"])/2 <= overlap_end]
        require(len(stock_in_window) >= 3, "Need three actual stock link-state queries during the same shared equilibrium interval")
        child = result["channels"]["native_child"]
        stock = result["channels"]["stock_child"]
        require(max(abs(a-b) for key in ("mean_force", "mean_torque") for a, b in zip(child[key], stock[key])) < .03,
                "Stock ModelPlugin and native child-to-parent sensor window means disagree")
        result["observations"] = {"common_window_span_s": (overlap_end-overlap_start)/1e9,
            "actual_equilibrium_queries_in_window": len(actual_in_window),
            "actual_stock_equilibrium_queries_in_window": len(stock_in_window),
            "weight_n": config["mass_kg"]*9.81, "lever_moment_nm": config["mass_kg"]*9.81*config["arm_m"],
            "measurement_point": "Separate joint anchors coincide with each rig child origin; native=(0,0,1.2), stock=(0,1,1.2), parallel world-aligned axes",
            "native_bridge": "Actual native ForceTorqueSensor WrenchStamped copied unchanged to ROS; no applied force/pose/trajectory",
            "stock_source": "Installed ModelPlugin reads stock_mount_joint body2 feedback on a separate physically identical stock_weighing_rig; child_to_parent in stock_load_link axes",
            "synchronization": "Distinct native/stock header windows; no claim of cross-channel exact-stamp sampling"}
    finally:
        try:
            if original_pause is not None:
                client.pause(original_pause)
                result["restore"]["pause_after"] = client.physics().pause
                for name, key in (("native", "final_truth"), ("stock", "stock_final_truth")):
                    result[key] = client.truth(name)
                    check_truth(result[key], config, name)
        finally:
            client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    path = (args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("wrench-verify-"+uuid.uuid4().hex+".json")).resolve()
    result = {"passed": False, "trace": [], "trace_path": str(path)}
    started = time.monotonic()
    try:
        experiment(result)
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
