#!/usr/bin/env python3
"""Audit real clock domains, paused stepping, ROS receipt, and isolated restarts."""
import argparse
import json
import os
from pathlib import Path
import time
import uuid
from config import prepared, digest
from lab import Client, cancellation, check_metadata, collect, compare_children, native_command, ns, require, save


def clock(snapshot):
    value = snapshot["world_statistics"]["native"]
    return {"sim_time_ns": ns(value["sim_time"]), "iterations": int(value["iterations"]),
            "paused": value["paused"], "world_real_time_ns": ns(value["real_time"]),
            "pause_time_ns": ns(value["pause_time"])}


def experiment(result, directory):
    config, record = prepared()  # Refuse modified/saved-but-unloaded files before controls.
    result.update(config=config, world_sha256=record["world_sha256"], source_sha256=record["source_sha256"],
                  observer_sha256=record["observer_sha256"], metadata_library_sha256=record["metadata_library_sha256"],
                  phases={}, restore={}, children=[])
    client, original = Client(record, directory/"main"), None
    try:
        original = client.physics().pause
        result["restore"]["pause_before"] = original
        client.pause(False)
        window = collect(client, config, "continuous", result["trace"], min_sim=2, nominal=True, ros=True)
        require(window["server_metadata"], "Actual server metadata was not observed")
        for row in window["server_metadata"]:
            check_metadata(row, config)
        result["phases"]["continuous"] = window
        client.pause(True)
        before = client.snapshot(True)
        start = time.monotonic_ns()
        time.sleep(.35)
        after = client.snapshot(True)
        b, a = clock(before), clock(after)
        require(b["sim_time_ns"] == a["sim_time_ns"] and b["iterations"] == a["iterations"], "Paused world progressed without a step")
        require(before["ros_service_world_sim_time_ns"] == after["ros_service_world_sim_time_ns"], "ROS service world-time readback progressed while paused")
        result["phases"]["pause_hold"] = {"before": before, "after": after, "wall_span_s": (time.monotonic_ns()-start)/1e9,
            "note": "Python CLOCK_MONOTONIC includes the pause; Gazebo World.RealTime excludes accumulated pause."}

        def step(name, count):
            before = client.snapshot(True)
            command = native_command(os.environ.copy(), "-s" if count == 1 else "-m", *([] if count == 1 else [str(count)]))
            deadline = time.monotonic()+5
            after = None
            while time.monotonic() < deadline:
                after = client.snapshot(True)
                if clock(after)["iterations"] >= clock(before)["iterations"]+count:
                    break
            b, a = clock(before), clock(after)
            require(a["iterations"]-b["iterations"] == count, "Actual native iteration delta differs from requested steps")
            wanted = round(count*config["physics_step_s"]*1e9)
            require(a["sim_time_ns"]-b["sim_time_ns"] == wanted, "Single/multi-step did not advance exact physics ticks")
            require(after["ros_service_world_sim_time_ns"]-before["ros_service_world_sim_time_ns"] == wanted, "ROS service world-time readback differs from native step delta")
            require(after["ros_service_paused"] is True and a["paused"] is True, "Stepping must retain paused state")
            return {"step_count": count, "before": before, "after": after, "command": command,
                    "note": "A physics step need not create a sample in every sensor; configured sensor periods remain distinct."}

        result["phases"]["single_step"] = step("single_step", 1)
        result["phases"]["multi_step"] = step("multi_step", round(.1/config["physics_step_s"]))
        compare_children(config, record, directory/"comparison", result["trace"], result["children"])
        require(digest(Path.cwd()/"world.sdf") == record["world_sha256"], "Child benchmark changed the personal source world")
        result["observations"] = {"main_ros_pipeline": "Native sensor -> Gazebo Transport -> SDK observer -> Python ROS publisher -> actual ROS subscriber. All clocks remain separate.",
            "lockstep": "Actual separate server starts; only argv --lockstep differs between children with identical unlimited-rate benchmark world.",
            "performance_metrics": "Unstamped diagnostic values preserved without treating their instantaneous rates as sample timestamps.",
            "comparison": "No assertion that off must lose frames, that on is faster, or that ROS delivery is hardware synchronized."}
    finally:
        try:
            if original is not None:
                client.pause(original)
                result["restore"]["pause_after"] = client.physics().pause
                require(result["restore"]["pause_after"] == original, "Original pause state was not restored")
        finally:
            result["restore"]["observer_cleanup"] = client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    session = Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))
    directory = session/("synchronization-"+uuid.uuid4().hex)
    path = (args.report or directory/"verify.json").resolve()
    result = {"passed": False, "trace": [], "trace_path": str(path)}
    started = time.monotonic()
    try:
        with cancellation():
            experiment(result, directory)
            result["passed"] = True
    except Exception as error:
        result.update(passed=False, error=type(error).__name__+": "+str(error))
    result["wall_seconds"] = time.monotonic()-started
    save(path, result)
    summary = {key: value for key, value in result.items() if key != "trace"}
    # Detailed native diagnostics remain in the report; stdout stays reviewable.
    for phase in summary.get("phases", {}).values():
        if "performance_metrics" in phase:
            phase["performance_metrics_count"] = len(phase.pop("performance_metrics"))
            phase["server_metadata_count"] = len(phase.pop("server_metadata"))
            phase["world_statistics_count"] = len(phase.pop("world_statistics"))
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
