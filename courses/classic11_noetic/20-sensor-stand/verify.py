#!/usr/bin/env python3
"""Nine real native-sensor contrasts; no sensor measurements are synthesized."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid
from config import prepared
from lab import SOURCES, MODELS, Client, check_reading, check_nav_truth, contexts, state_values


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def rotation_error(a, b):
    denominator = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    dot = abs(sum(x*y for x, y in zip(a, b)))/denominator
    return 2*math.acos(min(1, max(0, dot)))


def experiment(result):
    config, record = prepared()  # Saved/invalid source failures must precede control.
    result.update(config=config, world_sha256=record["world_sha256"], phases={},
                  native_bridge_source_sha256=record["native_bridge_source_sha256"],
                  native_bridge_library_sha256=record["native_bridge_library_sha256"])
    client, original_pause, originals, changed = Client(), None, {}, False
    try:
        original_pause = client.physics().pause
        result["restore"] = {"pause_before": original_pause, "models_before": {}}
        client.pause(True)
        originals = {station: client.state(station) for station in MODELS}
        result["restore"]["models_before"] = {station: state_values(value) for station, value in originals.items()}
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "Five actual native sensor publisher connections are required")

        def phase(name, context):
            nonlocal changed
            client.pause(True)
            changed = True
            x, y, z = context["nav_position_m"]
            client.place("nav", x, y, z, context["nav_yaw_deg"], context["nav_velocity_up_m_s"])
            client.place("sonar", -4, 0, context["sonar_height_m"])
            client.place("obstacle", -4, 0 if context["obstacle_present"] else 2, .3)
            client.place("target", *context["logical_target_world_m"])
            setup = {station: client.truth(station) for station in MODELS}
            epoch = client.sim_time_ns()
            client.drain()
            client.pause(False)
            guard = .8 if name == "sonar_out_of_range" else .3
            rows = {source: [] for source in SOURCES}
            seen = {source: set() for source in SOURCES}
            acquired = {source: False for source in SOURCES}
            errors, truth = {}, []
            deadline, next_truth = time.monotonic()+20, time.monotonic()
            while time.monotonic() < deadline:
                for row in client.drain():
                    source, stamp = row["source"], row["event_time_ns"]
                    identity = row["bridge_sequence"] if source == "logical" else row["source_stamp_ns"]
                    if stamp <= epoch or identity in seen[source]:
                        continue
                    seen[source].add(identity)
                    row.update(phase=name, disposition="guard")
                    result["trace"].append(row)
                    if stamp < epoch+round(guard*1e9):
                        continue
                    try:
                        evaluation = check_reading(row, config, context, epoch)
                    except RuntimeError as error:
                        errors[source] = str(error)
                        row["evaluation_error"] = str(error)
                        if acquired[source]:
                            raise
                        row["disposition"] = "acquiring"
                        continue
                    row.update(disposition="accepted", evaluation=evaluation)
                    acquired[source] = True
                    rows[source].append(row)
                if time.monotonic() >= next_truth:
                    actual = client.truth("nav")
                    actual["evaluation"] = check_nav_truth(actual, context, epoch)
                    truth.append(actual)
                    next_truth = time.monotonic()+.065
                if all(len(values) >= 12 and values[-1]["event_time_ns"]-values[0]["event_time_ns"] >= 1000000000 for values in rows.values()):
                    break
                time.sleep(.01)
            else:
                raise RuntimeError("Actual five-sensor windows timed out during "+name+": "+str(errors))
            client.pause(True)
            channels = {}
            for source, values in rows.items():
                times = [row["event_time_ns"] for row in values]
                sequences = [row["bridge_sequence"] for row in values]
                require(all(b > a for a, b in zip(sequences, sequences[1:])), "Per-bridge message sequence is not increasing")
                require(all(b >= a if source == "logical" else b > a for a, b in zip(times, times[1:])), "Native or receipt event times regressed")
                span = (times[-1]-times[0])/1e9
                rate = (len(values)-1)/span
                require(8 <= rate <= 12, "Observed "+source+" rate differs from configured10Hz during "+name)
                channels[source] = {"message_count": len(values), "bridge_sequences": sequences,
                    "event_times_ns": times, "source_stamps_ns": [row["source_stamp_ns"] for row in values],
                    "time_basis": values[0]["time_basis"], "span_s": span, "observed_rate_hz": rate,
                    "last_native": values[-1]["native"], "last_evaluation": values[-1]["evaluation"]}
            start = max(values[0]["event_time_ns"] for values in rows.values())
            end = min(values[-1]["event_time_ns"] for values in rows.values())
            actual_inside = [item for item in truth if start <= (item["clock_before_ns"]+item["clock_after_ns"])/2 <= end]
            require(len(actual_inside) >= 3, "Need three actual navigation state queries during the sensor windows")
            return {"context": context, "control_epoch_ns": epoch, "guard_s": guard,
                    "channels": channels, "truth": truth, "setup_truth": setup,
                    "common_observed_interval_s": (end-start)/1e9,
                    "note": "GPS/mag/alt/sonar use native time; logical uses bridge receipt time and its independent bridge sequence."}

        for name, context in contexts(config):
            result["phases"][name] = phase(name, context)
        result["observations"] = {"five_actual_factories": list(SOURCES), "phase_count": len(result["phases"]),
            "logical_no_occlusion": "Actual tracked target is still listed behind the separate collision occluder.",
            "sonar_model": "Native mesh-cone contacts with finite range_max and hysteresis; not an acoustic propagation or perfect single-ray model.",
            "gps_height": "Ellipsoid height from WGS84 spherical conversion, not an automatic mean-sea-level datum.",
            "motion": "Only initial pose/twist are set. nav_velocity uses a real gravity-disabled body evolving at vz=0.25m/s."}
    finally:
        try:
            if changed:
                client.pause(True)
                restored, errors = {}, {}
                for station, original in originals.items():
                    try:
                        client.apply(original)
                        actual, wanted = state_values(client.state(station)), state_values(original)
                        scalar = max(abs(a-b) for key in ("position", "linear", "angular") for a, b in zip(actual[key], wanted[key]))
                        angle = rotation_error(actual["orientation"], wanted["orientation"])
                        restored[station] = actual
                        require(scalar < 1e-7 and angle < 1e-6, "Original station pose/twist readback failed")
                    except Exception as error:
                        errors[station] = type(error).__name__+": "+str(error)
                result["restore"].update(models_after=restored, cleanup_errors=errors)
                require(not errors, "Restoration attempted for every station but had failures: "+str(errors))
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
    path = (args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("stand-verify-"+uuid.uuid4().hex+".json")).resolve()
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
