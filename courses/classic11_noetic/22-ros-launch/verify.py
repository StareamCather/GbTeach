#!/usr/bin/env python3
"""Verify actual roslaunch graph, a finite force pulse, coast, and state restoration."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid
from config import prepared
from lab import Client, cancellation, cleanup, require, save


def middle(row):
    return (row["world_time_before_ns"]+row["world_time_after_ns"])/2


def regression(rows):
    require(len(rows) >= 10, "Need ten actual state samples for a meaningful motion fit")
    origin = middle(rows[0])
    times = [(middle(row)-origin)/1e9 for row in rows]
    require(times[-1]-times[0] >= .15, "Motion fit needs at least .15 simulation seconds")
    values = [row["state"]["linear"][0] for row in rows]
    x_mean, y_mean = sum(times)/len(times), sum(values)/len(values)
    slope = sum((x-x_mean)*(y-y_mean) for x, y in zip(times, values))/sum((x-x_mean)**2 for x in times)
    return {"sample_count": len(rows), "span_s": times[-1]-times[0], "acceleration_m_s2": slope,
            "velocity_min_m_s": min(values), "velocity_max_m_s": max(values)}


def rotation_error(a, b):
    norm = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    require(norm > 0, "Invalid quaternion")
    dot = abs(sum(x*y for x, y in zip(a, b)))/norm
    return 2*math.acos(min(1, max(0, dot)))


def experiment(result):
    config, record = prepared()  # Refuse saved-but-unloaded inputs before ROS controls.
    result.update(config=config, world_sha256=record["world_sha256"], settings_sha256=record["settings_sha256"],
                  source_sha256=record["source_sha256"], phases={}, restore={})
    client, original_pause, original_state, changed = Client(config, record), None, None, False
    try:
        # Establish ownership and actual launch identities before changing the world.
        result["readiness"] = client.ready(allow_paused=True)
        result["world"] = client.world_ready()
        original_pause = client.physics().pause
        client.pause(True)
        original_state = client.state()
        result["restore"].update(pause_before=original_pause,
            state_before=client.common.pose_twist(original_state.pose, original_state.twist))
        changed = True
        result["initial_cancel"] = client.cancel()
        client.reset_state()
        client.drain()

        def collect_until(end, phase, name, timeout=15):
            truth, feedback = [], []
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                row = client.truth()
                row.update(kind="model_state_truth", phase=phase(middle(row)))
                require(row["world_time_after_ns"] >= row["world_time_before_ns"], "World time regressed during a state readback")
                truth.append(row)
                result["trace"].append(row)
                for actual in client.drain():
                    actual.update(kind="feedback", phase=name+"_feedback")
                    feedback.append(actual)
                    result["trace"].append(actual)
                if middle(row) >= end:
                    break
                time.sleep(.007)
            else:
                raise RuntimeError("Finite motion-observation timeout during "+name)
            return truth, feedback

        baseline_start = client.world_time_ns()
        client.pause(False)
        baseline, baseline_feedback = collect_until(baseline_start+250000000, lambda stamp: "baseline", "baseline")
        require(len(baseline) >= 10, "Need ten baseline state observations")
        for row in baseline:
            state = row["state"]
            require(max(abs(value) for value in state["linear"]+state["angular"]) < 1e-5, "Idle launch nodes unexpectedly move the body")
            require(max(abs(value-target) for value, target in zip(state["position"], (0, 0, 1))) < 1e-5, "Idle body pose changed")
        require(len(baseline_feedback) >= 3, "Need fresh real feedback during idle baseline")
        result["phases"]["baseline"] = {"truth": baseline, "feedback_count": len(baseline_feedback), "default_driver_submits_force": False}

        client.pause(True)
        command = client.pulse()
        result["command"] = command
        require(command["force_n"] == config["force_n"] and command["duration_ns"] == 300000000,
                "Actual driver command differs from launch parameters")
        try:
            client.pulse()
        except RuntimeError as error:
            require("already scheduled" in str(error), "Concurrent rejection lost its actual busy diagnostic: "+str(error))
            result["duplicate_pulse"] = {"rejected": True, "message": str(error)}
        else:
            raise RuntimeError("Driver accepted overlapping force jobs")
        start, end = command["requested_start_time_ns"], command["requested_end_time_ns"]
        client.drain()
        client.pause(False)
        truth, feedback = collect_until(end+550000000,
            lambda stamp: "prelead" if stamp < start else "active" if stamp <= end else "coast", "pulse_cycle")
        client.pause(True)
        active = [row for row in truth if start+30000000 <= middle(row) <= end-30000000]
        coast = [row for row in truth if middle(row) >= end+50000000]
        active_fit, coast_fit = regression(active), regression(coast)
        expected_a = config["force_n"]/config["mass_kg"]
        require(abs(active_fit["acceleration_m_s2"]/expected_a-1) <= .05, "Real acceleration does not match launch force/mass")
        require((middle(coast[-1])-middle(coast[0]))/1e9 >= .3, "Need .3 simulation seconds of actual coasting")
        require(abs(coast_fit["acceleration_m_s2"]) <= .05, "Acceleration continues after the finite pulse should expire")
        expected_v = expected_a*.3
        maximum_position_error = 0
        for row in truth:
            stamp, state = middle(row), row["state"]
            all_values = state["position"]+state["orientation"]+state["linear"]+state["angular"]
            require(all(math.isfinite(value) for value in all_values), "Non-finite real model state")
            t = max(0, (stamp-start)/1e9)
            active_time = min(.3, t)
            expected_x = .5*expected_a*active_time**2+expected_v*max(0, t-.3)
            error = abs(state["position"][0]-expected_x)
            maximum_position_error = max(maximum_position_error, error)
            require(error <= .01, "Real displacement differs from the finite force-pulse model")
            require(abs(state["position"][1]) <= .0001 and abs(state["position"][2]-1) <= .0001,
                    "Unexpected off-axis translation")
            require(max(abs(value) for value in state["linear"][1:]+state["angular"]) <= .0001,
                    "Unexpected off-axis velocity or rotation")
            if stamp >= end+50000000:
                require(abs(state["linear"][0]-expected_v) <= .02, "Coasting velocity differs from the actual commanded impulse")
        result["phases"]["active"] = {"truth": active, "fit": active_fit, "expected_acceleration_m_s2": expected_a,
            "edge_guard_s": .03, "maximum_position_error_m": maximum_position_error,
            "integration_note": "The native force scheduler includes both endpoints; up to one physics tick beyond nominal duration is allowed by the declared tolerances."}

        pairs = []
        coast_host_start = (coast[0]["host_monotonic_before_ns"]+coast[0]["host_monotonic_after_ns"])/2
        coast_host_end = (coast[-1]["host_monotonic_before_ns"]+coast[-1]["host_monotonic_after_ns"])/2
        for row in feedback:
            host = row["observer_receipt_monotonic_ns"]
            if not coast_host_start <= host <= coast_host_end:
                continue
            nearest = min(coast, key=lambda item: abs((item["host_monotonic_before_ns"]+item["host_monotonic_after_ns"])/2-host))
            host_mid = (nearest["host_monotonic_before_ns"]+nearest["host_monotonic_after_ns"])/2
            gap = abs(host_mid-host)/1e9
            if gap > .05:
                continue
            position_error = max(abs(a-b) for a, b in zip(row["state"]["position"], nearest["state"]["position"]))
            velocity_error = max(abs(a-b) for a, b in zip(row["state"]["linear"], nearest["state"]["linear"]))
            require(position_error <= .05 and velocity_error <= .02, "Actual feedback disagrees with nearby independent state-service readback")
            pairs.append({"source_message_sequence": row["source_message_sequence"], "observer_receipt_monotonic_ns": host,
                "truth_host_before_ns": nearest["host_monotonic_before_ns"], "truth_host_after_ns": nearest["host_monotonic_after_ns"],
                "host_mid_gap_s": gap, "position_error_m": position_error, "velocity_error_m_s": velocity_error})
        require(len(pairs) >= 5, "Need five actual feedback/independent-service consistency pairs during coast")
        result["phases"]["coast"] = {"truth": coast, "fit": coast_fit, "expected_velocity_m_s": expected_v,
            "feedback_pairs": pairs, "feedback_time_note": "Host receipt-neighbor consistency only. ModelStates has no native header; no capture synchronization is claimed."}
    finally:
        with cleanup():
            errors = {}
            if changed:
                for name, action in (("pause_for_restore", lambda: client.pause(True)), ("cancel", client.cancel)):
                    try:
                        result["restore"][name] = action()
                    except Exception as error:
                        errors[name] = str(error)
                if "cancel" in errors:
                    try:
                        client.clear_body()
                        result["restore"]["scoped_clear_fallback"] = True
                    except Exception as error:
                        errors["scoped_clear_fallback"] = str(error)
                try:
                    client.apply_state(original_state)
                    actual_state = client.state()
                    restored = client.common.pose_twist(actual_state.pose, actual_state.twist)
                    wanted = result["restore"]["state_before"]
                    error = max(abs(a-b) for key in ("position", "linear", "angular") for a, b in zip(restored[key], wanted[key]))
                    require(error < 1e-7 and rotation_error(restored["orientation"], wanted["orientation"]) < 1e-6, "Entry pose/twist readback failed")
                    result["restore"]["state_after"] = restored
                except Exception as error:
                    errors["model_state"] = str(error)
            try:
                if original_pause is not None:
                    client.pause(original_pause)
                    result["restore"]["pause_after"] = client.physics().pause
                    require(result["restore"]["pause_after"] == original_pause, "Entry pause state was not restored")
            except Exception as error:
                errors["pause_state"] = str(error)
            finally:
                client.close()
            result["restore"]["cleanup_errors"] = errors
            require(not errors, "Cleanup attempted every restoration step: "+str(errors))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    session = Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))
    path = (args.report or session/("ros-launch-verify-"+uuid.uuid4().hex+".json")).resolve()
    result = {"passed": False, "trace": [], "trace_path": str(path)}
    started = time.monotonic()
    try:
        with cancellation():
            experiment(result)
            result["passed"] = True
    except Exception as error:
        result.update(passed=False, error=type(error).__name__+": "+str(error))
    result["wall_seconds"] = time.monotonic()-started
    save(path, result)
    print(json.dumps({key: value for key, value in result.items() if key != "trace"}, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
