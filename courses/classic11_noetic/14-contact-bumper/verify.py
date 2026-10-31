#!/usr/bin/env python3
"""Verify a real miss, free fall, contact and sustained weight support."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
import uuid
from config import prepared
from lab import Client, check_rows, state_values


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def experiment(result):
    config, metadata = prepared()  # Reject saved-but-unloaded input BEFORE ROS/control.
    result.update(config=config, world_sha256=metadata["world_sha256"], phases={}, observations={})
    client, original_pause = Client(), None
    try:
        original_pause = client.physics().pause
        result["initial_paused"] = original_pause
        require(math.isclose(client.mass(), config["mass_kg"], abs_tol=1e-9), "Actual mass differs from prepared world; reload")
        gravity = client.physics().gravity
        require(abs(gravity.x)+abs(gravity.y) < 1e-9 and abs(gravity.z+9.81) < 1e-9, "Actual gravity differs from teaching rig")
        require(client.rospy.get_param("/use_sim_time", False), "Gazebo ROS simulation clock is not active")
        connection_deadline = time.monotonic()+5
        while not client.subscription.get_num_connections() and time.monotonic() < connection_deadline:
            time.sleep(0.02)
        require(client.subscription.get_num_connections() > 0, "No actual contact publisher connection")

        def release(x):
            client.pause(True)
            client.release(config["release_height_m"], x)
            placed = state_values(client.state())
            require(abs(placed["position"][0]-x) < 1e-7 and abs(placed["position"][2]-config["release_height_m"]) < 1e-7,
                    "SetModelState did not place actual body at requested release")
            require(placed["speed"] < 1e-8 and placed["angular_speed"] < 1e-8, "Release twist is not zero")
            stamp = client.rospy.Time.now().to_nsec()
            client.drain()
            client.pause(False)
            return stamp, placed

        def fresh(phase, after, seen):
            rows = []
            for row in client.drain():
                stamp = row["stamp_ns"]
                if stamp <= after or stamp in seen:
                    continue
                seen.add(stamp)
                row["phase"] = phase
                check_rows([row])
                result["trace"].append(row)
                rows.append(row)
            return sorted(rows, key=lambda row: row["stamp_ns"])

        # A real body settles on the uninstrumented floor outside the plate.
        off_start, off_placed = release(1.5)
        seen, quiet, quiet_start = set(), [], None
        deadline = time.monotonic()+9
        while time.monotonic() < deadline:
            rows = fresh("off_plate", off_start, seen)
            value = state_values(client.state())
            stable = abs(value["position"][2]-0.1) < 0.01 and abs(value["position"][0]-1.5) < 0.02 and value["speed"] < 0.03
            if rows and stable:
                if quiet_start is None:
                    quiet_start = rows[-1]["stamp_ns"]
                eligible = [row for row in rows if row["stamp_ns"] >= quiet_start+100000000]
                require(all(not row["states"] for row in eligible), "Bumper incorrectly reports a body resting outside its collision")
                quiet.extend(eligible)
                if len(quiet) >= 10 and quiet[-1]["stamp_ns"]-quiet[0]["stamp_ns"] >= 200000000:
                    break
            elif not stable:
                quiet_start, quiet = None, []
            time.sleep(0.035)
        else:
            raise RuntimeError("Off-plate floor settling / fresh empty contact window timed out")
        result["phases"]["off_plate"] = {"released": off_placed, "release_time_s": off_start/1e9,
            "settled": value, "empty_messages": len(quiet), "empty_span_s": (quiet[-1]["stamp_ns"]-quiet[0]["stamp_ns"])/1e9}

        # Reposition ONCE, then observe motion produced only by ODE.
        start, placed = release(0)
        result["phases"]["release"] = {"released": placed, "release_time_s": start/1e9}
        seen, airborne, first_contact, stable_since = set(), [], None, None
        steady, impact_forces = [], []
        deadline = time.monotonic()+12
        while time.monotonic() < deadline:
            rows = fresh("drop", start, seen)
            for row in rows:
                if row["states"]:
                    if first_contact is None:
                        first_contact = row["stamp_ns"]
                    impact_forces.extend(check_rows([row]))
                elif first_contact is None:
                    airborne.append(row)
            value = state_values(client.state())
            stable = (abs(value["position"][2]-0.3) < 0.01 and abs(value["position"][0]) < 0.02
                      and abs(value["position"][1]) < 0.02 and value["speed"] < 0.03
                      and value["angular_speed"] < 0.03 and value["upright_error"] < 0.005)
            if rows and stable and first_contact is not None:
                if stable_since is None:
                    stable_since = rows[-1]["stamp_ns"]
                eligible = [row for row in rows if row["stamp_ns"] >= stable_since+200000000]
                for row in eligible:
                    require(row["states"], "Sustained support window contains an empty contact message")
                steady.extend(eligible)
                if len(steady) >= 20 and steady[-1]["stamp_ns"]-steady[0]["stamp_ns"] >= 400000000:
                    break
            elif not stable:
                stable_since, steady = None, []
            time.sleep(0.035)
        else:
            raise RuntimeError("Actual landing / sustained support window timed out")
        require(len(airborne) >= 3, "Did not observe three fresh empty contact messages before impact")
        require(first_contact is not None, "No real plate/puck collision observed")
        measured_fall = (first_contact-start)/1e9
        predicted_fall = math.sqrt(2*(config["release_height_m"]-0.3)/9.81)
        require(abs(measured_fall-predicted_fall) <= 0.10, "Observed first-contact time differs from actual release free fall")
        # Every total_wrench sums SPATIAL contact points at one native instant.
        # states within a 50-Hz message may cover different 1-kHz physics instants.
        # Thus average state norms within each message; never sum temporal states.
        per_message = [statistics.mean(check_rows([row])) for row in steady]
        all_steady_forces = check_rows(steady)
        weight = config["mass_kg"]*9.81
        mean, median = statistics.mean(per_message), statistics.median(per_message)
        require(abs(mean-weight) <= 0.15*weight and abs(median-weight) <= 0.15*weight,
                "Steady support mean/median does not match actual mg within15%")
        require(all(abs(force-weight) <= 0.35*weight for force in per_message),
                "A support-message average differs from mg by more than35%")
        steady_stamps = [row["stamp_ns"] for row in steady]
        result["phases"]["airborne"] = {"empty_messages": len(airborne), "first_contact_time_s": first_contact/1e9,
            "observed_fall_s": measured_fall, "predicted_fall_s": predicted_fall}
        result["phases"]["impact"] = {"max_observed_state_force_n": max(impact_forces),
            "note": "50-Hz batching may miss or include multiple native impact instants; no peak force accuracy claim"}
        result["phases"]["settled"] = {"message_count": len(steady), "native_contact_states": len(all_steady_forces),
            "span_s": (steady_stamps[-1]-steady_stamps[0])/1e9, "stamp_ns": steady_stamps}
        result["observations"] = {"settled_center_z": value["position"][2], "settled_speed": value["speed"],
            "settled_state": value, "expected_weight_n": weight,
            "support_mean_n": mean, "support_median_n": median,
            "support_min_n": min(per_message), "support_max_n": max(per_message),
            "state_force_min_n": min(all_steady_forces), "state_force_max_n": max(all_steady_forces),
            "steady_message_rate_hz": (len(steady)-1)/((steady_stamps[-1]-steady_stamps[0])/1e9),
            "wrench_frame": "collision1 link frame; this fixed rig stays axis aligned"}
    finally:
        try:
            if original_pause is not None:
                client.pause(original_pause)
                result["final_paused"] = client.physics().pause
        finally:
            client.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report_path = args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("contact-verify-"+uuid.uuid4().hex+".json")
    result = {"passed": False, "trace": [], "trace_path": str(report_path.resolve())}
    began = time.monotonic()
    try:
        experiment(result)
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-began
    report_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = report_path.with_name(report_path.name+".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(report_path))
    summary = {key: value for key, value in result.items() if key != "trace"}
    summary["trace_messages"] = len(result["trace"])
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
