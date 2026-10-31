#!/usr/bin/env python3
"""Observe five actual IMU regimes, including specific-force free-fall contrast."""
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
from lab import Client, FIXTURES, check_row, norm, quaternion_error, state_values, yaw


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def mean_vector(rows, key):
    return [statistics.mean(row[key][axis] for row in rows) for axis in range(3)]


def experiment(result):
    parameters, record = prepared()  # Before any ROS initialization or control.
    result.update(parameters=parameters, world_sha256=record["world_sha256"],
                  parameters_sha256=record["parameters_sha256"], phases={})
    client, originals, original_pause = Client(), {}, None
    try:
        original_pause = client.physics().pause
        result["restore"] = {"models_before": {}, "pause_before": original_pause}
        originals = {source: client.state(source) for source in FIXTURES}
        result["restore"]["models_before"] = {source: state_values(state) for source, state in originals.items()}
        physics = client.physics()
        require(norm([physics.gravity.x, physics.gravity.y, physics.gravity.z+9.81]) < 1e-9, "Unexpected actual world gravity")
        for source in FIXTURES:
            properties = client.properties(source)
            require(properties.gravity_mode is (source == "freefall") and abs(properties.mass-1) < 1e-9,
                    "Actual IMU fixture gravity/mass differs from prepared rig")
        require(client.rospy.get_param("/use_sim_time", False), "ROS simulation clock is not active")
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not all(sub.get_num_connections() for sub in client.subscriptions):
            time.sleep(.02)
        require(all(sub.get_num_connections() for sub in client.subscriptions), "Both real IMU publisher connections are required")

        def phase(name, roll=0, speed=0, falling=False):
            source = "freefall" if falling else "turntable"
            client.pause(True)
            client.setup("turntable", roll=roll, speed=speed)
            if falling:
                client.setup("freefall")
            epoch = client.rospy.Time.now().to_nsec()
            client.drain()
            client.pause(False)
            guard = .15 if falling else .2
            minimum, duration = (30, .3) if falling else (50, .6)
            expected_gyro = [0, 0, speed]
            expected_force = [0, 0, 0] if falling else [0, 9.81*math.sin(roll), 9.81*math.cos(roll)]
            expected_q = [math.sin(roll/2), 0, 0, math.cos(roll/2)]
            accepted, parallel, truth, seen = [], [], [], {body: set() for body in FIXTURES}
            deadline = time.monotonic()+12
            next_truth = time.monotonic()

            def receive():
                for row in client.drain():
                    body, stamp = row["source"], row["stamp_ns"]
                    if stamp <= epoch or stamp in seen[body]:
                        continue
                    seen[body].add(stamp)
                    row.update(phase=name, disposition="guard")
                    result["trace"].append(row)
                    check_row(row)
                    if stamp < epoch+round(guard*1e9):
                        continue
                    if body != source:
                        row["disposition"] = "parallel"
                        if falling:
                            require(max(abs(value-expected) for value, expected in zip(row["linear_acceleration"], [0, 0, 9.81])) < .05,
                                    "Simultaneous floating reference does not read +g")
                            parallel.append(row)
                        continue
                    row["disposition"] = "accepted"
                    require(max(abs(value-expected) for value, expected in zip(row["angular_velocity"], expected_gyro)) < .02,
                            name+" actual body-frame gyroscope differs from initialized principal-axis motion")
                    require(max(abs(value-expected) for value, expected in zip(row["linear_acceleration"], expected_force)) < .05,
                            name+" actual specific force differs from R^T(a-g)")
                    if speed == 0:
                        require(quaternion_error(row["orientation"], expected_q) < .005,
                                name+" orientation does not match actual initialized rest attitude")
                    accepted.append(row)

            while time.monotonic() < deadline:
                receive()
                if time.monotonic() >= next_truth:
                    truth.append(client.truth(source))
                    next_truth = time.monotonic()+.035
                if accepted and falling:
                    require((accepted[-1]["stamp_ns"]-epoch)/1e9 < .65,
                            "Free-fall window was not completed safely before floor contact")
                if len(accepted) >= minimum and (accepted[-1]["stamp_ns"]-accepted[0]["stamp_ns"])/1e9 >= duration:
                    break
                time.sleep(.01)
            else:
                raise RuntimeError("Actual IMU window timed out during "+name)
            client.pause(True)
            receive()
            stamps = [row["stamp_ns"] for row in accepted]
            require(all(b > a for a, b in zip(stamps, stamps[1:])), "IMU unique headers are not strictly increasing")
            span = (stamps[-1]-stamps[0])/1e9
            rate = (len(stamps)-1)/span
            require(80 <= rate <= 120, "Actual native IMU header rate differs from nominal100Hz")
            intervals = [(right-left)/1e9 for left, right in zip(stamps, stamps[1:])]
            unwrapped = [0.0]
            for left, right in zip(accepted, accepted[1:]):
                increment = (yaw(right["orientation"])-yaw(left["orientation"])+math.pi)%(2*math.pi)-math.pi
                unwrapped.append(unwrapped[-1]+increment)
            for row, angle in zip(accepted, unwrapped):
                expected = speed*(row["stamp_ns"]-stamps[0])/1e9
                require(abs(angle-expected) < .02, "Actual unwrapped quaternion yaw does not match omega times native header interval")
            comparisons = []
            for item in truth:
                midpoint = (item["clock_before_ns"]+item["clock_after_ns"])/2
                if not stamps[0] <= midpoint <= stamps[-1]:
                    continue
                nearest = min(accepted, key=lambda row: abs(row["stamp_ns"]-midpoint))
                gap = abs(nearest["stamp_ns"]-midpoint)/1e9
                width = (item["clock_after_ns"]-item["clock_before_ns"])/1e9
                require(0 <= width <= .05 and gap <= .06, "Actual link service timing does not support an IMU attitude comparison")
                error = quaternion_error(nearest["orientation"], item["state"]["orientation"])
                require(error <= abs(speed)*(gap+width)+.02, "IMU attitude differs from independently queried actual world link attitude")
                comparison = {"imu_stamp_ns": nearest["stamp_ns"], "clock_gap_s": gap,
                              "service_interval_s": width, "orientation_error_rad": error}
                if falling:
                    elapsed = (midpoint-epoch)/1e9
                    expected_z, expected_vz = 3-.5*9.81*elapsed**2, -9.81*elapsed
                    z, vz = item["state"]["position"][2], item["state"]["linear"][2]
                    require(z > .5 and vz < -.5 and abs(z-expected_z) < .04 and abs(vz-expected_vz) < .08,
                            "Specific-force zero is not backed by actual free-fall position/velocity")
                    comparison.update(elapsed_s=elapsed, expected_z_m=expected_z, actual_z_m=z,
                                      expected_vz_m_s=expected_vz, actual_vz_m_s=vz)
                comparisons.append(comparison)
            require(len(comparisons) >= 3, "Need three independently queried actual link states within accepted window")
            if falling:
                require(len(parallel) >= 20, "No simultaneous +g floating reference during actual free fall")
            return {"source": source, "message_count": len(accepted), "stamp_ns": stamps, "span_s": span, "rate_hz": rate,
                    "expected_gyro": expected_gyro, "expected_specific_force": expected_force,
                    "guard_s": guard, "control_epoch_ns": epoch, "truth": truth, "truth_comparisons": comparisons,
                    "observations": {"mean_gyro": mean_vector(accepted, "angular_velocity"),
                        "mean_specific_force": mean_vector(accepted, "linear_acceleration"),
                        "yaw_delta_rad": unwrapped[-1], "expected_yaw_delta_rad": speed*span,
                        "dt_min_s": min(intervals), "dt_median_s": statistics.median(intervals), "dt_max_s": max(intervals),
                        "parallel_float_messages": len(parallel),
                        "parallel_float_mean_specific_force": mean_vector(parallel, "linear_acceleration") if parallel else None}}

        tilt = math.radians(parameters["tilt_deg"])
        for name, roll, speed in (("upright_static", 0, 0), ("tilted_static", tilt, 0),
                                  ("positive_yaw", 0, parameters["speed_rad_s"]),
                                  ("negative_yaw", 0, -parameters["speed_rad_s"])):
            result["phases"][name] = phase(name, roll, speed)
        result["phases"]["freefall"] = phase("freefall", falling=True)
    finally:
        try:
            if originals:
                client.pause(True)
                restored = {}
                errors, cleanup_errors = {}, {}
                for source, original in originals.items():
                    try:
                        client.apply(original)
                        actual, expected = state_values(client.state(source)), state_values(original)
                        restored[source] = actual
                        scalar_error = max(abs(a-b) for key in ("position", "linear", "angular") for a, b in zip(actual[key], expected[key]))
                        rotation_error = quaternion_error(actual["orientation"], expected["orientation"])
                        errors[source] = {"position_twist_max_error": scalar_error, "orientation_error_rad": rotation_error}
                        require(scalar_error < 1e-7 and rotation_error < 1e-6, "Actual model pose/twist restoration failed: "+source)
                    except Exception as error:
                        cleanup_errors[source] = type(error).__name__+": "+str(error)
                result["restore"].update(models_after=restored, errors=errors, cleanup_errors=cleanup_errors)
                require(not cleanup_errors, "Model restoration attempts completed with failures: "+str(cleanup_errors))
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
    path = args.report or Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))/("imu-verify-"+uuid.uuid4().hex+".json")
    result = {"passed": False, "trace": [], "trace_path": str(path.resolve())}
    start = time.monotonic()
    try:
        experiment(result)
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    result["wall_seconds"] = time.monotonic()-start
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
