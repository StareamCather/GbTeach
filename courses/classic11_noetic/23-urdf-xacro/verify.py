#!/usr/bin/env python3
"""Compare actual URDF physics with link-tree geometry, then apply finite joint efforts."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import uuid
from config import JOINTS, prepared
from lab import Client, POSES, angle_error, cancellation, cleanup, quaternion, require


def middle(sample):
    return (sample["world_time_before_ns"]+sample["world_time_after_ns"])/2


def fk(config, q):
    first = config["base_yaw_rad"]+q[0]
    second = first+config["joint2_origin_yaw_rad"]+q[1]
    z = config["base_height_m"]+config["joint1_height_m"]
    elbow = [config["length1_m"]*math.cos(first), config["length1_m"]*math.sin(first), z]
    tip = [elbow[0]+config["length2_m"]*math.cos(second), elbow[1]+config["length2_m"]*math.sin(second), z]
    return {"base_link": {"position": [0, 0, config["base_height_m"]], "orientation": quaternion([0, 0, config["base_yaw_rad"]])},
            "arm1": {"position": [0, 0, z], "orientation": quaternion([0, 0, first])},
            "arm2": {"position": elbow, "orientation": quaternion([0, 0, second])},
            "tool_link": {"position": tip, "orientation": quaternion([0, 0, second])}}


def compare_fk(config, snapshot):
    q = [snapshot["joint_readback"][name]["position"][0] for name in JOINTS]
    expected, errors = fk(config, q), {}
    for name, reading in snapshot["link_readback"].items():
        actual = reading["link_state"]
        position = math.sqrt(sum((a-b)**2 for a, b in zip(actual["position"], expected[name]["position"])))
        rotation = angle_error(actual["orientation"], expected[name]["orientation"])
        require(position < 1e-5 and rotation < 1e-5, "Actual link frame differs from URDF FK: "+name)
        errors[name] = {"position_error_m": position, "orientation_error_rad": rotation}
    return expected, errors


def limits(sample):
    for name, bound in zip(JOINTS, (1.4, 1.2)):
        value = sample["joint_readback"][name]
        require(all(math.isfinite(x) for x in value["position"]+value["rate"]), "Nonfinite moving joint measurement")
        require(abs(value["position"][0]) <= bound+.02, "Real articulated joint crossed its declared physical position limit")


def experiment(result):
    config, record = prepared()
    result.update(config=config, world_sha256=record["world_sha256"], settings_sha256=record["settings_sha256"],
        source_sha256=record["source_sha256"], generated_urdf_path=record["generated_urdf_path"],
        generated_urdf_sha256=record["generated_urdf_sha256"], converted_sdf_path=record["converted_sdf_path"],
        converted_sdf_sha256=record["converted_sdf_sha256"], static_phases={}, dynamic_phases={}, restore={})
    client, original_pause, original, changed = Client(config, record), None, None, False
    try:
        result["readiness"] = client.ready(timeout=3)
        result["joint_name_uniqueness"] = client.assert_unique_joints()
        original_pause = client.physics().pause
        client.pause(True)
        original = client.snapshot()
        result["restore"].update(pause_before=original_pause, snapshot_before=original)
        changed = True
        for name, requested in POSES.items():
            client.initialize(requested)
            snapshot = client.snapshot()
            require(snapshot["paused"], "Static FK must be observed while paused")
            actual_q = [snapshot["joint_readback"][joint]["position"][0] for joint in JOINTS]
            require(max(abs(a-b) for a, b in zip(actual_q, requested)) < 1e-6, "Stage-boundary joint initialization readback differs")
            expected, errors = compare_fk(config, snapshot)
            phase = {**snapshot, "requested_q": requested, "expected_fk": expected, "fk_errors": errors,
                     "note": "SetModelConfiguration at a paused boundary initializes pose; it is not a dynamics experiment."}
            result["static_phases"][name] = phase
            result["trace"].append({"kind": "static_snapshot", "phase": name, **snapshot})

        def dynamic(name, initial_q, effort, duration):
            client.pause(True)
            client.initialize(initial_q)
            initial = client.snapshot()
            command = client.effort(effort, duration)
            require(command["scheduling_world_time_ns"] == initial["world_time_ns"], "Effort should be scheduled at the same paused boundary")
            client.pause(False)
            samples = []
            deadline = time.monotonic()+15
            finish = command["requested_end_time_ns"]+100000000
            while time.monotonic() < deadline:
                sample = client.sample()
                require(sample["world_time_after_ns"] >= sample["world_time_before_ns"], "World time regressed during joint readback")
                limits(sample)
                sample.update(kind="joint_state_truth", phase=name)
                samples.append(sample)
                result["trace"].append(sample)
                if middle(sample) >= finish:
                    break
                time.sleep(.007)
            else:
                raise RuntimeError("Finite joint observation timed out: "+name)
            client.pause(True)
            final = client.snapshot()
            require(len(samples) >= 10, "Need ten fresh joint-service observations")
            elbow = JOINTS[1]
            q_start = initial["joint_readback"][elbow]["position"][0]
            q_end = final["joint_readback"][elbow]["position"][0]
            positions = [row["joint_readback"][elbow]["position"][0] for row in samples]
            rates = [row["joint_readback"][elbow]["rate"][0] for row in samples]
            summary = {"elbow_position_change_rad": q_end-q_start, "elbow_position_min_rad": min(positions),
                       "elbow_position_max_rad": max(positions), "elbow_rate_min_rad_s": min(rates), "elbow_rate_max_rad_s": max(rates)}
            if name != "upper_limit":
                sign = 1 if effort > 0 else -1
                require(sign*(q_end-q_start) >= .01, "Finite native joint effort did not produce the expected signed elbow motion")
                require(max(sign*value for value in rates) >= .05, "Finite effort did not produce a meaningful actual elbow-rate response")
            else:
                require(max(positions) >= 1.17 and max(positions) <= 1.22, "Near-limit experiment did not reach and respect the real elbow stop")
                late = [row for row in samples if command["requested_end_time_ns"]-60000000 <= middle(row) <= command["requested_end_time_ns"]]
                require(len(late) >= 3, "Need three actual observations in the last60ms while effort is still scheduled")
                errors = [abs(row["joint_readback"][elbow]["position"][0]-1.2) for row in late]
                late_rates = [abs(row["joint_readback"][elbow]["rate"][0]) for row in late]
                require(sum(errors)/len(errors) <= .02 and max(late_rates) <= .2, "Actual joint did not settle against its declared upper stop")
                summary.update(late_sample_count=len(late), mean_upper_error_rad=sum(errors)/len(errors), max_late_abs_rate_rad_s=max(late_rates))
            return {"initial": initial, "command": command, "samples": samples, "final": final, "summary": summary,
                "note": "Only phase boundaries set joint pose and clear velocities. All samples here follow real finite native effort; shoulder coupling is allowed, and no single-rod I*alpha claim is made."}

        result["dynamic_phases"]["motion_positive"] = dynamic("motion_positive", [0, 0], .03, .2)
        result["dynamic_phases"]["motion_negative"] = dynamic("motion_negative", [0, 0], -.03, .2)
        result["dynamic_phases"]["upper_limit"] = dynamic("upper_limit", [0, 1.12], .12, .35)
        result["observations"] = {"fixed_joint": "Actual physical links and merged inertial properties were queried, not inferred from robot_description alone.",
            "frames": "GetLinkState returns the actual physical link frame. A lumped URDF tool name is not assumed to be an available physics link or TF.",
            "effort": "Finite ApplyJointEffort uses scoped names; ClearJointForces uses unique raw basenames in this SDK. Empty clear responses are not measurement proof."}
    finally:
        with cleanup():
            errors = {}
            if changed:
                try:
                    client.pause(True)
                except Exception as error:
                    errors["pause_for_restore"] = str(error)
                from gazebo_msgs.srv import JointRequest
                for joint in JOINTS:
                    try:
                        client.call("/gazebo/clear_joint_forces", JointRequest, joint_name=joint)
                    except Exception as error:
                        errors["clear_"+joint] = str(error)
                try:
                    client.set_q([original["joint_readback"][name]["position"][0] for name in JOINTS])
                except Exception as error:
                    errors["restore_q"] = str(error)
                for name, value in original["link_readback"].items():
                    try:
                        client.set_link(name, value["link_state"])
                    except Exception as error:
                        errors["restore_link_"+name] = str(error)
                try:
                    restored = client.snapshot()
                    result["restore"]["snapshot_after"] = restored
                    for name in JOINTS:
                        for field in ("position", "rate"):
                            require(abs(restored["joint_readback"][name][field][0]-original["joint_readback"][name][field][0]) < 1e-5, "Restored actual joint "+field+" differs")
                    for name, value in original["link_readback"].items():
                        actual, wanted = restored["link_readback"][name]["link_state"], value["link_state"]
                        require(max(abs(a-b) for field in ("position", "linear", "angular") for a, b in zip(actual[field], wanted[field])) < 1e-6 and angle_error(actual["orientation"], wanted["orientation"]) < 1e-6, "Restored actual link pose/twist differs: "+name)
                except Exception as error:
                    errors["restore_readback"] = str(error)
            try:
                if original_pause is not None:
                    client.pause(original_pause)
                    result["restore"]["pause_after"] = client.physics().pause
                    require(result["restore"]["pause_after"] == original_pause, "Entry pause was not restored")
            except Exception as error:
                errors["restore_pause"] = str(error)
            finally:
                client.close()
            result["restore"]["cleanup_errors"] = errors
            require(not errors, "All restoration steps were attempted: "+str(errors))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    session = Path(os.environ.get("GAZEBOTEACH_SESSION_DIR", Path.cwd()/"build"))
    path = (args.report or session/("urdf-xacro-verify-"+uuid.uuid4().hex+".json")).resolve()
    result = {"passed": False, "trace": [], "trace_path": str(path)}
    started = time.monotonic()
    try:
        with cancellation():
            experiment(result)
            result["passed"] = True
    except Exception as error:
        result.update(passed=False, error=type(error).__name__+": "+str(error))
    result["wall_seconds"] = time.monotonic()-started
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(str(temporary), str(path))
    print(json.dumps({key: value for key, value in result.items() if key != "trace"}, ensure_ascii=False, allow_nan=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
