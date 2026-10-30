#!/usr/bin/env python3
"""L11: independently integrate real wheel encoders and real axle ground truth.

--math-only is numerical, never physical acceptance. Normal, changed-speed and
low-friction sessions are separate private copies. Measurements use actual
constant-window start poses/encoders, without writing a body pose or velocity.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import types
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson06_acceptance import actual_stats
from tests.lesson091_acceptance import control_directory, digest, read_json, require, snapshot, source_hashes
from tests.lesson10_acceptance import bounded_steps, request_phase

KEY = "classic11-noetic:11"
DT, RADIUS, SEPARATION = .001, .1, .36
PHASES = ("straight", "spin", "arc")


def close(actual, expected, label, tolerance=1e-7):
    require(math.isfinite(actual) and math.isfinite(expected) and abs(actual-expected) <= tolerance,
            "{}: actual {} != expected {} +/- {}".format(label, actual, expected, tolerance))


def angle_delta(value):
    return math.atan2(math.sin(value), math.cos(value))


def sinc(value):
    return 1-value*value/6+value**4/120 if abs(value) < 1e-6 else math.sin(value)/value


def advance(pose, distance, angle):
    """Exact body-frame SE(2) increment, stable for straight or tiny curvature."""
    scale = distance*sinc(angle/2)
    return [pose[0]+scale*math.cos(pose[2]+angle/2),
            pose[1]+scale*math.sin(pose[2]+angle/2), pose[2]+angle]


def ideal(v, omega, duration):
    return advance([0., 0., 0.], v*duration, omega*duration)


def encoder_continuity(previous, current):
    errors = []
    for side in ("left", "right"):
        difference = angle_delta(current[side+"_q_before_rad"]-previous[side+"_q_after_rad"])
        close(difference, 0., "Actual {} encoder continuity at phase {} stage {} step {}".format(
            side, current.get("phase", "numeric-example"), current.get("stage", "numeric-example"), current.get("phase_step", "?")), 1e-9)
        errors.append(abs(difference))
    return max(errors)


def math_regressions():
    for args, expected in (((.25, 0., 2.), [.5, 0., 0.]),
                           ((0., .6, 2.), [0., 0., 1.2]),
                           ((.25, .5, 2.), [.42073549240394825, .22984884706593015, 1.]),
                           ((.35, .5, 2.), [.5890296893655275, .3217883858923022, 1.])):
        for actual, want in zip(ideal(*args), expected):
            close(actual, want, "Independent constant-twist reference", 1e-12)
    tiny = ideal(.25, 1e-14, 2.)
    close(tiny[0], .5, "Near-straight SE2 limit", 1e-12)
    close(angle_delta((-math.pi+.01)-(math.pi-.01)), .02, "Wrapped SDK yaw crossing", 1e-12)
    pose = [0., 0., 0.]
    for _ in range(2000):
        pose = advance(pose, .25*DT, .5*DT)
    for actual, expected in zip(pose, ideal(.25, .5, 2.)):
        close(actual, expected, "Incremental SE2 and full-interval reference", 1e-11)
    reversed_wheel_sign = advance([0., 0., 0.], 0., -.36*.6*2/.36)
    require(abs(reversed_wheel_sign[2]-1.2) > 2., "Yaw-sign numerical check did not distinguish wheel order")
    stopped_rows = [{"stage": "stopping_final", "command_v_m_s": 0., "command_yaw_rate_rad_s": 0.,
                     "left_w_after_rad_s": 0., "right_w_after_rad_s": 0., "body_vx_m_s": 0.,
                     "body_vy_m_s": 0., "body_vz_m_s": 0., "body_yaw_rate_rad_s": 0.} for _ in range(50)]
    evidence = {"confirmed": True, "duration_s": .05, "consecutive_s": .05, "left_w_rad_s": 0.,
                "right_w_rad_s": 0., "body_linear_speed_m_s": 0., "body_yaw_rate_rad_s": 0.}
    independent_stop(stopped_rows, "stopping_final", evidence)
    broken = [dict(row) for row in stopped_rows]
    broken[-1]["body_vy_m_s"] = .02
    try:
        independent_stop(broken, "stopping_final", evidence)
    except AssertionError:
        false_ack_rejected = True
    else:
        raise AssertionError("A stop flag hid a real nonzero raw body velocity")
    previous = {"left_q_after_rad": 3.14, "right_q_after_rad": -2.}
    following = {"left_q_before_rad": 3.14-2*math.pi, "right_q_before_rad": -2.+2*math.pi}
    encoder_continuity(previous, following)
    discontinuous = dict(following, left_q_before_rad=following["left_q_before_rad"]+2e-8)
    try:
        encoder_continuity(previous, discontinuous)
    except AssertionError:
        discontinuity_rejected = True
    else:
        raise AssertionError("Cross-row raw encoder continuity did not reject a true discontinuity")
    return {"passed": True, "kind": "numerical_SE2_and_wrap_regressions_not_physics", "gazebo_started": False,
            "arc_reference": ideal(.25, .5, 2.), "changed_speed_arc_reference": ideal(.35, .5, 2.),
            "tiny_curvature_reference": tiny, "actual_incremental_calculation": pose,
            "fabricated_stop_ack_rejected_by_raw_velocities": false_ack_rejected,
            "wrapped_encoder_continuity_accepted_and_real_discontinuity_rejected": discontinuity_rejected}


def xml_world(workspace):
    return ET.parse(str(workspace/"world.sdf")).getroot().find("world")


def surface_source(workspace):
    world = xml_world(workspace)
    car = world.find("model[@name='differential_car']")
    values = {}
    for prefix, link, collision in (("left", car.find("link[@name='left_wheel']"), "wheel_collision"),
                                     ("right", car.find("link[@name='right_wheel']"), "wheel_collision"),
                                     ("floor", world.find("model[@name='floor']/link[@name='ground']"), "floor_collision"),
                                     ("caster", car.find("link[@name='caster']"), "caster_collision")):
        for key in ("mu", "mu2"):
            values[prefix+"_"+key] = float(link.findtext("collision[@name='%s']/surface/friction/ode/%s" % (collision, key)))
    wheel = [values[name] for name in ("left_mu", "left_mu2", "right_mu", "right_mu2")]
    require(all(math.isfinite(value) for value in values.values()), "Source surfaces are non-finite")
    require(max(wheel)-min(wheel) <= 1e-12 and any(abs(wheel[0]-value) <= 1e-12 for value in (.8, .001)),
            "Both actual wheel collision mu/mu2 must select the same supported experiment")
    values["surface_mode"] = "normal" if wheel[0] > .1 else "slip"
    return values


def check_model(workspace, status):
    m, source = status["model"], surface_source(workspace)
    fixed = {"wheel_radius_m": RADIUS, "wheel_separation_m": SEPARATION,
             "left_collision_bbox_max_m": .2, "right_collision_bbox_max_m": .2,
             "left_mass_kg": .2, "right_mass_kg": .2, "chassis_mass_kg": 2.,
             "caster_mass_kg": .05, "physics_step_s": DT}
    for side in ("left", "right"):
        fixed.update({side+"_collision_radius_m": .1, side+"_collision_length_m": .01,
                      side+"_Ixx_kg_m2": .000501666666666667, side+"_Iyy_kg_m2": .001,
                      side+"_Izz_kg_m2": .000501666666666667})
    for name, expected in fixed.items():
        close(float(m[name]), expected, "Actual SDK drive geometry "+name, 1e-8)
    for name, expected in source.items():
        if name != "surface_mode":
            close(float(m[name]), expected, "Source/current actual collision surface "+name, 1e-8)
    car = xml_world(workspace).find("model[@name='differential_car']")
    for prefix, link_name in (("left", "left_wheel"), ("right", "right_wheel"), ("chassis", "chassis"), ("caster", "caster")):
        inertial = car.find("link[@name='%s']/inertial" % link_name)
        for axis in ("xx", "yy", "zz"):
            close(float(m[prefix+"_I"+axis+"_kg_m2"]), float(inertial.findtext("inertia/i"+axis)),
                  "Current source/actual SDK inertia "+prefix+axis, 1e-9)
        pose = [float(value) for value in inertial.findtext("pose", "0 0 0 0 0 0").split()]
        for axis, value in zip("xyz", pose[:3]):
            close(float(m[prefix+"_com_"+axis+"_m"]), value, "Current source/actual SDK COM "+prefix+axis, 1e-9)
    require(m["surface_mode"] == source["surface_mode"] and m["stock_subscriber_connected"] is True,
            "Actual wheel surfaces or stock command subscription do not match this experiment")
    for name, expected in (("initial_left_axis_xyz", [0., 1., 0.]), ("initial_right_axis_xyz", [0., 1., 0.]),
                            ("initial_left_anchor_xyz", [0., .18, .1002]),
                            ("initial_right_anchor_xyz", [0., -.18, .1002])):
        require(len(m[name]) == 3, "Incomplete initial actual joint geometry")
        for actual, target in zip(m[name], expected):
            close(float(actual), target, "Actual SDK drive joint "+name, 1e-8)
    distance = math.sqrt(sum((a-b)**2 for a, b in zip(m["initial_left_anchor_xyz"], m["initial_right_anchor_xyz"])))
    close(distance, m["wheel_separation_m"], "Stock separation from actual initial anchor distance", 1e-10)
    close(m["wheel_radius_m"], m["left_collision_bbox_max_m"]/2, "Stock radius from cached Init collision bounding box", 1e-10)
    require(status["command_message_type"] == "gazebo.msgs.Pose"
            and status["command_topic"].endswith("/differential_car/vel_cmd"), "Wrong actual stock Classic command interface")
    return {"actual_sdk_model": m, "source_collision_surfaces": source,
            "ground_truth_reference": "midpoint of actual wheel joint anchors, chassis world yaw"}


def check_stop(stop):
    require(stop["confirmed"] is True and .05-1e-8 <= stop["consecutive_s"] <= stop["duration_s"]+1e-8
            and stop["duration_s"] <= 2+1e-8, "Actual zero-command stop lacks its bounded continuous acknowledgement")
    for name, limit in (("left_w_rad_s", .1), ("right_w_rad_s", .1),
                        ("body_linear_speed_m_s", .01), ("body_yaw_rate_rad_s", .03)):
        value = float(stop[name])
        require(math.isfinite(value) and abs(value) < limit, "Actual stopped speed exceeds its declared limit: "+name)


def read_trace(session):
    with (control_directory(session)/"drive.csv").open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def preserve_drive_evidence(session, destination):
    evidence = snapshot(session, destination)
    for path in control_directory(session).glob("*.jsonl"):
        target = destination/path.name
        target.write_bytes(path.read_bytes())
        evidence[path.name] = {"path": str(target), "sha256": digest(target), "bytes": target.stat().st_size}
    return evidence


def numeric(records):
    labels = {"episode", "phase", "stage", "config_sha256"}
    return [{key: value if key in labels else float(value) for key, value in row.items()} for row in records]


def independent_stop(rows, stage, evidence):
    selected = [row for row in rows if row["stage"] == stage]
    require(selected, "Missing actual zero-command stop trace: "+stage)
    consecutive = 0
    for row in selected:
        close(row["command_v_m_s"], 0., "Stop published linear zero")
        close(row["command_yaw_rate_rad_s"], 0., "Stop published angular zero")
        speed = math.sqrt(sum(row[key]**2 for key in ("body_vx_m_s", "body_vy_m_s", "body_vz_m_s")))
        good = abs(row["left_w_after_rad_s"]) < .1 and abs(row["right_w_after_rad_s"]) < .1 and speed < .01 and abs(row["body_yaw_rate_rad_s"]) < .03
        consecutive = consecutive+1 if good else 0
    require(consecutive >= 50 and len(selected) <= 2000, "Actual stop trace lacks the predeclared 50 ms continuous bounded zero ACK")
    close(evidence["duration_s"], len(selected)*DT, "Actual stop duration from raw step count", 1e-8)
    close(evidence["consecutive_s"], consecutive*DT, "Actual continuous stop interval from raw SDK states", 1e-8)
    last = selected[-1]
    for key, expected in (("left_w_rad_s", last["left_w_after_rad_s"]), ("right_w_rad_s", last["right_w_after_rad_s"]),
                           ("body_yaw_rate_rad_s", last["body_yaw_rate_rad_s"]),
                           ("body_linear_speed_m_s", math.sqrt(sum(last[field]**2 for field in ("body_vx_m_s", "body_vy_m_s", "body_vz_m_s"))))):
        close(evidence[key], expected, "Stop evidence is supported by the actual raw endpoint", 1e-8)
    return {"stage": stage, "actual_stop_steps": len(selected), "actual_continuous_stopped_steps": consecutive,
            "last_actual_sdk_state": last}


def phase_command(parameters, phase):
    return (0. if phase == "spin" else parameters["forward_speed_m_s"],
            0. if phase == "straight" else parameters["spin_yaw_rate_rad_s" if phase == "spin" else "arc_yaw_rate_rad_s"])


def independent_phase(records, item, parameters, model, sha):
    phase, baseline, stops = item["phase"], item["baseline"], item["stop_evidence"]
    for stop in stops.values():
        check_stop(stop)
    rows = numeric(records)
    require(rows, "Actual drive phase has no trace")
    maximum_encoder_discontinuity = 0.
    for index, row in enumerate(rows):
        require(row["phase"] == phase and row["config_sha256"] == sha, "Raw drive phase/configuration SHA differs")
        require(all(math.isfinite(value) for value in row.values() if isinstance(value, float)), "Non-finite actual drive trace")
        close(row["phase_step"], index+1, "Actual consecutive phase steps")
        close(row["phase_elapsed_s"], (index+1)*DT, "Explicit post-step phase elapsed time")
        close(row["dt_s"], DT, "Actual physics interval", 1e-9)
        close(row["sim_time_s"]-rows[0]["sim_time_s"], index*DT, "Actual advancing world clock", 1e-8)
        close(row["legacy_pose_x"], row["command_v_m_s"], "Real Classic Pose linear encoding")
        close(row["legacy_pose_yaw"], -row["command_yaw_rate_rad_s"], "Real Classic Pose legacy angular sign")
        if index:
            maximum_encoder_discontinuity = max(maximum_encoder_discontinuity, encoder_continuity(rows[index-1], row))
    selected = [row for row in rows if row["stage"] == "measure"]
    require(len(selected) == 2000, "Constant-velocity window must have 2000 real 1 ms samples")
    require(sum(row["stage"] == "ramp_up" for row in rows) == 500
            and sum(row["stage"] == "ramp_down" for row in rows) == 500
            and sum(row["stage"] == "settling" for row in rows) >= 300,
            "Preparation ramp/settling time was included in or omitted from the declared experiment")
    actual_stops = [independent_stop(rows, stage, stops[key])
                    for stage, key in (("stopping_initial", "initial"), ("stopping_final", "final"))]
    first = selected[0]
    for name, field in (("gt_x_m", "gt_x_before_m"), ("gt_y_m", "gt_y_before_m"), ("gt_yaw_rad", "gt_yaw_before_rad"),
                         ("left_q_rad", "left_q_before_rad"), ("right_q_rad", "right_q_before_rad"), ("sim_time_s", "sim_time_s")):
        close(float(baseline[name]), first[field], "Actual measurement start baseline "+name)
    v, omega = phase_command(parameters, phase)
    expected_left, expected_right = (v-omega*SEPARATION/2)/RADIUS, (v+omega*SEPARATION/2)/RADIUS
    encoder, wrongly_calibrated = [0., 0., 0.], [0., 0., 0.]
    yaw = path = left_travel = right_travel = 0.
    max_xy = max_yaw = max_encoder_xy = max_encoder_yaw = lateral = max_wheel_error = 0.
    max_encoder_replay = max_endpoint_velocity_residual = max_heading = 0.
    endpoint_diagnostics = {side: {"actual_encoder_increment_rad": 0., "pre_velocity_integral_rad": 0.,
                                   "post_velocity_integral_rad": 0., "maximum_absolute_trapezoid_residual_rad": 0.}
                            for side in ("left", "right")}
    c, s = math.cos(baseline["gt_yaw_rad"]), math.sin(baseline["gt_yaw_rad"])
    crossings = 0
    for index, row in enumerate(selected):
        close(row["measurement_step"], index+1, "Actual measurement step")
        close(row["measurement_elapsed_s"], (index+1)*DT, "Actual measurement interval")
        close(row["sim_time_s"]-first["sim_time_s"], index*DT, "Actual constant-window world time", 1e-8)
        close(row["command_v_m_s"], v, "Constant measured v command")
        close(row["command_yaw_rate_rad_s"], omega, "Constant measured physical CCW omega command")
        if index:
            old = selected[index-1]
            for before, after in (("gt_x_before_m", "gt_x_after_m"), ("gt_y_before_m", "gt_y_after_m")):
                close(row[before], old[after], "Actual axle-center pre/post continuity", 1e-7)
            close(angle_delta(row["gt_yaw_before_rad"]-old["gt_yaw_after_rad"]), 0., "Actual yaw pre/post continuity", 1e-7)
        increments = []
        for side, expected in (("left", expected_left), ("right", expected_right)):
            actual_speed = row[side+"_w_after_rad_s"]
            error = abs(actual_speed-expected)
            max_wheel_error = max(max_wheel_error, error)
            require(error <= max(.1, .05*abs(expected)), "Actual wheel speed did not follow the stock mapping: "+side)
            dq = angle_delta(row[side+"_q_after_rad"]-row[side+"_q_before_rad"])
            # QuickStep integrates pose with temporary ERP-corrected velocity,
            # then restores non-ERP velocity before GetVelocity is observed.
            # Endpoints therefore do not give a valid bound for this residual.
            # Source: gazebo11_11.15.1/deps/opende/src/quickstep_update_bodies.cpp
            # lines 60-66, 237, 240-262. This is a diagnostic, never a hard gate
            # or a claim to isolate all ERP contributions.
            q_residual = dq-DT*(row[side+"_w_before_rad_s"]+actual_speed)/2
            max_endpoint_velocity_residual = max(max_endpoint_velocity_residual, abs(q_residual))
            diagnostic = endpoint_diagnostics[side]
            diagnostic["actual_encoder_increment_rad"] += dq
            diagnostic["pre_velocity_integral_rad"] += DT*row[side+"_w_before_rad_s"]
            diagnostic["post_velocity_integral_rad"] += DT*actual_speed
            diagnostic["maximum_absolute_trapezoid_residual_rad"] = max(
                diagnostic["maximum_absolute_trapezoid_residual_rad"], abs(q_residual))
            increments.append(RADIUS*dq)
        dl, dr = increments
        left_travel += dl
        right_travel += dr
        encoder = advance(encoder, (dl+dr)/2, (dr-dl)/SEPARATION)
        wrongly_calibrated = advance(wrongly_calibrated, (dl+dr)/2, (dr-dl)/.48)
        for actual, field in zip(encoder, ("encoder_x_m", "encoder_y_m", "encoder_yaw_rad")):
            close(row[field], actual, "Independent actual wheel-encoder SE2 replay", 2e-6)
            max_encoder_replay = max(max_encoder_replay, abs(row[field]-actual))
        raw_yaw_change = row["gt_yaw_after_rad"]-row["gt_yaw_before_rad"]
        crossings += abs(raw_yaw_change) > math.pi
        yaw += angle_delta(raw_yaw_change)
        max_heading = max(max_heading, abs(yaw))
        close(row["gt_yaw_unwrapped_rad"], yaw, "Independent actual ground-truth yaw unwrap", 2e-6)
        dx, dy = row["gt_x_after_m"]-baseline["gt_x_m"], row["gt_y_after_m"]-baseline["gt_y_m"]
        gx, gy = c*dx+s*dy, -s*dx+c*dy
        path += math.hypot(row["gt_x_after_m"]-row["gt_x_before_m"], row["gt_y_after_m"]-row["gt_y_before_m"])
        predicted = ideal(v, omega, (index+1)*DT)
        max_xy, max_yaw = max(max_xy, math.hypot(gx-predicted[0], gy-predicted[1])), max(max_yaw, abs(yaw-predicted[2]))
        max_encoder_xy, max_encoder_yaw = max(max_encoder_xy, math.hypot(gx-encoder[0], gy-encoder[1])), max(max_encoder_yaw, abs(yaw-encoder[2]))
        lateral = max(lateral, abs(gy))
    net, gap = math.hypot(gx, gy), math.hypot(gx-encoder[0], gy-encoder[1])
    mode = model["surface_mode"]
    if mode == "slip":
        require(phase == "straight" and (left_travel+right_travel)/2 >= .35 and abs(left_travel-right_travel) <= .02,
                "Slip demonstration lacks substantial approximately equal actual wheel travel")
        require(path <= .10 and net <= .08 and max_heading <= .2 and gap >= .25,
                "Low friction did not produce the predeclared real body-path/encoder disagreement")
    else:
        require(max_xy <= .04 and max_yaw <= .08 and max_encoder_xy <= .04 and max_encoder_yaw <= .08,
                "Normal-contact actual trajectory exceeds the full-window ideal/encoder error bounds")
        if phase == "straight":
            require(lateral <= .03 and max_heading <= .05, "Actual straight motion has excessive full-window lateral/heading error")
        if phase == "spin":
            require(net <= .04, "Actual in-place turn translated too far")
    close(wrongly_calibrated[2], encoder[2]*SEPARATION/.48, "Same-trace wrong-separation yaw calibration", 1e-10)
    for diagnostic in endpoint_diagnostics.values():
        diagnostic["trapezoid_velocity_integral_rad"] = (diagnostic["pre_velocity_integral_rad"]+diagnostic["post_velocity_integral_rad"])/2
        diagnostic["signed_encoder_minus_trapezoid_rad"] = diagnostic["actual_encoder_increment_rad"]-diagnostic["trapezoid_velocity_integral_rad"]
    result = {"phase": phase, "actual_measurement_samples": len(selected), "duration_s": 2.,
              "actual_baseline": baseline, "actual_first_measurement": first, "actual_last_measurement": selected[-1],
              "actual_local_ground_truth_SE2": [gx, gy, yaw], "command_SE2_reference": ideal(v, omega, 2.),
              "independent_encoder_local_SE2": encoder, "left_actual_wheel_travel_m": left_travel,
              "right_actual_wheel_travel_m": right_travel, "actual_body_path_length_m": path,
              "actual_body_net_displacement_m": net, "actual_final_encoder_GT_gap_m": gap,
              "full_window_maximum_ideal_xy_error_m": max_xy, "full_window_maximum_ideal_yaw_error_rad": max_yaw,
              "full_window_maximum_encoder_GT_xy_error_m": max_encoder_xy, "full_window_maximum_encoder_GT_yaw_error_rad": max_encoder_yaw,
              "maximum_lateral_m": lateral, "maximum_actual_wheel_speed_mapping_error_rad_s": max_wheel_error,
              "maximum_absolute_actual_heading_change_rad": max_heading, "independent_actual_stops": actual_stops,
              "maximum_encoder_replay_error": max_encoder_replay,
              "maximum_all_stage_encoder_cross_row_discontinuity_rad": maximum_encoder_discontinuity,
              "maximum_endpoint_velocity_trapezoid_residual_rad": max_endpoint_velocity_residual,
              "encoder_endpoint_velocity_diagnostic": {"is_pass_fail_gate": False,
                                                        "does_not_isolate_or_quantify_all_ERP_contributions": True,
                                                        "source": "https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11_11.15.1/deps/opende/src/quickstep_update_bodies.cpp",
                                                        "per_wheel": endpoint_diagnostics},
              "actual_yaw_branch_crossings": crossings,
              "offline_wrong_separation_replay": {"new_physics_run": False, "source_episode": item["episode"],
                                                  "assumed_separation_m": .48, "actual_separation_m": SEPARATION,
                                                  "replayed_encoder_SE2": wrongly_calibrated,
                                                  "same_unchanged_ground_truth_SE2": [gx, gy, yaw]}}
    return result


def await_ready(session):
    deadline, last = time.monotonic()+15, None
    while time.monotonic() < deadline:
        require(session.running, "Stock differential-drive server exited before ready")
        try:
            last = read_json(control_directory(session)/"status.json")
            if last["model"]["stock_subscriber_connected"]:
                require(not last["error"], "Actual differential observer error: "+str(last["error"]))
                require(last["config_sha256"] == digest(session.workspace/"parameters.json")
                        and last["world_sha256"] == digest(session.workspace/"world.sdf")
                        and last["source_sha256"] == digest(session.lesson.folder/"drive_lab.cc"),
                        "Loaded differential experiment does not identify the current files")
                check_model(session.workspace, last)
                return last
        except (OSError, KeyError, ValueError):
            pass
        time.sleep(.05)
    raise AssertionError("No ready current stock differential-drive status: "+repr(last))


def verify_case(session, name, diagnostic=None):
    before = actual_stats(session)
    personal = source_hashes(session.workspace)
    trace = control_directory(session)/"drive.csv"
    old_trace = digest(trace)
    signature_keys = ("episode", "phase", "stage", "phase_step", "measurement_step", "config_sha256", "world_sha256")
    old_status = read_json(control_directory(session)/"status.json")
    old_signature = {key: old_status[key] for key in signature_keys}
    old_episodes = {row["episode"] for row in read_trace(session)}
    offset = session.log_path.stat().st_size
    result = session.verify()
    raw = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
    after = actual_stats(session)
    require(before["paused"] == after["paused"], "Differential verifier did not preserve actual pause")
    require(personal == source_hashes(session.workspace), "Differential verifier changed personal source files")
    item = {"name": name, "passed": False, "verifier_result": result, "raw_verifier_output": raw,
            "before_stats": before, "after_stats": after, "pause_preserved": True, "personal_files_unchanged": True}
    if diagnostic:
        require(before["paused"] and result["passed"] is False and diagnostic.lower() in raw.lower(),
                "Invalid or stale drive input lacks its relevant rejection: "+diagnostic+"\n"+raw)
        require(before["iterations"] == after["iterations"] and before["sim_time_sec_nsec"] == after["sim_time_sec_nsec"],
                "Invalid differential input changed physics before rejection")
        current = read_json(control_directory(session)/"status.json")
        require(old_signature == {key: current[key] for key in signature_keys} and old_trace == digest(trace),
                "Rejected input published a command, started a phase or appended samples")
        item["rejected_before_physics_or_command"] = True
    else:
        require(result["passed"] is True, "Canonical differential verification failed: "+raw)
        canonical = read_json(control_directory(session)/"verification.json")
        parameters = read_json(session.workspace/"parameters.json")
        source = surface_source(session.workspace)
        require(canonical["parameters"] == parameters and canonical["config_sha256"] == digest(session.workspace/"parameters.json")
                and canonical["world_sha256"] == digest(session.workspace/"world.sdf"), "Verification used stale private inputs")
        model_check = check_model(session.workspace, dict(old_status, model=canonical["actual_model"]))
        expected_phases = list(PHASES) if source["surface_mode"] == "normal" else ["straight"]
        require([phase["phase"] for phase in canonical["phases"]] == expected_phases, "Differential verification omitted its required phases")
        rows = read_trace(session)
        measured = []
        for phase in canonical["phases"]:
            require(phase["episode"] not in old_episodes, "Verification reused old motion instead of running a new actual phase")
            actual = [row for row in rows if row["episode"] == phase["episode"]]
            measured.append(independent_phase(actual, phase, parameters, canonical["actual_model"], canonical["config_sha256"]))
        item.update(canonical_measurements=canonical, independent_actual_phases=measured, actual_model_source_crosscheck=model_check)
    item["passed"] = True
    return item


def write_parameters(workspace, **changes):
    path = workspace/"parameters.json"
    values = read_json(path)
    values.update(changes)
    path.write_text(json.dumps(values, indent=2)+"\n", encoding="utf-8")


def set_wheel_friction(workspace, value):
    path = workspace/"world.sdf"
    tree = ET.parse(str(path))
    car = tree.getroot().find("world/model[@name='differential_car']")
    for name in ("left_wheel", "right_wheel"):
        for key in ("mu", "mu2"):
            car.find("link[@name='%s']/collision/surface/friction/ode/%s" % (name, key)).text = str(value)
    tree.write(str(path), encoding="utf-8", xml_declaration=True)


def invalid_group(session):
    params, world = session.workspace/"parameters.json", session.workspace/"world.sdf"
    valid_params, valid_world = params.read_bytes(), world.read_bytes()
    items = []
    for name, changes, diagnosis in (("nonfinite_linear_command", {"forward_speed_m_s": float("nan")}, "forward_speed_m_s"),
                                      ("outside_supported_angular_command", {"arc_yaw_rate_rad_s": -1.}, "arc_yaw_rate_rad_s")):
        try:
            write_parameters(session.workspace, **changes)
            items.append(verify_case(session, name, diagnosis))
        finally:
            params.write_bytes(valid_params)
    try:
        tree = ET.parse(str(world))
        reference = tree.getroot().find("world/model[@name='differential_car']/plugin[@name='native_differential_drive']/left_joint")
        require(reference is not None, "No actual stock left_joint reference in current source")
        reference.text = "missing_joint"
        tree.write(str(world), encoding="utf-8", xml_declaration=True)
        items.append(verify_case(session, "missing_stock_joint_reference_rejected_before_start", "world.sdf"))
    finally:
        world.write_bytes(valid_world)
    return {"name": "invalid_commands_and_missing_stock_joint_rejected_before_control", "passed": True,
            "checks": items, "original_bytes_restored": params.read_bytes() == valid_params and world.read_bytes() == valid_world}


def stale_file_group(session):
    params, world = session.workspace/"parameters.json", session.workspace/"world.sdf"
    old_params, old_world = params.read_bytes(), world.read_bytes()
    items = []
    try:
        write_parameters(session.workspace, forward_speed_m_s=.3)
        items.append(verify_case(session, "valid_parameter_saved_but_not_reloaded", "重载"))
    finally:
        params.write_bytes(old_params)
    try:
        set_wheel_friction(session.workspace, .001)
        items.append(verify_case(session, "valid_surface_saved_but_not_reloaded", "重载"))
    finally:
        world.write_bytes(old_world)
    return {"name": "valid_private_files_require_explicit_reload", "passed": True, "checks": items}


def reset_from_real_motion(session):
    """Use the public model reset while stock still holds a nonzero command."""
    deadline = time.monotonic()+40
    session.pause(True)
    personal = source_hashes(session.workspace)
    verified_count = sum(item.get("event") == "verified" for item in session.storage.records())
    episode = "drive-before-reset-"+uuid.uuid4().hex
    request_phase(session, episode, "straight")
    acknowledgements = [bounded_steps(session, 1200, deadline)]
    moving = read_json(control_directory(session)/"status.json")
    require(moving["episode"] == episode and moving["active"] is True and not moving["error"], "Reset setup did not activate a real stock motion phase")
    old = numeric([row for row in read_trace(session) if row["episode"] == episode])
    require(old and abs(old[-1]["command_v_m_s"]) > .1 and abs(old[-1]["left_w_after_rad_s"]) > 1.
            and abs(old[-1]["right_w_after_rad_s"]) > 1.
            and math.hypot(old[-1]["body_vx_m_s"], old[-1]["body_vy_m_s"]) > .05,
            "Reset regression would be a false positive because the car was already stopped")
    before = actual_stats(session)
    session.reset_models()
    after_reset = actual_stats(session)
    require(after_reset["paused"] and after_reset["sim_time"] >= before["sim_time"]
            and after_reset["iterations"] >= before["iterations"], "Model reset changed pause or rewound world time")
    acknowledgements.append(bounded_steps(session, 200, deadline))
    stopped = read_json(control_directory(session)/"status.json")
    require(stopped["episode"] != episode and stopped["episode"].startswith("reset-")
            and stopped["phase"] == "reset" and stopped["stage"] == "stopped"
            and stopped["active"] is False and stopped["finished"] is True
            and stopped["phase_complete"] is False and not stopped["error"],
            "Public reset did not terminate the active experiment and acknowledge the actual stopped state")
    evidence = stopped["reset_evidence"]
    require(evidence["zero_published"] is True and evidence["zero_confirmed"] is True,
            "Public reset did not distinguish actual zero-command acknowledgement from publication")
    require(evidence["episode"] == stopped["episode"]
            and evidence["count"] > moving["reset_evidence"]["count"],
            "Reset acknowledgement reused evidence from an earlier public reset")
    check_stop(stopped["stop_evidence"]["final"])
    reset_rows = numeric([row for row in read_trace(session) if row["episode"] == stopped["episode"]])
    actual_stop = independent_stop(reset_rows, "reset-stopping", stopped["stop_evidence"]["final"])
    for row in reset_rows:
        require(row["measurement_step"] == 0, "Reset pose jump was appended to an old measurement window")
        for key in ("encoder_x_m", "encoder_y_m", "encoder_yaw_rad", "gt_yaw_unwrapped_rad"):
            close(row[key], 0., "Reset episode must not integrate the public model-pose jump", 1e-8)
    require(verified_count == sum(item.get("event") == "verified" for item in session.storage.records()),
            "Reset stopping acknowledgement was incorrectly counted as a completed course verification")
    require(personal == source_hashes(session.workspace), "Reset overwrote personal inputs")
    return {"name": "public_reset_clears_real_nonzero_stock_command", "passed": True,
            "actual_moving_state": old[-1], "before_reset_stats": before, "after_reset_stats": after_reset,
            "bounded_step_acknowledgements": acknowledgements, "native_reset_evidence": evidence,
            "independent_actual_stop": actual_stop, "new_reset_episode": stopped["episode"],
            "additional_verification_recorded": False, "test_writes_body_pose_or_velocity": False}


def configuration_regressions(lesson, private):
    module = types.ModuleType("_l11_config_probe")
    module.__file__ = str(lesson.folder/"config.py")
    exec(compile((lesson.folder/"config.py").read_bytes(), module.__file__, "exec"), module.__dict__)
    folder = private/"config-check"
    folder.mkdir()
    for name in ("parameters.json", "world.sdf"):
        (folder/name).write_bytes((lesson.folder/name).read_bytes())
    original = (folder/"world.sdf").read_bytes()
    normal = module.read_world(folder/"world.sdf", lesson.folder/"world.sdf")
    set_wheel_friction(folder, .001)
    slip = module.read_world(folder/"world.sdf", lesson.folder/"world.sdf")
    require(normal["surface_mode"] == "normal" and slip["surface_mode"] == "slip", "Editable wheel surface classifier is incorrect")
    path = folder/"world.sdf"
    tree = ET.parse(str(path))
    tree.getroot().find("world/model[@name='differential_car']/link[@name='left_wheel']/collision/surface/friction/ode/mu").text = ".8"
    tree.write(str(path), encoding="utf-8", xml_declaration=True)
    try:
        module.read_world(path, lesson.folder/"world.sdf")
    except ValueError as error:
        require("mixed" in str(error), "Mixed-wheel surface refusal lacks its relevant diagnosis")
        diagnosis = str(error)
    else:
        raise AssertionError("Mixed normal/slip wheels were accepted as the controlled friction comparison")
    path.write_bytes(original)
    require(module.read_world(path, lesson.folder/"world.sdf") == normal, "Restored actual world was not reaccepted")
    write_parameters(folder, forward_speed_m_s=.35)
    changed = module.read_parameters(folder/"parameters.json")
    return {"kind": "configuration_only_not_physics", "normal_accepted": normal, "slip_accepted": slip,
            "mixed_wheel_surface_rejected": diagnosis, "restored_bytes_reaccepted": True, "changed_speed_accepted": changed}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--math-only", action="store_true")
    mode.add_argument("--configuration-only", action="store_true")
    args = parser.parse_args()
    report = {"passed": False, "lesson": KEY, "physical_sessions_started": 0, "scenarios": [], "math_regressions": math_regressions()}
    output = Path(args.report).expanduser().resolve() if args.report else None
    artifacts = output.with_name(output.stem+"-artifacts") if output else Path(tempfile.mkdtemp(prefix="gazeboteach-11-evidence-"))
    sessions, lesson, source_before = [], None, None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-11-private-") as folder:
        private = Path(folder)
        try:
            if not args.math_only:
                lessons, errors = load_catalog(ROOT)
                candidates = [candidate for candidate in lessons if candidate.key == KEY]
                require(len(candidates) == 1, "Expected one L11 course: "+repr(errors))
                lesson = candidates[0]
                source_before = source_hashes(lesson.folder)
                report["configuration_regressions"] = configuration_regressions(lesson, private)
            if not args.math_only and not args.configuration_only:
                baseline = LessonSession(ROOT, lesson, storage=Storage(private/"normal"))
                sessions.append(("normal", baseline))
                report["physical_sessions_started"] += 1
                baseline.start(gui=False, terminals=False)
                report["normal_initial_status"] = await_ready(baseline)
                baseline.pause(False)
                normal = verify_case(baseline, "normal_straight_spin_arc_actual_motion")
                report["scenarios"].append(normal)
                baseline.stop(reason="L11 baseline accepted")

                storage = Storage(private/"changed_speed")
                practice = storage.prepare(lesson)
                write_parameters(practice, forward_speed_m_s=.35)
                changed = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("changed_speed", changed))
                report["physical_sessions_started"] += 1
                changed.start(gui=False, terminals=False)
                report["changed_speed_initial_status"] = await_ready(changed)
                changed.pause(True)
                variant = verify_case(changed, "only_forward_speed_changed_to_point35")
                report["scenarios"].append(variant)
                old_straight = normal["independent_actual_phases"][0]["actual_local_ground_truth_SE2"][0]
                new_straight = variant["independent_actual_phases"][0]["actual_local_ground_truth_SE2"][0]
                require(new_straight-old_straight > .12, "Actual changed forward command did not increase straight displacement")
                report["actual_single_command_effect"] = {"baseline_straight_m": old_straight, "changed_straight_m": new_straight,
                                                          "difference_m": new_straight-old_straight}
                report["scenarios"].append(invalid_group(changed))
                report["scenarios"].append(verify_case(changed, "repaired_changed_speed_world_reaccepted"))
                report["scenarios"].append(stale_file_group(changed))
                report["scenarios"].append(reset_from_real_motion(changed))
                changed.stop(reason="L11 variant and public reset accepted")

                storage = Storage(private/"slip")
                practice = storage.prepare(lesson)
                set_wheel_friction(practice, .001)
                slip = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("slip", slip))
                report["physical_sessions_started"] += 1
                slip.start(gui=False, terminals=False)
                report["slip_initial_status"] = await_ready(slip)
                slip.pause(True)
                report["scenarios"].append(verify_case(slip, "actual_low_friction_wheels_spin_body_moves_less"))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"] = {}
            for name, session in reversed(sessions):
                session.stop(reason="L11 private acceptance finished")
                report["evidence"][name] = preserve_drive_evidence(session, artifacts/name)
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers, source_templates_unchanged=(source_before == source_hashes(lesson.folder)) if lesson else None,
                          elapsed_seconds=round(time.monotonic()-started, 3))
            if any(leftovers.values()) or report["source_templates_unchanged"] is False:
                report["passed"] = False
            if output:
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
