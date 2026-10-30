"""Independent SE(2), wheel encoder and actual ground-truth measurements."""
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

OFFICIAL = Path(os.environ.get("GAZEBOTEACH_LESSON_SOURCE", Path(__file__).resolve().parent)).resolve()
WORKSPACE = Path(os.environ.get("GAZEBOTEACH_WORKSPACE", Path.cwd())).resolve()
sys.path.insert(0, str(OFFICIAL))
from config import PHASES, digest, read_parameters, read_world, validate_model
sys.path.insert(0, str(OFFICIAL.parent/"_control_support"))
from pause import read_pause, set_pause


def directory():
    if not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("请在本课实验终端中运行，缺少会话环境")
    return Path(os.environ["GAZEBOTEACH_SESSION_DIR"])/"control"


def status():
    path = directory()/"status.json"
    if not path.is_file():
        raise RuntimeError("真实差速观测器尚无状态，请查看构建和运行日志")
    return json.loads(path.read_text(encoding="utf-8"))


def inspect():
    p = read_parameters(WORKSPACE/"parameters.json")
    world = read_world(WORKSPACE/"world.sdf", OFFICIAL/"world.sdf")
    current = status()
    actual = (digest(WORKSPACE/"parameters.json"), digest(WORKSPACE/"world.sdf"), digest(OFFICIAL/"drive_lab.cc"))
    loaded = tuple(current[key] for key in ("config_sha256", "world_sha256", "source_sha256"))
    if actual != loaded:
        raise AssertionError("parameters/world/source SHA已改变，运行实例尚未重载 reload")
    if current["parameters"] != p:
        raise AssertionError("实际加载参数不同，请重载")
    stock = Path("/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libDiffDrivePlugin.so")
    if not stock.is_file() or digest(stock) != current["stock_plugin_sha256"]:
        raise AssertionError("实际stock插件文件与本实例构建指纹不同")
    validate_model(current["model"], world)
    if current["error"]:
        raise AssertionError("实际差速实验错误："+current["error"])
    return current


def rows_for(episode):
    with (directory()/"drive.csv").open(newline="", encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row["episode"] == episode]


def delta(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


def se2(v, omega, duration):
    if abs(omega) < 1e-12:
        return v*duration, 0., omega*duration
    return v/omega*math.sin(omega*duration), v/omega*(1-math.cos(omega*duration)), omega*duration


def integrate(x, y, yaw, dl, dr, separation):
    ds, da = (dl+dr)/2, (dr-dl)/separation
    sinc = 1. if abs(da) < 1e-10 else math.sin(da/2)/(da/2)
    return x+ds*sinc*math.cos(yaw+da/2), y+ds*sinc*math.sin(yaw+da/2), yaw+da


def close(actual, expected, label, tolerance=1e-7):
    if not math.isfinite(actual) or abs(actual-expected) > tolerance*(1+abs(expected)):
        raise AssertionError("%s: actual=%g expected=%g" % (label, actual, expected))


def analyse(rows, phase, p, model, baseline, stops):
    if not rows:
        raise AssertionError("真实差速阶段没有样本")
    for name in ("initial", "final"):
        stop = stops[name]
        if not stop["confirmed"] or stop["consecutive_s"] < .05-1e-8 or stop["duration_s"] > 2+1e-8:
            raise AssertionError("真实零命令停稳未确认："+name)
        if abs(stop["left_w_rad_s"]) >= .1 or abs(stop["right_w_rad_s"]) >= .1 or stop["body_linear_speed_m_s"] >= .01 or abs(stop["body_yaw_rate_rad_s"]) >= .03:
            raise AssertionError("停稳ACK与实际速度不符")
    numeric = [key for key in rows[0] if key not in ("episode", "phase", "stage", "config_sha256")]
    parsed = [{key: float(row[key]) for key in numeric} for row in rows]
    for index, (raw, row) in enumerate(zip(rows, parsed), 1):
        if raw["phase"] != phase or raw["config_sha256"] != digest(WORKSPACE/"parameters.json"):
            raise AssertionError("真实阶段/参数SHA不一致")
        if not all(math.isfinite(x) for x in row.values()):
            raise AssertionError("实际差速CSV含非有限数")
        close(row["phase_step"], index, "actual consecutive phase step")
        close(row["phase_elapsed_s"], index*.001, "explicit post-step phase time")
        close(row["dt_s"], .001, "actual physics dt", 1e-9)
        close(row["legacy_pose_x"], row["command_v_m_s"], "actual Pose linear encoding")
        close(row["legacy_pose_yaw"], -row["command_yaw_rate_rad_s"], "actual legacy angular sign")
    selected = [row for raw, row in zip(rows, parsed) if raw["stage"] == "measure"]
    if len(selected) != 2000:
        raise AssertionError("恒速真实测量窗必须有2000步")
    v = 0. if phase == "spin" else p["forward_speed_m_s"]
    omega = 0. if phase == "straight" else p["spin_yaw_rate_rad_s" if phase == "spin" else "arc_yaw_rate_rad_s"]
    radius, separation = model["wheel_radius_m"], model["wheel_separation_m"]
    wl, wr = (v-omega*separation/2)/radius, (v+omega*separation/2)/radius
    first = selected[0]
    for key, actual in (("gt_x_m", first["gt_x_before_m"]), ("gt_y_m", first["gt_y_before_m"]),
                        ("gt_yaw_rad", first["gt_yaw_before_rad"]), ("left_q_rad", first["left_q_before_rad"]),
                        ("right_q_rad", first["right_q_before_rad"]), ("sim_time_s", first["sim_time_s"])):
        close(baseline[key], actual, "real measurement baseline/"+key)
    x = y = yaw = gt_yaw = path = dl_total = dr_total = 0.
    max_xy = max_yaw = max_encoder_xy = max_encoder_yaw = max_wheel_error = max_lateral = max_absolute_yaw = 0.
    previous_gt_yaw = first["gt_yaw_before_rad"]
    c, s = math.cos(baseline["gt_yaw_rad"]), math.sin(baseline["gt_yaw_rad"])
    final_gt = (0., 0., 0.)
    for index, row in enumerate(selected, 1):
        close(row["measurement_step"], index, "measurement step")
        close(row["measurement_elapsed_s"], index*.001, "measurement elapsed")
        close(row["command_v_m_s"], v, "constant published v")
        close(row["command_yaw_rate_rad_s"], omega, "constant published CCW omega")
        for key, want in (("left_w_after_rad_s", wl), ("right_w_after_rad_s", wr)):
            error = abs(row[key]-want)
            max_wheel_error = max(max_wheel_error, error)
            if error > max(.1, .05*abs(want)):
                raise AssertionError("实际轮速未响应原生差速映射："+key)
        dl = radius*delta(row["left_q_after_rad"]-row["left_q_before_rad"])
        dr = radius*delta(row["right_q_after_rad"]-row["right_q_before_rad"])
        dl_total += dl
        dr_total += dr
        x, y, yaw = integrate(x, y, yaw, dl, dr, separation)
        for key, value in (("encoder_x_m", x), ("encoder_y_m", y), ("encoder_yaw_rad", yaw)):
            close(row[key], value, "independent wheel encoder replay/"+key, 2e-6)
        gt_yaw += delta(row["gt_yaw_after_rad"]-previous_gt_yaw)
        previous_gt_yaw = row["gt_yaw_after_rad"]
        close(row["gt_yaw_unwrapped_rad"], gt_yaw, "independent groundtruth yaw unwrap", 2e-6)
        dx, dy = row["gt_x_after_m"]-baseline["gt_x_m"], row["gt_y_after_m"]-baseline["gt_y_m"]
        gx, gy = c*dx+s*dy, -s*dx+c*dy
        final_gt = gx, gy, gt_yaw
        path += math.hypot(row["gt_x_after_m"]-row["gt_x_before_m"], row["gt_y_after_m"]-row["gt_y_before_m"])
        predicted = se2(v, omega, index*.001)
        max_xy = max(max_xy, math.hypot(gx-predicted[0], gy-predicted[1]))
        max_yaw = max(max_yaw, abs(gt_yaw-predicted[2]))
        max_absolute_yaw = max(max_absolute_yaw, abs(gt_yaw))
        max_encoder_xy = max(max_encoder_xy, math.hypot(gx-x, gy-y))
        max_encoder_yaw = max(max_encoder_yaw, abs(gt_yaw-yaw))
        max_lateral = max(max_lateral, abs(gy))
    net = math.hypot(final_gt[0], final_gt[1])
    gap = math.hypot(x-final_gt[0], y-final_gt[1])
    metrics = {"measurement_samples": 2000, "duration_s": 2., "command_v_m_s": v, "command_yaw_rate_rad_s": omega,
               "predicted_left_rad_s": wl, "predicted_right_rad_s": wr,
               "predicted_local_se2": list(se2(v, omega, 2)), "actual_local_se2": list(final_gt),
               "encoder_local_se2": [x, y, yaw], "left_travel_m": dl_total, "right_travel_m": dr_total,
               "mean_wheel_travel_m": (dl_total+dr_total)/2, "wheel_max_mapping_error_rad_s": max_wheel_error,
               "gt_path_length_m": path, "gt_net_displacement_m": net,
               "maximum_ideal_xy_error_m": max_xy, "maximum_ideal_yaw_error_rad": max_yaw,
               "maximum_encoder_gt_xy_error_m": max_encoder_xy, "maximum_encoder_gt_yaw_error_rad": max_encoder_yaw,
               "final_encoder_gt_xy_gap_m": gap, "maximum_lateral_m": max_lateral,
               "maximum_absolute_gt_yaw_change_rad": max_absolute_yaw,
               "raw_left_angle_span_rad": max(row["left_q_after_rad"] for row in selected)-min(row["left_q_after_rad"] for row in selected)}
    if model["surface_mode"] == "slip":
        if phase != "straight" or (dl_total+dr_total)/2 < .35 or abs(dl_total-dr_total) > .02 or path > .10 or net > .08 or max_absolute_yaw > .2 or gap < .25:
            raise AssertionError("低摩擦未实际呈现轮转/车体路径/编码器漂移对照："+json.dumps(metrics))
    else:
        if max_xy > .04 or max_yaw > .08 or max_encoder_xy > .04 or max_encoder_yaw > .08:
            raise AssertionError("正常接触下真实车体偏离SE2/编码器范围："+json.dumps(metrics))
        if phase == "straight" and (max_lateral > .03 or max_yaw > .05):
            raise AssertionError("真实直行横向/方向误差过大")
        if phase == "spin" and net > .04:
            raise AssertionError("真实原地旋转发生过大平移")
    return metrics


def _phase(phase, timeout=45):
    initial = inspect()
    if phase not in PHASES or (initial["model"]["surface_mode"] == "slip" and phase != "straight"):
        raise ValueError("当前表面不支持此有限阶段："+phase)
    episode = "drive-"+uuid.uuid4().hex
    request = directory()/"phase.request"
    temporary = request.with_suffix(".tmp")
    temporary.write_text(episode+" "+phase+"\n", encoding="ascii")
    os.replace(str(temporary), str(request))
    deadline = time.monotonic()+timeout
    while True:
        current = status()
        if current["error"]:
            raise AssertionError("真实差速阶段失败："+current["error"])
        if current["episode"] == episode and current["phase_complete"]:
            break
        if time.monotonic() > deadline:
            raise RuntimeError("真实差速阶段超时，保留CSV/状态："+phase)
        time.sleep(.06)
    inspect()
    rows = rows_for(episode)
    result = {"phase": phase, "episode": episode, "samples": len(rows), "baseline": current["baseline"],
              "stop_evidence": current["stop_evidence"],
              "metrics": analyse(rows, phase, initial["parameters"], initial["model"], current["baseline"], current["stop_evidence"])}
    print("真实差速阶段完成："+phase, flush=True)
    return result


def _preserve(function, output):
    initial = inspect()
    original = read_pause()
    try:
        set_pause(False)
        result = function(initial)
    finally:
        cleanup = set_pause(original)
        if not cleanup["confirmed"]:
            raise RuntimeError("Cannot confirm original simulator pause after drive measurement")
    result.update(original_pause=original, pause_cleanup=cleanup)
    (directory()/output).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def experiment():
    def perform(initial):
        phases = PHASES if initial["model"]["surface_mode"] == "normal" else ("straight",)
        measured = [_phase(phase) for phase in phases]
        return {"passed": True, "parameters": initial["parameters"], "config_sha256": initial["config_sha256"],
                "world_sha256": initial["world_sha256"], "source_sha256": initial["source_sha256"],
                "actual_model": initial["model"], "surface_mode": initial["model"]["surface_mode"], "phases": measured,
                "telemetry_csv": str(directory()/"drive.csv"), "status_json": str(directory()/"status.json"),
                "time_convention": "pre/post actual physics states; measurement_elapsed_s=completed measurement steps*0.001; world time kept separately"}
    return _preserve(perform, "verification.json")


def run_phase(phase):
    return _preserve(lambda initial: {"passed": True, "parameters": initial["parameters"], "phases": [_phase(phase)],
                                     "telemetry_csv": str(directory()/"drive.csv")}, "phase-validation.json")


def replay_calibration(assumed_separation=.48):
    current = inspect()
    if not math.isfinite(assumed_separation) or not .2 <= assumed_separation <= .6:
        raise ValueError("assumed wheel separation must be finite within [.2,.6]")
    rows = rows_for(current["episode"])
    selected = [row for row in rows if row["stage"] == "measure"]
    if len(selected) != 2000 or not current["phase_complete"]:
        raise ValueError("先完成一个真实有限阶段，再离线重放标定参数")
    x = y = yaw = 0.
    for row in selected:
        dl = current["model"]["wheel_radius_m"]*delta(float(row["left_q_after_rad"])-float(row["left_q_before_rad"]))
        dr = current["model"]["wheel_radius_m"]*delta(float(row["right_q_after_rad"])-float(row["right_q_before_rad"]))
        x, y, yaw = integrate(x, y, yaw, dl, dr, assumed_separation)
    return {"offline_replay_only": True, "new_simulation_started": False, "episode": current["episode"],
            "actual_separation_m": current["model"]["wheel_separation_m"], "assumed_separation_m": assumed_separation,
            "replayed_encoder_local_se2": [x, y, yaw], "source_trace": str(directory()/"drive.csv")}
