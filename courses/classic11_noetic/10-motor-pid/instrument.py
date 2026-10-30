"""Trusted measurement of actual native Classic PID and rotor dynamics."""
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
from config import PHASES, digest, read_parameters, validate_model
sys.path.insert(0, str(OFFICIAL.parent/"_control_support"))
from pause import read_pause, set_pause


def directory():
    if not os.environ.get("GAZEBOTEACH_SESSION_DIR"):
        raise RuntimeError("请在本课已启动的实验终端中运行，缺少会话环境")
    return Path(os.environ["GAZEBOTEACH_SESSION_DIR"])/"control"


def status():
    path = directory()/"status.json"
    if not path.is_file():
        raise RuntimeError("原生关节控制器尚无状态，请查看构建和运行日志")
    return json.loads(path.read_text(encoding="utf-8"))


def inspect():
    # Validate the actual current disk configuration before any control or SHA gate.
    p = read_parameters(WORKSPACE/"parameters.json")
    if digest(WORKSPACE/"world.sdf") != digest(OFFICIAL/"world.sdf"):
        raise AssertionError("world.sdf已改变：当前测量固定转子模型，请恢复模板并重载")
    current = status()
    current["current_config_sha256"] = digest(WORKSPACE/"parameters.json")
    current["reload_required"] = (current["current_config_sha256"] != current["config_sha256"] or
                                  digest(WORKSPACE/"world.sdf") != current["world_sha256"] or
                                  digest(OFFICIAL/"motor_lab.cc") != current["source_sha256"])
    if current["reload_required"]:
        raise AssertionError("parameters/source/world SHA已改变但运行控制器仍是旧版本，请先重载 reload")
    if current["parameters"] != p:
        raise AssertionError("实际加载的控制参数与当前parameters.json不一致，请重载")
    validate_model(current["model"])
    if current["error"]:
        raise AssertionError("实际控制器错误："+current["error"])
    return current


def rows_for(episode):
    with (directory()/"control.csv").open(newline="", encoding="utf-8") as handle:
        return [row for row in csv.DictReader(handle) if row["episode"] == episode]


def _close(actual, expected, name, tolerance=1e-7):
    if not math.isfinite(actual) or abs(actual-expected) > tolerance*(1+abs(expected)):
        raise AssertionError("%s: actual=%g expected=%g" % (name, actual, expected))


def analyse(rows, name, p, reset):
    duration = dict(PHASES)[name]
    if reset.get("initializer_method") != "captured_absolute_link_pose_then_native_counter_sync" or not reset.get("counter_sync_called") or reset.get("counter_sync_calls") != 1:
        raise AssertionError("Each phase requires exactly one full native state initialization")
    for key in ("captured_initial_world_pose", "pre_counter_world_pose", "post_world_pose"):
        pose = reset[key]
        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
            raise AssertionError("Missing finite full world pose: "+key)
        for observed, expected in zip(pose[:3], (0., 0., .6)):
            _close(observed, expected, "actual initializer world position", 1e-9)
        if abs(abs(pose[3])-1) > 1e-9 or any(abs(value) > 1e-9 for value in pose[4:]):
            raise AssertionError("Actual initializer orientation differs from captured identity")
    _close(reset["post_q_rad"], 0., "actual SDK q after full initialization")
    _close(reset["post_omega_rad_s"], 0., "actual SDK velocity after full initialization")
    if reset["counter_offset_removed"] != (abs(reset["pre_counter_q_rad"]) > 1e-7):
        raise AssertionError("Counter synchronization and removed-offset flags disagree")
    if len(rows) != round(duration/.001):
        raise AssertionError("完整真实阶段样本数错误："+name+" "+str(len(rows)))
    numeric = [key for key in rows[0] if key not in ("episode", "phase", "controller_mode", "config_sha256")]
    values = [{key: float(row[key]) for key in numeric} for row in rows]
    if not all(math.isfinite(value) for row in values for value in row.values()):
        raise AssertionError("原生CSV含非有限状态/控制量")
    if any(abs(float(reset[key])) > 1e-12 for key in ("p_error", "i_error", "d_error", "command_Nm")) or not reset["commands_empty"]:
        raise AssertionError("实际原生PID未在阶段开始前完全清空")
    _close(values[0]["q_before_rad"], 0., "actual phase initial q")
    _close(values[0]["omega_before_rad_s"], 0., "actual phase initial velocity")
    cap = p["saturation_limit_Nm"] if name == "saturation" else p["command_limit_Nm"]
    pg = p["speed_p_kp"] if name == "speed-p" else (p["speed_pi_kp"] if name in ("speed-pi", "saturation") else p["position_kp"])
    ig = p["speed_pi_ki"] if name in ("speed-pi", "saturation") else 0.
    dg = p["position_kd"] if name == "position-pd" else 0.
    integral, previous = 0., 0.
    max_balance, total_force_impulse = 0., 0.
    momentum = 0.
    previous_q = values[0]["q_before_rad"]
    previous_unwrapped = previous_q
    config_sha = digest(WORKSPACE/"parameters.json")
    for index, (raw_row, row) in enumerate(zip(rows, values), 1):
        if raw_row["phase"] != name or raw_row["config_sha256"] != config_sha:
            raise AssertionError("实际运行阶段或参数指纹不一致")
        _close(row["phase_step"], index, "phase step")
        _close(row["phase_elapsed_s"], index*.001, "explicit post-step elapsed time")
        _close(row["dt_s"], .001, "actual native controller dt", 1e-10)
        expected_load = p["load_torque_Nm"] if name in ("speed-p", "speed-pi", "saturation") and index > 2000 else 0.
        _close(row["load_torque_Nm"], expected_load, "actual requested external load")
        if name == "torque":
            expected = p["torque_command_Nm"]
            _close(row["motor_command_Nm"], expected, "native persistent force")
        else:
            error = (row["q_before_rad"]-p["position_target_rad"] if name.startswith("position")
                     else row["omega_before_rad_s"]-p["velocity_target_rad_s"])
            integral += .001*error
            if ig:
                term = max(-p["integral_term_limit_Nm"], min(p["integral_term_limit_Nm"], ig*integral))
                integral = term/ig
            derivative = (error-previous)/.001
            previous = error
            expected = -(pg*error+ig*integral+dg*derivative)
            for key, want in (("pid_p_error", error), ("pid_i_error", integral), ("pid_d_error", derivative),
                              ("pid_p_gain", pg), ("pid_i_gain", ig), ("pid_d_gain", dg),
                              ("pid_i_term_Nm", ig*integral), ("pid_raw_command_Nm", expected)):
                _close(row[key], want, "native PID replay/"+key, 2e-6)
            _close(row["motor_command_Nm"], max(-cap, min(cap, expected)), "native output clamp", 2e-6)
        _close(row["joint_applied_Nm"], row["motor_command_Nm"]-row["load_torque_Nm"], "actual SetForce ledger", 1e-8)
        residual = .03125*(row["omega_after_rad_s"]-row["omega_before_rad_s"])-.001*(row["joint_applied_Nm"]-.1*row["omega_before_rad_s"])
        momentum += residual
        total_force_impulse += .001*abs(row["joint_applied_Nm"])
        max_balance = max(max_balance, abs(momentum))
        if abs(momentum) > 2e-5+1e-5*total_force_impulse:
            raise AssertionError("真实力矩与转子动量不平衡："+str(momentum))
        _close(row["rotor_x_axis_x"], math.cos(row["q_after_rad"]), "actual rotor cos(q)", 2e-6)
        _close(row["rotor_x_axis_y"], math.sin(row["q_after_rad"]), "actual rotor sin(q)", 2e-6)
        delta = (row["q_after_rad"]-previous_q+math.pi) % (2*math.pi)-math.pi
        previous_unwrapped += delta
        _close(row["q_unwrapped_after_rad"], previous_unwrapped, "independent joint unwrap", 1e-7)
        previous_q = row["q_after_rad"]
    tail = [row for row in values if duration-1-1e-9 <= row["phase_elapsed_s"] <= duration+1e-9]
    mean = lambda key: sum(row[key] for row in tail)/len(tail)
    metrics = {"samples": len(rows), "terminal_samples": len(tail), "duration_s": duration,
               "terminal_velocity_mean_rad_s": mean("omega_after_rad_s"),
               "terminal_motor_mean_Nm": mean("motor_command_Nm"),
               "terminal_i_term_mean_Nm": mean("pid_i_term_Nm"),
               "maximum_momentum_residual_Nms": max_balance,
               "momentum_tolerance_Nms": 2e-5+1e-5*total_force_impulse,
               "actual_peak_command_Nm": max(abs(row["motor_command_Nm"]) for row in values),
               "native_angle_raw_span_rad": max(row["q_after_rad"] for row in values)-min(row["q_after_rad"] for row in values),
               "final_unwrapped_angle_rad": values[-1]["q_unwrapped_after_rad"]}
    if name == "torque":
        checks = []
        for when in (.2, .5):
            row = values[round(when/.001)-1]
            expected_w = p["torque_command_Nm"]/.1*(1-math.exp(-.1*when/.03125))
            expected_q = p["torque_command_Nm"]/.1*(when-.03125/.1*(1-math.exp(-.1*when/.03125)))
            if abs(row["omega_after_rad_s"]-expected_w) > .02 or abs(row["q_unwrapped_after_rad"]-expected_q) > .015:
                raise AssertionError("真实力矩响应与惯量/阻尼解析预测不符")
            checks.append({"time_s": when, "actual_velocity_rad_s": row["omega_after_rad_s"],
                           "predicted_velocity_rad_s": expected_w, "actual_angle_rad": row["q_unwrapped_after_rad"],
                           "predicted_angle_rad": expected_q})
        metrics["analytic_response"] = checks
    elif name in ("speed-p", "speed-pi", "saturation"):
        target = p["velocity_target_rad_s"]
        expected = ((p["speed_p_kp"]*target-p["load_torque_Nm"])/(p["speed_p_kp"]+.1) if name == "speed-p"
                    else ((p["saturation_limit_Nm"]-p["load_torque_Nm"])/.1 if name == "saturation" else target))
        if abs(mean("omega_after_rad_s")-expected) > .02:
            raise AssertionError("速度末窗不符合该控制方式的实际稳态："+name)
        metrics["predicted_loaded_steady_velocity_rad_s"] = expected
        metrics["terminal_target_error_RMS_rad_s"] = math.sqrt(sum((row["omega_after_rad_s"]-target)**2 for row in tail)/len(tail))
        if name == "speed-pi" and metrics["terminal_target_error_RMS_rad_s"] > .02:
            raise AssertionError("速度PI末窗目标误差RMS超过0.02rad/s")
        if name == "saturation":
            if abs(mean("motor_command_Nm")-cap) > 1e-7 or abs(mean("pid_i_term_Nm")+p["integral_term_limit_Nm"]) > 1e-6:
                raise AssertionError("输出饱和与积分项限幅未在真实原生PID中发生")
    else:
        target = p["position_target_rad"]
        sign = 1 if target > 0 else -1
        peak = max(sign*row["q_after_rad"] for row in values)
        metrics["signed_normalized_peak_rad"] = peak
        metrics["terminal_position_error_RMS_rad"] = math.sqrt(sum((row["q_after_rad"]-target)**2 for row in tail)/len(tail))
        metrics["terminal_velocity_RMS_rad_s"] = math.sqrt(sum(row["omega_after_rad_s"]**2 for row in tail)/len(tail))
        if name == "position-p" and peak <= 1.25*abs(target):
            raise AssertionError("位置P的欠阻尼超调没有实际出现")
        if name == "position-pd" and (peak >= 1.05*abs(target) or metrics["terminal_position_error_RMS_rad"] > .01 or metrics["terminal_velocity_RMS_rad_s"] > .03):
            raise AssertionError("位置PD的阻尼/末窗收敛未满足当前实验条件")
    return metrics


def _phase(name, timeout=35):
    initial = inspect()
    if name not in dict(PHASES):
        raise ValueError("Unknown bounded phase: "+name)
    episode = "motor-"+uuid.uuid4().hex
    request = directory()/"phase.request"
    temporary = request.with_suffix(".tmp")
    temporary.write_text(episode+" "+name+"\n", encoding="ascii")
    os.replace(str(temporary), str(request))
    deadline = time.monotonic()+timeout
    while True:
        current = status()
        if current["error"]:
            raise AssertionError("真实原生控制器失败："+current["error"])
        if current["episode"] == episode and current["phase_complete"]:
            break
        if time.monotonic() > deadline:
            raise RuntimeError("真实电机阶段超时，保留CSV和状态："+name)
        time.sleep(.08)
    rows = rows_for(episode)
    if digest(WORKSPACE/"parameters.json") != initial["config_sha256"]:
        raise AssertionError("阶段期间参数发生变化，请保存并重载")
    inspect()  # Recheck all actual disk/runtime fingerprints, including the final phase.
    result = {"phase": name, "episode": episode, "samples": len(rows), "reset_evidence": current["reset_evidence"],
              "metrics": analyse(rows, name, initial["parameters"], current["reset_evidence"])}
    print("真实电机阶段完成："+name, flush=True)
    return result


def _preserve(function, output):
    initial = inspect()  # Every rejection above this point must leave the world untouched.
    original = read_pause()
    try:
        set_pause(False)
        result = function(initial)
    finally:
        cleanup = set_pause(original)
        if not cleanup["confirmed"]:
            raise RuntimeError("Cannot confirm original simulator pause after motor measurement")
    result.update(original_pause=original, pause_cleanup=cleanup)
    (directory()/output).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def experiment():
    def perform(initial):
        measured = [_phase(name) for name, _ in PHASES]
        return {"passed": True, "parameters": initial["parameters"], "config_sha256": initial["config_sha256"],
                "world_sha256": initial["world_sha256"], "actual_model": initial["model"], "phases": measured,
                "telemetry_csv": str(directory()/"control.csv"), "status_json": str(directory()/"status.json"),
                "time_convention": "phase_elapsed_s=completed_physics_steps*0.001; world sim_time_s retained separately"}
    return _preserve(perform, "verification.json")


def run_phase(name):
    return _preserve(lambda initial: {"passed": True, "parameters": initial["parameters"], "phases": [_phase(name)],
                                     "telemetry_csv": str(directory()/"control.csv")}, "phase-validation.json")
