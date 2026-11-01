"""Bounded Classic 11 quantitative instrument; resets models/time, keeps results.

All predictions come from the current workspace. A single native Link Wrench
message acts in link coordinates for one physics step, not a wall-clock hold.
"""
import json
import math
import re
import subprocess
import time
import sys
from pathlib import Path

from mechanics import NAMES, configuration, predictions

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from _control_support.models import pose as native_model_pose

DEADLINE = None


def run(*arguments):
    remaining = DEADLINE-time.monotonic()
    if remaining <= .1:
        raise RuntimeError("惯性实验达到有界时间上限")
    result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=min(2.5, remaining))
    if result.returncode:
        raise RuntimeError("命令失败：%s\n%s" % (" ".join(arguments), (result.stderr or result.stdout)[-1000:]))
    return result.stdout


def read(*arguments):
    for attempt in range(2):
        try:
            return run(*arguments)
        except subprocess.TimeoutExpired as error:
            if attempt or DEADLINE-time.monotonic() < .3:
                raise RuntimeError("只读查询连续超时，有限重试已用完：" + " ".join(arguments)) from error
            print("RETRY：仅重试一次只读查询：" + " ".join(arguments), flush=True)


def message(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_]*|[{}:]', text)
    cursor = [0]

    def parse(nested=False):
        result = {}
        while cursor[0] < len(tokens):
            key = tokens[cursor[0]]
            cursor[0] += 1
            if key == "}":
                if not nested:
                    raise RuntimeError("运行消息括号不匹配")
                return result
            if cursor[0] >= len(tokens):
                raise RuntimeError("运行消息缺少字段值")
            if tokens[cursor[0]] == ":":
                cursor[0] += 1
            if tokens[cursor[0]] == "{":
                cursor[0] += 1
                value = parse(True)
            else:
                value = tokens[cursor[0]]
                cursor[0] += 1
                if value.startswith('"'):
                    value = json.loads(value)
            result.setdefault(key, []).append(value)
        if nested:
            raise RuntimeError("运行消息缺少右括号")
        return result

    return parse()


def first(node, key):
    if key not in node:
        raise RuntimeError("运行消息缺少字段：" + key)
    return node[key][0]


def stats(world):
    output = read("gz", "topic", "-e", "/gazebo/%s/world_stats" % world, "-d", "1")
    # A one-second echo may contain multiple complete messages. Each field's
    # last occurrence belongs to its last sample; no physics runs while paused.
    iterations = re.findall(r"\biterations:\s*(\d+)", output)
    paused = re.findall(r"\bpaused:\s*(true|false)", output)
    clocks = re.findall(r"\bsim_time\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}", output)
    if not iterations or not paused or not clocks:
        raise RuntimeError("world_stats缺少iterations/sim_time/paused")
    sec, nsec = clocks[-1]
    return {"iterations": int(iterations[-1]), "sim_time_s": int(sec)+int(nsec)*1e-9,
            "paused": paused[-1] == "true"}


def pause(world, value):
    run("gz", "world", "-w", world, "-p", "1" if value else "0")


def live_pose(world, name):
    raw=native_model_pose(world,name,DEADLINE)
    if raw is not None:return raw
    output = read("gz", "model", "-w", world, "-m", name, "-p")
    for line in reversed(output.splitlines()):
        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            continue
        if len(values) == 6 and all(math.isfinite(value) for value in values):
            return values
    raise RuntimeError("没有实际模型位姿：" + name)


def quaternion(raw):
    r, p, y = [value/2 for value in raw[3:]]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def check_message_pose(label, actual, expected):
    xyz = [float(first(first(actual, "position"), key)) for key in ("x", "y", "z")]
    q = [float(first(first(actual, "orientation"), key)) for key in ("x", "y", "z", "w")]
    if not all(math.isfinite(value) for value in xyz+q):
        raise AssertionError(label + "实时pose不是有限值")
    dot = abs(sum(a*b for a, b in zip(q, quaternion(expected))))
    if (math.sqrt(sum((a-b)**2 for a, b in zip(xyz, expected[:3]))) > .004 or
            2*math.acos(max(-1, min(1, dot))) > .01):
        raise AssertionError(label + "实际pose与当前文件不一致；保存后请重载")


def near(actual, expected, tolerance, label):
    if not math.isfinite(actual) or abs(actual-expected) > tolerance:
        raise AssertionError("%s不符：实测%.9g，预测%.9g，容差±%.4g" % (label, actual, expected, tolerance))


def live_configuration(config):
    observed = {}
    world = config["world"]
    for name in NAMES:
        saved = config["models"][name]
        actual = message(read("gz", "model", "-w", world, "-m", name, "-i"))
        static = [actual[key][0] for key in ("is_static", "static") if key in actual]
        if not static or any(value not in ("false", "0") for value in static):
            raise AssertionError(name + "真实Model必须dynamic/static=false")
        check_message_pose(name + "重置位姿", first(actual, "pose"), saved["pose"])
        links = actual.get("link", [])
        if len(links) != 1 or str(first(links[0], "name")).split("::")[-1] != saved["link"]:
            raise AssertionError(name + "实际link与当前文件不一致")
        link = links[0]
        inertial = first(link, "inertial")
        measured_mass = float(first(inertial, "mass"))
        near(measured_mass, saved["mass"], max(1e-7, saved["mass"]*1e-5), name + "实际质量")
        check_message_pose(name + "实际inertial.pose(COM坐标系)", first(inertial, "pose"), saved["center"])
        measured_tensor = {}
        for key, expected in saved["tensor"].items():
            measured_tensor[key] = float(first(inertial, key))
            near(measured_tensor[key], expected, max(1e-8, abs(expected)*1e-5), name + "实际" + key)
        measured_gravity = first(link, "gravity")
        expected_gravity = "false" if name.startswith("flywheel_") else "true"
        if measured_gravity != expected_gravity:
            raise AssertionError(name + "真实link gravity与实验隔离条件不一致")
        collisions = link.get("collision", [])
        if len(collisions) != 1:
            raise AssertionError(name + "实际碰撞包络数量不一致")
        geometry = first(collisions[0], "geometry")
        if "size" in saved:
            size = first(first(geometry, "box"), "size")
            for axis, expected in zip(("x", "y", "z"), saved["size"]):
                near(float(first(size, axis)), expected, 1e-6, name + "实际box " + axis)
        else:
            cylinder = first(geometry, "cylinder")
            for key, expected in zip(("radius", "length"), saved["cylinder"]):
                near(float(first(cylinder, key)), expected, 1e-6, name + "实际cylinder " + key)
        observed[name] = {"mass_kg": measured_mass, "inertial_pose": saved["center"],
                          "tensor_kg_m2": measured_tensor, "gravity": measured_gravity,
                          "initial_pose": live_pose(world, name)}
        print("PASS：%s实际质量%.6gkg、COM/inertial坐标、6项惯性、重力及碰撞包络与当前文件一致" %
              (name, measured_mass), flush=True)
    return observed


def step(config, count):
    before = stats(config["world"])
    if not before["paused"]:
        raise RuntimeError("定步仪器要求暂停，拒绝在运行时追加步数")
    run("gz", "world", "-w", config["world"], "-m", str(count))
    target = before["iterations"]+count
    while True:
        after = stats(config["world"])
        if after["iterations"] >= target:
            break
        if DEADLINE-time.monotonic() < 1:
            raise RuntimeError("指定多步尚未实际执行完成")
    if not after["paused"] or after["iterations"] != target:
        raise AssertionError("固定步数执行不符：%s → %s，指定%d步" % (before, after, count))
    near(after["sim_time_s"]-before["sim_time_s"], count*config["dt"], 1e-7, "实际固定步数仿真时间")
    return {"requested_steps": count, "before": before, "after": after}


def z_axis(raw):
    r, p, y = raw[3:]
    return [math.cos(y)*math.sin(p)*math.cos(r)+math.sin(y)*math.sin(r),
            math.sin(y)*math.sin(p)*math.cos(r)-math.cos(y)*math.sin(r),
            math.cos(p)*math.cos(r)]


def delta_angle(end, start):
    return math.atan2(math.sin(end-start), math.cos(end-start))


def experiment(workspace=None):
    global DEADLINE
    # Invalid inertia must fail before pause, reset, publication or step.
    config = configuration(workspace)
    DEADLINE = time.monotonic()+83
    world, parameters = config["world"], config["parameters"]
    original = stats(world)
    result = {"predictions": predictions(config), "original_pause": original["paused"],
              "reset_semantics": "drain_historical_wrench_queue_one_step_then_reset_ALL_models_velocities_and_time_then_keep_results",
              "state_restored": "pause_only_not_original_pose_velocity_or_simulation_time"}
    error = None
    try:
        pause(world, True)
        if not stats(world)["paused"]:
            raise RuntimeError("未确认暂停，未开始重置仪器")
        # Classic Link::ResetPhysicsStates clears current forces/velocities,
        # but not queued wrenchMsgs. One acknowledged step consumes historical
        # input before reset ALL establishes the actual measurement baseline.
        # This step is preparation, never counted as a measured impulse.
        print("仪器操作：先推进1步排空历史Link wrench输入队列，再重置本课全部模型、速度与仿真时间。", flush=True)
        result["historical_input_drain"] = step(config, 1)
        print("正式采样从reset ALL后的真实stats基线开始；结束保留实验结果，仅恢复原暂停状态。", flush=True)
        run("gz", "world", "-w", world, "-r")
        baseline = stats(world)
        if not baseline["paused"]:
            raise RuntimeError("reset ALL后未保持暂停")
        result["reset_baseline"] = baseline
        result["live_models"] = live_configuration(config)
        fall_before = {name: live_pose(world, name) for name in NAMES[:2]}
        fall_steps = step(config, parameters["freefall_steps"])
        fall_after = {name: live_pose(world, name) for name in NAMES[:2]}
        elapsed = fall_steps["after"]["sim_time_s"]-fall_steps["before"]["sim_time_s"]
        expected_distance = -config["gravity"][2]*elapsed*(elapsed+config["dt"])/2
        distances = {name: fall_before[name][2]-fall_after[name][2] for name in NAMES[:2]}
        for name, distance in distances.items():
            near(distance, expected_distance, max(.003, expected_distance*.015), name + "固定步数落差")
            print("PASS：%s质量%.6gkg，%d步/%.6gs实际落差%.7gm，按当前g与步长预测%.7gm" %
                  (name, config["models"][name]["mass"], parameters["freefall_steps"], elapsed, distance, expected_distance), flush=True)
        near(distances[NAMES[0]], distances[NAMES[1]], .001, "不同质量的自由落体落差相同")
        result["freefall"] = {"steps": fall_steps, "before": fall_before, "after": fall_after,
                              "distance_m": distances, "expected_distance_m": expected_distance}
        # The rotors have seen no gravity, contact, or torque. Their unchanged
        # angles across these steps independently check that reset cleared ω.
        for name in NAMES[4:]:
            raw = live_pose(world, name)
            near(delta_angle(raw[5], config["models"][name]["pose"][5]), 0, 1e-5, name + "重置后无力角速度检验")
        tower_steps = step(config, parameters["tower_steps"])
        tower_result = {}
        normal = [-math.sin(config["slope_angle"]), 0, math.cos(config["slope_angle"])]
        for name in NAMES[2:4]:
            raw = live_pose(world, name)
            axis = z_axis(raw)
            tilt = math.acos(max(-1, min(1, sum(a*b for a, b in zip(axis, normal)))))
            expected_stable = config["models"][name]["predicted_stable"]
            if (expected_stable and tilt > .10) or (not expected_stable and tilt < .5):
                raise AssertionError("%s质心支撑预测与实际倾覆不符：相对坡面法线倾角%.6grad，预测stable=%s" %
                                     (name, tilt, expected_stable))
            tower_result[name] = {"pose": raw, "relative_tilt_rad": tilt, "predicted_stable": expected_stable,
                                  "com_height_m": config["models"][name]["com_height"],
                                  "projection_x_m": config["models"][name]["support_projection_x"],
                                  "support_half_width_m": config["models"][name]["support_half_width"]}
            print("PASS：%s COM距脚底%.6gm，重力投影x=%.6gm/半支撑宽%.6gm，实际相对倾角%.6grad，预测%s" %
                  (name, config["models"][name]["com_height"], config["models"][name]["support_projection_x"],
                   config["models"][name]["support_half_width"], tilt, "稳定" if expected_stable else "倾倒"), flush=True)
        result["towers"] = {"steps": tower_steps, "models": tower_result}
        rotor_before = {name: live_pose(world, name) for name in NAMES[4:]}
        for name in NAMES[4:]:
            link = config["models"][name]["link"].replace("::", "/")
            topic = "/gazebo/%s/%s/%s/wrench" % (world, name, link)
            wrench = "force: {x: 0 y: 0 z: 0} torque: {x: 0 y: 0 z: %.12g}" % parameters["torque_z_Nm"]
            run("gz", "topic", "-p", topic, "-m", wrench)
        impulse_steps = step(config, 1)
        rotor_impulse = {name: live_pose(world, name) for name in NAMES[4:]}
        coast_steps = step(config, parameters["flywheel_coast_steps"])
        rotor_after = {name: live_pose(world, name) for name in NAMES[4:]}
        coast_time = coast_steps["after"]["sim_time_s"]-coast_steps["before"]["sim_time_s"]
        rotors = {}
        for name in NAMES[4:]:
            saved = config["models"][name]
            angle = delta_angle(rotor_after[name][5], rotor_impulse[name][5])
            omega = angle/coast_time
            near(omega, saved["predicted_omega"], max(.002, saved["predicted_omega"]*.04), name + "实际自由转动角速度")
            for index in range(3):
                near(rotor_after[name][index], rotor_before[name][index], .002, name + "隔空无平移")
            if max(abs(value) for value in rotor_after[name][3:5]) > .015:
                raise AssertionError(name + "偏离主轴纯转，不能使用τz=Izz αz标量公式")
            rotors[name] = {"before": rotor_before[name], "after_impulse": rotor_impulse[name],
                            "after_coast": rotor_after[name], "coast_angle_rad": angle,
                            "measured_omega_rad_s": omega, "predicted_omega_rad_s": saved["predicted_omega"],
                            "measured_angular_momentum_Nms": saved["tensor"]["izz"]*omega,
                            "Izz_kg_m2": saved["tensor"]["izz"]}
            print("PASS：%s Izz=%.7gkg·m²，同一%.7gNm单步脉冲后，无力滑转角%.7grad/%.6gs，实测ω=%.7g，预测ω=%.7grad/s" %
                  (name, saved["tensor"]["izz"], parameters["torque_z_Nm"], angle, coast_time, omega, saved["predicted_omega"]), flush=True)
        expected_ratio = config["models"]["flywheel_ring"]["tensor"]["izz"]/config["models"]["flywheel_solid"]["tensor"]["izz"]
        measured_ratio = rotors["flywheel_solid"]["measured_omega_rad_s"]/rotors["flywheel_ring"]["measured_omega_rad_s"]
        near(measured_ratio, expected_ratio, expected_ratio*.05, "同角冲量两飞轮ω比=反比惯量")
        result["flywheels"] = {"torque_z_Nm": parameters["torque_z_Nm"], "impulse_steps": impulse_steps,
                               "angular_impulse_Nms": parameters["torque_z_Nm"]*config["dt"],
                               "coast_steps": coast_steps, "models": rotors,
                               "expected_omega_ratio": expected_ratio, "measured_omega_ratio": measured_ratio}
        result["passed"] = True
        print("MEASUREMENTS_JSON " + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        return result
    except Exception as caught:
        error = caught
        raise
    finally:
        DEADLINE = time.monotonic()+4
        try:
            pause(world, original["paused"])
            if stats(world)["paused"] != original["paused"]:
                raise RuntimeError("未确认恢复原暂停状态")
            print("已恢复原暂停状态%s；实验后的模型位姿、速度和仿真时间保留，没有声称恢复原完整state。" % original["paused"], flush=True)
        except Exception as cleanup:
            if error is not None:
                print("附加暂停恢复失败：%s；原始实验错误保留" % cleanup, flush=True)
            else:
                raise
