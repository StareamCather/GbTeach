"""Bounded, active-body contact-geometry measurements; no internal LCP claims."""
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import transport as gz
from stability import NAMES, PROBES, configuration, predictions


def physics_info(world):
    metadata_path = Path.cwd() / "build/physics_probe.json"
    if not metadata_path.is_file():
        raise RuntimeError("缺少只读physics探针；请在应用重载，或先运行python3 prepare.py")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    source = Path(__file__).with_name("physics_probe.cc")
    if metadata.get("source_sha256") != hashlib.sha256(source.read_bytes()).hexdigest():
        raise RuntimeError("只读physics探针源码版本不符，请重载重新构建")
    binary = Path(metadata["binary"])
    if not binary.is_file():
        raise RuntimeError("只读physics探针缓存已不存在，请重载重新构建")
    actual = gz.message(gz.read(str(binary), world))
    if gz.first(actual, "type") != "ODE":
        raise AssertionError("physics_info实际后端不是ODE")
    result = {"type": "ODE", "solver_type": gz.first(actual, "solver_type")}
    for key in ("max_step_size", "iters", "cfm", "erp", "contact_max_correcting_vel", "contact_surface_layer",
                "real_time_update_rate", "real_time_factor"):
        value = float(gz.first(actual, key))
        if not math.isfinite(value):
            raise AssertionError("physics_info非有限字段：" + key)
        result[key] = int(value) if key == "iters" else value
    gravity = gz.first(actual, "gravity")
    result["gravity"] = [float(gz.first(gravity, axis)) for axis in ("x", "y", "z")]
    return result


def check_physics(config):
    actual = physics_info(config["world"])
    for key, expected in config["engine"].items():
        if key == "solver_type":
            if actual[key] != expected:
                raise AssertionError("实际solver与当前保存文件不符，请重载")
        elif key == "gravity":
            for measured, value in zip(actual[key], expected):
                gz.near(measured, value, 1e-8, "实际gravity")
        else:
            gz.near(actual[key], expected, max(1e-12, abs(expected)*1e-7), "实际physics " + key)
    print("PASS：原生physics_info确认实际solver=%s、dt=%.6gs、iters=%d、global CFM/ERP及接触/运行配置" %
          (actual["solver_type"], actual["max_step_size"], actual["iters"]), flush=True)
    return actual


def model_message(world, name):
    return gz.message(gz.read("gz", "model", "-w", world, "-m", name, "-i"))


def raw_pose(message):
    node = gz.first(message, "pose")
    position, orientation = gz.first(node, "position"), gz.first(node, "orientation")
    xyz = [float(gz.first(position, key)) for key in ("x", "y", "z")]
    x, y, z, w = [float(gz.first(orientation, key)) for key in ("x", "y", "z", "w")]
    if not all(math.isfinite(value) for value in xyz+[x, y, z, w]) or abs(x*x+y*y+z*z+w*w-1) > .002:
        raise AssertionError("Model pose非有限或四元数未归一")
    return xyz+[math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                math.asin(max(-1, min(1, 2*(w*y-z*x)))),
                math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def live_models(config):
    result = {}
    for name in NAMES:
        expected = config["models"][name]
        actual = model_message(config["world"], name)
        static = gz.first(actual, "is_static") in ("true", "1")
        if static == expected["dynamic"]:
            raise AssertionError(name + "实际static不符")
        gz.check_message_pose(name + "重置初始pose", gz.first(actual, "pose"), expected["pose"])
        links = actual.get("link", [])
        if len(links) != 1 or str(gz.first(links[0], "name")).split("::")[-1] != expected["link"]:
            raise AssertionError(name + "实际link集合不符")
        link = links[0]
        collisions = link.get("collision", [])
        if len(collisions) != 1:
            raise AssertionError(name + "实际collision数量不符")
        collision = collisions[0]
        shape = gz.first(gz.first(collision, "geometry"), expected["shape"])
        dimensions = [float(gz.first(shape, "radius"))] if expected["dynamic"] else [
            float(gz.first(gz.first(shape, "size"), axis)) for axis in ("x", "y", "z")]
        for measured, value in zip(dimensions, expected["dimensions"]):
            gz.near(measured, value, 1e-7, name + "实际几何")
        surface = gz.first(collision, "surface")
        saved = expected["surface"]
        runtime_expected = dict(saved)
        if static:
            runtime_expected["max_vel"] = 0.0  # Verified Classic ODECollision::Load rule.
        observed = {}
        for key, field in (("restitution", "restitution_coefficient"), ("threshold", "bounce_threshold"),
                           ("kp", "kp"), ("kd", "kd"), ("max_vel", "max_vel"), ("min_depth", "min_depth")):
            observed[key] = float(gz.first(surface, field))
            gz.near(observed[key], runtime_expected[key], max(1e-8, abs(runtime_expected[key])*1e-7), name + "实际surface " + key)
        if expected["dynamic"]:
            if gz.first(link, "gravity") not in ("true", "1"):
                raise AssertionError(name + "实际重力未开启")
            inertial = gz.first(link, "inertial")
            gz.near(float(gz.first(inertial, "mass")), expected["mass_kg"], 1e-7, name + "实际质量")
            gz.check_message_pose(name + "实际COM", gz.first(inertial, "pose"), [0]*6)
            for key, value in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), expected["inertia"]):
                gz.near(float(gz.first(inertial, key)), value, 1e-8, name + "实际" + key)
        result[name] = {"actual_surface": observed, "saved_surface": saved,
                        "expected_runtime_surface": runtime_expected, "actual_dimensions": dimensions,
                        "initial_pose": raw_pose(actual), "dynamic": expected["dynamic"]}
    return result


def snapshot(config, baseline, stats):
    poses, enabled = {}, {}
    for name in PROBES:
        message = model_message(config["world"], name)
        poses[name] = raw_pose(message)
        link = gz.first(message, "link")
        enabled[name] = gz.first(link, "enabled") in ("true", "1")
        if not enabled[name]:
            raise AssertionError(name + "尾部已sleep/disabled，不能把静止标成活跃约束收敛")
    return {"stats": stats, "relative_time_s": stats["sim_time_s"]-baseline["sim_time_s"],
            "poses": poses, "enabled": enabled}


def step(config, count):
    before = gz.stats(config["world"])
    if not before["paused"]:
        raise RuntimeError("定步测量必须暂停")
    gz.run("gz", "world", "-w", config["world"], "-m", str(count))
    target = before["iterations"]+count
    while True:
        after = gz.stats(config["world"])
        if after["iterations"] >= target:
            break
        if gz.DEADLINE-time.monotonic() < 1:
            raise RuntimeError("未收到定步完成ack")
    if after["iterations"] != target or not after["paused"]:
        raise AssertionError("实际步数/暂停状态不符合请求")
    gz.near(after["sim_time_s"]-before["sim_time_s"], count*config["dt_s"], 1e-7, "实际步长与仿真时间")
    return {"requested_steps": count, "before": before, "after": after}


def metrics(config, samples):
    result = {}
    for name in PROBES:
        pair = config["pairs"][name]
        zs = [item["poses"][name][2] for item in samples]
        overlaps = [pair["contact_center_z"]-value for value in zs]
        mean = statistics.mean(overlaps)
        deviation = mean-pair["predicted_overlap_m"]
        tolerance = max(.001, pair["predicted_overlap_m"]*.25)
        gz.near(mean, pair["predicted_overlap_m"], tolerance, name + "平均接触几何重叠")
        range_z = max(zs)-min(zs)
        if range_z > .002:
            raise AssertionError(name + "尾部采样波动超过.002m，当前采样未稳定")
        if min(overlaps) < -.001 or max(overlaps) > pair["radius_m"]*.25:
            raise AssertionError(name + "已失去支撑或重叠超出小变形测量范围")
        initial = config["models"][name]["pose"]
        if any(math.hypot(item["poses"][name][0]-initial[0], item["poses"][name][1]-initial[1]) > .01 for item in samples):
            raise AssertionError(name + "明显横漂，超出单点竖直承载模型")
        result[name] = {"overlap_samples_m": overlaps, "mean_overlap_m": mean,
                        "range_z_m": range_z, "std_z_m": statistics.pstdev(zs),
                        "predicted_overlap_m": pair["predicted_overlap_m"], "mean_error_m": deviation,
                        "absolute_mean_error_m": abs(deviation), "mean_tolerance_m": tolerance}
    if result["probe_soft"]["mean_overlap_m"]-result["probe_firm"]["mean_overlap_m"] < .005:
        raise AssertionError("真实soft/firm几何压缩未形成可辨对比")
    return result


def experiment(workspace=None):
    config = configuration(workspace)  # Reject invalid input before any control.
    gz.DEADLINE = time.monotonic()+65
    started = time.monotonic()
    original = gz.stats(config["world"])
    result = {"predictions": predictions(config), "original_pause": original["paused"], "repeats": [],
              "state_restored": "pause_only_last_experiment_pose_velocity_time_retained",
              "wall_time_scope": "instrument_end_to_end_including_CLI_transport_ACK_and_sampling_not_pure_physics_CPU_or_free_running_RTF"}
    error = None
    try:
        gz.pause(config["world"], True)
        if not gz.stats(config["world"])["paused"]:
            raise RuntimeError("未确认暂停")
        result["live_physics"] = check_physics(config)
        for index in range(config["parameters"]["repeats"]):
            tick = time.monotonic()
            item = {"index": index+1, "samples": [], "step_acknowledgements": []}
            result["repeats"].append(item)
            item["historical_input_drain"] = step(config, 1)
            gz.run("gz", "world", "-w", config["world"], "-r")
            baseline = gz.stats(config["world"])
            if not baseline["paused"]:
                raise RuntimeError("reset ALL未保持暂停")
            item["reset_baseline"] = baseline
            if index == 0:
                result["live_models"] = live_models(config)
            acknowledgement = step(config, config["warmup_steps"])
            item["step_acknowledgements"].append(acknowledgement)
            item["samples"].append(snapshot(config, baseline, acknowledgement["after"]))
            for _ in range(1, config["parameters"]["tail_samples"]):
                acknowledgement = step(config, config["tail_interval_steps"])
                item["step_acknowledgements"].append(acknowledgement)
                item["samples"].append(snapshot(config, baseline, acknowledgement["after"]))
            item["metrics"] = metrics(config, item["samples"])
            item["instrument_wall_s"] = time.monotonic()-tick
            for name, observed in item["metrics"].items():
                print("PASS：repeat%d %s活跃承载，平均几何重叠%.9gm（理想%.9g），尾部5点std%.4g/range%.4gm" %
                      (index+1, name, observed["mean_overlap_m"], observed["predicted_overlap_m"], observed["std_z_m"], observed["range_z_m"]), flush=True)
        result["repeatability"] = {name: {"mean_overlap_difference_m": abs(result["repeats"][1]["metrics"][name]["mean_overlap_m"]-
                                      result["repeats"][0]["metrics"][name]["mean_overlap_m"]),
                                         "note": "same_seed_reset_repeat_difference_not_cross_hardware_bitwise_guarantee"} for name in PROBES}
        result.update(passed=True, instrument_wall_s=time.monotonic()-started)
        print("MEASUREMENTS_JSON " + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        return result
    except Exception as caught:
        error = caught
        result.update(passed=False, error="%s: %s" % (type(caught).__name__, caught), instrument_wall_s=time.monotonic()-started)
        print("FAILED_MEASUREMENTS_JSON " + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        raise
    finally:
        gz.DEADLINE = time.monotonic()+4
        try:
            gz.pause(config["world"], original["paused"])
            if gz.stats(config["world"])["paused"] != original["paused"]:
                raise RuntimeError("原暂停状态未恢复")
            print("已恢复原pause=%s；保留最后一次承载结果/速度/时钟，未修改world和参数。" % original["paused"], flush=True)
        except Exception as cleanup:
            if error is not None:
                print("附加pause恢复失败：%s；原始失败保留" % cleanup, flush=True)
            else:
                raise
