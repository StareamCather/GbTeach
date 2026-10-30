"""Observe native ODE sliding and a first rebound peak in bounded time."""
import json
import math
import time

import transport as gz
from contacts import NAMES, configuration, local_position, predictions

DYNAMIC = ("slider_low", "slider_high", "ball_dead", "ball_bouncy")


def step(config, count):
    before = gz.stats(config["world"])
    if not before["paused"] or count < 1:
        raise RuntimeError("本仪器仅在暂停时推进正整数步")
    gz.run("gz", "world", "-w", config["world"], "-m", str(count))
    target = before["iterations"]+count
    while True:
        after = gz.stats(config["world"])
        if after["iterations"] >= target:
            break
        if gz.DEADLINE-time.monotonic() < 1:
            raise RuntimeError("未等到实际多步完成")
    if after["iterations"] != target or not after["paused"]:
        raise AssertionError("实际iterations/暂停状态与定步请求不符")
    gz.near(after["sim_time_s"]-before["sim_time_s"], count*config["dt_s"], 1e-7, "定步仿真时间")
    return {"requested_steps": count, "before": before, "after": after}


def live_surfaces(config):
    result = {}
    for name in NAMES:
        expected = config["models"][name]
        actual = gz.message(gz.read("gz", "model", "-w", config["world"], "-m", name, "-i"))
        is_static = str(gz.first(actual, "is_static")) in ("true", "1")
        if is_static == expected["dynamic"]:
            raise AssertionError(name + "实际static与当前文件不符")
        runtime_surface = dict(expected["surface"])
        conversions = []
        if is_static:
            # Gazebo Classic 11.15.1 ODECollision::Load (lines 65--69)
            # overrides maxVel after loading SDF for static collisions.
            # The Model message reports this runtime value, not the declaration.
            runtime_surface["max_vel"] = 0.0
            conversions.append({"field": "max_vel", "declared": expected["surface"]["max_vel"],
                                "runtime": 0.0, "reason": "verified static collision: ODECollision::Load forces maxVel=0",
                                "source": "https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODECollision.cc#L65-L69"})
        gz.check_message_pose(name + "reset初始位姿", gz.first(actual, "pose"), expected["pose"])
        links = actual.get("link", [])
        if len(links) != 1 or str(gz.first(links[0], "name")).split("::")[-1] != expected["link"]:
            raise AssertionError(name + "真实link集合不符")
        link = links[0]
        collisions = link.get("collision", [])
        if len(collisions) != 1 or str(gz.first(collisions[0], "name")).split("::")[-1] != expected["collision"]:
            raise AssertionError(name + "实际collision集合不符")
        collision = collisions[0]
        geometry = gz.first(collision, "geometry")
        shape = gz.first(geometry, expected["shape"])
        if expected["shape"] == "sphere":
            measured_dims = [float(gz.first(shape, "radius"))]
        else:
            size = gz.first(shape, "size")
            measured_dims = [float(gz.first(size, axis)) for axis in ("x", "y", "z")]
        for measured, saved in zip(measured_dims, expected["dimensions"]):
            gz.near(measured, saved, 1e-6, name + "实际collision几何")
        surface = gz.first(collision, "surface")
        friction = gz.first(surface, "friction")
        observed = {}
        for key in ("mu", "mu2", "slip1", "slip2"):
            observed[key] = float(gz.first(friction, key))
        fd = gz.first(friction, "fdir1")
        observed["fdir1"] = [float(gz.first(fd, axis)) for axis in ("x", "y", "z")]
        for key, message_key in (("restitution", "restitution_coefficient"), ("threshold", "bounce_threshold"),
                                 ("kp", "kp"), ("kd", "kd"), ("max_vel", "max_vel"), ("min_depth", "min_depth")):
            observed[key] = float(gz.first(surface, message_key))
        for key, saved in runtime_surface.items():
            if key == "fdir1":
                for measured, component in zip(observed[key], saved):
                    gz.near(measured, component, 1e-6, name + "实际fdir1")
            else:
                gz.near(observed[key], saved, max(1e-7, abs(saved)*1e-6), name + "实际surface " + key)
        if expected["dynamic"]:
            inertial = gz.first(link, "inertial")
            gz.near(float(gz.first(inertial, "mass")), expected["mass_kg"], 1e-6, name + "实际质量")
            for key, saved in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), expected["inertia"]):
                gz.near(float(gz.first(inertial, key)), saved, 1e-7, name + "实际惯量")
        result[name] = {"actual_surface": observed, "saved_surface": expected["surface"],
                        "expected_runtime_surface": runtime_surface, "engine_conversions": conversions,
                        "actual_dimensions": measured_dims, "dynamic": expected["dynamic"]}
        print("PASS：%s真实Model collision.surface全部参数匹配当前文件%s" %
              (name, "及已证实的static max_vel→0引擎转换" if is_static else ""), flush=True)
    return result


def sample(config, baseline, stats, events):
    return {"stats": stats, "relative_time_s": stats["sim_time_s"]-baseline["sim_time_s"],
            "relative_steps": stats["iterations"]-baseline["iterations"], "events": events,
            "poses": {name: gz.live_pose(config["world"], name) for name in DYNAMIC}}


def sliding_measurements(config, samples, baseline):
    start = samples[0]
    finish = next(item for item in samples if "slide" in item["events"])
    models = {}
    ramp_pose = config["models"]["friction_ramp"]["pose"]
    for name, expected in config["sliding"].items():
        before, after = start["poses"][name], finish["poses"][name]
        b, a = local_position(before, ramp_pose), local_position(after, ramp_pose)
        distance = b[0]-a[0]
        lateral, normal = a[1]-b[1], a[2]-b[2]
        tolerance = max(.045, expected["predicted_displacement_m"]*.18) if expected["predicted_state"] == "sliding" else .025
        gz.near(distance, expected["predicted_displacement_m"], tolerance, name + "真实沿坡位移")
        gz.near(lateral, 0, .025, name + "横向漂移")
        gz.near(normal, 0, .025, name + "保持坡面接触")
        if max(abs(after[index]-before[index]) for index in (3, 4)) > .08:
            raise AssertionError(name + "出现明显翻转，超出单纯滑移测量范围")
        models[name] = dict(expected, measured_displacement_m=distance, tolerance_m=tolerance,
                            lateral_displacement_m=lateral, normal_displacement_m=normal,
                            before=before, after=after)
        print("PASS：%s当前沿坡取%s=%.5g，预测%s/位移%.6gm，实际%.6gm" %
              (name, expected["along_slope_field"], expected["effective_coefficient"], expected["predicted_state"],
               expected["predicted_displacement_m"], distance), flush=True)
    return {"before_stats": baseline, "after_stats": finish["stats"],
            "sample_index": samples.index(finish), "fdir_axis": config["fdir_axis"], "models": models}


def fit_peak(points):
    (t1, z1), (t2, z2), (t3, z3) = points
    if not (t1 < t2 < t3 and z2 > z1+.002 and z2 > z3+.002):
        raise AssertionError("真实峰三点未形成上升/下降括区；不能用理论峰代替实际采样")
    slope1, slope2 = (z2-z1)/(t2-t1), (z3-z2)/(t3-t2)
    a = (slope2-slope1)/(t3-t1)
    if a >= 0:
        raise AssertionError("真实反弹峰拟合不是向下开口")
    t_peak = (t1+t2-slope1/a)/2
    if not t1 < t_peak < t3:
        raise AssertionError("拟合峰落在真实采样括区外")
    return {"time_s": t_peak, "height_z": z1+slope1*(t_peak-t1)+a*(t_peak-t1)*(t_peak-t2),
            "quadratic_a_m_s2": a}


def bounce_measurements(config, samples):
    name = "ball_bouncy"
    expected = config["bounce"][name]
    bracket = [next(item for item in samples if event in item["events"]) for event in ("peak_before", "peak", "peak_after")]
    points = [[item["relative_time_s"], item["poses"][name][2]] for item in bracket]
    if min(z for _, z in points) < expected["contact_center_z"]+.04:
        raise AssertionError("反弹峰括区混入接触阶段，不能用自由飞行抛物线拟合")
    fitted = fit_peak(points)
    gz.near(fitted["quadratic_a_m_s2"], config["gravity"][2]/2, max(.15, -config["gravity"][2]*.08), "真实峰附近自由飞行曲率")
    rise = fitted["height_z"]-expected["contact_center_z"]
    tolerance = max(.045, expected["ideal_peak_rise_m"]*.15)
    gz.near(rise, expected["ideal_peak_rise_m"], tolerance, "ball_bouncy实际首次反弹峰升高")
    impact = next(item for item in samples if "impact" in item["events"])
    gz.near(impact["poses"][name][2], expected["contact_center_z"], .035, "真实首次触地附近中心高度")
    if samples[0]["poses"][name][2]-impact["poses"][name][2] < expected["drop_height_m"]*.8:
        raise AssertionError("没有观察到球从初始高度实际下落")
    dead_expected = config["bounce"]["ball_dead"]
    after_impact = [item for item in samples if item["relative_steps"] >= config["target_steps"]["after_impact"]]
    dead_peak = max(item["poses"]["ball_dead"][2] for item in after_impact)
    dead_rise = max(0, dead_peak-dead_expected["contact_center_z"])
    gz.near(dead_rise, 0, .025, "ball_dead无明显反弹对照")
    for ball_name in ("ball_dead", "ball_bouncy"):
        initial = samples[0]["poses"][ball_name]
        for item in samples:
            if math.hypot(item["poses"][ball_name][0]-initial[0], item["poses"][ball_name][1]-initial[1]) > .02:
                raise AssertionError(ball_name + "明显横漂，超出竖直反弹测量范围")
    observed_peak = max(z for _, z in points)
    peak_error_bound = -config["gravity"][2]*max(points[1][0]-points[0][0], points[2][0]-points[1][0])**2/8
    models = {"ball_bouncy": dict(expected, sample_peak_z=observed_peak, fitted_peak_z=fitted["height_z"],
                                  peak_rise_m=rise, measured_height_ratio=rise/expected["drop_height_m"],
                                  rise_tolerance_m=tolerance, peak_bracket=points, fit=fitted,
                                  sample_peak_discretization_bound_m=peak_error_bound),
              "ball_dead": dict(dead_expected, sample_peak_z=dead_peak, fitted_peak_z=None,
                                 peak_rise_m=dead_rise, measured_height_ratio=dead_rise/dead_expected["drop_height_m"],
                                 peak_bracket=None, rise_tolerance_m=.025)}
    print("PASS：ball_bouncy真实上升/下降括峰，sample最大z=%.7g、拟合峰z=%.7g；升高%.7gm，e²H预测%.7gm；ball_dead升高%.6gm" %
          (observed_peak, fitted["height_z"], rise, expected["ideal_peak_rise_m"], dead_rise), flush=True)
    return {"samples": samples, "models": models}


def experiment(workspace=None):
    config = configuration(workspace)  # Reject every invalid input before control.
    gz.DEADLINE = time.monotonic()+69
    started = time.monotonic()
    original = gz.stats(config["world"])
    error = None
    result = {"predictions": predictions(config), "original_pause": original["paused"],
              "state_restored": "pause_only_results_time_and_velocities_retained"}
    try:
        gz.pause(config["world"], True)
        if not gz.stats(config["world"])["paused"]:
            raise RuntimeError("未确认暂停")
        print("仪器先1步排空历史输入，再reset ALL模型/速度/仿真时间；保留测量结果，仅恢复原pause。", flush=True)
        result["historical_input_drain"] = step(config, 1)
        gz.run("gz", "world", "-w", config["world"], "-r")
        baseline = gz.stats(config["world"])
        if not baseline["paused"]:
            raise RuntimeError("reset ALL后未暂停")
        result["reset_baseline"] = baseline
        result["live_surfaces"] = live_surfaces(config)
        samples = [sample(config, baseline, baseline, ["baseline"])]
        # Preserve the evolving raw record even if transport or later analysis
        # fails. These are real timestamps/poses, never synthetic predictions.
        result["samples"] = samples
        current = 0
        for target in sorted(set(config["target_steps"].values())):
            acknowledgment = step(config, target-current)
            current = target
            events = [event for event, value in config["target_steps"].items() if value == target]
            samples.append(sample(config, baseline, acknowledgment["after"], events))
        result["sliding"] = sliding_measurements(config, samples, baseline)
        result["bounce"] = bounce_measurements(config, samples)
        result.update(passed=True, elapsed_wall_s=time.monotonic()-started)
        print("MEASUREMENTS_JSON " + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        return result
    except Exception as caught:
        error = caught
        result.update(passed=False, error="%s: %s" % (type(caught).__name__, caught),
                      elapsed_wall_s=time.monotonic()-started)
        print("FAILED_MEASUREMENTS_JSON " + json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)
        raise
    finally:
        gz.DEADLINE = time.monotonic()+4
        try:
            gz.pause(config["world"], original["paused"])
            if gz.stats(config["world"])["paused"] != original["paused"]:
                raise RuntimeError("原暂停状态未恢复")
            print("已恢复原pause=%s；位姿、速度与时钟保留为实验结果，资产文件未修改。" % original["paused"], flush=True)
        except Exception as cleanup:
            if error is not None:
                print("附加暂停恢复失败：%s；原实验错误保留" % cleanup, flush=True)
            else:
                raise
