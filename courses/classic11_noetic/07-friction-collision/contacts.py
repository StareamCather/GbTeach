"""Restricted, current-workspace ODE contact predictions for this lesson rig."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

NAMES = ("friction_ramp", "slider_low", "slider_high", "pad_dead", "pad_bouncy", "ball_dead", "ball_bouncy")


def values(text, count, label):
    try:
        result = [float(value) for value in (text or "").split()]
    except ValueError as error:
        raise ValueError(label + "必须为数值") from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError("%s需要%d个有限数值" % (label, count))
    return result


def scalar(node, path, label):
    return values(node.findtext(path), 1, label + "/" + path)[0]


def pose(node, label):
    entry = node.find("pose")
    if entry is not None and entry.get("relative_to"):
        raise ValueError(label + "本rig使用默认父坐标系")
    return values(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")


def equal(actual, expected, label, tolerance=1e-6):
    if len(actual) != len(expected) or any(abs(a-b) > max(tolerance, abs(b)*1e-6) for a, b in zip(actual, expected)):
        raise ValueError("%s不符：%s，应为%s" % (label, actual, expected))


def local_position(raw, ramp):
    theta = -ramp[4]
    dx, dy, dz = [a-b for a, b in zip(raw[:3], ramp[:3])]
    return [math.cos(theta)*dx+math.sin(theta)*dz, dy,
            -math.sin(theta)*dx+math.cos(theta)*dz]


def surface(collision, label):
    node = collision.find("surface")
    if node is None:
        raise ValueError(label + "需要显式surface参数")
    result = {key: scalar(node, path, label) for key, path in (
        ("mu", "friction/ode/mu"), ("mu2", "friction/ode/mu2"),
        ("slip1", "friction/ode/slip1"), ("slip2", "friction/ode/slip2"),
        ("restitution", "bounce/restitution_coefficient"), ("threshold", "bounce/threshold"),
        ("kp", "contact/ode/kp"), ("kd", "contact/ode/kd"),
        ("max_vel", "contact/ode/max_vel"), ("min_depth", "contact/ode/min_depth"))}
    result["fdir1"] = values(node.findtext("friction/ode/fdir1"), 3, label + " fdir1")
    if not all(0 <= result[key] <= 2 for key in ("mu", "mu2")):
        raise ValueError(label + "摩擦系数mu/mu2在本课采用0至2的有限非负值（两者是方向系数）")
    if not 0 <= result["restitution"] <= 1:
        raise ValueError(label + "恢复系数restitution在本课必须位于[0,1]")
    if not 0 <= result["threshold"] <= 100:
        raise ValueError(label + "bounce/threshold在本课采用0至100m/s")
    if result["slip1"] != 0 or result["slip2"] != 0:
        raise ValueError(label + "本rig保持slip1/slip2=0以单独比较Coulomb方向摩擦")
    max_vel_limit = 20 if label.startswith("ball_") else .2
    if not (1e7 <= result["kp"] <= 1e12 and 0 <= result["kd"] <= 10 and
            .001 <= result["max_vel"] <= max_vel_limit and 0 <= result["min_depth"] <= .001):
        raise ValueError(label + "接触纠穿参数超出本rig已支持范围：kp[1e7,1e12]、kd[0,10]、max_vel[.001,%.3g]、min_depth[0,.001]" % max_vel_limit)
    return result


def configuration(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    world = ET.parse(str(workspace / "world.sdf")).getroot().find("world")
    if world is None:
        raise ValueError("world.sdf缺少world")
    p = json.loads((workspace / "parameters.json").read_text(encoding="utf-8"))
    slide_steps = p.get("slide_steps")
    half_window = float(p.get("peak_half_window_s", 0))
    if isinstance(slide_steps, bool) or not isinstance(slide_steps, int) or not 50 <= slide_steps <= 1800:
        raise ValueError("slide_steps应为50至1800的整数，实际还需满足可分辨位移和跑道余量")
    if not math.isfinite(half_window) or not .04 <= half_window <= .15:
        raise ValueError("peak_half_window_s需在[.04,.15]秒，实际峰采样会按飞行时长缩窗")
    dt = scalar(world, "physics/max_step_size", "physics")
    gravity = values(world.findtext("gravity"), 3, "gravity")
    global_max_vel = scalar(world, "physics/ode/constraints/contact_max_correcting_vel", "physics")
    if not .0005 <= dt <= .002 or abs(gravity[0])+abs(gravity[1]) > 1e-8 or not 5 <= -gravity[2] <= 12:
        raise ValueError("本rig支持dt[.0005,.002]s、竖直向下g[5,12]m/s²")
    models = {}
    for name in NAMES:
        entries = world.findall("model[@name='%s']" % name)
        if len(entries) != 1:
            raise ValueError("需要唯一模型" + name)
        node = entries[0]
        dynamic = name.startswith("slider_") or name.startswith("ball_")
        is_static = node.findtext("static", "false").strip().lower() in ("true", "1")
        if is_static == dynamic:
            raise ValueError(name + "static设置与实验角色不一致")
        links = node.findall("link")
        if len(links) != 1:
            raise ValueError(name + "本rig需要单link")
        link = links[0]
        equal(pose(link, name + " link"), [0]*6, name + "link原点")
        collisions = link.findall("collision")
        visuals = link.findall("visual")
        if len(collisions) != 1 or len(visuals) != 1:
            raise ValueError(name + "本rig采用单collision/visual")
        shape = "sphere" if name.startswith("ball_") else "box"
        dims = []
        for kind, entry in (("collision", collisions[0]), ("visual", visuals[0])):
            equal(pose(entry, name + " " + kind), [0]*6, name + kind + "几何原点")
            text = entry.findtext("geometry/" + shape + ("/radius" if shape == "sphere" else "/size"))
            dim = values(text, 1 if shape == "sphere" else 3, name + " " + kind)
            if min(dim) <= 0:
                raise ValueError(name + "几何尺寸必须为正")
            dims.append(dim)
        equal(dims[0], dims[1], name + "collision/visual尺寸")
        data = {"name": name, "pose": pose(node, name), "link": link.get("name"),
                "collision": collisions[0].get("name"), "dynamic": dynamic, "shape": shape,
                "dimensions": dims[0], "surface": surface(collisions[0], name)}
        if dynamic:
            inertial = link.find("inertial")
            if inertial is None or link.findtext("gravity", "true").strip().lower() not in ("true", "1"):
                raise ValueError(name + "需要惯性和开启重力")
            equal(pose(inertial, name + " inertial"), [0]*6, name + "均匀刚体COM")
            mass = scalar(inertial, "mass", name)
            if not .1 <= mass <= 10:
                raise ValueError(name + "本rig质量范围.1至10kg")
            if shape == "sphere":
                expected = [.4*mass*dims[0][0]**2]*3
            else:
                x, y, z = dims[0]
                expected = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
            tensor = [scalar(inertial, "inertia/" + key, name) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
            equal(tensor, expected+[0]*3, name + "均匀几何中心惯量", 1e-8)
            data.update(mass_kg=mass, inertia=tensor)
        models[name] = data
    ramp = models["friction_ramp"]
    theta = -ramp["pose"][4]
    if abs(ramp["pose"][3])+abs(ramp["pose"][5]) > 1e-8 or not math.radians(12) <= theta <= math.radians(25):
        raise ValueError("斜面只绕Y轴负转，坡角需12至25度")
    axis = None
    for name in NAMES[:3]:
        fd = models[name]["surface"]["fdir1"]
        current = "X" if abs(abs(fd[0])-1) < 1e-8 and abs(fd[1])+abs(fd[2]) < 1e-8 else (
            "Y" if abs(abs(fd[1])-1) < 1e-8 and abs(fd[0])+abs(fd[2]) < 1e-8 else None)
        if current is None or (axis is not None and current != axis):
            raise ValueError("坡面与两滑块fdir1须同为切向±X或同为±Y，暂不支持冲突/任意斜轴")
        axis = current
    coefficient = "mu" if axis == "X" else "mu2"
    slide_time = slide_steps*dt
    sliding = {}
    for name in ("slider_low", "slider_high"):
        data = models[name]
        equal(data["pose"][3:], ramp["pose"][3:], name + "与斜坡初始方向")
        u, v, normal = local_position(data["pose"], ramp["pose"])
        expected_normal = ramp["dimensions"][2]/2+data["dimensions"][2]/2
        if not 0 <= normal-expected_normal <= .003:
            raise ValueError(name + "需放在坡面上方0至.003m，不能穿透或高空落下")
        half_length = data["dimensions"][0]/2
        ramp_half_length = ramp["dimensions"][0]/2
        if u-half_length < -ramp_half_length+.15 or u+half_length > ramp_half_length-.15:
            raise ValueError(name + "初始坡向支撑超出坡面：完整滑块需距上下坡边界至少.15m")
        if abs(v)+data["dimensions"][1]/2 > ramp["dimensions"][1]/2-.05:
            raise ValueError(name + "横向初始位置超出坡面")
        mu = min(data["surface"][coefficient], ramp["surface"][coefficient])
        margin = math.tan(theta)-mu
        if abs(margin) < .04:
            raise ValueError(name + "摩擦系数太接近静止/滑移边界，本仪器需|tan坡角-mu_eff|≥.04")
        acceleration = max(0, -gravity[2]*(math.sin(theta)-mu*math.cos(theta)))
        distance = acceleration*slide_time*slide_time/2
        if acceleration > 0 and distance < .15:
            raise ValueError(name + "预测滑移不足.15m，增加slide_steps以获得可分辨位移")
        if data["dimensions"][2]/2*math.tan(theta) >= data["dimensions"][0]/2-.04:
            raise ValueError(name + "滑块过高，可能倾覆而非单纯滑移")
        sliding[name] = {"effective_coefficient": mu, "along_slope_field": coefficient,
                         "predicted_state": "sliding" if acceleration > 0 else "held",
                         "predicted_acceleration_m_s2": acceleration, "predicted_displacement_m": distance,
                         "initial_ramp_position": [u, v, normal]}
    rebound = {}
    for suffix in ("dead", "bouncy"):
        ball, pad = models["ball_"+suffix], models["pad_"+suffix]
        equal(ball["pose"][3:], [0]*3, ball["name"] + "初始方向")
        equal(pad["pose"][3:], [0]*3, pad["name"] + "水平接触面")
        equal(ball["pose"][:2], pad["pose"][:2], "球在对应pad中央")
        radius = ball["dimensions"][0]
        top = pad["pose"][2]+pad["dimensions"][2]/2
        height = ball["pose"][2]-top-radius
        if not .6 <= height <= 2.5:
            raise ValueError(ball["name"] + "净落高H需.6至2.5m")
        e = min(ball["surface"]["restitution"], pad["surface"]["restitution"])
        threshold = min(ball["surface"]["threshold"], pad["surface"]["threshold"])
        impact_time = math.sqrt(2*height/-gravity[2])
        speed = -gravity[2]*impact_time
        rebound[ball["name"]] = {"radius_m": radius, "pad_top_z": top, "contact_center_z": top+radius,
                                  "drop_height_m": height, "impact_time_s": impact_time,
                                  "impact_speed_m_s": speed, "effective_restitution": e,
                                  "effective_threshold_m_s": threshold, "ideal_peak_rise_m": e*e*height}
    equal([rebound["ball_dead"][key] for key in ("radius_m", "pad_top_z", "drop_height_m")],
          [rebound["ball_bouncy"][key] for key in ("radius_m", "pad_top_z", "drop_height_m")], "两球落体几何")
    dead, bouncy = rebound["ball_dead"], rebound["ball_bouncy"]
    maximum_rebound_velocity = max(item["effective_restitution"]*item["impact_speed_m_s"] for item in rebound.values())
    required_max_vel = maximum_rebound_velocity+.5
    cap_sources = {"global contact_max_correcting_vel": global_max_vel}
    cap_sources.update({name + " contact/max_vel": models[name]["surface"]["max_vel"] for name in ("ball_dead", "ball_bouncy")})
    for label, limit in cap_sources.items():
        if limit < required_max_vel:
            raise ValueError("QuickStep反弹目标限幅：%s=%.6gm/s，需至少max(e*v_impact)+.5=%.6gm/s；请同步提高全局与两球max_vel，不能把截断后的微弹当恢复系数结果" %
                             (label, limit, required_max_vel))
    ball_contact_fields = ("kp", "kd", "max_vel", "min_depth")
    equal([models["ball_dead"]["surface"][key] for key in ball_contact_fields],
          [models["ball_bouncy"]["surface"][key] for key in ball_contact_fields],
          "两球接触数值参数需相同，避免以额外接触控制差异制造反弹对比")
    if dead["effective_restitution"] != 0:
        raise ValueError("ball_dead配对恢复系数保持0作为无反弹对照")
    if bouncy["effective_threshold_m_s"] >= bouncy["impact_speed_m_s"]-.3:
        raise ValueError("有效threshold关闭或接近反弹窗口：本峰测量器要求预计撞速高于配对threshold至少.3m/s；高阈值可手动观察，复原后再定量验收")
    if not .4 <= bouncy["effective_restitution"] <= .9 or bouncy["ideal_peak_rise_m"] < .12:
        raise ValueError("反弹峰信号不足：本rig要求配对e在[.4,.9]且e²H≥.12m，不把微小反弹当可测峰")
    ascent_time = bouncy["effective_restitution"]*bouncy["impact_time_s"]
    half_window = min(half_window, .4*ascent_time)
    if half_window < .04 or -gravity[2]*half_window*half_window/2 < .004:
        raise ValueError("反弹峰采样窗口不足：半窗需≥.04s且预计邻点高度差≥.004m")
    peak_time = bouncy["impact_time_s"]+ascent_time
    targets = {"slide": slide_steps,
               "pre_impact": max(1, round((bouncy["impact_time_s"]-.03)/dt)),
               "impact": max(1, round(bouncy["impact_time_s"]/dt)),
               "after_impact": max(1, round((bouncy["impact_time_s"]+.04)/dt)),
               "peak_before": round((peak_time-half_window)/dt),
               "peak": round(peak_time/dt), "peak_after": round((peak_time+half_window)/dt)}
    final_time = max(targets.values())*dt
    for name, predicted in sliding.items():
        run = predicted["predicted_acceleration_m_s2"]*final_time*final_time/2
        left = predicted["initial_ramp_position"][0]-run-models[name]["dimensions"][0]/2
        if left < -ramp["dimensions"][0]/2+.15:
            raise ValueError(name + "在全部反弹采样结束前可能滑出坡面；缩短落高/采样或调整坡内起点")
    return {"world": world.get("name", "default"), "workspace": workspace, "parameters": p,
            "dt_s": dt, "gravity": gravity, "models": models, "slope_angle_rad": theta,
            "fdir_axis": axis, "sliding": sliding, "bounce": rebound, "target_steps": targets,
            "peak_half_window_s": half_window, "final_sample_time_s": final_time,
            "contact_velocity_limits": {"maximum_expected_rebound_velocity_m_s": maximum_rebound_velocity,
                                        "headroom_m_s": .5, "required_minimum_max_vel_m_s": required_max_vel,
                                        "declared_limits_m_s": cap_sources}}


def predictions(config):
    return {key: value for key, value in config.items() if key not in ("workspace",)}
