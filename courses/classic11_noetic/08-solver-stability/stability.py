"""Current-rig predictions for single-contact, damped ODE support constraints."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

NAMES = ("support_soft", "probe_soft", "support_firm", "probe_firm")
PROBES = ("probe_soft", "probe_firm")


def numbers(text, count, label):
    try:
        result = [float(value) for value in (text or "").split()]
    except ValueError as error:
        raise ValueError(label + "必须是数值") from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError("%s需要%d个有限数值" % (label, count))
    return result


def scalar(node, path, label):
    return numbers(node.findtext(path), 1, label+"/"+path)[0]


def pose(node, label):
    entry = node.find("pose")
    if entry is not None and entry.get("relative_to"):
        raise ValueError(label + "本rig使用默认父坐标系")
    return numbers(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")


def equal(actual, expected, label, tolerance=1e-7):
    if len(actual) != len(expected) or any(abs(a-b) > max(tolerance, abs(b)*1e-6) for a, b in zip(actual, expected)):
        raise ValueError("%s不符：%s，应为%s" % (label, actual, expected))


def configuration(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    world = ET.parse(str(workspace / "world.sdf")).getroot().find("world")
    if world is None:
        raise ValueError("world.sdf缺少world")
    physics = world.find("physics")
    if physics is None or physics.get("type") != "ode":
        raise ValueError("本课固定使用Gazebo Classic ODE")
    solver = physics.findtext("ode/solver/type", "")
    if solver not in ("quick", "world"):
        raise ValueError("solver/type只能为quick或world")
    dt = scalar(physics, "max_step_size", "physics")
    iterations = scalar(physics, "ode/solver/iters", "physics")
    if not .0005 <= dt <= .002 or not iterations.is_integer() or not 10 <= iterations <= 200:
        raise ValueError("本rig步长dt需[.0005,.002]s，记录的iters需10至200整数（只对quick有效）")
    gravity = numbers(world.findtext("gravity"), 3, "gravity")
    if abs(gravity[0])+abs(gravity[1]) > 1e-8 or not 5 <= -gravity[2] <= 12:
        raise ValueError("本rig使用竖直向下g[5,12]m/s²")
    engine = {"solver_type": solver, "max_step_size": dt, "iters": int(iterations),
              "cfm": scalar(physics, "ode/constraints/cfm", "physics"),
              "erp": scalar(physics, "ode/constraints/erp", "physics"),
              "contact_max_correcting_vel": scalar(physics, "ode/constraints/contact_max_correcting_vel", "physics"),
              "contact_surface_layer": scalar(physics, "ode/constraints/contact_surface_layer", "physics"),
              "real_time_update_rate": scalar(physics, "real_time_update_rate", "physics"), "gravity": gravity}
    if (not 0 <= engine["cfm"] <= .001 or not 0 < engine["erp"] <= 1 or
            engine["contact_max_correcting_vel"] <= 0 or not 0 <= engine["contact_surface_layer"] <= .001 or
            engine["real_time_update_rate"] != 0):
        raise ValueError("global CFM/ERP/接触上限无效，或update_rate非0；本测量保留无节流运行条件")
    parameters = json.loads((workspace / "parameters.json").read_text(encoding="utf-8"))
    warmup = float(parameters.get("warmup_time_s", 0))
    interval = float(parameters.get("tail_interval_s", 0))
    count, repeats = parameters.get("tail_samples"), parameters.get("repeats")
    if (not math.isfinite(warmup) or not .5 <= warmup <= 3 or
            not math.isfinite(interval) or not .1 <= interval <= .2 or type(count) is not int or count != 5 or
            type(repeats) is not int or repeats != 2):
        raise ValueError("本仪器warmup需.5至3s、tail_interval需.1至.2s、tail_samples=5、repeats=2以保证观测与75s预算")
    warmup_steps, interval_steps = round(warmup/dt), round(interval/dt)
    if max(abs(warmup_steps*dt-warmup), abs(interval_steps*dt-interval)) > dt*.1:
        raise ValueError("warmup和tail_interval需对应整数物理步，避免时间取整混淆")
    models = {}
    for name in NAMES:
        nodes = world.findall("model[@name='%s']" % name)
        if len(nodes) != 1:
            raise ValueError("需要唯一模型" + name)
        node = nodes[0]
        dynamic = name in PROBES
        if (node.findtext("static", "false").strip().lower() in ("true", "1")) == dynamic:
            raise ValueError(name + "static与角色不一致")
        links = node.findall("link")
        if len(links) != 1:
            raise ValueError(name + "需要单link")
        link = links[0]
        equal(pose(link, name), [0]*6, name + "link原点")
        location = pose(node, name)
        equal(location[3:], [0]*3, name + "水平初始姿态")
        shapes = []
        for kind in ("collision", "visual"):
            entries = link.findall(kind)
            if len(entries) != 1:
                raise ValueError(name + "需要单collision/visual")
            entry = entries[0]
            equal(pose(entry, name + kind), [0]*6, name + kind + "几何原点")
            shape = "sphere" if dynamic else "box"
            dimensions = numbers(entry.findtext("geometry/"+shape+("/radius" if dynamic else "/size")), 1 if dynamic else 3, name + kind)
            if min(dimensions) <= 0:
                raise ValueError(name + "几何尺寸需正值")
            shapes.append(dimensions)
        equal(shapes[0], shapes[1], name + "collision/visual几何")
        collision = link.find("collision")
        if collision.findtext("max_contacts") != "1":
            raise ValueError(name + "本rig只允许单接触点")
        surface = {key: scalar(collision, "surface/contact/ode/"+key, name) for key in ("kp", "kd", "max_vel", "min_depth")}
        surface.update(restitution=scalar(collision, "surface/bounce/restitution_coefficient", name),
                       threshold=scalar(collision, "surface/bounce/threshold", name))
        if (not 500 <= surface["kp"] <= 1e6 or not 5 <= surface["kd"] <= 300 or
                not .1 <= surface["max_vel"] <= 20 or not 0 <= surface["min_depth"] <= .0005 or
                surface["restitution"] != 0 or surface["threshold"] < 0):
            raise ValueError(name + "接触kp/kd需正且在rig量程内：kp[500,1e6]、kd[5,300]、max_vel[.1,20]、min_depth[0,.0005]、e=0")
        data = {"name": name, "dynamic": dynamic, "pose": location, "dimensions": shapes[0],
                "shape": "sphere" if dynamic else "box", "link": link.get("name"),
                "collision": collision.get("name"), "surface": surface}
        if dynamic:
            if node.findtext("allow_auto_disable", "true").strip().lower() not in ("false", "0"):
                raise ValueError(name + "需显式allow_auto_disable=false，不能用sleep伪装稳定")
            if link.findtext("gravity", "true").strip().lower() not in ("true", "1"):
                raise ValueError(name + "需要重力承载")
            for kind in ("linear", "angular"):
                if scalar(link, "velocity_decay/"+kind, name) != 0:
                    raise ValueError(name + "本rig关闭额外速度衰减，阻尼只来自接触kd")
            inertial = link.find("inertial")
            if inertial is None:
                raise ValueError(name + "缺少惯性")
            equal(pose(inertial, name), [0]*6, name + "均匀球COM")
            mass = scalar(inertial, "mass", name)
            if not .2 <= mass <= 5:
                raise ValueError(name + "球质量需.2至5kg")
            inertia = [scalar(inertial, "inertia/"+key, name) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
            equal(inertia, [.4*mass*shapes[0][0]**2]*3+[0]*3, name + "均匀球惯量")
            data.update(mass_kg=mass, inertia=inertia, allow_auto_disable=False)
        models[name] = data
    pairs = {}
    minimum_warmup = .6
    for suffix in ("soft", "firm"):
        ball, support = models["probe_"+suffix], models["support_"+suffix]
        equal(ball["pose"][:2], support["pose"][:2], "球需位于平台中心")
        radius = ball["dimensions"][0]
        top = support["pose"][2]+support["dimensions"][2]/2
        gap = ball["pose"][2]-top-radius
        if not 0 <= gap <= .005 or min(support["dimensions"][:2]) < 4*radius:
            raise ValueError(ball["name"] + "需平台内单点承载、初始gap[0,.005]m且平台足够宽")
        a, b = ball["surface"], support["surface"]
        kp = 1/(1/a["kp"]+1/b["kp"])
        kd = a["kd"]+b["kd"]
        denominator = dt*kp+kd
        cfm, erp = 1/denominator, dt*kp/denominator
        compression = ball["mass_kg"]*(-gravity[2])/kp
        # A static support has no ODE body; the dynamic body's local min_depth
        # is used by contact.cpp, rather than silently substituting global CFM.
        predicted = a["min_depth"]+compression
        if not .0001 <= predicted <= radius*.25:
            raise ValueError(ball["name"] + "预测承载重叠不在可分辨且小变形rig量程[.0001,R/4]m")
        discriminant = kd*kd-4*ball["mass_kg"]*kp
        rate = 2*kp/(kd+math.sqrt(discriminant)) if discriminant >= 0 else kd/(2*ball["mass_kg"])
        settle_window = 5/rate+math.sqrt(2*gap/-gravity[2])
        minimum_warmup = max(minimum_warmup, settle_window)
        required_cap = cfm*ball["mass_kg"]*(-gravity[2])+.25
        if min(a["max_vel"], engine["contact_max_correcting_vel"]) < required_cap:
            raise ValueError(ball["name"] + "纠穿速度上限过低，会截断稳态软约束目标；请提高动态max_vel及global cap")
        pairs[ball["name"]] = {"support_name": support["name"], "radius_m": radius, "pad_top_z": top,
                               "contact_center_z": top+radius, "kp_pair_N_m": kp, "kd_pair_Ns_m": kd,
                               "derived_local_cfm": cfm, "derived_local_erp": erp,
                               "dynamic_min_depth_m": a["min_depth"], "predicted_overlap_m": predicted,
                               "elastic_compression_m": compression, "estimated_minimum_warmup_s": settle_window,
                               "required_steady_target_cap_m_s": required_cap}
    if warmup < minimum_warmup:
        raise ValueError("warmup观察窗口不足：当前%.6gs，按接触阻尼时标需至少%.6gs；增加warmup_time_s，时标只是仪器保守条件" % (warmup, minimum_warmup))
    if pairs["probe_soft"]["predicted_overlap_m"]-pairs["probe_firm"]["predicted_overlap_m"] < .005:
        raise ValueError("soft/firm预测几何重叠差不足.005m，超出本仪器可辨对比范围")
    return {"world": world.get("name", "default"), "workspace": workspace, "parameters": parameters,
            "engine": engine, "models": models, "pairs": pairs, "dt_s": dt,
            "warmup_steps": warmup_steps, "tail_interval_steps": interval_steps,
            "minimum_warmup_s": minimum_warmup, "declared_launch_seed": 42,
            "measurement_kind": "pose_derived_contact_geometry_and_sampled_z_fluctuation_not_internal_LCP_residual"}


def predictions(config):
    return {key: value for key, value in config.items() if key != "workspace"}
