"""Restricted passive-rig configuration and independent mechanics predictions."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def values(text, count, label):
    try:
        result = [float(value) for value in (text or "").split()]
    except ValueError as error:
        raise ValueError(label + "必须为数值") from error
    if len(result) != count or not all(math.isfinite(value) for value in result):
        raise ValueError("%s需要%d个有限值" % (label, count))
    return result


def scalar(node, path, label):
    return values(node.findtext(path), 1, label+"/"+path)[0]


def pose(node, label):
    return values(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")


def equal(actual, expected, label, tolerance=1e-7):
    if len(actual) != len(expected) or any(abs(a-b) > max(tolerance, abs(b)*1e-6) for a, b in zip(actual, expected)):
        raise ValueError("%s不符：%s，应为%s" % (label, actual, expected))


def passive_joint(model, name, kind, parent, child, xyz):
    entries = model.findall("joint[@name='%s']" % name)
    if len(entries) != 1:
        raise ValueError("需要唯一关节" + name)
    joint = entries[0]
    if joint.get("type") != kind or joint.findtext("parent") != parent or joint.findtext("child") != child:
        raise ValueError(name + "关节类型/parent/child拓扑不符")
    entry = joint.find("pose")
    reference = child if kind == "fixed" else parent
    if entry is None or entry.get("relative_to") != reference:
        raise ValueError(name + "需显式joint.pose relative_to=" + reference)
    equal(pose(joint, name), [0]*6, name + "锚点")
    data = {"name": name, "type": kind, "parent": parent, "child": child, "pose": [0]*6, "pose_relative_to": reference}
    if kind != "fixed":
        axis = joint.find("axis/xyz")
        if axis is None or axis.get("expressed_in") != parent:
            raise ValueError(name + "需SDF1.7 xyz expressed_in=" + parent)
        equal(values(axis.text, 3, name + " axis"), xyz, name + "axis")
        if joint.find("axis/use_parent_model_frame") is not None:
            raise ValueError("SDF1.7不要混入legacy use_parent_model_frame")
        data.update(axis=xyz, expressed_in=parent,
                    lower=scalar(joint, "axis/limit/lower", name), upper=scalar(joint, "axis/limit/upper", name),
                    damping=scalar(joint, "axis/dynamics/damping", name), friction=scalar(joint, "axis/dynamics/friction", name))
        if data["lower"] >= data["upper"]:
            raise ValueError(name + "限位需lower<upper")
        for field in ("effort", "velocity"):
            if scalar(joint, "axis/limit/"+field, name) != -1:
                raise ValueError(name + "本被动rig保持effort/velocity=-1，不额外限幅或驱动")
        equal([scalar(joint,"axis/limit/"+field,name) for field in ("stiffness","dissipation")],
              [1e6,100],name+"本仪器固定stop stiffness/dissipation")
    return data


def body(link, name, shape):
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(name + "缺少惯性")
    equal(pose(inertial, name), [0]*6, name + "COM坐标")
    mass = scalar(inertial, "mass", name)
    if not .5 <= mass <= 2:
        raise ValueError(name + "本rig质量需.5至2kg")
    if link.findtext("gravity", "true").lower() not in ("true", "1"):
        raise ValueError(name + "需要重力驱动")
    for field in ("linear", "angular"):
        if scalar(link, "velocity_decay/"+field, name) != 0:
            raise ValueError(name + "应关闭额外速度衰减")
    collisions = link.findall("collision")
    if len(collisions) != 1:
        raise ValueError(name + "本rig需要单碰撞几何")
    equal(pose(collisions[0], name), [0]*6, name + "碰撞几何中心")
    if shape == "sphere":
        radius = scalar(collisions[0], "geometry/sphere/radius", name)
        if not .06 <= radius <= .15:
            raise ValueError(name + "球半径需.06至.15m")
        dimensions = [radius]
        expected_i = [.4*mass*radius*radius]*3
    else:
        dimensions = values(collisions[0].findtext("geometry/box/size"), 3, name + " box")
        if not .1 <= min(dimensions) <= max(dimensions) <= .4:
            raise ValueError(name + "滑块各边需.1至.4m")
        x, y, z = dimensions
        expected_i = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
    inertia = [scalar(inertial, "inertia/"+key, name) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
    equal(inertia, expected_i+[0]*3, name + "均匀几何惯性")
    return {"mass_kg": mass, "dimensions": dimensions, "inertia": inertia, "pose": pose(link, name)}


def slider_position(time_s, mass, damping, friction, g):
    # Downward motion, q positive upward: friction opposes it by +F_f.
    return -(mass*g-friction)/damping*(time_s-mass/damping*(1-math.exp(-damping*time_s/mass)))


def configuration(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    root = ET.parse(str(workspace / "world.sdf")).getroot()
    if root.get("version") != "1.7" or root.find("world") is None:
        raise ValueError("本课采用显式参考系的SDF1.7 world")
    world = root.find("world")
    dt = scalar(world, "physics/max_step_size", "physics")
    gravity = values(world.findtext("gravity"), 3, "gravity")
    if not .0005 <= dt <= .002 or abs(gravity[0])+abs(gravity[1]) > 1e-8 or not 5 <= -gravity[2] <= 12:
        raise ValueError("本rig需dt[.0005,.002]s及竖直g[5,12]m/s²")
    if world.find("physics").get("type") != "ode" or world.findtext("physics/ode/solver/type") != "quick":
        raise ValueError("本rig使用ODE quick求解器")
    engine = {"solver_type": "quick", "max_step_size": dt, "gravity": gravity,
              "iters": scalar(world, "physics/ode/solver/iters", "physics")}
    if not engine["iters"].is_integer() or not 80 <= engine["iters"] <= 200:
        raise ValueError("本关节测量器采用整数iters[80,200]，求解器调参请回L08")
    models, joints = {}, {}
    for name, parent, child, fixed_name, moving_name, joint_type, axis, shape in (
        ("pendulum_rig", "frame", "bob", "anchor_fixed", "swing_joint", "revolute", [0,1,0], "sphere"),
        ("slider_rig", "guide", "carriage", "guide_fixed", "slide_joint", "prismatic", [0,0,1], "box")):
        entries = world.findall("model[@name='%s']" % name)
        if len(entries) != 1:
            raise ValueError("需要唯一模型" + name)
        model = entries[0]
        if model.findtext("static", "false").lower() not in ("false", "0"):
            raise ValueError(name + "需动态模型，不能把整个机构static固定")
        if model.findtext("allow_auto_disable", "true").lower() not in ("false", "0"):
            raise ValueError(name + "需关闭auto disable")
        equal(pose(model, name)[3:], [0]*3, name + "模型初始朝向")
        links = {node.get("name"):node for node in model.findall("link")}
        if set(links) != {parent, child} or len(model.findall("joint")) != 2:
            raise ValueError(name + "本rig限定两link与两joint的树拓扑")
        equal(pose(links[parent], parent), [0]*6, parent + "固定基座link原点")
        # The parent link itself stays dynamic; a world fixed joint removes DOF.
        for link_name,link in links.items():
            if link.findtext("kinematic","false").lower() not in ("false","0"):
                raise ValueError("本rig不以kinematic代替被动关节："+link_name)
            if link.findtext("gravity","true").lower() not in ("true","1"):
                raise ValueError(link_name+"需gravity=true")
            for field in ("linear","angular"):
                if scalar(link,"velocity_decay/"+field,link_name)!=0:
                    raise ValueError(link_name+"应关闭额外速度衰减")
        anchor_inertial=links[parent].find("inertial")
        if anchor_inertial is None:raise ValueError(parent+"缺少动态基座惯性")
        equal(pose(anchor_inertial,parent),[0]*6,parent+"COM")
        anchor_mass=scalar(anchor_inertial,"mass",parent)
        if not .5<=anchor_mass<=2:raise ValueError(parent+"动态基座质量需.5至2kg")
        anchor_i=[scalar(anchor_inertial,"inertia/"+key,parent) for key in ("ixx","iyy","izz","ixy","ixz","iyz")]
        equal(anchor_i,[anchor_mass*.15**2/6]*3+[0]*3,parent+".15m基座立方体惯性")
        joints[name] = {fixed_name: passive_joint(model, fixed_name, "fixed", "world", parent, None),
                        moving_name: passive_joint(model, moving_name, joint_type, parent, child, axis)}
        # A fixed joint's explicit pose uses its child, since world is not a
        # model-local named frame in SDFormat's pose-expression graph.
        models[name] = {"pose": pose(model, name), "parent_link": parent, "child_link": child,
                        "parent":{"mass_kg":anchor_mass,"inertia":anchor_i,"pose":[0]*6},
                        "child": body(links[child], child, shape)}
    pend_model = world.find("model[@name='pendulum_rig']")
    bob = models["pendulum_rig"]["child"]
    raw = bob["pose"]
    theta = raw[4]
    if not .1 <= abs(theta) <= .25 or abs(raw[1])+abs(raw[3])+abs(raw[5]) > 1e-8:
        raise ValueError("初始几何摆角需|theta0|在[.1,.25]rad，且只有绕Y转动")
    length = math.hypot(raw[0], raw[2])
    if not .6 <= length <= 1.1:
        raise ValueError("COM到铰轴长度L需.6至1.1m")
    equal(raw[:3], [-length*math.sin(theta), 0, -length*math.cos(theta)], "初始bob位置需随Ry(theta0)同步旋转")
    if models["pendulum_rig"]["pose"][2]-length-bob["dimensions"][0] < .3:
        raise ValueError("摆球最低位置需离地至少.3m，避免地面碰撞改变自由摆")
    rod = pend_model.find("link[@name='bob']/visual[@name='rod_visual']")
    if rod is None:
        raise ValueError("缺少质量忽略的rod_visual")
    equal(pose(rod, "rod_visual"), [0,0,length/2,0,0,0], "示意杆中心")
    equal([scalar(rod, "geometry/cylinder/length", "rod")], [length], "示意杆长度")
    swing = joints["pendulum_rig"]["swing_joint"]
    if (not 0 <= swing["damping"] <= .01 or swing["friction"] != 0 or
            swing["lower"] > min(0,-2*theta)-.08 or swing["upper"] < max(0,-2*theta)+.08):
        raise ValueError("自由小角摆需damping[0,.01]、friction=0、限位远离预计振幅")
    mass = bob["mass_kg"]
    ip = bob["inertia"][1]+mass*length*length
    period0 = 2*math.pi*math.sqrt(ip/(mass*(-gravity[2])*length))
    zeta = swing["damping"]/(2*math.sqrt(ip*mass*(-gravity[2])*length))
    corrected_period = period0/math.sqrt(1-zeta*zeta)*(1+theta*theta/16+11*theta**4/3072)
    pendulum = {"length_m":length,"radius_m":bob["dimensions"][0],"mass_kg":mass,"initial_angle_rad":theta,
                "I_com_kg_m2":bob["inertia"][1],"I_pivot_kg_m2":ip,"small_angle_period_s":period0,
                "corrected_period_s":corrected_period,"damping_ratio":zeta}
    carriage = models["slider_rig"]["child"]
    equal(carriage["pose"], [0]*6, "carriage初始位姿/关节零位")
    slide = joints["slider_rig"]["slide_joint"]
    if not -.8 <= slide["lower"] <= -.35 or not .05 <= slide["upper"] <= .2 or not .1 <= slide["damping"] <= 2 or not 0 <= slide["friction"] <= .5:
        raise ValueError("slide需lower[-.8,-.35]m/upper[.05,.2]、damping[.1,2]、friction[0,.5]")
    if models["slider_rig"]["pose"][2]+slide["lower"]-carriage["dimensions"][2]/2 < .2:
        raise ValueError("滑轨下限位需离地，避免把落地碰撞冒充joint限位")
    early_time = .2
    early_q = slider_position(early_time, carriage["mass_kg"], slide["damping"], slide["friction"], -gravity[2])
    if not early_q < -.08 or early_q < slide["lower"]+.05:
        raise ValueError("滑轨早期位移不足.08m或采样前已接近限位")
    slider = {"mass_kg":carriage["mass_kg"],"damping_Ns_m":slide["damping"],"friction_N":slide["friction"],
              "lower_m":slide["lower"],"upper_m":slide["upper"],"early_time_s":early_time,"predicted_early_q_m":early_q}
    parameters = json.loads((workspace/"parameters.json").read_text(encoding="utf-8"))
    interval = float(parameters.get("sample_interval_s", 0));periods = float(parameters.get("periods_to_observe", 0))
    if not .03 <= interval <= .06 or not 2.8 <= periods <= 3.2 or interval > corrected_period/24:
        raise ValueError("采样需interval[.03,.06]s、每周期至少24点、观察2.8至3.2周期")
    sample_steps = round(interval/dt)
    if max(abs(sample_steps*dt-interval), abs(round(early_time/interval)*interval-early_time)) > 1e-8:
        raise ValueError("采样间隔须对应整数物理步，并覆盖t=.2s滑轨样本")
    sample_count = math.ceil(periods*corrected_period/interval)
    if sample_count > 180 or sample_count*interval > 8:
        raise ValueError("采样超过180点/8仿真秒的本机仪器预算；调整interval或观察周期数")
    return {"world":world.get("name","default"),"workspace":workspace,"engine":engine,"models":models,"joints":joints,
            "pendulum":pendulum,"slider":slider,"parameters":parameters,"dt_s":dt,
            "sample_steps":sample_steps,"sample_count":sample_count,"sample_interval_s":interval}


def predictions(config):
    return {key:value for key,value in config.items() if key!="workspace"}
