"""Current-workspace mechanics predictions, independent of Gazebo transport."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ring_mesh import URI as RING_URI, check_ring

NAMES = ("fall_light", "fall_heavy", "tower_low", "tower_high", "flywheel_solid", "flywheel_ring")
MIN_FALL_DISTANCE = .02
MIN_COAST_ANGLE = .02
MAX_COAST_ANGLE = 2.8
# instrument.near uses an absolute angular-speed floor of .002 rad/s.
# A minimum six-times-larger signal keeps zero motion outside that tolerance,
# including long coast intervals where an angle-only lower bound is weaker.
MIN_ANGULAR_SPEED = .012


def numbers(text, count, label):
    try:
        values = [float(value) for value in (text or "").split()]
    except ValueError as error:
        raise ValueError(label + "必须是数值") from error
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError("%s需要%d个有限数值" % (label, count))
    return values


def scalar(node, path, label):
    result = numbers(node.findtext(path), 1, label + "/" + path)[0]
    return result


def pose(node, label):
    entry = node.find("pose")
    if entry is not None and entry.get("relative_to"):
        raise ValueError(label + "本课使用默认父坐标系")
    return numbers(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")


def close(actual, expected, label, relative=1e-5):
    if any(abs(a-b) > max(1e-9, abs(b)*relative) for a, b in zip(actual, expected)):
        raise ValueError("%s不符：当前%s，按当前几何/质量应为%s" % (label, actual, expected))


def eigenvalues(matrix):
    """Jacobi rotations for a finite symmetric 3x3 tensor; no NumPy needed."""
    a = [list(row) for row in matrix]
    for _ in range(32):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda pair: abs(a[pair[0]][pair[1]]))
        if abs(a[p][q]) <= max(abs(a[index][index]) for index in range(3))*1e-14:
            break
        angle = .5*math.atan2(2*a[p][q], a[q][q]-a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c*c*app-2*s*c*apq+s*s*aqq
        a[q][q] = s*s*app+2*s*c*apq+c*c*aqq
        a[p][q] = a[q][p] = 0.0
        for index in range(3):
            if index not in (p, q):
                old_p, old_q = a[index][p], a[index][q]
                a[index][p] = a[p][index] = c*old_p-s*old_q
                a[index][q] = a[q][index] = s*old_p+c*old_q
    return sorted(a[index][index] for index in range(3))


def inertia(link, label):
    node = link.find("inertial")
    if node is None:
        raise ValueError(label + "缺少inertial")
    mass = scalar(node, "mass", label)
    if mass <= 0:
        raise ValueError(label + "质量必须大于0 kg")
    center = pose(node, label + "/inertial")
    values = {key: scalar(node, "inertia/" + key, label) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    xx, yy, zz, xy, xz, yz = [values[key] for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
    matrix = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
    principal = eigenvalues(matrix)
    if min(principal) <= 0:
        raise ValueError(label + "惯性张量必须对称正定；主惯量均须大于0 kg·m²")
    if principal[0]+principal[1] < principal[2]-max(principal)*1e-8:
        raise ValueError(label + "主惯量违反三角不等式（正定仍不保证物理可实现）")
    if max(abs(value) for value in center[3:]) > 1e-8 or max(abs(xy), abs(xz), abs(yz)) > max(principal)*1e-7:
        raise ValueError(label + "本实验先把inertial坐标轴与link主轴对齐；旋转张量需R I Rᵀ，不能直接套标量公式")
    return {"mass": mass, "center": center, "tensor": values, "principal": principal}


def box_inertia(mass, size):
    x, y, z = size
    return [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]


def same_shape(link, label, kind):
    collisions = link.findall("collision")
    if len(collisions) != 1:
        raise ValueError(label + "本课采用单一碰撞包络")
    collision = collisions[0]
    if max(abs(value) for value in pose(collision, label + "/collision")) > 1e-8:
        raise ValueError(label + "碰撞包络应居中于link")
    shape = collision.find("geometry/" + kind)
    if shape is None:
        raise ValueError(label + "需要" + kind + "碰撞包络")
    if kind == "box":
        size = numbers(shape.findtext("size"), 3, label + " box size")
        if min(size) <= 0:
            raise ValueError(label + "box尺寸必须正值")
        return size
    radius, length = scalar(shape, "radius", label), scalar(shape, "length", label)
    if min(radius, length) <= 0:
        raise ValueError(label + "圆柱半径与长度必须正值")
    return [radius, length]


def configuration(workspace=None):
    workspace = Path(workspace or Path.cwd()).resolve()
    world = ET.parse(str(workspace / "world.sdf")).getroot().find("world")
    if world is None:
        raise ValueError("当前world.sdf缺少world")
    parameters = json.loads((workspace / "parameters.json").read_text(encoding="utf-8"))
    for key in ("freefall_steps", "tower_steps", "flywheel_coast_steps"):
        value = parameters.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or not 20 <= value <= 5000:
            raise ValueError(key + "应为20至5000的整数步数")
    torque = float(parameters.get("torque_z_Nm", 0))
    if not math.isfinite(torque) or not 1 <= torque <= 200:
        raise ValueError("torque_z_Nm应为1至200 Nm")
    dt = scalar(world, "physics/max_step_size", "physics")
    gravity = numbers(world.findtext("gravity"), 3, "gravity")
    if not .0001 <= dt <= .002 or abs(gravity[0])+abs(gravity[1]) > 1e-8 or gravity[2] >= 0:
        raise ValueError("本课采用.0001至.002s固定步长与竖直向下重力")
    fall_distance = -gravity[2]*dt*dt*parameters["freefall_steps"]*(parameters["freefall_steps"]+1)/2
    if fall_distance < MIN_FALL_DISTANCE:
        raise ValueError("自由落体采样落差不足：预测%.7gm，本测量器需至少%.3gm（绝对容差.003m）；请增加采样步数并保持落体未触地" %
                         (fall_distance, MIN_FALL_DISTANCE))
    models = {}
    for name in NAMES:
        nodes = world.findall("model[@name='%s']" % name)
        if len(nodes) != 1:
            raise ValueError("需要唯一模型：" + name)
        node = nodes[0]
        if node.findtext("static", "false").strip().lower() in ("true", "1"):
            raise ValueError(name + "必须为动态模型")
        links = node.findall("link")
        if len(links) != 1:
            raise ValueError(name + "应使用一个刚体link")
        link = links[0]
        if any(abs(value) > 1e-8 for value in pose(link, name + "/link")):
            raise ValueError(name + "link采用模型原点")
        data = inertia(link, name)
        data.update(name=name, pose=pose(node, name), link=link.get("name"), node=node, link_node=link)
        models[name] = data
    falls = [models[name] for name in NAMES[:2]]
    for data in falls:
        data["size"] = same_shape(data["link_node"], data["name"], "box")
        close(data["center"], [0]*6, data["name"] + "均匀箱质心")
        close([data["tensor"][key] for key in ("ixx", "iyy", "izz")], box_inertia(data["mass"], data["size"]), data["name"] + "均匀箱惯量")
        if max(abs(value) for value in data["pose"][3:]) > 1e-8 or data["link_node"].findtext("gravity", "true").strip().lower() in ("false", "0"):
            raise ValueError(data["name"] + "应零旋转且开启重力")
        elapsed = dt*parameters["freefall_steps"]
        if data["pose"][2] + gravity[2]*elapsed*(elapsed+dt)/2 <= data["size"][2]/2+.2:
            raise ValueError("定步自由落体会接近地面；提高初始z或减少步数")
    close(falls[0]["size"], falls[1]["size"], "自由落体两箱尺寸")
    close([falls[0]["pose"][2]], [falls[1]["pose"][2]], "自由落体两箱初始高度")
    if abs(falls[0]["mass"]-falls[1]["mass"]) < min(data["mass"] for data in falls)*.2:
        raise ValueError("自由落体两箱应具有明显不同的质量")
    ramp = world.find("model[@name='incline']")
    if ramp is None or ramp.findtext("static", "false").strip().lower() not in ("true", "1"):
        raise ValueError("需要静态incline斜面")
    ramp_pose = pose(ramp, "incline")
    if abs(ramp_pose[3])+abs(ramp_pose[5]) > 1e-8 or not .15 < -ramp_pose[4] < .5:
        raise ValueError("斜面采用绕世界Y轴负向旋转，坡角约9至28度")
    angle = -ramp_pose[4]
    column_mass = float(parameters.get("tower_column_mass_kg", 0))
    ballast_mass = float(parameters.get("tower_ballast_mass_kg", 0))
    if not all(math.isfinite(value) and value > 0 for value in (column_mass, ballast_mass)):
        raise ValueError("塔的柱质量和配重质量参数必须为正kg")
    for name in NAMES[2:4]:
        data = models[name]
        size = same_shape(data["link_node"], name, "box")
        data["size"] = size
        close([data["mass"]], [column_mass+ballast_mass], name + "柱与配重总质量")
        ballast = data["link_node"].find("visual[@name='ballast_visual']")
        if ballast is None:
            raise ValueError(name + "缺少配重几何标记ballast_visual")
        ballast_size = numbers(ballast.findtext("geometry/box/size"), 3, name + " ballast size")
        ballast_pose = pose(ballast, name + " ballast")
        if (min(ballast_size) <= 0 or max(abs(value) for value in ballast_pose[:2]+ballast_pose[3:]) > 1e-8 or
                any(abs(offset)+small/2 > large/2+1e-8 for offset, small, large in zip(ballast_pose[:3], ballast_size, size))):
            raise ValueError(name + "配重采用柱内、沿z移动且零旋转的小box")
        expected_center = [0, 0, ballast_pose[2]*ballast_mass/data["mass"]]
        close(data["center"][:3], expected_center, name + "按柱与配重求COM")
        column_i = box_inertia(column_mass, size)
        ballast_i = box_inertia(ballast_mass, ballast_size)
        shift = column_mass*expected_center[2]**2 + ballast_mass*(ballast_pose[2]-expected_center[2])**2
        expected_i = [column_i[0]+ballast_i[0]+shift, column_i[1]+ballast_i[1]+shift, column_i[2]+ballast_i[2]]
        close([data["tensor"][key] for key in ("ixx", "iyy", "izz")], expected_i, name + "COM处平行轴合成惯量")
        # Tower link origin is its geometric centre. COM z is relative to it.
        height = data["center"][2]+size[2]/2
        if abs(data["center"][1]) > 1e-8 or not 0 < height < size[2]:
            raise ValueError(name + "COM应在柱内部并处于XZ中面")
        close(data["pose"][3:], ramp_pose[3:], name + "初始方向与斜面")
        data["com_height"] = height
        data["support_projection_x"] = data["center"][0]-height*math.tan(angle)
        data["support_half_width"] = size[0]/2
        data["predicted_stable"] = abs(data["support_projection_x"]) < size[0]/2
        friction = scalar(data["link_node"], "collision/surface/friction/ode/mu", name)
        if friction < 1:
            raise ValueError(name + "本课用高摩擦抑制先滑动，mu应≥1")
    close(models["tower_low"]["size"], models["tower_high"]["size"], "两塔外尺寸")
    close([models["tower_low"]["mass"]], [models["tower_high"]["mass"]], "两塔总质量")
    if not models["tower_low"]["predicted_stable"] or models["tower_high"]["predicted_stable"]:
        raise ValueError("当前COM/坡角不满足本课低塔稳定、高塔倾覆的对比；请先重算支撑投影")
    high = models["tower_high"]
    h, b, mass = high["com_height"], high["support_half_width"], high["mass"]
    # Initial torque about the downhill support edge divided by its inertia.
    # This sets a conservative observation window for this no-slip experiment;
    # the support projection does not predict the complete tipping transient.
    initial_alpha = mass*(-gravity[2])*(h*abs(math.sin(angle))-b*math.cos(angle))/(high["tensor"]["iyy"]+mass*(h*h+b*b))
    if not math.isfinite(initial_alpha) or initial_alpha <= 0:
        raise ValueError("高塔初始倾覆角加速度需为正且有限，无法建立本测量器观察时标")
    tower_min_time = 2*math.sqrt(1/initial_alpha)+.2
    tower_time = (parameters["freefall_steps"]+parameters["tower_steps"])*dt
    tower_min_steps = max(20, math.ceil(tower_min_time/dt)-parameters["freefall_steps"])
    if tower_time < tower_min_time:
        raise ValueError("倾覆观察窗口不足：总时长(fall_steps+tower_steps)*dt=%.6gs，当前初始α=%.6grad/s²需要至少%.6gs；当前fall步数/步长下请将tower_steps设为至少%d；这是本课保守仪器窗口，并非完整瞬态预测" %
                         (tower_time, initial_alpha, tower_min_time, tower_min_steps))
    for name in NAMES[4:]:
        data = models[name]
        data["cylinder"] = same_shape(data["link_node"], name, "cylinder")
        if data["link_node"].findtext("gravity", "true").strip().lower() not in ("false", "0"):
            raise ValueError(name + "应gravity=false隔离重力")
        close(data["center"], [0]*6, name + "居中惯性坐标系")
        close(data["pose"][3:], [0]*3, name + "初始主轴对齐")
        radius, length = data["cylinder"]
        inner = 0 if name == "flywheel_solid" else float(parameters["ring_inner_radius_m"])
        if not math.isfinite(inner) or not 0 <= inner < radius:
            raise ValueError("环形飞轮内半径应在0与外半径之间")
        expected = [data["mass"]*(3*(radius*radius+inner*inner)+length*length)/12]*2 + [data["mass"]*(radius*radius+inner*inner)/2]
        close([data["tensor"][key] for key in ("ixx", "iyy", "izz")], expected, name + "中心主惯量")
        data["inner_radius"] = inner
        data["predicted_omega"] = torque*dt/data["tensor"]["izz"]
        data["predicted_coast_angle"] = data["predicted_omega"]*dt*parameters["flywheel_coast_steps"]
        if abs(data["predicted_coast_angle"]) < MIN_COAST_ANGLE:
            raise ValueError(name + "飞轮coast采样转角不足：本两姿态测量器需预测转角至少.02rad；请增加coast步数或扭矩")
        if abs(data["predicted_omega"]) < MIN_ANGULAR_SPEED:
            raise ValueError(name + "角速度采样信号不足：预测角速度需至少.012rad/s（绝对容差.002的6倍）；请增大torque_z_Nm或减小飞轮质量/惯量；这是仪器量程，不是旋转物理限制")
        if abs(data["predicted_coast_angle"]) >= MAX_COAST_ANGLE:
            raise ValueError(name + "预测coast转角需小于2.8rad：这是本课两姿态角差测量器的防混叠界限，并非Gazebo旋转极限；请减小扭矩或coast步数")
        if name == "flywheel_ring":
            if data["link_node"].findtext("visual[@name='body_visual']/geometry/mesh/uri") != RING_URI:
                raise ValueError("环形飞轮应使用本课离线annulus网格")
            check_ring(workspace, radius, inner, length)
    close(models["flywheel_solid"]["cylinder"], models["flywheel_ring"]["cylinder"], "两飞轮外尺寸")
    close([models["flywheel_solid"]["mass"]], [models["flywheel_ring"]["mass"]], "两飞轮总质量")
    limits = {"minimum_fall_distance_m": MIN_FALL_DISTANCE, "predicted_fall_distance_m": fall_distance,
              "tower_initial_alpha_rad_s2": initial_alpha, "tower_minimum_observation_s": tower_min_time,
              "tower_actual_observation_s": tower_time, "tower_minimum_steps_given_fall": tower_min_steps,
              "minimum_coast_angle_rad": MIN_COAST_ANGLE,
              "maximum_coast_angle_exclusive_rad": MAX_COAST_ANGLE, "minimum_angular_speed_rad_s": MIN_ANGULAR_SPEED}
    return {"world": world.get("name", "default"), "workspace": workspace, "parameters": parameters,
            "dt": dt, "gravity": gravity, "incline_pose": ramp_pose, "slope_angle": angle,
            "instrument_limits": limits, "models": models}


def predictions(config):
    p, dt = config["parameters"], config["dt"]
    return {"dt_s": dt, "gravity_m_s2": config["gravity"], "parameters": dict(p),
            "instrument_limits": config["instrument_limits"],
            "fall_time_s": p["freefall_steps"]*dt,
            "predicted_fall_distance_m": -config["gravity"][2]*dt*dt*p["freefall_steps"]*(p["freefall_steps"]+1)/2,
            "slope_degrees": math.degrees(config["slope_angle"]),
            "models": {name: {key: value for key, value in data.items()
                               if key not in ("node", "link_node")} for name, data in config["models"].items()}}
