#!/usr/bin/env python3
"""Measure real falls against geometry in the learner's cwd/world.sdf."""
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


DEADLINE = None
PAIRS = (("solid_probe", "solid_platform"), ("ghost_probe", "ghost_platform"))


def run(*args):
    remaining = DEADLINE - time.monotonic()
    if remaining <= 0.1:
        raise RuntimeError("验收达到时间上限；请检查服务器日志和实时因子")
    result = subprocess.run(args, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, timeout=min(2.1, remaining))
    if result.returncode:
        raise RuntimeError("命令失败：%s\n%s" %
                           (" ".join(args), (result.stderr or result.stdout)[-1000:]))
    return result.stdout


def vec(text, count, label):
    try:
        values = [float(v) for v in (text or "").split()]
    except ValueError as exc:
        raise ValueError("%s 必须填写数值" % label) from exc
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise ValueError("%s 需要%d个有限数值" % (label, count))
    return values


def offset(node, label, centered=False):
    element = node.find("pose")
    if element is not None and element.get("relative_to"):
        raise ValueError("%s 的 relative_to 不在本课的轴对齐几何验收范围内" % label)
    value = vec(node.findtext("pose", "0 0 0 0 0 0"), 6, label + " pose")
    if any(abs(v) > 1e-8 for v in value[3:]):
        raise ValueError("本课只修改平台高度和碰撞定义；请保持 %s 零转角" % label)
    if centered and any(abs(v) > 1e-8 for v in value[:3]):
        raise ValueError("%s 应与探针模型原点中心对齐" % label)
    return value[:3]


def add(a, b):
    return [x+y for x,y in zip(a,b)]


def dimensions(node, label):
    size = vec(node.findtext("geometry/box/size"), 3, label + " box size")
    if min(size) <= 0:
        raise ValueError("%s 的 box尺寸必须为正" % label)
    return size


def model_link(world, name, static):
    model = world.find("model[@name='%s']" % name)
    if model is None:
        raise ValueError("本课请保留模型名称 %s" % name)
    is_static = model.findtext("static", "false").strip().lower() in ("1", "true")
    if is_static != static:
        raise ValueError("%s 应为%s模型；static 是是否参与动力学推进，不是大质量替代" %
                         (name, "静态" if static else "动态"))
    links = model.findall("link")
    if len(links) != 1:
        raise ValueError("%s 本课限定一个link" % name)
    return model, links[0]


def probe_config(world, name):
    model, link = model_link(world, name, False)
    initial = offset(model, name)
    offset(link, name + " link", centered=True)
    collision = link.find("collision")
    visual = link.find("visual")
    inertial = link.find("inertial")
    if collision is None or visual is None or inertial is None:
        raise ValueError("动态探针%s需要collision、visual和inertial；允许缺collision的是平台" % name)
    if len(link.findall("collision")) != 1:
        raise ValueError("探针%s本课限定一个box collision" % name)
    for element, suffix in ((collision,"collision"), (visual,"visual"), (inertial,"inertial")):
        offset(element, name + " " + suffix, centered=True)
    size = dimensions(collision, name + " collision")
    if any(abs(a-b)>1e-8 for a,b in zip(size, dimensions(visual,name+" visual"))):
        raise ValueError("探针%s的collision/visual尺寸应一致；平台二者则允许有意不同" % name)
    mass = float(inertial.findtext("mass", "nan"))
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("探针%s质量必须为正" % name)
    inertia = inertial.find("inertia")
    if inertia is None:
        raise ValueError("探针%s缺少惯性" % name)
    sx, sy, sz = size
    expected_i = (mass*(sy*sy+sz*sz)/12, mass*(sx*sx+sz*sz)/12, mass*(sx*sx+sy*sy)/12)
    for key, expected in zip(("ixx","iyy","izz"), expected_i):
        value = float(inertia.findtext(key,"nan"))
        if not math.isfinite(value) or abs(value-expected)>max(expected*1e-4,1e-10):
            raise ValueError("探针%s的%s不符合均匀box尺寸和质量；请按首课公式更新惯性" % (name,key))
    for key in ("ixy","ixz","iyz"):
        value = float(inertia.findtext(key,"0"))
        if not math.isfinite(value) or abs(value)>max(max(expected_i)*1e-4,1e-10):
            raise ValueError("探针%s的惯性交叉项应为0" % name)
    return {"name":name, "initial":initial, "size":size, "half":sz/2}


def configuration():
    path = Path.cwd()/"world.sdf"
    world = ET.parse(str(path)).getroot().find("world")
    if world is None:
        raise ValueError("工作副本world.sdf缺少world")
    gravity = vec(world.findtext("gravity","0 0 -9.8"),3,"gravity")
    if abs(gravity[0])+abs(gravity[1])>1e-8 or gravity[2]>=0:
        raise ValueError("本课需要沿-z方向的重力")
    ground, ground_link = model_link(world,"ground",True)
    floor = ground_link.find("collision")
    if floor is None or floor.find("geometry/plane") is None:
        raise ValueError("本课需要ground的碰撞平面，不能只有地面外观")
    if vec(floor.findtext("geometry/plane/normal"),3,"地面法向") != [0.0,0.0,1.0]:
        raise ValueError("本课地面法向应为0 0 1")
    floor_z = add(add(offset(ground,"ground"),offset(ground_link,"ground link")),offset(floor,"ground collision"))[2]
    platforms, visible_tops = [], [floor_z]
    for _, name in PAIRS:
        model, link = model_link(world,name,True)
        initial = offset(model,name)
        origin = add(initial,offset(link,name+" link"))
        visual = link.find("visual")
        if visual is None:
            raise ValueError("平台%s应保留visual，以比较看得到和接触得到的差别" % name)
        visual_size = dimensions(visual,name+" visual")
        visible_tops.append(add(origin,offset(visual,name+" visual"))[2]+visual_size[2]/2)
        collisions = []
        for collision in link.findall("collision"):
            size = dimensions(collision,name+" collision")
            center = add(origin,offset(collision,name+" collision"))
            collisions.append((center,size))
            visible_tops.append(center[2]+size[2]/2)
        platforms.append({"name":name,"initial":initial,"collisions":collisions})
    probes = []
    for probe_name, platform_name in PAIRS:
        probe = probe_config(world,probe_name)
        support_z, support_name = floor_z,"ground碰撞平面"
        for platform in platforms:
            for center,size in platform["collisions"]:
                # Full footprint support avoids grading an unstable edge balance.
                inside = all(abs(probe["initial"][i]-center[i])+probe["size"][i]/2 <= size[i]/2+1e-8 for i in (0,1))
                top = center[2]+size[2]/2
                if inside and top>support_z:
                    support_z,support_name = top,platform["name"]+" collision"
        probe["expected"] = support_z+probe["half"]
        probe["support"] = support_name
        probe["tolerance"] = max(.025,probe["size"][2]*.05)
        print("预测：%s由%s支撑，中心z=%.4f m，容差±%.4f m" %
              (probe_name,support_name,probe["expected"],probe["tolerance"]),flush=True)
        probes.append(probe)
    lift = max(visible_tops)+max(1.0,max(p["size"][2] for p in probes))
    print("配置来源：%s；平台没有collision是一种合法教学配置" % path,flush=True)
    return world.get("name","default"),probes,platforms,lift


def pose(world,name):
    output = run("gz","model","-w",world,"-m",name,"-p")
    for line in reversed(output.splitlines()):
        if len(line.split())==6:
            try:
                value=[float(v) for v in line.split()]
                if all(math.isfinite(v) for v in value):
                    return value
            except ValueError:
                pass
    raise RuntimeError("无法读取%s的实时位姿：%r" % (name,output[-400:]))


def paused_state(world):
    output = run("gz","topic","-e","/gazebo/%s/world_stats"%world,"-d","1")
    values = re.findall(r"\bpaused:\s*(true|false)\b",output)
    if not values:
        raise RuntimeError("world_stats没有暂停状态，不能可靠恢复实验")
    return values[-1]=="true"


def set_paused(world,value):
    run("gz","topic","-p","/gazebo/%s/world_control"%world,
        "-m","pause: %s"%("true" if value else "false"))


def verify():
    global DEADLINE
    DEADLINE=time.monotonic()+12.0
    world,probes,platforms,lift=configuration()
    original_paused=paused_state(world)
    experiment_error=None
    try:
        set_paused(world,True)
        if not paused_state(world):
            raise RuntimeError("未确认暂停，未移动任何探针")
        for platform in platforms:
            actual=pose(world,platform["name"])
            if any(abs(a-b)>.02 for a,b in zip(actual[:3],platform["initial"])):
                raise RuntimeError("%s的实时位置与工作副本不符：文件%s，实测%s。保存后请点击重载项目。"%
                                   (platform["name"],platform["initial"],actual[:3]))
        for probe in probes:
            x,y,_=probe["initial"]
            run("gz","model","-w",world,"-m",probe["name"],
                "-x",str(x),"-y",str(y),"-z",str(lift),"-R","0","-P","0","-Y","0")
            actual=pose(world,probe["name"])
            if abs(actual[2]-lift)>.03:
                raise RuntimeError("%s暂停抬升未确认：目标%.3f，实测%.3f"%(probe["name"],lift,actual[2]))
            probe["high"]=actual[2]
            probe["settled"]=[]
            probe["last"]=actual
        print("两枚探针均已暂停抬升到%.3f m，开始真实落体"%lift,flush=True)
        set_paused(world,False)
        while DEADLINE-time.monotonic()>.5:
            for probe in probes:
                actual=pose(world,probe["name"])
                probe["last"]=actual
                now=time.monotonic()
                if abs(actual[2]-probe["expected"])<=probe["tolerance"] and max(abs(actual[3]),abs(actual[4]))<.08:
                    probe["settled"].append((now,actual[2]))
                    probe["settled"]=probe["settled"][-4:]
                else:
                    probe["settled"]=[]
            stable=all(len(p["settled"])>=3 and p["settled"][-1][0]-p["settled"][0][0]>=.25
                       and max(z for _,z in p["settled"])-min(z for _,z in p["settled"])<=p["tolerance"]*.4 for p in probes)
            if stable:
                for probe in probes:
                    if probe["high"]-probe["last"][2]<.3:
                        raise RuntimeError("%s未呈现足够实际下落"%probe["name"])
                    print("PASS：%s预测z=%.4f，测量z=%.4f，容差±%.4f m，支撑来源=%s"%
                          (probe["name"],probe["expected"],probe["last"][2],probe["tolerance"],probe["support"]),flush=True)
                observed=probes[0]["last"][2]-probes[1]["last"][2]
                predicted=probes[0]["expected"]-probes[1]["expected"]
                print("PASS：落点差预测=%.4f m，实际=%.4f m；补collision后差值接近0也属于正确结果"%(predicted,observed),flush=True)
                return
            time.sleep(.07)
        detail="；".join("%s预测%.4f、实测%s"%(p["name"],p["expected"],p["last"]) for p in probes)
        raise RuntimeError("两枚探针未按当前碰撞配置稳定落地：%s。检查是否保存并重载、碰撞覆盖、服务器运行状态。"%detail)
    except Exception as exc:
        experiment_error=exc
        raise
    finally:
        DEADLINE=time.monotonic()+3.0
        try:
            set_paused(world,original_paused)
            if paused_state(world)!=original_paused:
                raise RuntimeError("未确认恢复原暂停状态")
            print("已恢复原暂停状态：%s"%original_paused,flush=True)
        except Exception as exc:
            if experiment_error is not None:
                print("附加恢复失败：%s；原实验失败原因保留"%exc,file=sys.stderr,flush=True)
            else:
                raise


if __name__=="__main__":
    try:
        verify()
    except Exception as exc:
        print("FAIL：%s"%exc,file=sys.stderr,flush=True)
        sys.exit(1)
