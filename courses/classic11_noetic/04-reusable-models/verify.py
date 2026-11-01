#!/usr/bin/env python3
"""Verify offline include resolution, live template geometry and real falling instances.

All helpers are local to this script. Configuration always comes from cwd,
which is the learner's workspace, rather than the installed source template.
"""
import json
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from _control_support.models import pose as native_model_pose

DEADLINE = None
POSITION_TOLERANCE = .015
ANGLE_TOLERANCE = .015

def run(*args):
    remaining=DEADLINE-time.monotonic()
    if remaining<=.1:
        raise RuntimeError("位姿验收达到有界时间上限")
    result=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                          timeout=min(2.1,remaining))
    if result.returncode:
        raise RuntimeError("命令失败：%s\n%s"%(" ".join(args),(result.stderr or result.stdout)[-1000:]))
    return result.stdout


def finite(text,count,label):
    try:
        value=[float(x) for x in (text or "").split()]
    except ValueError as exc:
        raise ValueError("%s必须是数值"%label) from exc
    if len(value)!=count or not all(math.isfinite(x) for x in value):
        raise ValueError("%s需要%d个有限数值；pose用6数米/弧度，box size用3个长度值"%(label,count))
    return value


def normalized(q):
    length=math.sqrt(sum(v*v for v in q))
    if not math.isfinite(length) or length<1e-12:
        raise ValueError("无效的零四元数或非有限四元数")
    return [v/length for v in q]


def quaternion(rpy):
    r,p,y=[v/2 for v in rpy]
    sr,cr=math.sin(r),math.cos(r); sp,cp=math.sin(p),math.cos(p); sy,cy=math.sin(y),math.cos(y)
    return normalized([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                       cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def transform(raw):
    return (raw[:3],quaternion(raw[3:]))


def protobuf_message(text):
    # Parse nested DebugString fields instead of a regex that could mistake an
    # inertial/visual pose for the link's own model-relative pose.
    tokens=re.findall(r'"(?:\\.|[^"\\])*"|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_]*|[{}:]',text)
    cursor=[0]
    def message(nested=False):
        result={}
        while cursor[0]<len(tokens):
            key=tokens[cursor[0]]; cursor[0]+=1
            if key=="}":
                if not nested:
                    raise RuntimeError("运行消息包含多余右括号")
                return result
            if cursor[0]>=len(tokens):
                raise RuntimeError("运行消息字段缺少值")
            if tokens[cursor[0]]==":":
                cursor[0]+=1
            if tokens[cursor[0]]=="{":
                cursor[0]+=1; value=message(True)
            else:
                value=tokens[cursor[0]]; cursor[0]+=1
                if value.startswith('"'):
                    value=json.loads(value)
            result.setdefault(key,[]).append(value)
        if nested:
            raise RuntimeError("运行消息缺少右括号")
        return result
    return message()


def field(message,name):
    if name not in message:
        raise RuntimeError("运行消息缺少字段%s"%name)
    return message[name][0]


def message_pose(message):
    position=field(message,"position"); orientation=field(message,"orientation")
    t=[float(field(position,key)) for key in ("x","y","z")]
    q=[float(field(orientation,key)) for key in ("x","y","z","w")]
    if not all(math.isfinite(v) for v in t+q):
        raise RuntimeError("运行消息的pose含非有限值")
    if abs(math.sqrt(sum(v*v for v in q))-1.0)>1e-3:
        raise RuntimeError("运行消息不是单位四元数；请检查姿态接口的数据约定")
    return (t,normalized(q))


def compare(label,actual,expected):
    distance=math.sqrt(sum((a-b)**2 for a,b in zip(actual[0],expected[0])))
    # q and -q denote the same rotation. The absolute dot product also avoids
    # false failure when a reported Euler angle wraps around +/-pi or 2pi.
    dot=abs(sum(a*b for a,b in zip(normalized(actual[1]),normalized(expected[1]))))
    angle=2*math.acos(max(-1.0,min(1.0,dot)))
    if distance>POSITION_TOLERANCE or angle>ANGLE_TOLERANCE:
        raise AssertionError("%s不符：预测位置%s/四元数%s，测量位置%s/四元数%s，位置误差%.6fm、角距%.6frad；保存文件后请重载项目"%
                             (label,expected[0],expected[1],actual[0],actual[1],distance,angle))
    print("PASS：%s；预测位置=%s，测量=%s；位置误差%.6fm，四元数角距%.6frad"%
          (label,[round(v,5) for v in expected[0]],[round(v,5) for v in actual[0]],distance,angle),flush=True)

# Resource and experiment checks follow.

def scalar(node,name,label):
    value=float(node.findtext(name,"nan"))
    if not math.isfinite(value):
        raise ValueError("%s缺少有效%s"%(label,name))
    return value


def raw_pose(node,label,centered=False):
    pose=node.find("pose")
    if pose is not None and pose.get("relative_to"):
        raise ValueError("本课include布局采用默认世界参考，%s暂不添加relative_to"%label)
    raw=finite(node.findtext("pose","0 0 0 0 0 0"),6,label+" pose")
    if abs(raw[3])+abs(raw[4])>1e-8:
        raise ValueError("本课箱子只改变yaw，%s的roll/pitch应为0"%label)
    if centered and any(abs(v)>1e-8 for v in raw):
        raise ValueError("公共箱子模板的%s应保持居中零pose"%label)
    return raw


def box_size(node,label):
    size=finite(node.findtext("geometry/box/size"),3,label+" box size")
    if min(size)<=0:
        raise ValueError("%s的box尺寸必须为正"%label)
    return size


def inside(path,parent):
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def model_resource(uri,workspace):
    if not uri or not uri.startswith("model://"):
        raise ValueError("include缺少有效本地模型URI；本课使用model://teach_crate")
    bundle=uri[len("model://"):].strip().strip("/")
    if not bundle or "/" in bundle or "\\" in bundle or bundle in (".",".."):
        raise ValueError("本课URI应指向模型包目录：model://teach_crate，不直接指向model.sdf")
    search=[Path(p).expanduser() for p in os.environ.get("GAZEBO_MODEL_PATH","").split(os.pathsep) if p]
    directory=next((base/bundle for base in search if (base/bundle).is_dir()),None)
    if directory is None:
        raise ValueError("找不到本地模型资源%s；GAZEBO_MODEL_PATH应包含models的父集合目录（本课为%s）"%(uri,workspace/"models"))
    if not inside(directory,workspace):
        raise ValueError("模型URI%s命中练习目录外的同名资源%s；请先使用当前工作副本的models路径"%(uri,directory))
    config=directory/"model.config"
    if not config.is_file():
        raise ValueError("模型包缺少model.config：%s"%config)
    metadata=ET.parse(str(config)).getroot()
    if metadata.tag!="model" or not metadata.findtext("name","").strip() or not metadata.findtext("version","").strip():
        raise ValueError("model.config需要model根、name和模型发布version；不要与sdf格式version混淆")
    entries=metadata.findall("sdf")
    if len(entries)!=1:
        raise ValueError("本课model.config采用一个明确的sdf入口，请保留单个sdf元素")
    entry=entries[0]
    filename=(entry.text or "").strip()
    if not filename:
        raise ValueError("model.config的sdf元素缺少入口文件名")
    target=directory/filename
    if not inside(target,directory):
        raise ValueError("模型入口文件应位于本模型包目录内")
    if not target.is_file():
        raise ValueError("模型入口文件不存在：%s；检查model.config的sdf文本"%target)
    root=ET.parse(str(target)).getroot()
    if root.tag!="sdf" or root.get("version") not in ("1.6","1.7") or entry.get("version")!=root.get("version"):
        raise ValueError("model.config入口sdf格式version与model.sdf根版本不一致，或超出本课SDF1.6/1.7范围")
    models=root.findall("model")
    if len(models)!=1:
        raise ValueError("公共model.sdf应描述一个模型，而不是world或多个模型")
    model=models[0]
    if model.findtext("static","false").strip().lower() in ("true","1"):
        raise ValueError("本课共享箱子需要动态static=false，以实际落体验证物理几何")
    raw_pose(model,"模板model",True)
    links=model.findall("link")
    if len(links)!=1:
        raise ValueError("本课共享箱子采用一个居中link")
    link=links[0];raw_pose(link,"模板link",True)
    collisions=link.findall("collision");visuals=link.findall("visual")
    if len(collisions)!=1 or len(visuals)!=1:
        raise ValueError("本课共享箱子应有一个box collision和一个box visual")
    collision,visual=collisions[0],visuals[0]
    raw_pose(collision,"模板collision",True);raw_pose(visual,"模板visual",True)
    size=box_size(collision,"模板collision")
    if any(abs(a-b)>1e-8 for a,b in zip(size,box_size(visual,"模板visual"))):
        raise ValueError("公共箱子的collision与visual尺寸应同步修改")
    inertial=link.find("inertial")
    if inertial is None:
        raise ValueError("动态箱子缺少inertial")
    raw_pose(inertial,"模板inertial",True)
    mass=scalar(inertial,"mass","模板inertial")
    if mass<=0:
        raise ValueError("动态箱子质量必须为正")
    inertia=inertial.find("inertia")
    if inertia is None:
        raise ValueError("动态箱子缺少惯性矩阵")
    sx,sy,sz=size
    expected_i=[mass*(sy*sy+sz*sz)/12,mass*(sx*sx+sz*sz)/12,mass*(sx*sx+sy*sy)/12]
    for key,expected in zip(("ixx","iyy","izz"),expected_i):
        value=scalar(inertia,key,"惯性矩阵")
        if abs(value-expected)>max(expected*1e-4,1e-10):
            raise ValueError("模板%s=%.8g，应为%.8g kg·m²；公共尺寸改变后也要更新惯性"%(key,value,expected))
    for key in ("ixy","ixz","iyz"):
        value=float(inertia.findtext(key,"0"))
        if not math.isfinite(value) or abs(value)>max(max(expected_i)*1e-4,1e-10):
            raise ValueError("本课均匀主轴对齐箱子的惯性交叉项应为0")
    return {"uri":uri,"directory":directory.resolve(),"entry":target.resolve(),
            "link":link.get("name"),"size":size,"mass":mass,"inertia":expected_i}


def configuration():
    workspace=Path.cwd().resolve()
    world=ET.parse(str(workspace/"world.sdf")).getroot().find("world")
    if world is None:
        raise ValueError("工作副本world.sdf缺少world")
    ground=world.find("model[@name='ground']")
    if ground is None or ground.findtext("static","false").strip().lower() not in ("true","1"):
        raise ValueError("本课保留静态ground碰撞平面")
    floor=ground.find("link/collision")
    if floor is None or floor.find("geometry/plane") is None:
        raise ValueError("ground必须有plane collision")
    ground_pose=raw_pose(ground,"ground")
    if any(abs(v)>1e-8 for v in ground_pose):
        raise ValueError("本课ground位于世界原点，不移动或旋转地面")
    gravity=finite(world.findtext("gravity","0 0 -9.8"),3,"gravity")
    if abs(gravity[0])+abs(gravity[1])>1e-8 or gravity[2]>=0:
        raise ValueError("本课使用竖直向下重力")
    includes=world.findall("include")
    if not 2<=len(includes)<=4:
        raise ValueError("本课复用验收限定2至4个include实例；至少两个用于独立性比较，更多实例留给后续批量实验课")
    resource=None;instances=[];names=set()
    for include in includes:
        name=include.findtext("name","").strip()
        if not name or name in names:
            raise ValueError("每个include需要显式、唯一的实例name")
        if world.find("model[@name='%s']"%name) is not None:
            raise ValueError("include实例%s与世界已有model重名"%name)
        names.add(name)
        current=model_resource(include.findtext("uri","").strip(),workspace)
        if resource is None:
            resource=current
        elif current["entry"]!=resource["entry"]:
            raise ValueError("本课各include应引用同一公共模型入口，以验证共享模板")
        initial=raw_pose(include,"include "+name)
        if initial[2]<resource["size"][2]/2-.01:
            raise ValueError("实例%s初始中心高度低于箱子半高，已经穿入地面；请同步调整include pose.z"%name)
        instances.append({"name":name,"initial":initial})
    radius=math.hypot(resource["size"][0],resource["size"][1])/2
    for i,left in enumerate(instances):
        for right in instances[i+1:]:
            distance=math.hypot(left["initial"][0]-right["initial"][0],left["initial"][1]-right["initial"][1])
            if distance<2*radius+.1:
                raise ValueError("实例%s/%s水平距离过近；本课布局需避免箱子相互碰撞"%(left["name"],right["name"]))
    print("资源解析：%s → %s → %s"%(resource["uri"],resource["directory"]/'model.config',resource["entry"]),flush=True)
    print("公共模板：box=%s m，质量=%.4g kg，落地中心预测=%.4f m；实例名=%s"%
          (resource["size"],resource["mass"],resource["size"][2]/2,','.join(p["name"] for p in instances)),flush=True)
    return world.get("name","default"),resource,instances


def read_model(world,name,flag):
    """Retry one timed-out, idempotent read inside the existing total budget.

    A fresh Classic CLI can occasionally miss a transport response during
    discovery. A second read is safe; reissuing a move or world-control request
    would have a different risk and is intentionally not handled here.
    """
    if flag not in ("-p","-i"):
        raise ValueError("只读模型入口仅允许-p/-i")
    for attempt in range(2):
        try:
            return run("gz","model","-w",world,"-m",name,flag)
        except subprocess.TimeoutExpired as exc:
            remaining=DEADLINE-time.monotonic()
            if attempt==1:
                raise RuntimeError("只读模型查询%s %s连续超时；有限重试已用完（共2次），剩余总预算%.2fs"%
                                   (name,flag,max(0.0,remaining))) from exc
            if remaining<=.3:
                raise RuntimeError("只读模型查询%s %s超时，剩余总预算不足，不再重试"%(name,flag)) from exc
            print("RETRY：只读模型查询%s %s首次超时（%.2fs），在剩余总预算%.2fs内重试1次；不重发状态变更命令"%
                  (name,flag,exc.timeout,remaining),flush=True)


def live_pose(world,name):
    raw=native_model_pose(world,name,DEADLINE)
    if raw is not None:return raw
    output=read_model(world,name,"-p")
    for line in reversed(output.splitlines()):
        if len(line.split())==6:
            try:
                return finite(line,6,"实时位姿 "+name)
            except ValueError:
                pass
    raise RuntimeError("未获取实例%s的实时位姿；检查URI解析、实例名和服务器日志"%name)


def live_template(world,name,resource):
    message=protobuf_message(read_model(world,name,"-i"))
    link=next((entry for entry in message.get("link",[]) if field(entry,"name") in
               (resource["link"],name+"::"+resource["link"])),None)
    if link is None:
        raise RuntimeError("实例%s的运行模型没有模板link %s"%(name,resource["link"]))
    sizes=[]
    for kind in ("collision","visual"):
        boxes=[]
        for entry in link.get(kind,[]):
            if "geometry" in entry and "box" in field(entry,"geometry"):
                size=field(field(field(entry,"geometry"),"box"),"size")
                boxes.append([float(field(size,key)) for key in ("x","y","z")])
        if len(boxes)!=1:
            raise RuntimeError("实例%s的真实%s没有唯一box几何"%(name,kind))
        if any(abs(a-b)>1e-5 for a,b in zip(boxes[0],resource["size"])):
            raise AssertionError("实例%s的真实%s尺寸%s与模板%s不符；保存公共模型后请重载项目"%
                                 (name,kind,boxes[0],resource["size"]))
        sizes.append(boxes[0])
    inertial=field(link,"inertial")
    mass=float(field(inertial,"mass"))
    if abs(mass-resource["mass"])>max(resource["mass"]*1e-4,1e-8):
        raise AssertionError("实例%s的真实质量%.6g与公共模板%.6g不符"%(name,mass,resource["mass"]))
    for key,expected in zip(("ixx","iyy","izz"),resource["inertia"]):
        if abs(float(field(inertial,key))-expected)>max(expected*1e-4,1e-8):
            raise AssertionError("实例%s真实惯性%s没有使用更新后的公共模板"%(name,key))
    print("PASS：%s真实collision/visual尺寸=%s，质量/惯性与共享入口一致"%(name,sizes[0]),flush=True)


def paused_state(world):
    output=run("gz","topic","-e","/gazebo/%s/world_stats"%world,"-d","1")
    values=re.findall(r"\bpaused:\s*(true|false)\b",output)
    if not values:
        raise RuntimeError("world_stats未返回暂停状态")
    return values[-1]=="true"


def pause(world,value):
    run("gz","topic","-p","/gazebo/%s/world_control"%world,
        "-m","pause: %s"%("true" if value else "false"))


def move(world,name,raw):
    args=["gz","model","-w",world,"-m",name]
    for key,value in zip(("-x","-y","-z","-R","-P","-Y"),raw):
        args.extend((key,str(value)))
    run(*args)


def verify():
    global DEADLINE
    DEADLINE=time.monotonic()+24
    world,resource,instances=configuration()
    original_paused=paused_state(world)
    original={};moved=False;experiment_error=None
    expected_z=resource["size"][2]/2
    tolerance=max(.025,resource["size"][2]*.04)
    lift=max(2.0,expected_z+1.0)
    try:
        pause(world,True)
        if not paused_state(world):
            raise RuntimeError("未确认暂停，未开始移动实例")
        for instance in instances:
            name=instance["name"]
            live_template(world,name,resource)
            raw=live_pose(world,name);original[name]=raw
            expected=list(instance["initial"]);expected[2]=raw[2]
            compare(name+"的include平面布局/yaw",transform(raw),transform(expected))
        for index,instance in enumerate(instances):
            target=list(instance["initial"]);target[2]=lift
            moved=True
            move(world,instance["name"],target)
            compare(instance["name"]+"暂停抬升已确认",transform(live_pose(world,instance["name"])),transform(target))
            if index==0:
                other=instances[-1]["name"]
                compare("只移动%s时%s保持原位，证明实例独立"%(instance["name"],other),
                        transform(live_pose(world,other)),transform(original[other]))
            instance["settled"]=[];instance["last"]=target
        pause(world,False)
        while DEADLINE-time.monotonic()>.5:
            for instance in instances:
                raw=live_pose(world,instance["name"]);instance["last"]=raw
                if abs(raw[2]-expected_z)<=tolerance and max(abs(raw[3]),abs(raw[4]))<.08:
                    instance["settled"].append((time.monotonic(),raw[2]))
                    instance["settled"]=instance["settled"][-4:]
                else:
                    instance["settled"]=[]
            stable=all(len(p["settled"])>=3 and p["settled"][-1][0]-p["settled"][0][0]>=.25 and
                       max(z for _,z in p["settled"])-min(z for _,z in p["settled"])<=tolerance*.4 for p in instances)
            if stable:
                for instance in instances:
                    if lift-instance["last"][2]<.3:
                        raise AssertionError("实例%s没有呈现真实下落"%instance["name"])
                    print("PASS：%s真实落体，预测中心z=%.4f，测量z=%.4f，容差±%.4fm"%
                          (instance["name"],expected_z,instance["last"][2],tolerance),flush=True)
                print("PASS：所有实例使用同一模板，却保有各自名称和布局；参数变更已由真实几何和落体验证。",flush=True)
                return
            time.sleep(.07)
        raise AssertionError("实例未按公共模板高度稳定落地：%s；检查重载、碰撞、初始布局与服务器日志"%
                             '; '.join('%s: z=%.4f'%(p['name'],p['last'][2]) for p in instances))
    except Exception as exc:
        experiment_error=exc
        raise
    finally:
        DEADLINE=time.monotonic()+8
        cleanup_error=None
        try:
            if moved:
                pause(world,True)
                if not paused_state(world):
                    raise RuntimeError("恢复前未确认暂停")
                for name,raw in original.items():
                    move(world,name,raw)
                for name,raw in original.items():
                    compare("恢复实例%s原位姿"%name,transform(live_pose(world,name)),transform(raw))
        except Exception as exc:
            cleanup_error=exc
        finally:
            # Pose-read retry exhaustion must not skip restoring pause. This
            # final bounded reserve caps total worst-case time at 34.8s,
            # below the lesson's 45s process timeout.
            DEADLINE=max(DEADLINE,time.monotonic()+2.8)
            try:
                pause(world,original_paused)
                if paused_state(world)!=original_paused:
                    raise RuntimeError("未确认恢复原暂停状态")
                print("已恢复原暂停状态%s"%original_paused,flush=True)
            except Exception as exc:
                cleanup_error=RuntimeError("%s；暂停恢复也失败：%s"%(cleanup_error,exc)) if cleanup_error else exc
        if cleanup_error is not None:
            if experiment_error is not None:
                print("附加恢复失败：%s；原实验失败原因保留"%cleanup_error,file=sys.stderr,flush=True)
            else:
                raise cleanup_error
        else:
            print("已恢复所有实例原位姿；未修改模板或布局文件。",flush=True)


if __name__=="__main__":
    try:
        verify()
    except Exception as exc:
        print("FAIL：%s"%exc,file=sys.stderr,flush=True)
        sys.exit(1)
