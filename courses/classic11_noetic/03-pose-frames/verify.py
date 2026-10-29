#!/usr/bin/env python3
"""Resolve the workspace frame graph independently, then query real Gazebo poses.

Transforms use column vectors, Rz(yaw)*Ry(pitch)*Rx(roll), and unit xyzw
quaternions. The program never imports a parser or private helper from another
course, and never treats relative_to/attached_to as a mechanical constraint.
"""
import json
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path


DEADLINE = None
IDENTITY = ([0.0,0.0,0.0],[0.0,0.0,0.0,1.0])
POSITION_TOLERANCE = .01
ANGLE_TOLERANCE = .01


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
        raise ValueError("%s需要%d个有限数值；SDF1.7的pose是6数RPY，不能直接填7数四元数"%(label,count))
    return value


def normalized(q):
    length=math.sqrt(sum(v*v for v in q))
    if not math.isfinite(length) or length<1e-12:
        raise ValueError("无效的零四元数或非有限四元数")
    return [v/length for v in q]


def multiply(a,b):
    x,y,z,w=a; X,Y,Z,W=b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def quaternion(rpy):
    r,p,y=[v/2 for v in rpy]
    sr,cr=math.sin(r),math.cos(r); sp,cp=math.sin(p),math.cos(p); sy,cy=math.sin(y),math.cos(y)
    return normalized([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                       cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def rotate(q,v):
    result=multiply(multiply(q,list(v)+[0.0]),[-q[0],-q[1],-q[2],q[3]])
    return result[:3]


def compose(a,b):
    rotated=rotate(a[1],b[0])
    return ([a[0][i]+rotated[i] for i in range(3)], normalized(multiply(a[1],b[1])))


def inverse(a):
    q=[-a[1][0],-a[1][1],-a[1][2],a[1][3]]
    return (rotate(q,[-v for v in a[0]]),q)


def transform(raw):
    return (raw[:3],quaternion(raw[3:]))


def xml_pose(element,label):
    if element is None:
        return IDENTITY
    if element.get("degrees") is not None or element.get("rotation_format") is not None:
        raise ValueError("%s使用了本课SDF1.7未采用的角度/旋转格式属性；请填米与弧度的6数RPY"%label)
    raw=finite(element.text,6,label)
    # These are explicit limits of this small installation exercise, rather
    # than claims that larger positions/angles are invalid SDF in general.
    if max(abs(v) for v in raw[:3])>20:
        raise ValueError("%s超出本课20m安装台范围：确认长度单位是米，不要直接填写毫米数"%label)
    if max(abs(v) for v in raw[3:])>8*math.pi:
        raise ValueError("%s角度超出本课±8π弧度范围：确认角度单位，90度应写π/2≈1.570796而不是90"%label)
    return transform(raw)


def graph_configuration():
    path=Path.cwd()/"world.sdf"
    root=ET.parse(str(path)).getroot()
    if root.get("version")!="1.7":
        raise ValueError("本课采用SDF1.7的frame语义；这是描述格式版本，不是Gazebo版本号")
    world=root.find("world")
    if world is None:
        raise ValueError("world.sdf缺少world")
    nodes={"world":(None,IDENTITY,"")}
    attachments={}
    models={m.get("name"):m for m in world.findall("model")}
    if len(models)!=len(world.findall("model")):
        raise ValueError("world中出现重复模型名称")
    def register(name,node,default,scope):
        if not name or name in nodes:
            raise ValueError("重复或空坐标系名称：%s"%name)
        pose=node.find("pose")
        reference=(pose.get("relative_to") or default) if pose is not None else default
        nodes[name]=(reference,xml_pose(pose,name+" pose"),scope)
    for name,model in models.items():
        register(name,model,"world","")
        for link in model.findall("link"):
            register(name+"::"+link.get("name",""),link,name,name)
        for frame in model.findall("frame"):
            target=frame.get("attached_to") or "__model__"
            key=name+"::"+frame.get("name","")
            register(key,frame,target,name)
            attachments[key]=(target,name)
    for frame in world.findall("frame"):
        key=frame.get("name","")
        target=frame.get("attached_to") or "world"
        register(key,frame,target,"")
        attachments[key]=(target,"")
    def reference_key(name,scope):
        if name in ("world","__world__",world.get("name")):
            return "world"
        if name=="__model__" and scope:
            return scope
        if scope and scope+"::"+name in nodes:
            return scope+"::"+name
        if name in nodes:
            return name
        raise ValueError("未知参考系：%s（当前作用域%s）；请检查relative_to/attached_to拼写和作用域"%(name,scope or "world"))
    # The attachment graph is checked separately: an attachment relationship
    # is not the same graph as the numerical pose-expression relationship.
    def attachment_chain(key,stack):
        if key in stack:
            raise ValueError("attached_to构成循环：%s"%" -> ".join(stack+[key]))
        if key in attachments:
            target,scope=attachments[key]
            attachment_chain(reference_key(target,scope),stack+[key])
    for key in attachments:
        attachment_chain(key,[])
    resolved={"world":IDENTITY}
    def resolve(key,stack=None):
        if key in resolved:
            return resolved[key]
        stack=stack or []
        if key in stack:
            raise ValueError("位姿参考图构成循环：%s"%" -> ".join(stack+[key]))
        reference,local,scope=nodes[key]
        parent=reference_key(reference,scope)
        resolved[key]=compose(resolve(parent,stack+[key]),local)
        return resolved[key]
    for key in nodes:
        resolve(key)
    for name in ("rig","sensor_marker","mount_axes"):
        if name not in models:
            raise ValueError("本课请保留%s模型"%name)
        if models[name].findtext("static","false").strip().lower() not in ("true","1"):
            raise ValueError("本课坐标观察使用static模型，请保留%s的static=true"%name)
    if "mount_frame" not in attachments:
        raise ValueError("本课请保留命名frame mount_frame")
    for key in ("rig::body","sensor_marker::body"):
        if key not in nodes:
            raise ValueError("本课请保留link %s"%key)
    print("配置来源：%s；位置容差%.4fm，姿态角距容差%.4frad"%(path,POSITION_TOLERANCE,ANGLE_TOLERANCE),flush=True)
    print("解析流程：按relative_to/默认参考构建数值变换链；单独检查attached_to图；不创建机械约束",flush=True)
    return world.get("name","default"),resolved


def live_model(world,name):
    output=run("gz","model","-w",world,"-m",name,"-p")
    for line in reversed(output.splitlines()):
        if len(line.split())==6:
            try:
                raw=finite(line,6,name+"运行位姿")
                return raw,transform(raw)
            except ValueError:
                pass
    raise RuntimeError("未获取%s的实时model位姿：%r"%(name,output[-400:]))


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


def live_link_relative(world,model,link):
    message=protobuf_message(run("gz","model","-w",world,"-m",model,"-i"))
    wanted=model+"::"+link
    for candidate in message.get("link",[]):
        if field(candidate,"name") in (wanted,link):
            return message_pose(field(candidate,"pose"))
    raise RuntimeError("实时model消息没有找到link %s"%wanted)


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


def move_rig(world,raw):
    args=["gz","model","-w",world,"-m","rig"]
    for key,value in zip(("-x","-y","-z","-R","-P","-Y"),raw):
        args.extend((key,str(value)))
    run(*args)


def verify():
    global DEADLINE
    DEADLINE=time.monotonic()+12
    world,expected=graph_configuration()
    measured={}; rig_raw=None; moved=False; experiment_error=None
    try:
        for model in ("rig","sensor_marker","mount_axes"):
            raw,actual=live_model(world,model)
            compare(model+"的world位姿",actual,expected[model])
            measured[model]=actual
            if model=="rig":
                rig_raw=raw
        for model in ("rig","sensor_marker"):
            actual=live_link_relative(world,model,"body")
            expected_relative=compose(inverse(expected[model]),expected[model+"::body"])
            compare(model+"::body的model-relative位姿",actual,expected_relative)
            compare(model+"::body的组合world位姿",compose(measured[model],actual),expected[model+"::body"])
        shifted=list(rig_raw);shifted[0]+=.35
        moved=True  # Even an uncertain command outcome requires restoration.
        move_rig(world,shifted)
        _,actual_rig=live_model(world,"rig")
        compare("安装架已实际移动0.35m",actual_rig,transform(shifted))
        _,after_marker=live_model(world,"sensor_marker")
        compare("独立标记没有因relative_to/attached_to而被拖动",after_marker,measured["sensor_marker"])
        print("PASS：变换预测来自当前frame图；跨模型初始位姿关系不构成运行期机械连接。验收未改变暂停状态。",flush=True)
    except Exception as exc:
        experiment_error=exc
        raise
    finally:
        if moved:
            DEADLINE=time.monotonic()+3
            try:
                move_rig(world,rig_raw)
                _,restored=live_model(world,"rig")
                compare("恢复安装架原位姿",restored,transform(rig_raw))
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
