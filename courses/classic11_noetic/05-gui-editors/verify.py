#!/usr/bin/env python3
"""Verify current saved editor boxes, actual loaded geometry and real support.

This verifies resource reload and collision behaviour, not the learner's mouse
actions. Only table_probe is moved; assets and static assemblies are read-only.
"""
import json
import math
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from geometry import (combine, contains_xy, numbers, pose, resource,
                      room_geometry, table_geometry)

DEADLINE = None


def run(*arguments):
    remaining = DEADLINE-time.monotonic()
    if remaining <= .1:
        raise RuntimeError("本课验收达到有界时间上限")
    result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=min(2.4, remaining))
    if result.returncode:
        raise RuntimeError("命令失败：%s\n%s" % (" ".join(arguments), (result.stderr or result.stdout)[-1000:]))
    return result.stdout


def read_model(world, name, flag):
    if flag not in ("-p", "-i"):
        raise ValueError("有限重试只用于只读模型查询")
    for attempt in range(2):
        try:
            return run("gz", "model", "-w", world, "-m", name, flag)
        except subprocess.TimeoutExpired as error:
            if attempt or DEADLINE-time.monotonic() < .3:
                raise RuntimeError("只读模型查询%s %s连续超时；有限重试已用完" % (name, flag)) from error
            print("RETRY：只读查询%s %s首次超时；仅重试一次" % (name, flag), flush=True)


def protobuf_message(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_]*|[{}:]', text)
    cursor = [0]

    def parse(nested=False):
        result = {}
        while cursor[0] < len(tokens):
            key = tokens[cursor[0]]
            cursor[0] += 1
            if key == "}":
                if not nested:
                    raise RuntimeError("运行消息包含多余右括号")
                return result
            if cursor[0] >= len(tokens):
                raise RuntimeError("运行消息字段缺少值")
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


def field(message, name):
    if name not in message:
        raise RuntimeError("运行消息缺少字段：" + name)
    return message[name][0]


def quaternion(raw):
    r, p, y = [value/2 for value in raw[3:]]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def message_pose(message):
    position, orientation = field(message, "position"), field(message, "orientation")
    xyz = [float(field(position, axis)) for axis in ("x", "y", "z")]
    q = [float(field(orientation, axis)) for axis in ("x", "y", "z", "w")]
    if not all(math.isfinite(value) for value in xyz+q) or abs(sum(value*value for value in q)-1) > .002:
        raise ValueError("实时pose含非有限坐标或非单位四元数")
    return xyz, q


def check_pose(label, measured, expected, tolerance=.015):
    xyz, q = measured
    distance = math.sqrt(sum((a-b)**2 for a, b in zip(xyz, expected[:3])))
    dot = abs(sum(a*b for a, b in zip(q, quaternion(expected))))
    angle = 2*math.acos(max(-1, min(1, dot)))
    if distance > tolerance or angle > .015:
        raise AssertionError("%s与当前保存SDF不一致：预测%s，实际位置%s，误差%.6gm/%.6grad；保存后请重载项目" %
                             (label, expected, xyz, distance, angle))


def live_pose(world, name):
    output = read_model(world, name, "-p")
    for line in reversed(output.splitlines()):
        if len(line.split()) == 6:
            try:
                return numbers(line, 6, "实时pose " + name)
            except ValueError:
                pass
    raise RuntimeError("未获取实时模型位姿：" + name)


def check_raw_pose(label, actual, expected):
    check_pose(label, (actual[:3], quaternion(actual)), expected)


def paused_state(world):
    output = run("gz", "topic", "-e", "/gazebo/%s/world_stats" % world, "-d", "1")
    values = re.findall(r"\bpaused:\s*(true|false)\b", output)
    if not values:
        raise RuntimeError("world_stats未返回暂停状态")
    return values[-1] == "true"


def pause(world, value):
    # Mutating requests are never retried implicitly.
    run("gz", "topic", "-p", "/gazebo/%s/world_control" % world,
        "-m", "pause: %s" % ("true" if value else "false"))


def move(world, name, raw):
    arguments = ["gz", "model", "-w", world, "-m", name]
    for key, value in zip(("-x", "-y", "-z", "-R", "-P", "-Y"), raw):
        arguments.extend((key, str(value)))
    run(*arguments)


def configuration():
    workspace = Path.cwd().resolve()
    world = ET.parse(str(workspace / "world.sdf")).getroot().find("world")
    if world is None:
        raise ValueError("当前world.sdf缺少world")
    ground = world.find("model[@name='ground']")
    if (ground is None or ground.findtext("static", "false").strip().lower() not in ("true", "1") or
            ground.find("link/collision/geometry/plane") is None or any(abs(value) > 1e-8 for value in pose(ground, "ground"))):
        raise ValueError("本课保留世界原点处静态ground碰撞平面")
    gravity = numbers(world.findtext("gravity", "0 0 -9.81"), 3, "gravity")
    if abs(gravity[0])+abs(gravity[1]) > 1e-8 or gravity[2] >= 0:
        raise ValueError("本课需要竖直向下重力")
    packages = {}
    for name, analyze in (("table_example", table_geometry), ("room_example", room_geometry)):
        includes = [node for node in world.findall("include") if node.findtext("name") == name]
        if len(includes) != 1:
            raise ValueError("world需要唯一include实例：" + name)
        node = includes[0]
        current = resource(node.findtext("uri", "").strip(), workspace)
        current["placement"] = pose(node, "include " + name)
        if abs(current["placement"][2]+current["min_z"]) > .025:
            raise ValueError("%s最低collision面未对齐地面；运行lab.py use-exports/use-reference后重载" % name)
        current["analysis"] = analyze(current)
        packages[name] = current
        print("资源解析：%s → %s；include pose=%s" % (name, current["entry"], current["placement"]), flush=True)
    probe = world.find("model[@name='table_probe']")
    if probe is None or probe.findtext("static", "false").strip().lower() not in ("false", "0"):
        raise ValueError("本课需要动态table_probe")
    links = probe.findall("link")
    if len(links) != 1:
        raise ValueError("table_probe应有一个link")
    link = links[0]
    if any(abs(value) > 1e-8 for value in pose(link, "probe link")):
        raise ValueError("本课探针link应居中")
    sizes = []
    for kind in ("collision", "visual"):
        shapes = link.findall(kind)
        if len(shapes) != 1 or any(abs(value) > 1e-8 for value in pose(shapes[0], "probe " + kind)):
            raise ValueError("本课探针需要居中box collision/visual")
        sizes.append(numbers(shapes[0].findtext("geometry/box/size"), 3, "probe size"))
    if any(abs(a-b) > 1e-8 for a, b in zip(*sizes)) or min(sizes[0]) <= 0 or max(sizes[0]) > .2:
        raise ValueError("本课探针collision/visual需要相同的小box（各边≤.2m）")
    return world.get("name", "default"), packages, sizes[0]


def short_name(value):
    return str(value).split("::")[-1]


def actual_shapes(link, kind, label):
    """Keep user geometry without flattening Classic's display-node tree.

    A Link message includes a geometry-free visual with type LINK and the
    link's own name/id. Each collision also owns two nested type COLLISION
    visuals, including its orange __COLLISION_VISUAL__ helper. These describe
    display hierarchy, not extra SDF visual elements. Direct user visuals have
    type VISUAL and geometry; unknown or malformed entries still fail closed.
    """
    result = {}
    link_name = field(link, "name")
    collision_names = {field(shape, "name") for shape in link.get("collision", [])}
    for shape in link.get(kind, []):
        full_name = field(shape, "name")
        if kind == "visual":
            visual_type = field(shape, "type")
            if (visual_type == "LINK" and full_name == link_name and "geometry" not in shape and
                    shape.get("id") == link.get("id")):
                continue
            # Usually nested under collision and therefore never reached here;
            # require both enum and exact generated collision-based name if a
            # Classic build emits this helper directly under the link.
            if (visual_type == "COLLISION" and field(shape, "parent_name") == link_name and
                    (full_name in collision_names or
                     full_name in {name+"__COLLISION_VISUAL__" for name in collision_names})):
                continue
            if visual_type != "VISUAL":
                raise AssertionError(label + "出现无法识别的visual消息类型：" + str(visual_type))
        if "geometry" not in shape:
            raise AssertionError(label + "实际用户几何缺少geometry：" + str(full_name))
        name = short_name(full_name)
        if name in result:
            raise AssertionError(label + "实际用户几何名称重复：" + name)
        result[name] = shape
    return result


def live_geometry(world, name, saved):
    message = protobuf_message(read_model(world, name, "-i"))
    static = [message[key][0] for key in ("is_static", "static") if key in message]
    if not static or any(str(value).lower() not in ("true", "1") for value in static):
        raise AssertionError(name + "真实Model消息未确认static=true")
    check_raw_pose(name + "实际include位姿", live_pose(world, name), saved["placement"])
    if "pose" in message:
        check_pose(name + "Model消息位姿", message_pose(field(message, "pose")), saved["placement"])
    actual_links = {short_name(field(link, "name")): link for link in message.get("link", [])}
    if set(actual_links) != set(saved["links"]):
        raise AssertionError(name + "真实link集合与当前保存资源不一致；请重载")
    for link_name, expected_pose in saved["links"].items():
        link = actual_links[link_name]
        check_pose(name + "::" + link_name + " link局部pose", message_pose(field(link, "pose")), expected_pose)
        for kind in ("collision", "visual"):
            expected_shapes = [shape for shape in saved[kind] if shape["link"] == link_name]
            observed_shapes = actual_shapes(link, kind, name + "/" + link_name)
            if set(observed_shapes) != set(shape["name"] for shape in expected_shapes):
                raise AssertionError("%s/%s实际%s集合与保存文件不一致" % (name, link_name, kind))
            for expected in expected_shapes:
                shape = observed_shapes[expected["name"]]
                size = field(field(field(shape, "geometry"), "box"), "size")
                actual = [float(field(size, axis)) for axis in ("x", "y", "z")]
                if not all(math.isfinite(value) for value in actual) or any(abs(a-b) > 1e-5 for a, b in zip(actual, expected["size"])):
                    raise AssertionError("%s/%s/%s真实box尺寸%s与保存值%s不符；请重载" %
                                         (name, link_name, expected["name"], actual, expected["size"]))
                check_pose("%s/%s/%s %s局部pose" % (name, link_name, expected["name"], kind),
                           message_pose(field(shape, "pose")), expected["local"])
    print("PASS：%s真实static、全部link/box collision/visual及组合pose与当前保存SDF一致" % name, flush=True)


def verify():
    global DEADLINE
    DEADLINE = time.monotonic()+41
    world, packages, probe_size = configuration()
    original_paused = paused_state(world)
    original_probe = None
    moved = False
    experiment_error = None
    try:
        pause(world, True)
        if not paused_state(world):
            raise RuntimeError("未确认暂停，不开始探针实验")
        for name, saved in packages.items():
            live_geometry(world, name, saved)
        room = packages["room_example"]["analysis"]
        print("PASS：从当前collision分块独立求得真实门洞宽%.5gm、高%.5gm、墙厚%.5gm；洞内从地面到门楣无collision" %
              (room["width"], room["height"], room["wall_thickness"]), flush=True)
        table = packages["table_example"]
        top = table["analysis"]["top"]
        if not contains_xy(top, top["pose"][0], top["pose"][1], max(probe_size[:2])*.75):
            raise ValueError("探针对于当前桌面过大")
        supported = combine(table["placement"], [top["pose"][0], top["pose"][1],
                                                table["analysis"]["surface_z"]+probe_size[2]/2, 0, 0, 0])
        original_probe = live_pose(world, "table_probe")
        raised = list(supported)
        raised[2] += 1.1
        moved = True
        move(world, "table_probe", raised)
        check_raw_pose("探针暂停抬升", live_pose(world, "table_probe"), raised)
        pause(world, False)
        samples = []
        while DEADLINE-time.monotonic() > .5:
            measured = live_pose(world, "table_probe")
            if (abs(measured[2]-supported[2]) < .025 and
                    math.hypot(measured[0]-supported[0], measured[1]-supported[1]) < .04 and
                    max(abs(measured[3]), abs(measured[4])) < .08):
                samples.append((time.monotonic(), measured[2]))
                samples = samples[-4:]
            else:
                samples = []
            stable = (len(samples) >= 3 and samples[-1][0]-samples[0][0] >= .25 and
                      max(z for _, z in samples)-min(z for _, z in samples) < .006)
            if stable:
                if raised[2]-measured[2] < .4:
                    raise AssertionError("探针没有发生实际下落")
                print("PASS：真实探针下落并稳定由桌面支撑；桌面世界z=%.5gm，预测探针中心z=%.5gm，实际z=%.5gm" %
                      (supported[2]-probe_size[2]/2, supported[2], measured[2]), flush=True)
                print("自动检查通过；Model/Building Editor创建、另存、退出等人工技能仍需正文清单确认。", flush=True)
                return
            time.sleep(.07)
        raise AssertionError("探针未稳定落在当前保存桌面：预测%s，最后测量%s；检查碰撞与重载" % (supported, measured))
    except Exception as error:
        experiment_error = error
        raise
    finally:
        DEADLINE = time.monotonic()+10
        cleanup_error = None
        try:
            if moved and original_probe is not None:
                pause(world, True)
                if not paused_state(world):
                    raise RuntimeError("恢复探针前未确认暂停")
                move(world, "table_probe", original_probe)
                check_raw_pose("恢复探针原位姿", live_pose(world, "table_probe"), original_probe)
        except Exception as error:
            cleanup_error = error
        finally:
            # A failed read/move must not prevent the independent pause restore.
            DEADLINE = max(DEADLINE, time.monotonic()+3.4)
            try:
                pause(world, original_paused)
                if paused_state(world) != original_paused:
                    raise RuntimeError("未确认恢复原暂停状态")
                print("已恢复原暂停状态%s；未修改world、模型文件或静态实例。" % original_paused, flush=True)
            except Exception as error:
                cleanup_error = RuntimeError("%s；暂停恢复失败：%s" % (cleanup_error, error)) if cleanup_error else error
        if cleanup_error is not None:
            if experiment_error is not None:
                print("附加恢复失败：%s；原验收错误保留" % cleanup_error, file=sys.stderr, flush=True)
            else:
                raise cleanup_error


if __name__ == "__main__":
    try:
        verify()
    except Exception as error:
        print("FAIL：" + str(error), file=sys.stderr, flush=True)
        sys.exit(1)
