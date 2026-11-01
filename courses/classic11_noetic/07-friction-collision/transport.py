"""Bounded native Classic reads and pose/statistics helpers for this rig."""
import json
import math
import re
import subprocess
import time
import sys
from pathlib import Path


sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from _control_support.models import pose as native_model_pose

DEADLINE = None


def run(*arguments):
    remaining = DEADLINE-time.monotonic()
    if remaining <= .1:
        raise RuntimeError("接触实验达到有界时间上限")
    result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=min(2.5, remaining))
    if result.returncode:
        raise RuntimeError("命令失败：%s\n%s" % (" ".join(arguments), (result.stderr or result.stdout)[-1000:]))
    return result.stdout


def read(*arguments):
    for attempt in range(2):
        try:
            return run(*arguments)
        except subprocess.TimeoutExpired as error:
            if attempt or DEADLINE-time.monotonic() < .3:
                raise RuntimeError("只读查询连续超时，有限重试已用完：" + " ".join(arguments)) from error
            print("RETRY：仅重试一次只读查询：" + " ".join(arguments), flush=True)


def message(text):
    tokens = re.findall(r'"(?:\\.|[^"\\])*"|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_][A-Za-z0-9_]*|[{}:]', text)
    cursor = [0]

    def parse(nested=False):
        result = {}
        while cursor[0] < len(tokens):
            key = tokens[cursor[0]]
            cursor[0] += 1
            if key == "}":
                if not nested:
                    raise RuntimeError("运行消息括号不匹配")
                return result
            if cursor[0] >= len(tokens):
                raise RuntimeError("运行消息缺少字段值")
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


def first(node, key):
    if key not in node:
        raise RuntimeError("运行消息缺少字段：" + key)
    return node[key][0]


def stats(world):
    output = read("gz", "topic", "-e", "/gazebo/%s/world_stats" % world, "-d", "1")
    # A one-second echo may contain multiple complete messages. Each field's
    # last occurrence belongs to its last sample; no physics runs while paused.
    iterations = re.findall(r"\biterations:\s*(\d+)", output)
    paused = re.findall(r"\bpaused:\s*(true|false)", output)
    clocks = re.findall(r"\bsim_time\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}", output)
    if not iterations or not paused or not clocks:
        raise RuntimeError("world_stats缺少iterations/sim_time/paused")
    sec, nsec = clocks[-1]
    return {"iterations": int(iterations[-1]), "sim_time_s": int(sec)+int(nsec)*1e-9,
            "paused": paused[-1] == "true"}


def pause(world, value):
    run("gz", "world", "-w", world, "-p", "1" if value else "0")


def live_pose(world, name):
    raw=native_model_pose(world,name,DEADLINE)
    if raw is not None:return raw
    output = read("gz", "model", "-w", world, "-m", name, "-p")
    for line in reversed(output.splitlines()):
        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            continue
        if len(values) == 6 and all(math.isfinite(value) for value in values):
            return values
    raise RuntimeError("没有实际模型位姿：" + name)


def quaternion(raw):
    r, p, y = [value/2 for value in raw[3:]]
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


def check_message_pose(label, actual, expected):
    xyz = [float(first(first(actual, "position"), key)) for key in ("x", "y", "z")]
    q = [float(first(first(actual, "orientation"), key)) for key in ("x", "y", "z", "w")]
    if not all(math.isfinite(value) for value in xyz+q):
        raise AssertionError(label + "实时pose不是有限值")
    dot = abs(sum(a*b for a, b in zip(q, quaternion(expected))))
    if (math.sqrt(sum((a-b)**2 for a, b in zip(xyz, expected[:3]))) > .004 or
            2*math.acos(max(-1, min(1, dot))) > .01):
        raise AssertionError(label + "实际pose与当前文件不一致；保存后请重载")


def near(actual, expected, tolerance, label):
    if not math.isfinite(actual) or abs(actual-expected) > tolerance:
        raise AssertionError("%s不符：实测%.9g，预测%.9g，容差±%.4g" % (label, actual, expected, tolerance))


