"""Bounded, observed Gazebo Sim experiments; no Classic message interfaces."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


FORTRESS = os.environ.get('GAZEBOTEACH_PROFILE') == 'fortress-humble'
SIM_CLI = 'ign' if FORTRESS else 'gz'
SYSTEM_NAMESPACE = 'ignition::gazebo::systems::' if FORTRESS else 'gz::sim::systems::'


def native_message_type(name):
    if FORTRESS and name.startswith('gz.msgs.'):
        return 'ignition.msgs.' + name[len('gz.msgs.'):]
    return name


def run(arguments, timeout=8, check=True):
    process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, start_new_session=True)
    try:
        out, err = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()
        raise RuntimeError('命令超时：' + repr(arguments))
    if check and process.returncode:
        raise RuntimeError('命令失败：' + repr(arguments) + '\n' + err[-2000:])
    return out


def topic(name, timeout=8):
    text = run([SIM_CLI, 'topic', '-e', '-t', name, '-n', '1', '--json-output'], timeout)
    try:
        return json.JSONDecoder().raw_decode(text[text.index('{'):])[0]
    except (ValueError, json.JSONDecodeError) as error:
        raise RuntimeError('没有有效 JSON 主题消息：' + name + '\n' + text[-1000:]) from error


def pose(name):
    message = topic('/world/default/pose/info')
    for item in message.get('pose', []):
        if item.get('name') == name:
            return item
    raise RuntimeError('动态位姿消息没有模型：' + name)


def xyz(name):
    value = pose(name).get('position', {})
    return [float(value.get(axis, 0)) for axis in 'xyz']


def service(name, request_type, request, reply_type='gz.msgs.Boolean'):
    text = run([SIM_CLI, 'service', '-s', name, '--reqtype', native_message_type(request_type),
                '--reptype', native_message_type(reply_type),
                '--timeout', '3000', '--req', request], timeout=6)
    if reply_type == 'gz.msgs.Boolean' and 'data: true' not in text:
        raise RuntimeError('服务未确认接收：' + name + '\n' + text[-1200:])
    return text


def publish(name, kind, value):
    run([SIM_CLI, 'topic', '-t', name, '-m', native_message_type(kind), '-p', value])


def move_to(name, position):
    value = 'name: "' + name + '", position: {x: %f, y: %f, z: %f}, orientation: {w: 1}' % tuple(position)
    service('/world/default/set_pose', 'gz.msgs.Pose', value)


def yaw(pose_message):
    q = pose_message.get('orientation', {})
    x, y, z, w = [float(q.get(axis, 1 if axis == 'w' else 0)) for axis in ('x', 'y', 'z', 'w')]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wait_for(predicate, timeout=10):
    deadline, last = time.monotonic()+timeout, None
    while time.monotonic() < deadline:
        try:
            value = predicate()
            if value:
                return value
        except RuntimeError as error:
            last = error
        time.sleep(.1)
    raise RuntimeError('条件等待超时：' + str(last))


def result(values):
    print(json.dumps(values, ensure_ascii=False, indent=2), flush=True)


def ensure_running():
    service('/world/default/control', 'gz.msgs.WorldControl', 'pause: false')
