#!/usr/bin/env python3
"""Read actual native/stock wrench streams without imposing forces or motion."""
import argparse
from collections import deque
import json
import math
import re
import subprocess
import statistics
import threading
import time
from config import RIGS, edit, prepared

CHANNELS = {
    "native_child": ("/lesson19/native_child", "load_link"),
    "native_parent_to_child": ("/lesson19/native_parent_to_child", "load_link"),
    "native_sensor_x90": ("/lesson19/native_sensor_x90", "sensor_x90"),
    "stock_child": ("/lesson19/stock_wrench", "stock_load_link")}


def vector(value):
    return [value.x, value.y, value.z]


def norm(values):
    return math.sqrt(sum(value*value for value in values))


def expected(config):
    weight = config["mass_kg"]*config["gravity_m_s2"]
    moment = weight*config["arm_m"]
    return {"native_child": {"force": [0, 0, -weight], "torque": [0, moment, 0]},
            "native_parent_to_child": {"force": [0, 0, weight], "torque": [0, -moment, 0]},
            "native_sensor_x90": {"force": [0, -weight, 0], "torque": [0, 0, -moment]},
            "stock_child": {"force": [0, 0, -weight], "torque": [0, moment, 0]}}


def check_row(row, config):
    if row["frame"].lstrip("/") != CHANNELS[row["source"]][1]:
        raise RuntimeError("Actual wrench header frame label is unexpected")
    if any(not math.isfinite(value) for key in ("force", "torque") for value in row[key]):
        raise RuntimeError("Actual wrench contains non-finite values")
    wanted = expected(config)[row["source"]]
    for key, tolerance in (("force", .05), ("torque", .03)):
        if max(abs(a-b) for a, b in zip(row[key], wanted[key])) > tolerance:
            raise RuntimeError("Actual "+row["source"]+" "+key+" differs from signed anchored-load prediction: actual="+str(row[key])+", expected="+str(wanted[key]))


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


def first(message, name):
    if name not in message:
        raise RuntimeError("运行消息缺少字段：" + name)
    return message[name][0]



class Client:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import WrenchStamped
        self.rospy = rospy
        rospy.init_node("gazeboteach_wrench_lab", anonymous=True, disable_signals=True)
        self.rows, self.lock = deque(maxlen=6000), threading.Lock()
        self.subscriptions = [rospy.Subscriber(topic, WrenchStamped, lambda message, source=source: self.receive(source, message), queue_size=300)
                              for source, (topic, frame) in CHANNELS.items()]

    def receive(self, source, message):
        row = {"source": source, "stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id,
               "force": vector(message.wrench.force), "torque": vector(message.wrench.torque)}
        with self.lock:
            self.rows.append(row)

    def drain(self):
        with self.lock:
            rows = list(self.rows)
            self.rows.clear()
        return rows

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()

    def call(self, name, service_type, *args, timeout=4):
        self.rospy.wait_for_service(name, timeout=timeout)
        done, result = threading.Event(), []
        def worker():
            try:
                proxy = self.rospy.ServiceProxy(name, service_type)
                try:
                    result.append((True, proxy(*args)))
                finally:
                    proxy.close()
            except Exception as error:
                result.append((False, error))
            finally:
                done.set()
        threading.Thread(target=worker, daemon=True).start()
        if not done.wait(timeout):
            raise RuntimeError("ROS service wall deadline; control not resent: "+name)
        success, value = result[0]
        if not success:
            raise value
        if hasattr(value, "success") and not value.success:
            raise RuntimeError(name+": "+value.status_message)
        return value

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, paused):
        from std_srvs.srv import Empty
        if self.physics().pause != paused:
            self.call("/gazebo/"+("pause_physics" if paused else "unpause_physics"), Empty)
        if self.physics().pause != paused:
            raise RuntimeError("Actual pause state did not match requested value")

    def sim_time_ns(self):
        from gazebo_msgs.srv import GetWorldProperties
        value = self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time
        if not math.isfinite(value):
            raise RuntimeError("Actual world simulation time is not finite")
        return round(value*1e9)

    def properties(self, rig="native"):
        from gazebo_msgs.srv import GetLinkProperties
        return self.call("/gazebo/get_link_properties", GetLinkProperties, RIGS[rig]["model"]+"::"+RIGS[rig]["link"])

    def truth(self, rig="native"):
        from gazebo_msgs.srv import GetLinkState
        before = self.sim_time_ns()
        response = self.call("/gazebo/get_link_state", GetLinkState, RIGS[rig]["model"]+"::"+RIGS[rig]["link"], "world")
        after = self.sim_time_ns()
        value, q = response.link_state, response.link_state.pose.orientation
        return {"clock_before_ns": before, "clock_after_ns": after,
                "position": vector(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
                "linear": vector(value.twist.linear), "angular": vector(value.twist.angular)}
    def geometry(self, rig, config):
        identity = RIGS[rig]
        command = ['gz', 'model', '-w', 'lesson19', '-m', identity['model'], '-i']
        deadline = time.monotonic()+8; attempts = []; before = self.sim_time_ns()
        for attempt in range(2):
            remaining = deadline-time.monotonic()
            if remaining <= .1:raise RuntimeError('Read-only actual model geometry deadline expired: '+rig)
            try:
                reply = subprocess.run(command, text=True, capture_output=True, timeout=min(4, remaining))
            except subprocess.TimeoutExpired:
                attempts.append({'attempt': attempt+1, 'timeout': True})
                if attempt:raise RuntimeError('Actual model geometry read timed out twice: '+rig)
                continue
            if reply.returncode:raise RuntimeError('Actual model geometry query failed: '+rig+' '+reply.stderr[-1200:])
            attempts.append({'attempt': attempt+1, 'returncode': reply.returncode})
            message = protobuf_message(reply.stdout)
            def named(values, name):
                found = [v for v in values if first(v, 'name').rsplit('::', 1)[-1] == name]
                if len(found) != 1:raise RuntimeError('Actual entity_info identity/count differs: '+name)
                return found[0]
            if first(message, 'name').rsplit('::', 1)[-1] != identity['model']:
                raise RuntimeError('Actual model response belongs to another rig')
            link = named(message.get('link', []), identity['link'])
            joint = named(message.get('joint', []), identity['joint'])
            if first(joint, 'type') not in ('FIXED', '8') or first(joint, 'parent') != 'world' or first(joint, 'child').rsplit('::', 1)[-1] != identity['link']:
                raise RuntimeError('Actual rig joint is not the expected independent world-fixed child')
            anchor = parsed_pose(first(joint, 'pose'))
            if norm(anchor['position']) > 1e-9 or norm(anchor['orientation'][:3]) > 1e-9 or abs(abs(anchor['orientation'][3])-1) > 1e-9:
                raise RuntimeError('Actual joint anchor/axes differ from child origin: '+rig)
            expected_sensors = 3 if rig == 'native' else 0
            if len(joint.get('sensor', [])) != expected_sensors:
                raise RuntimeError('Actual stock/native sensor topology no longer isolates joint readers')
            payload = named(link.get('collision', []), 'payload_collision')
            geometry = first(payload, 'geometry');size = parsed_vector(first(first(geometry, 'box'), 'size'))
            pose = parsed_pose(first(payload, 'pose'))
            if norm([v-.2 for v in size]) > 1e-9 or norm([a-b for a,b in zip(pose['position'], (config['arm_m'],0,0))]) > 1e-9 or norm(pose['orientation'][:3]) > 1e-9 or abs(abs(pose['orientation'][3])-1)>1e-9:
                raise RuntimeError('Actual payload collision size/CoM alignment differs: '+rig)
            return {'model': first(message, 'name'), 'link': first(link, 'name'), 'joint': first(joint, 'name'),
                    'anchor_child_pose': anchor, 'payload_collision_pose': pose, 'payload_box_size_m': size,
                    'native_sensor_count': len(joint.get('sensor', [])), 'clock_before_ns': before,
                    'clock_after_ns': self.sim_time_ns(), 'read_attempts': attempts, 'raw_model': reply.stdout,
                    'note': 'Actual entity_info and ROS link-state reads are separate observations, not an atomic snapshot.'}
        raise RuntimeError('No actual model geometry response: '+rig)


def parsed_vector(message):
    value = [float(first(message, axis)) for axis in ('x','y','z')]
    if not all(math.isfinite(v) for v in value):raise RuntimeError('Actual native entity vector is nonfinite')
    return value


def parsed_pose(message):
    position = parsed_vector(first(message, 'position'))
    orientation = [float(first(first(message, 'orientation'), axis)) for axis in ('x','y','z','w')]
    if not all(math.isfinite(v) for v in orientation) or abs(norm(orientation)-1)>1e-6:
        raise RuntimeError('Actual native entity quaternion is invalid')
    return {'position': position, 'orientation': orientation}


def check_properties(properties, config):
    q = properties.com.orientation
    if not properties.gravity_mode or abs(properties.mass-config["mass_kg"]) > 1e-9:
        raise RuntimeError("Actual load mass/gravity differs from prepared fixture")
    if norm([properties.com.position.x-config["arm_m"], properties.com.position.y, properties.com.position.z]) > 1e-9 or norm([q.x, q.y, q.z]) > 1e-9 or abs(abs(q.w)-1) > 1e-9:
        raise RuntimeError("Actual center of mass differs from prepared lever arm")
    inertia = config["mass_kg"]*.2**2/6
    if any(abs(getattr(properties, key)-inertia) > 1e-10 for key in ("ixx", "iyy", "izz")):
        raise RuntimeError("Actual payload inertia is not the uniform cube inertia about its CoM")
    if any(abs(getattr(properties, key)) > 1e-10 for key in ("ixy", "ixz", "iyz")):
        raise RuntimeError("Actual payload inertia off-diagonal is not zero")
    return {"mass_kg": properties.mass, "gravity_mode": properties.gravity_mode,
            "com_position": vector(properties.com.position), "com_orientation": [q.x,q.y,q.z,q.w],
            "inertia_diagonal": [properties.ixx, properties.iyy, properties.izz],
            "inertia_offdiagonal": [properties.ixy, properties.ixz, properties.iyz]}


def check_truth(truth, config, rig="native"):
    if norm([a-b for a, b in zip(truth["position"], config["rigs"][rig]["anchor_world_m"])]) > .005:
        raise RuntimeError("Load link origin is not held near actual fixed-joint anchor")
    if norm(truth["linear"]) > .01 or norm(truth["angular"]) > .01 or norm(truth["orientation"][:3]) > .005 or abs(norm(truth["orientation"])-1) > 1e-6:
        raise RuntimeError("Wrench observation is not accompanied by actual static equilibrium")


def ready(config, timeout=12):
    client, found = Client(), {}
    try:
        properties = {name: check_properties(client.properties(name), config) for name in RIGS}
        geometry = {name: client.geometry(name, config) for name in RIGS}
        truth = {name: client.truth(name) for name in RIGS}
        for name, value in truth.items():check_truth(value, config, name)
        deadline, last_error = time.monotonic()+timeout, None
        while time.monotonic() < deadline and len(found) < len(CHANNELS):
            for row in client.drain():
                try:
                    check_row(row, config)
                    found[row["source"]] = row
                except RuntimeError as error:
                    last_error = str(error)
            time.sleep(.02)
        if len(found) != len(CHANNELS):
            raise RuntimeError("Actual four-channel wrench readiness timed out: "+str(last_error))
        return {"ready": True, "channels": found, "rig_properties": properties, "rig_geometry": geometry, "rig_truth": truth}
    finally:
        client.close()


def observe(config, seconds):
    if not math.isfinite(seconds) or not .2 <= seconds <= 30:
        raise ValueError("Observation must bewithin[.2,30]wall seconds")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(.02)
        result = {}
        for source in CHANNELS:
            unique = {row["stamp_ns"]: row for row in rows if row["source"] == source}
            for row in unique.values():
                check_row(row, config)
            result[source] = {"messages": len(unique),
                **{"mean_"+key: [statistics.mean(row[key][axis] for row in unique.values()) for axis in range(3)] if unique else None for key in ("force", "torque")}}
        return result
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    cfg = commands.add_parser("config")
    cfg.add_argument("--mass", type=float)
    cfg.add_argument("--arm", type=float)
    obs = commands.add_parser("observe")
    obs.add_argument("--seconds", type=float, default=3)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.mass is None and args.arm is None:
            parser.error("config requires --mass or --arm")
        result = {"config": edit(args.mass, args.arm), "reload_required": True}
    else:
        config, record = prepared()
        result = ready(config) if args.command == "ready" else observe(config, args.seconds)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
