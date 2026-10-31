#!/usr/bin/env python3
"""Inspect five actual protobuf sensor streams, with honest per-source time bases."""
import argparse
from collections import deque
import copy
import json
import math
from pathlib import Path
import threading
import time
from config import edit, prepared

SOURCES = {
    "gps": ("gps", "gazebo.msgs.GPS"),
    "mag": ("magnetometer", "gazebo.msgs.Magnetometer"),
    "alt": ("altimeter", "gazebo.msgs.Altimeter"),
    "sonar": ("sonar", "gazebo.msgs.SonarStamped"),
    "logical": ("logical_camera", "gazebo.msgs.LogicalCameraImage")}
MODELS = {"nav": "nav_rig", "sonar": "sonar_rig", "obstacle": "sonar_obstacle", "target": "tracked_target"}


class Client:
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        self.rospy = rospy
        rospy.init_node("gazeboteach_sensor_stand", anonymous=True, disable_signals=True)
        self.rows, self.lock, self.error = deque(maxlen=8000), threading.Lock(), None
        self.subscriptions = [rospy.Subscriber("/lesson20/"+source, String,
            lambda message, source=source: self.receive(source, message), queue_size=300) for source in SOURCES]

    def receive(self, source, message):
        try:
            envelope = json.loads(message.data)
            kind, native_type = SOURCES[source]
            if envelope["source_kind"] != kind or envelope["native_type"] != native_type:
                raise ValueError("Unexpected actual native factory/protobuf type")
            stamp = envelope.get("source_stamp_ns")
            stamp = int(stamp) if stamp is not None else None
            receipt = int(envelope["receipt_world_time_ns"])
            sequence = int(envelope["bridge_sequence"])
            if source == "logical":
                if stamp is not None or "time" in envelope["native"]:
                    raise ValueError("LogicalCameraImage has no native time; do not invent a sample stamp")
            else:
                native_time = envelope["native"]["time"]
                if stamp != int(native_time["sec"])*1000000000+int(native_time["nsec"]):
                    raise ValueError("Forwarded native sensor stamp differs from original protobuf time")
            row = {"source": source, "source_stamp_ns": stamp, "receipt_world_time_ns": receipt,
                   "bridge_sequence": sequence, "event_time_ns": stamp if stamp is not None else receipt,
                   "time_basis": "native" if stamp is not None else "bridge_receipt_world",
                   "native_topic": envelope["native_topic"], "native_type": native_type,
                   "native": envelope["native"], "wire_json": message.data}
            with self.lock:
                self.rows.append(row)
        except Exception as error:
            self.error = type(error).__name__+": "+str(error)

    def drain(self):
        if self.error:
            raise RuntimeError("Native sensor forwarding error: "+self.error)
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

    def state(self, station):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import GetModelState
        answer = self.call("/gazebo/get_model_state", GetModelState, MODELS[station], "world")
        value = ModelState()
        value.model_name, value.reference_frame = MODELS[station], "world"
        value.pose, value.twist = copy.deepcopy(answer.pose), copy.deepcopy(answer.twist)
        return value

    def apply(self, value):
        from gazebo_msgs.srv import SetModelState
        self.call("/gazebo/set_model_state", SetModelState, value)

    def place(self, station, x, y, z, yaw_deg=0, vz=0):
        from gazebo_msgs.msg import ModelState
        value = ModelState()
        value.model_name, value.reference_frame = MODELS[station], "world"
        value.pose.position.x, value.pose.position.y, value.pose.position.z = x, y, z
        angle = math.radians(yaw_deg)
        value.pose.orientation.z, value.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        value.twist.linear.z = vz
        self.apply(value)

    def truth(self, station):
        before = self.sim_time_ns()
        value = self.state(station)
        after = self.sim_time_ns()
        return {"station": station, "clock_before_ns": before, "clock_after_ns": after, "state": state_values(value)}


def state_values(value):
    def vec(item):
        return [item.x, item.y, item.z]
    q = value.pose.orientation
    return {"position": vec(value.pose.position), "orientation": [q.x, q.y, q.z, q.w],
            "linear": vec(value.twist.linear), "angular": vec(value.twist.angular)}


def geodetic_from_world(x, y, z):
    """Independent WGS84 inverse for lat/lon0, elevation100, heading180.

    Classic11's legacy LOCAL transform negates horizontal axes at heading0;
    heading180 compensates it, so this fixture's X/Y are East/North.
    """
    a, b = 6378137.0, 6356752.314245
    eccentricity2 = 1-(b*b)/(a*a)
    second_eccentricity2 = a*a/(b*b)-1
    ecef_x, ecef_y, ecef_z = a+100+z, x, y
    p = math.hypot(ecef_x, ecef_y)
    theta = math.atan2(ecef_z*a, p*b)
    latitude = math.atan2(ecef_z+second_eccentricity2*b*math.sin(theta)**3,
                          p-eccentricity2*a*math.cos(theta)**3)
    longitude = math.atan2(ecef_y, ecef_x)
    curvature = a/math.sqrt(1-eccentricity2*math.sin(latitude)**2)
    elevation = p/math.cos(latitude)-curvature
    return [math.degrees(latitude), math.degrees(longitude), elevation]


def contexts(config):
    height = config["initial_height_m"]
    baseline = {"nav_position_m": [0, 0, height], "nav_yaw_deg": 0, "nav_velocity_up_m_s": 0,
                "sonar_height_m": 1.5, "obstacle_present": False, "logical_target_visible": True,
                "logical_target_world_m": [2, -4, 1]}
    result = []
    for name, changes in (
        ("baseline", {}), ("nav_up", {"nav_position_m": [0, 0, height+1]}),
        ("nav_east_north", {"nav_position_m": [10, 5, height]}),
        ("nav_yaw90", {"nav_yaw_deg": 90}), ("nav_velocity", {"nav_velocity_up_m_s": .25}),
        ("sonar_obstacle", {"obstacle_present": True}), ("sonar_out_of_range", {"sonar_height_m": 4}),
        ("logical_outside", {"logical_target_visible": False, "logical_target_world_m": [-2, -4, 1]}),
        ("logical_restored", {})):
        result.append((name, dict(baseline, **changes)))
    return result


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeError("Non-finite actual native value: "+name)
    return value


def vector(value):
    return [finite(value[axis], axis) for axis in ("x", "y", "z")]


def close_values(actual, wanted, tolerance, description):
    errors = [a-b for a, b in zip(actual, wanted)]
    if len(actual) != len(wanted) or max(abs(error) for error in errors) > tolerance:
        raise RuntimeError(description+": actual="+str(actual)+", expected="+str(wanted))
    return errors


def check_reading(row, config, context, epoch):
    source, native = row["source"], row["native"]
    elapsed = (row["event_time_ns"]-epoch)/1e9
    x, y, z = context["nav_position_m"]
    vz = context["nav_velocity_up_m_s"]
    z += vz*elapsed
    if source == "gps":
        wanted = geodetic_from_world(x, y, z)
        actual = [finite(native[key], key) for key in ("latitude_deg", "longitude_deg", "altitude")]
        angular = close_values(actual[:2], wanted[:2], 1e-8, "Actual GPS geodetic angles differ from WGS84 world conversion")
        altitude = close_values(actual[2:], wanted[2:], .002, "Actual GPS ellipsoid height differs from actual pose")
        velocity = [finite(native[key], key) for key in ("velocity_east", "velocity_north", "velocity_up")]
        close_values(velocity, [0, 0, vz], .005, "Actual GPS velocity does not match physical link velocity")
        return {"expected_geodetic": wanted, "angular_error_deg": angular, "ellipsoid_height_error_m": altitude[0],
                "expected_enu_velocity_m_s": [0, 0, vz]}
    if source == "alt":
        reference = finite(native["vertical_reference"], "vertical_reference")
        close_values([reference], [config["initial_height_m"]], 1e-9, "Actual altimeter reference was not retained from sensor loading")
        position = finite(native["vertical_position"], "vertical_position")
        velocity = finite(native["vertical_velocity"], "vertical_velocity")
        close_values([position], [z-config["initial_height_m"]], .002, "Actual altimeter displacement differs from world-Z minus loaded reference")
        close_values([velocity], [vz], .005, "Actual altimeter vertical velocity differs from physical link motion")
        return {"expected_reference_m": config["initial_height_m"], "expected_vertical_position_m": z-config["initial_height_m"],
                "expected_vertical_velocity_m_s": vz}
    if source == "mag":
        bx, by, bz = config["magnetic_field_tesla"]
        angle = math.radians(context["nav_yaw_deg"])
        wanted = [bx*math.cos(angle)+by*math.sin(angle), -bx*math.sin(angle)+by*math.cos(angle), bz]
        actual = vector(native["field_tesla"])
        error = close_values(actual, wanted, 1e-9, "Actual body-frame magnetometer differs from R^T world field")
        return {"expected_field_tesla": wanted, "error_tesla": error}
    if source == "sonar":
        sonar = native["sonar"]
        if sonar["geometry"] != "cone":
            raise RuntimeError("Actual native sonar is not the configured cone-contact model")
        close_values([finite(sonar[key], key) for key in ("range_min", "range_max", "radius")],
                     [0, 3, .2], 1e-9, "Actual sonar configuration differs from the fixed stand")
        close_values(vector(sonar["world_pose"]["position"]), [-4, 0, context["sonar_height_m"]], .005,
                     "Actual sonar world pose differs from initialized station")
        value = finite(sonar["range"], "sonar.range")
        absent = context["sonar_height_m"] > 3
        wanted = 3 if absent else (.9 if context["obstacle_present"] else 1.5)
        close_values([value], [wanted], 1e-6 if absent else .06,
                     "Actual sonar contact-volume range differs from floor/obstacle/no-contact contrast")
        return {"nominal_geometry_range_m": wanted, "range_error_m": value-wanted,
                "is_range_max": absent, "contact_present": "contact" in sonar,
                "note": "Mesh contact approximation; cached contact field is not a fresh echo at range_max."}
    if source == "logical":
        close_values(vector(native["pose"]["position"]), [0, -4, 1], 1e-6, "Actual logical camera world pose is unexpected")
        models = native.get("model", [])
        def matches(name):
            return [model for model in models if model["name"] == name or model["name"].endswith("::"+name)]
        target, occluder = matches("tracked_target"), matches("occluder")
        if bool(target) != context["logical_target_visible"] or len(occluder) != 1:
            raise RuntimeError("Actual logical-camera AABB membership differs from controlled inside/outside/occluder arrangement")
        close_values(vector(occluder[0]["pose"]["position"]), [1, 0, 0], 1e-5, "Logical occluder pose is not camera-relative")
        if target:
            if len(target) != 1:
                raise RuntimeError("Logical target appears more than once")
            tx, ty, tz = context["logical_target_world_m"]
            close_values(vector(target[0]["pose"]["position"]), [tx, ty+4, tz-1], 1e-5,
                         "Actual logical target pose is not relative to the sensor")
        return {"model_names": [model["name"] for model in models], "target_visible": bool(target),
                "occluder_visible": True, "source_has_native_stamp": False}
    raise RuntimeError("Unknown native source")


def check_nav_truth(truth, context, epoch):
    state = truth["state"]
    elapsed = ((truth["clock_before_ns"]+truth["clock_after_ns"])/2-epoch)/1e9
    expected_position = list(context["nav_position_m"])
    expected_position[2] += context["nav_velocity_up_m_s"]*elapsed
    close_values(state["position"], expected_position, .005, "Actual link trajectory differs from initialized inertial motion")
    close_values(state["linear"], [0, 0, context["nav_velocity_up_m_s"]], .005, "Actual link velocity is not the physical GPS/altimeter reference")
    close_values(state["angular"], [0, 0, 0], .005, "Navigation rig unexpectedly rotates")
    wanted = [0, 0, math.sin(math.radians(context["nav_yaw_deg"])/2), math.cos(math.radians(context["nav_yaw_deg"])/2)]
    q = state["orientation"]
    dot = abs(sum(a*b for a, b in zip(q, wanted)))/math.sqrt(sum(a*a for a in q))
    if 2*math.acos(min(1, max(0, dot))) > .005:
        raise RuntimeError("Actual navigation orientation differs from initialized yaw")
    return {"elapsed_s": elapsed, "expected_position_m": expected_position}


def probe(seconds=3):
    if not math.isfinite(seconds) or not .3 <= seconds <= 30:
        raise ValueError("Probe duration must bewithin[.3,30]wall seconds")
    client, rows = Client(), []
    try:
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(.02)
        latest, counts = {}, {}
        for source in SOURCES:
            unique = {row["bridge_sequence"]: row for row in rows if row["source"] == source}
            counts[source] = len(unique)
            if unique:
                latest[source] = unique[max(unique)]
        return {"diagnostic_only": True, "five_actual_sources_observed": all(counts[source] >= 2 for source in SOURCES),
                "counts": counts, "latest": latest, "trace": rows}
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    cfg = commands.add_parser("config")
    cfg.add_argument("--field-scale", type=int)
    cfg.add_argument("--initial-height", type=int)
    observation = commands.add_parser("probe")
    observation.add_argument("--seconds", type=float, default=3)
    observation.add_argument("--report", type=Path)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    mover = commands.add_parser("move")
    mover.add_argument("--station", choices=tuple(MODELS), required=True)
    mover.add_argument("--x", type=float, required=True)
    mover.add_argument("--y", type=float, required=True)
    mover.add_argument("--z", type=float, required=True)
    mover.add_argument("--yaw", type=float, default=0)
    mover.add_argument("--vz", type=float, default=0)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.field_scale is None and args.initial_height is None:
            parser.error("config requires --field-scale or --initial-height")
        result = {"config": edit(args.field_scale, args.initial_height), "reload_required": True}
    else:
        prepared()
        if args.command == "move":
            values = [args.x, args.y, args.z, args.yaw, args.vz]
            if not all(math.isfinite(value) for value in values) or not -12 <= args.x <= 12 or not -8 <= args.y <= 8 or not .2 <= args.z <= 5 or not -180 <= args.yaw <= 180 or not -.5 <= args.vz <= .5:
                parser.error("Finite pose required: x[-12,12],y[-8,8],z[.2,5]m,yaw[-180,180]deg,vz[-.5,.5]m/s")
            if args.station != "nav" and args.vz != 0:
                parser.error("Only the inertial navigation station supports this velocity exercise")
            client, paused = Client(), None
            try:
                paused = client.physics().pause
                client.pause(True)
                client.place(args.station, args.x, args.y, args.z, args.yaw, args.vz)
                result = {"station": args.station, "actual_state": state_values(client.state(args.station)), "paused": paused,
                          "note": "One actual initialization; the gravity-disabled dynamic body evolves freely after resuming."}
            finally:
                try:
                    if paused is not None:
                        client.pause(paused)
                finally:
                    client.close()
            print(json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)
            return
        result = probe(4 if args.command == "ready" else args.seconds)
        if args.command == "probe" and args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        if not result["five_actual_sources_observed"]:
            print(json.dumps({key: value for key, value in result.items() if key != "trace"}, ensure_ascii=False, allow_nan=False), flush=True)
            raise RuntimeError("One or more native factories/topics did not produce actual messages")
    print(json.dumps({key: value for key, value in result.items() if key != "trace"}, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
