#!/usr/bin/env python3
"""Real ROS contact observations and bounded, single-shot service controls."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import statistics
import threading
import time
from config import edit, prepared

PAIR = {"bumper::plate::bumper_collision", "puck::link::collision"}


def vector(value):
    return [value.x, value.y, value.z]


def length(value):
    return math.sqrt(sum(x*x for x in value))


def contact_row(message):
    states = []
    for item in message.states:
        states.append({"collision1": item.collision1_name, "collision2": item.collision2_name,
                       "positions": [vector(x) for x in item.contact_positions],
                       "normals": [vector(x) for x in item.contact_normals], "depths": list(item.depths),
                       "force": vector(item.total_wrench.force), "torque": vector(item.total_wrench.torque),
                       "point_forces": [vector(x.force) for x in item.wrenches], "info": item.info})
    return {"stamp_ns": message.header.stamp.to_nsec(), "frame": message.header.frame_id, "states": states}


class Client:
    def __init__(self):
        import rospy
        from gazebo_msgs.msg import ContactsState
        self.rospy = rospy
        rospy.init_node("gazeboteach_contact_lab", anonymous=True, disable_signals=True)
        self.rows = deque(maxlen=4000)
        self.lock = threading.Lock()
        self.subscription = rospy.Subscriber("/lesson14/contacts", ContactsState, self.receive, queue_size=500)

    def receive(self, message):
        with self.lock:
            self.rows.append(contact_row(message))

    def drain(self):
        with self.lock:
            result = list(self.rows)
            self.rows.clear()
        return result

    def close(self):
        self.subscription.unregister()

    def wait_message(self, timeout=6):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rows = self.drain()
            if rows:
                return rows[-1]
            time.sleep(0.02)
        raise RuntimeError("No actual /lesson14/contacts message within wall-clock deadline")

    def call(self, name, service_type, *args, timeout=4):
        """The TCPROS request is made once, in a daemon with a finite wall limit."""
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
            raise RuntimeError("ROS service wall deadline; control is not resent: "+name)
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
            raise RuntimeError("Actual pause state differs from requested state")

    def state(self):
        from gazebo_msgs.srv import GetModelState
        return self.call("/gazebo/get_model_state", GetModelState, "puck", "world")

    def mass(self):
        from gazebo_msgs.srv import GetLinkProperties
        return self.call("/gazebo/get_link_properties", GetLinkProperties, "puck::link").mass

    def release(self, height, x=0):
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import SetModelState
        value = ModelState()
        value.model_name, value.reference_frame = "puck", "world"
        value.pose.position.x, value.pose.position.z = x, height
        value.pose.orientation.w = 1
        self.call("/gazebo/set_model_state", SetModelState, value)


def check_rows(rows):
    """A state sums spatial contact points; states can repeat at different times."""
    forces = []
    for row in rows:
        if row["frame"] not in ("world", "/world"):
            raise RuntimeError("Unexpected frame label: "+row["frame"])
        for state in row["states"]:
            if {state["collision1"], state["collision2"]} != PAIR:
                raise RuntimeError("Unexpected bumper collision pair")
            count = len(state["positions"])
            if not count or any(len(state[key]) != count for key in ("normals", "depths", "point_forces")):
                raise RuntimeError("Inconsistent contact positions/normals/depths/wrenches")
            values = state["force"]+state["torque"]+state["depths"]
            values += [v for key in ("positions", "normals", "point_forces") for xyz in state[key] for v in xyz]
            if not all(math.isfinite(x) for x in values):
                raise RuntimeError("Non-finite actual contact data")
            summed = [sum(force[axis] for force in state["point_forces"]) for axis in range(3)]
            if length([summed[i]-state["force"][i] for i in range(3)]) > 1e-7*max(1, length(summed)):
                raise RuntimeError("total_wrench does not match pointwise vector sum")
            if any(abs(point[2]-0.2) > 0.02 for point in state["positions"]):
                raise RuntimeError("Contact positions are not near actual plate top")
            forces.append(length(state["force"]))
    return forces


def state_values(value):
    q = value.pose.orientation
    return {"position": vector(value.pose.position), "speed": length(vector(value.twist.linear)),
            "angular_speed": length(vector(value.twist.angular)),
            "upright_error": length([q.x, q.y, q.z])}


def observe(seconds):
    if not math.isfinite(seconds) or not 0.2 <= seconds <= 30:
        raise ValueError("Observation wall time must be within [0.2,30] seconds")
    prepared()
    client = Client()
    try:
        deadline, rows = time.monotonic()+seconds, []
        while time.monotonic() < deadline:
            rows.extend(client.drain())
            time.sleep(0.02)
        unique = {row["stamp_ns"]: row for row in rows}
        forces = check_rows(unique.values())
        return {"messages": len(unique), "empty_messages": sum(not x["states"] for x in unique.values()),
                "contact_states": len(forces), "force_state_norm_mean_n": statistics.mean(forces) if forces else None,
                "note": "No message differs from empty states. Wrench is in collision1 link frame; no frame transform is performed."}
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    config_parser = commands.add_parser("config")
    config_parser.add_argument("--mass", type=float)
    config_parser.add_argument("--height", type=float)
    observer = commands.add_parser("observe")
    observer.add_argument("--seconds", type=float, default=3)
    commands.add_parser("ready")
    args = parser.parse_args()
    if args.command == "config":
        if args.mass is None and args.height is None:
            parser.error("config requires --mass or --height")
        result = {"config": edit(args.mass, args.height), "reload_required": True}
    elif args.command == "ready":
        config, record = prepared()
        client = Client()
        try:
            if not math.isclose(client.mass(), config["mass_kg"], abs_tol=1e-9):
                raise RuntimeError("Actual body mass differs from prepared world; reload")
            row = client.wait_message()
            check_rows([row])
            result = {"ready": True, "message": row, "world_sha256": record["world_sha256"]}
        finally:
            client.close()
    else:
        result = observe(args.seconds)
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("FAIL: "+str(error))
