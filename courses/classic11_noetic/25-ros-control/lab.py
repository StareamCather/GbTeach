#!/usr/bin/env python3
"""Actual ros_control graph, service decisions and physical joint observations."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import threading
import time
from config import JOINT, MODEL, NAMESPACE, SERVOS, prepared


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


class Lab:
    def __init__(self):
        self.config, self.record = prepared()
        import rospy
        from std_msgs.msg import Float64, String
        from sensor_msgs.msg import JointState
        from control_msgs.msg import JointControllerState
        self.rospy = rospy
        if not rospy.core.is_initialized():
            rospy.init_node("lesson25_inspector", anonymous=True, disable_signals=True)
        self.events, self.latest, self.lock = [], {}, threading.Lock()
        self.publishers = {name: rospy.Publisher(NAMESPACE+"/"+name+"/command", Float64, queue_size=1)
                           for name in SERVOS}
        self.subscriptions = [rospy.Subscriber(NAMESPACE+"/joint_states", JointState,
                              lambda message: self.receive("joint_states", message), queue_size=500)]
        self.subscriptions += [rospy.Subscriber(NAMESPACE+"/"+name+"/state", JointControllerState,
                               lambda message, source=name: self.receive(source, message), queue_size=500)
                               for name in SERVOS]
        self.subscriptions.append(rospy.Subscriber(NAMESPACE+"/configuration", String,
                                                   lambda message: self.receive("configuration", message), queue_size=1))

    def receive(self, source, message):
        try:
            if source == "configuration":
                value = json.loads(message.data)
            elif source == "joint_states":
                value = {"stamp_ns": message.header.stamp.to_nsec(), "name": list(message.name),
                         "position": list(message.position), "velocity": list(message.velocity), "effort": list(message.effort)}
            else:
                value = {key: getattr(message, key) for key in
                         ("set_point", "process_value", "process_value_dot", "error", "time_step", "command", "p", "i", "d", "i_clamp")}
                value["stamp_ns"] = message.header.stamp.to_nsec()
            with self.lock:
                event = {"id": len(self.events), "source": source, "receive_monotonic_ns": time.monotonic_ns(),
                         "ros_time_at_receive_ns": self.rospy.Time.now().to_nsec(), "data": value}
                self.events.append(event)
                self.latest[source] = event
        except Exception as error:
            with self.lock:
                self.events.append({"source": "decode_error", "error": str(error)})

    def call(self, service_path, service, **kwargs):
        self.rospy.wait_for_service(service_path, timeout=4)
        completed, result = threading.Event(), {}
        def invoke():
            proxy = self.rospy.ServiceProxy(service_path, service)
            try:
                result["value"] = proxy(**kwargs)
            except Exception as error:
                result["error"] = str(error)
            finally:
                proxy.close()
                completed.set()
        threading.Thread(target=invoke, daemon=True).start()
        require(completed.wait(5), "Finite ROS service deadline: "+service_path)
        require("error" not in result, result.get("error", ""))
        response = result["value"]
        require(not hasattr(response, "success") or response.success,
                service_path+": "+getattr(response, "status_message", "refused"))
        return response

    def physics(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return self.call("/gazebo/get_physics_properties", GetPhysicsProperties)

    def pause(self, paused):
        from std_srvs.srv import Empty
        if self.physics().pause != bool(paused):
            self.call("/gazebo/pause_physics" if paused else "/gazebo/unpause_physics", Empty)
        require(self.physics().pause == bool(paused), "Actual pause state did not change")

    def world_time(self):
        from gazebo_msgs.srv import GetWorldProperties
        return self.call("/gazebo/get_world_properties", GetWorldProperties).sim_time

    def joint(self):
        from gazebo_msgs.srv import GetJointProperties
        before, host = self.world_time(), time.monotonic_ns()
        value = self.call("/gazebo/get_joint_properties", GetJointProperties, joint_name=MODEL+"::"+JOINT)
        return {"world_time_before": before, "world_time_after": self.world_time(),
                "receive_monotonic_ns": host, "position": list(value.position), "rate": list(value.rate)}

    def controllers(self):
        from controller_manager_msgs.srv import ListControllers
        value = self.call(NAMESPACE+"/controller_manager/list_controllers", ListControllers)
        return [{"name": item.name, "state": item.state, "type": item.type,
                 "claimed_resources": [{"hardware_interface": resource.hardware_interface,
                                        "resources": list(resource.resources)} for resource in item.claimed_resources]}
                for item in value.controller]

    def active(self):
        return [item["name"] for item in self.controllers() if item["name"] in SERVOS and item["state"] == "running"]

    def switch_raw(self, start, stop):
        from controller_manager_msgs.srv import SwitchController
        paused = self.physics().pause
        self.pause(False)
        try:
            response = self.call(NAMESPACE+"/controller_manager/switch_controller", SwitchController,
                                 start_controllers=list(start), stop_controllers=list(stop), strictness=2,
                             start_asap=False, timeout=2.0)
            return {"ok": bool(response.ok), "start": list(start), "stop": list(stop),
                    "strictness": 2, "controllers_after": self.controllers()}
        finally:
            self.pause(paused)

    def switch(self, name):
        require(name in SERVOS, "Choose position_servo or soft_servo")
        before = self.active()
        start = [] if name in before else [name]
        stop = [value for value in before if value != name]
        result = self.switch_raw(start, stop) if start or stop else {"ok": True, "controllers_after": self.controllers()}
        require(result["ok"] and self.active() == [name], "Controller switch refused")
        return result

    def command(self, name, target):
        from std_msgs.msg import Float64
        require(name in SERVOS and math.isfinite(target) and abs(target) <= 1.5, "Finite command within +/-1.5rad required")
        deadline = time.monotonic()+4
        while self.publishers[name].get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(.02)
        require(self.publishers[name].get_num_connections() > 0, "Controller command subscriber unavailable")
        self.publishers[name].publish(Float64(data=target))
        return {"controller": name, "requested_target_rad": target, "host_monotonic_ns": time.monotonic_ns()}

    def ready(self):
        self.config, self.record = prepared()
        paused = self.physics().pause
        deadline = time.monotonic()+25
        while time.monotonic() < deadline:
            try:
                listed = self.controllers()
                names = {item["name"] for item in listed}
                active = self.active()
                with self.lock:
                    status = self.latest.get("configuration")
                    joints = self.latest.get("joint_states")
                    controller = self.latest.get(active[0]) if active else None
                if names == {"joint_state", *SERVOS} and status and (paused or (joints and controller)):
                    break
            except RuntimeError:
                pass
            time.sleep(.05)
        else:
            raise RuntimeError("Actual controllers/latched configuration/joint feedback not ready")
        require(status["data"]["settings_sha256"] == self.record["settings_sha256"], "Loaded configuration is stale; reload/restart required")
        require(status["data"]["vendor_library_sha256"] == self.record["vendor_library_sha256"], "Loaded official controller library differs")
        if joints:
            require(joints["data"]["name"] == [JOINT], "Unexpected actual JointState names")
        require(active and len(active) == 1, "Exactly one effort position controller must be running")
        for item in listed:
            if item["name"] in SERVOS:
                require(item["type"] == "effort_controllers/JointPositionController", "Unexpected servo implementation")
        for name, gains in (("position_servo", (self.config["p"], self.config["d"])), ("soft_servo", (1., .35))):
            for key, expected in zip(("p", "d"), gains):
                require(abs(float(self.rospy.get_param(NAMESPACE+"/"+name+"/pid/"+key))-expected) < 1e-10,
                        "Runtime PID parameter drift; reload/restart required")
        if controller:
            expected = (self.config["p"], self.config["d"]) if active[0] == "position_servo" else (1., .35)
            require(abs(controller["data"]["p"]-expected[0]) < 1e-10 and abs(controller["data"]["d"]-expected[1]) < 1e-10,
                    "Actual controller's cached gains differ; reload/restart required")
        return {"ready": True, "config": self.config, "controllers": listed, "configuration": status,
                "joint_state": joints, "controller_state": controller, "gazebo_joint": self.joint(),
                "dynamic_observation_deferred": bool(paused and (not joints or not controller)),
                "note": "Paused cold clients do not advance physics for readiness. Dynamic gains/feedback are checked during actual experiment advancement."}

    def link(self, name):
        from gazebo_msgs.srv import GetLinkState
        response = self.call("/gazebo/get_link_state", GetLinkState, link_name=MODEL+"::"+name, reference_frame="world")
        p, q, v, w = response.link_state.pose.position, response.link_state.pose.orientation, response.link_state.twist.linear, response.link_state.twist.angular
        return {"position": [p.x,p.y,p.z], "orientation": [q.x,q.y,q.z,q.w], "linear": [v.x,v.y,v.z], "angular": [w.x,w.y,w.z]}

    def set_link(self, name, state):
        from gazebo_msgs.msg import LinkState
        from gazebo_msgs.srv import GetLinkProperties, SetLinkState
        properties = self.call("/gazebo/get_link_properties", GetLinkProperties, link_name=MODEL+"::"+name)
        c = properties.com.position
        x,y,z,s = state["orientation"]
        offset = [(1-2*(y*y+z*z))*c.x+2*(x*y-z*s)*c.y+2*(x*z+y*s)*c.z,
                  2*(x*y+z*s)*c.x+(1-2*(x*x+z*z))*c.y+2*(y*z-x*s)*c.z,
                  2*(x*z-y*s)*c.x+2*(y*z+x*s)*c.y+(1-2*(x*x+y*y))*c.z]
        w = state["angular"]
        correction = [w[1]*offset[2]-w[2]*offset[1],w[2]*offset[0]-w[0]*offset[2],w[0]*offset[1]-w[1]*offset[0]]
        value = LinkState(link_name=MODEL+"::"+name, reference_frame="world")
        value.pose.position.x,value.pose.position.y,value.pose.position.z=state["position"]
        value.pose.orientation.x,value.pose.orientation.y,value.pose.orientation.z,value.pose.orientation.w=state["orientation"]
        value.twist.linear.x,value.twist.linear.y,value.twist.linear.z=[a+b for a,b in zip(state["linear"],correction)]
        value.twist.angular.x,value.twist.angular.y,value.twist.angular.z=w
        self.call("/gazebo/set_link_state", SetLinkState, link_state=value)

    def set_q(self, angle):
        from gazebo_msgs.srv import SetModelConfiguration
        require(self.physics().pause, "State initialization requires a paused boundary")
        self.call("/gazebo/set_model_configuration", SetModelConfiguration, model_name=MODEL,
                  urdf_param_name=NAMESPACE+"/robot_description", joint_names=[JOINT], joint_positions=[angle])

    def snapshot(self):
        paused = self.physics().pause
        self.pause(True)
        try:
            saved = {"paused": paused, "world_time": self.world_time(), "joint": self.joint(),
                     "links": {name: self.link(name) for name in ("base_link", "rotor")},
                     "active": self.active(), "controller_states": self.controllers()}
            with self.lock:
                needs_warm = paused or any(name not in self.latest for name in saved["active"])
                mark = len(self.events)
            if needs_warm and saved["active"]:
                # A cold paused subscriber cannot infer the controller's held
                # target. Capture physical state first, observe real updates,
                # then restore that state; report the non-rewound time/history.
                saved["target_observation_interval"] = self.advance(.08)
                self.pause(True)
                with self.lock:
                    require(all(name in self.latest and self.latest[name]["id"] >= mark for name in saved["active"]),
                            "Fresh held-controller target was not observed")
                    saved["target_observation_events"] = [self.latest[name] for name in saved["active"]]
                self.set_q(saved["joint"]["position"][0])
                for name, state in saved["links"].items():
                    self.set_link(name, state)
            with self.lock:
                saved["commands"] = {name: self.latest[name]["data"]["set_point"] for name in saved["active"]}
            return saved
        finally:
            self.pause(paused)

    def initialize(self, angle=0.):
        self.switch_raw([], self.active())
        self.pause(True)
        self.set_q(angle)
        for name in ("base_link", "rotor"):
            state = self.link(name)
            state.update(linear=[0.,0.,0.], angular=[0.,0.,0.])
            self.set_link(name, state)

    def restore(self, saved):
        self.switch_raw([], self.active())
        require(not self.active(), "Servos must stop before restoring inactive lifecycle state")
        # Starting an unused controller changes initialized -> running -> stopped.
        # Recreate only those originally initialized inactive controllers through
        # the real manager, preserving the enum rather than relabelling evidence.
        from controller_manager_msgs.srv import LoadController, UnloadController
        reinitialized = []
        self.pause(False)
        for original in saved["controller_states"]:
            name = original["name"]
            current = {item["name"]: item for item in self.controllers()}
            if name in SERVOS and original["state"] == "initialized" and current[name]["state"] != "initialized":
                require(current[name]["state"] != "running", "Cannot unload a running controller")
                unloaded = self.call(NAMESPACE+"/controller_manager/unload_controller", UnloadController, name=name)
                require(unloaded.ok and name not in {item["name"] for item in self.controllers()},
                        "Inactive controller did not unload")
                loaded = self.call(NAMESPACE+"/controller_manager/load_controller", LoadController, name=name)
                current = {item["name"]: item for item in self.controllers()}
                require(loaded.ok and current.get(name, {}).get("state") == "initialized",
                        "Inactive controller did not return to initialized")
                reinitialized.append(name)
        self.pause(True)
        self.set_q(saved["joint"]["position"][0])
        for name,state in saved["links"].items():
            self.set_link(name,state)
        if saved["active"]:
            require(self.switch_raw(saved["active"], [])["ok"], "Restoring original controller failed")
            for name in saved["active"]:
                self.command(name,saved["commands"].get(name,saved["joint"]["position"][0]))
            self.advance(.03)
        self.pause(True)
        # Controller startup resets its internal target/PID. After settling the
        # command delivery, restore the physical snapshot again at a boundary.
        self.set_q(saved["joint"]["position"][0])
        for name,state in saved["links"].items():
            self.set_link(name,state)
        readback = {"joint": self.joint(), "links": {name:self.link(name) for name in saved["links"]}}
        controllers_after = self.controllers()
        require({item["name"]: item["state"] for item in controllers_after} ==
                {item["name"]: item["state"] for item in saved["controller_states"]},
                "Controller lifecycle state was not restored")
        self.pause(saved["paused"])
        return {"paused_readback": readback, "active": self.active(), "pause_restored": self.physics().pause,
                "controllers_after": controllers_after, "reinitialized_controllers": reinitialized,
                "note": "Physical pose/twist, controller enums and last observed target restored. Reinitialization recreates inactive controllers; PID/subscriber identities, elapsed time and observer histories are not rewound."}

    def advance(self, seconds):
        require(0 <= seconds <= 8, "Observe 0..8 actual simulation seconds")
        start = self.world_time()
        self.pause(False)
        deadline = time.monotonic()+max(8,seconds*5)
        while self.world_time()-start < seconds:
            require(time.monotonic() < deadline, "Simulation failed to advance before deadline")
            time.sleep(.015)
        return {"start": start, "end": self.world_time()}

    def demo(self, duration=3., target=None, controller="position_servo"):
        self.ready()
        target = self.config["target_rad"] if target is None else float(target)
        saved = self.snapshot()
        result = {"controller": controller, "requested_target_rad": target, "entry": saved}
        try:
            self.initialize(0.)
            self.switch(controller)
            self.pause(True)
            with self.lock:
                mark = len(self.events)
            result["command"] = self.command(controller,target)
            result["interval"] = self.advance(duration)
            self.pause(True)
            result["gazebo_joint"] = self.joint()
            with self.lock:
                result["events"] = self.events[mark:]
            samples = [event["data"] for event in result["events"] if event["source"] == controller]
            joints = [event["data"] for event in result["events"] if event["source"] == "joint_states"]
            require(len(samples) >= 30 and len(joints) >= 30, "Too few real controller/joint messages")
            expected_gains = (self.config["p"], self.config["d"]) if controller == "position_servo" else (1., .35)
            require(all(abs(row["p"]-expected_gains[0]) < 1e-10 and abs(row["d"]-expected_gains[1]) < 1e-10 for row in samples),
                    "Real running controller cached gains differ from prepared settings")
            expected = max(-.8,min(.8,target))
            recent = samples[-10:]
            result["summary"] = {"expected_limited_target_rad": expected, "controller_samples": len(samples),
                                 "joint_samples": len(joints), "final_position_rad": result["gazebo_joint"]["position"][0],
                                 "last_controller_error_rad": recent[-1]["error"],
                                 "tail_mean_abs_error_rad": sum(abs(row["process_value"]-expected) for row in recent)/len(recent),
                                 "max_abs_native_effort_nm": max(abs(row["effort"][0]) for row in joints if row["effort"]),
                                 "peak_abs_raw_pid_effort_nm": max(abs(row["command"]) for row in samples),
                                 "peak_abs_velocity_rad_s": max(abs(row["velocity"][0]) for row in joints if row["velocity"])}
            require(any(abs(row["error"]) > .02 for row in samples), "No nontrivial tracking transient was observed")
            require(abs(result["gazebo_joint"]["position"][0]-expected) < .06, "Actual joint did not approach the clamped setpoint")
            require(max(abs(row["position"][0]) for row in joints) <= .825, "Physical joint exceeded the actual position limits")
            require(result["summary"]["max_abs_native_effort_nm"] <= 2.02, "Native joint effort exceeds declared hardware limit")
            result["passed"] = True
        finally:
            result["restore"] = self.restore(saved)
        return result

    def conflict(self):
        self.ready()
        saved = self.snapshot()
        result = {}
        try:
            require(self.switch_raw([],self.active())["ok"], "Could not stop controllers before conflict")
            result["resource_conflict"] = self.switch_raw(SERVOS,[])
            require(not result["resource_conflict"]["ok"] and not self.active(), "Two controllers incorrectly acquired the same effort joint")
            self.switch("position_servo")
            result["unknown_controller"] = self.switch_raw(["missing_controller"],["position_servo"])
            require(not result["unknown_controller"]["ok"] and self.active() == ["position_servo"], "STRICT rejection changed active control")
            result["passed"] = True
        finally:
            result["restore"] = self.restore(saved)
        return result

    def limits(self):
        return self.demo(duration=3.,target=1.1)

    def step(self, steps):
        require(type(steps) is int and 1 <= steps <= 1000, "Use 1..1000 steps")
        physics = self.physics()
        require(physics.pause, "Pause the actual simulation before stepping")
        before = self.world_time()
        subprocess.run(["gz","world","-w","lesson25","-m",str(steps)],check=True,timeout=6,capture_output=True)
        expected = before + steps * physics.time_step
        deadline = time.monotonic() + 5
        after = self.world_time()
        while after < expected - 1e-7 and time.monotonic() < deadline:
            time.sleep(.02)
            after = self.world_time()
        require(abs(after - expected) < 1e-7 and self.physics().pause,
                "Requested ticks did not complete exactly at a paused boundary")
        return {"before": before, "after": after, "requested_steps": steps,
                "physics_time_step": physics.time_step, "paused": True}

    def verify(self):
        return {"passed": True, "ready": self.ready(), "main_response": self.demo(),
                "soft_response": self.demo(controller="soft_servo"), "lifecycle": self.conflict(), "limits": self.limits()}

    def close(self):
        for subscription in self.subscriptions:
            subscription.unregister()
        for publisher in self.publishers.values():
            publisher.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation",choices=("ready","status","demo","switch","conflict","limits","step","verify"))
    parser.add_argument("--seconds",type=float,default=3.)
    parser.add_argument("--target",type=float)
    parser.add_argument("--name",choices=SERVOS,default="position_servo")
    parser.add_argument("--controller",choices=SERVOS,default="position_servo")
    parser.add_argument("--steps",type=int,default=100)
    args=parser.parse_args()
    lab=Lab()
    try:
        if args.operation in ("ready","status"):result=lab.ready()
        elif args.operation=="demo":result=lab.demo(duration=args.seconds,target=args.target,controller=args.controller)
        elif args.operation=="switch":result=lab.switch(args.name)
        elif args.operation=="step":result=lab.step(args.steps)
        else:result=getattr(lab,args.operation)()
        print(json.dumps(result,ensure_ascii=False,allow_nan=False))
    finally:lab.close()


if __name__=="__main__":main()
