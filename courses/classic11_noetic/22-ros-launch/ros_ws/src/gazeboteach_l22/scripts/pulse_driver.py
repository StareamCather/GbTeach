#!/usr/bin/env python3
"""An explicit Trigger schedules one finite Gazebo wrench job; startup is idle."""
import math
import threading
import time
import uuid
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, BodyRequest, GetWorldProperties
from geometry_msgs.msg import Point, Wrench
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from common import call, identity, publish


class Driver:
    def __init__(self):
        self.lock, self.busy_until, self.sequence = threading.Lock(), 0, 0
        self.info = identity()
        self.force = float(rospy.get_param("~force_n"))
        self.duration = float(rospy.get_param("~duration_s"))
        self.lead = float(rospy.get_param("~lead_s"))
        self.body = rospy.get_param("~body_name")
        if self.force not in (2, 4) or self.duration != .3 or self.lead != .1 or self.body != "launch_slider::body":
            raise RuntimeError("Unexpected private force/duration/body parameters")
        self.names = {key: rospy.resolve_name(name) for key, name in
            (("pulse_service", "pulse"), ("cancel_service", "cancel"), ("apply_service", "apply_wrench"),
             ("clear_service", "clear_wrench"), ("world_service", "world_properties"), ("status_topic", "status"))}
        self.info.update(parameters={"force_n": self.force, "duration_s": self.duration, "lead_s": self.lead, "body_name": self.body}, resolved_names=self.names)
        self.publisher = rospy.Publisher("status", String, queue_size=1, latch=True)
        self.pulse_service = rospy.Service("pulse", Trigger, self.pulse)
        self.cancel_service = rospy.Service("cancel", Trigger, self.cancel)
        self.status("idle")

    def status(self, state, **extra):
        publish(self.publisher, {**self.info, "state": state, "command_sequence": self.sequence,
            "status_monotonic_ns": time.monotonic_ns(), **extra})

    def pulse(self, request):
        with self.lock:
            try:
                now = round(call(self.names["world_service"], GetWorldProperties).sim_time*1e9)
                if self.busy_until and now <= self.busy_until:
                    return TriggerResponse(False, "A pulse is already scheduled; concurrent jobs are refused")
                start = now+round(self.lead*1e9)
                duration = round(self.duration*1e9)
                self.busy_until = start+duration
                wrench = Wrench()
                wrench.force.x = self.force
                reply = call(self.names["apply_service"], ApplyBodyWrench, body_name=self.body,
                    reference_frame="world", reference_point=Point(), wrench=wrench,
                    start_time=rospy.Time(start//1000000000, start%1000000000), duration=rospy.Duration.from_sec(self.duration))
                self.sequence += 1
                job = {"pulse_id": uuid.uuid4().hex, "command_sequence": self.sequence, "force_n": self.force,
                    "duration_ns": duration, "requested_start_time_ns": start, "requested_end_time_ns": start+duration,
                    "scheduling_world_time_ns": now, "body_name": self.body, "reference_frame": "world",
                    "apply_response_success": reply.success, "request_monotonic_ns": time.monotonic_ns()}
                self.status("scheduled", last_pulse=job)
                import json
                return TriggerResponse(True, json.dumps(job, separators=(",", ":"), allow_nan=False))
            except Exception as error:
                # An uncertain RPC is never repeated; the reservation prevents overlap.
                self.status("error", error=str(error))
                return TriggerResponse(False, str(error))

    def cancel(self, request):
        with self.lock:
            try:
                call(self.names["clear_service"], BodyRequest, body_name=self.body)
                self.busy_until = 0
                self.status("cancelled", clear_response_has_measurement_proof=False)
                return TriggerResponse(True, "Scoped clear requested; actual stopped acceleration requires observation")
            except Exception as error:
                return TriggerResponse(False, str(error))


if __name__ == "__main__":
    rospy.init_node("driver")
    node = Driver()
    rospy.spin()
