"""Current learner configuration, validated before any simulator mutation."""
import hashlib
import json
import math
from pathlib import Path

PHASES = (("torque", 1.), ("speed-p", 6.), ("speed-pi", 6.),
          ("position-p", 3.), ("position-pd", 3.), ("saturation", 6.))
MODEL = {"mass_kg": 1., "radius_m": .25, "length_m": .06,
         "Ixx_kg_m2": .015925, "Iyy_kg_m2": .015925, "Izz_kg_m2": .03125,
         "damping_Nms_rad": .1, "effort_limit_Nm": 3., "physics_step_s": .001,
         "gravity_z": -9.81}
DEFAULT_KEYS = {"schema", "velocity_target_rad_s", "speed_p_kp", "speed_pi_kp", "speed_pi_ki",
                "position_target_rad", "position_kp", "position_kd",
                "integral_term_limit_Nm", "command_limit_Nm", "saturation_limit_Nm",
                "load_torque_Nm", "load_start_s", "torque_command_Nm"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_parameters(path):
    p = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(p, dict) or set(p) != DEFAULT_KEYS:
        raise ValueError("parameters.json keys/schema do not match this motor exercise")
    if type(p["schema"]) is not int or p["schema"] != 1:
        raise ValueError("schema must be integer 1")
    for key in DEFAULT_KEYS-{"schema"}:
        value = p[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(key+" must be a finite number, not boolean")
    intervals = {"velocity_target_rad_s": (2., 5.), "speed_p_kp": (.2, 1.), "speed_pi_kp": (.3, .6),
                 "speed_pi_ki": (.4, 1.5), "position_kp": (1.5, 2.5),
                 "position_kd": (.3, .6), "integral_term_limit_Nm": (.7, 1.),
                 "command_limit_Nm": (1.5, 2.5), "saturation_limit_Nm": (.3, .45),
                 "load_torque_Nm": (.1, .25), "torque_command_Nm": (.3, .5)}
    for key, (lower, upper) in intervals.items():
        if not lower <= p[key] <= upper:
            raise ValueError("%s outside instrument range [%g,%g]" % (key, lower, upper))
    if not .5 <= abs(p["position_target_rad"]) <= 1.:
        raise ValueError("position_target_rad magnitude must be between .5 and 1 rad")
    if p["load_start_s"] != 2.:
        raise ValueError("load_start_s must remain 2 s for the paired load-step measurement")
    needed = .1*p["velocity_target_rad_s"]+p["load_torque_Nm"]
    if p["integral_term_limit_Nm"] < needed+.05:
        raise ValueError("integral_term_limit_Nm too small for the standard PI equilibrium")
    if p["command_limit_Nm"] < needed+.2:
        raise ValueError("command_limit_Nm too small for the standard unsaturated equilibrium")
    cap = p["saturation_limit_Nm"]
    if not (.05 <= cap-p["load_torque_Nm"] and cap <= needed-.1):
        raise ValueError("saturation_limit_Nm must permit positive motion but remain below required torque")
    zeta = (.1+p["position_kd"])/(2*math.sqrt(.03125*p["position_kp"]))
    if not .75 <= zeta <= 1.5:
        raise ValueError("position_kd/position_kp outside the damped comparison range")
    return p


def validate_model(actual):
    for key, expected in MODEL.items():
        if not math.isfinite(float(actual[key])) or abs(float(actual[key])-expected) > 1e-9:
            raise AssertionError("Actual SDK rotor/model parameter differs: "+key)
    if len(actual["axis_xyz"]) != 3 or any(not math.isfinite(float(a)) or abs(a-b) > 1e-9 for a, b in zip(actual["axis_xyz"], [0., 0., 1.])):
        raise AssertionError("Actual joint axis must be world +Z")
    return actual
