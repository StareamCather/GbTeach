"""Current personal inputs and actual SDK rig validation; no simulator control."""
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_parameters(path):
    p = json.loads(Path(path).read_text(encoding="utf-8"))
    if set(p) != {"schema", "open_target_m"} or type(p["schema"]) is not int or p["schema"] != 1:
        raise ValueError("parameters.json requires integer schema=1 and open_target_m only")
    value = p["open_target_m"]
    if type(value) not in (int, float) or not math.isfinite(value) or not .2 <= value <= .7:
        raise ValueError("open_target_m must be a finite number within [0.2,0.7] m")
    return p


def read_world(path, template):
    if Path(path).read_bytes() != Path(template).read_bytes():
        raise ValueError("world.sdf fixed rig/topology/gains differ; restore the lesson world template and reload")


def validate_model(m):
    expected = {"mass_kg": 2., "Ixx_kg_m2": .10726666666666667, "Iyy_kg_m2": .16666666666666667,
                "Izz_kg_m2": .0606, "damping_Ns_m": 4., "lower_limit_m": 0., "upper_limit_m": .8,
                "effort_limit_N": 20., "physics_step_s": .001, "pid_p": 80., "pid_i": 0., "pid_d": 20., "pid_command_limit_N": 20., "pid_command_min_N": -20.}
    for key, want in expected.items():
        value = m.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value-want) > 1e-8:
            raise AssertionError("Actual SDK gate model mismatch: "+key)
    for key, want in (("box_size_xyz", [.6, .06, .8]), ("initial_axis_xyz", [1., 0., 0.]),
                      ("initial_world_pose", [0., 0., .6, 1., 0., 0., 0.])):
        value = m.get(key)
        if not isinstance(value, list) or len(value) != len(want) or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x-y) > 1e-8 for x, y in zip(value, want)):
            raise AssertionError("Actual SDK gate geometry mismatch: "+key)
    if m.get("joint_parent") != "world" or m.get("joint_child") != "panel":
        raise AssertionError("Actual SDK slide joint topology differs")
