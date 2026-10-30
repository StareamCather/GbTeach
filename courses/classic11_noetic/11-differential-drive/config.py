"""Strict, read-only validation of this lesson's editable inputs."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PHASES = ("straight", "spin", "arc")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_parameters(path):
    p = json.loads(Path(path).read_text(encoding="utf-8"))
    if set(p) != {"schema", "forward_speed_m_s", "spin_yaw_rate_rad_s", "arc_yaw_rate_rad_s"}:
        raise ValueError("parameters.json fields must be schema/forward_speed_m_s/spin_yaw_rate_rad_s/arc_yaw_rate_rad_s")
    if type(p["schema"]) is not int or p["schema"] != 1:
        raise ValueError("schema must be integer 1")
    for key in sorted(set(p)-{"schema"}):
        value = p[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(key+" must be a finite number")
        low, high = (.1, .4) if key == "forward_speed_m_s" else (.2, 1.2)
        if not low <= value <= high:
            raise ValueError(key+" must be within [%g,%g]" % (low, high))
    return p


def _normalized(text):
    values = (text or "").split()
    try:
        numbers = [float(value) for value in values]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("world.sdf contains non-finite numeric text")
        return " ".join(format(value, ".15g") for value in numbers)
    except ValueError:
        if any(value.lower() in ("nan", "inf", "+inf", "-inf", "infinity") for value in values):
            raise ValueError("world.sdf contains non-finite numeric text")
        return " ".join(values)


def read_world(path, template):
    root = ET.parse(str(path)).getroot()
    fixed = ET.parse(str(template)).getroot()
    model = root.find("world/model[@name='differential_car']")
    if model is None:
        raise ValueError("world.sdf differential_car model is missing")
    wheel_values = []
    for name in ("left_wheel", "right_wheel"):
        nodes = model.findall("link[@name='%s']/collision/surface/friction/ode" % name)
        if len(nodes) != 1:
            raise ValueError("world.sdf "+name+" must have exactly one wheel collision/surface")
        for key in ("mu", "mu2"):
            try:
                value = float(nodes[0].findtext(key, "nan"))
            except ValueError:
                raise ValueError("world.sdf wheel "+key+" must be finite")
            if not math.isfinite(value) or not any(abs(value-x) < 1e-12 for x in (.8, .001)):
                raise ValueError("world.sdf wheel mu/mu2 must all be 0.8 normal or 0.001 slip")
            wheel_values.append(value)
    if max(wheel_values)-min(wheel_values) > 1e-12:
        raise ValueError("world.sdf mixed wheel surfaces: both wheels mu/mu2 must agree")

    def signature(node, wheel=False):
        wheel = wheel or (node.tag == "link" and node.get("name") in ("left_wheel", "right_wheel"))
        text = "EDITABLE_WHEEL_MU" if wheel and node.tag in ("mu", "mu2") else _normalized(node.text)
        return node.tag, tuple(sorted(node.attrib.items())), text, tuple(signature(child, wheel) for child in node)
    if signature(root) != signature(fixed):
        raise ValueError("world.sdf fixed rig topology/geometry/inertia/axis/stock joint reference differs; only both wheel mu/mu2 may change")
    return {"surface_mode": "normal" if wheel_values[0] > .1 else "slip", "wheel_mu": wheel_values[0]}


def validate_model(m, world):
    expected = {"wheel_radius_m": .1, "wheel_separation_m": .36, "left_collision_bbox_max_m": .2,
                "right_collision_bbox_max_m": .2, "left_mass_kg": .2, "right_mass_kg": .2,
                "chassis_mass_kg": 2., "caster_mass_kg": .05, "physics_step_s": .001,
                "floor_mu": .8, "floor_mu2": .8, "caster_mu": .001, "caster_mu2": .001,
                "left_mu": world["wheel_mu"], "left_mu2": world["wheel_mu"],
                "right_mu": world["wheel_mu"], "right_mu2": world["wheel_mu"]}
    for name in ("left", "right"):
        expected.update({name+"_Ixx_kg_m2": .000501666666666667, name+"_Iyy_kg_m2": .001,
                         name+"_Izz_kg_m2": .000501666666666667, name+"_com_x_m": 0.,
                         name+"_com_y_m": 0., name+"_com_z_m": 0.,
                         name+"_collision_radius_m": .1, name+"_collision_length_m": .01})
    expected.update({"chassis_Ixx_kg_m2": .0106666666666667, "chassis_Iyy_kg_m2": .0304666666666667,
                     "chassis_Izz_kg_m2": .039, "chassis_com_x_m": -.1, "chassis_com_y_m": 0., "chassis_com_z_m": 0.,
                     "caster_Ixx_kg_m2": .0000245, "caster_Iyy_kg_m2": .0000245, "caster_Izz_kg_m2": .0000245,
                     "caster_com_x_m": 0., "caster_com_y_m": 0., "caster_com_z_m": 0.})
    for key, want in expected.items():
        value = m.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value-want) > 1e-8:
            raise AssertionError("Actual SDK model/surface mismatch: "+key)
    for key, want in (("initial_left_axis_xyz", (0., 1., 0.)), ("initial_right_axis_xyz", (0., 1., 0.))):
        value = m.get(key)
        if not isinstance(value, list) or len(value) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x-y) > 1e-8 for x, y in zip(value, want)):
            raise AssertionError("Actual SDK wheel axis mismatch: "+key)
    for key, want in (("initial_left_anchor_xyz", (0., .18, .1002)), ("initial_right_anchor_xyz", (0., -.18, .1002))):
        value = m.get(key)
        if not isinstance(value, list) or len(value) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) or abs(x-y) > 1e-8 for x, y in zip(value, want)):
            raise AssertionError("Actual SDK initial anchor mismatch: "+key)
    if m.get("surface_mode") != world["surface_mode"] or not m.get("stock_subscriber_connected"):
        raise AssertionError("Actual stock subscriber/surface mode differs")
