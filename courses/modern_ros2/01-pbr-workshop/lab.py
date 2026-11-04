"""Observe true Gazebo Sim landings for two independent geometry controls."""

import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import _bootstrap  # Adds the installed course support directory.
from common import FORTRESS, ensure_running, move_to, result, xyz


STATIONS = ("station_a", "station_b", "station_c")


def vector(node, path, length):
    values = tuple(float(value) for value in node.findtext(path, "").split())
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError("Invalid SDF vector: " + path)
    return values


def sample_definition(world, name):
    model = world.find(f"model[@name='{name}']")
    x, y, z, roll, pitch, yaw = vector(model, "pose", 6)
    if (roll, pitch, yaw) != (0, 0, 0) or model.findtext("static") != "true":
        raise ValueError("Station must remain static and axis-aligned: " + name)
    link = model.find("link")
    visual = vector(link, "visual[@name='sample_visual']/geometry/box/size", 3)
    collision = vector(link, "collision[@name='support_collision']/geometry/box/size", 3)
    if min(visual + collision) <= 0:
        raise ValueError("Station dimensions must be positive: " + name)
    return {"center": [x, y, z], "visual_size": visual, "collision_size": collision}


def wait_for_landing(x, y, start_z, expected_z):
    move_to("probe", [x, y, start_z])
    deadline = time.monotonic() + 10
    samples = []
    last = None
    while time.monotonic() < deadline:
        last = xyz("probe")
        if (abs(last[0] - x) < .04 and abs(last[1] - y) < .04 and
                abs(last[2] - expected_z) < .04):
            if samples and abs(last[2] - samples[-1][2]) > .003:
                samples.clear()
            samples.append(last)
            if len(samples) == 3:
                return {"expected_z": expected_z, "landing": last, "settled_samples": samples}
        else:
            samples.clear()
        time.sleep(.08)
    raise RuntimeError(f"Probe did not settle near z={expected_z:.3f} m, last pose {last}")


def experiment():
    world_file = "world.fortress.sdf" if FORTRESS else "world.sdf"
    world = ET.parse(world_file).getroot().find("world")
    arch = world.find("model[@name='arch']")
    if len(arch.findall("link/collision")) != 3:
        raise ValueError("The arch must have three separate collision shapes")
    maps = [item.text.strip() for tag in ("albedo_map", "roughness_map", "normal_map")
            for item in world.findall(f".//{tag}") if item.text]
    if len(set(maps)) < 3 or not all(Path(item).is_file() for item in maps):
        raise ValueError("The local albedo, roughness and normal maps are required")
    stations = {name: sample_definition(world, name) for name in STATIONS}
    a, b, c = (stations[name] for name in STATIONS)
    if (a["collision_size"] != b["collision_size"] or
            a["visual_size"] == b["visual_size"] or
            b["visual_size"] != c["visual_size"] or
            c["collision_size"][:2] != b["collision_size"][:2] or
            c["collision_size"][2] <= b["collision_size"][2] or
            len({sample["center"][2] for sample in stations.values()}) != 1):
        raise ValueError("Keep A/B visual-only and B/C collision-height-only")
    probe = world.find("model[@name='probe']/link")
    probe_half_height = vector(probe, "collision/geometry/box/size", 3)[2] / 2

    ensure_running()
    beam_center = vector(arch, "pose", 6)
    beam = arch.find("link/collision[@name='beam_collision']")
    beam_top = vector(beam, "pose", 6)[2] + vector(beam, "geometry/box/size", 3)[2] / 2
    beam_landing = wait_for_landing(beam_center[0], beam_center[1], 3.7,
                                    beam_center[2] + beam_top + probe_half_height)
    floor_landing = wait_for_landing(0, 2.65, 1.8, probe_half_height)
    for sample in stations.values():
        x, y, z = sample["center"]
        expected = z + sample["collision_size"][2] / 2 + probe_half_height
        sample.update(wait_for_landing(x, y, expected + 1.05, expected))
    dz_visual = b["landing"][2] - a["landing"][2]
    dz_collision = c["landing"][2] - b["landing"][2]
    expected_delta = (c["collision_size"][2] - b["collision_size"][2]) / 2
    if abs(dz_visual) > .015 or abs(dz_collision - expected_delta) > .02:
        raise AssertionError(f"Unexpected measured contrast, A to B={dz_visual}, B to C={dz_collision}")

    return {
        "passed": True,
        "world": world_file,
        "beam_landing": beam_landing,
        "floor_landing": floor_landing,
        "stations": stations,
        "visual_only_height_delta_m": dz_visual,
        "collision_only_height_delta_m": dz_collision,
        "collision_only_expected_delta_m": expected_delta,
        "pbr_local_texture_files": sorted(set(maps)),
        "visual_check": "Inspect Ogre2 materials, lighting and silhouettes in the GUI.",
    }


if __name__ == "__main__":
    result(experiment())
