#!/usr/bin/env python3
"""L13: independently read numeric terrain, real resting probes and Actor frames.

--math-only and --configuration-only never start Gazebo. Numerical PNG and
interpolation examples are validator inputs, never substitute observed frames.
Physical acceptance reads the actual current personal SDF, native heightfield
and ray metadata, and a new run of post-physics observations for each case.
Rendering objectives belong to the separate real-GUI capture acceptance.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import types
import uuid
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson06_acceptance import actual_stats
from tests.lesson091_acceptance import control_directory, digest, read_json, require, source_hashes
from tests.lesson10_acceptance import bounded_steps
from tests.lesson12_acceptance import collect_lab, preserve_evidence

KEY = "classic11-noetic:13"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
HEIGHTMAP_VERTICES = 65
TERRAIN_POINTS = {"probe_nw": (-1., 1.), "probe_ne": (1., 1.),
                  "probe_sw": (-1., -1.), "probe_se": (1., -1.)}
TWIN_POINTS = {"probe_matte": (3.4, -.9), "probe_emissive": (3.4, .9)}
REST_POSITION_TOLERANCE = .01
REST_HORIZONTAL_TOLERANCE = .03
REST_SPEED_LIMIT = .03
REST_CONTINUOUS_SECONDS = .2
RAY_HEIGHT_TOLERANCE = 1e-4
MARKER_POSITION_TOLERANCE = 1e-5


def finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), label+" must be a finite real number")
    return float(value)


def close(actual, expected, label, tolerance=1e-8):
    actual, expected = finite(actual, label), finite(expected, label)
    require(abs(actual-expected) <= tolerance,
            "{}: actual {} != expected {} +/- {}".format(label, actual, expected, tolerance))


def paeth(left, up, upper_left):
    prediction = left+up-upper_left
    distances = [abs(prediction-left), abs(prediction-up), abs(prediction-upper_left)]
    return (left, up, upper_left)[distances.index(min(distances))]


def read_numeric_png(raw):
    """Strict lossless grayscale decoder; PNG sample values remain numeric data."""
    require(isinstance(raw, bytes) and raw.startswith(PNG_SIGNATURE), "Heightmap lacks a valid PNG signature")
    position, header, chunks, compressed, ended = 8, None, [], bytearray(), False
    while position < len(raw):
        require(position+12 <= len(raw), "Truncated PNG chunk header")
        size = struct.unpack(">I", raw[position:position+4])[0]
        kind = raw[position+4:position+8]
        end = position+12+size
        require(end <= len(raw), "Truncated PNG chunk payload")
        body = raw[position+8:position+8+size]
        crc = struct.unpack(">I", raw[position+8+size:end])[0]
        require((zlib.crc32(kind+body) & 0xffffffff) == crc, "Invalid PNG chunk CRC: "+repr(kind))
        require(not ended, "Unexpected data after PNG IEND")
        if kind == b"IHDR":
            require(header is None and not chunks and size == 13, "PNG requires one first IHDR chunk")
            header = struct.unpack(">IIBBBBB", body)
            width, height, depth, color, compression, filtering, interlace = header
            require(width == height == HEIGHTMAP_VERTICES and (width-1) & (width-2) == 0,
                    "Lesson heightmap must be square 65x65 (2^n+1) pixels for the pinned renderer batches")
            require((depth, color, compression, filtering, interlace) == (8, 0, 0, 0, 0),
                    "Lesson heightmap must be noninterlaced 8-bit grayscale PNG")
        elif kind == b"IDAT":
            require(header is not None, "PNG image data precedes its dimensions")
            compressed.extend(body)
        elif kind == b"IEND":
            require(size == 0 and compressed, "PNG IEND requires image data and no payload")
            ended = True
        else:
            require(kind not in (b"PLTE", b"gAMA", b"sRGB", b"iCCP", b"tRNS"),
                    "Color/palette/gamma metadata must not transform numeric heightmap samples")
            require(kind and kind[0] & 32, "Unsupported critical PNG chunk: "+repr(kind))
        chunks.append(kind.decode("ascii"))
        position = end
    require(header is not None and ended and position == len(raw), "Incomplete PNG image")
    width, height = header[:2]
    expected = height*(width+1)
    decoder = zlib.decompressobj()
    scanlines = decoder.decompress(bytes(compressed), expected+1)
    require(len(scanlines) == expected and decoder.eof and not decoder.unused_data and not decoder.unconsumed_tail,
            "PNG decompressed sample count does not match its declared dimensions")
    rows, previous = [], [0]*width
    for y in range(height):
        filter_type = scanlines[y*(width+1)]
        require(filter_type <= 4, "Invalid PNG row filter")
        row = []
        for x, value in enumerate(scanlines[y*(width+1)+1:(y+1)*(width+1)]):
            left, up, corner = row[x-1] if x else 0, previous[x], previous[x-1] if x else 0
            predictor = (0, left, up, (left+up)//2, paeth(left, up, corner))[filter_type]
            row.append((value+predictor) & 255)
        rows.append(row)
        previous = row
    return {"width": width, "height": height, "bit_depth": 8, "color_type": 0,
            "pixels": rows, "minimum": min(map(min, rows)), "maximum": max(map(max, rows)),
            "sha256": hashlib.sha256(raw).hexdigest(), "chunks": chunks}


def numeric_png_example(filter_type=0, vertices=HEIGHTMAP_VERTICES):
    """In-memory numerical regression input only, never a physical observation."""
    split = (vertices-1)//2
    pixels = [[85 if v < split and u < split else 170 if v < split else 0 if u < split else 255
               for u in range(vertices)] for v in range(vertices)]
    data, previous = bytearray(), [0]*vertices
    for row in pixels:
        data.append(filter_type)
        for x, value in enumerate(row):
            left, up, corner = row[x-1] if x else 0, previous[x], previous[x-1] if x else 0
            predictor = (0, left, up, (left+up)//2, paeth(left, up, corner))[filter_type]
            data.append((value-predictor) & 255)
        previous = row
    def chunk(kind, body):
        return struct.pack(">I", len(body))+kind+body+struct.pack(">I", zlib.crc32(kind+body) & 0xffffffff)
    image = PNG_SIGNATURE+chunk(b"IHDR", struct.pack(">IIBBBBB", vertices, vertices, 8, 0, 0, 0, 0))
    image += chunk(b"IDAT", zlib.compress(bytes(data)))+chunk(b"IEND", b"")
    return image, pixels


def pixel_coordinates(x, y, size=4., vertices=HEIGHTMAP_VERTICES):
    # Top-down raw PNG rows, ODE flipY=false followed by the native +90deg roll.
    return (x+size/2)*(vertices-1)/size, (size/2-y)*(vertices-1)/size


def plateau_heights(png, height):
    require(png["minimum"] == 0 and png["maximum"] == 255,
            "Numeric terrain needs black/white anchors for a known native normalization")
    expected_values = dict(zip(TERRAIN_POINTS, (85, 170, 0, 255)))
    result = {}
    for name, (x, y) in TERRAIN_POINTS.items():
        u, v = pixel_coordinates(x, y)
        require(u.is_integer() and v.is_integer(), "Probe does not align to its declared indexed height sample")
        u, v = int(u), int(v)
        value = png["pixels"][v][u]
        require(value == expected_values[name], "Asymmetric numeric heightmap quadrant is mirrored or modified: "+name)
        # Ball contact footprint and offset ray both lie safely inside a flat
        # plateau. No bilinear approximation to sloped ODE triangles is used.
        ray_u, ray_v = pixel_coordinates(x+.35, y)
        grid_spacing = 4./(HEIGHTMAP_VERTICES-1)
        footprint = math.ceil(.1/grid_spacing)
        indices = {(ix, iy) for ix in range(u-footprint, u+footprint+1) for iy in range(v-footprint, v+footprint+1)}
        indices.update((ix, iy) for ix in (math.floor(ray_u), math.ceil(ray_u))
                       for iy in (math.floor(ray_v), math.ceil(ray_v)))
        require(all(png["pixels"][iy][ix] == value for ix, iy in indices),
                "Probe/ray footprint is not on the declared wide flat plateau: "+name)
        result[name] = {"ix": u, "iy": v, "gray": value, "surface_z_m": height*value/255,
                        "center_z_m": height*value/255+.1, "position_xy_m": [x, y],
                        "ray_xy_m": [x+.35, y]}
    return result


def smoothstep(value):
    return value*value*(3-2*value)


def marker_position(script_time, cycle):
    phase = script_time % cycle
    times = [fraction*cycle for fraction in (0., 5/12, .5, 11/12, 1.)]
    positions = [-1., 1., 1., -1., -1.]
    for index in range(4):
        if times[index] <= phase <= times[index+1]:
            u = (phase-times[index])/(times[index+1]-times[index])
            return [positions[index]+(positions[index+1]-positions[index])*smoothstep(u), -2.7, .15]
    raise AssertionError("Native script phase falls outside the closed trajectory")


def renderer_batch_regression():
    """Pinned-source LOD arithmetic, not an executed renderer or physical test."""
    # Gazebo 11.15.1 Heightmap.cc fixes the batch sizes at 17 and 65.
    # OGRE v1-9-0 OgreTerrain.cpp:979-982 derives these three quantities.
    minimum_batch, maximum_batch = 17, 65
    leaf_levels = int(math.log2(maximum_batch-1)-math.log2(minimum_batch-1)+1)
    checks = []
    for vertices in (17, 33, 65):
        total_levels = int(math.log2(vertices-1)-math.log2(minimum_batch-1)+1)
        tree_depth = total_levels-leaf_levels+1
        compatible = total_levels >= leaf_levels and tree_depth >= 1
        require(compatible is (vertices == 65), "Pinned renderer LOD arithmetic changed unexpectedly")
        checks.append({"vertices": vertices, "total_lod_levels": total_levels,
                       "leaf_lod_levels": leaf_levels, "tree_depth_before_unsigned_storage": tree_depth,
                       "compatible_with_these_fixed_batches": compatible})
    rejected = []
    for vertices in (17, 33, 64):
        image, _ = numeric_png_example(vertices=vertices)
        try:
            read_numeric_png(image)
        except AssertionError as error:
            require("65x65" in str(error), "Wrong-size numeric fixture lacks its relevant diagnosis")
            rejected.append(vertices)
        else:
            raise AssertionError("A renderer-incompatible lesson PNG was accepted: "+str(vertices))
    return {"passed": True, "kind": "pinned_source_LOD_and_numeric_PNG_inputs_not_actual_rendering",
            "gazebo_started": False, "minimum_batch": minimum_batch, "maximum_batch": maximum_batch,
            "checks": checks, "wrong_lesson_dimensions_rejected": rejected,
            "fixed_world_grid_spacing_m": 4./(HEIGHTMAP_VERTICES-1),
            "sources": ["https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Heightmap.cc#L754",
                        "https://github.com/OGRECave/ogre/blob/v1-9-0/Components/Terrain/src/OgreTerrain.cpp#L979"]}


def math_regressions():
    base, expected = numeric_png_example()
    for mode in range(5):
        image, _ = numeric_png_example(mode)
        require(read_numeric_png(image)["pixels"] == expected, "PNG lossless filter reconstruction is wrong")
    decoded = read_numeric_png(base)
    heights = plateau_heights(decoded, .6)
    for name, value in zip(TERRAIN_POINTS, (.3, .5, .1, .7)):
        close(heights[name]["center_z_m"], value, "Independent quantitative terrain reference")
    corrupted = bytearray(base)
    corrupted[-5] ^= 1
    try:
        read_numeric_png(bytes(corrupted))
    except AssertionError:
        crc_rejected = True
    else:
        raise AssertionError("A corrupted heightmap CRC was accepted")
    mirrored = dict(decoded, pixels=list(reversed(decoded["pixels"])))
    try:
        plateau_heights(mirrored, .6)
    except AssertionError:
        vertical_mirror_rejected = True
    else:
        raise AssertionError("Raw image row direction was confused with world Y")
    for cycle in (6., 8.):
        for time_value, x in ((0., -1.), (cycle*5/24, 0.), (cycle*5/12, 1.),
                              (cycle*.5, 1.), (cycle*17/24, 0.), (cycle*11/12, -1.), (cycle, -1.)):
            close(marker_position(time_value, cycle)[0], x, "Zero-tangent marker trajectory reference", 1e-12)
    return {"passed": True, "kind": "pure_numeric_PNG_geometry_and_trajectory_not_physics",
            "gazebo_started": False, "all_five_lossless_PNG_filters_round_trip": True,
            "renderer_batch_compatibility": renderer_batch_regression(),
            "bad_crc_rejected": crc_rejected, "vertical_image_mirror_rejected": vertical_mirror_rejected,
            "baseline_flat_plateau_expectations": heights,
            "height_only_point9_expectations": plateau_heights(decoded, .9),
            "marker_cycles_checked_s": [6, 8], "same_epoch_snapshot_regression": epoch_snapshot_regression(),
            "joint_rest_window_regression": joint_rest_regression(decoded)}


def xml_tree(raw):
    return ET.fromstring(raw, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))


def numeric_text(text):
    tokens = (text or "").split()
    try:
        numbers = [float(token) for token in tokens]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError("nonfinite XML")
        return tuple(value.hex() for value in numbers)
    except ValueError:
        return " ".join(tokens)


def semantic_world(raw, edited=None):
    """Include comments and mask only the explicitly edited scalar fields."""
    root = xml_tree(raw)
    masked = {}
    if edited == "height":
        for element in root.findall(".//heightmap/size"):
            values = [float(value) for value in element.text.split()]
            masked[id(element)] = (values[0].hex(), values[1].hex(), "<edited-height>")
    elif edited == "cycle":
        for element in root.findall("./world/actor/script/trajectory/waypoint/time"):
            masked[id(element)] = "<edited-time>"
    elif edited == "daylight":
        element = root.find("./world/light[@name='daylight']/diffuse")
        values = [float(value) for value in element.text.split()]
        masked[id(element)] = ("<edited-rgb>", values[3].hex())
    elif edited == "shadows":
        for element in root.findall("./world/light/cast_shadows"):
            masked[id(element)] = "<edited-shadow-enable>"
    def visit(element):
        tag = element.tag if isinstance(element.tag, str) else "<comment>"
        text = element.text if tag == "<comment>" else masked.get(id(element), numeric_text(element.text))
        return tag, tuple(sorted(element.attrib.items())), text, tuple(visit(child) for child in element)
    return visit(root)


def load_config(lesson):
    module = types.ModuleType("_l13_config_probe")
    module.__file__ = str(lesson.folder/"config.py")
    exec(compile((lesson.folder/"config.py").read_bytes(), module.__file__, "exec"), module.__dict__)
    return module


def current_parameters(workspace):
    root = ET.parse(str(workspace/"world.sdf")).getroot()
    height = float(root.find(".//collision/geometry/heightmap/size").text.split()[2])
    marker = root.find("./world/actor[@name='marker']")
    cycle = float(marker.findall("script/trajectory/waypoint/time")[-1].text)
    daylight = float(root.findtext("./world/light[@name='daylight']/diffuse").split()[0])
    return {"height_m": height, "actor_cycle_s": cycle, "daylight": daylight}


def edit_regressions(lesson, private):
    config = load_config(lesson)
    folder = private/"explicit-SDF-edits"
    folder.mkdir()
    original = (lesson.folder/"world.sdf").read_text(encoding="utf-8")
    varied = original.replace("<size>4 4 .6</size>", "<size>4.0\t4.0   0.6</size>")
    varied = varied.replace('<light name="daylight" type="directional">', "<light type = 'directional' name = 'daylight' >")
    comment = "<!-- learner-preservation-marker: do not restore my material -->"
    varied = varied.replace('<model name="matte_support">', comment+'\n<model name="matte_support">')
    varied, count = re.subn(r'(<model name="matte_support".*?<diffuse>)[^<]+(</diffuse>)',
                          r'\g<1>.12 .23 .34 1\2', varied, count=1, flags=re.S)
    require(count == 1 and varied != original, "Editor regression did not create its legitimate personal format/material variant")
    path = folder/"world.sdf"
    results = []
    for field, value, key in (("height", .9, "height_m"), ("cycle", 8., "actor_cycle_s"), ("daylight", .2, "daylight")):
        path.write_text(varied, encoding="utf-8")
        before_params = config.read_world(path, lesson.folder/"world.sdf")
        before = path.read_bytes()
        result = config.edit_world(path, lesson.folder/"world.sdf", field, value)
        after = path.read_bytes()
        require(before != after and result["passed"] is True and result["reload_required"] is True,
                "An explicit editor silently succeeded without changing the requested personal field")
        after_params = config.read_world(path, lesson.folder/"world.sdf")
        close(after_params[key], value, "Explicit SDF helper requested value")
        for other in before_params:
            if other != key:
                close(after_params[other], before_params[other], "Explicit edit preserved unrelated parameter "+other)
        require(semantic_world(before, field) == semantic_world(after, field),
                "Explicit editor changed unrelated XML semantics or comments: "+field)
        require(comment.encode() in after and b".12 .23 .34 1" in after,
                "Explicit editor silently restored the learner's comment or allowed material modification")
        results.append({"field": field, "requested": value, "before_parameters": before_params,
                        "actual_after_parameters": after_params, "only_requested_semantics_changed": True,
                        "personal_material_and_comment_preserved": True, "result": result})
    return {"passed": True, "kind": "pure_private_SDF_edit_regressions_not_physics", "gazebo_started": False,
            "numeric_tabs_single_quotes_and_attribute_order_accepted": True, "checks": results}


def integer(value, label):
    number = finite(value, label)
    require(number.is_integer() and number >= 0, label+" must be a nonnegative integer")
    return int(number)


def vector(values, length, label):
    require(isinstance(values, list) and len(values) == length, label+" has the wrong dimension")
    return [finite(value, label) for value in values]


def pose(values, label):
    values = vector(values, 7, label)
    close(sum(x*x for x in values[3:]), 1., label+" quaternion norm", 1e-6)
    return values


def check_model(workspace, status):
    parameters = current_parameters(workspace)
    png = read_numeric_png((workspace/"assets/heightmap.png").read_bytes())
    levels = plateau_heights(png, parameters["height_m"])
    require(status["parameters"] == parameters and status["png_sha256"] == png["sha256"],
            "Native metadata is not loaded from the current personal terrain/world")
    model, terrain = status["model"], status["model"]["heightmap"]
    require(terrain["uri"] == "file://assets/heightmap.png" and terrain["sampling"] == 1
            and terrain["vertex_count"] == [HEIGHTMAP_VERTICES]*2, "Actual SDK heightfield URI or sampling geometry differs")
    for actual, expected in zip(vector(terrain["size_m"], 3, "SDK heightmap size"), [4., 4., parameters["height_m"]]):
        close(actual, expected, "Native SDK heightfield size")
    require(vector(terrain["pos_m"], 3, "SDK heightmap pos") == [0., 0., 0.], "Heightmap offset moved the fixed local/world frame")
    require(terrain["collision_name"].endswith("terrain::terrain_link::terrain_collision"), "Wrong actual terrain collision identity")
    indexed = {item["id"]: item for item in terrain["indexed_heights"]}
    require(set(indexed) == set(TERRAIN_POINTS), "Native indexed heights omitted a coordinate quadrant")
    for name, expected in levels.items():
        actual = indexed[name]
        require(actual["ix"] == expected["ix"] and actual["iy"] == expected["iy"],
                "ODE raw indexed height was confused with reversed transport rows")
        close(actual["height_m"], expected["surface_z_m"], "Raw SDK height independently derived from PNG "+name, 1e-6)
    close(model["physics_step_s"], .001, "Actual physics step")
    require(vector(model["gravity_m_s2"], 3, "Actual gravity") == [0., 0., -9.81], "Actual gravity differs from the experiment")
    probes = {item["id"]: item for item in model["probes"]}
    require(set(probes) == set(TERRAIN_POINTS) | set(TWIN_POINTS), "Actual six dynamic probes are incomplete")
    for name, xy in dict(TERRAIN_POINTS, **TWIN_POINTS).items():
        item = probes[name]
        for actual, expected in zip(vector(item["initial_position_m"], 3, "Actual initial probe"), list(xy)+[2.]):
            close(actual, expected, "Actual initialized native probe "+name)
        close(item["radius_m"], .1, "Actual sphere radius")
        close(item["mass_kg"], .1, "Actual sphere mass")
        require(item["gravity"] is True and item["kinematic"] is False and item["is_static"] is False,
                "A dynamic probe was replaced by a kinematic/static animation")
    supports = {item["name"]: item for item in model["supports"]}
    for name, y in (("matte_support", -.9), ("emissive_support", .9)):
        support = supports[name]
        require(vector(support["size_m"], 3, "Actual support box size") == [.6, .6, .6], "Material twins changed their collision shape")
        p = pose(support["world_pose"], "Actual support collision world pose")
        for actual, expected in zip(p[:3], [3.4, y, .3]):
            close(actual, expected, "Actual support collision position")
        close(abs(p[3]), 1., "Actual support collision orientation")
    walker = model["actors"]["walker"]
    require(walker["mesh_node_count"] == 31 and any(item["name"] == "walking" and item["node_count"] > 0
                and item["length_s"] > 0 for item in walker["animations"]),
            "The native walk asset has no valid actual skeleton animation")
    marker_links = [item for item in model["actors"]["marker"]["links"] if item["name"] == "marker_link"]
    require(len(marker_links) == 1 and all(type(marker_links[0][key]) is bool for key in ("kinematic", "gravity", "is_static")),
            "Bare marker initialization metadata omitted its observed native link flags")
    lights = {item["name"].split("::")[-1]: item for item in model["lights"]}
    require({"daylight", "point_lamp", "spot_lamp"} <= set(lights), "Actual server light metadata is incomplete")
    for channel in ("r", "g", "b"):
        close(lights["daylight"]["diffuse"][channel], parameters["daylight"], "Loaded server daylight declaration")
    walk = status["assets"]["walk"]
    asset = Path(walk["path"])
    require(asset.is_file() and asset.stat().st_size == 2277322
            and digest(asset) == walk["sha256"] == "49af0df3a319d1cb8ca2cebf02dbd00f625e5d5bec820bc5e109925b18b65c6e",
            "Actual offline walk asset does not match its declared package provenance")
    return {"parameters": parameters, "independent_PNG_heights": levels, "actual_native_model": model,
            "actual_asset_sha256": walk["sha256"], "light_visual_metadata_scope": "server_declaration_not_renderer_readback"}


def read_jsonl(path):
    require(path.is_file(), "Missing actual observation stream: "+str(path))
    result = []
    for line in path.read_text(encoding="utf-8").splitlines(keepends=True):
        if not line.endswith("\n"):
            # A running observer may currently be writing its next row. Never
            # promote the incomplete fragment into an observation.
            break
        if line.strip():
            result.append(json.loads(line))
    return result


def probe_rest_analysis(rows, parameters, png):
    levels = plateau_heights(png, parameters["height_m"])
    require(len(rows) >= 201, "Rest proof needs at least 201 actual one-ms snapshots")
    previous, runs, longest, extrema = None, {}, {}, {}
    joint_start, joint_longest, joint_count, joint_best_count = None, 0., 0, 0
    for row in rows:
        elapsed = finite(row["elapsed_s"], "Actual probe-tail elapsed time")
        require(3.-1e-8 <= elapsed <= 3.5+1e-8, "Probe-tail stream escaped the declared physical window")
        if previous:
            require(integer(row["iteration"], "Actual tail iteration") == integer(previous["iteration"], "Actual tail iteration")+1,
                    "Real probe-tail physics iterations were skipped or repeated")
            close(row["sim_time_s"]-previous["sim_time_s"], .001, "Actual one-ms tail clock interval", 1e-8)
        actual = {item["id"]: item for item in row["probes"]}
        require(set(actual) == set(TERRAIN_POINTS) | set(TWIN_POINTS), "A real tail row omitted a probe")
        joint_good = True
        for name, xy in dict(TERRAIN_POINTS, **TWIN_POINTS).items():
            item = actual[name]
            p, velocity = vector(item["position_m"], 3, "Actual resting sphere position"), vector(item["velocity_m_s"], 3, "Actual resting sphere velocity")
            target = levels[name]["center_z_m"] if name in levels else .7
            horizontal, speed, error = math.hypot(p[0]-xy[0], p[1]-xy[1]), math.sqrt(sum(x*x for x in velocity)), abs(p[2]-target)
            good = error <= REST_POSITION_TOLERANCE and horizontal <= REST_HORIZONTAL_TOLERANCE and speed <= REST_SPEED_LIMIT
            joint_good = joint_good and good
            if good:
                runs[name] = runs.get(name, [])+[row["sim_time_s"]]
                longest[name] = max(longest.get(name, 0.), runs[name][-1]-runs[name][0])
            else:
                runs[name] = []
            values = extrema.setdefault(name, {"maximum_height_error_m": 0., "maximum_horizontal_offset_m": 0., "maximum_speed_m_s": 0., "expected_center_z_m": target})
            values["maximum_height_error_m"] = max(values["maximum_height_error_m"], error)
            values["maximum_horizontal_offset_m"] = max(values["maximum_horizontal_offset_m"], horizontal)
            values["maximum_speed_m_s"] = max(values["maximum_speed_m_s"], speed)
        if joint_good:
            joint_start = row["sim_time_s"] if joint_start is None else joint_start
            joint_count += 1
            joint_longest = max(joint_longest, row["sim_time_s"]-joint_start)
            joint_best_count = max(joint_best_count, joint_count)
        else:
            joint_start, joint_count = None, 0
        previous = row
    for name in extrema:
        require(longest.get(name, 0.) >= REST_CONTINUOUS_SECONDS-1e-8,
                "Actual sphere lacks .2s of consecutive one-ms resting snapshots: "+name)
        extrema[name]["actual_longest_stable_snapshot_span_s"] = longest[name]
    require(joint_longest >= REST_CONTINUOUS_SECONDS-1e-8 and joint_best_count >= 201,
            "All six actual spheres need one common .2s interval of at least 201 consecutive stable snapshots")
    return {"actual_tail_snapshots": len(rows), "actual_tail_first_last_s": [rows[0]["elapsed_s"], rows[-1]["elapsed_s"]],
            "actual_one_ms_physics_rows": True, "minimum_required_snapshots_for_point_span": 201,
            "joint_longest_stable_span_s": joint_longest, "joint_longest_stable_snapshots": joint_best_count,
            "probes": extrema, "first_actual_snapshot": rows[0], "last_actual_snapshot": rows[-1]}


def observation_complete(status, duration):
    snapshot = status.get("snapshot", {})
    return (status.get("ready") is True and not status.get("error") and status.get("reset_pending") is False
            and isinstance(status.get("run_id"), str) and bool(status["run_id"])
            and snapshot.get("run_id") == status.get("run_id") and "epoch" in snapshot
            and snapshot["epoch"] == status.get("epoch")
            and snapshot.get("elapsed_s", -1) >= duration)


def epoch_snapshot_regression():
    current = {"run_id": "pure-test", "epoch": 1, "ready": True, "error": "", "reset_pending": False,
               "snapshot": {"run_id": "pure-test", "epoch": 0, "elapsed_s": 99.}}
    require(not observation_complete(current, 12.6), "A previous epoch's long observation was accepted after reset")
    current["snapshot"] = {"run_id": "pure-test", "epoch": 1, "elapsed_s": .02}
    require(not observation_complete(current, 12.6), "The new epoch's first frame was mistaken for a complete observation")
    current["snapshot"]["elapsed_s"] = 12.6
    require(observation_complete(current, 12.6), "A current completed epoch was incorrectly rejected")
    return {"passed": True, "kind": "pure_reset_snapshot_scope_not_physics", "old_epoch_elapsed99_rejected": True}


def joint_rest_regression(png):
    """Deliberately inconsistent numerical inputs test the common-window guard."""
    expected = plateau_heights(png, .6)
    rows = []
    for index in range(501):
        probes = []
        for order, (name, xy) in enumerate(dict(TERRAIN_POINTS, **TWIN_POINTS).items()):
            good = index <= 250 if order < 3 else index >= 250
            probes.append({"id": name, "position_m": list(xy)+[expected[name]["center_z_m"] if name in expected else .7],
                           "velocity_m_s": [0., 0., 0. if good else .04]})
        rows.append({"iteration": index, "sim_time_s": 3+index*.001, "elapsed_s": 3+index*.001, "probes": probes})
    try:
        probe_rest_analysis(rows, {"height_m": .6}, png)
    except AssertionError as error:
        require("common" in str(error), "Staggered-rest regression did not specifically exercise the joint stable window")
    else:
        raise AssertionError("Different spheres' disjoint .25s windows were combined into a false common rest pass")
    return {"passed": True, "kind": "pure_numeric_staggered_rest_input_not_physics",
            "each_individual_window_s": .25, "common_interval_s": 0., "disjoint_windows_rejected": True}


def actor_analysis(rows, cycle):
    result = {}
    for name in ("marker", "walker"):
        valid, wraps, distinct, previous = [], [], 0, None
        skipped = 0
        for row in rows:
            item = row["actors"][name]
            t = finite(item["script_time_s"], "Actual native ScriptTime")
            p = pose(item["world_pose"], "Actual Actor world pose")
            if not item["has_animation_frame"]:
                skipped += 1
                require(not valid, "An initialized Actor lost its animation frame within the same reset epoch")
                continue
            require(item["active"] is True, "Actor entered the unsafe inactive path")
            if previous:
                changed = abs(t-previous["script_time_s"]) > 1e-10
                require(item["distinct_update"] is changed, "Repeated observer row was incorrectly called a new Actor animation update")
                if changed:
                    distinct += 1
                if t < previous["script_time_s"]-cycle/2:
                    wraps.append(row["sim_time_s"])
            else:
                distinct += 1
            if name == "marker":
                expected = marker_position(t, cycle)
                for actual, target in zip(p[:3], expected):
                    close(actual, target, "Native marker smoothstep at actual ScriptTime", MARKER_POSITION_TOLERANCE)
                close(abs(p[3]), 1., "Marker actual identity orientation", 1e-5)
                for value in p[4:]:
                    close(value, 0., "Marker actual identity orientation", 1e-5)
            valid.append((row, item))
            previous = item
        require(len(valid) >= 100 and distinct >= 40 and len(wraps) >= 2,
                "Actual observation lacks valid Actor frames, distinct updates, or two complete loop wraps: "+name)
        periods = [right-left for left, right in zip(wraps, wraps[1:])]
        for period in periods:
            close(period, cycle, "Actual world-time interval between native ScriptTime wraps", .08)
        positions = [item["world_pose"][:3] for _, item in valid]
        travel = max(p[0] for p in positions)-min(p[0] for p in positions)
        require(travel >= 1.5, "Actor root did not actually travel over its trajectory: "+name)
        item = {"valid_observer_frames": len(valid), "initial_not_yet_animated_rows": skipped,
                "actual_distinct_ScriptTime_updates": distinct, "actual_wrap_world_times_s": wraps,
                "independent_world_time_periods_s": periods, "actual_horizontal_x_range_m": travel}
        if name == "walker":
            motion = {}
            for foot in ("left_foot_relative_hips", "right_foot_relative_hips"):
                positions = [pose(actor[foot], "Native foot relative to Hips")[:3] for _, actor in valid]
                spread = max(math.sqrt(sum((a-b)**2 for a, b in zip(left, right))) for left in positions for right in positions)
                require(spread >= .03, "Root translation was mistaken for actual relative skeleton motion: "+foot)
                motion[foot] = {"maximum_actual_pairwise_position_change_m": spread,
                                "first_actual_relative_pose": valid[0][1][foot], "last_actual_relative_pose": valid[-1][1][foot]}
            item["independent_relative_bone_motion"] = motion
        result[name] = item
    return result


def configuration_regressions(lesson, private):
    config = load_config(lesson)
    folder = private/"configuration"
    (folder/"assets").mkdir(parents=True)
    world, image = folder/"world.sdf", folder/"assets/heightmap.png"
    original_world = (lesson.folder/"world.sdf").read_bytes()
    original_image = (lesson.folder/"assets/heightmap.png").read_bytes()
    world.write_bytes(original_world)
    image.write_bytes(original_image)
    valid = config.read_world(world, lesson.folder/"world.sdf")
    png = read_numeric_png(original_image)
    plateau_heights(png, valid["height_m"])
    config.read_png(image)
    checks = []
    for name, field, value, diagnosis in (
            ("inactive_bare_actor", "./world/actor[@name='marker']/script/auto_start", "false", "unsafe"),
            ("nonlooping_bare_actor", "./world/actor[@name='marker']/script/loop", "false", "unsafe"),
            ("negative_delay", "./world/actor[@name='marker']/script/delay_start", "-1", "unsafe"),
            ("missing_numeric_heightmap_resource", ".//collision/geometry/heightmap/uri", "file://assets/missing.png", "URI"),
            ("collision_visual_height_disagreement", ".//visual/geometry/heightmap/size", "4 4 .9", "agree")):
        tree = ET.fromstring(original_world)
        tree.find(field).text = value
        world.write_bytes(ET.tostring(tree))
        try:
            config.read_world(world, lesson.folder/"world.sdf")
        except ValueError as error:
            require(diagnosis.lower() in str(error).lower(), "Relevant configuration diagnosis missing: "+name)
            checks.append({"name": name, "diagnosis": str(error)})
        else:
            raise AssertionError("Unsafe or inconsistent current SDF was accepted: "+name)
        finally:
            world.write_bytes(original_world)
    for vertices in (17, 33, 64):
        incompatible, _ = numeric_png_example(vertices=vertices)
        image.write_bytes(incompatible)
        try:
            config.read_png(image)
        except ValueError as error:
            require("65x65" in str(error), "Renderer-incompatible terrain lacks the new prelaunch dimension diagnosis")
            checks.append({"name": "renderer_incompatible_numeric_heightmap_"+str(vertices), "diagnosis": str(error)})
        else:
            raise AssertionError("A terrain with incompatible pinned-renderer dimensions reached later loading")
        finally:
            image.write_bytes(original_image)
    corrupt = bytearray(original_image)
    corrupt[-5] ^= 1
    image.write_bytes(corrupt)
    try:
        config.read_png(image)
    except ValueError as error:
        require("CRC" in str(error), "Corrupt numeric PNG lacks its CRC diagnosis")
        checks.append({"name": "corrupt_numeric_heightmap", "diagnosis": str(error)})
    else:
        raise AssertionError("Corrupt height data was accepted for later Gazebo loading")
    finally:
        image.write_bytes(original_image)
    require(config.read_world(world, lesson.folder/"world.sdf") == valid, "Exact valid personal SDF was not reaccepted")
    require(config.read_png(image)["sha256"] == png["sha256"], "Exact numeric image bytes were not reaccepted")
    return {"passed": True, "kind": "configuration_only_not_physics", "parameters": valid,
            "independent_heightmap_png_sha256": png["sha256"], "checks": checks,
            "exact_world_and_image_bytes_restored": True}


def lab_command(session, *arguments, accepted=True, timeout=28):
    output = control_directory(session)/("l13-"+uuid.uuid4().hex+".client.log")
    wrapper = ("import os,sys;fd=os.open(sys.argv[1],os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600);"
               "os.dup2(fd,1);os.dup2(fd,2);os.close(fd);os.execv(sys.executable,[sys.executable]+sys.argv[2:])")
    process = session._spawn("L13 bounded observation/camera helper", [sys.executable, "-c", wrapper, str(output),
                             str(session.lesson.folder/"lab.py")]+list(arguments))
    return collect_lab(session, {"process": process, "path": output, "started": time.monotonic(),
                                "arguments": list(arguments)}, accepted=accepted, timeout=timeout)


def checked_status(session):
    value = read_json(control_directory(session)/"status.json")
    require(value["run_id"] == session.token and value["world"] == session.world_name,
            "Gallery status came from another actual session or world")
    require(value["ready"] is True and not value["error"], "Actual SDK gallery observer failed: "+str(value.get("error")))
    for key, path in (("world_sha256", session.workspace/"world.sdf"),
                      ("source_sha256", session.lesson.folder/"gallery.cc"),
                      ("client_source_sha256", session.lesson.folder/"gallery_camera.cc"),
                      ("png_sha256", session.workspace/"assets/heightmap.png")):
        require(value[key] == digest(path), "Actual gallery was not reloaded from current source/resource bytes: "+key)
    return value


def await_ready(session):
    deadline, last = time.monotonic()+20, None
    while time.monotonic() < deadline:
        require(session.running, "Gallery server exited before an actual SDK observation")
        try:
            last = checked_status(session)
            if observation_complete(last, 0.):
                check_model(session.workspace, last)
                return last
        except (OSError, ValueError):
            pass
        time.sleep(.05)
    raise AssertionError("Gallery has no current-epoch actual initial frame: "+repr(last))


def independent_observation(session, canonical):
    epoch = canonical["epoch"]
    end_frame = integer(canonical["last_actual_frame"]["frame"], "Canonical final observed frame")
    rows = [row for row in read_jsonl(control_directory(session)/"gallery-frames.jsonl")
            if row["run_id"] == session.token and row["epoch"] == epoch and row["frame"] <= end_frame]
    tail = [row for row in read_jsonl(control_directory(session)/"probe-tail.jsonl")
            if row["run_id"] == session.token and row["epoch"] == epoch]
    require(len(rows) == canonical["samples"] and rows and rows[0] == canonical["first_actual_frame"]
            and rows[-1] == canonical["last_actual_frame"], "Canonical frame range does not match actual retained observations")
    parameters = current_parameters(session.workspace)
    require(rows[0]["elapsed_s"] <= .025 and rows[-1]["elapsed_s"] >= 2.1*parameters["actor_cycle_s"]-1e-8,
            "New actual epoch does not cover initialization and 2.1 real Actor cycles")
    start = rows[0]["sim_time_s"]-rows[0]["elapsed_s"]
    previous = None
    for index, row in enumerate(rows, 1):
        require(integer(row["frame"], "Actual observer frame") == index, "Actual observer frames were skipped or reused")
        close(row["sim_time_s"]-row["elapsed_s"], start, "Actual reset-epoch clock origin", 1e-8)
        if previous:
            close(row["sim_time_s"]-previous["sim_time_s"], .02, "Actual 50Hz frame clock", 1e-8)
            require(integer(row["iteration"], "Actual frame iteration")-integer(previous["iteration"], "Actual frame iteration") == 20,
                    "Observed animation frame cadence does not correspond to 20 real physics steps")
        previous = row
    png = read_numeric_png((session.workspace/"assets/heightmap.png").read_bytes())
    levels = plateau_heights(png, parameters["height_m"])
    for item in rows[0]["probes"]:
        close(item["position_m"][2], 2., "Native reset returned actual falling sphere to its initial height", .001)
    rays = []
    for row in rows:
        if not 3.-1e-8 <= row["elapsed_s"] <= 3.5+1e-8:
            continue
        actual = {item["id"]: item for item in row["rays"]}
        require(set(actual) == set(TERRAIN_POINTS), "Actual collision ray omitted a terrain quadrant")
        for name, expected in levels.items():
            ray = actual[name]
            require(ray["entity"] == canonical["actual_model"]["heightmap"]["collision_name"],
                    "A visual surface, ball or other floor was mistaken for actual terrain collision")
            close(ray["x_m"], expected["ray_xy_m"][0], "Actual offset ray X")
            close(ray["y_m"], expected["ray_xy_m"][1], "Actual offset ray Y")
            close(ray["z_m"], expected["surface_z_m"], "Actual ODE collision height independently predicted from PNG", RAY_HEIGHT_TOLERANCE)
        rays.append(row["rays"])
    require(len(rays) >= 20, "No sufficiently sampled independent terrain collision query window")
    return {"epoch": epoch, "actual_animation_observer_frames": len(rows), "actual_duration_s": rows[-1]["elapsed_s"],
            "independent_rest": probe_rest_analysis(tail, parameters, png),
            "independent_actor_motion": actor_analysis(rows, parameters["actor_cycle_s"]),
            "actual_collision_ray_frames": len(rays), "first_ray_frame": rays[0], "last_ray_frame": rays[-1],
            "initial_has_animation_frame": {name: rows[0]["actors"][name]["has_animation_frame"] for name in ("marker", "walker")},
            "visual_material_or_lighting_objectives_graded_by_headless_run": False}


def verify_case(session, name, diagnosis=None):
    before = actual_stats(session)
    personal = source_hashes(session.workspace)
    previous = read_json(control_directory(session)/"status.json")
    offset = session.log_path.stat().st_size
    hashes = {filename: digest(control_directory(session)/filename) for filename in ("gallery-frames.jsonl", "probe-tail.jsonl")}
    result = session.verify()
    output = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
    after = actual_stats(session)
    require(before["paused"] == after["paused"], "Gallery verifier did not restore the original actual pause state")
    require(personal == source_hashes(session.workspace), "Gallery verifier overwrote personal SDF/resources")
    item = {"name": name, "verifier_result": result, "actual_before_stats": before, "actual_after_stats": after,
            "raw_verifier_output": output, "pause_preserved": True, "personal_files_unchanged": True}
    if diagnosis:
        require(before["paused"] and result["passed"] is False and diagnosis.lower() in output.lower(),
                "Invalid/stale gallery input lacks its relevant refusal: "+diagnosis+"\n"+output)
        current = read_json(control_directory(session)/"status.json")
        require(before["iterations"] == after["iterations"] and before["sim_time_sec_nsec"] == after["sim_time_sec_nsec"],
                "Invalid gallery inputs advanced physics before being refused")
        require(current["epoch"] == previous["epoch"] and current["reset_count"] == previous["reset_count"],
                "Invalid input reset the scene or started a new observation")
        require(hashes == {filename: digest(control_directory(session)/filename) for filename in hashes},
                "Invalid input appended new physical measurements")
        item["rejected_before_reset_or_physics"] = True
    else:
        require(result["passed"] is True, "Canonical actual scene verification failed:\n"+output)
        canonical = read_json(control_directory(session)/"verification.json")
        require(canonical["epoch"] > previous["epoch"] and canonical["world_sha256"] == digest(session.workspace/"world.sdf")
                and canonical["png_sha256"] == digest(session.workspace/"assets/heightmap.png")
                and canonical["parameters"] == current_parameters(session.workspace),
                "Gallery verification reused old epoch frames or stale personal input")
        model_status = dict(checked_status(session), model=canonical["actual_model"])
        item.update(canonical_measurements=canonical, independent_model=check_model(session.workspace, model_status),
                    independent_actual_observation=independent_observation(session, canonical))
    item["passed"] = True
    return item


def simple_png(width, height, level=0, original=None):
    """Numerical format inputs used only for explicit private fault injection."""
    def chunk(kind, body):
        return struct.pack(">I", len(body))+kind+body+struct.pack(">I", zlib.crc32(kind+body) & 0xffffffff)
    if original is None:
        decoded = bytes(value for _ in range(height) for value in [0]+[0 if x < width//2 else 255 for x in range(width)])
    else:
        pixels = read_numeric_png(original)["pixels"]
        decoded = b"".join(bytes([0]+row) for row in pixels)
    return (PNG_SIGNATURE+chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            +chunk(b"IDAT", zlib.compress(decoded, level))+chunk(b"IEND", b""))


def invalid_and_stale_inputs(session):
    session.pause(True)
    world, png = session.workspace/"world.sdf", session.workspace/"assets/heightmap.png"
    original_world, original_png = world.read_bytes(), png.read_bytes()
    results = []
    for name, xpath, value, diagnosis in (
            ("unsafe_no_skin_actor_inactive", "./world/actor[@name='marker']/script/auto_start", "false", "unsafe"),
            ("missing_heightmap_URI", ".//collision/geometry/heightmap/uri", "file://assets/missing.png", "URI")):
        try:
            tree = ET.fromstring(original_world)
            tree.find(xpath).text = value
            world.write_bytes(ET.tostring(tree))
            results.append(verify_case(session, name, diagnosis))
        finally:
            world.write_bytes(original_world)
    try:
        tree = ET.fromstring(original_world)
        link = tree.find("./world/model[@name='gallery_floor']/link")
        collision = ET.SubElement(link, "collision", name="forbidden_hidden_floor")
        plane = ET.SubElement(ET.SubElement(collision, "geometry"), "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "14 12"
        world.write_bytes(ET.tostring(tree))
        results.append(verify_case(session, "hidden_collision_floor_changes_fixed_topology", "topology"))
    finally:
        world.write_bytes(original_world)
    for name, data, diagnosis in (("corrupted_numeric_PNG", bytes(bytearray(original_png[:-1])+bytes([original_png[-1] ^ 1])), "CRC"),
                                   ("wrong_64_square_heightmap_dimensions", simple_png(64, 64), "65x65")):
        try:
            png.write_bytes(data)
            results.append(verify_case(session, name, diagnosis))
        finally:
            png.write_bytes(original_png)
    try:
        config = load_config(session.lesson)
        config.edit_world(world, session.lesson.folder/"world.sdf", "daylight", .2)
        results.append(verify_case(session, "valid_daylight_saved_without_reloading", "reload"))
    finally:
        world.write_bytes(original_world)
    try:
        different_bytes = simple_png(HEIGHTMAP_VERTICES, HEIGHTMAP_VERTICES, original=original_png)
        require(different_bytes != original_png and read_numeric_png(different_bytes)["pixels"] == read_numeric_png(original_png)["pixels"],
                "Equivalent numeric PNG test did not actually change only its encoded bytes")
        png.write_bytes(different_bytes)
        results.append(verify_case(session, "valid_equivalent_PNG_bytes_saved_without_reloading", "reload"))
    finally:
        png.write_bytes(original_png)
    require(world.read_bytes() == original_world and png.read_bytes() == original_png,
            "Exact personal SDF/PNG bytes were not restored after fault injection")
    return {"name": "invalid_resources_unsafe_flags_topology_and_stale_inputs_refused_before_control", "passed": True,
            "checks": results, "exact_personal_source_and_resource_bytes_restored": True}


def public_reset_probe(session):
    session.pause(True)
    before_stats, before = actual_stats(session), checked_status(session)
    personal = source_hashes(session.workspace)
    session.reset_models()
    reset_stats = actual_stats(session)
    require(reset_stats["paused"] and reset_stats["iterations"] >= before_stats["iterations"]
            and reset_stats["sim_time"] >= before_stats["sim_time"], "Public model reset rewound world time or changed pause")
    steps = bounded_steps(session, 200, time.monotonic()+35)
    current = checked_status(session)
    require(current["epoch"] == before["epoch"]+1 and current["reset_count"] == before["reset_count"]+1
            and observation_complete(current, 0.), "Model reset reused an old snapshot or failed to create a live new epoch")
    rows = [row for row in read_jsonl(control_directory(session)/"gallery-frames.jsonl")
            if row["run_id"] == session.token and row["epoch"] == current["epoch"]]
    require(rows and rows[0]["elapsed_s"] <= .025, "Public reset has no actual new initial frame")
    for item in rows[0]["probes"]:
        close(item["position_m"][2], 2., "Public model reset actual sphere initial state", .001)
    last = current["snapshot"]
    for name in ("marker", "walker"):
        actor = last["actors"][name]
        require(actor["active"] is True and actor["has_animation_frame"] is True
                and 0 <= actor["script_time_s"] <= .3, "Public reset did not safely restart actual native Actor playback")
    marker = last["actors"]["marker"]
    for actual, expected in zip(marker["world_pose"][:3], marker_position(marker["script_time_s"], current["parameters"]["actor_cycle_s"])):
        close(actual, expected, "Actual marker path after public model reset", MARKER_POSITION_TOLERANCE)
    require(personal == source_hashes(session.workspace), "Public reset silently restored or changed personal files")
    return {"name": "public_model_reset_restarts_actual_Actors_and_probes_without_time_rewind", "passed": True,
            "actual_before_stats": before_stats, "actual_after_reset_stats": reset_stats,
            "owned_200_step_acknowledgement": steps, "old_epoch": before["epoch"], "new_epoch": current["epoch"],
            "first_actual_new_frame": rows[0], "actual_restarted_Actors": last["actors"],
            "test_writes_actor_or_probe_pose_velocity": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--math-only", action="store_true")
    group.add_argument("--configuration-only", action="store_true")
    args = parser.parse_args()
    report = {"passed": False, "lesson": KEY, "physical_sessions_started": 0, "scenarios": [],
              "math_regressions": math_regressions(), "visual_objectives_verified_by_headless_run": False}
    output = Path(args.report).expanduser().resolve() if args.report else None
    artifacts = output.with_name(output.stem+"-artifacts") if output else Path(tempfile.mkdtemp(prefix="gazeboteach-13-evidence-"))
    sessions, lesson, source_before = [], None, None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-13-private-") as folder:
        private = Path(folder)
        try:
            if not args.math_only:
                lessons, errors = load_catalog(ROOT)
                found = [candidate for candidate in lessons if candidate.key == KEY]
                require(len(found) == 1, "Expected the current L13 course: "+repr(errors))
                lesson = found[0]
                source_before = source_hashes(lesson.folder)
                report["configuration_regressions"] = configuration_regressions(lesson, private)
                report["explicit_edit_regressions"] = edit_regressions(lesson, private)
            if not args.math_only and not args.configuration_only:
                baseline = LessonSession(ROOT, lesson, storage=Storage(private/"baseline"))
                sessions.append(("baseline", baseline))
                report["physical_sessions_started"] += 1
                baseline.start(gui=False, terminals=False)
                report["baseline_initial_status"] = await_ready(baseline)
                missing_gui = lab_command(baseline, "view", "--station", "overview", "--timeout", "1", accepted=False, timeout=15)
                require("GUI not connected" in missing_gui["combined_stdout_stderr"],
                        "Headless camera request did not provide its bounded no-GUI diagnosis")
                report["bounded_no_GUI_camera_diagnosis"] = missing_gui
                baseline.pause(False)
                report["scenarios"].append(verify_case(baseline, "actual_baseline_terrain_material_twins_and_two_native_Actor_cycles"))
                baseline.stop(reason="L13 baseline accepted")
                storage = Storage(private/"height_only")
                workspace = storage.prepare(lesson)
                load_config(lesson).edit_world(workspace/"world.sdf", lesson.folder/"world.sdf", "height", .9)
                higher = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("height_only", higher))
                report["physical_sessions_started"] += 1
                higher.start(gui=False, terminals=False)
                report["height_initial_status"] = await_ready(higher)
                higher.pause(True)
                report["scenarios"].append(verify_case(higher, "only_height_changed_point6_to_point9_actual_collision_and_ball_centers"))
                old = report["scenarios"][0]["independent_actual_observation"]["independent_rest"]["last_actual_snapshot"]["probes"]
                new = report["scenarios"][1]["independent_actual_observation"]["independent_rest"]["last_actual_snapshot"]["probes"]
                old, new = {p["id"]: p for p in old}, {p["id"]: p for p in new}
                require(new["probe_se"]["position_m"][2]-old["probe_se"]["position_m"][2] > .28,
                        "Changing actual heightfield size did not raise the highest physical probe")
                for name in TWIN_POINTS:
                    close(new[name]["position_m"][2], old[name]["position_m"][2], "Terrain parameter preserved material-twin support physics", .01)
                higher.stop(reason="L13 height-only variant accepted")
                storage = Storage(private/"cycle_only")
                workspace = storage.prepare(lesson)
                load_config(lesson).edit_world(workspace/"world.sdf", lesson.folder/"world.sdf", "cycle", 8.)
                slower = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(("cycle_only", slower))
                report["physical_sessions_started"] += 1
                slower.start(gui=False, terminals=False)
                report["cycle_initial_status"] = await_ready(slower)
                slower.pause(True)
                report["scenarios"].append(verify_case(slower, "only_Actor_cycle_changed_six_to_eight_actual_script_wraps"))
                report["scenarios"].append(invalid_and_stale_inputs(slower))
                report["scenarios"].append(verify_case(slower, "exact_repaired_bytes_reaccepted_with_new_epoch_full_observation"))
                report["scenarios"].append(public_reset_probe(slower))
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"], report["cleanup_errors"] = {}, []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L13 private headless acceptance finished")
                except Exception as error:
                    report["cleanup_errors"].append(name+": "+repr(error))
                try:
                    report["evidence"][name] = preserve_evidence(session, artifacts/name)
                except Exception as error:
                    report["cleanup_errors"].append(name+" evidence: "+repr(error))
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers, source_templates_unchanged=(source_before == source_hashes(lesson.folder)) if lesson else None,
                          elapsed_seconds=round(time.monotonic()-started, 3), scenario_count=len(report["scenarios"]))
            if any(leftovers.values()) or report["cleanup_errors"] or report["source_templates_unchanged"] is False:
                report["passed"] = False
            if output:
                output.parent.mkdir(parents=True, exist_ok=True)
                output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
