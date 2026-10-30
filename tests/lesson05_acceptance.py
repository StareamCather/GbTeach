"""L05 saved resources, live geometry, physical support, and safe lab switching.

Fresh temporary workspaces keep all edits and simulator sessions private. Each
positive case queries actual Gazebo Model messages, drops the real dynamic
table_probe onto the current table, then runs the canonical source verifier.
The verifier must preserve resource hashes, static poses, and pause state.
Optional editor exports are copied verbatim, selected by lab.py, and reloaded in
a new real server; this verifies saved artifacts, not the user's mouse skills.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes


def asset_hashes(folder):
    paths = [folder / "world.sdf"] + sorted((folder / "models").rglob("*"))
    return {path.relative_to(folder).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths if path.is_file()}


def command(session, *arguments, read_only=False):
    """Only explicit read queries may retry once after a transport timeout."""
    for attempt in range(2 if read_only else 1):
        try:
            result = subprocess.run(list(arguments), cwd=str(session.workspace), env=session.env,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=4)
        except subprocess.TimeoutExpired:
            if read_only and attempt == 0:
                continue
            raise
        if result.returncode:
            raise AssertionError("Command failed: {}\n{}".format(" ".join(arguments), result.stderr[-1800:]))
        return result.stdout


def actual_paused(session):
    output = command(session, "gz", "topic", "-e", "/gazebo/{}/world_stats".format(session.world_name),
                     "-d", "1", read_only=True)
    values = re.findall(r"\bpaused:\s*(true|false)\b", output)
    if not values:
        raise AssertionError("No actual pause state received from this world's statistics")
    return values[-1] == "true"


def actual_pose(session, name):
    output = command(session, "gz", "model", "-w", session.world_name, "-m", name, "-p", read_only=True)
    for line in reversed(output.splitlines()):
        try:
            values = [float(value) for value in line.split()]
        except ValueError:
            continue
        if len(values) == 6 and all(math.isfinite(value) for value in values):
            return values
    raise AssertionError("No live pose for {}: {}".format(name, output))


def parsed_model(output):
    """Read nested DebugString fields without mixing link and shape geometry."""
    root = {}
    stack = [root]
    for raw in output.splitlines():
        line = raw.strip()
        if not line:
            continue
        opened = re.fullmatch(r"([A-Za-z_][A-Za-z_0-9]*)\s*\{", line)
        if opened:
            value = {}
            stack[-1].setdefault(opened.group(1), []).append(value)
            stack.append(value)
        elif line == "}":
            if len(stack) == 1:
                raise AssertionError("Unexpected closing brace in live Model message")
            stack.pop()
        else:
            field = re.fullmatch(r"([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(.+)", line)
            if not field:
                raise AssertionError("Unrecognized live Model message field: " + line)
            value = field.group(2)
            if value.startswith('"'):
                value = json.loads(value)
            stack[-1].setdefault(field.group(1), []).append(value)
    if len(stack) != 1:
        raise AssertionError("Incomplete live Model message")
    return root


def first(message, field, default=None):
    values = message.get(field, [])
    return values[0] if values else default


def short_name(value):
    return str(value).split("::")[-1]


def selected_packages(workspace):
    world = ET.parse(str(workspace / "world.sdf")).getroot().find("world")
    result = {}
    for entry in world.findall("include"):
        name = entry.findtext("name")
        if name not in ("table_example", "room_example"):
            continue
        uri = entry.findtext("uri", "")
        if not uri.startswith("model://") or "/" in uri[len("model://"):]:
            raise AssertionError("Expected a local one-folder model URI: " + uri)
        folder = workspace / "models" / uri[len("model://"):]
        config = ET.parse(str(folder / "model.config")).getroot()
        entries = sorted(config.findall("sdf"), key=lambda node: tuple(int(x) for x in node.get("version", "1.4").split(".")))
        if not entries:
            raise AssertionError("No SDF entry in model.config")
        path = (folder / entries[-1].text.strip()).resolve()
        if folder.resolve() not in path.parents:
            raise AssertionError("Model entry escapes its package folder")
        model = ET.parse(str(path)).getroot().find("model")
        if model is None:
            raise AssertionError("Saved model file has no model element")
        result[name] = {"uri": uri, "folder": folder, "path": path, "model": model,
                        "pose": [float(x) for x in entry.findtext("pose", "0 0 0 0 0 0").split()]}
    if set(result) != {"table_example", "room_example"}:
        raise AssertionError("Both documented table and room instances must be present")
    return result


def live_geometry(session):
    evidence = {}
    for name, package in selected_packages(session.workspace).items():
        message = parsed_model(command(session, "gz", "model", "-w", session.world_name,
                                       "-m", name, "-i", read_only=True))
        links = {short_name(first(link, "name")): link for link in message.get("link", [])}
        observed = []
        for source_link in package["model"].findall("link"):
            link_name = source_link.get("name")
            if link_name not in links:
                raise AssertionError("Loaded {} lacks saved link {}".format(name, link_name))
            link = links[link_name]
            for kind in ("collision", "visual"):
                actual_shapes = {short_name(first(shape, "name")): shape for shape in link.get(kind, [])}
                for shape in source_link.findall(kind):
                    box = shape.find("geometry/box/size")
                    if box is None:
                        continue
                    shape_name = shape.get("name")
                    actual = actual_shapes.get(shape_name)
                    if actual is None:
                        raise AssertionError("Live {} {} lacks shape {}".format(name, link_name, shape_name))
                    geometry = first(actual, "geometry", {})
                    size = first(first(geometry, "box", {}), "size", {})
                    measured = [float(first(size, axis, "nan")) for axis in ("x", "y", "z")]
                    expected = [float(x) for x in box.text.split()]
                    if len(expected) != 3 or any(not math.isfinite(a) or abs(a - b) > 1e-5
                                                 for a, b in zip(measured, expected)):
                        raise AssertionError("Live box dimensions differ from saved {} {} {}: {} != {}".format(
                            name, link_name, shape_name, measured, expected))
                    observed.append({"link": link_name, "kind": kind, "shape": shape_name,
                                     "actual_size": measured, "source_size": expected})
        if len([item for item in observed if item["kind"] == "collision"]) < 2:
            raise AssertionError("The saved composed model needs multiple actual collision shapes")
        evidence[name] = {"uri": package["uri"], "live_static_fields": {
            key: message[key] for key in ("is_static", "static") if key in message}, "boxes": observed}
    return evidence


def physical_table_drop(session, expected_z):
    """Observe a real fall; a server starting successfully cannot satisfy it."""
    original_paused = actual_paused(session)
    table = selected_packages(session.workspace)["table_example"]["pose"]
    raised_z = expected_z + 1.2
    samples = []
    try:
        session.pause(True)
        command(session, "gz", "model", "-w", session.world_name, "-m", "table_probe",
                "-x", str(table[0]), "-y", str(table[1]), "-z", str(raised_z),
                "-R", "0", "-P", "0", "-Y", "0")
        raised = actual_pose(session, "table_probe")
        if abs(raised[2] - raised_z) > 0.02:
            raise AssertionError("Dynamic probe did not actually move above the table")
        session.pause(False)
        deadline = time.monotonic() + 12
        previous = None
        stable = 0
        while time.monotonic() < deadline:
            measured = actual_pose(session, "table_probe")
            samples.append(measured)
            close = abs(measured[2] - expected_z) < 0.025
            stable = stable + 1 if close and previous is not None and abs(measured[2] - previous[2]) < 0.003 else 0
            if stable >= 2:
                return {"raised_pose": raised, "expected_support_center_z": expected_z,
                        "actual_settled_pose": measured, "actual_pose_samples": samples}
            previous = measured
        raise AssertionError("Probe did not settle onto the saved table surface: " + repr(samples))
    finally:
        if session.running:
            session.pause(original_paused)


def verify_case(session, name, should_pass=True, expected_z=None):
    before_assets = asset_hashes(session.workspace)
    before_paused = actual_paused(session)
    before_poses = {name: actual_pose(session, name) for name in ("table_example", "room_example")}
    offset = session.log_path.stat().st_size
    item = {"name": name, "expected_verifier_pass": should_pass, "original_paused": before_paused,
            "poses_before": before_poses}
    try:
        result = session.verify()
        item["verifier_result"] = result
        item["evidence"] = session.log_path.read_bytes()[offset:].decode("utf-8", errors="replace")
        if result.get("passed") is not should_pass:
            raise AssertionError("Unexpected canonical verifier result for " + name)
        after_paused = actual_paused(session)
        item.update(actual_paused_after=after_paused, pause_restored=after_paused == before_paused)
        if after_paused != before_paused:
            raise AssertionError("L05 verifier changed the original pause state")
        item["assets_unchanged"] = before_assets == asset_hashes(session.workspace)
        item["asset_hashes"] = before_assets
        if not item["assets_unchanged"]:
            raise AssertionError("Verifier modified the world or saved models")
        after_poses = {name: actual_pose(session, name) for name in before_poses}
        item["poses_after"] = after_poses
        if any(max(abs(a - b) for a, b in zip(before_poses[name], after_poses[name])) > 0.025
               for name in before_poses):
            raise AssertionError("Verifier changed a static saved-model instance pose")
        if should_pass:
            item["live_saved_geometry"] = live_geometry(session)
            if expected_z is not None:
                item["independent_physical_drop"] = physical_table_drop(session, expected_z)
        elif "模型入口文件不存在" not in item["evidence"] or "model_missing.sdf" not in item["evidence"]:
            raise AssertionError("Missing package entry lacks the clear expected diagnosis")
        item["passed"] = True
        return item
    finally:
        if session.running and actual_paused(session) != before_paused:
            session.pause(before_paused)


def set_shape(link, center, size):
    pose = link.find("pose")
    if pose is None:
        pose = ET.SubElement(link, "pose")
    pose.text = " ".join(str(x) for x in list(center) + [0, 0, 0])
    for kind in ("collision", "visual"):
        shapes = link.findall(kind)
        if len(shapes) != 1 or shapes[0].find("geometry/box/size") is None:
            raise AssertionError("Reference challenge expects one box per link")
        shapes[0].find("geometry/box/size").text = " ".join(str(x) for x in size)


def challenge_files(workspace):
    packages = selected_packages(workspace)
    table = packages["table_example"]
    links = table["model"].findall("link")
    if len(links) != 5:
        raise AssertionError("Reference table must have one top and four legs")
    top = max(links, key=lambda link: float(link.findtext("collision/geometry/box/size").split()[0]))
    set_shape(top, (0, 0, 1.1), (1.5, 1.0, 0.2))
    for link in links:
        if link is top:
            continue
        original = [float(x) for x in link.findtext("pose", "0 0 0 0 0 0").split()]
        set_shape(link, (math.copysign(0.65, original[0]), math.copysign(0.4, original[1]), 0.5), (0.1, 0.1, 1.0))
    root = ET.Element("sdf", {"version": "1.6"})
    root.append(table["model"])
    ET.ElementTree(root).write(str(table["path"]), encoding="utf-8", xml_declaration=True)

    room = packages["room_example"]
    changed = []
    for link in room["model"].findall("link"):
        center = [float(x) for x in link.findtext("pose", "0 0 0 0 0 0").split()][:3]
        size = [float(x) for x in link.findtext("collision/geometry/box/size").split()]
        if abs(center[1] + 1.25) > 1e-5:
            continue
        if abs(center[0]) < 0.1:
            set_shape(link, (0, -1.25, 1.7), (1.2, 0.12, 0.6))
        else:
            set_shape(link, (math.copysign(1.05, center[0]), -1.25, 1.0), (0.9, 0.12, 2.0))
        changed.append(link.get("name"))
    if len(changed) != 3:
        raise AssertionError("Expected both front wall segments and the door lintel")
    root = ET.Element("sdf", {"version": "1.6"})
    root.append(room["model"])
    ET.ElementTree(root).write(str(room["path"]), encoding="utf-8", xml_declaration=True)
    return {"expected_probe_center_z": 1.25, "expected_door_width": 1.2, "changed_front_links": changed}


def lab_command(workspace, action, expected_success):
    result = subprocess.run([sys.executable, str(workspace / "lab.py"), action], cwd=str(workspace),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10)
    if (result.returncode == 0) is not expected_success:
        raise AssertionError("Unexpected lab.py {} result: {}\n{}\n{}".format(
            action, result.returncode, result.stdout, result.stderr))
    return {"action": action, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}


def rejected_empty_exports(workspace):
    before = asset_hashes(workspace)
    result = lab_command(workspace, "use-exports", False)
    if before != asset_hashes(workspace):
        raise AssertionError("Rejected export switching changed saved resources or world")
    result.update(name="missing_student_exports_rejected_without_modification", assets_unchanged=True, passed=True)
    return result


def centered_export_variants(workspace):
    """Create declared synthetic export variants; these are not GUI evidence."""
    inputs = (("reference_table", "student_table", 0.46), ("reference_room", "student_room", 1.0))
    for reference, student, origin_z in inputs:
        target = workspace / "models" / student
        shutil.copytree(str(workspace / "models" / reference), str(target))
        path = target / "model.sdf"
        tree = ET.parse(str(path))
        model = tree.getroot().find("model")
        model.set("name", student)
        if student == "student_table":
            for number, link in enumerate(model.findall("link")):
                link.set("name", "saved_part_{}".format(number))
                pose = link.find("pose")
                if pose is None:
                    pose = ET.SubElement(link, "pose")
                    pose.text = "0 0 0 0 0 0"
                values = [float(x) for x in pose.text.split()]
                values[2] -= origin_z
                pose.text = " ".join(str(x) for x in values)
        else:
            # A native export may put multiple independent boxes in one link.
            combined = ET.Element("link", {"name": "saved_walls"})
            for number, link in enumerate(list(model.findall("link"))):
                values = [float(x) for x in link.findtext("pose", "0 0 0 0 0 0").split()]
                if max(abs(x) for x in values[3:]) > 1e-9:
                    raise AssertionError("Reference wall segmentation assumes zero wall rotations")
                for kind in ("collision", "visual"):
                    for shape in link.findall(kind):
                        exported = copy.deepcopy(shape)
                        exported.set("name", "saved_{}_{}".format(kind, number))
                        pose = exported.find("pose")
                        if pose is None:
                            pose = ET.SubElement(exported, "pose")
                            pose.text = "0 0 0 0 0 0"
                        local = [float(x) for x in pose.text.split()]
                        local[:3] = [a + b for a, b in zip(local[:3], values[:3])]
                        local[2] -= origin_z
                        pose.text = " ".join(str(x) for x in local)
                        combined.append(exported)
                model.remove(link)
            model.append(combined)
        tree.getroot().set("version", "1.4")
        tree.write(str(path), encoding="utf-8", xml_declaration=True)
        config_path = target / "model.config"
        config = ET.parse(str(config_path))
        config.getroot().find("name").text = student
        for entry in config.getroot().findall("sdf"):
            entry.set("version", "1.4")
            entry.text = "model.sdf"
        config.write(str(config_path), encoding="utf-8", xml_declaration=True)
    # A valid table plus invalid room detects a non-atomic implementation that
    # writes the first include before validating the second package.
    room_config = workspace / "models/student_room/model.config"
    original_config = room_config.read_bytes()
    try:
        broken = ET.parse(str(room_config))
        for entry in broken.getroot().findall("sdf"):
            entry.text = "model_missing.sdf"
        broken.write(str(room_config), encoding="utf-8", xml_declaration=True)
        before_rejection = asset_hashes(workspace)
        rejected = lab_command(workspace, "use-exports", False)
        if "模型入口文件不存在" not in rejected["stdout"] + rejected["stderr"]:
            raise AssertionError("Invalid second export package lacks a missing-entry diagnosis")
        if before_rejection != asset_hashes(workspace):
            raise AssertionError("Partial export validation failure modified the world or saved packages")
        rejected["world_and_saved_packages_unchanged"] = True
    finally:
        room_config.write_bytes(original_config)
    before_models = {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}
    switch = lab_command(workspace, "use-exports", True)
    after_models = {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}
    if before_models != after_models:
        raise AssertionError("Lab export selection rewrote saved model resources")
    packages = selected_packages(workspace)
    for instance, expected_z in (("table_example", 0.46), ("room_example", 1.0)):
        if abs(packages[instance]["pose"][2] - expected_z) > 1e-5:
            raise AssertionError("Lab did not compensate the centered saved origin of " + instance)
    return {"artifact_origin": "synthetic_export_shape_variants_not_GUI_interaction",
            "sdf_version": "1.4", "table_origin_offset": 0.46, "room_origin_offset": 1.0,
            "invalid_second_package_rejected_atomically": rejected,
            "selection_after_repair": switch, "saved_resources_unchanged_by_selection": True,
            "include_poses": {name: item["pose"] for name, item in packages.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    parser.add_argument("--export-table", help="Actual Model Editor export folder; copied, never edited")
    parser.add_argument("--export-room", help="Actual Building Editor export folder; copied, never edited")
    args = parser.parse_args()
    if bool(args.export_table) != bool(args.export_room):
        parser.error("Provide both --export-table and --export-room")
    lessons, errors = load_catalog(ROOT)
    matches = [lesson for lesson in lessons if lesson.key == "classic11-noetic:05"]
    if len(matches) != 1:
        raise RuntimeError("Expected one L05 course; catalog errors: " + repr(errors))
    lesson = matches[0]
    source_assets = asset_hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "scenarios": [],
              "editor_mouse_workflow_verified_here": False}
    sessions = []
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l05-") as directory:
        private = Path(directory)
        try:
            storage = Storage(private / "baseline")
            workspace = storage.prepare(lesson)
            report["scenarios"].append(rejected_empty_exports(workspace))
            baseline = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(baseline)
            baseline.start(gui=False, terminals=False)
            baseline.pause(False)
            report["scenarios"].append(verify_case(baseline, "reference_saved_models_actual_geometry_and_support", expected_z=0.9))
            baseline.stop(reason="L05 reference resources accepted")

            storage = Storage(private / "challenge")
            workspace = storage.prepare(lesson)
            variants = challenge_files(workspace)
            challenge = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(challenge)
            challenge.start(gui=False, terminals=False)
            challenge.pause(True)
            item = verify_case(challenge, "changed_table_height_and_wider_real_doorway", expected_z=variants["expected_probe_center_z"])
            item["challenge"] = variants
            report["scenarios"].append(item)
            config_path = selected_packages(workspace)["table_example"]["folder"] / "model.config"
            valid = config_path.read_bytes()
            try:
                tree = ET.parse(str(config_path))
                for entry in tree.getroot().findall("sdf"):
                    entry.text = "model_missing.sdf"
                tree.write(str(config_path), encoding="utf-8", xml_declaration=True)
                report["scenarios"].append(verify_case(challenge, "missing_saved_model_entry_rejected", False))
            finally:
                config_path.write_bytes(valid)
            report["scenarios"].append(verify_case(challenge, "repaired_saved_model_entry_reaccepted", expected_z=1.25))
            challenge.stop(reason="L05 resource repair accepted")

            storage = Storage(private / "centered-exports")
            workspace = storage.prepare(lesson)
            centered = centered_export_variants(workspace)
            adjusted = LessonSession(ROOT, lesson, storage=storage)
            sessions.append(adjusted)
            adjusted.start(gui=False, terminals=False)
            adjusted.pause(True)
            item = verify_case(adjusted, "centered_sdf14_exports_renamed_links_and_wall_segmentation", expected_z=0.9)
            item["synthetic_saved_export_variants"] = centered
            report["scenarios"].append(item)
            adjusted.stop(reason="L05 saved origin compensation accepted")
            before_models = {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}
            restored = lab_command(workspace, "use-reference", True)
            if before_models != {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}:
                raise AssertionError("Reference switching overwrote synthetic student exports")
            packages = selected_packages(workspace)
            if any(packages[name]["uri"] != "model://" + package for name, package in
                   (("table_example", "reference_table"), ("room_example", "reference_room"))):
                raise AssertionError("Reference switching did not restore reference package selection")
            restored.update(name="reference_switch_preserves_saved_export_artifacts", saved_resources_unchanged=True, passed=True)
            report["scenarios"].append(restored)

            if args.export_table:
                storage = Storage(private / "editor-exports")
                workspace = storage.prepare(lesson)
                inputs = {"student_table": Path(args.export_table).expanduser().resolve(),
                          "student_room": Path(args.export_room).expanduser().resolve()}
                original = {}
                for name, source in inputs.items():
                    original[name] = {path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                                      for path in source.rglob("*") if path.is_file()}
                    shutil.copytree(str(source), str(workspace / "models" / name))
                switch = lab_command(workspace, "use-exports", True)
                export_assets = asset_hashes(workspace)
                exported = LessonSession(ROOT, lesson, storage=storage)
                sessions.append(exported)
                exported.start(gui=False, terminals=False)
                exported.pause(True)
                item = verify_case(exported, "actual_editor_saved_packages_selected_and_reloaded")
                item["switch"] = switch
                item["artifact_origin"] = {name: str(path) for name, path in inputs.items()}
                if export_assets != asset_hashes(workspace):
                    raise AssertionError("Editor export artifacts changed during verification")
                item["original_exports_unchanged"] = all(original[name] == {
                    path.relative_to(source).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in source.rglob("*") if path.is_file()} for name, source in inputs.items())
                if not item["original_exports_unchanged"]:
                    raise AssertionError("External editor export files were modified")
                report["scenarios"].append(item)
                exported.stop(reason="L05 actual editor exports reloaded")
                before_models = {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}
                switch_back = lab_command(workspace, "use-reference", True)
                after_models = {key: value for key, value in asset_hashes(workspace).items() if key.startswith("models/")}
                if before_models != after_models:
                    raise AssertionError("Switching back overwrote a student model package")
                switch_back.update(name="switch_back_preserves_student_exports", passed=True)
                report["scenarios"].append(switch_back)
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["log_tails"] = [session.log_path.read_text(encoding="utf-8", errors="replace")[-30000:]
                                   for session in sessions if session.log_path.exists()]
        finally:
            for session in reversed(sessions):
                session.stop(reason="L05 private acceptance finished")
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(0.05)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            report.update(leftovers=leftovers, source_assets_unchanged=asset_hashes(lesson.folder) == source_assets,
                          elapsed_seconds=round(time.monotonic() - started, 3))
            if any(leftovers.values()) or not report["source_assets_unchanged"]:
                report["passed"] = False
            if args.report:
                path = Path(args.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
