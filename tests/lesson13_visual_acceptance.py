#!/usr/bin/env python3
"""Capture only owned real Gazebo windows after actual camera-pose readback.

With --case all, three private GUI sessions compare baseline, daylight-only .2,
and light shadow enable-only false. The default captures baseline only. The program checks camera transport, source isolation and
captures; material/light/shadow appearance remains an explicit human review.
--math-only needs no GUI and starts no Gazebo.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson091_acceptance import control_directory, digest, require, source_hashes
from tests.lesson10_acceptance import bounded_steps
from tests.lesson12_acceptance import preserve_evidence
from tests.lesson13_acceptance import (KEY, await_ready, checked_status, current_parameters, lab_command,
                                       load_config, pose, semantic_world, xml_tree)
from tests.ui_probe import gazebo_window_id

PRESETS = {"overview": [8., -10., 9., 0., .62, 2.18],
           "materials": [6.8, -3.5, 3.1, 0., .4, 2.35],
           "specular": [5.8, 4.3, 2.5, 0., .6, -2.25],
           "lights": [-6.8, -5., 4., 0., .5, .98],
           "terrain": [5., -6., 7., 0., .72, 2.26],
           "actors": [3., -7., 2.4, 0., .3, 2.25]}


def requested_pose(station):
    x, y, z, roll, pitch, yaw = PRESETS[station]
    cr, sr, cp, sp, cy, sy = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2)
    return [x, y, z, cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def camera_matches(actual, expected):
    for values in (actual, expected):
        if (not isinstance(values, (list, tuple)) or len(values) != 7
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            return False
    distance = math.sqrt(sum((a-b)**2 for a, b in zip(actual[:3], expected[:3])))
    dot = sum(a*b for a, b in zip(actual[3:], expected[3:]))
    actual_norm = math.sqrt(sum(v*v for v in actual[3:]))
    expected_norm = math.sqrt(sum(v*v for v in expected[3:]))
    if (not math.isfinite(actual_norm) or not math.isfinite(expected_norm)
            or abs(actual_norm-1) > 1e-6 or abs(expected_norm-1) > 1e-6):
        return False
    return distance <= 1e-4 and abs(abs(dot/(actual_norm*expected_norm))-1.) <= 1e-8


def practice_manifest(workspace, input_names=None):
    """Protect the fixed template inputs; report generated artifacts separately."""
    workspace = Path(workspace)
    if input_names is None:
        metadata = json.loads((workspace/".gazeboteach-template.json").read_text(encoding="utf-8"))
        declared = metadata.get("files")
        require(isinstance(declared, dict) and len(declared) == 12,
                "Expected exactly the twelve template-declared personal inputs")
        input_names = tuple(sorted(declared))
    else:
        input_names = tuple(sorted(input_names))
    for name in input_names:
        path = Path(name)
        require(isinstance(name, str) and name and path.parts and not path.is_absolute() and ".." not in path.parts
                and name != ".gazeboteach-template.json" and path.parts[0] != "build",
                "Invalid template-declared input path: "+repr(name))
    current = {Path(name).as_posix(): value for name, value in source_hashes(workspace).items()}
    inputs = {name: current.get(name) for name in input_names}
    generated = {name: value for name, value in current.items()
                 if name == ".gazeboteach-template.json" or Path(name).parts[0] == "build"}
    unexpected = {name: value for name, value in current.items()
                  if name not in inputs and name not in generated}
    return {"input_names": list(input_names), "inputs": inputs, "generated": generated, "unexpected": unexpected}


def require_inputs_unchanged(before, after):
    require(before["input_names"] == after["input_names"] and before["inputs"] == after["inputs"],
            "GUI camera/capture changed or removed a declared personal input")
    require(not before["unexpected"] and not after["unexpected"],
            "GUI camera/capture created unexpected files outside its declared inputs/build metadata")


def input_guard_regression():
    with tempfile.TemporaryDirectory(prefix="gt13-input-guard-pure-") as directory:
        workspace = Path(directory)
        names = ["world.sdf", "assets/heightmap.png", "helper.py"]+["fixture-%02d.txt" % i for i in range(9)]
        for name in names:
            p = workspace/name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(("pure validator input "+name).encode())
        metadata = workspace/".gazeboteach-template.json"
        metadata.write_text(json.dumps({"files": {name: digest(workspace/name) for name in names}, "prepared_at": "first"}), encoding="utf-8")
        before = practice_manifest(workspace)
        require(all(before["inputs"].values()), "Pure fixture omitted a declared input")
        build = workspace/"build"
        build.mkdir()
        (build/"gallery.json").write_text('{"pure_build_fixture":true}', encoding="utf-8")
        value = json.loads(metadata.read_text(encoding="utf-8"))
        value.update(prepared_at="second", updated_files=[])
        metadata.write_text(json.dumps(value), encoding="utf-8")
        after = practice_manifest(workspace, before["input_names"])
        require_inputs_unchanged(before, after)
        require(after["generated"] != before["generated"], "Pure fixture did not exercise generated-artifact changes")
        rejected = []
        for name in ("world.sdf", "assets/heightmap.png", "helper.py"):
            p = workspace/name
            original = p.read_bytes()
            p.write_bytes(original+b" changed")
            actual = practice_manifest(workspace, before["input_names"])
            try:
                require_inputs_unchanged(before, actual)
            except AssertionError:
                rejected.append(name)
            else:
                raise AssertionError("Changed original input was hidden by the generated-artifact guard: "+name)
            finally:
                p.write_bytes(original)
        require_inputs_unchanged(before, practice_manifest(workspace, before["input_names"]))
    return {"passed": True, "kind": "pure_input_manifest_not_GUI_or_physics", "gazebo_started": False,
            "fixed_input_count": 12, "build_creation_and_metadata_refresh_allowed": True,
            "changed_original_inputs_rejected": rejected}


def validate_automatic_initialization(records):
    require(len(records) == 1, "New GUI needs exactly one automatic overview record before any explicit view")
    record = records[0]
    require(record.get("passed") is True and record.get("station") == "overview",
            "New GUI automatic overview did not finish successfully")
    events = record.get("network_events", [])
    sent = [(i, e) for i, e in enumerate(events) if e.get("event") == "published_once"]
    confirmed = [(i, e) for i, e in enumerate(events) if e.get("event") == "view_confirmed"]
    ready = [e for e in events if e.get("event") == "gui_ready"]
    require(len(sent) == len(confirmed) == len(ready) == 1,
            "Automatic overview needs one actual readiness, publication and camera-output confirmation")
    index, publication = sent[0]
    end_index, confirmation = confirmed[0]
    require(index < end_index and publication.get("type") == "gazebo.msgs.GUI"
            and publication.get("input_topic") == "/gazebo/default/gui"
            and confirmation.get("output_topic") == "/gazebo/default/user_camera/pose",
            "Automatic overview did not follow the real Classic GUI input/output route")
    target = requested_pose("overview")
    require(camera_matches(publication.get("requested_world_pose"), target)
            and camera_matches(confirmation.get("actual_world_pose"), target),
            "Automatic overview actual camera does not match the independently calculated preset")
    pre = [e for e in events[:index] if e.get("event") == "camera_pose" and e.get("after_publish") is False
           and camera_matches(e.get("world_pose"), e.get("world_pose"))]
    actual = [e for e in events[index+1:end_index] if e.get("event") == "camera_pose"
              and e.get("after_publish") is True and camera_matches(e.get("world_pose"), target)]
    require(pre and actual and ready[0].get("input_connected") is True and ready[0].get("actual_pose_seen") is True
            and camera_matches(ready[0].get("first_actual_world_pose"), pre[0]["world_pose"]),
            "Automatic overview lacks genuine pre-publication readiness or a post-publication actual pose")
    return {"first_actual_prepublication_pose": pre[0]["world_pose"],
            "actual_camera_pose": confirmation["actual_world_pose"], "requested_camera_pose": target,
            "post_publication_actual_messages": len(actual), "raw_helper_record": record}


def automatic_gui_initialization(session):
    path = control_directory(session)/"camera-events.jsonl"
    require(path.is_file(), "New GUI has no automatic overview camera-event log")
    lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    require(lines and all(line.endswith("\n") for line in lines), "Automatic camera-event log is incomplete")
    result = validate_automatic_initialization([json.loads(line) for line in lines if line.strip()])
    result.update(raw_log_path=str(path), initial_log_sha256=digest(path), initial_log_record_count=1)
    return result


def automatic_initialization_guard_regression():
    for records in ([], [{"passed": True, "station": "overview", "network_events": []}]):
        try:
            validate_automatic_initialization(records)
        except AssertionError:
            pass
        else:
            raise AssertionError("Automatic initialization accepted an empty log or success flag without real camera ACKs")
    return {"passed": True, "kind": "pure_negative_camera_log_validation_not_runtime_evidence",
            "gazebo_started": False, "missing_log_and_missing_actual_ACK_rejected": True}


def math_regressions():
    target = requested_pose("materials")
    require(camera_matches(target, target) and camera_matches(target[:3]+[-x for x in target[3:]], target),
            "Camera comparison rejected equivalent quaternion signs")
    require(not camera_matches([target[0]+.01]+target[1:], target), "Camera comparison accepted an incorrect real position")
    recorded = [5.800000190734863, 4.300000190734863, 2.5, .41191866993904114,
                .26663830876350403, .12742137908935547, -.8619691729545593]
    specular = requested_pose("specular")
    require(camera_matches(recorded, specular) and camera_matches(recorded[:3]+[-v for v in recorded[3:]], specular),
            "Actual retained OGRE float quaternion or its sign equivalent was falsely rejected")
    c, s = math.cos(.01/2), math.sin(.01/2)
    w, x, y, z = target[3:]
    rotated = target[:3]+[c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w]
    require(not camera_matches(rotated, target), "A true .01rad rotation error passed the unchanged orientation tolerance")
    require(not camera_matches(target[:3]+[0, 0, 0, 0], target)
            and not camera_matches(target[:3]+[v*1.01 for v in target[3:]], target),
            "Zero/grossly non-unit camera quaternion was accepted")
    require(not camera_matches([math.inf]+target[1:], target)
            and not camera_matches(target[:3]+[math.nan]+target[4:], target),
            "Non-finite native camera pose was accepted")
    return {"passed": True, "kind": "pure_camera_pose_math_not_GUI", "gazebo_started": False,
            "quaternion_double_cover_accepted": True, "incorrect_position_rejected": True,
            "retained_actual_float_sample_accepted_after_normalization": True,
            "true_rotation_error_zero_nonunit_and_nonfinite_rejected": True,
            "unchanged_position_tolerance_m": 1e-4, "unchanged_normalized_dot_tolerance": 1e-8,
            "quaternion_norm_tolerance": 1e-6, "input_guard_regression": input_guard_regression(),
            "automatic_initialization_negative_regression": automatic_initialization_guard_regression()}


def confirm_view(session, station):
    result = lab_command(session, "view", "--station", station, "--timeout", "4", timeout=18)
    data = result["data"]
    events = data["network_events"]
    sent = [event for event in events if event.get("event") == "published_once"]
    confirmed = [event for event in events if event.get("event") == "view_confirmed"]
    require(len(sent) == len(confirmed) == 1 and sent[0]["type"] == "gazebo.msgs.GUI"
            and sent[0]["input_topic"] == "/gazebo/default/gui"
            and confirmed[0]["output_topic"] == "/gazebo/default/user_camera/pose",
            "Station view did not use one actual Classic GUI input and real camera output")
    target = requested_pose(station)
    require(camera_matches(sent[0]["requested_world_pose"], target)
            and camera_matches(confirmed[0]["actual_world_pose"], target),
            "Native camera output does not match the independently calculated station preset")
    matching = [event for event in events if event.get("event") == "camera_pose" and event.get("after_publish") is True
                and camera_matches(event["world_pose"], target)]
    require(matching, "No actual post-publication user_camera/pose event supports this view")
    return {"station": station, "actual_camera_pose": confirmed[0]["actual_world_pose"],
            "requested_camera_pose": target, "actual_output_pose_messages": len(matching), "raw_helper_result": result,
            "server_light_fields_scope": "server_declaration_not_renderer_readback"}


def settle_epoch(session):
    deadline = time.monotonic()+15
    while time.monotonic() < deadline:
        value = checked_status(session)
        if value["snapshot"].get("elapsed_s", -1) >= 3.5:
            session.pause(True)
            return checked_status(session)
        time.sleep(.05)
    raise AssertionError("Scene did not reach its actual settling time before capture")


def capture(application, session, destination):
    from PyQt5 import QtCore, QtGui
    require(session.gui is not None and session.gui.poll() is None, "Owned Gazebo GUI exited before capture")
    xid, candidates = gazebo_window_id(session)
    ownership = subprocess.run(["xprop", "-id", hex(xid), "_NET_WM_PID", "WM_CLASS", "_NET_WM_NAME"],
                               text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3)
    require(ownership.returncode == 0, "Owned native GUI window disappeared before capture")
    window = QtGui.QWindow.fromWinId(xid)
    area = application.primaryScreen().availableGeometry()
    window.showNormal()
    window.resize(min(1280, area.width()-30), min(800, area.height()-30))
    window.raise_()
    window.requestActivate()
    # Rendering remains active while the physics world is paused. This wait is
    # only for expose/repaint, not a claim about new physical observations.
    deadline = time.monotonic()+.8
    while time.monotonic() < deadline:
        application.processEvents(QtCore.QEventLoop.AllEvents, 20)
        time.sleep(.02)
    pixmap = application.primaryScreen().grabWindow(xid)
    require(not pixmap.isNull() and pixmap.width() > 100 and pixmap.height() > 100, "Owned GUI capture is empty")
    destination.parent.mkdir(parents=True, exist_ok=True)
    require(pixmap.save(str(destination), "PNG") and destination.stat().st_size > 1000, "Owned GUI PNG could not be saved")
    final_xid, _ = gazebo_window_id(session)
    require(final_xid == xid and session.gui.poll() is None, "Actual GUI identity changed during screenshot capture")
    return {"path": str(destination), "sha256": digest(destination), "xid": hex(xid),
            "owned_gui_pid": session.gui.pid, "width": pixmap.width(), "height": pixmap.height(),
            "window_candidates": candidates, "native_xprop_readback": ownership.stdout,
            "scope": "only_this_session_gzclient_window"}


def checkpoint(application, session, station, path):
    before = checked_status(session)
    view = confirm_view(session, station)
    image = capture(application, session, path)
    after = checked_status(session)
    require(before["iteration"] == after["iteration"] and before["sim_time_s"] == after["sim_time_s"],
            "Camera/render capture unexpectedly advanced the paused physical world")
    return {"view": view, "capture": image, "actual_snapshot": after["snapshot"],
            "server_light_declarations": after["model"]["lights"], "world_sha256": after["world_sha256"]}


def shadow_variant(workspace):
    import xml.etree.ElementTree as ET
    path = workspace/"world.sdf"
    before = path.read_bytes()
    tree = xml_tree(before)
    lights = tree.findall("./world/light/cast_shadows")
    require(len(lights) == 3, "Shadow comparison requires the same three real lights")
    for element in lights:
        element.text = "false"
    after = ET.tostring(tree, encoding="utf-8")
    require(semantic_world(before, "shadows") == semantic_world(after, "shadows"), "Shadow variant changed unrelated scene fields")
    path.write_bytes(after)


def actor_second_phase(session, first):
    initial = first["actual_snapshot"]["actors"]["walker"]
    deadline, acknowledgements = time.monotonic()+60, []
    for _ in range(20):
        acknowledgements.append(bounded_steps(session, 100, deadline))
        current = checked_status(session)["snapshot"]["actors"]["walker"]
        if not current["has_animation_frame"]:
            continue
        root = math.hypot(current["world_pose"][0]-initial["world_pose"][0], current["world_pose"][1]-initial["world_pose"][1])
        foot_changes = {key: math.sqrt(sum((a-b)**2 for a, b in zip(current[key][:3], initial[key][:3])))
                        for key in ("left_foot_relative_hips", "right_foot_relative_hips")}
        foot = max(foot_changes.values())
        if root >= .15 and foot >= .03 and current["script_time_s"] != initial["script_time_s"]:
            return {"owned_step_acknowledgements": acknowledgements, "actual_root_displacement_m": root,
                    "actual_relative_foot_change_m": foot, "per_foot_position_change_m": foot_changes,
                    "first_actual_Actor": initial, "second_actual_Actor": current}
    raise AssertionError("Two real GUI Actor phases lack distinct root travel and relative limb motion")


def preserve_visual_evidence(session, destination):
    copied = preserve_evidence(session, destination)
    for name in ("world.sdf", "assets/heightmap.png", "build/gallery.json"):
        source = session.workspace/name
        if source.is_file():
            target = destination/"practice"/name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(str(source), str(target))
            copied[name] = {"path": str(target), "sha256": digest(target)}
    logs = session.session_dir/"gazebo-log"
    if logs.is_dir():
        for source in logs.rglob("*"):
            if source.is_file():
                target = destination/"gazebo-log"/source.relative_to(logs)
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(str(source), str(target))
                copied["gazebo-log/"+source.relative_to(logs).as_posix()] = {"path": str(target), "sha256": digest(target)}
    return copied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report")
    parser.add_argument("--output", default=".deploy/lesson13-visual-images")
    parser.add_argument("--case", choices=("baseline", "daylight_only", "shadows_only", "all"), default="baseline",
                        help="Run only the explicitly approved GUI window; all requires approval for three sequential sessions")
    parser.add_argument("--baseline-report", help="Saved successful baseline capture report for a separately approved contrast window")
    parser.add_argument("--math-only", action="store_true")
    args = parser.parse_args()
    report = {"passed": False, "lesson": KEY, "math_regressions": math_regressions(),
              "requested_case": args.case,
              "physical_sessions_started": 0, "sessions": [], "captures": {}, "visual_objectives_verified": False,
              "runner_source_sha256": digest(Path(__file__)),
              "manual_review_required": ["material texture and emissive", "specular highlight from two views",
                                         "light footprints and shadow disappearance", "daylight contrast", "terrain and two human animation phases"]}
    output = Path(args.output).expanduser().resolve()
    report_path = Path(args.report).expanduser().resolve() if args.report else None
    sessions, lesson, source_before = [], None, None
    references, baseline_sources = None, None
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="gazeboteach-13-visual-private-") as directory:
        private = Path(directory)
        try:
            if not args.math_only:
                if args.case in ("daylight_only", "shadows_only"):
                    require(args.baseline_report, "A separate contrast window requires its saved baseline capture report")
                    baseline_path = Path(args.baseline_report).expanduser().resolve()
                    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
                    require(baseline.get("passed") is True and "baseline" in baseline.get("captures", {}),
                            "Contrast reference lacks a successful actual baseline capture")
                    references = baseline["captures"]["baseline"]
                    baseline_sources = baseline.get("source_hashes_after")
                    report["baseline_reference"] = {"path": str(baseline_path), "sha256": digest(baseline_path),
                                                    "captures": references}
                require(os.environ.get("DISPLAY") and os.environ.get("QT_QPA_PLATFORM", "").lower() not in ("offscreen", "minimal"),
                        "Real GUI capture requires the actual graphical session")
                from PyQt5 import QtWidgets
                application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
                require(application.platformName() not in ("offscreen", "minimal"), "Offscreen rendering is not GUI evidence")
                report.update(display=os.environ["DISPLAY"], qt_platform=application.platformName(),
                              screen_name=application.primaryScreen().name(),
                              server_light_fields_scope="server_declaration_not_renderer_readback")
                lessons, errors = load_catalog(ROOT)
                found = [item for item in lessons if item.key == KEY]
                require(len(found) == 1, "Expected current L13 course: "+repr(errors))
                lesson, source_before = found[0], source_hashes(found[0].folder)
                if references is not None:
                    require(baseline_sources == source_before, "Baseline and contrast do not share the same frozen course sources")
                report["source_hashes_before"] = source_before
                cases = ("baseline", "daylight_only", "shadows_only") if args.case == "all" else (args.case,)
                for name in cases:
                    require(not (output/name).exists(), "Capture directory already exists; choose a new --output to preserve prior evidence")
                    storage = Storage(private/name)
                    workspace = storage.prepare(lesson)
                    original = (workspace/"world.sdf").read_bytes()
                    if name == "daylight_only":
                        load_config(lesson).edit_world(workspace/"world.sdf", lesson.folder/"world.sdf", "daylight", .2)
                        require(semantic_world(original, "daylight") == semantic_world((workspace/"world.sdf").read_bytes(), "daylight"),
                                "Daylight contrast changed another scene property")
                    elif name == "shadows_only":
                        shadow_variant(workspace)
                    personal_manifest = practice_manifest(workspace)
                    personal = personal_manifest["inputs"]
                    require(all(personal.values()) and not personal_manifest["unexpected"],
                            "Fresh private practice has missing/unexpected original inputs")
                    session = LessonSession(ROOT, lesson, storage=storage)
                    sessions.append((name, session))
                    session_record = {"name": name, "token": session.token, "master_uri": session.env["GAZEBO_MASTER_URI"],
                                      "practice_hashes_before": personal, "generated_files_before": personal_manifest["generated"],
                                      "unexpected_files_before": personal_manifest["unexpected"]}
                    report["sessions"].append(session_record)
                    report["physical_sessions_started"] += 1
                    log_path = session.session_dir/"gazebo-log"
                    log_path.mkdir(exist_ok=True)
                    session.env["GAZEBO_LOG_PATH"] = str(log_path)
                    session.start(gui=True, terminals=False)
                    session_record["owned_gzclient_pid"] = session.gui.pid
                    session_record["automatic_gui_initialization"] = automatic_gui_initialization(session)
                    await_ready(session)
                    status = settle_epoch(session)
                    captures = {}
                    report["captures"][name] = captures
                    if name == "baseline":
                        for station in ("overview", "materials", "specular", "lights", "terrain", "actors"):
                            captures[station] = checkpoint(application, session, station, output/name/(station+".png"))
                        motion = actor_second_phase(session, captures["actors"])
                        captures["actors_second_phase"] = checkpoint(application, session, "actors", output/name/"actors-second.png")
                        captures["actual_two_phase_Actor_evidence"] = motion
                    else:
                        station = "materials" if name == "daylight_only" else "lights"
                        captures[station] = checkpoint(application, session, station, output/name/(station+".png"))
                        reference = (references if references is not None else report["captures"]["baseline"])[station]
                        require(camera_matches(captures[station]["view"]["actual_camera_pose"], reference["view"]["actual_camera_pose"]),
                                "Lighting comparison did not use the same actually acknowledged camera")
                        if name == "shadows_only":
                            require(all(light["cast_shadows"] is False for light in status["model"]["lights"]),
                                    "Fresh shadow comparison did not load the changed shadow-enable declarations")
                    after_manifest = practice_manifest(workspace, personal_manifest["input_names"])
                    session_record["practice_hashes_after"] = after_manifest["inputs"]
                    session_record["generated_files_after"] = after_manifest["generated"]
                    session_record["unexpected_files_after"] = after_manifest["unexpected"]
                    require_inputs_unchanged(personal_manifest, after_manifest)
                    report["captures"][name] = captures
                    session.stop(reason="L13 private GUI checkpoints captured")
            report["passed"] = True
            report["automated_scope"] = "capture_and_actual_camera_readback_only_manual_material_light_shadow_review_still_required"
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
        finally:
            report["evidence"], report["cleanup_errors"] = {}, []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L13 visual acceptance finished")
                    report["evidence"][name] = preserve_visual_evidence(session, output/name/"runtime-evidence")
                except Exception as error:
                    report["cleanup_errors"].append(name+": "+repr(error))
            deadline = time.monotonic()+5
            leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(.05)
                leftovers = {session.token: live_token_processes(session.token) for _, session in sessions}
            report.update(leftovers=leftovers, source_templates_unchanged=(source_before == source_hashes(lesson.folder)) if lesson else None,
                          elapsed_seconds=round(time.monotonic()-started, 3))
            if lesson:
                report["source_hashes_after"] = source_hashes(lesson.folder)
            if any(leftovers.values()) or report["cleanup_errors"] or report["source_templates_unchanged"] is False:
                report["passed"] = False
            if report_path:
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
