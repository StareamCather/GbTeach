#!/usr/bin/env python3
"""Independent real ros_control claims, physical responses, limits and reload checks.

The ROS worker publishes Float64 commands itself and reads stock ROS messages and
Gazebo services. It does not call the lesson's motion/demo implementation, write
joint poses during a response, or synthesize measurement rows. Graphical checks
remain a separate desktop acceptance.
"""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson23_acceptance import Probe as ServiceProbe
from tests.lesson23_acceptance import hashes, ros_value, run, save, sha

NS, MODEL, JOINT = "/lesson25", "servo_arm", "lesson25_hinge"
MAIN, SOFT = "position_servo", "soft_servo"


def require(value, message):
    if not value:
        raise AssertionError(message)


class Probe(ServiceProbe):
    def __init__(self, record=False):
        import rospy
        rospy.init_node("independent_l25_probe", anonymous=True, disable_signals=True)
        self.rospy, self.events, self.lock = rospy, [], threading.Lock()
        self.subscribers, self.publishers = [], {}
        if record:
            from control_msgs.msg import JointControllerState
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64
            self.subscribers.append(rospy.Subscriber(NS + "/joint_states", JointState,
                                                    lambda msg: self.receive("joint_state", "", msg), queue_size=200))
            for controller in (MAIN, SOFT):
                self.subscribers.append(rospy.Subscriber(NS + "/" + controller + "/state", JointControllerState,
                    lambda msg, name=controller: self.receive("controller_state", name, msg), queue_size=200))
                self.publishers[controller] = rospy.Publisher(NS + "/" + controller + "/command", Float64,
                                                             queue_size=1, latch=False)

    def receive(self, source, controller, msg):
        with self.lock:
            self.events.append({"event_id": len(self.events) + 1, "source": source, "controller": controller,
                                "stamp_ns": msg.header.stamp.to_nsec(), "receive_monotonic_ns": time.monotonic_ns(),
                                "ros_time_at_receive_ns": self.rospy.Time.now().to_nsec(),
                                "callerid": getattr(msg, "_connection_header", {}).get("callerid", ""),
                                "data": ros_value(msg)})

    def controllers(self):
        from controller_manager_msgs.srv import ListControllers
        result = self.call(NS + "/controller_manager/list_controllers", ListControllers)
        return ros_value(result)["controller"]

    def pause(self, paused):
        from std_srvs.srv import Empty
        self.call("/gazebo/" + ("pause_physics" if paused else "unpause_physics"), Empty)

    def joint(self):
        from gazebo_msgs.srv import GetJointProperties, GetWorldProperties
        host = time.monotonic_ns()
        before = self.call("/gazebo/get_world_properties", GetWorldProperties)
        joint = self.call("/gazebo/get_joint_properties", GetJointProperties, joint_name=MODEL + "::" + JOINT)
        after = self.call("/gazebo/get_world_properties", GetWorldProperties)
        require(len(joint.position) == len(joint.rate) == 1, "Expected one actual moving joint axis")
        return {"host_monotonic_before_ns": host, "host_monotonic_after_ns": time.monotonic_ns(),
                "world_time_before_ns": round(before.sim_time * 1e9), "world_time_after_ns": round(after.sim_time * 1e9),
                "joint": ros_value(joint)}

    def snapshot(self):
        from gazebo_msgs.srv import GetPhysicsProperties
        return {"joint": self.joint(), "controllers": self.controllers(),
                "physics": ros_value(self.call("/gazebo/get_physics_properties", GetPhysicsProperties))}

    def switch(self, start, stop):
        from controller_manager_msgs.srv import SwitchController
        # A paused world cannot execute a switch at a controller update boundary.
        return ros_value(self.call(NS + "/controller_manager/switch_controller", SwitchController,
                                  start_controllers=start, stop_controllers=stop,
                                  strictness=2, start_asap=False, timeout=2.0))

    def command(self, controller, target):
        from std_msgs.msg import Float64
        publisher = self.publishers[controller]
        deadline = time.monotonic() + 3
        while publisher.get_num_connections() == 0:
            require(time.monotonic() < deadline, "Actual controller command subscriber did not connect")
            time.sleep(.015)
        before = self.joint()
        host = time.monotonic_ns()
        publisher.publish(Float64(data=target))
        return {"controller": controller, "requested_position_rad": target,
                "publish_monotonic_ns": host, "before": before}

    def phase(self, name, controller, target, duration=3.2, inspect=True):
        self.pause(False)
        with self.lock:
            first = len(self.events)
        command = self.command(controller, target)
        start = command["before"]["world_time_after_ns"]
        deadline, truth = time.monotonic() + max(12, duration * 5), []
        self.last_phase = {"name": name, "command": command, "truth": truth}
        while True:
            row = self.joint()
            truth.append(row)
            if (row["world_time_after_ns"] - start) / 1e9 >= duration:
                break
            require(time.monotonic() < deadline, "Actual physical response did not cover requested simulation duration")
            time.sleep(.018)
        self.pause(True)
        final = self.joint()
        # Drain already-published messages without advancing the paused world.
        time.sleep(.12)
        with self.lock:
            events = list(self.events[first:])
        result = {"name": name, "command": command, "truth": truth, "events": events, "final": final,
                  "time_basis": "Gazebo service brackets, original ROS header stamps and host receipt clocks are distinct."}
        self.last_phase = result
        if inspect:
            result["checked"] = inspect_phase(result, controller, max(-.8, min(.8, target)))
        return result


def index_joint(event):
    data = event["data"]
    require(JOINT in data["name"], "Actual JointState omitted the controlled joint")
    index = data["name"].index(JOINT)
    require(all(len(data[field]) == len(data["name"]) for field in ("position", "velocity", "effort")),
            "Actual ros_control JointState field arrays differ")
    return {field: data[field][index] for field in ("position", "velocity", "effort")}


def inspect_phase(result, controller, target):
    truth, events = result["truth"], result["events"]
    require(len(truth) >= 25 and len(events) >= 40, "Insufficient actual observations")
    require(all(b["event_id"] > a["event_id"] and b["receive_monotonic_ns"] > a["receive_monotonic_ns"]
                for a, b in zip(events, events[1:])), "Raw callback arrival order was changed")
    q = [row["joint"]["position"][0] for row in truth]
    v = [row["joint"]["rate"][0] for row in truth]
    require(all(math.isfinite(value) for value in q + v), "Actual native joint state is not finite")
    require(all(-.825 <= value <= .825 for value in q), "Physical joint exceeded declared stop range")
    start_q = result["command"]["before"]["joint"]["position"][0]
    span = target - start_q
    if abs(span) > .08:
        require(max(abs(value) for value in v) > .08, "No real nonzero joint velocity followed the command")
        require(any(.05 < (value - start_q) / span < .95 for value in q),
                "No queried intermediate physical position; command may not have produced a physical trajectory")
    end_ns = truth[-1]["world_time_before_ns"]
    tail = [row["joint"]["position"][0] for row in truth if end_ns - row["world_time_before_ns"] <= 400000000]
    require(len(tail) >= 4 and max(abs(value - target) for value in tail) < .055,
            "Actual joint did not settle near the requested/clamped target")
    require(abs(sum(tail) / len(tail) - target) < .028, "Mean actual terminal position is inaccurate")
    joints = [event for event in events if event["source"] == "joint_state"]
    states = [event for event in events if event["source"] == "controller_state" and event["controller"] == controller
              and abs(event["data"]["set_point"] - target) < 1e-9]
    require(len(joints) >= 20 and len(states) >= 20, "Actual JointState/controller stream missing")
    samples = [index_joint(event) for event in joints]
    require(all(abs(value["effort"]) <= 2.00001 for value in samples), "Applied Gazebo joint effort exceeded 2Nm")
    require(max(abs(value["effort"]) for value in samples) > .005, "No nonzero actual joint effort observed")
    for event in states:
        value = event["data"]
        require(abs(value["error"] - (value["set_point"] - value["process_value"])) < 1e-7,
                "Native controller error is not target minus actual joint position")
        require(value["time_step"] > 0 and all(math.isfinite(value[field]) for field in
                ("set_point", "process_value", "process_value_dot", "error", "command", "p", "i", "d")),
                "Invalid actual controller feedback")
    last_joint = index_joint(joints[-1])
    final_q, final_v = result["final"]["joint"]["position"][0], result["final"]["joint"]["rate"][0]
    # ROS messages and RPC values have different acquisition times; use a stated
    # small lag budget instead of declaring their timestamps identical.
    tolerance = abs(final_v) * .05 + .002
    require(abs(last_joint["position"] - final_q) < tolerance,
            "Final actual JointState position disagrees with independent Gazebo service")
    require(abs(states[-1]["data"]["process_value"] - final_q) < tolerance,
            "Final actual controller position disagrees with independent Gazebo service")
    return {"requested_position_rad": result["command"]["requested_position_rad"], "effective_target_rad": target,
            "terminal_mean_rad": sum(tail) / len(tail), "terminal_max_error_rad": max(abs(value - target) for value in tail),
            "peak_abs_native_velocity_rad_s": max(abs(value) for value in v),
            "peak_abs_applied_joint_effort_nm": max(abs(value["effort"]) for value in samples),
            "peak_abs_raw_pid_command_nm": max(abs(event["data"]["command"]) for event in states),
            "native_query_count": len(truth), "joint_message_count": len(joints), "controller_message_count": len(states),
            "final_ros_vs_gazebo_tolerance_rad": tolerance,
            "effort_note": "JointState effort is Gazebo GetForce readback; controller.command is raw PID before hardware effort limiting."}


def state_map(controllers):
    return {item["name"]: item["state"] for item in controllers}


def mapped_controller_library(session, artifacts, name):
    """Prove the owned native server mapped the prepared private controller ELF."""
    record = json.loads((session.workspace / "build/servo_lab.json").read_text(encoding="utf-8"))
    vendor = (session.workspace / "vendor/effort_controllers").resolve()
    candidates = {vendor / "libgazeboteach_l25_effort.so", vendor / "lib/libgazeboteach_l25_effort.so"}
    declared = Path(record["vendor_library_path"]).resolve()
    require(declared in candidates and sha(declared) == record["vendor_library_sha256"],
            "Prepared unique private controller library fingerprint differs")
    owned = live_token_processes(session.token)
    pids = [session.server.pid] + [pid for pid in owned if pid != session.server.pid]
    observations = []
    for pid in pids:
        if pid not in owned:
            continue
        directory = Path("/proc", str(pid))
        try:
            if directory.joinpath("comm").read_text().strip() != "gzserver":
                continue
            lines = directory.joinpath("maps").read_text().splitlines()
            selected = []
            files = {}
            for line in lines:
                parts = line.split(None, 5)
                if len(parts) != 6:
                    continue
                mapped = Path(parts[5]).resolve()
                if mapped not in candidates:
                    continue
                info = mapped.stat()
                major, minor = (int(value, 16) for value in parts[3].split(":"))
                require(int(parts[4]) == info.st_ino and (major, minor) == (os.major(info.st_dev), os.minor(info.st_dev)),
                        "Native mapped controller device/inode differs from the prepared file")
                require(sha(mapped) == record["vendor_library_sha256"], "Native mapped private controller ELF hash differs")
                selected.append(line)
                files[str(mapped)] = {"sha256": sha(mapped), "inode": info.st_ino,
                                      "device_major": major, "device_minor": minor}
            if selected:
                require(pid in live_token_processes(session.token), "Native PID lost its current-session identity")
                path = artifacts / (name + "-controller-library.maps")
                path.write_text("\n".join(selected) + "\n", encoding="utf-8")
                observations.append({"pid": pid, "process": "gzserver", "matched_map_lines": selected,
                                     "mapped_files": files, "maps_artifact": str(path), "maps_sha256": sha(path)})
        except (OSError, ValueError):
            continue
    require(len(observations) == 1, "Expected exactly one owned gzserver mapping the unique prepared private controller")
    return {"declared_library_path": str(declared), "prepared_library_sha256": record["vendor_library_sha256"],
            "actual_native_mappings": observations}


def exercise(probe, extended):
    config = json.loads(Path("parameters.json").read_text())
    result = {"parameters": config, "initial": probe.snapshot(), "phases": [], "manager_checks": []}
    probe.exercise_progress = result
    controllers = result["initial"]["controllers"]
    require(state_map(controllers).get(MAIN) == "running" and state_map(controllers).get(SOFT) != "running",
            "Expected the primary running controller and an inactive alternative")
    main = next(item for item in controllers if item["name"] == MAIN)
    require(main["type"] == "effort_controllers/JointPositionController", "Not the stock effort position controller")
    claims = [item for item in main["claimed_resources"] if JOINT in item["resources"]]
    require(len(claims) == 1 and "EffortJointInterface" in claims[0]["hardware_interface"],
            "Primary controller did not claim the actual EffortJointInterface resource")
    probe.pause(False)
    before = probe.controllers()
    for name, start in (("running_resource_conflict", [SOFT]), ("strict_unknown_controller", ["does_not_exist_l25"])):
        response = probe.switch(start, [] if name == "running_resource_conflict" else [MAIN])
        after = probe.controllers()
        require(response["ok"] is False and state_map(after) == state_map(before),
                "Strict rejected switch changed the running controller set")
        result["manager_checks"].append({"name": name, "response": response, "before": before, "after": after})
    probe.phase("physical_zero_hold", MAIN, 0, duration=1.6, inspect=False)
    result["phases"].append(probe.phase("configured_step", MAIN, config["target_rad"]))
    require(all(abs(event["data"]["p"] - config["p"]) < 1e-9
                and abs(event["data"]["d"] - config["d"]) < 1e-9 for event in result["phases"][-1]["events"]
                if event["source"] == "controller_state" and event["controller"] == MAIN),
            "Actual controller gains do not match the prepared exercise configuration")
    result["phases"].append(probe.phase("opposite_step", MAIN, -config["target_rad"]))
    if extended:
        result["phases"].append(probe.phase("upper_target_clamp", MAIN, 1.2, duration=4))
        result["phases"].append(probe.phase("lower_target_clamp", MAIN, -1.2, duration=4))
        probe.pause(False)
        switched = probe.switch([SOFT], [MAIN])
        after = probe.controllers()
        require(switched["ok"] and state_map(after).get(SOFT) == "running" and state_map(after).get(MAIN) != "running",
                "Actual exclusive controller switch did not complete")
        result["manager_checks"].append({"name": "exclusive_switch_to_soft", "response": switched, "after": after})
        result["phases"].append(probe.phase("soft_controller_real_motion", SOFT, .2, duration=4))
        probe.pause(False)
        require(probe.switch([MAIN], [SOFT])["ok"], "Could not restore primary controller ownership")
    result["final"] = probe.snapshot()
    result["restoration_note"] = "Disposable independent world; no pose writes during response. Session stop destroys only this private world."
    return result


def worker(operation, output, extended):
    result = {"passed": False, "operation": operation}
    probe = None
    try:
        probe = Probe(record=operation in ("exercise", "seed"))
        if operation == "seed":
            result["seed_phase"] = probe.phase("actual_nonzero_entry", MAIN, .45, duration=.18, inspect=False)
            result["snapshot"] = probe.snapshot()
            actual = result["snapshot"]["joint"]["joint"]
            require(abs(actual["position"][0]) > .005 and abs(actual["rate"][0]) > .05,
                    "Actual command did not create the required nonzero paused entry state")
        else:
            result["snapshot" if operation == "snapshot" else "exercise"] = (
                probe.snapshot() if operation == "snapshot" else exercise(probe, extended))
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__ + ": " + str(error)
        if probe is not None:
            result["partial_exercise"] = getattr(probe, "exercise_progress", None)
            result["last_phase"] = getattr(probe, "last_phase", None)
            with probe.lock:
                result["unmodified_events_on_failure"] = list(probe.events)
    output.parent.mkdir(parents=True, exist_ok=True)
    save(output, result)
    return 0 if result["passed"] else 1


def probe_result(session, artifacts, name, operation="snapshot", extended=False):
    output = artifacts / (name + ".json")
    command = [sys.executable, str(Path(__file__).resolve()), "--worker", operation, "--report", str(output)]
    if extended:
        command.append("--extended")
    status = run(session, command, artifacts / (name + ".log"), timeout=160 if extended else 90)
    result = json.loads(output.read_text())
    require(status == 0 and result["passed"], "Independent real ros_control probe failed: " + str(result.get("error")))
    return result


def rejected_saved_edit(session, artifacts):
    session.pause(True)
    before = probe_result(session, artifacts, "saved-edit-before")["snapshot"]
    config = session.workspace / "parameters.json"
    original = config.read_bytes()
    fingerprints = hashes(session.workspace)
    try:
        value = json.loads(original)
        value["p"] += .1
        save(config, value)
        output = artifacts / "saved-without-reload.json"
        status = run(session, [sys.executable, str(session.workspace / "verify.py"), "--report", str(output)],
                     artifacts / "saved-without-reload.log", timeout=50)
        require(status != 0 and output.is_file(), "Saved-but-not-reloaded controller inputs were not rejected")
        result = json.loads(output.read_text())
        require(result.get("passed") is False and result.get("error"), "Preflight fault lacks a diagnostic")
    finally:
        config.write_bytes(original)
    after = probe_result(session, artifacts, "saved-edit-after")["snapshot"]
    require(before["joint"]["world_time_after_ns"] == after["joint"]["world_time_before_ns"],
            "Rejected preflight advanced the paused simulation")
    require(before["joint"]["joint"] == after["joint"]["joint"] and before["controllers"] == after["controllers"],
            "Rejected preflight changed actual joint state or controller ownership")
    require(hashes(session.workspace) == fingerprints, "Rejected preflight changed other exercise input files")
    return {"name": "saved_without_reload", "passed": True, "report": str(output), "diagnosis": result["error"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--worker", choices=("snapshot", "exercise", "seed"))
    parser.add_argument("--extended", action="store_true")
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker, args.report, args.extended)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem + "-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == "classic11-noetic:25"]
    require(len(found) == 1, "Expected published L25 manifest: " + repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    dependency = ROOT / "resources/third_party/effort_controllers"
    dependency_source = hashes(dependency) if dependency.is_dir() else {}
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": [],
              "official_controller_source_sha256": dependency_source,
              "graphical_checks_performed": False, "test_algorithm": __doc__}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l25-") as temporary:
        try:
            configurations = (("baseline", {"p": 4., "d": .35, "target_rad": .45}),
                              ("gain_only", {"p": 2., "d": .35, "target_rad": .45}),
                              ("target_only", {"p": 4., "d": .35, "target_rad": -.3}))
            for name, config in configurations:
                storage = Storage(Path(temporary) / name)
                workspace = storage.prepare(lesson)
                save(workspace / "parameters.json", config)
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((name, session))
                session.start(gui=False, terminals=False)
                mapping = mapped_controller_library(session, artifacts, name)
                if name == "baseline":
                    seeded = probe_result(session, artifacts, "nonzero-paused-entry", "seed")["snapshot"]
                    report["scenarios"].append(rejected_saved_edit(session, artifacts))
                    good = artifacts / "restored-input-course-verification.json"
                    status = run(session, [sys.executable, str(workspace / "verify.py"), "--report", str(good)],
                                 artifacts / "restored-input-course-verification.log", timeout=160)
                    require(status == 0 and json.loads(good.read_text()).get("passed") is True,
                            "Original exact inputs did not recover course verification")
                    restored = probe_result(session, artifacts, "cold-paused-verify-exit")["snapshot"]
                    expected_joint, actual_joint = seeded["joint"]["joint"], restored["joint"]["joint"]
                    require(restored["physics"]["pause"] and state_map(seeded["controllers"]) == state_map(restored["controllers"]),
                            "Cold-paused verifier did not restore pause/controller ownership")
                    errors = {field: abs(expected_joint[field][0] - actual_joint[field][0]) for field in ("position", "rate")}
                    require(errors["position"] < 1e-7 and errors["rate"] < 1e-7,
                            "Cold-paused verifier did not restore nonzero actual joint pose/velocity: " + repr(errors))
                    report["scenarios"].append({"name": "restored_exact_inputs", "passed": True,
                                                "report": str(good), "nonzero_entry_restore_errors": errors})
                    step_log = artifacts / "real-100-step-cli.log"
                    step_status = run(session, [sys.executable, str(workspace / "lab.py"), "step", "--steps", "100"],
                                      step_log, timeout=25)
                    require(step_status == 0, "Actual 100-step CLI failed; inspect " + str(step_log))
                    candidates = []
                    for line in step_log.read_text(encoding="utf-8").splitlines():
                        try:
                            value = json.loads(line)
                        except ValueError:
                            continue
                        if isinstance(value, dict) and {"before", "after", "requested_steps"} <= set(value):
                            candidates.append(value)
                    require(len(candidates) == 1, "The step CLI did not report one unambiguous raw result")
                    step = candidates[0]
                    stepped = probe_result(session, artifacts, "real-100-step-exit")["snapshot"]
                    native_delta = (stepped["joint"]["world_time_before_ns"]
                                    - restored["joint"]["world_time_after_ns"]) / 1e9
                    require(step["requested_steps"] == 100 and abs(step["physics_time_step"] - .001) < 1e-12,
                            "The step CLI used a different tick count or actual physics step")
                    require(abs(step["after"] - step["before"] - .1) < 1e-7 and abs(native_delta - .1) < 1e-7
                            and step["paused"] is True and stepped["physics"]["pause"],
                            "100 ticks did not complete as exactly 0.1 simulation seconds at a paused boundary")
                    save(artifacts / "real-100-step-cli.json", step)
                    report["scenarios"].append({"name": "real_100_step_cli", "passed": True,
                        "raw_result": step, "native_world_time_delta_seconds": native_delta, "raw_log": str(step_log)})
                raw = probe_result(session, artifacts, name, "exercise", extended=name == "baseline")
                report["scenarios"].append({"name": name, "passed": True, "parameters": config,
                    "input_sha256": hashes(workspace), "raw_report": str(artifacts / (name + ".json")),
                    "actual_controller_library_mapping": mapping,
                    "checked_phases": [phase["checked"] for phase in raw["exercise"]["phases"]]})
                for relative in ("parameters.json", "build/servo.urdf", "build/controllers.yaml", "build/servo_lab.json"):
                    if (workspace / relative).is_file():
                        shutil.copy2(str(workspace / relative), str(artifacts / (name + "-" + Path(relative).name)))
                session.stop(reason="L25 independent physical case complete")
                print(json.dumps({"scenario": name, "passed": True}), flush=True)
            baseline = next(case for case in report["scenarios"] if case["name"] == "baseline")["checked_phases"][0]
            changed = next(case for case in report["scenarios"] if case["name"] == "gain_only")["checked_phases"][0]
            require(baseline["peak_abs_native_velocity_rad_s"] > changed["peak_abs_native_velocity_rad_s"] * 1.08,
                    "A lower proportional gain did not measurably change the real response speed")
            report["single_variable_gain_effect"] = {"baseline_peak_rad_s": baseline["peak_abs_native_velocity_rad_s"],
                "lower_gain_peak_rad_s": changed["peak_abs_native_velocity_rad_s"]}
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__ + ": " + str(error)
        finally:
            report["cleanup_errors"] = []
            for name, session in reversed(sessions):
                try:
                    session.stop(reason="L25 final independent cleanup")
                    if session.log_path.exists():
                        shutil.copy2(str(session.log_path), str(artifacts / (name + "-runtime.log")))
                    metadata = session.session_dir / "session.json"
                    if metadata.is_file():
                        shutil.copy2(str(metadata), str(artifacts / (name + "-session.json")))
                except Exception as error:
                    report["cleanup_errors"].append(name + ": " + str(error))
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["official_controller_source_unchanged"] = dependency_source == (hashes(dependency) if dependency.is_dir() else {})
            report["elapsed_seconds"] = round(time.monotonic() - started, 3)
            if (report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]
                    or not report["official_controller_source_unchanged"]):
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key not in ("scenarios", "source_sha256")},
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
