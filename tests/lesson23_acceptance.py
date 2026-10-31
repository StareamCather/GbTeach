#!/usr/bin/env python3
"""Independently check generated robot geometry and actual 2R arm observations."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gazeboteach.catalog import load_catalog
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.integration_probe import live_token_processes
from tests.lesson05_acceptance import first, parsed_model, short_name


def require(value, message):
    if not value:
        raise AssertionError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hashes(folder):
    return {path.relative_to(folder).as_posix(): sha(path) for path in sorted(Path(folder).rglob("*"))
            if path.is_file() and not any(part in ("__pycache__", "build", "devel", "install") for part in path.relative_to(folder).parts)}


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def numbers(value, count=3):
    result = [float(part) for part in value.split()]
    require(len(result) == count and all(math.isfinite(v) for v in result), "Invalid robot coordinate vector")
    return result


def identity():
    return [[float(a == b) for b in range(4)] for a in range(4)]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def axis_rotation(axis, angle):
    length = math.sqrt(sum(value*value for value in axis))
    require(length > 0, "Joint axis has zero norm")
    x, y, z = [value/length for value in axis]
    c, s, t = math.cos(angle), math.sin(angle), 1-math.cos(angle)
    result = identity()
    result[0][:3] = [t*x*x+c, t*x*y-s*z, t*x*z+s*y]
    result[1][:3] = [t*x*y+s*z, t*y*y+c, t*y*z-s*x]
    result[2][:3] = [t*x*z-s*y, t*y*z+s*x, t*z*z+c]
    return result


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = multiply(multiply(axis_rotation((0, 0, 1), rpy[2]), axis_rotation((0, 1, 0), rpy[1])), axis_rotation((1, 0, 0), rpy[0]))
    for index, value in enumerate(xyz):
        result[index][3] = value
    return result


def origin(node):
    return transform(numbers(node.get("xyz", "0 0 0")), numbers(node.get("rpy", "0 0 0"))) if node is not None else identity()


def forward_kinematics(robot, positions):
    """URDF parent-origin then measured joint motion; no course FK is imported."""
    joints = list(robot.findall("joint"))
    roots = set(node.get("name") for node in robot.findall("link"))-{node.find("child").get("link") for node in joints}
    require(len(roots) == 1, "Generated URDF is not one rooted link tree")
    poses = {next(iter(roots)): identity()}
    while joints:
        ready = [node for node in joints if node.find("parent").get("link") in poses]
        require(ready, "Generated URDF contains a cycle/disconnected subtree")
        for node in ready:
            pose = origin(node.find("origin"))
            kind, name = node.get("type"), node.get("name")
            if kind != "fixed":
                require(name in positions and kind in ("revolute", "continuous"), "Unexpected or unobserved moving joint")
                axis = numbers(node.find("axis").get("xyz", "1 0 0"))
                pose = multiply(pose, axis_rotation(axis, positions[name]))
            poses[node.find("child").get("link")] = multiply(poses[node.find("parent").get("link")], pose)
            joints.remove(node)
    return poses


def matrix_from_pose(pose):
    """Accept an unmodified ROS geometry_msgs/Pose dictionary."""
    p, q = pose["position"], pose["orientation"]
    x, y, z, w = [q[key] for key in ("x", "y", "z", "w")]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    require(norm > 0, "Actual pose has zero quaternion")
    x, y, z, w = [value/norm for value in (x, y, z, w)]
    result = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), p["x"]],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), p["y"]],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), p["z"]], [0., 0., 0., 1.]]
    return result


def pose_error(actual, expected):
    position = math.sqrt(sum((actual[i][3]-expected[i][3])**2 for i in range(3)))
    cosine = (sum(actual[i][j]*expected[i][j] for i in range(3) for j in range(3))-1)/2
    return position, math.acos(min(1, max(-1, cosine)))


def transpose(value):
    return [list(row) for row in zip(*value)]


def inertia_matrix(node, sdf=False):
    fields = {key: float(node.findtext(key) if sdf else node.get(key)) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    return [[fields["ixx"], fields["ixy"], fields["ixz"]],
            [fields["ixy"], fields["iyy"], fields["iyz"]], [fields["ixz"], fields["iyz"], fields["izz"]]]


def aggregate_inertia(robot, names, reference):
    """Independently apply rigid transforms and the parallel-axis theorem."""
    zero = {joint.get("name"): 0 for joint in robot.findall("joint") if joint.get("type") != "fixed"}
    poses = forward_kinematics(robot, zero)
    reference_pose = poses[reference]
    reference_rotation = [row[:3] for row in reference_pose[:3]]
    bodies = []
    for name in names:
        link = robot.find("link[@name='"+name+"']")
        inertial = link.find("inertial")
        world = multiply(poses[name], origin(inertial.find("origin")))
        rotation = multiply(transpose(reference_rotation), [row[:3] for row in world[:3]])
        position = [sum(reference_rotation[k][i]*(world[k][3]-reference_pose[k][3]) for k in range(3)) for i in range(3)]
        tensor = multiply(multiply(rotation, inertia_matrix(inertial.find("inertia"))), transpose(rotation))
        bodies.append((float(inertial.find("mass").get("value")), position, tensor))
    mass = sum(item[0] for item in bodies)
    center = [sum(m*p[i] for m, p, _ in bodies)/mass for i in range(3)]
    tensor = [[0.]*3 for _ in range(3)]
    for m, p, local in bodies:
        delta = [p[i]-center[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                tensor[i][j] += local[i][j]+m*((sum(v*v for v in delta) if i == j else 0)-delta[i]*delta[j])
    return {"mass": mass, "com": center, "inertia": tensor}


def ros_value(value):
    if hasattr(value, "__slots__"):
        return {field: ros_value(getattr(value, field)) for field in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_value(item) for item in value]
    return value


class Probe:
    """A separate real rospy client with bounded calls, independent of lab.py."""
    def __init__(self):
        import rospy
        rospy.init_node("independent_l23_probe", anonymous=True, disable_signals=True)
        self.rospy = rospy

    def call(self, service, kind, **kwargs):
        self.rospy.wait_for_service(service, timeout=4)
        done, result = threading.Event(), {}
        def invoke():
            proxy = self.rospy.ServiceProxy(service, kind)
            try:
                result["value"] = proxy(**kwargs)
            except Exception as error:
                result["error"] = error
            finally:
                proxy.close()
                done.set()
        threading.Thread(target=invoke, daemon=True).start()
        require(done.wait(4), "Independent ROS request timed out: "+service)
        if "error" in result:
            raise result["error"]
        value = result["value"]
        require(not hasattr(value, "success") or value.success, "Actual service rejected "+service+": "+getattr(value, "status_message", ""))
        return value

    def snapshot(self):
        from gazebo_msgs.srv import GetJointProperties, GetLinkProperties, GetLinkState
        from gazebo_msgs.srv import GetModelProperties, GetModelState, GetPhysicsProperties, GetWorldProperties
        started = time.monotonic_ns()
        before = self.call("/gazebo/get_world_properties", GetWorldProperties)
        model = self.call("/gazebo/get_model_properties", GetModelProperties, model_name="xacro_arm")
        state = self.call("/gazebo/get_model_state", GetModelState, model_name="xacro_arm", relative_entity_name="world")
        links, properties = {}, {}
        for name in model.body_names:
            simple = name.rsplit("::", 1)[-1]
            scoped = "xacro_arm::"+simple
            links[simple] = ros_value(self.call("/gazebo/get_link_state", GetLinkState, link_name=scoped, reference_frame="world"))
            properties[simple] = ros_value(self.call("/gazebo/get_link_properties", GetLinkProperties, link_name=scoped))
        joints = {name: ros_value(self.call("/gazebo/get_joint_properties", GetJointProperties, joint_name="xacro_arm::"+name))
                  for name in ("lesson23_shoulder", "lesson23_elbow")}
        physics = self.call("/gazebo/get_physics_properties", GetPhysicsProperties)
        after = self.call("/gazebo/get_world_properties", GetWorldProperties)
        return {"host_monotonic_before_ns": started, "host_monotonic_after_ns": time.monotonic_ns(),
                "world_time_before_ns": round(before.sim_time*1e9), "world_time_after_ns": round(after.sim_time*1e9),
                "model_properties": ros_value(model), "model_state": ros_value(state), "link_states": links,
                "link_properties": properties, "joint_properties": joints, "physics": ros_value(physics),
                "time_basis": "Independent service brackets; API ROS header time is not a native capture timestamp."}


def worker(operation, path):
    result = {"passed": False, "operation": operation}
    probe, saved_properties = None, None
    try:
        probe = Probe()
        if operation in ("bad_mass", "bad_gravity"):
            from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties
            result["before"] = probe.snapshot()
            saved_properties = probe.call("/gazebo/get_link_properties", GetLinkProperties, link_name="xacro_arm::arm1")
            values = {key: getattr(saved_properties, key) for key in ("com", "gravity_mode", "mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
            values["mass" if operation == "bad_mass" else "gravity_mode"] = 2.0 if operation == "bad_mass" else True
            probe.call("/gazebo/set_link_properties", SetLinkProperties, link_name="xacro_arm::arm1", **values)
            result["fault_readback"] = ros_value(probe.call("/gazebo/get_link_properties", GetLinkProperties, link_name="xacro_arm::arm1"))
            raw_path = Path(path).with_name(Path(path).stem+"-course.json")
            with Path(path).with_suffix(".course.log").open("w", encoding="utf-8") as stream:
                completed = subprocess.run([sys.executable, str(Path.cwd()/"verify.py"), "--report", str(raw_path)],
                                           stdout=stream, stderr=subprocess.STDOUT, timeout=100, start_new_session=True)
            result["course_returncode"], result["course_report"] = completed.returncode, json.loads(raw_path.read_text())
        elif operation == "seed":
            from gazebo_msgs.srv import GetLinkState, SetLinkState, SetModelConfiguration
            require(probe.snapshot()["physics"]["pause"], "Seed fixture must use a paused private world")
            probe.call("/gazebo/set_model_configuration", SetModelConfiguration, model_name="xacro_arm", urdf_param_name="",
                       joint_names=["lesson23_shoulder", "lesson23_elbow"], joint_positions=[.1, -.2])
            link = probe.call("/gazebo/get_link_state", GetLinkState, link_name="xacro_arm::arm2", reference_frame="world").link_state
            original_pose = matrix_from_pose(ros_value(link.pose))
            link.link_name, link.reference_frame = "xacro_arm::arm2", "world"
            link.twist.linear.x, link.twist.linear.y, link.twist.linear.z = .003, -.002, .001
            link.twist.angular.x, link.twist.angular.y, link.twist.angular.z = 0, 0, .004
            probe.call("/gazebo/set_link_state", SetLinkState, link_state=link)
            actual = probe.call("/gazebo/get_link_state", GetLinkState, link_name="xacro_arm::arm2", reference_frame="world")
            result["seed_twist_actual_readback"] = ros_value(actual)
            linear = [getattr(actual.link_state.twist.linear, axis) for axis in ("x", "y", "z")]
            angular = [getattr(actual.link_state.twist.angular, axis) for axis in ("x", "y", "z")]
            require(all(math.isfinite(value) for value in linear+angular)
                    and max(abs(value) for value in linear) > 1e-6 and max(abs(value) for value in angular) > 1e-6,
                    "Entry twist fixture was not independently read back as nonzero")
            error, angle = pose_error(matrix_from_pose(ros_value(actual.link_state.pose)), original_pose)
            require(error < 1e-9 and angle < 1e-7, "Twist fixture changed the actual link pose")
        result["snapshot"] = probe.snapshot()
        result["passed"] = True
    except Exception as error:
        result["error"] = type(error).__name__+": "+str(error)
    finally:
        if saved_properties is not None:
            try:
                from gazebo_msgs.srv import SetLinkProperties
                values = {key: getattr(saved_properties, key) for key in ("com", "gravity_mode", "mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
                probe.call("/gazebo/set_link_properties", SetLinkProperties, link_name="xacro_arm::arm1", **values)
                result["after_restore"] = probe.snapshot()
            except Exception as error:
                result.update(passed=False, restore_error=repr(error))
    save(path, result)
    return 0 if result["passed"] else 1


def run(session, argv, log_path, timeout=100):
    environment = dict(session.env)
    environment["ROS_HOME"] = str(session.session_dir/"ros-home")
    environment["ROS_LOG_DIR"] = str(session.session_dir/"ros-home/log")
    with Path(log_path).open("w", encoding="utf-8") as stream:
        completed = subprocess.run(argv, cwd=str(session.workspace), env=environment, stdout=stream,
                                   stderr=subprocess.STDOUT, timeout=timeout, start_new_session=True)
    return completed.returncode


def worker_result(session, artifacts, name, operation="snapshot"):
    path = artifacts/(name+".json")
    status = run(session, [sys.executable, str(Path(__file__).resolve()), "--worker", operation, "--report", str(path)],
                 artifacts/(name+".stdout.log"), timeout=130 if operation.startswith("bad_") else 45)
    result = json.loads(path.read_text())
    require(status == 0 and result["passed"], "Independent physical snapshot failed: "+str(result.get("error") or result.get("restore_error")))
    return result


def snapshot(session, artifacts, name, operation="snapshot"):
    return worker_result(session, artifacts, name, operation)["snapshot"]


def same_snapshot(a, b):
    require(a["physics"]["pause"] == b["physics"]["pause"], "Entry pause state was not restored")
    require(set(a["link_states"]) == set(b["link_states"]), "Physical links changed during verification")
    for name, before in a["link_states"].items():
        after = b["link_states"][name]
        position, angle = pose_error(matrix_from_pose(before["link_state"]["pose"]), matrix_from_pose(after["link_state"]["pose"]))
        require(position < 1e-5 and angle < 1e-5, "Actual entry link pose was not restored: "+name)
        for field in ("linear", "angular"):
            require(max(abs(before["link_state"]["twist"][field][axis]-after["link_state"]["twist"][field][axis]) for axis in ("x", "y", "z")) < 1e-5,
                    "Actual entry link twist was not restored: "+name)
    for name, before in a["joint_properties"].items():
        after = b["joint_properties"][name]
        require(max(abs(x-y) for key in ("position", "rate") for x, y in zip(before[key], after[key])) < 1e-5,
                "Actual entry joint state was not restored: "+name)


def rotated_inertia(matrix, pose):
    rotation = [row[:3] for row in pose[:3]]
    return multiply(multiply(rotation, matrix), transpose(rotation))


def compare_inertia(mass, center, tensor, expected, label, serialized=False):
    def tolerance(value):
        # Installed CLI and spawn_urdf conversion both serialize six significant
        # digits. Keep those values; compare theory with half the final quantum,
        # then separately compare actual properties to converted SDF at 1e-8.
        return max(1e-12, 5.01*10**(math.floor(math.log10(abs(value)))-6)) if serialized and abs(value) > 1e-12 else 1e-8
    require(abs(mass-expected["mass"]) <= tolerance(expected["mass"]), label+": mass differs from the rigid-body composition")
    require(all(abs(a-b) <= tolerance(b) for a, b in zip(center, expected["com"])), label+": combined COM is incorrect")
    require(all(abs(tensor[i][j]-expected["inertia"][i][j]) <= tolerance(expected["inertia"][i][j]) for i in range(3) for j in range(3)), label+": combined inertia is incorrect")


def inspect_generated(session, physical, length, preserve, artifacts, label):
    workspace = session.workspace
    urdf, sdf = workspace/"build/arm.urdf", workspace/"build/arm.sdf"
    record = json.loads((workspace/"build/arm_lab.json").read_text())
    require(sha(urdf) == record["generated_urdf_sha256"] and sha(sdf) == record["converted_sdf_sha256"], "Generated artifact fingerprints differ from the prepared record")
    require(record["settings_sha256"] == sha(workspace/"arm_settings.json") and record["world_sha256"] == sha(workspace/"world.sdf"), "Loaded input fingerprints differ")
    require(all(sha(workspace/name) == digest for name, digest in record["source_sha256"].items()), "Loaded source fingerprints differ")
    robot = ET.parse(str(urdf)).getroot()
    require(robot.tag == "robot" and {node.get("name") for node in robot.findall("link")} == {"world", "base_link", "arm1", "arm2", "tool_link"}, "Xacro changed the required URDF topology")
    require(not any(node.tag.startswith("{http://www.ros.org/wiki/xacro}") for node in robot.iter()), "Unexpanded Xacro instructions reached the URDF")
    for name, arm_length, mass in (("arm1", length, 1.), ("arm2", .4, .5)):
        link = robot.find("link[@name='"+name+"']")
        for kind in ("visual", "collision"):
            require(numbers(link.find(kind+"/geometry/box").get("size")) == [arm_length, .06, .06], "Xacro length is not propagated to actual geometry")
            require(numbers(link.find(kind+"/origin").get("xyz")) == [arm_length/2, 0, 0], "Geometry COM-origin convention differs")
        expected = aggregate_inertia(robot, [name], name)
        require(expected["mass"] == mass and max(abs(a-b) for a, b in zip(expected["com"], [arm_length/2, 0, 0])) < 1e-12, "Generated inertial mass/COM differs")
        formula = [mass*(.06**2+.06**2)/12, mass*(arm_length**2+.06**2)/12, mass*(arm_length**2+.06**2)/12]
        require(max(abs(expected["inertia"][i][j]-(formula[i] if i == j else 0)) for i in range(3) for j in range(3)) < 1e-12, "Generated box inertia is not about its declared COM")
    elbow = robot.find("joint[@name='lesson23_elbow']")
    require(numbers(elbow.find("origin").get("xyz")) == [length, 0, 0]
            and numbers(elbow.find("origin").get("rpy")) == [0, 0, .15], "Xacro length did not propagate to the elbow origin")
    tool = robot.find("link[@name='tool_link']")
    require(float(tool.find("inertial/mass").get("value")) == .02
            and float(tool.find("collision/geometry/sphere").get("radius")) == .025, "Conditional generation changed the physical tool")
    tool_tensor = inertia_matrix(tool.find("inertial/inertia"))
    require(max(abs(tool_tensor[i][j]-(.4*.02*.025**2 if i == j else 0)) for i in range(3) for j in range(3)) < 1e-12, "Tool sphere inertia is incorrect")
    extension = robot.find("gazebo[@reference='lesson23_tool_fixed']/preserveFixedJoint")
    require((extension is not None and extension.text.strip().lower() == "true") is preserve, "Conditional preserveFixedJoint generation differs")
    model = ET.parse(str(sdf)).getroot().find("model")
    require(model is not None, "Actual gz sdf conversion did not produce a model")
    links = {node.get("name"): node for node in model.findall("link")}
    for joint_name, limit in (("lesson23_shoulder", 1.4), ("lesson23_elbow", 1.2)):
        source_joint = robot.find("joint[@name='"+joint_name+"']")
        converted_joint = model.find("joint[@name='"+joint_name+"']")
        require(source_joint.get("type") == converted_joint.get("type") == "revolute"
                and numbers(source_joint.find("axis").get("xyz")) == [0, 0, 1]
                and numbers(converted_joint.findtext("axis/xyz")) == [0, 0, 1], "URDF/native conversion changed the moving joint type/axis")
        require(float(source_joint.find("limit").get("upper")) == float(converted_joint.findtext("axis/limit/upper")) == limit
                and float(source_joint.find("limit").get("lower")) == float(converted_joint.findtext("axis/limit/lower")) == -limit,
                "URDF/native conversion changed the declared joint limits")
    required_names = {"base_link", "arm1", "arm2"} | ({"tool_link"} if preserve else set())
    require(required_names == set(links), "Converted fixed-joint physical link topology differs")
    require(required_names == set(physical["link_properties"]), "Actual spawned physical link topology differs")
    composite = {}
    for name in required_names:
        members = ["arm2", "tool_link"] if name == "arm2" and not preserve else [name]
        expected = aggregate_inertia(robot, members, name)
        inertial = links[name].find("inertial")
        pose = numbers(inertial.findtext("pose", "0 0 0 0 0 0"), 6)
        frame = transform(pose[:3], pose[3:])
        tensor = rotated_inertia(inertia_matrix(inertial.find("inertia"), sdf=True), frame)
        converted = {"mass": float(inertial.findtext("mass")), "com": pose[:3], "inertia": tensor}
        compare_inertia(converted["mass"], converted["com"], tensor, expected, "converted "+name, serialized=True)
        actual = physical["link_properties"][name]
        require(actual["gravity_mode"] is False, "Actual physical link unexpectedly has gravity enabled")
        com = actual["com"]
        fields = ET.Element("inertia", {key: str(actual[key]) for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")})
        tensor = rotated_inertia(inertia_matrix(fields), matrix_from_pose(com))
        actual_center = [com["position"][axis] for axis in ("x", "y", "z")]
        compare_inertia(actual["mass"], actual_center, tensor, expected, "actual vs theory "+name, serialized=True)
        compare_inertia(actual["mass"], actual_center, tensor, converted, "actual vs converted "+name)
        require(len(links[name].findall("collision")) == len(members), "Converted collision geometry disappeared during fixed-link reduction")
        composite[name] = expected
    for path in (urdf, sdf, workspace/"build/arm_lab.json"):
        shutil.copy2(str(path), str(artifacts/(label+"-"+path.name)))
    return robot, {"source_record": record, "physical_names": sorted(required_names), "inertials": composite,
                   "converted_precision_rule": "Six-significant-digit CLI/spawn conversion: half the final decimal quantum vs theory; actual ROS properties vs converted SDF checked at 1e-8."}


def inspect_native_geometry(session, artifacts, label):
    completed = subprocess.run(["gz", "model", "-w", session.world_name, "-m", "xacro_arm", "-i"],
                               cwd=str(session.workspace), env=session.env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
    (artifacts/(label+"-native-model.txt")).write_text(completed.stdout+completed.stderr, encoding="utf-8")
    require(completed.returncode == 0, "Native actual model geometry query failed")
    native = parsed_model(completed.stdout)
    links = {short_name(first(link, "name")): link for link in native.get("link", [])}
    model = ET.parse(str(session.workspace/"build/arm.sdf")).getroot().find("model")
    checked = []
    for link in model.findall("link"):
        name = link.get("name")
        if name == "world":
            continue
        require(name in links, "Converted link is absent from actual native Model")
        shapes = {short_name(first(item, "name")): item for item in links[name].get("collision", [])}
        require(len(shapes) == len(link.findall("collision")), "Actual native collision count differs from the converted SDF")
        for shape in link.findall("collision"):
            actual = shapes.get(shape.get("name"))
            require(actual is not None, "Actual native collision name is missing")
            geometry, expected = first(actual, "geometry", {}), shape.find("geometry")
            if expected.find("box") is not None:
                actual_size = first(first(geometry, "box", {}), "size", {})
                dimensions = [float(first(actual_size, axis, "nan")) for axis in ("x", "y", "z")]
                predicted = numbers(expected.findtext("box/size"))
            else:
                kind = "sphere" if expected.find("sphere") is not None else "cylinder"
                fields = ("radius",) if kind == "sphere" else ("radius", "length")
                dimensions = [float(first(first(geometry, kind, {}), field, "nan")) for field in fields]
                predicted = [float(expected.findtext(kind+"/"+field)) for field in fields]
            require(all(math.isfinite(a) and abs(a-b) < 1e-8 for a, b in zip(dimensions, predicted)), "Actual native collision dimensions differ")
            checked.append({"link": name, "collision": shape.get("name"), "actual_dimensions": dimensions})
    return checked


def flat_pose(state):
    return {"position": dict(zip(("x", "y", "z"), state["position"])),
            "orientation": dict(zip(("x", "y", "z", "w"), state["orientation"]))}


def inspect_statics(result, robot, physical_names):
    expected_cases = {"zero": [0, 0], "positive_mixed": [.5235987756, -.7853981634], "negative_mixed": [-.4, .6]}
    require(set(result["static_phases"]) == set(expected_cases), "Missing independent 2R geometry case")
    checked = {}
    for name, requested in expected_cases.items():
        phase = result["static_phases"][name]
        require(max(abs(a-b) for a, b in zip(phase["requested_q"], requested)) < 1e-8, "Static configuration differs from the prescribed case")
        actual_q = {joint: phase["joint_readback"][joint]["position"][0] for joint in ("lesson23_shoulder", "lesson23_elbow")}
        require(max(abs(actual_q[joint]-value) for joint, value in zip(("lesson23_shoulder", "lesson23_elbow"), requested)) < 1e-4,
                "Actual joint readbacks differ from the stage-boundary configuration request")
        expected = forward_kinematics(robot, actual_q)
        errors = {}
        for link in physical_names:
            row = phase["link_readback"][link]
            require(row["host_monotonic_before_ns"] <= row["host_monotonic_after_ns"]
                    and row["world_time_before_ns"] == row["world_time_after_ns"], "Static link transform was not measured in a paused world")
            state = row["link_state"]
            require(all(math.isfinite(value) for key in ("position", "orientation", "linear", "angular") for value in state[key]), "Non-finite actual link state")
            error, angle = pose_error(matrix_from_pose(flat_pose(state)), expected[link])
            require(error < 2e-5 and angle < 2e-5, "Actual link frame disagrees with independent URDF-origin/axis FK: "+link)
            errors[link] = {"position_error_m": error, "rotation_error_rad": angle}
        checked[name] = {"actual_q": actual_q, "link_errors": errors}
    return checked


def inspect_dynamics(result):
    phases = result["dynamic_phases"]
    require(set(phases) == {"motion_positive", "motion_negative", "upper_limit"}, "Missing finite-effort direction/limit experiment")
    checked = {}
    for name, effort, duration in (("motion_positive", .03, 200000000), ("motion_negative", -.03, 200000000), ("upper_limit", .12, 350000000)):
        phase, command = phases[name], phases[name]["command"]
        require(command["joint_name"] == "xacro_arm::lesson23_elbow" and command["raw_clear_name"] == "lesson23_elbow"
                and command["effort_nm"] == effort and command["duration_ns"] == duration
                and command["requested_end_time_ns"]-command["requested_start_time_ns"] == duration,
                "Finite effort request/body scope differs from the actual experiment")
        require(phase["initial"]["paused"] is True and phase["final"]["paused"] is True
                and command["scheduling_world_time_ns"] == phase["initial"]["world_time_ns"]
                and command["requested_start_time_ns"]-command["scheduling_world_time_ns"] == 50000000,
                "Future finite effort was not scheduled from the actual paused stage boundary")
        rows = phase["samples"]
        require(len(rows) >= 10, "Finite joint response needs ten real joint-feedback samples")
        stamps = [(row["world_time_before_ns"]+row["world_time_after_ns"])/2 for row in rows]
        require(rows[-1]["world_time_after_ns"] >= command["requested_end_time_ns"]+100000000, "Finite effort observation ended before expiry/coast")
        require(all(b >= a for a, b in zip(stamps, stamps[1:])), "Joint observation simulation brackets regressed")
        for row in rows:
            require(row["host_monotonic_before_ns"] <= row["host_monotonic_after_ns"]
                    and row["world_time_before_ns"] <= row["world_time_after_ns"], "Actual joint readback brackets are reversed")
            for joint, limit in (("lesson23_shoulder", 1.4), ("lesson23_elbow", 1.2)):
                actual = row["joint_readback"][joint]
                require(len(actual["position"]) == len(actual["rate"]) == 1
                        and all(math.isfinite(value) for field in ("position", "rate") for value in actual[field]), "Invalid moving-joint feedback")
                require(abs(actual["position"][0]) <= limit+.02, "Actual joint exceeded its declared physical limit")
        angles = [row["joint_readback"]["lesson23_elbow"]["position"][0] for row in rows]
        rates = [row["joint_readback"]["lesson23_elbow"]["rate"][0] for row in rows]
        if name == "upper_limit":
            end = command["requested_end_time_ns"]
            tail = [(angle, rate) for stamp, angle, rate in zip(stamps, angles, rates)
                    if end-60000000 <= stamp <= end]
            require(len(tail) >= 3 and 1.17 <= max(angles) <= 1.22, "Positive effort never actually reached the upper-limit region")
            mean_error = sum(abs(angle-1.2) for angle, _ in tail)/len(tail)
            maximum_rate = max(abs(rate) for _, rate in tail)
            require(mean_error <= .02 and maximum_rate <= .2, "Actual near-limit response failed to settle at the elbow stop")
            checked[name] = {"maximum_actual_q2_rad": max(angles), "tail_sample_count": len(tail), "tail_mean_limit_error_rad": mean_error, "tail_maximum_abs_q2_rate_rad_s": maximum_rate}
        else:
            sign = 1 if effort > 0 else -1
            initial = phase["initial"]["joint_readback"]["lesson23_elbow"]["position"][0]
            final = phase["final"]["joint_readback"]["lesson23_elbow"]["position"][0]
            delta, peak = sign*(final-initial), max(sign*value for value in rates)
            require(delta >= .01 and peak >= .05, "Finite signed effort did not produce a real signed joint response")
            checked[name] = {"signed_actual_delta_q2_rad": delta, "signed_actual_peak_q2_rate_rad_s": peak,
                             "sample_count": len(rows), "simulation_coverage_s": (stamps[-1]-stamps[0])/1e9}
    return checked


def input_hashes(workspace):
    result = hashes(workspace)
    for name in ("build/arm.urdf", "build/arm.sdf", "build/arm_lab.json"):
        result[name] = sha(workspace/name)
    return result


def verify_case(session, artifacts, label, length, preserve, expected=True, diagnosis=None):
    before = snapshot(session, artifacts, label+"-entry")
    fingerprints = input_hashes(session.workspace)
    for name in ("world.sdf", "arm_settings.json", "build/arm.urdf", "build/arm.sdf", "build/arm_lab.json"):
        shutil.copy2(str(session.workspace/name), str(artifacts/(label+"-input-"+Path(name).name)))
    output = artifacts/(label+".json")
    status = run(session, [sys.executable, str(session.workspace/"verify.py"), "--report", str(output)], artifacts/(label+".stdout.log"), timeout=120)
    result = json.loads(output.read_text())
    require((status == 0) is expected and result["passed"] is expected, label+": unexpected verifier status: "+str(result.get("error")))
    if diagnosis:
        require(diagnosis.lower() in result.get("error", "").lower(), "Rejected for an unrelated error: "+str(result.get("error")))
    after = snapshot(session, artifacts, label+"-exit")
    same_snapshot(before, after)
    require(input_hashes(session.workspace) == fingerprints, "Verifier changed personal inputs or generated artifacts")
    if expected:
        robot, generated = inspect_generated(session, before, length, preserve, artifacts, label)
        checked = {"generated": generated, "static_geometry": inspect_statics(result, robot, generated["physical_names"]),
                   "dynamic_response": inspect_dynamics(result)}
    else:
        require(not result["trace"] and not result.get("static_phases"), "Invalid input/physical model reached experimental control")
        if before["physics"]["pause"]:
            require(before["world_time_after_ns"] == after["world_time_before_ns"], "Rejected paused experiment advanced physics")
        checked = {"diagnosis": result["error"]}
    print(json.dumps({"scenario": label, "passed": True, "raw_report": str(output)}), flush=True)
    return {"name": label, "passed": True, "raw_report": str(output), "input_sha256": fingerprints, "checked": checked}


def actual_fault(session, artifacts, operation):
    result = worker_result(session, artifacts, operation, operation)
    field, value, diagnosis = ("mass", 2.0, "mass") if operation == "bad_mass" else ("gravity_mode", True, "gravity")
    require(result["fault_readback"][field] == value, "Physical fault setter was not independently read back")
    raw = result["course_report"]
    require(result["course_returncode"] != 0 and raw["passed"] is False and diagnosis in raw.get("error", "").lower(), "Invalid actual physical property was not diagnosed")
    require(not raw["trace"] and not raw.get("static_phases"), "Invalid actual body reached experimental manipulation")
    same_snapshot(result["before"], result["after_restore"])
    original = result["before"]["link_properties"]["arm1"]
    restored = result["after_restore"]["link_properties"]["arm1"]
    require(original == restored, "Original real physical properties were not restored exactly")
    return {"name": operation, "passed": True, "fault_readback": result["fault_readback"], "diagnosis": raw["error"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--worker", choices=("snapshot", "seed", "bad_mass", "bad_gravity"))
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker, args.report)
    output = args.report.resolve()
    artifacts = output.with_name(output.stem+"-artifacts")
    artifacts.mkdir(parents=True, exist_ok=True)
    lessons, errors = load_catalog(ROOT)
    found = [lesson for lesson in lessons if lesson.key == "classic11-noetic:23"]
    require(len(found) == 1, "Expected L23: "+repr(errors))
    lesson, sessions, started = found[0], [], time.monotonic()
    source = hashes(lesson.folder)
    report = {"passed": False, "lesson": lesson.key, "source_sha256": source, "scenarios": []}
    with tempfile.TemporaryDirectory(prefix="gazeboteach-l23-") as temporary:
        try:
            for label, length, preserve, paused in (("baseline", .6, True, True), ("length_only", .8, True, False), ("lump_only", .6, False, True)):
                storage = Storage(Path(temporary)/label)
                workspace = storage.prepare(lesson)
                if label != "baseline":
                    save(workspace/"arm_settings.json", {"length1_m": length, "preserve_tool": preserve})
                session = LessonSession(ROOT, lesson, storage=storage)
                sessions.append((label, session))
                session.start(gui=False, terminals=False)
                session.pause(paused)
                if label == "lump_only":
                    snapshot(session, artifacts, "nonzero_entry_q", "seed")
                physical = snapshot(session, artifacts, label+"-loaded")
                require(physical["physics"]["time_step"] == .001 and physical["model_properties"]["is_static"] is False, "Actual articulated model/physics configuration differs")
                report["scenarios"].append(verify_case(session, artifacts, label, length, preserve))
                report["scenarios"][-1]["native_collisions"] = inspect_native_geometry(session, artifacts, label)
                if label == "lump_only":
                    settings, generated = workspace/"arm_settings.json", workspace/"build/arm.urdf"
                    original_settings, original_generated = settings.read_bytes(), generated.read_bytes()
                    try:
                        save(settings, {"length1_m": .8, "preserve_tool": preserve})
                        report["scenarios"].append(verify_case(session, artifacts, "saved_without_reload", length, preserve, False, "reload"))
                    finally:
                        settings.write_bytes(original_settings)
                    try:
                        generated.write_bytes(original_generated+b"\n<!-- independent derived-artifact drift probe -->\n")
                        report["scenarios"].append(verify_case(session, artifacts, "generated_artifact_drift", length, preserve, False, "generated"))
                    finally:
                        generated.write_bytes(original_generated)
                    for operation in ("bad_mass", "bad_gravity"):
                        report["scenarios"].append(actual_fault(session, artifacts, operation))
                    report["scenarios"].append(verify_case(session, artifacts, "exact_inputs_and_properties_repaired", length, preserve))
                session.stop(reason="L23 independent case completed")
            report["passed"] = True
        except Exception as error:
            report["error"] = type(error).__name__+": "+str(error)
        finally:
            report["cleanup_errors"] = []
            for label, session in reversed(sessions):
                try:
                    session.stop(reason="L23 final acceptance cleanup")
                    shutil.copy2(str(session.log_path), str(artifacts/(label+"-runtime.log")))
                    shutil.copy2(str(session.session_dir/"session.json"), str(artifacts/(label+"-session.json")))
                    for index, path in enumerate(sorted((session.session_dir/"ros-home/log").rglob("*"))):
                        if path.is_file():
                            shutil.copy2(str(path), str(artifacts/(label+"-ros-"+str(index)+"-"+path.name)))
                except Exception as error:
                    report["cleanup_errors"].append(label+": "+repr(error))
            report["leftovers"] = {session.token: live_token_processes(session.token) for _, session in sessions}
            report["source_templates_unchanged"] = source == hashes(lesson.folder)
            report["elapsed_seconds"] = round(time.monotonic()-started, 3)
            if report["cleanup_errors"] or any(report["leftovers"].values()) or not report["source_templates_unchanged"]:
                report["passed"] = False
            output.parent.mkdir(parents=True, exist_ok=True)
            save(output, report)
    print(json.dumps({key: value for key, value in report.items() if key != "scenarios"}, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
