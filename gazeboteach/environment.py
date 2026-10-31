"""Installed/active discovery and per-lesson environments without global changes."""

import os
import platform
import re
import shutil
import subprocess
import uuid
from pathlib import Path


def _output(arguments, timeout=4, accepted_codes=(0,)):
    try:
        result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout, errors="replace")
        return result.stdout.strip() if result.returncode in accepted_codes else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _load_setup(setup, environment):
    """Load an installed setup into a child shell; never modify the app environment."""
    result = subprocess.run(["bash", "--noprofile", "--norc", "-c",
                             '. "$1" >/dev/null && env -0', "gazeboteach", str(setup)],
                            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=15)
    if result.returncode:
        raise RuntimeError("加载运行环境失败（{}）：{}".format(
            setup, result.stderr.decode("utf-8", "replace")))
    return dict(item.split("=", 1) for item in
                result.stdout.decode("utf-8", "replace").split("\0") if "=" in item)


def detect_environment():
    names = ["gazebo", "gzserver", "gzclient", "gz", "ign", "roscore", "roslaunch",
             "rospack", "rosrun", "gnome-terminal", "xterm", "bash", "cmake", "make",
             "g++", "setsid", "gedit", "xdg-open", "python3"]
    tools = {name: shutil.which(name) for name in names}
    # Classic 11.15's version-only path returns 255 on the verified Ubuntu build.
    classic_text = _output([tools["gazebo"], "--version"], accepted_codes=(0, 255)) if tools["gazebo"] else ""
    match = re.search(r"Gazebo(?:\s+multi-robot\s+simulator,)?\s+version\s+(\d+\.\d+(?:\.\d+)?)", classic_text, re.I)
    classic = match.group(1) if match else ""
    modern = []
    for binary, arguments in (("gz", ["sim", "--versions"]), ("ign", ["gazebo", "--versions"])):
        if tools[binary]:
            output = _output([tools[binary]] + arguments, timeout=3)
            if re.fullmatch(r"[0-9.,\s]+", output or "x"):
                modern.append({"command": binary, "versions": output})
    ros = sorted(p.name for p in Path("/opt/ros").glob("*") if (p / "setup.bash").is_file())
    if "noetic" in ros:
        for name in names:
            installed = Path("/opt/ros/noetic/bin") / name
            if not tools[name] and installed.is_file() and os.access(str(installed), os.X_OK):
                tools[name] = str(installed)
    packages = []
    if "noetic" in ros and tools["bash"]:
        package_text = _output([tools["bash"], "--noprofile", "--norc", "-c",
                                '. /opt/ros/noetic/setup.bash && rospack list-names'], timeout=12)
        packages = sorted(set(package_text.splitlines()))
    plugin_paths = [p for p in os.environ.get("GAZEBO_PLUGIN_PATH", "").split(os.pathsep) if p]
    plugin_paths += ["/usr/lib/x86_64-linux-gnu/gazebo-11/plugins", "/opt/ros/noetic/lib",
                     "/usr/local/lib", "/usr/lib/aarch64-linux-gnu/gazebo-11/plugins"]
    plugins = sorted({p.name for folder in plugin_paths for p in Path(folder).glob("*.so") if p.is_file()})
    profiles = ["classic11-noetic"] if classic.startswith("11.") else []
    os_name = platform.platform()
    release = Path("/etc/os-release")
    if release.exists():
        for line in release.read_text(encoding="utf-8").splitlines():
            if line.startswith("PRETTY_NAME="):
                os_name = line.split("=", 1)[1].strip('"')
    return {
        "os": os_name, "architecture": platform.machine(), "python": platform.python_version(),
        "classic_version": classic, "modern_simulators": modern, "profiles": profiles,
        "ros_installed": ros, "ros_active": os.environ.get("ROS_DISTRO", ""),
        "display": os.environ.get("DISPLAY", ""), "tools": tools,
        "plugin_paths": plugin_paths, "plugins": plugins, "ros_packages": packages,
        "course_profile": profiles[0] if profiles else None,
    }


def lesson_environment(root, workspace, lesson, gazebo_port, ros_port, session_token=None):
    """Source only this lesson's ROS installation, then isolate every transport."""
    root, workspace = Path(root), Path(workspace)
    environment = os.environ.copy()
    inherited_package_paths = environment.get("ROS_PACKAGE_PATH", "")
    # Setting a lesson resource path replaces Gazebo's internal defaults. Load the
    # installed shader/material/model/OGRE paths before adding our local assets.
    gazebo_setups = [Path("/usr/share/gazebo-11/setup.sh"), Path("/usr/share/gazebo/setup.sh"),
                    Path("/usr/local/share/gazebo-11/setup.sh"), Path("/usr/local/share/gazebo/setup.sh")]
    for setup in gazebo_setups:
        if setup.is_file():
            environment.update(_load_setup(setup, environment))
            break
    if lesson.requires_ros:
        setup = Path("/opt/ros/noetic/setup.bash")
        if not setup.is_file():
            raise RuntimeError("ROS Noetic 环境不存在：" + str(setup))
        environment.update(_load_setup(setup, environment))
    package_paths = []
    workspace_root = workspace.resolve()
    for relative in lesson.data.get("ros_package_paths", []):
        if not lesson.requires_ros or Path(relative).is_absolute():
            raise ValueError("ros_package_paths needs relative paths and requires_ros=true")
        directory = (workspace_root / relative).resolve()
        # Recheck the personal copy: a student may have replaced a source
        # directory with a symlink since its manifest was validated.
        if directory == workspace_root or workspace_root not in directory.parents:
            raise ValueError("ROS package paths must stay inside the personal workspace: " + relative)
        package_paths.append(str(directory))
    if package_paths:
        # A distribution setup can replace ROS_PACKAGE_PATH. Keep both its
        # installed packages and the caller's other package roots in this session.
        for value in (environment.get("ROS_PACKAGE_PATH", ""), inherited_package_paths):
            package_paths.extend(path for path in value.split(os.pathsep)
                                 if path and path not in package_paths)
        environment["ROS_PACKAGE_PATH"] = os.pathsep.join(package_paths)
    token = session_token or uuid.uuid4().hex
    environment.update({
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(gazebo_port),
        "GAZEBO_IP": "127.0.0.1", "ROS_MASTER_URI": "http://127.0.0.1:{}".format(ros_port),
        "ROS_IP": "127.0.0.1", "ROS_HOSTNAME": "127.0.0.1",
        "IGN_PARTITION": "gazeboteach-" + token,
        "GAZEBOTEACH_SOURCE_ROOT": str(root), "GAZEBOTEACH_WORKSPACE": str(workspace),
        "GAZEBOTEACH_LESSON_ID": lesson.id, "GAZEBOTEACH_SESSION_TOKEN": token,
        "PYTHONUNBUFFERED": "1", "QT_X11_NO_MITSHM": "1", "GAZEBO_MODEL_DATABASE_URI": "",
    })
    paths = {
        "GAZEBO_MODEL_PATH": [workspace / "models", root / "models"],
        "GAZEBO_RESOURCE_PATH": [workspace, root / "resources"],
        "GAZEBO_PLUGIN_PATH": [workspace / "build", workspace / "build/lib", root / "build", root / "build/lib"],
    }
    for name, entries in paths.items():
        environment[name] = os.pathsep.join([str(p) for p in entries] +
                                           ([environment[name]] if environment.get(name) else []))
    return environment
