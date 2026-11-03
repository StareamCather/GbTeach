"""Installed/active discovery and per-lesson environments without global changes."""

import os
import importlib.util
import platform
import re
import shutil
import subprocess
import shlex
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path


def _output(arguments, timeout=4, accepted_codes=(0,), env=None):
    try:
        result = subprocess.run(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=timeout, errors="replace", env=env)
        return result.stdout.strip() if result.returncode in accepted_codes else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


SDK_PACKAGES = ("gazebo", "roscpp", "sensor_msgs", "geometry_msgs", "std_msgs", "OGRE",
                "OGRE-Paging", "OGRE-Terrain", "OGRE-RTShaderSystem", "Qt5Widgets", "Qt5Core")


def detect_development_capabilities(tools, environment):
    """One bounded snapshot, shared by every manifest; never compile a course here."""
    pkg, compiler = tools.get("pkg-config"), tools.get("g++")
    packages = {}
    flags = {}
    for package in SDK_PACKAGES:
        version = _output([pkg, "--modversion", package], env=environment) if pkg else ""
        flag_text = _output([pkg, "--cflags", "--libs", package], env=environment) if version else ""
        packages[package] = {"version": version, "available": bool(version and flag_text),
                             "error": "" if version and flag_text else "pkg-config 未找到包或完整依赖"}
        flags[package] = flag_text
    sdk = {"headers_available": False, "error": "缺少 gazebo.pc 或 g++"}
    if compiler and packages["gazebo"]["available"]:
        # pkg-config can outlive a removed header package. Parse actual public
        # headers with the selected compiler, without producing an executable.
        command = [compiler, "-std=c++17", "-x", "c++", "-fsyntax-only", "-"]
        command += [flag for flag in shlex.split(flags["gazebo"]) if flag.startswith(("-I", "-D", "-pthread"))]
        try:
            checked = subprocess.run(command, input="#include <gazebo/gazebo.hh>\n", env=environment,
                                     text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=12)
            sdk = {"headers_available": checked.returncode == 0,
                   "error": checked.stderr.strip()[-1800:] if checked.returncode else ""}
        except (OSError, subprocess.TimeoutExpired) as error:
            sdk["error"] = str(error)
    qt = {"available": False, "path": "", "version": "", "error": "Qt5Core 开发包未找到"}
    if pkg and packages["Qt5Core"]["available"]:
        host_bins = _output([pkg, "--variable=host_bins", "Qt5Core"], env=environment)
        moc = Path(host_bins) / "moc" if host_bins else None
        version_text = _output([str(moc), "-v"], env=environment) if moc and moc.is_file() else ""
        match = re.search(r"\bmoc\s+(5\.\d+\.\d+)\b", version_text, re.I)
        version = match.group(1) if match else ""
        available = bool(version and version == packages["Qt5Core"]["version"])
        qt = {"available": available, "path": str(moc) if moc else "", "version": version,
              "error": "" if available else "Qt5Core host_bins/moc 不可执行或版本不匹配：" + version_text}
    libraries = {}
    for name in ("gazebo_gui", "gazebo_ode", "gazebo_util"):
        candidate = _output([compiler, "-print-file-name=lib" + name + ".so"], env=environment) if compiler else ""
        libraries[name] = candidate if candidate and Path(candidate).is_file() else None
    exact, selected = Path("/usr/bin/gzserver-11.15.1"), tools.get("gzserver")
    try:
        validated = bool(selected and exact.is_file() and os.path.samefile(str(exact), selected))
    except OSError:
        validated = False
    native = {"validated": validated, "path": selected,
              "reason": "" if validated else "当前 gzserver 未解析到 /usr/bin/gzserver-11.15.1"}
    control_supported = bool(validated and sdk["headers_available"] and packages["gazebo"]["version"] == "11.15.1")
    return {"pkg_config": packages, "sdk": sdk, "qt_moc": qt, "libraries": libraries,
            "native_server": native,
            "python_modules": {"numpy": importlib.util.find_spec("numpy") is not None},
            "control_capability": {"preferred_backend": "classic11-sdk-persistent" if control_supported else "legacy-cli",
                                   "persistent_supported": control_supported,
                                   "source": "只读开发能力探测；实际会话构建与后端记录见运行日志",
                                   "reason": "" if control_supported else "可选精确 SDK 控制增强不可用；基础世界仍可使用 legacy-cli"}}


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


def noetic_base_environment(environment):
    """Strip active ROS 2 prefixes from a child environment before ROS 1 setup.

    Preserve unrelated user paths and ROS 1 overlays. An AMENT/COLCON prefix
    identifies a ROS 2 overlay even when it lives outside /opt/ros.
    """
    result = dict(environment)
    roots = set()
    for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        roots.update(Path(value).resolve() for value in result.get(name, "").split(os.pathsep) if value)
    active = result.get("ROS_DISTRO", "")
    if active and active != "noetic" and (result.get("ROS_VERSION") == "2" or (Path("/opt/ros") / active / "bin/ros2").is_file()):
        roots.add((Path("/opt/ros") / active).resolve())
    for directory in Path("/opt/ros").glob("*"):
        if (directory / "bin/ros2").is_file():
            roots.add(directory.resolve())
    for name in ("PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH", "ROS_PACKAGE_PATH",
                 "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH", "GAZEBO_RESOURCE_PATH"):
        if name not in result:
            continue
        kept = []
        for value in result[name].split(os.pathsep):
            path = Path(value).resolve() if value else None
            if path is None or not any(path == root or root in path.parents for root in roots):
                kept.append(value)
        result[name] = os.pathsep.join(kept)
    for name in ("ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY",
                 "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "AMENT_PREFIX_PATH",
                 "AMENT_CURRENT_PREFIX", "COLCON_PREFIX_PATH", "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        result.pop(name, None)
    return result


def detect_environment():
    names = ["gazebo", "gzserver", "gzclient", "gz", "ign", "roscore", "roslaunch",
             "rospack", "rosrun", "gnome-terminal", "xterm", "bash", "cmake", "make",
             "g++", "setsid", "gedit", "xdg-open", "python3", "ros2", "pkg-config", "python3-config",
             "rosparam", "rosservice", "rostopic", "rosnode", "rosmsg", "rviz", "xacro", "colcon"]
    tools = {name: shutil.which(name) for name in names}
    incomplete, errors = [], []
    # Classic 11.15's version-only path returns 255 on the verified Ubuntu build.
    classic_text = _output([tools["gazebo"], "--version"], accepted_codes=(0, 255)) if tools["gazebo"] else ""
    match = re.search(r"Gazebo(?:\s+multi-robot\s+simulator,)?\s+version\s+(\d+\.\d+(?:\.\d+)?)", classic_text, re.I)
    classic = match.group(1) if match else ""
    classic_installations = []
    active_classic = str(Path(tools["gazebo"]).resolve()) if tools["gazebo"] else ""
    if classic:
        classic_installations.append({"path": active_classic, "version": classic, "active": True})
    seen = {active_classic}
    for directory in (Path("/usr/bin"), Path("/usr/local/bin")):
        for candidate in sorted(directory.glob("gazebo-*")):
            if not re.fullmatch(r"gazebo-\d+(?:\.\d+)*", candidate.name) or not candidate.is_file():
                continue
            path = str(candidate.resolve())
            if path in seen:
                continue
            seen.add(path)
            text = _output([path, "--version"], accepted_codes=(0, 255))
            version = re.search(r"Gazebo(?:\s+multi-robot\s+simulator,)?\s+version\s+(\d+\.\d+(?:\.\d+)?)", text, re.I)
            if version:
                classic_installations.append({"path": path, "version": version.group(1), "active": False})
    if tools["gazebo"] and not classic:
        incomplete.append("classic_version")
        errors.append("gazebo --version 未返回可识别的 Classic 版本（可能超时或执行失败）。")
    modern = []
    for binary, arguments in (("gz", ["sim", "--versions"]), ("ign", ["gazebo", "--versions"])):
        if tools[binary]:
            output = _output([tools[binary]] + arguments, timeout=3)
            if re.fullmatch(r"[0-9.,\s]+", output or "x"):
                modern.append({"command": binary, "path": tools[binary], "versions": output})
    ros = sorted(p.name for p in Path("/opt/ros").glob("*") if (p / "setup.bash").is_file())
    if "noetic" in ros:
        for name in names:
            installed = Path("/opt/ros/noetic/bin") / name
            if not tools[name] and installed.is_file() and os.access(str(installed), os.X_OK):
                tools[name] = str(installed)
    packages, package_versions = [], {}
    if "noetic" in ros and tools["bash"]:
        package_text = _output([tools["bash"], "--noprofile", "--norc", "-c",
                                '. /opt/ros/noetic/setup.bash && rospack list-names'], timeout=12,
                               env=noetic_base_environment(os.environ))
        packages = sorted(set(package_text.splitlines()))
        if not packages:
            incomplete.append("ros_packages")
            errors.append("Noetic 已安装，但 rospack list-names 未返回包列表。")
    plugin_paths = [p for p in os.environ.get("GAZEBO_PLUGIN_PATH", "").split(os.pathsep) if p]
    plugin_paths += ["/usr/lib/x86_64-linux-gnu/gazebo-11/plugins", "/opt/ros/noetic/lib",
                     "/usr/local/lib", "/usr/lib/aarch64-linux-gnu/gazebo-11/plugins"]
    plugins = set()
    for folder in plugin_paths:
        try:
            plugins.update(p.name for p in Path(folder).glob("*.so") if p.is_file())
        except OSError as error:
            incomplete.append("plugins")
            errors.append("插件目录无法读取：{} ({})".format(folder, error))
    profiles = ["classic11-noetic"] if classic.startswith("11.") else []
    os_name = platform.platform()
    release = Path("/etc/os-release")
    if release.exists():
        for line in release.read_text(encoding="utf-8").splitlines():
            if line.startswith("PRETTY_NAME="):
                os_name = line.split("=", 1)[1].strip('"')
    development_env = os.environ.copy()
    if "noetic" in ros and tools["bash"]:
        try:
            development_env = _load_setup(Path("/opt/ros/noetic/setup.bash"), noetic_base_environment(development_env))
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            errors.append("开发包环境加载失败：" + str(error))
        if tools.get("rospack"):
            for package in ("gazebo_plugins",):
                directory = _output([tools["rospack"], "find", package], env=development_env)
                manifest = Path(directory) / "package.xml" if directory else None
                try:
                    if manifest and manifest.is_file():
                        metadata = ET.parse(str(manifest))
                        if metadata.findtext("name", "").strip() != package:
                            raise ValueError("rospack 包名与清单不一致")
                        package_versions[package] = metadata.findtext("version", "").strip()
                except (OSError, ValueError, ET.ParseError) as error:
                    incomplete.append("ros_package_versions")
                    errors.append("ROS 包版本不可读：" + str(error))
    capabilities = detect_development_capabilities(tools, development_env)
    result = {
        "os": os_name, "architecture": platform.machine(), "python": platform.python_version(),
        "classic_version": classic, "modern_simulators": modern, "profiles": profiles,
        "classic_active_executable": active_classic, "classic_installations": classic_installations,
        "ros_installed": ros, "ros_active": os.environ.get("ROS_DISTRO", ""),
        "display": os.environ.get("DISPLAY", ""), "tools": tools,
        "plugin_paths": plugin_paths, "plugins": sorted(plugins), "ros_packages": packages,
        "ros_package_versions": package_versions,
        "course_profile": profiles[0] if profiles else None,
        "ros_active_version": os.environ.get("ROS_VERSION", ""),
        "ros_families": {name: ("ROS 2" if (Path("/opt/ros") / name / "bin/ros2").exists()
                                 else "ROS 1" if (Path("/opt/ros") / name / "bin/roscore").exists()
                                 else "未确认") for name in ros},
        "container_hint": Path("/.dockerenv").exists(),
        "incomplete_checks": sorted(set(incomplete)), "detection_errors": errors,
    }
    result.update(capabilities)
    from .modern import detect_modern, modern_detection_failures
    result['modern'] = detect_modern(tools, ros, modern)
    result['profiles'].extend(result['modern']['profiles'])
    modern_checks, modern_errors = modern_detection_failures(result['modern'])
    result['incomplete_checks'] = sorted(set(result['incomplete_checks'] + modern_checks))
    result['detection_errors'] = list(dict.fromkeys(result['detection_errors'] + modern_errors))
    if not result['course_profile'] and result['profiles']:
        result['course_profile'] = result['profiles'][0]
    return result


def lesson_environment(root, workspace, lesson, gazebo_port, ros_port, session_token=None):
    """Source only this lesson's ROS installation, then isolate every transport."""
    from .modern import is_modern, modern_lesson_environment
    if is_modern(lesson):
        return modern_lesson_environment(root, workspace, lesson, session_token or uuid.uuid4().hex)
    root, workspace = Path(root), Path(workspace)
    environment = noetic_base_environment(os.environ) if lesson.requires_ros else os.environ.copy()
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
