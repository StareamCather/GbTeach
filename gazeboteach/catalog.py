"""Versioned, validated lesson packages; independent of Qt and simulator imports."""

import json
import math
import os
import re
import shutil
from pathlib import Path


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(name + " must be a nonempty string")
    return value


class Lesson:
    def __init__(self, folder, data):
        if not isinstance(data, dict):
            raise ValueError("Lesson manifest must be an object")
        self.folder = Path(folder).resolve()
        self.data = dict(data)
        self.id = str(data["id"]).zfill(2)
        if not re.fullmatch(r"[0-9]{2,3}", self.id):
            raise ValueError("Lesson id must be a two/three digit number")
        self.title = _text(data["title"], "title")
        self.module = _text(data.get("module", "课程"), "module")
        self.profile = _text(data.get("profile", "classic11-noetic"), "profile")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", self.profile):
            raise ValueError("Invalid course profile")
        if isinstance(data.get("order", 0), bool):
            raise ValueError("order must be an integer")
        self.order = int(data.get("order", self.id))
        for name in ("requires_ros", "needs_display"):
            if name in data and not isinstance(data[name], bool):
                raise ValueError(name + " must be boolean")
        self.requires_ros = data.get("requires_ros", False)
        attempts = data.get("gui_start_attempts", 1)
        if type(attempts) is not int or not 1 <= attempts <= 3:
            raise ValueError("gui_start_attempts must be an integer within [1,3]")
        for name in ("requires_plugins", "requires_ros_packages", "requires_tools",
                     "prepare", "after_start", "after_startup", "after_gui", "server_args", "gui_args",
                     "objectives", "ros_package_paths"):
            values = data.get(name, [])
            if not isinstance(values, list) or any(not isinstance(x, str) or not x.strip() for x in values):
                raise ValueError(name + " must be a list of strings")
        for name in ("commands", "startup"):
            values = data.get(name, [])
            if not isinstance(values, list):
                raise ValueError(name + " must be a list")
            for item in values:
                if not isinstance(item, dict):
                    raise ValueError(name + " items must be objects")
                _text(item.get("command"), name + ".command")
                _text(item.get("title", item.get("name", "命令")), name + ".title/name")
        for name in ("prepare_timeout", "ready_timeout", "verify_timeout", "gui_ready_timeout"):
            if name in data and (isinstance(data[name], bool) or not isinstance(data[name], (int, float)) or data[name] <= 0):
                raise ValueError(name + " must be a positive duration")
        if data.get("requires_ros_packages") and not self.requires_ros:
            raise ValueError("ROS package requirements need requires_ros=true")
        if "ros_package_paths" in data:
            if not self.requires_ros:
                raise ValueError("ros_package_paths needs requires_ros=true")
            for relative in data["ros_package_paths"]:
                self.file(relative)
        timeout = data.get("after_startup_timeout", 90)
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                or not math.isfinite(timeout) or timeout <= 0):
            raise ValueError("after_startup_timeout must be a positive finite duration")
        if not self.world.is_file():
            raise ValueError("World file does not exist: " + str(self.world))
        if not self.file(data.get("lesson", "lesson.md")).is_file():
            raise ValueError("Lesson document does not exist")
        if "verify" in data and not self.file(data["verify"]).is_file():
            raise ValueError("Declared verification file does not exist")

    @property
    def key(self):
        return self.profile + ":" + self.id

    def file(self, relative):
        _text(relative, "relative file")
        if Path(relative).is_absolute():
            raise ValueError("Lesson files must use relative paths")
        path = (self.folder / relative).resolve()
        if path == self.folder or self.folder not in path.parents:
            raise ValueError("Lesson files must stay inside their course directory")
        return path

    @property
    def world(self):
        return self.file(self.data.get("world", "world.sdf"))

    @property
    def document(self):
        return self.file(self.data.get("lesson", "lesson.md")).read_text(encoding="utf-8")


def load_catalog(root):
    lessons, errors, keys = [], [], set()
    for manifest in sorted((Path(root) / "courses").glob("*/*/lesson.json")):
        try:
            lesson = Lesson(manifest.parent, json.loads(manifest.read_text(encoding="utf-8-sig")))
            if lesson.key in keys:
                raise ValueError("Duplicate lesson id in profile")
            keys.add(lesson.key)
            lessons.append(lesson)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            errors.append("{}: {}".format(manifest, exc))
    return sorted(lessons, key=lambda x: (x.profile, x.order, x.id)), errors


def compatibility(lesson, environment, gui=True, terminals=False):
    """A reason means execution is unavailable; reading remains possible."""
    if lesson.profile == "classic11-noetic":
        if not environment.get("classic_version", "").startswith("11."):
            return "本课需要 Gazebo Classic 11，当前未检测到对应模拟器。"
        if lesson.requires_ros and "noetic" not in environment.get("ros_installed", []):
            return "本课需要 ROS Noetic；独立 Gazebo 课程仍可学习。"
    elif lesson.profile not in environment.get("profiles", []):
        return "当前环境没有与本课匹配的课程运行配置。"
    required = {"gzserver", "gz", "bash"}
    if gui:
        required.add("gzclient")
    if lesson.requires_ros:
        required.add("roscore")
    required.update(lesson.data.get("requires_tools", []))
    tools = environment.get("tools", {})
    def available_tool(name):
        if name in tools:
            return tools[name]
        found = shutil.which(name)
        if not found and lesson.requires_ros:
            installed = Path("/opt/ros/noetic/bin") / name
            if installed.is_file() and os.access(str(installed), os.X_OK):
                found = str(installed)
        return found
    missing = sorted(x for x in required if not available_tool(x))
    if missing:
        return "缺少程序：" + ", ".join(missing)
    if (gui or terminals or lesson.data.get("needs_display", False)) and not environment.get("display"):
        return "本课需要图形会话 DISPLAY；请在 Ubuntu 桌面运行，渲染传感器也可能需要显示环境。"
    if terminals and not (tools.get("gnome-terminal") or tools.get("xterm")):
        return "缺少 gnome-terminal 或 xterm，无法打开课程终端。"
    plugins = set(environment.get("plugins", []))
    missing = [x for x in lesson.data.get("requires_plugins", []) if x not in plugins]
    if missing:
        return "缺少插件：" + ", ".join(missing)
    packages = set(environment.get("ros_packages", []))
    missing = [x for x in lesson.data.get("requires_ros_packages", []) if x not in packages]
    if missing:
        return "缺少 ROS 包：" + ", ".join(missing)
    return None
