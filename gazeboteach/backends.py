"""Execution backend contracts. Docker configuration is real; execution is reserved."""

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import subprocess


def _text(value, name, empty=False):
    if not isinstance(value, str) or any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ValueError(name + " must be a string without control characters")
    if not empty and not value.strip():
        raise ValueError(name + " must not be empty")
    return value


def _host_path(value, name):
    path = Path(_text(value, name)).expanduser()
    if not path.is_absolute():
        raise ValueError(name + " must be an absolute host path")
    path = path.resolve()
    if path.parent == path:
        raise ValueError(name + " must be a directory below the host filesystem root")
    return str(path)


def _container_path(value, name):
    value = _text(value, name)
    path = PurePosixPath(value)
    if (not path.is_absolute() or path == PurePosixPath("/") or value.startswith("//")
            or ".." in path.parts or "\\" in value):
        raise ValueError(name + " must be an absolute container path below / without ..")
    return str(path)


def _mapping(value, allowed, kind):
    if not isinstance(value, dict) or set(value) - set(allowed):
        raise ValueError("Invalid or unknown " + kind + " configuration fields")
    return value


@dataclass(frozen=True)
class MountSpec:
    source: str
    target: str
    read_only: bool = True

    def __post_init__(self):
        object.__setattr__(self, "source", _host_path(self.source, "mount.source"))
        object.__setattr__(self, "target", _container_path(self.target, "mount.target"))
        if type(self.read_only) is not bool:
            raise ValueError("mount.read_only must be boolean")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, value):
        value = _mapping(value, ("source", "target", "read_only"), "mount")
        if not {"source", "target"} <= set(value):
            raise ValueError("mount source and target are required")
        return cls(**value)


@dataclass(frozen=True)
class DockerConnectionConfig:
    image: str
    host_workspace: str
    container_workspace: str = "/workspace/GazeboTeach"
    display: str = ""
    context: str = ""
    container_name: str = ""
    mounts: tuple = field(default_factory=tuple)

    def __post_init__(self):
        image = _text(self.image, "image")
        if len(image) > 255 or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._:/@+\-]*", image) or "//" in image:
            raise ValueError("image must be one image-reference token, not a URL or command")
        object.__setattr__(self, "host_workspace", _host_path(self.host_workspace, "host_workspace"))
        object.__setattr__(self, "container_workspace", _container_path(self.container_workspace, "container_workspace"))
        display = _text(self.display, "display", empty=True)
        if display and not re.fullmatch(r"(?:[A-Za-z0-9_.-]+:|:)\d+(?:\.\d+)?", display):
            raise ValueError("display must be an X11 DISPLAY value, such as :0 or localhost:10.0")
        for name in ("context", "container_name"):
            value = _text(getattr(self, name), name, empty=True)
            if value and not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
                raise ValueError(name + " must be a single Docker identifier")
        if not isinstance(self.mounts, (tuple, list)):
            raise ValueError("mounts must be a list of MountSpec objects")
        mounts = tuple(self.mounts)
        if any(not isinstance(value, MountSpec) for value in mounts):
            raise ValueError("mounts must contain MountSpec objects")
        targets = [self.container_workspace] + [value.target for value in mounts]
        if len(targets) != len(set(targets)):
            raise ValueError("Mount targets must be unique, including container_workspace")
        if any(PurePosixPath(first) in PurePosixPath(second).parents
               for first in targets for second in targets if first != second):
            raise ValueError("Mount targets must not hide or overlap other configured mounts")
        object.__setattr__(self, "mounts", mounts)

    def to_dict(self):
        value = asdict(self)
        value["mounts"] = [mount.to_dict() for mount in self.mounts]
        return {"schema_version": 1, "backend": "docker", **value}

    @classmethod
    def from_dict(cls, value):
        value = dict(_mapping(value, ("schema_version", "backend", "image", "host_workspace",
                                      "container_workspace", "display", "context", "container_name", "mounts"), "Docker"))
        version = value.pop("schema_version", 1)
        if type(version) is not int or version != 1 or value.pop("backend", "docker") != "docker":
            raise ValueError("Unsupported Docker configuration schema/backend")
        if not {"image", "host_workspace"} <= set(value):
            raise ValueError("Docker image and host_workspace are required")
        mounts = value.pop("mounts", [])
        if not isinstance(mounts, list):
            raise ValueError("mounts must be a JSON list")
        return cls(mounts=tuple(MountSpec.from_dict(mount) for mount in mounts), **value)


class ExecutionBackend(ABC):
    """An adapter owns configuration/probing and later session construction."""

    backend_id = ""

    @abstractmethod
    def describe(self):
        raise NotImplementedError

    @abstractmethod
    def validate_configuration(self, value=None):
        raise NotImplementedError

    @abstractmethod
    def create_session(self, root, lesson, *, configuration=None, storage=None, notify=None):
        raise NotImplementedError


class NativeBackend(ExecutionBackend):
    backend_id = "native"

    def describe(self):
        return {"schema_version": 1, "id": self.backend_id, "title": "本机环境",
                "execution_implemented": True, "connection_configurable": False,
                "supported_host": platform.system() == "Linux",
                "message": "使用现有本机实验运行器；每课是否可运行仍由环境与依赖检测决定。"}

    def validate_configuration(self, value=None):
        value = _mapping({} if value is None else value, ("schema_version", "backend"), "native")
        version = value.get("schema_version", 1)
        if type(version) is not int or version != 1 or value.get("backend", "native") != "native":
            raise ValueError("Unsupported native configuration schema/backend")
        return {"schema_version": 1, "backend": "native"}

    def create_session(self, root, lesson, *, configuration=None, storage=None, notify=None):
        self.validate_configuration(configuration)
        from .modern import is_modern
        if is_modern(lesson):
            from .modern_runtime import ModernLessonSession
            return ModernLessonSession(root, lesson, storage=storage, notify=notify)
        from .runtime import LessonSession
        return LessonSession(root, lesson, storage=storage, notify=notify)


class DockerBackend(ExecutionBackend):
    backend_id = "docker"

    def describe(self):
        return {"schema_version": 1, "id": self.backend_id, "title": "Docker接口（预留）",
                "execution_implemented": False, "connection_configurable": True,
                "cli_detection_supported": True, "daemon_checked": False,
                "image_fields": ["image", "host_workspace", "container_workspace", "display",
                                 "context", "container_name", "mounts"],
                "message": "可校验并保存连接配置；当前版本尚未实现容器连接、挂载、显示转发或课程执行。"}

    def validate_configuration(self, value=None):
        if isinstance(value, DockerConnectionConfig):
            return value
        return DockerConnectionConfig.from_dict(value)

    def probe(self, timeout=3):
        """Inspect only the client binary. No daemon/contact/pull/container actions."""
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not 0 < timeout <= 10:
            raise ValueError("Docker client probe timeout must be within (0,10] seconds")
        path = shutil.which("docker")
        result = {"backend": self.backend_id, "execution_available": False,
                  "execution_implemented": False, "daemon_checked": False,
                  "cli": {"installed": bool(path), "path": path, "version": "", "error": ""}}
        if path:
            try:
                process = subprocess.run([path, "--version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                         text=True, errors="replace", timeout=timeout)
                if process.returncode:
                    result["cli"]["error"] = process.stderr.strip()[:2000] or "Docker client version probe failed"
                else:
                    result["cli"]["version"] = process.stdout.strip()[:2000]
            except (OSError, subprocess.TimeoutExpired) as error:
                result["cli"]["error"] = str(error)
        result["message"] = "检测仅确认Docker命令行程序；未检查daemon、镜像或容器，当前版本不提供Docker执行。"
        return result

    def create_session(self, root, lesson, *, configuration=None, storage=None, notify=None):
        self.validate_configuration(configuration)
        raise NotImplementedError("Docker连接与课程执行尚未实现；配置校验不代表容器可用。")


def get_backend(backend_id):
    constructors = {"native": NativeBackend, "docker": DockerBackend}
    if backend_id not in constructors:
        raise ValueError("Unknown execution backend: " + str(backend_id))
    return constructors[backend_id]()


def backend_descriptors():
    return [get_backend(name).describe() for name in ("native", "docker")]
