"""Session-owned Gazebo supervision; never clean processes by application name."""

import datetime
import json
import os
import re
import shlex
import shutil
import signal
import socket
import stat
import struct
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
import xmlrpc.client
from pathlib import Path

from .catalog import compatibility
from .environment import detect_environment, lesson_environment
from .storage import Storage


def allocate_ports():
    sockets = []
    try:
        for _ in range(2):
            sock = socket.socket()
            sock.bind(("127.0.0.1", 0))
            sockets.append(sock)
        return tuple(sock.getsockname()[1] for sock in sockets)
    finally:
        for sock in sockets:
            sock.close()


def _process_info(pid):
    """Read PID identity without treating a recycled number as ownership."""
    try:
        directory = Path("/proc") / str(pid)
        if directory.stat().st_uid != os.getuid():
            return None
        text = (directory / "stat").read_text()
        fields = text[text.rfind(")") + 2:].split()
        return {"pid": int(pid), "state": fields[0], "pgid": int(fields[2]),
                "sid": int(fields[3]), "start_time": fields[19],
                "comm": (directory / "comm").read_text().strip()}
    except (OSError, ValueError, IndexError):
        return None


def _has_token(pid, token):
    if not token:
        return False
    try:
        value = ("GAZEBOTEACH_SESSION_TOKEN=" + token).encode()
        return value in (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
    except OSError:
        return False


def _token_processes(token):
    if os.name != "posix" or not token:
        return []
    result = []
    for directory in Path("/proc").iterdir():
        if directory.name.isdigit():
            info = _process_info(directory.name)
            if info and info["state"] != "Z" and _has_token(info["pid"], token):
                result.append(info)
    return result


def _group_owned(pgid, token):
    members = [item for item in _token_processes(token) if item["pgid"] == pgid]
    # A shared terminal service is never an experiment's disposable window.
    if any("gnome-terminal-" in item["comm"] for item in members):
        return []
    return members


def _terminate_group(pgid, token, grace=3, first_signal=None):
    if pgid <= 1 or pgid == os.getpgrp():
        return
    first_signal = first_signal or signal.SIGTERM
    for sent, wait in ((first_signal, grace), (signal.SIGKILL, 1.5)):
        if not _group_owned(pgid, token):
            return
        try:
            os.killpg(pgid, sent)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if not _group_owned(pgid, token):
                return
            time.sleep(0.05)


def terminate_owned(process, grace=3, token=None, first_signal=None):
    """Clean a created process group even when its original parent has exited."""
    if os.name == "posix":
        token = token or getattr(process, "_gazeboteach_token", None)
        pgid = getattr(process, "_gazeboteach_pgid", process.pid)
        if token:
            info = _process_info(process.pid)
            recorded = getattr(process, "_gazeboteach_start_time", None)
            if info and recorded and info["start_time"] != recorded:
                return
            _terminate_group(pgid, token, grace=grace, first_signal=first_signal)
        elif process.poll() is None:
            # Backwards compatible for caller-owned Popen; do not signal its caller's group.
            try:
                if os.getpgid(process.pid) == process.pid:
                    os.killpg(process.pid, signal.SIGTERM)
                else:
                    process.terminate()
            except ProcessLookupError:
                return
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            if token:
                _terminate_group(pgid, token, grace=0.2)
    elif process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


class LessonSession:
    def __init__(self, root, lesson, storage=None, notify=None):
        self.root = Path(root).resolve()
        self.lesson = lesson
        self.storage = storage or Storage()
        self.notify = notify or (lambda message: None)
        self.workspace = self.storage.prepare(lesson)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        self.session_dir = self.storage.base / "sessions" / (lesson.profile + "-" + lesson.id + "-" + stamp)
        self.session_dir.mkdir(parents=True)
        self.log_path = self.session_dir / "runtime.log"
        self.log_path.touch()
        self.processes = []
        self.auxiliaries = []
        self.terminal_markers = []
        self.lock = threading.RLock()
        self.cancelled = threading.Event()
        self.token = uuid.uuid4().hex
        self.window_tag = "[GT-{}]".format(lesson.id)
        self.window_role = "gazeboteach-" + self.token
        self.state = "准备中"
        self.error = None
        self.paused = False
        self.gazebo_port, self.ros_port = allocate_ports()
        self.env = lesson_environment(self.root, self.workspace, lesson, self.gazebo_port,
                                      self.ros_port, session_token=self.token)
        self.env["GAZEBOTEACH_SESSION_DIR"] = str(self.session_dir)
        self.env["GAZEBOTEACH_LESSON_SOURCE"] = str(self.lesson.folder)
        gazebo_log_path = self.session_dir / "gazebo-log"
        gazebo_log_path.mkdir()
        self.env["GAZEBO_LOG_PATH"] = str(gazebo_log_path)
        self.server = None
        self._owned_socket = None
        self.gui = None
        self.native_control = None
        self.native_control_diagnosis = {"backend": "legacy-cli", "reason": "尚未检查可选 SDK 控制后端"}
        self.world_name = "default"
        self._save_metadata()
        self.log("本课 Gazebo 日志与地形缓存目录：" + self.env["GAZEBO_LOG_PATH"])

    @property
    def running(self):
        return self.server is not None and self.server.poll() is None and not self.cancelled.is_set()

    def log(self, message):
        line = "[{}] {}".format(datetime.datetime.now().strftime("%H:%M:%S"), message)
        with self.lock:
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(line + "\n")
        self.notify(line)

    def _record(self, event, **details):
        return self.storage.record(self.lesson.id, event, profile=self.lesson.profile, **details)

    def _save_metadata(self):
        with self.lock:
            metadata = {
                "lesson": self.lesson.id, "profile": self.lesson.profile, "state": self.state,
                "error": self.error, "owner_pid": os.getpid(), "token": self.token,
                "workspace": str(self.workspace), "gazebo_port": self.gazebo_port,
                "ros_port": self.ros_port, "ign_partition": self.env.get("IGN_PARTITION"),
                "gazebo_log_path": self.env.get("GAZEBO_LOG_PATH"),
                "native_control": (self.native_control.metadata() if getattr(self, "native_control", None)
                                   else getattr(self, "native_control_diagnosis", {"backend": "legacy-cli"})),
                "processes": [{"name": name, "pid": process.pid, "returncode": process.poll(),
                               "start_time": getattr(process, "_gazeboteach_start_time", None)}
                              for name, process in self.processes],
            }
            temporary = self.session_dir / (".session-" + uuid.uuid4().hex + ".json")
            temporary.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
            os.replace(str(temporary), str(self.session_dir / "session.json"))

    def _spawn(self, name, arguments):
        with self.lock:
            if self.cancelled.is_set():
                raise RuntimeError("实验启动已取消。")
            self.log("启动 {}：{}".format(name, shlex.join([str(x) for x in arguments])))
            with self.log_path.open("ab", buffering=0) as handle:
                process = subprocess.Popen(arguments, cwd=str(self.workspace), env=self.env,
                                           stdout=handle, stderr=subprocess.STDOUT,
                                           stdin=subprocess.DEVNULL, start_new_session=True)
            info = _process_info(process.pid) if os.name == "posix" else None
            process._gazeboteach_token = self.token
            process._gazeboteach_pgid = process.pid
            process._gazeboteach_start_time = info["start_time"] if info else None
            self.processes.append((name, process))
            self._save_metadata()
            return process

    def command(self, command, timeout=90, label="准备命令"):
        process = self._spawn(label, ["bash", "--noprofile", "--norc", "-c", command])
        try:
            result = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            terminate_owned(process)
            raise RuntimeError("命令超过 {} 秒：{}".format(timeout, command))
        if self.cancelled.is_set():
            raise RuntimeError("实验操作已取消。")
        if result:
            raise RuntimeError("命令执行失败（{}），请查看运行日志：{}".format(result, command))

    def _wait_ros(self, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.cancelled.is_set():
            try:
                with xmlrpc.client.ServerProxy(self.env["ROS_MASTER_URI"], transport=TimeoutTransport()) as proxy:
                    if proxy.getPid("/gazeboteach")[0] == 1:
                        self.log("本课 ROS Master 已就绪。")
                        return
            except (OSError, xmlrpc.client.Error):
                pass
            self.cancelled.wait(0.3)
        raise RuntimeError("本课 ROS Master 启动超时或已取消。")

    def _wait_world(self, timeout=35):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.cancelled.is_set():
            if self.server.poll() is not None:
                raise RuntimeError("Gazebo 服务端提前退出，请查看运行日志。")
            try:
                with socket.create_connection(("127.0.0.1", self.gazebo_port), timeout=0.3):
                    pass
                result = subprocess.run(["gz", "topic", "-l"], env=self.env, cwd=str(self.workspace),
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3,
                                        text=True, errors="replace")
                if result.returncode == 0 and "/world_stats" in result.stdout and self.server.poll() is None:
                    self.log("本课世界状态主题已出现，Gazebo world 已就绪。")
                    return
            except (OSError, subprocess.TimeoutExpired):
                pass
            self.cancelled.wait(0.4)
        raise RuntimeError("Gazebo 世界就绪超时或已取消：未检测到本课 world_stats 主题。")

    def start(self, gui=True, terminals=True):
        with self.lock:
            if self.server or self.cancelled.is_set():
                raise RuntimeError("该会话已启动或结束，请新建实验会话。")
        try:
            report = detect_environment()
            report["display"] = self.env.get("DISPLAY", "")
            reason = compatibility(self.lesson, report, gui=gui, terminals=terminals)
            if reason:
                raise RuntimeError(reason)
            self.log("使用练习副本：" + str(self.workspace))
            self.log("独立 Gazebo Master：" + self.env["GAZEBO_MASTER_URI"])
            self.log("独立 Ignition partition：" + self.env["IGN_PARTITION"])
            status = self.storage.template_status(self.lesson)
            if status["conflicts"]:
                self.log("保留学生修改；与新模板存在冲突：" + ", ".join(status["conflicts"]))
            if self.lesson.requires_ros:
                self._spawn("ROS Master", ["roscore", "-p", str(self.ros_port)])
                self._wait_ros()
            for command in self.lesson.data.get("prepare", []):
                self.command(command, timeout=int(self.lesson.data.get("prepare_timeout", 120)))
            world = (self.workspace / self.lesson.data.get("world", "world.sdf")).resolve()
            if self.workspace.resolve() not in world.parents or not world.is_file():
                raise RuntimeError("课程世界文件不存在或不在练习目录内：" + str(world))
            try:
                element = ET.parse(str(world)).getroot().find("world")
                if element is not None:
                    self.world_name = element.attrib.get("name", "default")
            except ET.ParseError as exc:
                raise RuntimeError("SDF XML 格式错误：" + str(exc))
            arguments = ["gzserver", "--verbose"]
            if self.lesson.requires_ros:
                arguments += ["-s", "libgazebo_ros_api_plugin.so"]
            arguments += [str(x) for x in self.lesson.data.get("server_args", [])]
            arguments.append(str(world))
            self.server = self._spawn("Gazebo 服务端", arguments)
            self._wait_world(timeout=int(self.lesson.data.get("ready_timeout", 35)))
            self._start_native_control()
            for command in self.lesson.data.get("after_start", []):
                self.command(command, timeout=90, label="启动后准备")
            for item in self.lesson.data.get("startup", []):
                process = self._spawn(item.get("name", "课程辅助节点"),
                                      ["bash", "--noprofile", "--norc", "-c", item["command"]])
                self.auxiliaries.append((item.get("name", "课程辅助节点"), process))
            for command in self.lesson.data.get("after_startup", []):
                self.command(command, timeout=self.lesson.data.get("after_startup_timeout", 90),
                             label="辅助节点就绪检查")
            self._owned_socket = self._snapshot_owned_socket()
            if gui:
                self.open_gui()
            if terminals:
                self.open_terminal(logs=True)
                self.open_terminal()
            with self.lock:
                if self.cancelled.is_set() or not self.running:
                    raise RuntimeError("实验在启动完成前被取消或服务端退出。")
                self.state = "运行中"
                self._save_metadata()
            self._record("started", session=str(self.session_dir),
                         gazebo_master=self.env["GAZEBO_MASTER_URI"], gui=gui)
            self.log("实验已准备好；请结合教材中的预期现象继续操作。")
        except Exception as exc:
            self.error = str(exc)
            self.stop(reason="启动失败，清理本课进程")
            raise

    def _start_native_control(self):
        from .native_control import try_create
        controller, diagnosis = try_create(self.root, self.env, self.world_name, self.session_dir)
        with self.lock:
            if self.cancelled.is_set() or not self.running:
                if controller: controller.close()
                raise RuntimeError("实验在原生控制后端初始化期间已结束。")
            self.native_control = controller
            self.native_control_diagnosis = diagnosis
            self._save_metadata()
        if controller:
            self.log("控制后端：Classic 11.15.1 持久 SDK 客户端；实际 world_stats 已就绪，PID={}".format(controller.process.pid))
        else:
            self.log("控制后端：legacy-cli；可选持久 SDK 后端未启用：" + diagnosis.get("reason", "未知原因"))

    def open_gui(self):
        if not self.running:
            raise RuntimeError("请先启动本课服务端。")
        if not self.env.get("DISPLAY"):
            raise RuntimeError("没有图形 DISPLAY，请在 Ubuntu 图形桌面运行 launch.sh。")
        if not shutil.which("gzclient", path=self.env.get("PATH")):
            raise RuntimeError("未找到 gzclient。")
        attempts = self.lesson.data.get("gui_start_attempts", 1)
        for attempt in range(1, attempts + 1):
            with self.lock:
                if self.gui and self.gui.poll() is None:
                    self.log("Gazebo 图形窗口仍在运行。")
                    return
                if not self.running:
                    raise RuntimeError("实验已取消或本课服务端已退出，停止打开 Gazebo 窗口。")
                self.log("Gazebo 图形初始化尝试 {}/{}。".format(attempt, attempts))
                arguments = ["gzclient", "--verbose"] + [str(x) for x in self.lesson.data.get("gui_args", [])]
                new_gui = self._spawn("Gazebo 图形窗口", arguments)
                self.gui = new_gui
            try:
                deadline = time.monotonic() + min(float(self.lesson.data.get("gui_ready_timeout", 3.0)), 10.0)
                while time.monotonic() < deadline and not self.cancelled.is_set():
                    if not self.running:
                        raise RuntimeError("本课服务端已退出，停止图形初始化。")
                    if new_gui.poll() is not None:
                        raise RuntimeError("Gazebo 客户端提前退出，请查看图形/渲染错误日志。")
                    self.cancelled.wait(0.1)
                if self.cancelled.is_set():
                    raise RuntimeError("打开 Gazebo 窗口已取消。")
                if new_gui.poll() is not None:
                    raise RuntimeError("Gazebo 客户端提前退出，请查看图形/渲染错误日志。")
                for command in self.lesson.data.get("after_gui", []):
                    if not self.running:
                        raise RuntimeError("实验已取消或本课服务端已退出，停止图形初始化。")
                    self.command(command, timeout=90, label="GUI 启动后准备")
                if self.cancelled.is_set():
                    raise RuntimeError("打开 Gazebo 窗口已取消。")
                if not self.running:
                    raise RuntimeError("本课服务端已退出，停止图形初始化。")
                if new_gui.poll() is not None:
                    raise RuntimeError("Gazebo 客户端提前退出，请查看图形/渲染错误日志。")
            except Exception as exc:
                self.log("Gazebo 图形初始化失败（尝试 {}/{}），仅关闭本次新建客户端：{}".format(attempt, attempts, exc))
                try:
                    terminate_owned(new_gui)
                finally:
                    with self.lock:
                        if self.gui is new_gui:
                            self.gui = None
                        self._save_metadata()
                if self.cancelled.is_set() or not self.running:
                    raise
                if attempt == attempts:
                    if attempts > 1:
                        raise RuntimeError("Gazebo 图形初始化已尝试 {} 次仍失败：{}".format(attempts, exc)) from exc
                    raise
                self.log("保留本课服务端，使用新客户端重试图形初始化（下一次 {}/{}）。".format(attempt + 1, attempts))
                continue
            self.log("Gazebo 客户端进程已启动；画面结果请在窗口中观察。")
            return

    def open_terminal(self, logs=False, command=None, title=None):
        if not self.running:
            raise RuntimeError("请先开始实验。")
        if not self.env.get("DISPLAY"):
            raise RuntimeError("当前会话没有图形显示，无法打开终端窗口。")
        terminal = (shutil.which("gnome-terminal", path=self.env.get("PATH")) or
                    shutil.which("xterm", path=self.env.get("PATH")))
        if not terminal:
            raise RuntimeError("缺少 gnome-terminal 或 xterm。")
        marker = self.session_dir / ("terminal-" + uuid.uuid4().hex + ".pid")
        rcfile = marker.with_suffix(".rc")
        script = marker.with_suffix(".py")
        prompt = "[GazeboTeach {}] ".format(self.lesson.id) + r"\w \$ "
        shell_probe = (
            "import json,pathlib,sys; p=pathlib.Path(sys.argv[1]); "
            "d=json.loads(p.read_text()); d['shell_flags']=sys.argv[2]; "
            "d['shell_job_control']='m' in sys.argv[2]; "
            "p.write_text(json.dumps(d))"
        )
        rcfile.write_text("export PS1=" + shlex.quote(prompt) + "\n" +
                          "export HISTFILE=" + shlex.quote(str(self.session_dir / "bash_history")) + "\n" +
                          "cd " + shlex.quote(str(self.workspace)) + "\n" +
                          "/usr/bin/python3 -c " + shlex.quote(shell_probe) + " " + shlex.quote(str(marker)) + ' "$-"\n' +
                          "printf '%s\\n' " + shlex.quote("本课环境已加载。练习目录：" + str(self.workspace)) + "\n",
                          encoding="utf-8")
        if command and not logs:
            with rcfile.open("a", encoding="utf-8") as stream:
                stream.write("printf '%s\\n' " + shlex.quote("即将执行：" + command) + "\n" + command + "\n")
        wrapper = [
            "import json,os,pathlib,subprocess",
            "pid=os.getpid()",
            "raw=pathlib.Path('/proc/%d/stat'%pid).read_text()",
            "fields=raw[raw.rfind(')')+2:].split()",
            "metadata={'pid':pid,'pgid':os.getpgid(pid),'sid':os.getsid(pid),'start_time':fields[19],"
            "'token':os.environ.get('GAZEBOTEACH_SESSION_TOKEN',''),'isatty':os.isatty(0),"
            "'tty_name':os.ttyname(0) if os.isatty(0) else None,'interactive':%r}" % (not logs),
            "try: metadata['tty_pgrp']=os.tcgetpgrp(0)",
            "except OSError: metadata['tty_pgrp']=None",
            "pathlib.Path(%r).write_text(json.dumps(metadata))" % str(marker),
        ]
        if logs:
            arguments = ["tail", "--pid=" + str(self.server.pid), "-n", "80", "-F", str(self.log_path)]
        else:
            arguments = ["bash", "--noprofile", "--rcfile", str(rcfile), "-i"]
        wrapper += ["args=%r" % arguments, "os.execvpe(args[0],args,os.environ)"]
        script.write_text("\n".join(wrapper) + "\n", encoding="utf-8")
        label = title or ("运行日志" if logs else "实验终端")
        arguments = [terminal]
        if Path(terminal).name == "gnome-terminal":
            arguments += ["--wait", "--role=" + self.window_role,
                          "--title", self.window_tag + " " + label, "--"]
        else:
            arguments += ["-name", self.window_role, "-T", self.window_tag + " " + label, "-e"]
        # VTE/xterm already establish an isolated session with a controlling PTY.
        # A second setsid would detach that PTY and disable interactive job control.
        arguments += ["/usr/bin/python3", str(script)]
        self.terminal_markers.append(marker)
        launcher = self._spawn(label, arguments)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not self.cancelled.is_set():
            if marker.is_file():
                try:
                    metadata = json.loads(marker.read_text())
                except (OSError, ValueError):
                    self.cancelled.wait(0.05)
                    continue
                isolated = (metadata.get("pid") == metadata.get("pgid") == metadata.get("sid")
                            and metadata.get("sid") != os.getsid(0))
                if (metadata.get("token") != self.token or not isolated
                        or not metadata.get("isatty") or metadata.get("tty_pgrp") != metadata.get("pgid")):
                    raise RuntimeError(label + "未获得独立且前台可用的终端会话/PTY。")
                if not logs:
                    if "shell_job_control" not in metadata:
                        self.cancelled.wait(0.05)
                        continue
                    if not metadata["shell_job_control"]:
                        raise RuntimeError("实验终端的 Bash 任务控制未启用。")
                if not self._terminal_info(marker):
                    raise RuntimeError(label + "的身份标记与实际进程不一致。")
                self.log(label + "已就绪（独立 PTY，任务控制正常）。")
                return
            if launcher.poll() is not None:
                raise RuntimeError(label + "在初始化标记生成前退出（{}），请查看终端/D-Bus 日志。".format(launcher.returncode))
            self.cancelled.wait(0.1)
        raise RuntimeError(label + "未就绪或启动已取消。")

    def _read_pause_state(self, timeout=3):
        """Read an actual world_stats sample through the owned command path."""
        if not self.running:
            raise RuntimeError("当前没有正在运行的实验，无法读取暂停状态。")
        controller = getattr(self, "native_control", None)
        if controller:
            return controller.status(timeout=timeout)["stats"]["paused"]
        capture = self.session_dir / (".pause-state-" + uuid.uuid4().hex + ".txt")
        topic = "/gazebo/{}/world_stats".format(self.world_name)
        try:
            self.command("gz topic -e {} -d 1 > {}".format(shlex.quote(topic), shlex.quote(str(capture))),
                         timeout=timeout, label="读取暂停状态")
            if not self.running:
                raise RuntimeError("实验已结束，未确认暂停状态。")
            text = capture.read_text(encoding="utf-8", errors="replace")
            values = re.findall(r"^\s*paused:\s*(true|false)\s*$", text, re.MULTILINE)
            if not values:
                raise RuntimeError("world_stats 未提供实际 paused 状态。")
            return values[-1] == "true"
        finally:
            try:
                capture.unlink()
            except FileNotFoundError:
                pass

    def _confirm_pause_state(self, paused, timeout=4):
        """Allow transport delivery time, retrying only bounded read-only samples."""
        deadline = time.monotonic() + timeout
        last = "尚未收到 world_stats"
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if not self.running:
                raise RuntimeError("实验已结束，未确认暂停状态。")
            try:
                observed = self._read_pause_state(timeout=min(3, remaining))
                if observed == paused:
                    return
                last = "world_stats paused={}".format(str(observed).lower())
            except (OSError, RuntimeError) as exc:
                last = str(exc)
            if not self.running:
                raise RuntimeError("实验已结束，未确认暂停状态。")
            self.cancelled.wait(min(0.1, max(0, deadline - time.monotonic())))
        raise RuntimeError("暂停状态回读未确认目标 paused={}：{}".format(str(paused).lower(), last))

    def pause(self, paused):
        if not self.running:
            raise RuntimeError("当前没有正在运行的实验。")
        paused = bool(paused)
        controller = getattr(self, "native_control", None)
        if controller:
            result = controller.pause(paused)
            with self.lock:
                if not self.running:
                    raise RuntimeError("实验已结束，未记录暂停状态。")
                self.paused = paused
                self.state = "已暂停" if paused else "运行中"
                self._save_metadata()
                self._record("paused" if paused else "resumed", observed_paused=paused,
                             observation="persistent SDK actual WorldStatistics", native_result=result)
            return
        observed = self._read_pause_state()
        command_error = None
        if observed != paused:
            try:
                self.command("gz world -w {} -p {}".format(shlex.quote(self.world_name), "1" if paused else "0"),
                             timeout=8, label="暂停/继续")
            except (OSError, RuntimeError) as exc:
                command_error = exc
                self.log("暂停/继续命令报告错误，保留原错误且仅回读，不重发：{}".format(exc))
            try:
                self._confirm_pause_state(paused)
            except RuntimeError as exc:
                if command_error is not None:
                    raise RuntimeError("{}；{}".format(command_error, exc)) from command_error
                raise
        with self.lock:
            if not self.running:
                raise RuntimeError("实验已结束，未记录暂停状态。")
            self.paused = paused
            self.state = "已暂停" if paused else "运行中"
            self._save_metadata()
            details = {"observed_paused": paused, "observation": "world_stats"}
            if command_error is not None:
                details["command_error"] = str(command_error)
                self.log("警告：暂停/继续命令报告错误（{}），但 world_stats 已确认 paused={}；未重发控制。".format(
                    command_error, str(paused).lower()))
            self._record("paused" if paused else "resumed", **details)

    def step(self, count=1):
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("步数必须为正整数。")
        if not self.running:
            raise RuntimeError("当前没有正在运行的实验。")
        # A GUI or lesson terminal may have changed the world since our last read.
        self.pause(True)
        controller = getattr(self, "native_control", None)
        if controller:
            result = controller.step(count)
            self._record("stepped", count=count, native_result=result)
            self._save_metadata()
            return
        flag = "-s" if count == 1 else "-m " + str(count)
        self.command("gz world -w {} {}".format(shlex.quote(self.world_name), flag),
                     timeout=8, label="单步/多步")
        self._record("stepped", count=count)

    def reset_models(self):
        if not self.running:
            raise RuntimeError("当前没有正在运行的实验。")
        controller = getattr(self, "native_control", None)
        if controller:
            result = controller.reset_models()
            self._record("reset_models", native_result=result)
            self._save_metadata()
            return
        self.command("gz world -w {} -o".format(shlex.quote(self.world_name)),
                     timeout=10, label="重置模型状态")
        self._record("reset_models")

    def verify(self):
        if not self.running:
            raise RuntimeError("请先运行本课实验，再执行验收。")
        # Run the current trusted course verifier, with the actual student world as cwd.
        # Practice copies may deliberately contain student-modified scripts.
        path = self.lesson.file(self.lesson.data.get("verify", "verify.py"))
        if not path.is_file():
            self.log("本课未提供自动验收脚本；请完成正文中的人工观察检查。")
            self._record("verification_unavailable")
            return {"passed": None, "message": "本课采用人工观察检查，未标记为自动验收通过。"}
        arguments = ["/usr/bin/python3", str(path)] if path.suffix == ".py" else ["bash", str(path)]
        process = self._spawn("课程验收", arguments)
        try:
            result = process.wait(timeout=int(self.lesson.data.get("verify_timeout", 60)))
        except subprocess.TimeoutExpired:
            terminate_owned(process)
            self._record("verification_failed", reason="timeout")
            raise RuntimeError("验收超时，请查看日志中的等待条件。")
        if self.cancelled.is_set():
            raise RuntimeError("验收已取消，未标记通过。")
        passed = result == 0
        self._record("verified" if passed else "verification_failed", returncode=result)
        self.log("验收通过。" if passed else "验收未通过，请根据日志中的观测值排查。")
        return {"passed": passed, "returncode": result, "log": str(self.log_path)}

    def _terminal_info(self, marker):
        try:
            data = json.loads(marker.read_text())
            if data.get("token") != self.token:
                return None
            info = _process_info(int(data["pid"]))
            if not info or info["start_time"] != data.get("start_time"):
                return None
            if not _has_token(info["pid"], self.token):
                return None
            return info
        except (OSError, ValueError, KeyError):
            return None

    def _close_terminal_groups(self):
        for marker in self.terminal_markers:
            info = self._terminal_info(marker)
            if info:
                groups = {item["pgid"] for item in _token_processes(self.token)
                          if item["sid"] == info["sid"]}
                for pgid in groups:
                    _terminate_group(pgid, self.token, grace=1, first_signal=signal.SIGHUP)

    def _snapshot_owned_socket(self):
        prefix = self.lesson.data.get("owned_socket_prefix")
        if not prefix or os.name != "posix" or not self.server or self.server.poll() is not None:
            return None
        path = Path("/tmp") / (prefix + self.token + ".sock")
        try:
            info = path.lstat()
        except OSError:
            return None
        if not (stat.S_ISSOCK(info.st_mode) and info.st_uid == os.getuid() and stat.S_IMODE(info.st_mode) == 0o600
                and hasattr(socket, "SO_PEERCRED")):
            return None
        identity = (info.st_dev, info.st_ino, info.st_uid, info.st_mode, info.st_ctime_ns)
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(0.25)
                client.connect(str(path))
                pid, uid, _ = struct.unpack("3i", client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))
            current = path.lstat()
            if (pid == self.server.pid and uid == os.getuid()
                    and (current.st_dev, current.st_ino, current.st_uid, current.st_mode, current.st_ctime_ns) == identity):
                return path, identity
        except OSError:
            pass
        return None

    def _remove_owned_socket(self):
        endpoint = getattr(self, "_owned_socket", None)
        if not endpoint or not self.server or self.server.poll() is None or _token_processes(self.token):
            return
        path, identity = endpoint
        try:
            info = path.lstat()
            if (info.st_dev, info.st_ino, info.st_uid, info.st_mode, info.st_ctime_ns) == identity:
                path.unlink()
                self.log("已清理异常退出遗留的本次实验套接字：" + str(path))
        except FileNotFoundError:
            pass
        except OSError as error:
            self.log("无法清理本次实验套接字：" + str(error))

    def stop(self, reason="结束实验"):
        self.cancelled.set()
        with self.lock:
            if self.state == "已结束":
                return
            previous = self.state
            controller = getattr(self, "native_control", None)
            if controller:
                cleanup = controller.close()
                self.log("持久原生控制客户端清理（先于服务端）：" + json.dumps(cleanup, ensure_ascii=False))
            self._close_terminal_groups()
            for name, process in reversed(self.processes):
                if process is self.server and self.lesson.data.get("graceful_server_shutdown"):
                    # Classic handles SIGINT through its normal shutdown path,
                    # which gives this opt-in course's plugin time to release
                    # filesystem endpoints. Ownership/kill bounds still apply.
                    terminate_owned(process, first_signal=signal.SIGINT)
                else:
                    terminate_owned(process)
            if os.name == "posix":
                # ROS launchers may create new process groups; the unguessable inherited
                # token proves ownership even after their immediate parent has exited.
                # Shared GNOME terminal services are deliberately excluded.
                for _ in range(3):
                    groups = {item["pgid"] for item in _token_processes(self.token)
                              if "gnome-terminal-" not in item["comm"]}
                    if not groups:
                        break
                    for pgid in groups:
                        _terminate_group(pgid, self.token, grace=0.5)
                    time.sleep(0.1)
            self._remove_owned_socket()
            self.state = "已结束"
            self._save_metadata()
        self._record("stopped", reason=reason, previous_state=previous)
        self.log(reason + "；仅清理当前实验拥有的进程，练习文件已保留。")

    def health(self):
        with self.lock:
            watched = [(name, process) for name, process in self.processes
                       if name in ("Gazebo 服务端", "ROS Master")] + list(self.auxiliaries)
            dead = [{"name": name, "returncode": process.poll()} for name, process in watched
                    if process.poll() is not None]
            controller = getattr(self, "native_control", None)
            if controller and not controller.closed and controller.process.poll() is not None:
                dead.append({"name": "持久原生控制客户端", "returncode": controller.process.returncode})
            terminal_count = sum(self._terminal_info(marker) is not None for marker in self.terminal_markers)
            return {
                "running": self.running, "state": self.state, "error": self.error,
                "failed_processes": dead, "gui_running": self.gui is not None and self.gui.poll() is None,
                "terminal_running": terminal_count,
                "window_tag": self.window_tag,
                "window_role": self.window_role,
                "auxiliary": [{"name": name, "running": process.poll() is None,
                               "returncode": process.poll()} for name, process in self.auxiliaries],
            }
