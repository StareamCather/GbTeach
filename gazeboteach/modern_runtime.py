"""Owned Gazebo Sim sessions using native world-control services."""

import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

from .catalog import compatibility
from .environment import detect_environment
from .modern import MODERN_PROFILES
from .runtime import LessonSession, terminate_owned


def parse_stats(text):
    """Parse one protobuf-text WorldStatistics sample (omitted scalars are 0)."""
    if not re.search(r'\bsim_time\s*\{', text):
        raise RuntimeError('未收到有效 Gazebo Sim WorldStatistics。')
    block = re.search(r'\bsim_time\s*\{([^}]*)\}', text).group(1)
    def number(field, source=text):
        match = re.search(r'\b' + field + r':\s*([0-9]+)', source)
        return int(match.group(1)) if match else 0
    return {'paused': bool(re.search(r'\bpaused:\s*true\b', text)),
            'iterations': number('iterations'),
            'sim_time': number('sec', block) + number('nsec', block) / 1e9,
            'sim_sec': number('sec', block), 'sim_nsec': number('nsec', block)}


class DomainLease:
    """Avoid DDS-domain reuse between application sessions of this Linux user."""
    def __init__(self):
        import fcntl
        directory = Path(tempfile.gettempdir()) / ('gazeboteach-dds-' + str(os.getuid()))
        directory.mkdir(mode=0o700, exist_ok=True)
        if directory.is_symlink() or directory.stat().st_uid != os.getuid():
            raise RuntimeError('DDS domain 锁目录身份不匹配。')
        self.handle = None
        for domain in range(20, 100):
            candidate = directory / ('domain-' + str(domain))
            handle = os.fdopen(os.open(str(candidate), os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600), 'r+')
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                handle.close()
                continue
            self.handle, self.domain = handle, domain
            return
        raise RuntimeError('当前用户的教学 DDS domain 已用尽。')

    def close(self):
        if self.handle:
            self.handle.close()
            self.handle = None


class ModernLessonSession(LessonSession):
    def __init__(self, root, lesson, storage=None, notify=None):
        self.domain_lease = None
        self.simulator = MODERN_PROFILES[lesson.profile]
        super().__init__(root, lesson, storage=storage, notify=notify)
        self.native_control_diagnosis = {'backend': self.simulator['sim_name'].lower() + '-world-control',
                                         'reason': 'Gazebo Sim WorldControl 服务及实际 WorldStatistics 回读'}
        self.last_stats = None
        self.domain_lease = DomainLease()
        self.env['ROS_DOMAIN_ID'] = str(self.domain_lease.domain)
        self.sim_cli = shutil.which(self.simulator['command'], path=self.env.get('PATH'))
        self.partition_key = {'gz': 'GZ_PARTITION', 'ign': 'IGN_PARTITION'}[self.simulator['command']]
        self._save_metadata()

    def _sim_command(self, *arguments):
        return [self.sim_cli, self.simulator['sim_subcommand'],
                '--force-version', str(self.simulator['sim_major']), *arguments]

    def _capture(self, arguments, timeout=5):
        capture = self.session_dir / ('.modern-' + uuid.uuid4().hex + '.txt')
        try:
            command = shlex.join([str(x) for x in arguments]) + ' > ' + shlex.quote(str(capture))
            self.command(command, timeout=timeout, label='Gazebo Sim 查询/控制')
            return capture.read_text(encoding='utf-8', errors='replace')
        finally:
            try:
                capture.unlink()
            except FileNotFoundError:
                pass

    def _stats(self, timeout=5):
        text = self._capture([self.sim_cli, 'topic', '-e', '-t', '/world/' + self.world_name + '/stats', '-n', '1'],
                             timeout)
        self.last_stats = parse_stats(text)
        return self.last_stats

    def _control(self, request):
        # A mutation is sent exactly once. A successful Boolean only confirms
        # acceptance; callers must still inspect actual world statistics.
        prefix = self.simulator['msg_prefix']
        text = self._capture([self.sim_cli, 'service', '-s', '/world/' + self.world_name + '/control',
            '--reqtype', prefix + '.WorldControl', '--reptype', prefix + '.Boolean',
            '--timeout', '3000', '--req', request], timeout=5)
        if not re.search(r'\bdata:\s*true\b', text):
            raise RuntimeError('WorldControl 未确认接收；保留现场，不自动重发：' + text[-1000:])

    def _wait_world(self, timeout=45):
        deadline = time.monotonic() + timeout
        last = ''
        while time.monotonic() < deadline and not self.cancelled.is_set():
            if not self.running:
                raise RuntimeError('Gazebo Sim 服务端提前退出，请查看日志。')
            try:
                text = self._capture([self.sim_cli, 'service', '-l'], timeout=min(5, max(.2, deadline-time.monotonic())))
                if '/world/' + self.world_name + '/control' in text.splitlines():
                    self._stats(timeout=min(5, max(.2, deadline-time.monotonic())))
                    self.log('实际 WorldStatistics 和本课 control 服务均已就绪。')
                    return
            except (OSError, RuntimeError) as error:
                last = str(error)
            self.cancelled.wait(.2)
        raise RuntimeError('Gazebo Sim 初始化超时：' + last)

    def start(self, gui=True, terminals=True):
        if self.server or self.cancelled.is_set():
            raise RuntimeError('请新建实验会话。')
        try:
            report = detect_environment()
            report['display'] = self.env.get('DISPLAY', '')
            reason = compatibility(self.lesson, report, gui=gui, terminals=terminals)
            if reason:
                raise RuntimeError(reason)
            if not self.sim_cli:
                raise RuntimeError('当前课程子环境缺少程序：' + self.simulator['command'])
            self.log('独立 ' + self.partition_key + '：' + self.env[self.partition_key])
            self.log('独立 ROS_DOMAIN_ID：' + self.env['ROS_DOMAIN_ID'])
            self.log('个人练习副本：' + str(self.workspace))
            for command in self.lesson.data.get('prepare', []):
                self.command(command, timeout=self.lesson.data.get('prepare_timeout', 120))
            world = (self.workspace / self.lesson.data.get('world', 'world.sdf')).resolve()
            if self.workspace.resolve() not in world.parents or not world.is_file():
                raise RuntimeError('世界文件必须位于个人练习目录。')
            element = ET.parse(str(world)).getroot().find('world')
            if element is None or not re.fullmatch(r'[A-Za-z0-9_]+', element.get('name', '')):
                raise RuntimeError('SDF 缺少合法命名 world。')
            self.world_name = element.get('name')
            self.server = self._spawn('Gazebo 服务端', self._sim_command('-s', '-r', '-v', '3')
                                      + self.lesson.data.get('server_args', []) + [str(world)])
            self._wait_world(self.lesson.data.get('ready_timeout', 45))
            for command in self.lesson.data.get('after_start', []):
                self.command(command, timeout=90, label='启动后准备')
            for item in self.lesson.data.get('startup', []):
                process = self._spawn(item.get('name', '课程辅助节点'),
                                      ['bash', '--noprofile', '--norc', '-c', item['command']])
                self.auxiliaries.append((item.get('name', '课程辅助节点'), process))
            for command in self.lesson.data.get('after_startup', []):
                self.command(command, timeout=self.lesson.data.get('after_startup_timeout', 90), label='辅助节点就绪检查')
            if gui:
                self.open_gui()
            if terminals:
                self.open_terminal(logs=True)
                self.open_terminal()
            if not self.running:
                raise RuntimeError('服务端在初始化完成前退出。')
            self.state = '运行中'
            self._save_metadata()
            self._record('started', session=str(self.session_dir), ros_domain_id=self.env['ROS_DOMAIN_ID'],
                         backend=self.native_control_diagnosis['backend'], gui=gui,
                         **{self.partition_key.lower(): self.env[self.partition_key]})
            self.log('Gazebo ' + self.simulator['sim_name'] + ' 实验已准备好。')
        except Exception as error:
            self.error = str(error)
            self.stop(reason='现代课程启动失败，清理本课进程')
            raise

    def open_gui(self):
        if not self.running:
            raise RuntimeError('请先启动实验。')
        if not self.env.get('DISPLAY'):
            raise RuntimeError('需要真实桌面 DISPLAY。')
        if self.gui and self.gui.poll() is None:
            self.log('Gazebo Sim GUI 仍在运行。')
            return
        self.gui = self._spawn('Gazebo 图形窗口', self._sim_command('-g', '-v', '3')
                               + self.lesson.data.get('gui_args', []))
        deadline = time.monotonic() + self.lesson.data.get('gui_ready_timeout', 3)
        while time.monotonic() < deadline and not self.cancelled.is_set():
            if self.gui.poll() is not None or not self.running:
                raise RuntimeError('Gazebo Sim GUI 初始化失败，请查看渲染日志。')
            self.cancelled.wait(.1)
        if self.cancelled.is_set():
            raise RuntimeError('GUI 初始化已取消。')
        for command in self.lesson.data.get('after_gui', []):
            self.command(command, timeout=60, label='图形后准备')

    def pause(self, paused):
        if not self.running:
            raise RuntimeError('当前没有实验。')
        paused = bool(paused)
        before = self._stats()
        if before['paused'] != paused:
            self._control('pause: ' + str(paused).lower())
        deadline = time.monotonic() + 7
        while time.monotonic() < deadline:
            stats = self._stats()
            if stats['paused'] == paused:
                self.paused, self.state = paused, ('已暂停' if paused else '运行中')
                self._save_metadata()
                self._record('paused' if paused else 'resumed', observed_paused=paused, native_result={'stats': stats})
                return
        raise RuntimeError('WorldStatistics 未确认暂停/继续状态，控制未重发。')

    def step(self, count=1):
        if type(count) is not int or not 1 <= count <= 100000:
            raise ValueError('步数必须是 [1,100000] 内的整数。')
        self.pause(True)
        baseline = self._stats()['iterations']
        self._control('pause: true, multi_step: ' + str(count))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            stats = self._stats()
            if stats['iterations'] == baseline + count and stats['paused']:
                self._record('stepped', count=count, native_result={'baseline_iterations': baseline, 'stats': stats})
                return
            if stats['iterations'] > baseline + count:
                raise RuntimeError('实际迭代数超过目标；其它客户端可能同时控制世界，未重发步进。')
        raise RuntimeError('单步未确认精确迭代增量；控制不会自动重发。')

    def reset_models(self):
        # Fortress and Harmonic acknowledge WorldControl before logging that
        # model_only is unsupported. A full reset would rewind the clock.
        raise RuntimeError('当前 Gazebo Sim 不支持保持时钟的仅模型复位。请使用“重启实验”完整恢复场景；重启会将本课仿真时钟归零。')

    def stop(self, reason='结束实验'):
        try:
            super().stop(reason)
        finally:
            with self.lock:
                if self.domain_lease:
                    self.domain_lease.close()
