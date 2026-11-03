"""Gazebo Sim 8 / ROS 2 Jazzy capabilities, separate from Classic interfaces.

No runtime is selected just because a binary is named ``gz``. The actual sim
version and the selected ROS installation's package index must both be probed.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess


PROFILE = "harmonic-jazzy"
_SIM_PREFIXES = ('GZ_SIM_', 'GZ_GUI_', 'GZ_RENDERING_', 'GZ_SENSORS_', 'GZ_PLUGIN_',
                 'IGN_GAZEBO_', 'IGN_GUI_', 'IGN_RENDERING_', 'IGN_SENSORS_', 'IGN_PLUGIN_')
_SIM_KEYS = {'GZ_CONFIG_PATH', 'IGN_CONFIG_PATH', 'GZ_VERSION', 'IGN_VERSION',
             'GZ_PARTITION', 'IGN_PARTITION'}


def is_modern(lesson):
    return lesson.profile == PROFILE


def ros2_base_environment(environment):
    """Remove ROS prefixes/overlays in a child environment before Jazzy setup."""
    result = dict(environment)
    roots = {p.resolve() for p in Path('/opt/ros').glob('*') if p.is_dir()}
    for key in ('CMAKE_PREFIX_PATH', 'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CATKIN_PREFIX_PATH'):
        roots.update(Path(p).resolve() for p in result.get(key, '').split(os.pathsep) if p)
    for key in ('PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'PKG_CONFIG_PATH', 'CMAKE_PREFIX_PATH'):
        if key in result:
            result[key] = os.pathsep.join(p for p in result[key].split(os.pathsep)
                if p and not any(Path(p).resolve() == root or root in Path(p).resolve().parents for root in roots))
    for key in tuple(result):
        if (key.startswith(('ROS_', 'AMENT_', 'COLCON_', 'CATKIN_', 'GAZEBO_') + _SIM_PREFIXES)
                or key in _SIM_KEYS or key in (
                    'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE')):
            result.pop(key, None)
    return result


def detect_modern(tools, installed_ros, modern_simulators):
    """Read-only bounded probe, also works in a shell sourced for ROS Noetic."""
    from .environment import _load_setup, _output
    environment = ros2_base_environment(os.environ)
    result = {'sim_major': None, 'sim_version': '', 'gz': tools.get('gz'),
              'ros_distro': 'jazzy', 'ros_available': False, 'packages': [], 'error': '',
              'sim_probe_error': '', 'profiles': [], 'tools': {}}
    if not modern_simulators and 'jazzy' not in installed_ros:
        return result
    if 'jazzy' in installed_ros:
        try:
            environment = _load_setup(Path('/opt/ros/jazzy/setup.bash'), environment)
            result['ros_available'] = shutil.which('ros2', path=environment.get('PATH')) is not None
            if not result['ros_available']:
                result['error'] = 'Jazzy setup 已加载，但其子环境 PATH 中没有可执行的 ros2。'
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            result['error'] = str(error)
    # Resolve every tool against the selected child environment. The outer
    # shell may expose a command from an incompatible ROS/Ionic overlay.
    result['tools'] = {name: shutil.which(name, path=environment.get('PATH'))
                       for name in sorted(set(tools) | {'gz', 'ros2', 'bash', 'gnome-terminal', 'xterm'})}
    binary = result['tools']['gz']
    result['gz'] = binary
    versions = _output([binary, 'sim', '--versions'], timeout=4, env=environment) if binary else ''
    if re.fullmatch(r'[0-9.,\s]+', versions or 'x'):
        matches = re.findall(r'\b8\.\d+(?:\.\d+)?\b', versions)
        if matches:
            result.update(sim_major=8, sim_version=matches[0], profiles=[PROFILE])
    elif binary:
        result['sim_probe_error'] = 'gz sim --versions 未返回可识别的版本；可能超时、缺少Sim子命令或执行失败。'
    if result['ros_available']:
        package_text = _output(['ros2', 'pkg', 'list'], timeout=12, env=environment)
        result['packages'] = sorted(set(package_text.splitlines()))
        if not result['packages']:
            result['error'] = 'Jazzy 包索引未返回结果，不能确认课程依赖。'
    return result


def modern_detection_failures(capability):
    """Shared metadata for real detection and replayed report snapshots."""
    if not isinstance(capability, dict):
        return [], []
    failures = (('sim_probe_error', 'modern_sim_version'), ('error', 'modern_ros_environment'))
    return ([check for key, check in failures if capability.get(key)],
            list(dict.fromkeys(str(capability[key]) for key, _ in failures if capability.get(key))))


def _tool_snapshot(lesson, capability, terminals):
    required = {'bash'} | set(lesson.data.get('requires_tools', []))
    if lesson.requires_ros:
        required.add('ros2')
    selected = capability.get('tools')
    if not isinstance(selected, dict):
        return {}, ['modern.tools']
    incomplete = ['modern.tools:' + name for name in sorted(required)
                  if name not in selected or not (selected[name] is None or isinstance(selected[name], str))]
    if terminals and not any(isinstance(selected.get(name), str) and selected[name] for name in ('gnome-terminal', 'xterm')):
        if any(name not in selected or not (selected[name] is None or isinstance(selected[name], str))
               for name in ('gnome-terminal', 'xterm')):
            incomplete.append('modern.tools:terminal')
    return selected, incomplete


def modern_compatibility(lesson, environment, gui=True, terminals=False):
    capability = environment.get('modern', {})
    if capability.get('sim_major') != 8 or not capability.get('gz'):
        if capability.get('sim_probe_error'):
            return 'Gazebo Sim 版本检测未完成：' + capability['sim_probe_error']
        return '本课需要 Gazebo Harmonic（gz sim 8）；不兼容 Classic、Fortress 或其它未适配主版本。'
    if lesson.requires_ros:
        if not capability.get('ros_available'):
            return '本课需要 ROS 2 Jazzy；请在 Ubuntu 24.04 配置相应环境。'
        if capability.get('error'):
            return 'ROS 2 环境检查未完成：' + capability['error']
        missing = sorted(set(lesson.data.get('requires_ros_packages', [])) - set(capability.get('packages', [])))
        if missing:
            return '缺少 Jazzy ROS 2 包：' + ', '.join(missing)
    if lesson.data.get('requires_gui') and not gui:
        return '本课需要真实 Gazebo Sim GUI；请加入 --gui。'
    tools, incomplete = _tool_snapshot(lesson, capability, terminals)
    if incomplete:
        return '现代运行子环境工具检查未完成：' + ', '.join(incomplete)
    required = ['bash'] + lesson.data.get('requires_tools', []) + (['ros2'] if lesson.requires_ros else [])
    missing = [name for name in required if not tools.get(name)]
    if missing:
        return '缺少程序：' + ', '.join(missing)
    for module in lesson.data.get('requires_python_modules', []):
        if not environment.get('python_modules', {}).get(module):
            return '缺少 Python 3 模块：' + module
    if (gui or terminals or lesson.data.get('needs_display')) and not environment.get('display'):
        return '本课需要图形会话 DISPLAY；请从 Linux 图形桌面启动。'
    if terminals and not (tools.get('gnome-terminal') or tools.get('xterm')):
        return '缺少 gnome-terminal 或 xterm。'
    return None


def modern_compatibility_row(lesson, detected, gui=True, terminals=True):
    capability = detected.get('modern')
    if not isinstance(capability, dict):
        status, reason = 'incomplete_detection', '尚未检测现代 Gazebo/ROS 2 环境；请刷新环境。'
    else:
        reason = modern_compatibility(lesson, detected, gui, terminals)
        unsupported = capability.get('sim_major') != 8 or (lesson.requires_ros and not capability.get('ros_available'))
        status = 'ready' if reason is None else ('unsupported' if unsupported else 'missing_dependencies')
        if lesson.requires_ros and capability.get('error'):
            status = 'incomplete_detection'
        if capability.get('sim_probe_error'):
            status = 'incomplete_detection'
        _, incomplete_tools = _tool_snapshot(lesson, capability, terminals)
        if incomplete_tools and not unsupported:
            status = 'incomplete_detection'
    fields, _ = modern_detection_failures(capability)
    if not isinstance(capability, dict):
        fields.append('modern')
    if isinstance(capability, dict):
        _, incomplete_tools = _tool_snapshot(lesson, capability, terminals)
        if status == 'incomplete_detection':
            fields.extend(incomplete_tools)
    return {'id': lesson.id, 'key': lesson.key, 'title': lesson.title, 'profile': lesson.profile,
            'requires_ros': lesson.requires_ros, 'status': status, 'compatible': status == 'ready',
            'reading_available': True, 'reasons': [] if reason is None else [
                {'code': 'modern_requirement', 'message': reason,
                 **({'fields': sorted(set(fields))} if fields else {})}]}


def modern_lesson_environment(root, workspace, lesson, token):
    from .environment import _load_setup, add_course_python_paths
    environment = ros2_base_environment(os.environ)
    if lesson.requires_ros or Path('/opt/ros/jazzy/setup.bash').is_file():
        setup = Path('/opt/ros/jazzy/setup.bash')
        if not setup.is_file():
            raise RuntimeError('ROS 2 Jazzy 环境不存在：' + str(setup))
        environment = _load_setup(setup, environment)
    add_course_python_paths(environment, workspace, lesson)
    inherited_resources = environment.get('GZ_SIM_RESOURCE_PATH', '')
    environment.update({
        'GZ_PARTITION': 'gazeboteach-' + token, 'IGN_PARTITION': 'gazeboteach-' + token,
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([str(workspace), str(Path(workspace) / 'models')]
                                               + ([inherited_resources] if inherited_resources else [])),
        'GAZEBOTEACH_SOURCE_ROOT': str(root), 'GAZEBOTEACH_WORKSPACE': str(workspace),
        'GAZEBOTEACH_LESSON_ID': lesson.id, 'GAZEBOTEACH_SESSION_TOKEN': token,
        'ROS_LOCALHOST_ONLY': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'PYTHONUNBUFFERED': '1', 'QT_X11_NO_MITSHM': '1',
    })
    # Domain lease is allocated by ModernLessonSession and then inherited by all
    # child terminals/bridge nodes. Never start a shared ROS CLI daemon here.
    return environment
