"""Versioned Gazebo Sim / ROS 2 capabilities, separate from Classic interfaces.

No runtime is selected just because a binary is named ``gz``. The actual sim
version and the selected ROS installation's package index must both be probed.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess


PROFILE = "harmonic-jazzy"
FORTRESS_PROFILE = "fortress-humble"
MODERN_PROFILES = {
    PROFILE: dict(command='gz', sim_subcommand='sim', sim_major=8, msg_prefix='gz.msgs',
                  ros_distro='jazzy', sim_name='Harmonic', ubuntu='24.04',
                  resource_key='GZ_SIM_RESOURCE_PATH'),
    FORTRESS_PROFILE: dict(command='ign', sim_subcommand='gazebo', sim_major=6, msg_prefix='ignition.msgs',
                           ros_distro='humble', sim_name='Fortress', ubuntu='22.04',
                           resource_key='IGN_GAZEBO_RESOURCE_PATH'),
}
_SIM_PREFIXES = ('GZ_SIM_', 'GZ_GUI_', 'GZ_RENDERING_', 'GZ_SENSORS_', 'GZ_PLUGIN_',
                 'IGN_GAZEBO_', 'IGN_GUI_', 'IGN_RENDERING_', 'IGN_SENSORS_', 'IGN_PLUGIN_')
_SIM_KEYS = {'GZ_CONFIG_PATH', 'IGN_CONFIG_PATH', 'GZ_VERSION', 'IGN_VERSION',
             'GZ_PARTITION', 'IGN_PARTITION'}


def is_modern(lesson):
    return lesson.profile in MODERN_PROFILES


def ros2_base_environment(environment):
    """Remove ROS prefixes and overlays before loading the selected ROS 2 setup."""
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
    base = ros2_base_environment(os.environ)
    seen = {item['command'] for item in modern_simulators}
    capabilities = {profile: _probe_profile(profile, config, tools, base, installed_ros, seen)
                    for profile, config in MODERN_PROFILES.items()}
    # The original Harmonic fields stay available to existing diagnostics clients.
    return {**capabilities[PROFILE], 'by_profile': capabilities,
            'profiles': [profile for profile, capability in capabilities.items() if capability['profiles']]}


def _probe_profile(profile, config, tools, base, installed_ros, seen):
    from .environment import _load_setup, _output
    distro, cli = config['ros_distro'], config['command']
    label = distro.capitalize()
    environment = base
    result = {'sim_major': None, 'sim_version': '', cli: None, 'sim_name': config['sim_name'],
              'ros_distro': distro, 'ros_available': False, 'packages': [], 'error': '',
              'sim_probe_error': '', 'profiles': [], 'tools': {}}
    if cli not in seen and distro not in installed_ros:
        return result
    if distro in installed_ros:
        try:
            environment = _load_setup(Path('/opt/ros') / distro / 'setup.bash', environment)
            if environment.get('ROS_DISTRO') != distro or environment.get('ROS_VERSION') != '2':
                raise RuntimeError(label + ' setup 未激活预期的 ROS 2 发行版。')
            result['ros_available'] = shutil.which('ros2', path=environment.get('PATH')) is not None
            if not result['ros_available']:
                result['error'] = label + ' setup 已加载，但其子环境 PATH 中没有可执行的 ros2。'
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            environment = base
            result['error'] = str(error)
    # Resolve every tool against the selected child environment. The outer
    # shell may expose a command from an incompatible ROS/Ionic overlay.
    result['tools'] = {name: shutil.which(name, path=environment.get('PATH'))
                       for name in sorted(set(tools) | {cli, 'ros2', 'bash', 'gnome-terminal', 'xterm'})}
    binary = result['tools'][cli]
    result[cli] = binary
    versions = _output([binary, config['sim_subcommand'], '--versions'], timeout=4, env=environment) if binary else ''
    if re.fullmatch(r'[0-9.,\s]+', versions or 'x'):
        matches = re.findall(r'\b{}\.\d+(?:\.\d+)?\b'.format(config['sim_major']), versions)
        if matches:
            result.update(sim_major=config['sim_major'], sim_version=matches[0], profiles=[profile])
    elif binary:
        result['sim_probe_error'] = '{} {} --versions 未返回可识别的版本。'.format(cli, config['sim_subcommand'])
    if result['ros_available']:
        package_text = _output(['ros2', 'pkg', 'list'], timeout=12, env=environment)
        result['packages'] = sorted(set(package_text.splitlines()))
        if not result['packages']:
            result['error'] = label + ' 包索引未返回结果，不能确认课程依赖。'
    return result


def modern_detection_failures(capability):
    """Shared metadata for real detection and replayed report snapshots."""
    if not isinstance(capability, dict):
        return [], []
    if isinstance(capability.get('by_profile'), dict):
        checks, errors = [], []
        for selected in capability['by_profile'].values():
            selected_checks, selected_errors = modern_detection_failures(selected)
            checks.extend(selected_checks)
            errors.extend(selected_errors)
        return list(dict.fromkeys(checks)), list(dict.fromkeys(errors))
    failures = (('sim_probe_error', 'modern_sim_version'), ('error', 'modern_ros_environment'))
    return ([check for key, check in failures if capability.get(key)],
            list(dict.fromkeys(str(capability[key]) for key, _ in failures if capability.get(key))))


def modern_capability(environment, profile):
    snapshot = environment.get('modern')
    if not isinstance(snapshot, dict):
        return None
    if isinstance(snapshot.get('by_profile'), dict):
        return snapshot['by_profile'].get(profile)
    # Older report snapshots contained a single, Harmonic-only capability.
    return snapshot if profile == PROFILE else {}


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
    config = MODERN_PROFILES[lesson.profile]
    capability = modern_capability(environment, lesson.profile) or {}
    distro = config['ros_distro'].capitalize()
    if capability.get('sim_major') != config['sim_major'] or not capability.get(config['command']):
        if capability.get('sim_probe_error'):
            return 'Gazebo Sim 版本检测未完成：' + capability['sim_probe_error']
        return '本课需要 Gazebo {}（{} {} 主版本 {}）。'.format(
            config['sim_name'], config['command'], config['sim_subcommand'], config['sim_major'])
    if lesson.requires_ros:
        if capability.get('error'):
            return 'ROS 2 环境检查未完成：' + capability['error']
        if not capability.get('ros_available'):
            return '本课需要 ROS 2 {}；请在 Ubuntu {} 配置相应环境。'.format(distro, config['ubuntu'])
        missing = sorted(set(lesson.data.get('requires_ros_packages', [])) - set(capability.get('packages', [])))
        if missing:
            return '缺少 {} ROS 2 包：'.format(distro) + ', '.join(missing)
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
    capability = modern_capability(detected, lesson.profile)
    if not isinstance(capability, dict):
        status, reason = 'incomplete_detection', '尚未检测现代 Gazebo/ROS 2 环境；请刷新环境。'
    else:
        reason = modern_compatibility(lesson, detected, gui, terminals)
        config = MODERN_PROFILES[lesson.profile]
        unsupported = (capability.get('sim_major') != config['sim_major']
                       or not capability.get(config['command'])
                       or (lesson.requires_ros and not capability.get('ros_available')))
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
    config = MODERN_PROFILES[lesson.profile]
    environment = ros2_base_environment(os.environ)
    setup = Path('/opt/ros') / config['ros_distro'] / 'setup.bash'
    if lesson.requires_ros or setup.is_file():
        if not setup.is_file():
            raise RuntimeError('ROS 2 {} 环境不存在：{}'.format(config['ros_distro'].capitalize(), setup))
        environment = _load_setup(setup, environment)
        if environment.get('ROS_DISTRO') != config['ros_distro'] or environment.get('ROS_VERSION') != '2':
            raise RuntimeError('课程子环境未激活预期的 ROS 2 发行版：' + config['ros_distro'])
    add_course_python_paths(environment, workspace, lesson)
    resource_key = config['resource_key']
    inherited_resources = environment.get(resource_key, '')
    environment.update({
        'GZ_PARTITION': 'gazeboteach-' + token, 'IGN_PARTITION': 'gazeboteach-' + token,
        resource_key: os.pathsep.join([str(workspace), str(Path(workspace) / 'models')]
                                     + ([inherited_resources] if inherited_resources else [])),
        'GAZEBOTEACH_SOURCE_ROOT': str(root), 'GAZEBOTEACH_WORKSPACE': str(workspace),
        'GAZEBOTEACH_LESSON_ID': lesson.id, 'GAZEBOTEACH_SESSION_TOKEN': token, 'GAZEBOTEACH_PROFILE': lesson.profile,
        'ROS_LOCALHOST_ONLY': '1', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'PYTHONUNBUFFERED': '1', 'QT_X11_NO_MITSHM': '1',
    })
    # Domain lease is allocated by ModernLessonSession and then inherited by all
    # child terminals/bridge nodes. Never start a shared ROS CLI daemon here.
    return environment
