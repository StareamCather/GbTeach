"""Keep modern/Classic boundaries explicit; test observed-control guarantees."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import compatibility, load_catalog
from gazeboteach.compatibility import build_compatibility_report, lesson_compatibility
from gazeboteach.modern import FORTRESS_PROFILE, PROFILE, detect_modern, modern_capability, ros2_base_environment
from gazeboteach.modern_runtime import ModernLessonSession, parse_stats
from tests.test_core import make_lesson


class ModernCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.lesson = make_lesson(Path(self.temp.name) / 'modern', profile=PROFILE, requires_ros=True,
                                  requires_ros_packages=['ros_gz_bridge', 'rclpy'])
        self.environment = {'classic_version': '', 'tools': {'bash': '/bin/bash'}, 'display': ':1',
                            'modern': {'sim_major': 8, 'gz': '/usr/bin/gz', 'ros_available': True,
                                       'packages': ['ros_gz_bridge', 'rclpy'], 'error': '',
                                       'tools': {'bash': '/bin/bash', 'ros2': '/opt/ros/jazzy/bin/ros2',
                                                 'colcon': '/usr/bin/colcon', 'xterm': '/usr/bin/xterm'}}}

    def tearDown(self):
        self.temp.cleanup()

    def test_modern_does_not_require_classic_server_or_roscore(self):
        self.assertIsNone(compatibility(self.lesson, self.environment, gui=False))
        self.assertEqual(lesson_compatibility(self.lesson, self.environment, gui=False, terminals=False)['status'], 'ready')

    def test_a_gz_name_or_installed_ros_name_is_not_version_evidence(self):
        del self.environment['modern']
        self.environment.update(profiles=[PROFILE], ros_installed=['jazzy'], tools={'gz': '/usr/bin/gz'})
        self.assertIn('Harmonic', compatibility(self.lesson, self.environment))
        self.assertEqual(lesson_compatibility(self.lesson, self.environment)['status'], 'incomplete_detection')

    def test_fortress_and_ionic_are_not_silently_claimed_compatible(self):
        for version in (6, 9):
            self.environment['modern']['sim_major'] = version
            self.assertEqual(lesson_compatibility(self.lesson, self.environment)['status'], 'unsupported')

    def test_missing_ros2_package_is_named(self):
        self.environment['modern']['packages'].remove('ros_gz_bridge')
        self.assertIn('ros_gz_bridge', compatibility(self.lesson, self.environment))

    def test_independent_sdf_remains_available_without_ros(self):
        self.lesson.requires_ros = False
        self.environment['modern']['ros_available'] = False
        self.assertIsNone(compatibility(self.lesson, self.environment, gui=False))

    def test_noetic_paths_and_overlay_do_not_leak_into_ros2_environment(self):
        overlay = str(Path(self.temp.name) / 'old-catkin' / 'devel')
        original = {'ROS_MASTER_URI': 'http://127.0.0.1:11311', 'ROS_DISTRO': 'noetic', 'ROS_VERSION': '1',
                    'CMAKE_PREFIX_PATH': overlay, 'PATH': overlay + os.sep + 'bin' + os.pathsep + '/usr/bin',
                    'PYTHONPATH': overlay + os.sep + 'lib', 'GAZEBO_PLUGIN_PATH': '/old/plugins', 'DISPLAY': ':1'}
        sanitized = ros2_base_environment(original)
        self.assertNotIn('ROS_MASTER_URI', sanitized)
        self.assertNotIn('GAZEBO_PLUGIN_PATH', sanitized)
        self.assertNotIn(overlay, sanitized['PATH'])
        self.assertEqual(sanitized['DISPLAY'], ':1')
        self.assertEqual(original['ROS_DISTRO'], 'noetic')

    def test_foreign_ionic_plugin_and_config_paths_are_removed_before_setup(self):
        foreign = {'PATH': '/usr/bin', 'DISPLAY': ':4',
                   'GZ_SIM_SYSTEM_PLUGIN_PATH': '/ionic/lib', 'GZ_SIM_SERVER_CONFIG_PATH': '/ionic/server.config',
                   'GZ_CONFIG_PATH': '/ionic/gz-tools', 'GZ_SIM_RESOURCE_PATH': '/ionic/models',
                   'GZ_RENDERING_PLUGIN_PATH': '/ionic/rendering', 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/fortress/lib',
                   'IGN_GAZEBO_SERVER_CONFIG_PATH': '/fortress/config', 'IGN_CONFIG_PATH': '/fortress/tools'}
        clean = ros2_base_environment(foreign)
        self.assertEqual(clean, {'PATH': '/usr/bin', 'DISPLAY': ':4'})
        self.assertIn('GZ_CONFIG_PATH', foreign)

    def test_probe_and_runtime_snapshot_use_jazzy_child_tools(self):
        def setup(path, environment):
            self.assertNotIn('GZ_CONFIG_PATH', environment)
            return {'PATH': '/selected/jazzy/bin', 'GZ_SIM_SYSTEM_PLUGIN_PATH': '/selected/jazzy/plugins',
                    'ROS_DISTRO': 'jazzy', 'ROS_VERSION': '2'}
        def which(name, path=None):
            self.assertEqual(path, '/selected/jazzy/bin')
            return '/selected/jazzy/bin/' + name
        def output(command, **options):
            self.assertEqual(options['env']['PATH'], '/selected/jazzy/bin')
            return '8.11.0' if 'sim' in command else 'rclpy\nros_gz_bridge'
        with mock.patch.dict(os.environ, {'GZ_CONFIG_PATH': '/foreign/ionic'}), \
                mock.patch('gazeboteach.environment._load_setup', side_effect=setup), \
                mock.patch('gazeboteach.modern.shutil.which', side_effect=which), \
                mock.patch('gazeboteach.environment._output', side_effect=output):
            selected = detect_modern({'gz': '/foreign/bin/gz', 'colcon': None}, ['jazzy'], [])
        self.assertEqual(selected['tools']['colcon'], '/selected/jazzy/bin/colcon')
        self.assertEqual(selected['gz'], '/selected/jazzy/bin/gz')
        self.assertEqual(selected['sim_major'], 8)

    def test_colocated_profiles_read_versions_and_packages_in_separate_ros_setups(self):
        def setup(path, environment):
            distro = path.parent.name
            self.assertNotIn('ROS_DISTRO', environment)
            return {'PATH': '/selected/' + distro, 'ROS_DISTRO': distro, 'ROS_VERSION': '2'}

        def output(command, **options):
            selected = options['env']['ROS_DISTRO']
            if command[0] == 'ros2':
                return 'rclpy\n' + ('ros_gz_bridge' if selected == 'humble' else 'service_msgs')
            self.assertEqual(command[1], 'gazebo' if selected == 'humble' else 'sim')
            return '6.18.0' if selected == 'humble' else '8.15.0'

        with mock.patch.dict(os.environ, {'ROS_DISTRO': 'noetic'}), \
                mock.patch('gazeboteach.environment._load_setup', side_effect=setup), \
                mock.patch('gazeboteach.modern.shutil.which', side_effect=lambda name, path: path + '/' + name), \
                mock.patch('gazeboteach.environment._output', side_effect=output):
            result = detect_modern({}, ['humble', 'jazzy'], [])
        self.assertEqual(result['profiles'], [PROFILE, FORTRESS_PROFILE])
        self.environment['modern'] = result
        fortress = make_lesson(Path(self.temp.name) / 'fortress', profile=FORTRESS_PROFILE, requires_ros=True,
                               requires_ros_packages=['ros_gz_bridge'])
        self.assertIsNone(compatibility(fortress, self.environment, gui=False))
        self.assertIn('ros_gz_bridge', compatibility(self.lesson, self.environment, gui=False))
        self.assertEqual(modern_capability(self.environment, FORTRESS_PROFILE)['ign'], '/selected/humble/ign')

    def test_declared_variant_keeps_distinct_storage_and_selects_own_world(self):
        root = Path(self.temp.name)
        folder = root / 'courses' / 'modern' / '01-fixture'
        make_lesson(folder, profile=PROFILE, variants={FORTRESS_PROFILE: {'world': 'fortress.sdf'}})
        (folder / 'fortress.sdf').write_text("<sdf version='1.9'><world name='fortress'/></sdf>", encoding='utf-8')
        lessons, errors = load_catalog(root)
        self.assertFalse(errors)
        self.assertEqual({lesson.key: lesson.world.name for lesson in lessons},
                         {PROFILE + ':01': 'world.sdf', FORTRESS_PROFILE + ':01': 'fortress.sdf'})
        from gazeboteach.storage import Storage
        storage = Storage(root / 'data')
        self.assertNotEqual(storage.prepare(lessons[0]), storage.prepare(lessons[1]))

    def test_outer_tool_cannot_fill_a_missing_selected_tool_or_hide_available_tool(self):
        self.lesson.data['requires_tools'] = ['colcon']
        self.environment['tools']['colcon'] = '/outer/ionic/bin/colcon'
        self.environment['modern']['tools']['colcon'] = None
        self.assertIn('colcon', compatibility(self.lesson, self.environment, gui=False))
        self.assertEqual(lesson_compatibility(self.lesson, self.environment, gui=False, terminals=False)['status'], 'missing_dependencies')
        self.environment['tools']['colcon'] = None
        self.environment['modern']['tools']['colcon'] = '/selected/jazzy/bin/colcon'
        self.assertIsNone(compatibility(self.lesson, self.environment, gui=False))
        del self.environment['modern']['tools']['colcon']
        row = lesson_compatibility(self.lesson, self.environment, gui=False, terminals=False)
        self.assertEqual(row['status'], 'incomplete_detection')
        self.assertIn('modern.tools:colcon', row['reasons'][0]['fields'])

    def test_probe_failures_propagate_into_global_summary_without_duplicate_errors(self):
        self.environment['modern']['sim_probe_error'] = 'Sim version probe timed out'
        self.environment['modern']['error'] = 'Jazzy package index unavailable'
        self.environment['detection_errors'] = ['Sim version probe timed out']
        report = build_compatibility_report([self.lesson], self.environment, gui=False, terminals=False)
        self.assertEqual(report['courses'][0]['status'], 'incomplete_detection')
        self.assertFalse(report['detection']['complete'])
        self.assertEqual(report['detection']['incomplete_checks'], ['modern_ros_environment', 'modern_sim_version'])
        self.assertEqual(report['detection']['errors'], ['Sim version probe timed out', 'Jazzy package index unavailable'])

    def test_missing_modern_snapshot_cannot_report_complete_detection(self):
        self.environment.pop('modern')
        report = build_compatibility_report([self.lesson], self.environment, gui=False, terminals=False)
        self.assertFalse(report['detection']['complete'])
        self.assertIn('modern', report['detection']['incomplete_checks'])

    def test_modern_failure_metadata_does_not_disable_classic_courses(self):
        from tests.test_core import environment
        classic = make_lesson(Path(self.temp.name)/'classic')
        snapshot = environment(incomplete_checks=['modern_sim_version'], modern=self.environment['modern'])
        snapshot['modern']['sim_probe_error'] = 'modern probe failed'
        row = lesson_compatibility(classic, snapshot, gui=False, terminals=False)
        self.assertEqual(row['status'], 'ready')


class ModernControlTests(unittest.TestCase):
    def test_omitted_proto_scalars_are_zero_but_not_an_absent_sample(self):
        sample = parse_stats('sim_time {}\nreal_time {sec: 1}\n')
        self.assertEqual(sample['iterations'], 0)
        self.assertEqual(sample['sim_time'], 0)
        self.assertFalse(sample['paused'])
        sample = parse_stats('sim_time {sec: 7 nsec: 120000000}\niterations: 7120\npaused: true')
        self.assertAlmostEqual(sample['sim_time'], 7.12)
        self.assertTrue(sample['paused'])
        with self.assertRaises(RuntimeError):
            parse_stats('gz: invalid command')

    def test_step_requires_exact_iteration_change_and_sends_mutation_once(self):
        session = object.__new__(ModernLessonSession)
        session.pause = mock.Mock()
        session._control = mock.Mock()
        session._record = mock.Mock()
        session._stats = mock.Mock(side_effect=[{'iterations': 25}, {'iterations': 25, 'paused': True},
                                                {'iterations': 26, 'paused': True}])
        session.step(1)
        session._control.assert_called_once_with('pause: true, multi_step: 1')
        self.assertEqual(session._record.call_args.kwargs['native_result']['stats']['iterations'], 26)

    def test_a_second_client_extra_step_is_not_false_success(self):
        session = object.__new__(ModernLessonSession)
        session.pause = mock.Mock(); session._control = mock.Mock(); session._record = mock.Mock()
        session._stats = mock.Mock(side_effect=[{'iterations': 25}, {'iterations': 27, 'paused': True}])
        with self.assertRaisesRegex(RuntimeError, '超过目标'):
            session.step()
        session._control.assert_called_once()
        session._record.assert_not_called()

    def test_model_only_reset_is_explicitly_unsupported_not_silent_all_reset(self):
        session = object.__new__(ModernLessonSession)
        session._control = mock.Mock()
        with self.assertRaisesRegex(RuntimeError, '不支持'):
            session.reset_models()
        session._control.assert_not_called()


class ModernGuiEvidenceTests(unittest.TestCase):
    def test_black_white_and_near_blank_drawables_cannot_pass_gui_evidence(self):
        from tests.modern_runtime_acceptance import image_metrics
        class Color:
            def __init__(self, value):self.value=value
            def red(self):return self.value[0]
            def green(self):return self.value[1]
            def blue(self):return self.value[2]
        class Frame:
            def __init__(self, function):self.function=function
            def width(self):return 1000
            def height(self):return 845
            def pixelColor(self,x,y):return Color(self.function(x,y))
        for pixels in (lambda x,y:(0,0,0), lambda x,y:(255,255,255),
                       lambda x,y:((x%256,y%256,100) if x<8 else (0,0,0))):
            self.assertFalse(image_metrics(Frame(pixels))['nontrivial_pixels'])
        varied = image_metrics(Frame(lambda x,y:(x%256,y%256,(x+y)%256)))
        self.assertTrue(varied['nontrivial_pixels'])
        self.assertGreater(varied['distinct_sampled_colors'], 8)


if __name__ == '__main__':
    unittest.main()
