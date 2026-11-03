"""Lesson-local ROS discovery and managed startup readiness; no ROS is started."""

import os
import signal
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import Lesson
from gazeboteach.environment import lesson_environment
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage
from tests.test_core import make_lesson


class PackagePathTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.lesson = make_lesson(self.root / "source", requires_ros=True,
                                  ros_package_paths=["ros_ws/src"])

    def build_environment(self, lesson, workspace, gazebo_port=19001, ros_port=19002, token="fixture"):
        def is_setup(path):
            return str(path).replace("\\", "/").endswith("/opt/ros/noetic/setup.bash")

        def source_setup(setup, environment):
            return dict(environment, ROS_PACKAGE_PATH="/fixture/noetic/share")

        with mock.patch("gazeboteach.environment.Path.is_file", is_setup), \
                mock.patch("gazeboteach.environment._load_setup", side_effect=source_setup):
            return lesson_environment(self.root, workspace, lesson, gazebo_port, ros_port, token)

    def test_manifest_rejects_bad_shapes_non_ros_and_source_path_escape(self):
        for value in ("ros_ws/src", None, {}, [None], [""], ["  "]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Lesson(self.lesson.folder, dict(self.lesson.data, ros_package_paths=value))
        for value in (["../outside"], [str(self.root / "outside")], ["."]):
            with self.subTest(path=value), self.assertRaises(ValueError):
                Lesson(self.lesson.folder, dict(self.lesson.data, ros_package_paths=value))
        for value in ([], ["ros_ws/src"]):
            with self.subTest(non_ros=value), self.assertRaisesRegex(ValueError, "requires_ros"):
                Lesson(self.lesson.folder, dict(self.lesson.data, requires_ros=False, ros_package_paths=value))

    def test_each_workspace_prepends_own_package_path_without_mutating_parent(self):
        inherited = os.pathsep.join(("/fixture/user/packages", "/fixture/noetic/share"))
        with mock.patch.dict(os.environ, {"ROS_PACKAGE_PATH": inherited, "DISPLAY": ":fixture",
                                         "XAUTHORITY": "/fixture/Xauthority"}):
            before = dict(os.environ)
            first = self.build_environment(self.lesson, self.root / "personal-a")
            second = self.build_environment(self.lesson, self.root / "personal-b", 19003, 19004, "second")
            self.assertEqual(dict(os.environ), before)
        for actual, personal in ((first, "personal-a"), (second, "personal-b")):
            self.assertEqual(actual["ROS_PACKAGE_PATH"].split(os.pathsep), [
                str((self.root / personal / "ros_ws/src").resolve()),
                "/fixture/noetic/share", "/fixture/user/packages"])
            self.assertEqual(actual["DISPLAY"], ":fixture")
            self.assertEqual(actual["XAUTHORITY"], "/fixture/Xauthority")
        for key in ("ROS_MASTER_URI", "GAZEBO_MASTER_URI", "IGN_PARTITION", "GAZEBOTEACH_SESSION_TOKEN"):
            self.assertNotEqual(first[key], second[key], key)

    def test_undeclared_paths_keep_existing_setup_behavior(self):
        data = dict(self.lesson.data)
        data.pop("ros_package_paths")
        with mock.patch.dict(os.environ, {"ROS_PACKAGE_PATH": "/fixture/user/packages"}):
            actual = self.build_environment(Lesson(self.lesson.folder, data), self.root / "legacy")
        self.assertEqual(actual["ROS_PACKAGE_PATH"], "/fixture/noetic/share")

    def test_personal_symlink_escape_is_rechecked_after_manifest_load(self):
        workspace, outside = self.root / "personal", self.root / "outside"
        workspace.mkdir()
        outside.mkdir()
        try:
            (workspace / "ros_ws").symlink_to(outside, target_is_directory=True)
        except (OSError, NotImplementedError) as exc:
            self.skipTest("Creating a directory symlink is unavailable: " + str(exc))
        with self.assertRaisesRegex(ValueError, "personal workspace"):
            self.build_environment(self.lesson, workspace)


class ManagedStartupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        lesson = make_lesson(self.root / "source", prepare=["prepare-fixture"],
                             after_start=["world-ready"],
                             startup=[{"name": "driver", "command": "launch-driver"},
                                      {"name": "monitor", "command": "launch-monitor"}],
                             after_startup=["graph-ready", "messages-ready"], after_startup_timeout=2.5)
        with mock.patch("gazeboteach.runtime.lesson_environment", return_value={
                "GAZEBO_MASTER_URI": "http://127.0.0.1:19001", "ROS_MASTER_URI": "http://127.0.0.1:19002",
                "IGN_PARTITION": "fixture", "PATH": "/fixture", "DISPLAY": ":fixture"}):
            self.session = LessonSession(self.root, lesson, storage=Storage(self.root / "data"))
        self.calls, self.children, self.terminated, self.timeouts = [], [], [], []
        self.fail_command = self.cancel_command = self.timeout_command = None
        report = {"classic_version": "11.15.1", "profiles": ["classic11-noetic"],
                  "display": ":fixture", "tools": {name: "/fixture/" + name for name in
                  ("gzserver", "gzclient", "gz", "bash", "gnome-terminal")}}
        for target, options in (
                ("gazeboteach.runtime.detect_environment", {"return_value": report}),
                ("gazeboteach.runtime.subprocess.Popen", {"side_effect": self.spawn}),
                ("gazeboteach.runtime.terminate_owned", {"side_effect": self.terminate}),
                ("gazeboteach.runtime._process_info", {"return_value": None}),
                ("gazeboteach.runtime._token_processes", {"return_value": []})):
            patcher = mock.patch(target, **options)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.session._wait_world = lambda **kw: self.calls.append("server-ready")
        self.session.open_gui = mock.Mock(side_effect=lambda: self.calls.append("gui"))
        self.session.open_terminal = mock.Mock(side_effect=lambda **kw: self.calls.append("logs" if kw.get("logs") else "terminal"))

    def spawn(self, arguments, **options):
        command = arguments[-1] if arguments[0] == "bash" else arguments[0]
        self.calls.append(command)
        process = mock.Mock(pid=920000+len(self.children))
        process.returncode = None
        process.poll.side_effect = lambda: process.returncode

        def wait(timeout=None):
            self.timeouts.append((command, timeout))
            if command == self.timeout_command:
                raise subprocess.TimeoutExpired(command, timeout)
            if command == self.cancel_command:
                self.session.cancelled.set()
            process.returncode = 7 if command == self.fail_command else 0
            return process.returncode

        process.wait.side_effect = wait
        self.children.append(process)
        return process

    def terminate(self, process, **options):
        self.terminated.append(process)
        if process.returncode is None:
            process.returncode = -15

    def test_hooks_wait_for_all_startup_processes_before_opening_windows(self):
        self.session.start(gui=True, terminals=True)
        self.assertEqual(self.calls, ["prepare-fixture", "gzserver", "server-ready", "world-ready",
                                     "launch-driver", "launch-monitor", "graph-ready", "messages-ready",
                                     "gui", "logs", "terminal"])
        self.assertEqual(self.timeouts[-2:], [("graph-ready", 2.5), ("messages-ready", 2.5)])
        self.assertEqual(self.session.state, "运行中")
        self.assertEqual(self.terminated, [])

    def test_omitted_hooks_leave_legacy_startup_order(self):
        self.session.lesson.data.pop("after_startup")
        self.session.start(gui=False, terminals=False)
        self.assertEqual(self.calls, ["prepare-fixture", "gzserver", "server-ready", "world-ready",
                                     "launch-driver", "launch-monitor"])

    def test_opt_in_graceful_shutdown_only_signals_the_owned_server_differently(self):
        self.session.lesson.data['graceful_server_shutdown']=True
        self.session.start(gui=False,terminals=False)
        with mock.patch('gazeboteach.runtime.terminate_owned',side_effect=self.terminate) as terminate:
            self.session.stop()
        changed=[entry for entry in terminate.call_args_list if entry.kwargs.get('first_signal') is not None]
        self.assertEqual(changed,[mock.call(self.session.server,first_signal=signal.SIGINT)])
        self.assertTrue(all(child.poll() is not None for child in self.children))

    def test_graceful_shutdown_manifest_flag_must_be_boolean(self):
        with self.assertRaisesRegex(ValueError,'graceful_server_shutdown'):
            Lesson(self.session.lesson.folder,dict(self.session.lesson.data,graceful_server_shutdown='yes'))

    def assert_failed_stack_cleaned(self, message):
        with self.assertRaisesRegex(RuntimeError, message):
            self.session.start(gui=True, terminals=True)
        self.assertIn("graph-ready", self.calls)
        self.assertNotIn("messages-ready", self.calls)
        self.session.open_gui.assert_not_called()
        self.session.open_terminal.assert_not_called()
        self.assertEqual(self.session.state, "已结束")
        self.assertTrue(self.session.cancelled.is_set())
        self.assertTrue(all(child.poll() is not None for child in self.children))
        self.assertTrue(self.session.error)

    def test_hook_failure_cleans_owned_stack_before_opening_windows(self):
        self.fail_command = "graph-ready"
        self.assert_failed_stack_cleaned("命令执行失败（7）")

    def test_hook_timeout_cleans_owned_stack_before_opening_windows(self):
        self.timeout_command = "graph-ready"
        self.assert_failed_stack_cleaned("命令超过 2.5 秒")

    def test_hook_cancellation_cleans_owned_stack_before_opening_windows(self):
        self.cancel_command = "graph-ready"
        self.assert_failed_stack_cleaned("实验操作已取消")

    def test_hook_schema_rejects_invalid_commands_and_unbounded_timeouts(self):
        for value in ("ready", None, [None], ["  "]):
            with self.subTest(commands=value), self.assertRaisesRegex(ValueError, "after_startup"):
                Lesson(self.session.lesson.folder, dict(self.session.lesson.data, after_startup=value))
        for value in (0, -1, True, "1", None, float("inf"), float("nan")):
            with self.subTest(timeout=value), self.assertRaisesRegex(ValueError, "after_startup_timeout"):
                Lesson(self.session.lesson.folder, dict(self.session.lesson.data, after_startup_timeout=value))


if __name__ == "__main__":
    unittest.main()
