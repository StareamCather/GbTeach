"""GUI initialization lifecycle tests; all process creation is simulated."""

import itertools
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import Lesson
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage


class FakeProcess:
    def __init__(self, pid, result=0, callback=None, timeout=False):
        self.pid = pid
        self.returncode = None
        self.result = result
        self.callback = callback
        self.timeout = timeout

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.callback:
            self.callback()
        if self.timeout:
            raise subprocess.TimeoutExpired("injected GUI hook", timeout)
        self.returncode = self.result
        return self.returncode


class AfterGuiTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source"
        self.source.mkdir()
        (self.source / "world.sdf").write_text("<sdf version='1.7'><world name='unit'/></sdf>", encoding="utf-8")
        (self.source / "lesson.md").write_text("# GUI hook fixture\n", encoding="utf-8")
        self.data = {"id": "00", "title": "GUI hook fixture", "profile": "classic11-noetic",
                     "requires_ros": False, "gui_ready_timeout": .01,
                     "after_gui": ["fixture-camera-ready"]}
        self.environment = {"DISPLAY": ":fixture", "PATH": "/fixture",
                            "GAZEBO_MASTER_URI": "http://127.0.0.1:19000",
                            "IGN_PARTITION": "fixture"}
        with mock.patch("gazeboteach.runtime.lesson_environment", return_value=self.environment.copy()):
            self.session = LessonSession(self.root, Lesson(self.source, self.data),
                                         storage=Storage(self.root / "data"))
        self.server = FakeProcess(900001)
        self.other = FakeProcess(900002)
        self.session.server = self.server
        self.session.state = "运行中"
        self.session.processes.extend((("Gazebo 服务端", self.server), ("已有辅助进程", self.other)))
        self.arguments, self.spawn_options, self.children, self.terminated = [], [], [], []
        self.hook_result, self.hook_timeout = 0, False
        self.hook_results = []
        self.hook_callback = self.gui_callback = None
        clock = itertools.count()
        # These fixtures exercise the legacy GUI startup path and replace its
        # process boundary. The persistent SDK protocol has its own stdio tests;
        # do not let this generic Popen double enter a real SDK/toolchain probe.
        for target, options in (
                ("gazeboteach.native_control.try_create", {"return_value": (
                    None, {"backend": "legacy-cli", "reason": "test legacy GUI startup path"})}),
                ("gazeboteach.runtime.subprocess.Popen", {"side_effect": self.spawn}),
                ("gazeboteach.runtime.terminate_owned", {"side_effect": self.terminate}),
                ("gazeboteach.runtime.shutil.which", {"return_value": "/fixture/gzclient"}),
                ("gazeboteach.runtime._process_info", {"return_value": None}),
                ("gazeboteach.runtime.time.monotonic", {"side_effect": lambda: next(clock)})):
            patcher = mock.patch(target, **options)
            patcher.start()
            self.addCleanup(patcher.stop)

    def spawn(self, arguments, **options):
        self.arguments.append(arguments)
        self.spawn_options.append(options)
        if arguments[0] == "bash":
            result = self.hook_results.pop(0) if self.hook_results else self.hook_result
            process = FakeProcess(910000+len(self.children), result=result,
                                  callback=self.hook_callback, timeout=self.hook_timeout)
        else:
            process = FakeProcess(910000+len(self.children))
            if arguments[0] == "gzclient" and self.gui_callback:
                self.gui_callback(process)
        self.children.append(process)
        return process

    def terminate(self, process, **options):
        self.terminated.append(process)
        if process.returncode is None:
            process.returncode = -15

    def hook_commands(self):
        return [arguments[-1] for arguments in self.arguments if arguments[0] == "bash"]

    def assert_server_preserved(self):
        self.assertIsNone(self.server.poll())
        self.assertIsNone(self.other.poll())
        health = self.session.health()
        self.assertTrue(health["running"])
        self.assertFalse(health["gui_running"])
        self.assertEqual(health["failed_processes"], [])

    def test_catalog_accepts_omitted_and_empty_hooks(self):
        omitted = dict(self.data)
        omitted.pop("after_gui")
        self.assertNotIn("after_gui", Lesson(self.source, omitted).data)
        self.assertEqual(Lesson(self.source, dict(self.data, after_gui=[])).data["after_gui"], [])

    def test_fresh_sessions_isolate_gazebo_logs_without_changing_parent_or_global_cache(self):
        external = self.root / "external-gazebo-log"
        external.mkdir()
        marker = external / "keep-global-cache.bin"
        marker.write_bytes(b"existing-user-cache")
        incoming = dict(self.environment, GAZEBO_LOG_PATH=str(external), XAUTHORITY="/fixture/Xauthority",
                        GAZEBO_RESOURCE_PATH="/fixture/media", GAZEBO_MODEL_PATH="/fixture/models")
        with mock.patch.dict(os.environ, {"GAZEBO_LOG_PATH": str(external)}):
            parent_environment = dict(os.environ)
            with mock.patch("gazeboteach.runtime.lesson_environment", side_effect=lambda *a, **k: incoming.copy()):
                first = LessonSession(self.root, Lesson(self.source, self.data), storage=Storage(self.root / "fresh"))
                second = LessonSession(self.root, Lesson(self.source, self.data), storage=Storage(self.root / "fresh"))
            self.assertTrue(dict(os.environ) == parent_environment, "Session initialization changed the parent environment")
        self.assertNotEqual(first.env["GAZEBO_LOG_PATH"], second.env["GAZEBO_LOG_PATH"])
        for session in (first, second):
            actual = Path(session.env["GAZEBO_LOG_PATH"])
            self.assertEqual(actual, session.session_dir / "gazebo-log")
            self.assertTrue(actual.is_dir())
            self.assertEqual(actual.parent.resolve(), session.session_dir.resolve())
            metadata = json.loads((session.session_dir / "session.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["gazebo_log_path"], str(actual))
            self.assertIn(str(actual), session.log_path.read_text(encoding="utf-8"))
            for key in ("DISPLAY", "XAUTHORITY", "GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_PATH"):
                self.assertEqual(session.env[key], incoming[key])
        self.assertEqual(list(external.iterdir()), [marker])
        self.assertEqual(marker.read_bytes(), b"existing-user-cache")

    def test_all_managed_spawns_inherit_log_isolation_from_headless_session(self):
        self.session.server = None
        self.session.processes = []
        report = {"classic_version": "11.15.1", "profiles": ["classic11-noetic"],
                  "tools": {"gzserver": "/fixture/gzserver", "gz": "/fixture/gz", "bash": "/fixture/bash"}}
        with mock.patch("gazeboteach.runtime.detect_environment", return_value=report), \
                mock.patch.object(self.session, "_wait_world"):
            self.session.start(gui=False, terminals=False)
        self.assertEqual([a[0] for a in self.arguments], ["gzserver"])
        self.session.open_gui()
        self.session.command("fixture-additional-command")
        self.assertEqual([a[0] for a in self.arguments], ["gzserver", "gzclient", "bash", "bash"])
        expected = str(self.session.session_dir / "gazebo-log")
        for options in self.spawn_options:
            self.assertIs(options["env"], self.session.env)
            self.assertEqual(options["env"]["GAZEBO_LOG_PATH"], expected)

    def test_catalog_rejects_invalid_hook_shapes_and_items(self):
        for value in ("echo camera", None, {}, ("echo camera",), [None], [True], [1], [""], ["  "]):
            with self.subTest(after_gui=value), self.assertRaisesRegex(ValueError, "after_gui"):
                Lesson(self.source, dict(self.data, after_gui=value))

    def test_catalog_validates_bounded_integer_gui_attempts(self):
        self.assertEqual(Lesson(self.source, self.data).data.get("gui_start_attempts", 1), 1)
        for attempts in (1, 2, 3):
            self.assertEqual(Lesson(self.source, dict(self.data, gui_start_attempts=attempts)).data["gui_start_attempts"], attempts)
        for value in (True, False, None, 1.0, "2", 0, 4, -1, []):
            with self.subTest(gui_start_attempts=value), self.assertRaisesRegex(ValueError, "gui_start_attempts"):
                Lesson(self.source, dict(self.data, gui_start_attempts=value))

    def test_first_hook_failure_retries_new_client_on_same_server(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        self.hook_results = [7, 0]
        self.session.open_gui()
        self.assertEqual([a[0] for a in self.arguments], ["gzclient", "bash", "gzclient", "bash"])
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertIs(self.session.gui, self.children[2])
        self.assertNotEqual(self.children[0].pid, self.session.gui.pid)
        self.assertIsNone(self.server.poll())
        self.assertIsNone(self.other.poll())
        self.assertTrue(self.session.health()["gui_running"])
        self.assertIn("下一次 2/3", self.session.log_path.read_text(encoding="utf-8"))

    def test_three_failed_attempts_exhaust_budget_without_stopping_server(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        self.hook_result = 7
        with self.assertRaisesRegex(RuntimeError, "已尝试 3 次仍失败"):
            self.session.open_gui()
        clients = [p for a, p in zip(self.arguments, self.children) if a[0] == "gzclient"]
        self.assertEqual(len(clients), 3)
        self.assertEqual(self.terminated, clients)
        self.assertIsNone(self.session.gui)
        self.assert_server_preserved()

    def test_cancellation_never_uses_remaining_gui_attempts(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        self.gui_callback = lambda process: self.session.cancelled.set()
        with self.assertRaisesRegex(RuntimeError, "已取消"):
            self.session.open_gui()
        self.assertEqual([a[0] for a in self.arguments], ["gzclient"])
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertIsNone(self.session.gui)

    def test_server_exit_during_hook_never_retries_gui(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        self.hook_callback = lambda: setattr(self.server, "returncode", 2)
        self.hook_result = 7
        with self.assertRaisesRegex(RuntimeError, "命令执行失败"):
            self.session.open_gui()
        self.assertEqual([a[0] for a in self.arguments], ["gzclient", "bash"])
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertIsNone(self.session.gui)

    def test_client_exit_before_hook_can_recover_with_new_client(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        def exit_first(process):
            if not self.children:
                process.returncode = 3
        self.gui_callback = exit_first
        self.session.open_gui()
        self.assertEqual([a[0] for a in self.arguments], ["gzclient", "gzclient", "bash"])
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertTrue(self.session.health()["gui_running"])

    def test_new_gui_runs_hooks_in_order_once(self):
        self.session.lesson.data["after_gui"] = ["fixture-first", "fixture-second"]
        self.session.open_gui()
        self.assertEqual([arguments[0] for arguments in self.arguments], ["gzclient", "bash", "bash"])
        self.assertEqual(self.hook_commands(), ["fixture-first", "fixture-second"])
        self.assertTrue(self.session.health()["gui_running"])
        self.assertEqual(self.terminated, [])

    def test_existing_gui_does_not_repeat_hook_or_spawn(self):
        self.session.lesson.data["gui_start_attempts"] = 3
        self.session.open_gui()
        original_gui = self.session.gui
        self.session.open_gui()
        self.assertIs(self.session.gui, original_gui)
        self.assertEqual(self.hook_commands(), ["fixture-camera-ready"])
        self.assertEqual(sum(arguments[0] == "gzclient" for arguments in self.arguments), 1)

    def test_reopening_exited_gui_initializes_only_new_client(self):
        self.session.open_gui()
        previous = self.session.gui
        previous.returncode = 0
        self.session.open_gui()
        self.assertIsNot(self.session.gui, previous)
        self.assertEqual(self.hook_commands(), ["fixture-camera-ready", "fixture-camera-ready"])
        self.assertEqual(self.terminated, [])
        self.assertEqual(self.session.health()["failed_processes"], [])

    def test_explicit_empty_hook_keeps_legacy_gui_path(self):
        self.session.lesson.data["after_gui"] = []
        self.session.open_gui()
        self.assertEqual(self.hook_commands(), [])
        self.assertTrue(self.session.health()["gui_running"])

    def test_headless_start_never_runs_gui_hook(self):
        self.session.server = None
        self.session.processes = []
        report = {"classic_version": "11.15.1", "profiles": ["classic11-noetic"],
                  "tools": {"gzserver": "/fixture/gzserver", "gz": "/fixture/gz", "bash": "/fixture/bash"}}
        with mock.patch("gazeboteach.runtime.detect_environment", return_value=report), \
                mock.patch.object(self.session, "_wait_world"):
            self.session.start(gui=False, terminals=False)
        self.assertEqual([arguments[0] for arguments in self.arguments], ["gzserver"])
        self.assertEqual(self.hook_commands(), [])
        self.assertTrue(self.session.running)

    def test_hook_failure_closes_new_gui_only_and_allows_retry(self):
        self.hook_result = 7
        with self.assertRaisesRegex(RuntimeError, "命令执行失败（7）"):
            self.session.open_gui()
        failed_gui = self.children[0]
        self.assertEqual(self.terminated, [failed_gui])
        self.assertIsNone(self.session.gui)
        self.assert_server_preserved()
        log = self.session.log_path.read_text(encoding="utf-8")
        self.assertIn("fixture-camera-ready", log)
        self.assertIn("图形初始化失败", log)
        self.hook_result = 0
        self.session.open_gui()
        self.assertIsNot(self.session.gui, failed_gui)
        self.assertTrue(self.session.health()["gui_running"])

    def test_hook_timeout_cleans_hook_group_and_new_gui_only(self):
        self.hook_timeout = True
        with self.assertRaisesRegex(RuntimeError, "命令超过 90 秒"):
            self.session.open_gui()
        gui, hook = self.children
        self.assertEqual(self.terminated, [hook, gui])
        self.assertIsNone(self.session.gui)
        self.assert_server_preserved()

    def test_client_exit_before_hook_never_runs_hook(self):
        self.gui_callback = lambda process: setattr(process, "returncode", 3)
        with self.assertRaisesRegex(RuntimeError, "客户端提前退出"):
            self.session.open_gui()
        self.assertEqual(self.hook_commands(), [])
        self.assertEqual(self.terminated, [self.children[0]])
        self.assert_server_preserved()

    def test_client_exit_during_hook_is_not_reported_ready(self):
        self.hook_callback = lambda: setattr(self.session.gui, "returncode", 4)
        with self.assertRaisesRegex(RuntimeError, "客户端提前退出"):
            self.session.open_gui()
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertIsNone(self.session.gui)
        self.assert_server_preserved()

    def test_cancellation_after_spawn_cleans_gui_without_running_hook(self):
        self.gui_callback = lambda process: self.session.cancelled.set()
        with self.assertRaisesRegex(RuntimeError, "已取消"):
            self.session.open_gui()
        self.assertEqual(self.terminated, [self.children[0]])
        self.assertIsNone(self.session.gui)
        self.assertEqual(self.hook_commands(), [])
        self.assertIsNone(self.server.poll())

    def test_complete_start_failure_keeps_existing_session_cleanup(self):
        self.session.server = None
        self.session.processes = []
        self.hook_result = 9
        report = {"classic_version": "11.15.1", "profiles": ["classic11-noetic"],
                  "tools": {"gzserver": "/fixture/gzserver", "gz": "/fixture/gz", "bash": "/fixture/bash"}}
        with mock.patch("gazeboteach.runtime.detect_environment", return_value=report), \
                mock.patch.object(self.session, "_wait_world"), \
                mock.patch("gazeboteach.runtime._token_processes", return_value=[]):
            with self.assertRaisesRegex(RuntimeError, "命令执行失败（9）"):
                self.session.start(gui=True, terminals=False)
        self.assertEqual(self.session.state, "已结束")
        self.assertTrue(self.session.cancelled.is_set())
        self.assertIsNotNone(self.session.server.poll())
        self.assertIsNone(self.session.gui)


if __name__ == "__main__":
    unittest.main()
