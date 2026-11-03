"""Qt widget/state tests with synthetic facts, not Linux/Gazebo acceptance.

On Windows these run with the offscreen Qt platform. No simulator, terminal,
Docker daemon or network connection is started. Real ikaros acceptance remains
a separate deployment check.
"""

import copy
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from PyQt5 import QtCore, QtWidgets, QtTest, QtGui
except ImportError:
    QtCore = QtWidgets = None

if QtWidgets is not None:
    from gazeboteach.app import MainWindow
    from gazeboteach.catalog import load_catalog
    from gazeboteach.storage import Storage


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipIf(QtWidgets is None, "PyQt5 is required for Qt state tests")
class EnvironmentDialogStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.application.setQuitOnLastWindowClosed(False)
        cls.lessons, errors = load_catalog(ROOT)
        if errors:
            raise AssertionError(errors)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.settings = QtCore.QSettings(str(self.root / "settings.ini"), QtCore.QSettings.IniFormat)
        tools = {"gzserver", "gzclient", "gz", "bash", "roscore", "gnome-terminal", "xterm"}
        tools.update(name for lesson in self.lessons for name in lesson.data.get("requires_tools", []))
        self.detected = {
            "os": "Ubuntu 20.04 synthetic fixture", "architecture": "x86_64", "python": "3.8.10",
            "classic_version": "11.15.1", "modern_simulators": [], "profiles": ["classic11-noetic"],
            "modern": {"sim_major": None, "gz": None, "ros_available": False, "packages": [], "error": ""},
            "course_profile": "classic11-noetic", "ros_installed": ["noetic"], "ros_active": "",
            "display": ":7", "tools": {name: "/fixture/bin/" + name for name in tools},
            "plugins": sorted({name for lesson in self.lessons for name in lesson.data.get("requires_plugins", [])}),
            "ros_packages": sorted({name for lesson in self.lessons
                                    for name in lesson.data.get("requires_ros_packages", [])}),
            "incomplete_checks": [], "detection_errors": [],
            "pkg_config": {name: {"version": version or "1.0.0", "available": True, "error": ""}
                           for lesson in self.lessons for name, version in lesson.data.get("requires_pkg_config", {}).items()},
            "sdk": {"headers_available": True, "error": ""}, "native_server": {"validated": True},
            "qt_moc": {"available": True, "path": "/fixture/qt5/moc", "version": "5.12.8"},
            "libraries": {"gazebo_gui": "/fixture/libgazebo_gui.so", "gazebo_ode": "/fixture/libgazebo_ode.so",
                          "gazebo_util": "/fixture/libgazebo_util.so"},
            "ros_package_versions": {"gazebo_plugins": "2.9.3"}, "python_modules": {"numpy": True},
        }
        self.docker_probe = {"backend": "docker", "execution_available": False, "daemon_checked": False,
                             "cli": {"installed": False, "path": None, "version": "", "error": ""}}
        self.requested_jobs = []
        self.windows = []
        self.patches = []

    def tearDown(self):
        for window in self.windows:
            window.timer.stop()
            window.autostart.setChecked(False)
            window.session = None
            if window.environment_dialog is not None:
                window.environment_dialog.reject()
            window._allow_close = True
            window.close()
            window.deleteLater()
        self.application.processEvents()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        for patch in reversed(self.patches):
            patch.stop()
        self.settings.sync()
        self.temp.cleanup()

    def make_window(self, *, autostart=False, fixture=None):
        self.settings.setValue("autostart", autostart)
        owner = self

        def capture_task(window, name, function, callback=None):
            if name == "检测环境":
                callback((copy.deepcopy(owner.detected), copy.deepcopy(owner.docker_probe)))
            else:
                # Preserve the user-visible launch decision without executing any
                # subprocess, background thread or simulator initialization.
                owner.requested_jobs.append(name)

        for patch in (mock.patch("gazeboteach.app.QtCore.QSettings", return_value=self.settings),
                      mock.patch.object(MainWindow, "run_task", capture_task)):
            patch.start()
            self.patches.append(patch)
        window = MainWindow(ROOT, fixture=fixture, storage=Storage(self.root / "data"))
        self.windows.append(window)
        window.timer.stop()
        window.show()
        self.application.processEvents()
        return window

    def acknowledge(self, window):
        window.environment_dialog.findChild(QtWidgets.QPushButton, "continue_learning").click()
        self.application.processEvents()

    def test_startup_shows_all_installed_projects_and_blocks_start_until_acknowledged(self):
        window = self.make_window()
        self.assertGreater(len(self.lessons), 0)
        self.assertEqual(len(window.lessons), len(self.lessons))
        self.assertTrue(window.environment_dialog.isVisible())
        self.assertEqual(window.environment_dialog.table.rowCount(), len(self.lessons))
        self.assertEqual(window.environment_report["course_count"], len(self.lessons))
        classic = [lesson for lesson in self.lessons if lesson.profile == "classic11-noetic"]
        self.assertEqual(window.environment_report["counts"]["ready"], len(classic))
        self.assertEqual([window.environment_dialog.table.item(row, 0).text()
                          for row in range(len(self.lessons))],
                         [lesson.id for lesson in self.lessons])
        self.assertTrue(window.select_lesson("01"))
        self.assertFalse(window.buttons["start"].isEnabled())
        self.assertTrue(window._environment_dialog_pending)
        self.acknowledge(window)
        self.assertFalse(window.environment_dialog.isVisible())
        self.assertFalse(window._environment_dialog_pending)
        self.assertTrue(window.buttons["start"].isEnabled())
        self.assertEqual(self.requested_jobs, [])

    def test_modern_ros2_mismatch_keeps_materials_readable_and_launch_disabled(self):
        self.detected.update(classic_version="", modern_simulators=[{"command": "gz", "versions": "8.0.0"}],
                             ros_installed=["humble"], profiles=[], course_profile=None)
        window = self.make_window()
        self.assertEqual(window.environment_report["counts"]["unsupported"], len(self.lessons))
        self.assertTrue(window.select_lesson("24"))
        self.acknowledge(window)
        self.assertFalse(window.buttons["start"].isEnabled())
        self.assertFalse(window.buttons["restart"].isEnabled())
        self.assertGreater(len(window.document.toPlainText()), 100)
        self.assertTrue(window.document.isReadOnly())
        self.assertIn("教材仍可阅读", window.session_status.text())
        self.assertEqual(self.requested_jobs, [])

    def test_autostart_waits_for_acknowledgment_then_starts_once(self):
        window = self.make_window(autostart=True)
        window.select_lesson("01")
        self.application.processEvents()
        self.assertEqual(self.requested_jobs, [])
        self.acknowledge(window)
        self.assertEqual(self.requested_jobs, ["准备实验"])

    def test_incompatible_autostart_cannot_bypass_disabled_launch_button(self):
        self.detected.update(classic_version="", modern_simulators=[{"command": "gz", "versions": "8.0.0"}],
                             ros_installed=["humble"], profiles=[], course_profile=None)
        window = self.make_window(autostart=True)
        window.select_lesson("24")
        self.acknowledge(window)
        self.assertEqual(self.requested_jobs, [], "Autostart bypassed the compatibility result")
        # The method is also used by queued signals, so enforce the decision at
        # this boundary as well as in the disabled button.
        window.start_lesson()
        window.restart_lesson()
        self.assertEqual(self.requested_jobs, [])

    def test_reopening_result_does_not_restart_a_running_lesson(self):
        window = self.make_window(autostart=True)
        window.select_lesson("01")
        window.session = SimpleNamespace(running=True, paused=False, lesson=window.current_lesson)
        self.acknowledge(window)
        window.environment_button.click()
        self.application.processEvents()
        self.assertTrue(window.environment_dialog.isVisible())
        self.acknowledge(window)
        self.assertEqual(self.requested_jobs, [])
        self.assertTrue(window.session.running)

    def test_delayed_recheck_cannot_reopen_or_acknowledge_a_stale_report(self):
        window = self.make_window(autostart=True)
        window.select_lesson("01")
        self.acknowledge(window)
        self.requested_jobs.clear()
        deferred = {}

        def hold_probe(name, function, callback=None):
            if name != "检测环境":
                self.requested_jobs.append(name)
                return
            deferred.update(function=function, callback=callback)
            window._jobs[9001] = (name, callback)
            window._refresh_buttons()

        window.run_task = hold_probe
        window.environment_button.click()
        self.application.processEvents()
        stale = window.environment_dialog
        self.assertTrue(stale.isVisible())
        stale.retry_button.click()
        self.application.processEvents()
        self.assertTrue(window._environment_detection_active)
        self.assertTrue(window._environment_dialog_pending)
        self.assertFalse(window.environment_button.isEnabled())
        self.assertFalse(stale.isVisible())
        window.show_environment_report()
        # A queued old-dialog callback cannot acknowledge the pending new result.
        stale.accept()
        self.application.processEvents()
        self.assertFalse(stale.isVisible())
        self.assertTrue(window._environment_dialog_pending)
        self.assertEqual(self.requested_jobs, [])

        window._task_completed(9001, (copy.deepcopy(self.detected), copy.deepcopy(self.docker_probe)))
        self.application.processEvents()
        self.assertIsNot(window.environment_dialog, stale)
        self.assertTrue(window.environment_dialog.isVisible())
        self.assertTrue(window._environment_dialog_pending)
        self.assertFalse(window.buttons["start"].isEnabled())
        self.assertEqual(self.requested_jobs, [])
        self.acknowledge(window)
        self.assertEqual(self.requested_jobs, ["准备实验"])

    def test_failed_detector_shows_incomplete_report_and_keeps_reading_available(self):
        window = self.make_window(autostart=True)
        window.select_lesson("24")
        self.acknowledge(window)
        self.requested_jobs.clear()
        deferred = {}

        def hold_probe(name, function, callback=None):
            if name != "检测环境":
                self.requested_jobs.append(name)
                return
            deferred.update(function=function, callback=callback)
            window._jobs[9002] = (name, callback)
            window._refresh_buttons()

        window.run_task = hold_probe
        window.detect_runtime_environment()
        backend = SimpleNamespace(probe=lambda: copy.deepcopy(self.docker_probe))
        with mock.patch("gazeboteach.app.detect_environment", side_effect=PermissionError("fixture unreadable probe")), \
                mock.patch("gazeboteach.app.get_backend", return_value=backend):
            result = deferred["function"]()
        window._task_completed(9002, result)
        self.application.processEvents()
        self.assertTrue(window.environment_dialog.isVisible())
        self.assertEqual(window.environment_report["counts"]["incomplete_detection"], len(self.lessons))
        self.assertIn("PermissionError", window.environment_report["detection"]["errors"][0])
        self.acknowledge(window)
        self.assertTrue(window.select_lesson("24"))
        self.assertGreater(len(window.document.toPlainText()), 100)
        self.assertFalse(window.buttons["start"].isEnabled())
        self.assertFalse(window.buttons["restart"].isEnabled())
        self.assertEqual(self.requested_jobs, [])

    def test_invalid_docker_configuration_is_not_persisted_and_valid_config_never_executes(self):
        window = self.make_window()
        dialog = window.environment_dialog
        save = dialog.findChild(QtWidgets.QPushButton, "save_docker_configuration")
        dialog.docker_configuration.setPlainText('{"image": "osrf/ros:noetic-desktop-full", "privileged": true}')
        save.click()
        self.assertFalse(self.settings.contains("docker_configuration"))
        self.assertIn("配置未保存", dialog.docker_status.text())
        valid = {"image": "osrf/ros:noetic-desktop-full", "host_workspace": str(self.root / "app"),
                 "container_workspace": "/workspace/GazeboTeach", "display": ":7"}
        dialog.docker_configuration.setPlainText(json.dumps(valid))
        save.click()
        stored = self.settings.value("docker_configuration")
        self.assertEqual(json.loads(stored)["backend"], "docker")
        self.assertIn("尚未连接 Docker", dialog.docker_status.text())
        self.assertFalse(next(item for item in dialog.backends if item["id"] == "docker")["execution_implemented"])
        dialog.docker_configuration.setPlainText("{invalid JSON")
        save.click()
        self.assertEqual(self.settings.value("docker_configuration"), stored)
        self.assertEqual(self.requested_jobs, [])
        self.assertFalse(window.docker_probe["execution_available"])

    def test_actual_prerequisite_click_keeps_running_lesson_and_personal_files(self):
        window = self.make_window()
        window.select_lesson("37")
        self.acknowledge(window)
        lesson = window.current_lesson
        workspace = window.storage.prepare(lesson)
        marker = workspace / "parameters.json"
        marker.write_text('{"student": "preserve this experiment"}', encoding="utf-8")
        expected = marker.read_bytes()
        session = SimpleNamespace(running=True, paused=False, lesson=lesson)
        window.session = session
        records_before = list(window.storage.records(150))
        cursor = window.document.document().find("第7课：摩擦与碰撞")
        self.assertFalse(cursor.isNull())
        cursor.setPosition(cursor.selectionStart() + 2)
        window.document.setTextCursor(cursor)
        window.document.ensureCursorVisible()
        self.application.processEvents()
        point = window.document.cursorRect(cursor).center()
        self.assertIn("07-friction-collision", window.document.anchorAt(point))
        QtTest.QTest.mouseClick(window.document.viewport(), QtCore.Qt.LeftButton, pos=point)
        self.application.processEvents()
        reader = window.prerequisite_reader
        self.assertIsNotNone(reader)
        self.assertTrue(reader.isVisible())
        self.assertEqual(reader.current_path.parent.name, "07-friction-collision")
        self.assertTrue(reader.browser.isReadOnly())
        self.assertIs(window.session, session)
        self.assertIs(window.current_lesson, lesson)
        self.assertEqual(marker.read_bytes(), expected)
        self.assertEqual(window.storage.records(150), records_before)
        self.assertEqual(self.requested_jobs, [])
        reader.back_button.click()
        self.assertEqual(reader.current_path.parent.name, "37-arm-push")
        reader.close()

    def test_internal_reader_rejects_outside_files_scripts_and_preserves_external_links(self):
        from gazeboteach.reader import DocumentRegistry, MarkdownReader
        window = self.make_window()
        lesson = next(item for item in self.lessons if item.id == "37")
        reader = MarkdownReader(window.document_registry, window)
        reader.display(lesson.file("lesson.md"))
        original = reader.current_path
        for link in ("javascript:alert(1)", "../../../../outside.md", "../07-friction-collision/world.sdf",
                     "httpsx://example.com/lesson.md"):
            reader.open_link(QtCore.QUrl(link))
            self.assertEqual(reader.current_path, original)
            self.assertTrue(reader.location.text().startswith("链接不可打开"))
        with mock.patch("gazeboteach.reader.QtGui.QDesktopServices.openUrl") as opened:
            reader.open_link(QtCore.QUrl("https://classic.gazebosim.org/tutorials"))
            opened.assert_called_once()
        reader.close()

    def test_reset_tooltip_explains_clock_and_private_state_boundary(self):
        window = self.make_window()
        text = window.buttons["reset"].toolTip()
        for phrase in ("MODEL reset", "保留仿真时钟", "私有状态", "重载项目"):
            self.assertIn(phrase, text)

    def test_modern_profile_selects_matching_materials_and_disables_unsupported_reset(self):
        from gazeboteach.catalog import Lesson
        folder = ROOT / "tests/fixtures/modern_diagnostic"
        lesson = Lesson(folder, json.loads((folder / "lesson.json").read_text(encoding="utf-8")))
        self.detected.update(classic_version="", profiles=["harmonic-jazzy"], course_profile="harmonic-jazzy",
                             ros_installed=["jazzy"], modern_simulators=[{"command": "gz", "versions": "8.10.0"}])
        self.detected["modern"] = {"sim_major": 8, "sim_version": "8.10.0", "gz": "/fixture/gz",
                                   "ros_available": True, "packages": [], "error": "", "sim_probe_error": "",
                                   "tools": dict(self.detected["tools"], ros2="/fixture/ros2")}
        window = self.make_window(fixture=lesson)
        self.assertEqual(window.profile_selector.currentData(), "harmonic-jazzy")
        self.assertEqual(window.environment_report["counts"]["ready"], 1)
        self.assertTrue(window.select_lesson("98", profile="harmonic-jazzy"))
        self.acknowledge(window)
        self.assertTrue(window.buttons["start"].isEnabled())
        window.session = SimpleNamespace(running=True, paused=False, lesson=lesson, token="fixture",
                                         state="运行中", workspace=self.root, native_control_diagnosis={})
        window._refresh_buttons()
        self.assertFalse(window.buttons["reset"].isEnabled())
        self.assertIn("不支持", window.buttons["reset"].toolTip())
        self.assertTrue(window.buttons["step"].isEnabled())
        window.session = None


if __name__ == "__main__":
    unittest.main()
