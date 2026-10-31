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
    from PyQt5 import QtCore, QtWidgets
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
            "course_profile": "classic11-noetic", "ros_installed": ["noetic"], "ros_active": "",
            "display": ":7", "tools": {name: "/fixture/bin/" + name for name in tools},
            "plugins": sorted({name for lesson in self.lessons for name in lesson.data.get("requires_plugins", [])}),
            "ros_packages": sorted({name for lesson in self.lessons
                                    for name in lesson.data.get("requires_ros_packages", [])}),
            "incomplete_checks": [], "detection_errors": [],
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

    def make_window(self, *, autostart=False):
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
        window = MainWindow(ROOT, storage=Storage(self.root / "data"))
        self.windows.append(window)
        window.timer.stop()
        window.show()
        self.application.processEvents()
        return window

    def acknowledge(self, window):
        window.environment_dialog.findChild(QtWidgets.QPushButton, "continue_learning").click()
        self.application.processEvents()

    def test_startup_shows_all_26_projects_and_blocks_start_until_acknowledged(self):
        window = self.make_window()
        self.assertEqual(len(window.lessons), 26)
        self.assertTrue(window.environment_dialog.isVisible())
        self.assertEqual(window.environment_dialog.table.rowCount(), 26)
        self.assertEqual(window.environment_report["counts"]["ready"], 26)
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
        self.assertEqual(window.environment_report["counts"]["unsupported"], 26)
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


if __name__ == "__main__":
    unittest.main()
