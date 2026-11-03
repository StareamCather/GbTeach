"""Dependency failures must be explained before any lesson prepares or starts."""

import ast
import copy
import json
import os
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from gazeboteach.catalog import compatibility, load_catalog
from gazeboteach.compatibility import build_compatibility_report, lesson_compatibility
from gazeboteach.environment import SDK_PACKAGES, detect_development_capabilities, noetic_base_environment, lesson_environment
from tests.test_core import environment, make_lesson

ROOT = Path(__file__).resolve().parents[1]


class DevelopmentRequirementsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.lesson = make_lesson(self.root / "sdk", requires_pkg_config={"gazebo": "11.15.1"},
                                  requires_native_server=True)
        self.detected = environment(pkg_config={"gazebo": {"available": True, "version": "11.15.1"}},
                                    sdk={"headers_available": True}, native_server={"validated": True})

    def tearDown(self):
        self.temp.cleanup()

    def test_pkg_config_executable_is_not_the_gazebo_development_package(self):
        self.detected["tools"]["pkg-config"] = "/fixture/bin/pkg-config"
        self.detected["pkg_config"]["gazebo"] = {"available": False, "version": "", "error": "missing gazebo.pc"}
        row = lesson_compatibility(self.lesson, self.detected)
        self.assertEqual(row["status"], "missing_dependencies")
        self.assertIn("gazebo.pc", row["reasons"][0]["message"])
        basic = make_lesson(self.root / "basic")
        self.assertEqual(lesson_compatibility(basic, self.detected)["status"], "ready")

    def test_wrong_sdk_exact_version_is_unsupported(self):
        self.detected["pkg_config"]["gazebo"]["version"] = "11.10.2"
        row = lesson_compatibility(self.lesson, self.detected)
        self.assertEqual(row["status"], "unsupported")
        self.assertIn("11.15.1", row["reasons"][0]["message"])
        self.assertTrue(row["reading_available"])

    def test_pc_without_actual_headers_is_not_ready(self):
        self.detected["sdk"] = {"headers_available": False, "error": "gazebo/gazebo.hh missing"}
        self.assertIn("gazebo.hh", compatibility(self.lesson, self.detected))

    def test_other_gzserver_cannot_pass_the_exact_native_sdk_guard(self):
        self.detected["native_server"] = {"validated": False, "reason": "another runtime"}
        self.assertIn("gzserver 11.15.1", compatibility(self.lesson, self.detected))

    def test_partial_or_malformed_capability_snapshot_is_incomplete(self):
        for value in ({}, {"gazebo": "11.15.1"}):
            self.detected["pkg_config"] = value
            row = lesson_compatibility(self.lesson, self.detected)
            self.assertEqual(row["status"], "incomplete_detection")

    def test_installed_noetic_is_usable_despite_active_ros2_shell(self):
        lesson = make_lesson(self.root / "ros", requires_ros=True,
                             requires_ros_package_versions={"gazebo_plugins": "2.9.3"})
        detected = environment(ros_active="humble", ros_active_version="2", ros_installed=["humble", "noetic"],
                               ros_package_versions={"gazebo_plugins": "2.9.3"})
        before = copy.deepcopy(detected)
        self.assertEqual(lesson_compatibility(lesson, detected)["status"], "ready")
        self.assertEqual(detected, before)
        detected["ros_package_versions"]["gazebo_plugins"] = "2.8.7"
        self.assertEqual(lesson_compatibility(lesson, detected)["status"], "unsupported")

    def test_gui_courses_reject_headless_smoke_before_storage_or_compilation(self):
        from gazeboteach import __main__ as cli
        lessons, errors = load_catalog(ROOT)
        self.assertFalse(errors)
        for course in ("29", "30"):
            lesson = next(item for item in lessons if item.id == course)
            self.assertIn("--gui", compatibility(lesson, self.detected, gui=False))
            arguments = SimpleNamespace(smoke=course, profile=None, gui=False, terminals=False)
            with mock.patch.object(cli, "new_storage") as storage, mock.patch.object(cli, "get_backend") as session:
                with self.assertRaisesRegex(RuntimeError, "--gui"):
                    cli.smoke(arguments, self.detected)
                storage.assert_not_called()
                session.assert_not_called()

    def test_qt_widgets_and_moc_are_independent_requirements(self):
        lesson = make_lesson(self.root / "qt", requires_pkg_config={"Qt5Widgets": ""}, requires_qt_moc=True)
        detected = environment(pkg_config={"Qt5Widgets": {"version": "", "available": False}},
                               qt_moc={"available": True})
        self.assertIn("Qt5Widgets.pc", compatibility(lesson, detected))
        detected["pkg_config"]["Qt5Widgets"] = {"available": True, "version": "5.12.8"}
        detected["qt_moc"] = {"available": False, "error": "version mismatch"}
        self.assertIn("moc", compatibility(lesson, detected))

    def test_every_actual_prepare_pkg_config_dependency_is_declared(self):
        lessons, errors = load_catalog(ROOT)
        self.assertFalse(errors)
        baseline = {"%02d" % value for value in range(1, 38)} | {"091", "092"}
        classic = {lesson.id for lesson in lessons if lesson.profile == "classic11-noetic"}
        self.assertTrue(baseline <= classic, "Preserve every original V2 project while adding profiles/courses")
        for lesson in lessons:
            path = lesson.folder / "prepare.py"
            if not path.exists():
                continue
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8-sig"))):
                if not isinstance(node, ast.List):
                    continue
                values = [item.value if isinstance(item, ast.Constant) else None for item in node.elts]
                if values and values[0] == "pkg-config" and any(flag in values for flag in ("--cflags", "--libs", "--modversion")):
                    packages = [value for value in values[1:] if isinstance(value, str) and not value.startswith("-")]
                    self.assertTrue(set(packages) <= set(lesson.data.get("requires_pkg_config", {})), lesson.key)


class DevelopmentProbeTests(unittest.TestCase):
    def test_active_ros2_prefixes_are_removed_only_in_child_before_noetic_setup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ros2, overlay, keep = root / "humble", root / "ros2-overlay", root / "personal-tools"
            inherited = {"ROS_DISTRO": "humble", "ROS_VERSION": "2", "ROS_DOMAIN_ID": "45",
                         "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "AMENT_PREFIX_PATH": str(ros2),
                         "COLCON_PREFIX_PATH": str(overlay), "KEEP_USER_SETTING": "yes"}
            for name in ("PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH"):
                inherited[name] = os.pathsep.join((str(ros2 / "lib"), str(overlay / "bin"), str(keep)))
            inherited["ROS_PACKAGE_PATH"] = str(keep / "ros1-packages")
            gazebo_paths = ("GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH", "GAZEBO_RESOURCE_PATH")
            learner, noetic = keep / "learner-assets", keep / "noetic-assets"
            for name in gazebo_paths:
                inherited[name] = os.pathsep.join((str(ros2 / "lib"), str(overlay / "models"), str(learner), str(noetic)))
            expected = copy.deepcopy(inherited)
            cleaned = noetic_base_environment(inherited)
            self.assertEqual(inherited, expected)
            for name in ("PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH"):
                self.assertEqual(cleaned[name], str(keep), name)
            self.assertNotIn("ROS_DOMAIN_ID", cleaned)
            self.assertNotIn("RMW_IMPLEMENTATION", cleaned)
            self.assertEqual(cleaned["KEEP_USER_SETTING"], "yes")
            self.assertEqual(cleaned["ROS_PACKAGE_PATH"], str(keep / "ros1-packages"))
            for name in gazebo_paths:
                self.assertEqual(cleaned[name], os.pathsep.join((str(learner), str(noetic))), name)
            lesson = make_lesson(root / "lesson", requires_ros=True)
            def source_noetic(setup, child):
                self.assertEqual(str(setup).replace("\\", "/"), "/opt/ros/noetic/setup.bash")
                self.assertEqual(child["PATH"], str(keep))
                self.assertNotIn("RMW_IMPLEMENTATION", child)
                return dict(child, ROS_DISTRO="noetic", ROS_VERSION="1")
            def installed(path):
                return str(path).replace("\\", "/") == "/opt/ros/noetic/setup.bash"
            with mock.patch.dict(os.environ, inherited, clear=True), \
                    mock.patch("gazeboteach.environment.Path.is_file", installed), \
                    mock.patch("gazeboteach.environment._load_setup", side_effect=source_noetic):
                activated = lesson_environment(root, root / "workspace", lesson, 18001, 18002)
                self.assertEqual(os.environ, inherited)
            self.assertEqual(activated["ROS_DISTRO"], "noetic")
            self.assertEqual(activated["ROS_VERSION"], "1")

    def test_qt_uses_host_bins_instead_of_broken_qtchooser_and_reports_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            moc = Path(directory) / "moc"
            moc.write_text("fixture", encoding="utf-8")
            calls = []
            def output(command, **options):
                calls.append(command)
                if "--variable=host_bins" in command:
                    return directory
                if command == [str(moc), "-v"]:
                    return "moc 5.12.8"
                if "--modversion" in command:
                    return "5.12.8" if command[-1].startswith("Qt5") else "11.15.1"
                if "--cflags" in command:
                    return "-I/fixture/include -L/fixture/lib -lfixture"
                return ""
            tools = {"pkg-config": "/fixture/pkg-config", "g++": None, "gzserver": None}
            before = dict(os.environ)
            with mock.patch("gazeboteach.environment._output", side_effect=output):
                report = detect_development_capabilities(tools, {})
            self.assertTrue(report["qt_moc"]["available"])
            self.assertEqual(report["qt_moc"]["path"], str(moc))
            self.assertNotIn(["/usr/bin/moc", "-v"], calls)
            self.assertEqual(os.environ, before)
            self.assertEqual(sum("--modversion" in call for call in calls), len(SDK_PACKAGES))
            # Compatibility reports share this snapshot; never probe per row.
            lessons, errors = load_catalog(ROOT)
            with mock.patch("gazeboteach.environment.subprocess.run") as execute:
                build_compatibility_report(lessons, environment(**report), errors)
                execute.assert_not_called()
            def wrong_moc(command, **options):
                return "moc 6.2.4" if command == [str(moc), "-v"] else output(command, **options)
            with mock.patch("gazeboteach.environment._output", side_effect=wrong_moc):
                mismatch = detect_development_capabilities(tools, {})
            self.assertFalse(mismatch["qt_moc"]["available"])


if __name__ == "__main__":
    unittest.main()
