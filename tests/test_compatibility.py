"""Availability reports distinguish actual mismatches from unfinished detection."""

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import compatibility
from gazeboteach.compatibility import build_compatibility_report, lesson_compatibility
from tests.test_core import environment, make_lesson


class CompatibilityReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.independent = make_lesson(root / "independent")
        self.ros = make_lesson(root / "ros", id="02", requires_ros=True,
                               requires_plugins=["libgazebo_ros_api_plugin.so"],
                               requires_ros_packages=["gazebo_ros"])
        self.detected = environment(plugins=["libgazebo_ros_api_plugin.so"],
                                    ros_packages=["gazebo_ros"], modern_simulators=[],
                                    incomplete_checks=[], detection_errors=[])

    def tearDown(self):
        self.temp.cleanup()

    def report(self, lesson=None, detected=None, **options):
        return lesson_compatibility(lesson or self.independent,
                                    self.detected if detected is None else detected, **options)

    def test_installed_noetic_works_without_shell_activation(self):
        self.assertEqual(self.detected["ros_active"], "")
        row = self.report(self.ros)
        self.assertEqual(row["status"], "ready")
        self.assertTrue(row["compatible"])
        self.assertIsNone(compatibility(self.ros, self.detected, terminals=True))

    def test_modern_ros2_only_is_unsupported_and_materials_stay_readable(self):
        detected = dict(self.detected, classic_version="", ros_installed=["humble"],
                        modern_simulators=[{"command": "gz", "versions": "8.0.0"}])
        for lesson in (self.independent, self.ros):
            row = self.report(lesson, detected)
            self.assertEqual(row["status"], "unsupported")
            self.assertFalse(row["compatible"])
            self.assertTrue(row["reading_available"])

    def test_classic_without_noetic_keeps_independent_courses_available(self):
        detected = dict(self.detected, ros_installed=[])
        self.assertEqual(self.report(detected=detected)["status"], "ready")
        self.assertEqual(self.report(self.ros, detected)["status"], "unsupported")

    def test_a_failed_version_probe_is_not_a_confirmed_mismatch(self):
        detected = dict(self.detected, classic_version="", incomplete_checks=["classic_version"])
        row = self.report(detected=detected)
        self.assertEqual(row["status"], "incomplete_detection")
        self.assertIn("classic_version", row["reasons"][0]["fields"])

    def test_known_wrong_version_is_actionable_despite_unrelated_probe_failure(self):
        detected = dict(self.detected, classic_version="9.19.0", incomplete_checks=["ros_packages"])
        self.assertEqual(self.report(self.ros, detected)["status"], "unsupported")

    def test_missing_dependency_is_not_failed_detection(self):
        detected = dict(self.detected, plugins=[])
        row = self.report(self.ros, detected)
        self.assertEqual(row["status"], "missing_dependencies")
        self.assertIn("libgazebo_ros_api_plugin.so", row["reasons"][0]["message"])

    def test_failed_package_probe_only_blocks_courses_using_package_facts(self):
        detected = dict(self.detected, ros_packages=[], incomplete_checks=["ros_packages"])
        self.assertEqual(self.report(detected=detected)["status"], "ready")
        self.assertEqual(self.report(self.ros, detected)["status"], "incomplete_detection")

    def test_partial_tool_snapshot_never_uses_operator_path_to_claim_ready(self):
        detected = copy.deepcopy(self.detected)
        del detected["tools"]["gzserver"]
        with mock.patch("gazeboteach.catalog.shutil.which", return_value="/operator/gzserver"):
            row = self.report(detected=detected)
        self.assertEqual(row["status"], "incomplete_detection")
        self.assertIn("tools:gzserver", row["reasons"][0]["fields"])

    def test_explicit_missing_tool_is_a_completed_negative_result(self):
        detected = copy.deepcopy(self.detected)
        detected["tools"]["gzserver"] = None
        row = self.report(detected=detected)
        self.assertEqual(row["status"], "missing_dependencies")
        self.assertIn("gzserver", row["reasons"][0]["message"])

    def test_headless_requirement_does_not_depend_on_client_or_terminal_detection(self):
        detected = copy.deepcopy(self.detected)
        del detected["tools"]["gzclient"]
        del detected["tools"]["gnome-terminal"]
        del detected["display"]
        self.assertEqual(self.report(detected=detected, gui=False, terminals=False)["status"], "ready")
        self.assertEqual(self.report(detected=detected)["status"], "incomplete_detection")

    def test_terminal_must_be_detected_but_either_choice_suffices(self):
        detected = copy.deepcopy(self.detected)
        detected["tools"].pop("gnome-terminal")
        self.assertEqual(self.report(detected=detected)["status"], "incomplete_detection")
        detected["tools"]["xterm"] = "/usr/bin/xterm"
        self.assertEqual(self.report(detected=detected)["status"], "ready")

    def test_report_is_serializable_with_complete_counts_and_probe_error_detail(self):
        detected = dict(self.detected, incomplete_checks=["ros_packages"],
                        detection_errors=["rospack probe timed out"])
        report = build_compatibility_report([self.independent, self.ros], detected, ["bad catalog entry"])
        self.assertEqual(report["course_count"], 2)
        self.assertEqual(report["counts"]["ready"], 1)
        self.assertEqual(report["counts"]["incomplete_detection"], 1)
        self.assertFalse(report["detection"]["complete"])
        self.assertEqual(report["catalog_errors"], ["bad catalog entry"])
        self.assertIn("rospack probe timed out", report["detection"]["errors"])
        json.dumps(report)


if __name__ == "__main__":
    unittest.main()
