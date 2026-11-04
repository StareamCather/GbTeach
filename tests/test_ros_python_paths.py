"""Generated ROS messages stay private and survive distribution setup ordering."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import Lesson
from gazeboteach.environment import add_course_python_paths, lesson_environment
from gazeboteach.modern import modern_lesson_environment
from tests.test_core import make_lesson


class GeneratedPythonPathTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.relative = "ros_ws/devel/lib/python3/dist-packages"
        self.lesson = make_lesson(self.root / "source", requires_ros=True,
                                  ros_python_paths=[self.relative])

    def test_manifest_rejects_malformed_non_ros_or_escaping_paths(self):
        for value in (None, {}, "devel", [None], [""], ["  "], ["."],
                      ["../outside"], [str(self.root / "outside")]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                Lesson(self.lesson.folder, dict(self.lesson.data, ros_python_paths=value))
        with self.assertRaisesRegex(ValueError, "requires_ros"):
            Lesson(self.lesson.folder, dict(self.lesson.data, requires_ros=False))

    def test_path_list_injection_rejected_in_manifest_and_personal_directory(self):
        for field in ("ros_python_paths", "ros_package_paths"):
            for relative in ("generated:/tmp/external", "generated;external"):
                with self.subTest(field=field, relative=relative), self.assertRaisesRegex(ValueError, "separator"):
                    Lesson(self.lesson.folder, dict(self.lesson.data, **{field:[relative]}))
        with self.assertRaisesRegex(ValueError, "path-list separator"):
            add_course_python_paths({}, self.root / ("personal" + os.pathsep + "external"), self.lesson)

    def test_generated_missing_directory_is_predeclared_and_private_per_workspace(self):
        with mock.patch.dict(os.environ, {"PYTHONPATH": "/fixture/parent"}):
            before = dict(os.environ)
            for profile in ("classic11-noetic", "harmonic-jazzy", "fortress-humble"):
                lesson = Lesson(self.lesson.folder, dict(self.lesson.data, profile=profile))
                for personal in ("personal-a", "personal-b"):
                    workspace = self.root / personal
                    generated = (workspace / self.relative).resolve()
                    self.assertFalse(generated.exists())
                    def setup(path, env):
                        return dict(env, PYTHONPATH="/fixture/selected-ros", ROS_DISTRO=path.parent.name,
                                    ROS_VERSION="1" if profile == "classic11-noetic" else "2")
                    with mock.patch("gazeboteach.environment.Path.is_file", return_value=True), \
                            mock.patch("gazeboteach.environment._load_setup", side_effect=setup):
                        if profile == "classic11-noetic":
                            result = lesson_environment(self.root, workspace, lesson, 19001, 19002, "fixture")
                        else:
                            result = modern_lesson_environment(self.root, workspace, lesson, "fixture")
                    self.assertEqual(result["PYTHONPATH"].split(os.pathsep),
                                     [str(generated), "/fixture/selected-ros"])
            self.assertEqual(dict(os.environ), before)

    def test_undeclared_field_keeps_environment_unchanged(self):
        lesson = Lesson(self.lesson.folder, dict(self.lesson.data, ros_python_paths=[]))
        environment = {"PYTHONPATH": "/fixture/ros" + os.pathsep}
        add_course_python_paths(environment, self.root / "personal", lesson)
        self.assertEqual(environment, {"PYTHONPATH": "/fixture/ros" + os.pathsep})

    def test_personal_symlink_escape_rejected(self):
        workspace, outside = self.root / "personal", self.root / "outside"
        workspace.mkdir()
        outside.mkdir()
        try:
            (workspace / "ros_ws").symlink_to(outside, target_is_directory=True)
        except (OSError, NotImplementedError) as exc:
            self.skipTest("Symlink creation unavailable: " + str(exc))
        with self.assertRaisesRegex(ValueError, "personal workspace"):
            add_course_python_paths({}, workspace, self.lesson)


if __name__ == "__main__":
    unittest.main()
