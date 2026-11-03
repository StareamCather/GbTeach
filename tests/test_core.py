"""Behavioral unit tests. Execute with python3 -m unittest discover -s tests -v."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

from gazeboteach.catalog import Lesson, compatibility, load_catalog
from gazeboteach.environment import detect_environment, _output
from gazeboteach.storage import Storage


def make_lesson(folder, **changes):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "world.sdf").write_text("<sdf version='1.6'><world name='unit'/></sdf>", encoding="utf-8")
    (folder / "lesson.md").write_text("# 初始教材\n", encoding="utf-8")
    data = {"id": "01", "title": "Unit fixture", "profile": "classic11-noetic",
            "requires_ros": False, "world": "world.sdf", "order": 1,
            "objectives": [], "commands": []}
    data.update(changes)
    (folder / "lesson.json").write_text(json.dumps(data), encoding="utf-8")
    return Lesson(folder, data)


def environment(**changes):
    result = {"classic_version": "11.15.1", "profiles": ["classic11-noetic"],
              "ros_installed": ["noetic"], "ros_active": "", "display": ":1",
              "tools": {"gazebo": "/usr/bin/gazebo", "gzserver": "/usr/bin/gzserver",
                        "gzclient": "/usr/bin/gzclient", "gz": "/usr/bin/gz", "bash": "/bin/bash",
                        "roscore": "/opt/ros/noetic/bin/roscore", "roslaunch": "/opt/ros/noetic/bin/roslaunch",
                        "rosrun": "/opt/ros/noetic/bin/rosrun", "rospack": "/opt/ros/noetic/bin/rospack",
                        "gnome-terminal": "/usr/bin/gnome-terminal", "setsid": "/usr/bin/setsid"}}
    result.update(changes)
    return result


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.lesson = make_lesson(self.root / "one")

    def tearDown(self):
        self.temp.cleanup()

    def test_classic11_is_compatible_even_if_ros_not_activated(self):
        lesson = make_lesson(self.root / "ros", requires_ros=True)
        self.assertIsNone(compatibility(lesson, environment(ros_active=""), gui=False))

    def test_no_ros_only_blocks_ros_lessons(self):
        report = environment(ros_installed=[])
        self.assertIsNone(compatibility(self.lesson, report, gui=False))
        lesson = make_lesson(self.root / "ros", requires_ros=True)
        self.assertIsNotNone(compatibility(lesson, report, gui=False))

    def test_modern_installation_does_not_run_classic_course(self):
        report = environment(classic_version="", profiles=["harmonic-jazzy"],
                             modern_simulators=[{"command": "gz", "versions": "8.0.0"}])
        self.assertIsNotNone(compatibility(self.lesson, report, gui=False))

    def test_wrong_classic_major_is_not_accepted(self):
        self.assertIsNotNone(compatibility(self.lesson, environment(classic_version="9.19.0"), gui=False))

    def test_missing_server_is_reported(self):
        report = environment()
        report["tools"]["gzserver"] = None
        self.assertIsNotNone(compatibility(self.lesson, report, gui=False))

    def test_headless_check_does_not_require_client(self):
        report = environment(display="")
        report["tools"]["gzclient"] = None
        self.assertIsNone(compatibility(self.lesson, report, gui=False))
        self.assertIsNotNone(compatibility(self.lesson, report, gui=True))

    def test_course_dependencies_are_individually_checked(self):
        lesson = make_lesson(self.root / "dependencies", requires_ros=True,
                             requires_plugins=["libfixture.so"],
                             requires_ros_packages=["fixture_package"], requires_tools=["cmake"])
        report = environment(plugins=["libfixture.so"], ros_packages=["fixture_package"])
        report["tools"]["cmake"] = "/usr/bin/cmake"
        self.assertIsNone(compatibility(lesson, report, gui=False))
        for key in ("plugins", "ros_packages"):
            broken = dict(report, **{key: []})
            with self.subTest(missing=key):
                self.assertIsNotNone(compatibility(lesson, broken, gui=False))
        report["tools"]["cmake"] = None
        self.assertIsNotNone(compatibility(lesson, report, gui=False))

    def test_rendering_sensor_still_needs_display_when_gui_is_disabled(self):
        lesson = make_lesson(self.root / "rendering", needs_display=True)
        self.assertIsNotNone(compatibility(lesson, environment(display=""), gui=False))

    def test_manifest_cannot_escape_course_directory(self):
        for relative in ("../outside.sdf", str(self.root / "outside.sdf")):
            with self.subTest(relative=relative):
                with self.assertRaises((ValueError, OSError)):
                    make_lesson(self.root / "bad", world=relative)

    def test_missing_world_is_rejected(self):
        data = dict(self.lesson.data, world="not-present.sdf")
        with self.assertRaises((ValueError, OSError)):
            Lesson(self.lesson.folder, data)

    def test_boolean_string_is_rejected(self):
        with self.assertRaises((ValueError, TypeError)):
            make_lesson(self.root / "badboolean", requires_ros="false")

    def test_bad_manifest_does_not_hide_good_courses(self):
        root = self.root / "repository"
        make_lesson(root / "courses" / "classic11_noetic" / "01-good")
        folder = root / "courses" / "classic11_noetic" / "02-broken"
        folder.mkdir()
        (folder / "lesson.json").write_text("{broken", encoding="utf-8")
        lessons, errors = load_catalog(root)
        self.assertEqual([x.id for x in lessons], ["01"])
        self.assertEqual(len(errors), 1)

    def test_gz_or_ign_tools_alone_do_not_imply_simulator(self):
        def output(arguments, timeout=4, accepted_codes=(0,), env=None):
            return "The command provides tools. Available commands: topic, log, sdf."
        with mock.patch("gazeboteach.environment.Path.glob", return_value=[]), \
                mock.patch("gazeboteach.environment.shutil.which", side_effect=lambda name, **_: "/tools/" + name if name in ("gz", "ign") else None), \
                mock.patch("gazeboteach.environment._output", side_effect=output):
            report = detect_environment()
        self.assertFalse(report["classic_version"])
        self.assertEqual(report["modern_simulators"], [])
        self.assertIsNone(report["course_profile"])


class EnvironmentProbeTests(unittest.TestCase):
    def _probe_legacy_version(self, returncode):
        version = "Gazebo multi-robot simulator, version 11.15.1\nCopyright Open Robotics\n"
        def which(name, **_):
            return "/fixture/" + name if name in ("gazebo", "gzserver", "gzclient") else None
        def execute(arguments, **options):
            return subprocess.CompletedProcess(arguments, returncode, stdout=version)
        with mock.patch("gazeboteach.environment.Path.glob", return_value=[]), \
                mock.patch("gazeboteach.environment.shutil.which", side_effect=which), \
                mock.patch("gazeboteach.environment.subprocess.run", side_effect=execute):
            return detect_environment()

    def test_gazebo_11_version_accepts_known_exit_255(self):
        report = self._probe_legacy_version(255)
        self.assertEqual(report["classic_version"], "11.15.1")
        self.assertIn("classic11-noetic", report["profiles"])

    def test_other_failed_version_probe_is_not_accepted(self):
        report = self._probe_legacy_version(7)
        self.assertFalse(report["classic_version"])

    def test_ordinary_failed_command_stdout_is_not_a_valid_probe(self):
        result = subprocess.CompletedProcess(["broken-probe"], 1, stdout="partial misleading result")
        with mock.patch("gazeboteach.environment.subprocess.run", return_value=result):
            self.assertEqual(_output(["broken-probe"]), "")

    def test_installed_ros_tools_are_found_without_outer_shell_activation(self):
        def which(name):
            return "/fixture/" + name if name in ("gazebo", "gzserver", "gzclient", "gz", "bash") else None
        def glob(path, pattern):
            return [Path("/opt/ros/noetic")] if str(path).replace("\\", "/") == "/opt/ros" else []
        def is_file(path):
            value = str(path).replace("\\", "/")
            return value == "/opt/ros/noetic/setup.bash" or value.startswith("/opt/ros/noetic/bin/")
        def output(arguments, **options):
            if arguments[0] == "/fixture/gazebo":
                return "Gazebo multi-robot simulator, version 11.15.1"
            if any("rospack list-names" in str(value) for value in arguments):
                return "gazebo_ros\ngazebo_plugins"
            return ""
        with mock.patch.dict(os.environ, {"ROS_DISTRO": ""}), \
                mock.patch("gazeboteach.environment.shutil.which", side_effect=which), \
                mock.patch("gazeboteach.environment.Path.glob", glob), \
                mock.patch("gazeboteach.environment.Path.is_file", is_file), \
                mock.patch("gazeboteach.environment.os.access", return_value=True), \
                mock.patch("gazeboteach.environment._output", side_effect=output):
            report = detect_environment()
        self.assertEqual(report["ros_installed"], ["noetic"])
        self.assertEqual(report["ros_active"], "")
        for tool in ("rosrun", "rospack", "roscore", "roslaunch"):
            self.assertTrue(report["tools"].get(tool), tool)
        with tempfile.TemporaryDirectory() as directory:
            lesson = make_lesson(Path(directory) / "ros-tools", requires_ros=True,
                                 requires_tools=["rosrun", "rospack"], requires_ros_packages=["gazebo_ros"])
            self.assertIsNone(compatibility(lesson, report, gui=False))


class StorageTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.lesson = make_lesson(self.root / "source")
        self.storage = Storage(self.root / "data")

    def tearDown(self):
        self.temp.cleanup()

    def test_reenter_preserves_student_changes(self):
        workspace = self.storage.prepare(self.lesson)
        student_text = "<student-modified-world/>"
        (workspace / "world.sdf").write_text(student_text, encoding="utf-8")
        again = self.storage.prepare(self.lesson)
        self.assertEqual(again, workspace)
        self.assertEqual((again / "world.sdf").read_text(encoding="utf-8"), student_text)

    def test_updated_template_refreshes_unmodified_file(self):
        workspace = self.storage.prepare(self.lesson)
        replacement = "# 新版教材\n已修复说明。\n"
        (self.lesson.folder / "lesson.md").write_text(replacement, encoding="utf-8")
        self.assertTrue(self.storage.template_status(self.lesson)["updated"])
        self.storage.prepare(self.lesson)
        self.assertEqual((workspace / "lesson.md").read_text(encoding="utf-8"), replacement)

    def test_template_update_preserves_conflicting_student_file(self):
        workspace = self.storage.prepare(self.lesson)
        (workspace / "world.sdf").write_text("student version", encoding="utf-8")
        (self.lesson.folder / "world.sdf").write_text("teacher version", encoding="utf-8")
        self.storage.prepare(self.lesson)
        self.assertEqual((workspace / "world.sdf").read_text(encoding="utf-8"), "student version")
        self.assertIn("world.sdf", self.storage.template_status(self.lesson)["conflicts"])

    def test_restore_keeps_backup_and_loads_current_template(self):
        workspace = self.storage.prepare(self.lesson)
        (workspace / "world.sdf").write_text("my experiment", encoding="utf-8")
        restored, backup = self.storage.restore(self.lesson)
        self.assertIsNotNone(backup)
        self.assertEqual((backup / "world.sdf").read_text(encoding="utf-8"), "my experiment")
        self.assertEqual((restored / "world.sdf").read_bytes(), (self.lesson.folder / "world.sdf").read_bytes())

    def test_same_course_number_in_two_profiles_has_separate_progress(self):
        self.storage.record("01", "read", profile="classic11-noetic")
        self.storage.record("01", "verified", profile="harmonic-jazzy")
        progress = self.storage.progress()
        self.assertEqual(progress["classic11-noetic:01"], {"read"})
        self.assertEqual(progress["harmonic-jazzy:01"], {"verified"})

    def test_missing_verifier_is_not_reported_as_passed(self):
        from gazeboteach.runtime import LessonSession
        session = LessonSession(self.root, self.lesson, storage=self.storage)
        # Only bypass the running precondition; the actual verifier selection and
        # persistent progress logic must still execute.
        with mock.patch.object(LessonSession, "running", new_callable=mock.PropertyMock, return_value=True):
            result = session.verify()
        self.assertIsNone(result["passed"])
        self.assertNotIn("verified", self.storage.progress().get("classic11-noetic:01", set()))


def process_live(pid):
    try:
        text = Path("/proc/{}/stat".format(pid)).read_text()
        return text.rsplit(") ", 1)[1].split()[0] != "Z"
    except FileNotFoundError:
        return False


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux /proc and POSIX process groups required")
class RuntimeOwnershipTests(unittest.TestCase):
    def test_exited_parent_child_is_cleaned_without_killing_unrelated_process(self):
        from gazeboteach.runtime import LessonSession
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lesson = make_lesson(root / "source")
            session = LessonSession(root, lesson, storage=Storage(root / "data"))
            unrelated = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(60)"], start_new_session=True)
            marker = root / "child.pid"
            code = ("import pathlib,subprocess,sys; "
                    "p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)']); "
                    "pathlib.Path(sys.argv[1]).write_text(str(p.pid))")
            child_pid = None
            try:
                parent = session._spawn("ownership-fixture", [sys.executable, "-c", code, str(marker)])
                parent.wait(timeout=5)
                self.assertTrue(marker.exists())
                child_pid = int(marker.read_text())
                self.assertTrue(process_live(child_pid))
                session.stop()
                deadline = time.monotonic() + 5
                while process_live(child_pid) and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertFalse(process_live(child_pid), "Same-session orphan process survived cleanup")
                self.assertIsNone(unrelated.poll(), "Unrelated process was incorrectly terminated")
            finally:
                session.stop()
                unrelated.terminate()
                unrelated.wait(timeout=5)
                if child_pid and process_live(child_pid):
                    # This pid was created by this test and read from its own
                    # private marker. Re-check the token before fallback cleanup
                    # so even unexpected PID reuse cannot affect another job.
                    try:
                        environment_values = Path("/proc/{}/environ".format(child_pid)).read_bytes().split(b"\0")
                        if ("GAZEBOTEACH_SESSION_TOKEN=" + session.token).encode() in environment_values:
                            os.kill(child_pid, 9)
                    except FileNotFoundError:
                        pass

    def test_sessions_use_separate_master_ports_and_ignition_partitions(self):
        from gazeboteach.runtime import LessonSession
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            lesson = make_lesson(root / "source")
            first = LessonSession(root, lesson, storage=Storage(root / "data-a"))
            second = LessonSession(root, lesson, storage=Storage(root / "data-b"))
            try:
                for key in ("GAZEBO_MASTER_URI", "ROS_MASTER_URI", "IGN_PARTITION"):
                    self.assertTrue(first.env.get(key))
                    self.assertNotEqual(first.env[key], second.env[key], key)
            finally:
                first.stop()
                second.stop()


if __name__ == "__main__":
    unittest.main()
