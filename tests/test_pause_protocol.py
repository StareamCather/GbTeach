"""Pause protocol injection tests; no Gazebo process or transport is started."""

from collections import deque
import itertools
import json
from pathlib import Path
import shlex
import tempfile
import unittest
from unittest import mock

from gazeboteach.catalog import Lesson
from gazeboteach.runtime import LessonSession
from gazeboteach.storage import Storage


class ManagedCommandFixture:
    """Inject CLI output/exit status at Popen, preserving runtime command logic."""

    def __init__(self, session, samples, control_code=0, control_callback=None, step_code=0):
        self.session = session
        self.samples = deque(samples)
        self.last_sample = None
        self.control_code = control_code
        self.control_callback = control_callback
        self.step_code = step_code
        self.calls = []
        self.reads = []
        self.controls = []
        self.steps = []

    def spawn(self, arguments, **options):
        self.calls.append((arguments, options))
        command = shlex.split(arguments[-1])
        callback = None
        if command[:3] == ["gz", "topic", "-e"]:
            self.reads.append(command)
            if self.samples:
                self.last_sample = self.samples.popleft()
            sample = self.last_sample
            if isinstance(sample, bool):
                sample = {"text": "paused: {}\n".format(str(sample).lower())}
            elif isinstance(sample, str):
                sample = {"text": sample}
            Path(command[-1]).write_text(sample["text"], encoding="utf-8")
            code = sample.get("code", 0)
            callback = sample.get("callback")
        elif "-p" in command:
            self.controls.append(command)
            code = self.control_code
            callback = self.control_callback
            if code:
                options["stdout"].write(b"boost::thread_resource_error: Resource deadlock avoided\n")
        else:
            self.steps.append(command)
            code = self.step_code
        process = mock.Mock(pid=900000 + len(self.calls))
        process.returncode = None
        process.poll.side_effect = lambda: process.returncode

        def wait(timeout):
            nonlocal callback
            process.returncode = code
            if callback:
                action, callback = callback, None
                action()
            return code

        process.wait.side_effect = wait
        return process


class PauseProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        source = root / "source"
        source.mkdir()
        (source / "world.sdf").write_text("<sdf version='1.6'><world name='fixture'/></sdf>", encoding="utf-8")
        (source / "lesson.md").write_text("# Fixture\n", encoding="utf-8")
        lesson = Lesson(source, {"id": "01", "title": "Pause fixture", "requires_ros": False})
        with mock.patch("gazeboteach.runtime.lesson_environment", return_value={
                "GAZEBO_MASTER_URI": "http://127.0.0.1:11111", "IGN_PARTITION": "pause-unit"}):
            self.session = LessonSession(root, lesson, storage=Storage(root / "data"))
        self.session.server = mock.Mock()
        self.session.server.poll.return_value = None
        self.session.world_name = "fixture"
        self.session.state = "运行中"
        # Advance only the read-back deadline, without any real polling delay.
        self.clock = mock.patch("gazeboteach.runtime.time.monotonic", side_effect=itertools.count(0, 0.25))
        self.clock.start()
        self.addCleanup(self.clock.stop)
        wait = mock.patch.object(self.session.cancelled, "wait", return_value=False)
        wait.start()
        self.addCleanup(wait.stop)

    def inject(self, samples, **options):
        fixture = ManagedCommandFixture(self.session, samples, **options)
        patcher = mock.patch("gazeboteach.runtime.subprocess.Popen", side_effect=fixture.spawn)
        patcher.start()
        self.addCleanup(patcher.stop)
        return fixture

    def events(self):
        return [entry["event"] for entry in self.session.storage.records()]

    def assert_uncommitted(self):
        self.assertFalse(self.session.paused)
        self.assertEqual(self.session.state, "运行中")
        self.assertEqual(self.events(), [])
        self.assertEqual(list(self.session.session_dir.glob(".pause-state-*.txt")), [])

    def test_same_observed_state_skips_write_and_refreshes_cache(self):
        for target in (True, False):
            with self.subTest(target=target):
                self.session.paused = not target
                fixture = self.inject([target])
                self.session.pause(target)
                self.assertEqual(fixture.controls, [])
                self.assertEqual(len(fixture.reads), 1)
                self.assertEqual(self.session.paused, target)
                self.assertEqual(self.session.state, "已暂停" if target else "运行中")
                record = self.session.storage.records()[-1]
                self.assertEqual(record["details"]["observed_paused"], target)
                self.assertEqual(record["details"]["observation"], "world_stats")

    def test_reads_use_exact_world_owned_process_and_environment(self):
        fixture = self.inject(["paused: false\npaused: true\n"])
        self.session.pause(True)
        arguments, options = fixture.calls[0]
        self.assertEqual(arguments[:4], ["bash", "--noprofile", "--norc", "-c"])
        self.assertEqual(fixture.reads[0][:6], ["gz", "topic", "-e", "/gazebo/fixture/world_stats", "-d", "1"])
        self.assertIs(options["env"], self.session.env)
        self.assertEqual(options["cwd"], str(self.session.workspace))
        self.assertTrue(options["start_new_session"])
        process = self.session.processes[0][1]
        self.assertEqual(process._gazeboteach_token, self.session.token)
        self.assertEqual(process._gazeboteach_pgid, process.pid)
        self.assertEqual(list(self.session.session_dir.glob(".pause-state-*.txt")), [])

    def test_resume_sends_once_and_requires_observed_false(self):
        self.session.paused = True
        self.session.state = "已暂停"
        fixture = self.inject([True, False])
        self.session.pause(False)
        self.assertEqual(fixture.controls, [["gz", "world", "-w", "fixture", "-p", "0"]])
        self.assertFalse(self.session.paused)
        self.assertEqual(self.events(), ["resumed"])

    def test_command_error_after_mutation_accepts_observation_and_keeps_warning(self):
        fixture = self.inject([False, True], control_code=139)
        self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assertEqual(len(fixture.reads), 2)
        self.assertTrue(self.session.paused)
        log = self.session.log_path.read_text(encoding="utf-8")
        self.assertIn("Resource deadlock avoided", log)
        self.assertIn("警告", log)
        self.assertIn("命令执行失败（139）", log)
        self.assertIn("未重发控制", log)
        record = self.session.storage.records()[-1]
        self.assertIn("命令执行失败（139）", record["details"]["command_error"])
        metadata = json.loads((self.session.session_dir / "session.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["state"], "已暂停")

    def test_command_error_without_effect_fails_without_resend_or_cache_update(self):
        fixture = self.inject([False], control_code=139)
        with self.assertRaisesRegex(RuntimeError, "命令执行失败（139）.*回读未确认") as error:
            self.session.pause(True)
        self.assertIsInstance(error.exception.__cause__, RuntimeError)
        self.assertEqual(len(fixture.controls), 1)
        self.assertGreater(len(fixture.reads), 1)
        self.assert_uncommitted()

    def test_command_error_with_unknown_readback_fails_without_resend(self):
        fixture = self.inject([False, "sim_time { sec: 12 }\n"], control_code=139)
        with self.assertRaisesRegex(RuntimeError, "命令执行失败（139）.*未提供实际 paused"):
            self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assert_uncommitted()

    def test_successful_command_without_observed_effect_is_not_success(self):
        fixture = self.inject([False])
        with self.assertRaisesRegex(RuntimeError, "回读未确认目标 paused=true"):
            self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assert_uncommitted()

    def test_unknown_initial_state_never_sends_mutation(self):
        fixture = self.inject(["paused: unknown\n"])
        with self.assertRaisesRegex(RuntimeError, "未提供实际 paused"):
            self.session.pause(True)
        self.assertEqual(fixture.controls, [])
        self.assert_uncommitted()

    def test_failed_initial_read_never_trusts_partial_stdout(self):
        fixture = self.inject([{"text": "paused: true\n", "code": 7}])
        with self.assertRaisesRegex(RuntimeError, "命令执行失败（7）"):
            self.session.pause(True)
        self.assertEqual(fixture.controls, [])
        self.assert_uncommitted()

    def test_delayed_effect_retries_only_reads(self):
        fixture = self.inject([False, False, True])
        self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assertEqual(len(fixture.reads), 3)
        self.assertTrue(self.session.paused)

    def test_cancellation_during_control_does_not_read_or_commit_after_stop(self):
        fixture = self.inject([False], control_callback=self.session.cancelled.set)
        with self.assertRaisesRegex(RuntimeError, "取消"):
            self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assertEqual(len(fixture.reads), 1)
        self.assert_uncommitted()

    def test_cancellation_during_confirmation_does_not_commit_observed_target(self):
        fixture = self.inject([False, {"text": "paused: true\n", "callback": self.session.cancelled.set}])
        with self.assertRaisesRegex(RuntimeError, "实验已结束"):
            self.session.pause(True)
        self.assertEqual(len(fixture.controls), 1)
        self.assertEqual(len(fixture.reads), 2)
        self.assert_uncommitted()

    def test_stopped_session_is_not_revived_by_confirmation(self):
        fixture = self.inject([False, {"text": "paused: true\n", "callback": self.session.stop}])
        with mock.patch("gazeboteach.runtime.terminate_owned"), \
                mock.patch("gazeboteach.runtime._token_processes", return_value=[]):
            with self.assertRaisesRegex(RuntimeError, "实验已结束"):
                self.session.pause(True)
        self.assertEqual(self.session.state, "已结束")
        self.assertFalse(self.session.paused)
        self.assertEqual(self.events(), ["stopped"])
        self.assertEqual(len(fixture.controls), 1)
        self.assertEqual(len(fixture.reads), 2)

    def test_cancelled_session_never_starts_a_read_or_write(self):
        fixture = self.inject([True])
        self.session.cancelled.set()
        with self.assertRaisesRegex(RuntimeError, "当前没有正在运行的实验"):
            self.session.pause(True)
        self.assertEqual(fixture.calls, [])
        self.assert_uncommitted()

    def test_server_exit_during_initial_read_does_not_write_or_commit(self):
        fixture = self.inject([{"text": "paused: true\n", "callback": lambda: setattr(
            self.session.server.poll, "return_value", 1)}])
        with self.assertRaisesRegex(RuntimeError, "实验已结束"):
            self.session.pause(True)
        self.assertEqual(fixture.controls, [])
        self.assert_uncommitted()

    def test_step_rechecks_stale_cache_pauses_then_sends_one_pulse(self):
        self.session.paused = True
        fixture = self.inject([False, True])
        self.session.step(5)
        self.assertEqual(fixture.controls, [["gz", "world", "-w", "fixture", "-p", "1"]])
        self.assertEqual(fixture.steps, [["gz", "world", "-w", "fixture", "-m", "5"]])
        self.assertEqual([shlex.split(call[0][-1])[1] for call in fixture.calls],
                         ["topic", "world", "topic", "world"])
        self.assertEqual(self.events(), ["paused", "stepped"])

    def test_step_error_is_not_replayed_or_recorded_as_stepped(self):
        fixture = self.inject([True], step_code=139)
        with self.assertRaisesRegex(RuntimeError, "命令执行失败（139）"):
            self.session.step()
        self.assertEqual(fixture.controls, [])
        self.assertEqual(fixture.steps, [["gz", "world", "-w", "fixture", "-s"]])
        self.assertEqual(self.events(), ["paused"])

    def test_step_does_not_send_pulse_when_pause_cannot_be_confirmed(self):
        self.session.paused = True
        fixture = self.inject(["sim_time { sec: 12 }\n"])
        with self.assertRaisesRegex(RuntimeError, "未提供实际 paused"):
            self.session.step(5)
        self.assertEqual(fixture.controls, [])
        self.assertEqual(fixture.steps, [])
        self.assertEqual(self.events(), [])


if __name__ == "__main__":
    unittest.main()
