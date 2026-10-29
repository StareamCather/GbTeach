#!/usr/bin/env python3
"""Real Gazebo framework acceptance; all owned sessions are stopped in finally.

Run on ikaros from the repository root:
  python3 tests/integration_probe.py --gui --terminals --report /tmp/framework.json
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.catalog import Lesson
from gazeboteach.runtime import LessonSession, terminate_owned
from gazeboteach.storage import Storage


def command(session, *arguments):
    result = subprocess.run(list(arguments), env=session.env, cwd=str(session.workspace),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=8)
    if result.returncode:
        raise RuntimeError("Command failed: {}\n{}".format(" ".join(arguments), result.stderr))
    return result.stdout


def simulation_time(session):
    output = command(session, "gz", "topic", "-e", "/gazebo/diagnostic/world_stats", "-d", "1")
    samples = re.findall(r"sim_time\s*\{([^}]*)\}", output, re.S)
    if not samples:
        raise RuntimeError("No real world_stats sim_time received: " + output[-1000:])
    sample = samples[-1]
    seconds = re.search(r"\bsec:\s*(\d+)", sample)
    nanoseconds = re.search(r"\bnsec:\s*(\d+)", sample)
    return float(seconds.group(1) if seconds else 0) + float(nanoseconds.group(1) if nanoseconds else 0) / 1e9


def model_pose(session):
    output = command(session, "gz", "model", "-m", "diagnostic_block", "-p")
    values = re.findall(r"[-+]?(?:\d*\.\d+|\d+\.?\d*)(?:[eE][-+]?\d+)?", output)
    if len(values) != 6:
        raise RuntimeError("Unexpected model pose: " + output)
    return [float(value) for value in values]


def live_token_processes(token):
    needle = ("GAZEBOTEACH_SESSION_TOKEN=" + token).encode()
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            state = (entry / "stat").read_text().rsplit(") ", 1)[1].split()[0]
            if state != "Z" and needle in (entry / "environ").read_bytes().split(b"\0"):
                result.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            pass
    return result


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--terminals", action="store_true")
    parser.add_argument("--report")
    options = parser.parse_args()
    report = {"passed": False, "checks": [], "graphical_client": options.gui,
              "real_terminals": options.terminals}
    sessions = []
    started = time.monotonic()

    def check(name, **observations):
        item = dict(name=name, **observations)
        report["checks"].append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    with tempfile.TemporaryDirectory(prefix="gazeboteach-acceptance-") as directory:
        fixture = ROOT / "tests/fixtures/diagnostic"
        lesson = Lesson(fixture, json.loads((fixture / "lesson.json").read_text(encoding="utf-8")))
        try:
            first = LessonSession(ROOT, lesson, storage=Storage(Path(directory) / "first"), notify=print)
            second = LessonSession(ROOT, lesson, storage=Storage(Path(directory) / "second"), notify=print)
            sessions = [first, second]
            for key in ("GAZEBO_MASTER_URI", "ROS_MASTER_URI", "IGN_PARTITION"):
                require(first.env.get(key) and first.env[key] != second.env.get(key), "Isolation missing for " + key)
            first.start(gui=options.gui, terminals=options.terminals)
            second.start(gui=False, terminals=False)
            check("independent_sessions_started", masters=[x.env["GAZEBO_MASTER_URI"] for x in sessions],
                  partitions=[x.env["IGN_PARTITION"] for x in sessions])

            first.pause(True)
            first_time = simulation_time(first)
            second_time = simulation_time(second)
            later_first = simulation_time(first)
            later_second = simulation_time(second)
            require(abs(later_first - first_time) < 0.002, "Paused world continued advancing")
            require(later_second - second_time > 0.2, "Pausing first session affected second")
            check("pause_isolation", paused_delta=later_first - first_time,
                  other_session_delta=later_second - second_time)

            first.step(count=25)
            stepped = simulation_time(first)
            require(abs((stepped - later_first) - 0.025) < 0.006,
                    "25 simulation steps did not advance approximately 25 ms")
            check("single_step", steps=25, delta_seconds=stepped - later_first)

            command(first, "gz", "model", "-m", "diagnostic_block", "-x", "4", "-z", "3")
            moved = model_pose(first)
            require(abs(moved[0] - 4) < 0.03, "Model did not accept test position")
            before_reset = simulation_time(first)
            first.reset_models()
            reset = model_pose(first)
            after_reset = simulation_time(first)
            require(abs(reset[0]) < 0.03 and abs(reset[2] - 2.5) < 0.03,
                    "Model reset did not restore the initial model pose")
            require(abs(after_reset - before_reset) < 0.002, "Model reset unexpectedly changed simulation time")
            check("reset_models_preserves_time", moved_pose=moved, reset_pose=reset,
                  time_before=before_reset, time_after=after_reset)

            first.pause(False)
            result = first.verify()
            require(result.get("passed") is True, "Real fall-and-contact diagnostic failed")
            check("real_model_verifier", result=result)

            if options.gui:
                require(first.gui is not None and first.gui.poll() is None, "Gazebo GUI did not stay alive")
                terminate_owned(first.gui, token=first.token)
                require(first.running, "Closing GUI incorrectly stopped server")
                first.open_gui()
                require(first.gui.poll() is None, "Gazebo GUI failed to reopen")
                check("graphical_client_recovery", health=first.health())

            if options.terminals:
                require(bool(first.health().get("terminal_running")), "Real terminal shells did not become ready")
                check("real_terminal_start", health=first.health())

            for session in reversed(sessions):
                session.stop()
            deadline = time.monotonic() + 5
            leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            while any(leftovers.values()) and time.monotonic() < deadline:
                time.sleep(0.1)
                leftovers = {session.token: live_token_processes(session.token) for session in sessions}
            require(not any(leftovers.values()), "Session-owned processes survived cleanup: " + str(leftovers))
            check("owned_process_cleanup", leftovers=leftovers)
            report["passed"] = True
        except Exception as error:
            report["error"] = "{}: {}".format(type(error).__name__, error)
            report["session_logs"] = [str(session.log_path) for session in sessions]
            report["runtime_log_tails"] = {}
            for session in sessions:
                try:
                    report["runtime_log_tails"][session.token] = session.log_path.read_text(encoding="utf-8", errors="replace")[-24000:]
                except OSError:
                    pass
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
            raise
        finally:
            for session in reversed(sessions):
                session.stop(reason="框架验收最终清理")
            report["elapsed_seconds"] = round(time.monotonic() - started, 3)
            if options.report:
                path = Path(options.report).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
