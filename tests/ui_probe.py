#!/usr/bin/env python3
"""Exercise the real Qt workbench through its own buttons, never OS input.

This check requires the real ikaros graphical session; offscreen rendering is
explicitly rejected. It creates private settings/data and closes only its own
window and session. Example:
  python3 tests/ui_probe.py --screenshot /tmp/gazeboteach-ui.png --report /tmp/ui.json
  python3 tests/ui_probe.py --lesson 01 --profile classic11-noetic --screenshot /tmp/lesson01.png

The default keeps the framework-only fixture. --lesson loads the published
catalog through the same course tree as the normal application. Reference
inspection compares the personal world hash before and after the read-only view.
For an exercise with learner.py, --expect-unfinished checks an ungraded demo;
--exercise-reference fills only this probe's temporary student copy.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


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


def gazebo_window_id(session):
    """Select only an X11 window owned by the tracked client or its token."""
    result = subprocess.run(["xprop", "-root", "_NET_CLIENT_LIST"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError("Cannot list X11 client windows: " + result.stderr.strip())
    candidates = []
    for identity in re.findall(r"0x[0-9a-fA-F]+", result.stdout):
        props = subprocess.run(["xprop", "-id", identity, "_NET_WM_PID"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        match = re.search(r"=\s*(\d+)", props.stdout)
        if not match:
            continue
        pid = int(match.group(1))
        candidates.append({"xid": identity, "pid": pid})
        if pid == session.gui.pid:
            return int(identity, 16), candidates
        try:
            process = Path("/proc/{}".format(pid))
            token = ("GAZEBOTEACH_SESSION_TOKEN=" + session.token).encode()
            if (process / "comm").read_text().strip() == "gzclient" and token in (process / "environ").read_bytes().split(b"\0"):
                return int(identity, 16), candidates
        except OSError:
            pass
    raise RuntimeError("No X11 window belongs to this Gazebo client: " + json.dumps(candidates))


def terminal_window_ids(session):
    """GNOME shares a daemon PID: the unique role, not that PID, proves ownership."""
    result = subprocess.run(["xprop", "-root", "_NET_CLIENT_LIST"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError("Cannot list session terminal windows: " + result.stderr.strip())
    role = "gazeboteach-" + session.token
    windows = []
    for identity in re.findall(r"0x[0-9a-fA-F]+", result.stdout):
        props = subprocess.run(["xprop", "-id", identity, "WM_WINDOW_ROLE"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
        match = re.search(r'=\s*"([^"]*)"', props.stdout)
        if match and match.group(1) == role:
            windows.append(int(identity, 16))
    return windows


def terminal_pty_status(session):
    """Require a real controlling PTY and interactive Bash monitor mode."""
    statuses = []
    for marker in session.terminal_markers:
        try:
            data = json.loads(marker.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        if data.get("token") != session.token:
            raise AssertionError("Terminal marker does not belong to this session")
        if not data.get("isatty") or data.get("tty_pgrp") is None:
            return None
        if data.get("pid") != data.get("pgid") or data.get("pgid") != data.get("sid"):
            raise AssertionError("Terminal wrapper lacks its own session/process group: " + str(data))
        if data["tty_pgrp"] != data["pgid"]:
            raise AssertionError("Terminal is not the foreground group of its controlling PTY: " + str(data))
        if data.get("sid") == os.getsid(0) or data.get("sid") == os.getsid(session.server.pid):
            raise AssertionError("Terminal session is not isolated from test/server sessions")
        if data.get("interactive") and data.get("shell_job_control") is not True:
            return None
        statuses.append(data)
    if len(statuses) < 2 or len({item["sid"] for item in statuses}) != len(statuses):
        return None
    if not any(item.get("interactive") for item in statuses):
        return None
    return statuses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--screenshot", required=True)
    parser.add_argument("--lesson", metavar="ID", help="Use a published course from the local catalog; default is diagnostic fixture 00")
    parser.add_argument("--profile", default="classic11-noetic", help="Course profile used with --lesson")
    parser.add_argument("--gazebo-screenshot", help="PNG of only this session's real Gazebo window")
    parser.add_argument("--report")
    parser.add_argument("--timeout", type=float, default=90)
    exercise = parser.add_mutually_exclusive_group()
    exercise.add_argument("--expect-unfinished", action="store_true",
                          help="Require a rejected assignment check and no completed learning record")
    exercise.add_argument("--exercise-reference", action="store_true",
                          help="Fill learner.py only in this probe's private Storage and select student mode")
    arguments = parser.parse_args()
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("UI acceptance requires the real X11 DISPLAY")
    if os.environ.get("QT_QPA_PLATFORM", "").lower() in ("offscreen", "minimal"):
        raise RuntimeError("Offscreen/minimal rendering cannot satisfy desktop acceptance")
    if arguments.timeout <= 0:
        raise ValueError("--timeout must be positive")

    from PyQt5 import QtCore, QtGui, QtWidgets
    from gazeboteach.app import create_application
    from gazeboteach.catalog import Lesson, load_catalog
    from gazeboteach.storage import Storage

    screenshot = Path(arguments.screenshot).expanduser().resolve()
    screenshot.parent.mkdir(parents=True, exist_ok=True)
    report = {"passed": False, "checks": [], "messages": [], "screenshot": str(screenshot),
              "display": os.environ["DISPLAY"]}
    started = time.monotonic()
    original_hook = sys.excepthook
    window = None
    application = None
    state = {"stage": "environment", "failed": False, "finished": False}

    def record(name, **observations):
        item = dict(name=name, elapsed_seconds=round(time.monotonic() - started, 3), **observations)
        report["checks"].append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    with tempfile.TemporaryDirectory(prefix="gazeboteach-ui-acceptance-") as directory:
        private = Path(directory)
        # MainWindow uses QSettings in addition to Storage. Isolate both so this
        # test does not change the learner's autostart preference or last lesson.
        QtCore.QSettings.setDefaultFormat(QtCore.QSettings.IniFormat)
        QtCore.QSettings.setPath(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope,
                                str(private / "settings"))
        if arguments.lesson:
            lessons, errors = load_catalog(ROOT)
            lesson_id = str(arguments.lesson).zfill(2)
            candidates = [candidate for candidate in lessons
                          if candidate.id == lesson_id and candidate.profile == arguments.profile]
            require(len(candidates) == 1, "Expected one published course {}:{}; catalog errors: {}".format(
                arguments.profile, lesson_id, errors))
            lesson = candidates[0]
            fixture_override = None
        else:
            fixture_folder = ROOT / "tests/fixtures/diagnostic"
            lesson = Lesson(fixture_folder, json.loads((fixture_folder / "lesson.json").read_text(encoding="utf-8")))
            fixture_override = lesson
        report.update(lesson=lesson.id, profile=lesson.profile, title=lesson.title,
                      fixture=fixture_override is not None)
        storage = Storage(private / "data")
        exercise_sources = {}
        if arguments.expect_unfinished or arguments.exercise_reference:
            require(arguments.lesson is not None, "Exercise flags require --lesson")
            for name in ("learner.py", "reference_controller.py", "runtime.json"):
                path = lesson.folder / name
                require(path.is_file(), "Exercise test contract is missing " + name)
                exercise_sources[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            practice = storage.prepare(lesson)
            if arguments.exercise_reference:
                (practice / "learner.py").write_bytes((lesson.folder / "reference_controller.py").read_bytes())
                (practice / "runtime.json").write_text(json.dumps({"mode": "student"}) + "\n", encoding="utf-8")
            else:
                require(json.loads((practice / "runtime.json").read_text(encoding="utf-8"))["mode"] == "reference_demo",
                        "Unfinished exercise probe requires the published default reference_demo mode")
            report["exercise_test"] = {
                "mode": "private_reference_as_student" if arguments.exercise_reference else "unfinished_demo",
                "private_learner_sha256": hashlib.sha256((practice / "learner.py").read_bytes()).hexdigest(),
                "source_learner_sha256": exercise_sources["learner.py"],
                "source_templates_unchanged": None,
            }
        application, window = create_application(ROOT, fixture=fixture_override, storage=storage)
        application.setQuitOnLastWindowClosed(False)
        require(application.platformName() not in ("offscreen", "minimal"), "Qt did not select a real desktop platform")
        window.autostart.setChecked(False)
        window.show()
        report["qt_platform"] = application.platformName()
        report["qt_version"] = QtCore.QT_VERSION_STR
        report["private_data"] = str(storage.base)

        def begin_close():
            if state["stage"] != "closing":
                state["stage"] = "closing"
                state["cleanup_deadline"] = time.monotonic() + 20
                window.close()

        def fail(message):
            if not state["failed"]:
                state["failed"] = True
                report["error"] = message
                print("UI_PROBE_FAILED " + message, flush=True)
            begin_close()

        def callback_exception(kind, value, tb):
            report["callback_traceback"] = "".join(traceback.format_exception(kind, value, tb))
            fail("Qt callback raised {}: {}".format(kind.__name__, value))

        sys.excepthook = callback_exception

        def click(name):
            button = window.buttons[name]
            require(button.isEnabled(), "Expected enabled button: " + name)
            button.click()

        def tick():
            if state["finished"]:
                return
            try:
                # These are only QWidget objects created inside this test's Qt
                # process. We never accept dialogs belonging to another app.
                for widget in application.topLevelWidgets():
                    if isinstance(widget, QtWidgets.QMessageBox) and widget.isVisible():
                        message = {"title": widget.windowTitle(), "text": widget.text()}
                        report["messages"].append(message)
                        widget.done(QtWidgets.QMessageBox.Cancel)
                        fail("Workbench displayed a warning dialog: " + message["text"])
                        return
                if state["stage"] != "closing" and time.monotonic() - started > arguments.timeout:
                    fail("Workbench flow exceeded {} seconds at {}".format(arguments.timeout, state["stage"]))
                    return

                stage = state["stage"]
                if stage == "environment":
                    if window.environment is None or window.busy:
                        return
                    require(window.isVisible(), "Workbench is not visible")
                    if lesson.profile == "classic11-noetic":
                        require(window.environment.get("classic_version", "").startswith("11."), "Expected detected Classic 11")
                    require(window.select_lesson(lesson.id, profile=lesson.profile), "Requested lesson is not selectable")
                    state["stage"] = "selected"
                    return

                if stage == "selected":
                    if window.busy:
                        return
                    require(window.current_lesson is not None and window.current_lesson.key == lesson.key, "Wrong current lesson")
                    require(lesson.title in window.course_title.text(), "Selected course title was not shown")
                    require(len(window.document.toPlainText().strip()) >= 80, "Lesson document is missing or too short")
                    record("environment_and_lesson", classic=window.environment.get("classic_version"),
                           lesson=window.current_lesson.id, profile=lesson.profile,
                           title=window.course_title.text(), document_characters=len(window.document.toPlainText()))
                    click("start")
                    state["stage"] = "starting"
                    return

                if stage == "starting":
                    if window.busy or window.session is None:
                        return
                    require(window.session.running, "Start button did not produce a running experiment")
                    health = window.session.health()
                    require(health.get("gui_running"), "Start button did not start the Gazebo client")
                    require(health.get("terminal_running", 0) >= 2, "Start button did not open both course terminals")
                    pty_status = terminal_pty_status(window.session)
                    if pty_status is None:
                        state.setdefault("pty_deadline", time.monotonic() + 5)
                        require(time.monotonic() < state["pty_deadline"],
                                "Terminal did not provide a controlling PTY and interactive Bash job control")
                        return
                    state["token"] = window.session.token
                    record("start_button_real_components", health=health)
                    record("real_terminal_pty_and_job_control", terminals=pty_status)
                    click("pause")
                    state["stage"] = "pausing"
                    return

                if stage == "pausing":
                    if window.busy:
                        return
                    require(window.session.paused, "Pause button did not pause the session")
                    require(window.buttons["pause"].text() == "继续", "Pause button label did not become Continue")
                    record("pause_button")
                    state["steps_before"] = sum(x.get("event") == "stepped" for x in storage.records())
                    click("step")
                    state["stage"] = "stepping"
                    return

                if stage == "stepping":
                    if window.busy:
                        return
                    steps = sum(x.get("event") == "stepped" for x in storage.records())
                    require(steps == state["steps_before"] + 1, "Step button did not record a completed step")
                    require(window.session.paused, "Single step unexpectedly left paused mode")
                    record("step_button", recorded_steps=steps)
                    click("pause")
                    state["stage"] = "resuming"
                    return

                if stage == "resuming":
                    if window.busy:
                        return
                    require(not window.session.paused, "Continue button did not resume the session")
                    require(window.buttons["pause"].text() == "暂停", "Continue button label did not become Pause")
                    record("continue_button")
                    click("verify")
                    state["stage"] = "verifying"
                    return

                if stage == "verifying":
                    if window.busy:
                        return
                    result = window._last_verification
                    require(result is not None and result[0] == window.session.token,
                            "Verify button did not return a result for this session")
                    expected = not arguments.expect_unfinished
                    require(result[1].get("passed") is expected,
                            "Course verification result differs from the explicitly requested exercise state")
                    completed = "verified" in storage.progress().get(lesson.key, set())
                    require(completed is expected,
                            "Exercise completion record differs from the actual assignment result")
                    if arguments.expect_unfinished or arguments.exercise_reference:
                        learner_hash = hashlib.sha256((window.session.workspace / "learner.py").read_bytes()).hexdigest()
                        require(learner_hash == report["exercise_test"]["private_learner_sha256"],
                                "UI verification changed the personal learner.py")
                    record("verify_button", result=result[1])
                    if lesson.data.get("reference"):
                        world = window.session.workspace / lesson.data.get("world", "world.sdf")
                        state["reference_world_hash"] = hashlib.sha256(world.read_bytes()).hexdigest()
                        click("reference")
                        state["stage"] = "reference"
                        return
                    click("records")
                    state["stage"] = "records"
                    return

                if stage == "reference":
                    if window.busy:
                        return
                    require(window.tabs.currentWidget() is window.reference_view, "Reference button did not open the reference tab")
                    require(window.reference_view.isReadOnly(), "Reference view is editable")
                    require(len(window.reference_view.toPlainText().strip()) >= 10, "Reference answer is empty")
                    world = window.session.workspace / lesson.data.get("world", "world.sdf")
                    actual = hashlib.sha256(world.read_bytes()).hexdigest()
                    require(actual == state["reference_world_hash"], "Opening the reference answer changed the personal world file")
                    record("reference_preserves_personal_world", world_sha256=actual,
                           reference_characters=len(window.reference_view.toPlainText()))
                    window.tabs.setCurrentWidget(window.document)
                    click("records")
                    state["stage"] = "records"
                    return

                if stage == "records":
                    require(window.tabs.currentWidget() is window.records_view, "Records button did not show the records tab")
                    text = window.records_view.toPlainText()
                    event = "verification_failed" if arguments.expect_unfinished else "verified"
                    require(event in text and "started" in text, "Records view is missing run and verification events")
                    if arguments.expect_unfinished:
                        require(not any(item.get("event") == "verified" and item.get("key") == lesson.key
                                        for item in storage.records(10000)),
                                "A reference demonstration was incorrectly recorded as completed work")
                    record("records_button", record_characters=len(text))
                    window.tabs.setCurrentWidget(window.document)
                    state["stage"] = "screenshot"
                    return

                if stage == "screenshot":
                    require(window.isVisible(), "Workbench disappeared before capture")
                    require(window.grab().save(str(screenshot), "PNG"), "Workbench screenshot could not be saved")
                    require(screenshot.is_file() and screenshot.stat().st_size > 1000, "Screenshot is empty")
                    record("own_window_screenshot", bytes=screenshot.stat().st_size)
                    if arguments.gazebo_screenshot:
                        require(window.session.health().get("gui_running"), "Gazebo exited before rendered-window capture")
                        try:
                            identity, candidates = gazebo_window_id(window.session)
                        except Exception:
                            diagnostic = Path(arguments.gazebo_screenshot).expanduser().resolve().with_suffix(".xwininfo.txt")
                            diagnostic.parent.mkdir(parents=True, exist_ok=True)
                            try:
                                info = subprocess.run(["xwininfo", "-root", "-tree"],
                                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                      text=True, timeout=5)
                                diagnostic.write_text(info.stdout, encoding="utf-8")
                                report["gazebo_window_diagnostic"] = str(diagnostic)
                            except (OSError, subprocess.TimeoutExpired):
                                pass
                            raise
                        state["gazebo_xid"] = identity
                        # Remove only this workbench and terminal windows carrying
                        # this exact session role. A shared GNOME daemon PID must
                        # never cause another user's terminal to be minimized.
                        terminals = terminal_window_ids(window.session)
                        require(len(terminals) >= 2, "Cannot prove ownership of both terminal windows by WM_WINDOW_ROLE")
                        state["terminal_window_wrappers"] = [QtGui.QWindow.fromWinId(xid) for xid in terminals]
                        for terminal in state["terminal_window_wrappers"]:
                            terminal.showMinimized()
                        window.hide()
                        state["gazebo_foreign_window"] = QtGui.QWindow.fromWinId(identity)
                        state["gazebo_foreign_window"].showNormal()
                        state["gazebo_foreign_window"].raise_()
                        state["gazebo_foreign_window"].requestActivate()
                        state["gazebo_capture_after"] = time.monotonic() + 0.8
                        record("remove_own_capture_obstructions", terminal_xids=[hex(xid) for xid in terminals])
                        state["stage"] = "gazebo_screenshot"
                        return
                    begin_close()
                    return

                if stage == "gazebo_screenshot":
                    if time.monotonic() < state["gazebo_capture_after"]:
                        return
                    target = Path(arguments.gazebo_screenshot).expanduser().resolve()
                    target.parent.mkdir(parents=True, exist_ok=True)
                    pixmap = application.primaryScreen().grabWindow(state["gazebo_xid"])
                    require(not pixmap.isNull() and pixmap.width() > 50 and pixmap.height() > 50,
                            "Gazebo window capture returned an empty or invalid image")
                    require(pixmap.save(str(target), "PNG"), "Gazebo window PNG could not be saved")
                    require(target.stat().st_size > 1000, "Gazebo window PNG is empty")
                    report["gazebo_screenshot"] = str(target)
                    record("own_gazebo_window_screenshot", xid=hex(state["gazebo_xid"]),
                           width=pixmap.width(), height=pixmap.height(), bytes=target.stat().st_size)
                    begin_close()
                    return

                if stage == "closing":
                    if window.isVisible():
                        if time.monotonic() > state["cleanup_deadline"]:
                            state["failed"] = True
                            report.setdefault("error", "Workbench did not finish its close/cleanup handler")
                            state["finished"] = True
                            application.quit()
                        return
                    token = state.get("token") or (window.session.token if window.session else None)
                    leftovers = live_token_processes(token) if token else []
                    if leftovers and time.monotonic() < state["cleanup_deadline"]:
                        return
                    require(not leftovers, "Workbench close left session processes alive: " + str(leftovers))
                    record("close_window_cleanup", leftovers=leftovers)
                    report["passed"] = not state["failed"]
                    state["finished"] = True
                    application.quit()
            except Exception as error:
                if state["stage"] == "closing":
                    report.setdefault("error", "{}: {}".format(type(error).__name__, error))
                    state["failed"] = True
                    state["finished"] = True
                    application.quit()
                else:
                    fail("{}: {}".format(type(error).__name__, error))

        timer = QtCore.QTimer()
        timer.timeout.connect(tick)
        timer.start(100)
        try:
            application.exec_()
        finally:
            timer.stop()
            if window.session is not None:
                # This is the exact session created by the test window. A final
                # direct stop protects against failures in the UI close callback.
                window.session.stop(reason="UI 验收最终清理")
                token = window.session.token
                leftovers = live_token_processes(token)
                if leftovers:
                    report["passed"] = False
                    report["cleanup_leftovers"] = leftovers
                try:
                    report["runtime_log_tail"] = window.session.log_path.read_text(encoding="utf-8", errors="replace")[-24000:]
                except OSError:
                    pass
            window._allow_close = True
            window.close()
            application.processEvents()
            sys.excepthook = original_hook
            report["elapsed_seconds"] = round(time.monotonic() - started, 3)
            if exercise_sources:
                unchanged = all(hashlib.sha256((lesson.folder / name).read_bytes()).hexdigest() == value
                                for name, value in exercise_sources.items())
                report["exercise_test"]["source_templates_unchanged"] = unchanged
                if not unchanged:
                    report["passed"] = False
                    report.setdefault("error", "UI exercise test changed a published source template")
            if arguments.report:
                path = Path(arguments.report).expanduser().resolve()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
