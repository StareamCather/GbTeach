"""Real modern desktop integration in an isolated user-data and settings directory."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gazeboteach.modern import MODERN_PROFILES, PROFILE
CLASSIC = "classic11-noetic"


def time_ns(stats):
    return stats["sim_sec"] * 10**9 + stats["sim_nsec"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=MODERN_PROFILES, default=PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=210)
    options = parser.parse_args()
    active_profile = options.profile

    output = options.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    private = Path(tempfile.mkdtemp(prefix="gazeboteach-expansion-regression-")).resolve()
    settings = private / "settings"
    settings.mkdir()
    os.environ["XDG_CONFIG_HOME"] = str(settings)

    from PyQt5 import QtCore, QtWidgets
    from gazeboteach.app import create_application
    from gazeboteach.runtime import _token_processes
    from gazeboteach.storage import Storage

    # The organization/application constructor can use NativeFormat despite
    # setDefaultFormat. Set both formats' user paths before building the window.
    QtCore.QSettings.setDefaultFormat(QtCore.QSettings.IniFormat)
    for format_ in (QtCore.QSettings.NativeFormat, QtCore.QSettings.IniFormat):
        QtCore.QSettings.setPath(format_, QtCore.QSettings.UserScope, str(settings))
    expected_settings = {settings / "GazeboTeach" / name for name in ("Desktop.conf", "Desktop.ini")}
    settings_probe = QtCore.QSettings("GazeboTeach", "Desktop")
    configured_file = Path(settings_probe.fileName()).resolve()
    del settings_probe
    if configured_file not in expected_settings:
        raise RuntimeError("Unexpected Qt settings file before desktop startup: {}".format(configured_file))

    storage = Storage(private / "data")
    application, window = create_application(ROOT, storage=storage)
    application.setQuitOnLastWindowClosed(False)
    window.settings.setFallbacksEnabled(False)
    window.autostart.setChecked(False)
    window.show()
    report = {"ok": False, "scope": "Real Qt cross-profile navigation and modern sessions",
              "profile": active_profile,
              "data_directory": str(private), "checks": [], "sessions": []}
    sessions = []
    current = None
    paused = None
    workspace = None
    modified_world = None
    source_digest = None
    scratch = b"G04 private exercise scratch\n"
    phase = "environment"
    started = time.monotonic()
    closing_started = None
    advancing = False

    def passed(name, **details):
        report["checks"].append({"name": name, "passed": True, **details})

    def selected(lesson, profile):
        value = window.current_lesson
        return value is not None and (value.id, value.profile) == (lesson, profile)

    def click(name):
        button = window.buttons[name]
        if not button.isEnabled():
            raise AssertionError(f"Qt button {name} unavailable: {window.session_status.text()}")
        button.click()

    def register(lesson):
        session = window.session
        if not session or not session.running or (session.lesson.id, session.lesson.profile) != (lesson, active_profile):
            raise AssertionError(f"Real modern session {lesson} failed: {window.session_status.text()}")
        if any(previous.token == session.token or previous.env["GZ_PARTITION"] == session.env["GZ_PARTITION"]
               for previous in sessions):
            raise AssertionError("Session token or Gazebo partition was reused")
        health = session.health()
        if not health["gui_running"] or health["terminal_running"] != 2 or health["failed_processes"]:
            raise AssertionError(f"Owned GUI/terminal/server setup incomplete: {health}")
        sessions.append(session)
        report["sessions"].append({
            "lesson": lesson, "token": session.token, "workspace": str(session.workspace),
            "directory": str(session.session_dir), "runtime_log": str(session.log_path),
            "partition": session.env["GZ_PARTITION"], "ros_domain": session.env["ROS_DOMAIN_ID"],
            "gui_pid": session.gui.pid, "terminals": health["terminal_running"],
        })
        return session

    def assert_released(session):
        remaining = _token_processes(session.token)
        if remaining or session.domain_lease.handle is not None or session.running:
            raise AssertionError(f"Session ownership survived stop: {session.token}, {remaining}")

    def verification(session):
        value = window._last_verification
        if not value or value[0] != session.token or value[1].get("passed") is not True:
            raise AssertionError(f"Actual Verify button did not pass: {value}, {window.session_status.text()}")
        text = session.log_path.read_text(encoding="utf-8")
        marker = '\n{\n  "passed"'
        start = text.rfind(marker)
        if start < 0:
            raise AssertionError("Real verifier did not emit its experiment observation")
        experiment = json.JSONDecoder().raw_decode(text[start + 1:])[0]
        if experiment.get("passed") is not True:
            raise AssertionError("Verifier observation disagrees with UI result")
        return experiment

    def close():
        nonlocal phase, closing_started
        phase = "closing"
        closing_started = time.monotonic()
        window.close()

    def tick():
        nonlocal phase, current, paused, workspace, modified_world, source_digest, advancing
        if advancing:
            return
        advancing = True
        try:
            if phase != "environment" and phase != "closing":
                for modal in window.findChildren(QtWidgets.QMessageBox):
                    if modal.isVisible():
                        message = modal.text()
                        modal.reject()
                        raise RuntimeError(f"Unexpected application dialog: {message}")
            if phase != "closing" and time.monotonic() - started > options.timeout:
                raise RuntimeError(f"Desktop regression timed out during {phase}")
            if phase == "environment":
                if window.environment is None:
                    return
                if window.environment_dialog and window.environment_dialog.isVisible():
                    window.environment_dialog.accept()
                if window._environment_dialog_pending:
                    return
                if Path(window.settings.fileName()).resolve() != configured_file:
                    raise AssertionError("Unexpected desktop settings file: {}".format(window.settings.fileName()))
                if window.profile_selector.currentData() != active_profile:
                    raise AssertionError("Desktop did not auto-select the detected profile")
                if not window.select_lesson("01"):
                    raise AssertionError("Modern course map cannot select M01")
                if not selected("01", active_profile) or not window.document.toPlainText().strip():
                    raise AssertionError("Modern M01 material is not visible")
                passed("private_settings_and_live_environment", settings_file=window.settings.fileName(),
                       detected_profiles=window.environment.get("profiles", []))
                phase = "start_m01"
                click("start")
            elif phase == "start_m01" and not window.busy:
                current = register("01")
                phase = "verify_m01"
                click("verify")
            elif phase == "verify_m01" and not window.busy:
                value = verification(current)
                passed("m01_real_verify_from_qt", station_count=len(value["stations"]))
                phase = "pause_m01"
                click("pause")
            elif phase == "pause_m01" and not window.busy:
                paused = current.last_stats
                if not paused or not paused["paused"] or window.buttons["pause"].text() != "继续":
                    raise AssertionError(f"Pause was not confirmed by native WorldStatistics: {paused}")
                phase = "step_m01"
                click("step")
            elif phase == "step_m01" and not window.busy:
                after = current.last_stats
                if not after["paused"] or after["iterations"] != paused["iterations"] + 1 or time_ns(after) - time_ns(paused) != 1_000_000:
                    raise AssertionError(f"Qt step did not advance exactly 1 ms and one iteration: {paused}, {after}")
                if window.buttons["reset"].isEnabled():
                    raise AssertionError("Gazebo Sim offered unsupported model-only reset")
                passed("m01_qt_pause_and_exact_native_step", before=paused, after=after)
                workspace = current.workspace
                template = current.lesson.world
                source_digest = hashlib.sha256(template.read_bytes()).hexdigest()
                world = workspace / template.relative_to(current.lesson.folder)
                modified_world = world.read_bytes() + b"\n<!-- private learner edit -->\n"
                world.write_bytes(modified_world)
                (workspace / "regression-scratch.txt").write_bytes(scratch)
                phase = "switch_m26"
                if not window.select_lesson("26"):
                    raise AssertionError("Qt did not select M26")
            elif phase == "switch_m26" and not window.busy:
                assert_released(current)
                if not selected("26", active_profile):
                    raise AssertionError("Qt did not display M26 after cleaning M01")
                passed("same_profile_switch_releases_old_gazebo_and_dds", old_token=current.token)
                phase = "start_m26"
                click("start")
            elif phase == "start_m26" and not window.busy:
                current = register("26")
                phase = "verify_m26"
                click("verify")
            elif phase == "verify_m26" and not window.busy:
                value = verification(current)
                if len(value.get("cycles", [])) != 2:
                    raise AssertionError("M26 did not verify both real lifecycle cycles")
                passed("m26_real_verify_from_qt", lifecycle_cycles=len(value["cycles"]))
                phase = "switch_classic"
                if not window.select_profile(CLASSIC):
                    raise AssertionError("Classic teaching map absent")
            elif phase == "switch_classic" and not window.busy:
                assert_released(current)
                if not window.select_lesson("01") or not selected("01", CLASSIC):
                    raise AssertionError("Classic L01 material absent")
                reason = window._lesson_problem(window.current_lesson)
                if not window.document.toPlainText().strip() or (reason and window.buttons["start"].isEnabled()):
                    raise AssertionError("Classic material/compatibility is incorrect")
                classic_workspace = storage.workspace(window.current_lesson)
                if classic_workspace == workspace:
                    raise AssertionError("Classic L01 shares the modern M01 workspace")
                passed("cross_profile_switch_releases_m26_and_keeps_classic_readable",
                       old_token=current.token, classic_runtime_executed=False,
                       classic_compatibility=reason or "ready", classic_workspace=str(classic_workspace))
                if not window.select_profile(active_profile) or not window.select_lesson("01") or not selected("01", active_profile):
                    raise AssertionError("Cannot return to M01 after reading Classic L01")
                phase = "reenter_m01"
                click("start")
            elif phase == "reenter_m01" and not window.busy:
                current = register("01")
                template = current.lesson.world
                if (current.workspace != workspace or (workspace / template.relative_to(current.lesson.folder)).read_bytes() != modified_world
                        or (workspace / "regression-scratch.txt").read_bytes() != scratch
                        or hashlib.sha256(template.read_bytes()).hexdigest() != source_digest):
                    raise AssertionError("Private practice edit/scratch was lost or template changed")
                passed("edited_private_workspace_survives_switch_and_new_session", workspace=str(workspace))
                progress = storage.progress()
                for key in (active_profile + ":01", active_profile + ":26"):
                    if not {"read", "started", "verified"} <= progress.get(key, set()):
                        raise AssertionError(f"Missing profile-scoped learning records for {key}")
                if progress.get(CLASSIC + ":01") != {"read"}:
                    raise AssertionError("Classic read was merged into modern runtime records")
                passed("profile_scoped_progress_persists", progress={key: sorted(items) for key, items in progress.items()})
                screenshot = output.with_name(output.stem + "-desktop.png")
                if not window.grab().save(str(screenshot), "PNG"):
                    raise RuntimeError("Failed to save Qt desktop evidence")
                passed("qt_desktop_evidence", screenshot=str(screenshot))
                close()
            elif phase == "closing":
                if not window.isVisible():
                    if window.session is not None and all(item.token != window.session.token for item in sessions):
                        sessions.append(window.session)
                    for session in sessions:
                        assert_released(session)
                    if "error" not in report:
                        passed("active_qt_close_releases_every_owned_session", sessions=len(sessions))
                        report["ok"] = True
                    application.quit()
                elif time.monotonic() - closing_started > 20:
                    raise RuntimeError("Qt application did not close after waiting for owned shutdown")
        except Exception as error:
            if "error" not in report:
                report["error"] = f"{type(error).__name__}: {error}"
                report["failed_phase"] = phase
                report["qt_status"] = window.session_status.text()
            if phase == "closing":
                application.quit()
            else:
                close()
        finally:
            advancing = False

    timer = QtCore.QTimer()
    timer.timeout.connect(tick)
    timer.start(120)
    try:
        application.exec_()
    finally:
        timer.stop()
        if window.session is not None and all(item.token != window.session.token for item in sessions):
            sessions.append(window.session)
        for session in sessions:
            try:
                session.stop(reason="G04 desktop regression cleanup")
                assert_released(session)
            except Exception as error:
                report.setdefault("cleanup_errors", []).append(f"{session.token}: {error}")
                report["ok"] = False
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
