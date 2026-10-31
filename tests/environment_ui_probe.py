#!/usr/bin/env python3
"""Real Qt startup popup, mismatch gating, reading, and reserved Docker UI checks."""

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    parser.add_argument("--screenshot", required=True)
    args = parser.parse_args()
    if not os.environ.get("DISPLAY") or os.environ.get("QT_QPA_PLATFORM") in ("offscreen", "minimal"):
        raise RuntimeError("This acceptance requires a real desktop display")
    from PyQt5 import QtCore
    from gazeboteach.app import create_application
    from gazeboteach.storage import Storage
    with tempfile.TemporaryDirectory(prefix="gazeboteach-environment-ui-") as directory:
        QtCore.QSettings.setDefaultFormat(QtCore.QSettings.IniFormat)
        QtCore.QSettings.setPath(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope, directory)
        app, window = create_application(ROOT, storage=Storage(Path(directory) / "data"))
        window.autostart.setChecked(False)
        window.show()
        deadline = time.monotonic() + 35
        try:
            while window.environment is None or window.busy:
                app.processEvents()
                if time.monotonic() > deadline:
                    raise AssertionError("Environment detection timed out")
                time.sleep(.03)
            app.processEvents()
            dialog = window.environment_dialog
            assert dialog.isVisible() and dialog.table.rowCount() == len(window.lessons)
            native_counts = window.environment_report["counts"]
            assert window.select_lesson("24")
            assert not window.buttons["start"].isEnabled(), "Autostart must wait for acknowledgement"
            dialog.accept()
            app.processEvents()
            assert window.buttons["start"].isEnabled(), window._lesson_problem(window.current_lesson)
            modern = dict(window.environment, os="Ubuntu 22.04 (synthetic compatibility case)",
                          classic_version="", modern_simulators=[{"command": "gz", "versions": "8.0.0"}],
                          profiles=[], ros_installed=["humble"], ros_active="humble", ros_packages=[])
            window._environment_dialog_pending = True
            window._environment_ready((modern, window.docker_probe))
            app.processEvents()
            dialog = window.environment_dialog
            assert dialog.isVisible()
            assert window.environment_report["counts"]["unsupported"] == len(window.lessons)
            assert Path(args.screenshot).parent.is_dir()
            assert dialog.grab().save(args.screenshot, "PNG")
            config = json.loads(dialog.docker_configuration.toPlainText())
            config["image"] = "bad image --privileged"
            dialog.docker_configuration.setPlainText(json.dumps(config))
            dialog._save_docker()
            assert "未保存" in dialog.docker_status.text()
            config["image"] = "osrf/ros:noetic-desktop-full"
            dialog.docker_configuration.setPlainText(json.dumps(config))
            dialog._save_docker()
            assert "尚未连接" in dialog.docker_status.text()
            assert not dialog.backends[1]["execution_implemented"]
            dialog.accept()
            app.processEvents()
            assert window.select_lesson("24")
            assert not window.buttons["start"].isEnabled()
            assert len(window.document.toPlainText()) > 500
            assert window.session is None
            window.show_environment_report()
            app.processEvents()
            assert window.environment_dialog.isVisible()
            window.environment_dialog.accept()
            result = {"passed": True, "native_counts": native_counts, "synthetic_modern_counts":
                      window.environment_report["counts"], "course_count": len(window.lessons),
                      "reading_available": True, "docker_configuration_validated": True,
                      "docker_execution_available": False, "no_session_created": window.session is None,
                      "screenshot": args.screenshot}
            Path(args.report).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(result, ensure_ascii=False))
        finally:
            if window.environment_dialog:
                window.environment_dialog.reject()
            window.close()
            app.processEvents()
            app.quit()


if __name__ == "__main__":
    main()
