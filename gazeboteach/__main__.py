"""Command line entry points for read-only diagnosis and bounded real experiments."""

import argparse
import json
import os
import signal
import sys
import tempfile
import time
from pathlib import Path

from .catalog import Lesson, compatibility, load_catalog
from .environment import detect_environment
from .runtime import LessonSession
from .storage import Storage


ROOT = Path(__file__).resolve().parent.parent


def emit(value):
    print(json.dumps(value, ensure_ascii=False, indent=2), flush=True)


def parser():
    result = argparse.ArgumentParser(description="GazeboTeach 原生桌面教学工作台")
    mode = result.add_mutually_exclusive_group()
    mode.add_argument("--diagnose", action="store_true", help="只读检查已安装环境，输出 JSON")
    mode.add_argument("--list", action="store_true", help="列出课包和兼容状态，输出 JSON")
    mode.add_argument("--smoke", metavar="ID", help="在新临时练习目录启动本课并执行真实验收")
    mode.add_argument("--gui-check", action="store_true", help="显示 Qt 窗口，截图并有界退出")
    result.add_argument("--smoke-fixture", metavar="PATH", help="使用独立诊断场景（目录内含 lesson.json）")
    result.add_argument("--gui", action="store_true", help="smoke 时同时启动真实 Gazebo GUI")
    result.add_argument("--terminals", action="store_true", help="smoke 时打开本课日志与操作终端")
    result.add_argument("--seconds", type=float, default=2.0, help="smoke 验收前观察秒数（默认 2）")
    result.add_argument("--screenshot", metavar="PATH", help="--gui-check 的 PNG 输出路径")
    result.add_argument("--data-dir", metavar="PATH", help="显式指定独立数据目录；smoke 默认每次新建临时目录")
    result.add_argument("--lesson", metavar="ID", help="桌面模式打开指定课程")
    result.add_argument("--profile", metavar="PROFILE", help="显式选择教学包，避免多个版本存在相同课号时产生歧义")
    return result


def fixture_lesson(path):
    folder = Path(path).expanduser().resolve()
    manifest = folder / "lesson.json"
    if not manifest.is_file():
        raise RuntimeError("诊断场景缺少 lesson.json：" + str(manifest))
    return Lesson(folder, json.loads(manifest.read_text(encoding="utf-8-sig")))


def new_storage(arguments, purpose):
    if arguments.data_dir:
        base = Path(arguments.data_dir).expanduser().resolve()
    else:
        base = Path(tempfile.mkdtemp(prefix="gazeboteach-" + purpose + "-"))
    # Logs intentionally remain here after exit; a fresh path prevents stale workspaces
    # from making an updated source lesson appear to pass an earlier version's check.
    return Storage(base)


def require_graphics():
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY") or
            os.environ.get("QT_QPA_PLATFORM") in ("offscreen", "minimal")):
        raise RuntimeError("没有图形会话。请在 Ubuntu 桌面运行，或仅使用 --diagnose / 无 --gui 的 --smoke。")


def smoke(arguments, environment, fixture=None):
    lessons, errors = load_catalog(ROOT)
    if fixture is not None:
        lesson = fixture
        if arguments.profile and lesson.profile != arguments.profile:
            raise RuntimeError("诊断场景的教学包与 --profile 不一致。")
    else:
        lesson_id = str(arguments.smoke).zfill(2)
        candidates = [x for x in lessons if x.id == lesson_id and (not arguments.profile or x.profile == arguments.profile) and compatibility(
            x, environment, gui=arguments.gui, terminals=arguments.terminals) is None]
        if len(candidates) != 1:
            reasons = [{"profile": x.profile, "reason": compatibility(
                x, environment, gui=arguments.gui, terminals=arguments.terminals)}
                       for x in lessons if x.id == lesson_id]
            raise RuntimeError("未找到唯一匹配的课程 {}：{}；课包错误：{}".format(lesson_id, reasons, errors))
        lesson = candidates[0]
    reason = compatibility(lesson, environment, gui=arguments.gui, terminals=arguments.terminals)
    if reason:
        raise RuntimeError(reason)
    if arguments.gui or arguments.terminals:
        require_graphics()
    storage = new_storage(arguments, "smoke")
    session = None
    started = time.monotonic()
    report = {"mode": "smoke", "lesson": lesson.id, "profile": lesson.profile,
              "data_dir": str(storage.base), "fresh_data_dir": not bool(arguments.data_dir),
              "gui_requested": arguments.gui, "terminals_requested": arguments.terminals}
    def interrupted(signum, frame):
        raise KeyboardInterrupt("收到结束信号 {}".format(signum))
    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        session = LessonSession(ROOT, lesson, storage=storage,
                                notify=lambda message: print(message, file=sys.stderr, flush=True))
        report.update(workspace=str(session.workspace), log=str(session.log_path),
                      session_dir=str(session.session_dir))
        session.start(gui=arguments.gui, terminals=arguments.terminals)
        session.cancelled.wait(max(0.0, min(arguments.seconds, 120.0)))
        health = session.health()
        report["health"] = health
        if not session.running:
            raise RuntimeError("验收前 Gazebo 服务端已退出。")
        if arguments.gui and not health.get("gui_running", False):
            raise RuntimeError("Gazebo 图形客户端已退出，请查看 runtime.log。")
        verification = session.verify()
        report["verification"] = verification
        report["ok"] = verification.get("passed") is True
        if verification.get("passed") is None:
            report["error"] = "本课没有自动验收脚本；仅启动成功不计为通过。"
    except KeyboardInterrupt as exc:
        report.update(ok=False, interrupted=True, error=str(exc))
    except Exception as exc:
        report.update(ok=False, error="{}: {}".format(type(exc).__name__, exc))
    finally:
        if session:
            try:
                session.stop(reason="CLI 诊断实验结束")
                report["final_health"] = session.health()
                if (report["final_health"].get("running") or report["final_health"].get("gui_running") or
                        report["final_health"].get("terminal_running", 0)):
                    report.update(ok=False, cleanup_error="当前会话仍有运行组件，请检查 final_health。")
            except Exception as exc:
                report.update(ok=False, cleanup_error=str(exc))
        signal.signal(signal.SIGTERM, old_term)
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        emit(report)
    return 0 if report.get("ok") else 1


def desktop(arguments, fixture=None):
    require_graphics()
    try:
        from PyQt5 import QtCore
        from .app import create_application
    except ImportError as exc:
        raise RuntimeError("缺少系统 PyQt5，无法打开桌面界面：" + str(exc))
    storage = new_storage(arguments, "gui-check") if arguments.gui_check else (
        Storage(arguments.data_dir) if arguments.data_dir else Storage())
    if arguments.gui_check:
        QtCore.QSettings.setDefaultFormat(QtCore.QSettings.IniFormat)
        QtCore.QSettings.setPath(QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope,
                                str(storage.base / "settings"))
    application, window = create_application(ROOT, fixture=fixture, storage=storage)
    if arguments.gui_check:
        window.autostart.blockSignals(True)
        window.autostart.setChecked(False)
        window.autostart.blockSignals(False)
    window.show()
    if arguments.profile and not window.select_profile(arguments.profile):
        raise RuntimeError("没有找到教学包：" + arguments.profile)
    if arguments.lesson:
        window.select_lesson(arguments.lesson, profile=arguments.profile)
    elif fixture is not None:
        window.select_lesson(fixture.id)
    if not arguments.gui_check:
        return application.exec_()

    screenshot = Path(arguments.screenshot).expanduser().resolve() if arguments.screenshot else storage.base / "gui-check.png"
    screenshot.parent.mkdir(parents=True, exist_ok=True)
    report = {"mode": "gui-check", "ok": False, "screenshot": str(screenshot),
              "qt_version": QtCore.QT_VERSION_STR, "data_dir": str(storage.base),
              "display": os.environ.get("DISPLAY", ""),
              "platform": os.environ.get("QT_QPA_PLATFORM", "native")}
    deadline = time.monotonic() + 18
    def capture():
        try:
            application.processEvents()
            visible = window.isVisible()
            dialog = window.environment_dialog
            popup_visible = bool(dialog and dialog.isVisible())
            popup_screenshot = screenshot.with_name(screenshot.stem + "-environment.png")
            popup_saved = bool(popup_visible and dialog.grab().save(str(popup_screenshot), "PNG"))
            if popup_visible:
                dialog.accept()
            saved = window.grab().save(str(screenshot), "PNG")
            report.update(ok=bool(visible and saved and popup_saved and window.environment is not None),
                          visible=visible, screenshot_saved=saved, courses=len(window.lessons),
                          environment_popup_visible=popup_visible, environment_popup_screenshot=str(popup_screenshot),
                          compatibility_counts=(window.environment_report or {}).get("counts"),
                          environment=window.environment, window_title=window.windowTitle(),
                          document_characters=len(window.document.toPlainText()))
            if window.environment is None:
                report["error"] = "环境检测未能在图形检查期限内完成。"
        except Exception as exc:
            report["error"] = str(exc)
        finally:
            emit(report)
            window.close()
            application.quit()
    def wait_ready():
        if window.environment is not None:
            QtCore.QTimer.singleShot(300, capture)
        elif time.monotonic() >= deadline:
            capture()
        else:
            QtCore.QTimer.singleShot(200, wait_ready)
    QtCore.QTimer.singleShot(700, wait_ready)
    application.exec_()
    return 0 if report.get("ok") else 1


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        if arguments.seconds < 0:
            raise RuntimeError("--seconds 不能小于零。")
        if arguments.diagnose:
            emit(detect_environment())
            return 0
        if arguments.list:
            environment = detect_environment()
            lessons, errors = load_catalog(ROOT)
            emit({"lessons": [{"id": x.id, "title": x.title, "module": x.module,
                                "profile": x.profile, "requires_ros": x.requires_ros,
                                "run_condition": compatibility(x, environment, gui=False, terminals=False)}
                               for x in lessons], "errors": errors})
            return 0 if not errors else 1
        fixture = fixture_lesson(arguments.smoke_fixture) if arguments.smoke_fixture else None
        if arguments.smoke or (fixture is not None and not arguments.gui_check):
            return smoke(arguments, detect_environment(), fixture=fixture)
        return desktop(arguments, fixture=fixture)
    except KeyboardInterrupt:
        print("操作已取消。", file=sys.stderr)
        return 130
    except Exception as exc:
        emit({"ok": False, "error": "{}: {}".format(type(exc).__name__, exc)})
        return 1


if __name__ == "__main__":
    sys.exit(main())
