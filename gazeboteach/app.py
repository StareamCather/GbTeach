"""Qt 5.12 native teaching workbench; expensive work runs outside the GUI thread."""

import html
import json
import os
import shutil
import subprocess
import threading
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from . import __version__
from .catalog import load_catalog
from .compatibility import build_compatibility_report, lesson_compatibility
from .backends import get_backend
from .environment import detect_environment
from .environment_dialog import EnvironmentDialog
from .markdown import render
from .modern import MODERN_PROFILES, is_modern, modern_capability
from .reader import DocumentRegistry, MarkdownReader
from .storage import Storage


class TaskSignals(QtCore.QObject):
    completed = QtCore.pyqtSignal(int, object)
    failed = QtCore.pyqtSignal(int, str)
    log = QtCore.pyqtSignal(str)
    closed = QtCore.pyqtSignal()


class MainWindow(QtWidgets.QMainWindow):
    """One active lesson session. Source templates are never edited by this window."""

    def __init__(self, root, fixture=None, storage=None):
        super().__init__()
        self.root = Path(root).resolve()
        self.storage = storage or Storage()
        self.lessons, self.catalog_errors = load_catalog(self.root)
        if fixture is not None:
            self.lessons = [fixture]
            self.catalog_errors = []
        self.document_registry = DocumentRegistry(self.root, self.lessons)
        self.prerequisite_reader = None
        self.environment = None
        self.environment_report = None
        self.environment_dialog = None
        self._environment_dialog_pending = True
        self._environment_detection_active = False
        self.session = None
        self.current_lesson = None
        self._jobs = {}
        self._next_job = 0
        self._closing = False
        self._allow_close = False
        self._last_health_error = None
        self._last_verification = None
        self._profile_initialized = False
        self.settings = QtCore.QSettings("GazeboTeach", "Desktop")
        self.signals = TaskSignals(self)
        self.signals.completed.connect(self._task_completed)
        self.signals.failed.connect(self._task_failed)
        self.signals.log.connect(self._append_log)
        self.signals.closed.connect(self._finish_close)
        self.setWindowTitle("GazeboTeach · 项目式仿真教学")
        self.setObjectName("GazeboTeachWindow")
        self.resize(1440, 950)
        self.setMinimumSize(1040, 720)
        self._build_ui()
        self._populate_tree()
        self._show_welcome()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._poll_session)
        self.timer.start(900)
        self.detect_runtime_environment()

    @property
    def busy(self):
        return bool(self._jobs)

    def _build_ui(self):
        self.buttons = {}
        self.setStyleSheet("""
            QMainWindow {background:#f3f6fb;}
            QWidget {font-family:'Noto Sans CJK SC','Microsoft YaHei',sans-serif;font-size:13px;color:#22364d;}
            QFrame#header {background:#102f50;border-radius:10px;}
            QLabel#brand {color:white;font-size:28px;font-weight:700;}
            QLabel#tagline {color:#bed2e7;font-size:13px;}
            QLabel#environmentBadge {color:#d8e9f7;background:#21486d;border-radius:6px;padding:9px;}
            QFrame#navigation {background:white;border:1px solid #dce5ef;border-radius:8px;}
            QLabel#sectionTitle {font-size:18px;font-weight:600;color:#173c60;}
            QLabel#courseTitle {font-size:25px;font-weight:700;color:#123653;}
            QLabel#courseSummary {color:#61728a;font-size:13px;}
            QLabel#sessionStatus {background:#e9f1fa;color:#234d70;border-radius:5px;padding:9px;}
            QPushButton {background:white;border:1px solid #cad8e6;border-radius:5px;padding:8px 12px;}
            QPushButton:hover {background:#eaf3fc;border-color:#88b0d3;}
            QPushButton:disabled {color:#a2acb7;background:#f0f3f6;border-color:#e2e7ed;}
            QPushButton#start_button {background:#156ca5;color:white;border:0;font-weight:600;}
            QPushButton#start_button:hover {background:#0c598d;}
            QPushButton#start_button:disabled {background:#d8e2ed;color:#8798a9;border:1px solid #cbd7e3;}
            QPushButton#stop_button {color:#a33c32;}
            QPushButton#verify_button {color:#13734e;border-color:#9bcab5;}
            QLineEdit {padding:8px;background:#f6f9fc;border:1px solid #d5e0ea;border-radius:5px;}
            QTreeWidget {border:0;background:white;font-size:13px;outline:0;}
            QTreeWidget::item {padding:7px 3px;}
            QTreeWidget::item:selected {background:#deecf9;color:#123b62;}
            QTabWidget::pane {border:1px solid #dbe4ef;background:white;}
            QTabBar::tab {padding:10px 18px;background:#edf2f8;border:0;}
            QTabBar::tab:selected {background:white;color:#12659b;border-top:3px solid #2682b8;}
            QTextBrowser {background:white;border:0;}
            QPlainTextEdit {background:#14273c;color:#d6e4f2;border:0;padding:12px;font-family:'DejaVu Sans Mono',monospace;font-size:12px;}
            QScrollArea {background:white;border:0;}
            QStatusBar {background:#e9f0f7;color:#536b82;}
        """)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(20, 16, 20, 12)
        outer.setSpacing(14)
        header = QtWidgets.QFrame()
        header.setObjectName("header")
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(22, 16, 22, 16)
        brand_group = QtWidgets.QVBoxLayout()
        brand = QtWidgets.QLabel("GazeboTeach")
        brand.setObjectName("brand")
        brand_group.addWidget(brand)
        tagline = QtWidgets.QLabel("从一个小项目开始，理解真实仿真的每一步")
        tagline.setObjectName("tagline")
        brand_group.addWidget(tagline)
        header_layout.addLayout(brand_group)
        header_layout.addStretch()
        self.environment_badge = QtWidgets.QLabel("正在识别 Gazebo / ROS 环境…")
        self.environment_badge.setObjectName("environmentBadge")
        self.environment_badge.setWordWrap(True)
        self.environment_badge.setMaximumWidth(620)
        header_layout.addWidget(self.environment_badge)
        self.environment_button = QtWidgets.QPushButton("环境检测结果")
        self.environment_button.setObjectName("environment_report_button")
        self.environment_button.clicked.connect(self.show_environment_report)
        self.environment_button.setEnabled(False)
        header_layout.addWidget(self.environment_button)
        outer.addWidget(header)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        outer.addWidget(splitter, 1)
        navigation = QtWidgets.QFrame()
        navigation.setObjectName("navigation")
        nav_layout = QtWidgets.QVBoxLayout(navigation)
        nav_layout.setContentsMargins(14, 16, 14, 14)
        title = QtWidgets.QLabel("课程地图")
        title.setObjectName("sectionTitle")
        nav_layout.addWidget(title)
        self.course_count = QtWidgets.QLabel("")
        nav_layout.addWidget(self.course_count)
        nav_layout.addWidget(QtWidgets.QLabel("教学包 / 运行环境"))
        self.profile_selector = QtWidgets.QComboBox()
        self.profile_selector.setObjectName("profile_selector")
        profiles = sorted(set(lesson.profile for lesson in self.lessons)) or ["classic11-noetic"]
        for profile in profiles:
            config = MODERN_PROFILES.get(profile)
            label = ("Gazebo {} / ROS 2 {}".format(config["sim_name"], config["ros_distro"].capitalize())
                     if config else {"classic11-noetic": "Classic 11 / ROS Noetic"}.get(profile, profile))
            self.profile_selector.addItem(label, profile)
        self.profile_selector.setEnabled(len(profiles) > 1)
        self.profile_selector.setToolTip("按实际环境选择独立教学包；教材可阅读，实验依版本与本课依赖开放。")
        self.profile_selector.currentIndexChanged.connect(self._profile_changed)
        nav_layout.addWidget(self.profile_selector)
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("搜索课号、主题或项目…")
        self.search.textChanged.connect(self._filter_tree)
        nav_layout.addWidget(self.search)
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setObjectName("course_tree")
        self.tree.setHeaderHidden(True)
        self.tree.currentItemChanged.connect(self._selection_changed)
        nav_layout.addWidget(self.tree, 1)
        self.autostart = QtWidgets.QCheckBox("进入课程后自动开始实验")
        self.autostart.setChecked(self.settings.value("autostart", False, type=bool))
        self.autostart.toggled.connect(lambda value: self.settings.setValue("autostart", value))
        nav_layout.addWidget(self.autostart)
        self.progress_hint = QtWidgets.QLabel("○ 未开始   ◐ 已运行   ✓ 已验收")
        self.progress_hint.setStyleSheet("color:#73859a;font-size:11px;")
        nav_layout.addWidget(self.progress_hint)
        splitter.addWidget(navigation)

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(16, 0, 0, 0)
        self.course_title = QtWidgets.QLabel("欢迎来到 GazeboTeach")
        self.course_title.setObjectName("courseTitle")
        self.course_title.setWordWrap(True)
        title_row = QtWidgets.QHBoxLayout()
        title_row.addWidget(self.course_title, 1)
        reference_button = QtWidgets.QPushButton("查看参考答案")
        reference_button.setObjectName("reference_button")
        reference_button.clicked.connect(self.show_reference)
        reference_button.setToolTip("只读查看独立答案；保留自己的练习文件。")
        self.buttons["reference"] = reference_button
        title_row.addWidget(reference_button)
        content_layout.addLayout(title_row)
        self.course_summary = QtWidgets.QLabel("先理解，再动手；观察现象，解释结果。")
        self.course_summary.setObjectName("courseSummary")
        self.course_summary.setWordWrap(True)
        content_layout.addWidget(self.course_summary)
        self.session_status = QtWidgets.QLabel("尚未开始实验")
        self.session_status.setObjectName("sessionStatus")
        self.session_status.setWordWrap(True)
        content_layout.addWidget(self.session_status)

        controls = QtWidgets.QGridLayout()
        definitions = [
            ("start", "开始实验", self.start_lesson), ("stop", "结束实验", self.stop_lesson),
            ("pause", "暂停", self.toggle_pause), ("step", "单步", self.step_lesson),
            ("reset", "重置模型状态", self.reset_models), ("restart", "重载项目", self.restart_lesson),
            ("gui", "重开 Gazebo 窗口", self.open_gui), ("terminal", "打开实验终端", self.open_terminal),
            ("edit", "编辑练习文件", self.edit_workspace), ("restore", "备份并恢复起点", self.restore_workspace),
            ("verify", "验收实验", self.verify_lesson), ("records", "查看实验记录", self.show_records),
        ]
        for index, (name, label, callback) in enumerate(definitions):
            button = QtWidgets.QPushButton(label)
            button.setObjectName(name + "_button")
            button.clicked.connect(callback)
            controls.addWidget(button, index // 6, index % 6)
            self.buttons[name] = button
        self.buttons["reset"].setToolTip("执行 Gazebo MODEL reset，保留仿真时钟；不保证任意插件私有状态或关节内部状态复位。\n"
                                           "需要完整重建本课实验时，请使用“重载项目”。")
        content_layout.addLayout(controls)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setObjectName("course_tabs")
        self.document = QtWidgets.QTextBrowser()
        self.document.setObjectName("lesson_document")
        self.document.setOpenLinks(False)
        self.document.setOpenExternalLinks(False)
        self.document.anchorClicked.connect(lambda url: self.open_document_link(url, self.document))
        self.tabs.addTab(self.document, "课程讲解")
        self.reference_view = QtWidgets.QTextBrowser()
        self.reference_view.setObjectName("reference_document")
        self.reference_view.setOpenLinks(False)
        self.reference_view.setOpenExternalLinks(False)
        self.reference_view.anchorClicked.connect(lambda url: self.open_document_link(url, self.reference_view))
        self.tabs.addTab(self.reference_view, "参考答案")
        self.commands_scroll = QtWidgets.QScrollArea()
        self.commands_scroll.setWidgetResizable(True)
        self.tabs.addTab(self.commands_scroll, "项目与命令")
        self.logs = QtWidgets.QPlainTextEdit()
        self.logs.setObjectName("runtime_logs")
        self.logs.setReadOnly(True)
        self.logs.setMaximumBlockCount(3500)
        self.tabs.addTab(self.logs, "运行日志")
        self.environment_view = QtWidgets.QPlainTextEdit()
        self.environment_view.setReadOnly(True)
        self.tabs.addTab(self.environment_view, "环境检测")
        self.records_view = QtWidgets.QTextBrowser()
        self.tabs.addTab(self.records_view, "实验记录")
        content_layout.addWidget(self.tabs, 1)
        splitter.addWidget(content)
        splitter.setSizes([300, 1120])
        splitter.setStretchFactor(1, 1)
        self.statusBar().showMessage("v{} · 课程与实验均在本机运行 · 原始模板与个人练习分别保存".format(__version__))
        self._refresh_buttons()

    def _populate_tree(self):
        chosen = (self.current_lesson.profile, self.current_lesson.id) if self.current_lesson else None
        active_profile = self.profile_selector.currentData()
        self.tree.blockSignals(True)
        self.tree.clear()
        self.lesson_items = {}
        groups = {}
        progress = self.storage.progress()
        for lesson in self.lessons:
            if lesson.profile != active_profile:
                continue
            group_key = (lesson.profile, lesson.module)
            if group_key not in groups:
                parent = QtWidgets.QTreeWidgetItem([lesson.module])
                parent.setFont(0, QtGui.QFont("", 12, QtGui.QFont.Bold))
                self.tree.addTopLevelItem(parent)
                parent.setExpanded(True)
                groups[group_key] = parent
            state = progress.get(lesson.profile + ":" + lesson.id, progress.get(lesson.id, set()))
            marker = "✓" if "verified" in state else "◐" if "started" in state else "○"
            item = QtWidgets.QTreeWidgetItem(["{}  {}  {}".format(marker, lesson.id, lesson.title)])
            item.setData(0, QtCore.Qt.UserRole, lesson)
            item.setToolTip(0, lesson.data.get("summary", ""))
            groups[group_key].addChild(item)
            self.lesson_items[(lesson.profile, lesson.id)] = item
            if chosen == (lesson.profile, lesson.id):
                self.tree.setCurrentItem(item)
        self.tree.blockSignals(False)
        count = sum(lesson.profile == active_profile for lesson in self.lessons)
        self.course_count.setText("{} 个项目 · 专业术语与实际操作对应".format(count))
        self._filter_tree(self.search.text())

    def _profile_changed(self, unused):
        if self._closing:
            return
        self._profile_initialized = True
        def activate(unused_result=None):
            self.current_lesson = None
            self.settings.setValue("last_profile", self.profile_selector.currentData())
            self._populate_tree()
            self._show_welcome()
            self.course_title.setText("欢迎来到 GazeboTeach")
            self.course_summary.setText("请从当前教学包选择一个项目。")
            self.session_status.setText("已选择教学包：" + str(self.profile_selector.currentData()))
            self._refresh_buttons()
        if self.session and not self.session.cancelled.is_set():
            self.run_task("结束上一运行环境", self.session.stop, activate)
        else:
            activate()

    def select_profile(self, profile):
        index = self.profile_selector.findData(profile)
        if index < 0:
            return False
        self._profile_initialized = True
        self.profile_selector.setCurrentIndex(index)
        return True

    def _filter_tree(self, text):
        query = text.strip().lower()
        for index in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(index)
            visible = False
            for child_index in range(parent.childCount()):
                child = parent.child(child_index)
                lesson = child.data(0, QtCore.Qt.UserRole)
                match = not query or query in (lesson.id + lesson.title + lesson.module + lesson.data.get("summary", "")).lower()
                child.setHidden(not match)
                visible = visible or match
            parent.setHidden(not visible)

    def _show_welcome(self):
        text = """# 让知识在真实仿真中发生

从左侧选择课程。每个项目围绕一个可观察问题，依次完成：

**理解原理 → 预测现象 → 运行实验 → 修改变量 → 查看数据 → 解释结果。**

## 如何开始

1. 阅读课程目标与必要背景。
2. 点击“开始实验”，应用准备个人练习并打开 Gazebo 与终端。
3. 在“项目与命令”查看文件位置和操作入口。
4. 修改练习、重载项目，再观察现象。
5. 完成独立挑战，并执行实验验收。

## 实验文件与状态

课程模板保持原样；自己的修改保存在独立练习目录。结束实验后仍可继续。
“重置模型状态”执行 MODEL reset 并保留仿真时钟；任意插件私有状态和关节内部状态不保证复位。
需要完整重建实验时使用“重载项目”；“备份并恢复起点”会先保存当前练习，再恢复课程模板。

## 专业知识，按需补充

正文保留准确的术语，跨领域背景配有简短说明与外部资料链接。
运行成功、阅读完成、实验验收分别记录。
"""
        if not self.lessons:
            text += "\n## 当前课包状态\n\n应用框架已就绪。正式课程将在逐课部署和验收后加入课程地图。\n"
        if self.catalog_errors:
            text += "\n## 课包检查信息\n\n" + "\n".join("- " + x for x in self.catalog_errors)
        self.document.setHtml(render(text))
        self.reference_view.setHtml(render("# 参考答案\n\n选择课程后，可查看该课单独提供的参考解答。参考解答只读展示，不替换自己的练习。"))
        self._build_commands(None)

    def _environment_ready(self, environment):
        if self.environment_dialog is not None and self.environment_dialog.isVisible():
            self.environment_dialog.reject()
        self._environment_detection_active = False
        self._environment_dialog_pending = True
        environment, self.docker_probe = environment
        self.environment = environment
        self.environment_report = build_compatibility_report(self.lessons, environment, self.catalog_errors)
        simulators = []
        for profile, config in MODERN_PROFILES.items():
            capability = modern_capability(environment, profile) or {}
            if capability.get("sim_major") == config["sim_major"]:
                simulators.append(config["sim_name"] + " " + capability.get("sim_version", ""))
        if environment.get("classic_version"):
            simulators.append("Classic " + environment["classic_version"])
        simulator = " / ".join(simulators) or "未检测到支持的版本"
        ros = ", ".join(environment.get("ros_installed", [])) or "未检测到 ROS"
        self.environment_badge.setText("{}\nGazebo {} · ROS {}".format(environment.get("os", ""), simulator, ros))
        self.environment_view.setPlainText(json.dumps({"detected": environment, "compatibility": self.environment_report,
                                                     "docker": self.docker_probe}, ensure_ascii=False, indent=2))
        self.environment_button.setEnabled(True)
        if not self._profile_initialized:
            self._profile_initialized = True
            preferred = self.settings.value("last_profile", environment.get("course_profile", "classic11-noetic"))
            ready = {}
            for course in self.environment_report["courses"]:
                ready[course["profile"]] = ready.get(course["profile"], 0) + int(course["compatible"])
            if ready and not ready.get(preferred, 0) and max(ready.values()) > 0:
                preferred = max(ready, key=ready.get)
            self.select_profile(preferred)
        self._refresh_buttons()
        if self.current_lesson:
            self._show_lesson_status()
        self.show_environment_report()

    def detect_runtime_environment(self):
        if self._environment_detection_active or self.busy:
            return
        self._environment_detection_active = True
        if self.environment_dialog is not None and self.environment_dialog.isVisible():
            self.environment_dialog.reject()
        self._environment_dialog_pending = True
        self.environment_button.setEnabled(False)
        def inspect():
            try:
                detected = detect_environment()
            except Exception as error:
                # A failed probe is not evidence of absence or compatibility.
                detected = {"incomplete_checks": ["classic_version", "profiles", "ros_installed", "tools",
                                                   "plugins", "ros_packages", "display"],
                            "detection_errors": ["环境检测失败：{}: {}".format(type(error).__name__, error)]}
            return detected, get_backend("docker").probe()
        self.run_task("检测环境", inspect, self._environment_ready)

    def show_environment_report(self):
        if self.environment_report is None or self._environment_detection_active:
            return
        if self.environment_dialog is not None and self.environment_dialog.isVisible():
            self.environment_dialog.raise_()
            return
        dialog = EnvironmentDialog(self.environment_report, self.docker_probe, self.root, self.settings, self)
        self.environment_dialog = dialog
        dialog.finished.connect(self._environment_report_closed)
        dialog.retry_button.clicked.connect(lambda: self._retry_environment(dialog))
        dialog.open()

    def _retry_environment(self, dialog):
        if self.busy or self._environment_detection_active:
            return
        self.detect_runtime_environment()

    def _environment_report_closed(self, result):
        if self._environment_detection_active:
            return
        was_pending = self._environment_dialog_pending
        self._environment_dialog_pending = False
        self._refresh_buttons()
        if (was_pending and result == QtWidgets.QDialog.Accepted and self.autostart.isChecked()
                and self.current_lesson and not self._lesson_problem(self.current_lesson)
                and not self.busy and not (self.session and self.session.running)):
            QtCore.QTimer.singleShot(0, self.start_lesson)

    def _lesson_problem(self, lesson):
        row = lesson_compatibility(lesson, self.environment, gui=True, terminals=True)
        return "；".join(reason["message"] for reason in row["reasons"])

    def select_lesson(self, lesson_id, profile=None):
        active_profile = profile or self.profile_selector.currentData()
        if profile and not self.select_profile(profile):
            return False
        for (profile, key), item in self.lesson_items.items():
            if profile == active_profile and key == str(lesson_id).zfill(2):
                self.tree.setCurrentItem(item)
                return True
        return False

    def _selection_changed(self, item, previous):
        if not item or self._closing:
            return
        lesson = item.data(0, QtCore.Qt.UserRole)
        if lesson is None:
            return
        if self.session and not self.session.cancelled.is_set() and (
                self.session.lesson.profile, self.session.lesson.id) != (lesson.profile, lesson.id):
            self.run_task("结束上一课", self.session.stop, lambda unused: self._display_lesson(lesson))
        else:
            self._display_lesson(lesson)

    def _display_lesson(self, lesson):
        self.current_lesson = lesson
        self.course_title.setText(lesson.id + "  " + lesson.title)
        self.course_summary.setText(lesson.data.get("summary", ""))
        path = lesson.file(lesson.data.get("lesson", "lesson.md"))
        self.document.setProperty("markdown_path", str(path))
        self.document.document().setBaseUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self.document.setHtml(render(lesson.document))
        self.reference_view.setHtml(render("# 参考答案\n\n点击右上角“查看参考答案”加载本课独立解答。自己的练习文件保持不变。"))
        self.tabs.setCurrentWidget(self.document)
        self.storage.record(lesson.id, "read", profile=lesson.profile)
        self._build_commands(lesson)
        self._show_lesson_status()
        self._refresh_buttons()
        self.settings.setValue("last_lesson", lesson.id)
        if (self.autostart.isChecked() and self.environment and not self.busy
                and not self._environment_dialog_pending and not self._lesson_problem(lesson)):
            QtCore.QTimer.singleShot(0, self.start_lesson)

    def open_document_link(self, url, source=None):
        """Reading prerequisites must never invoke selection, storage or runtime."""
        if url.scheme().lower() in ("http", "https"):
            QtGui.QDesktopServices.openUrl(url)
            return
        source = source or self.document
        current = source.property("markdown_path")
        if not current:
            self.statusBar().showMessage("当前页面没有已登记的课程文档路径。")
            return
        if not url.scheme() and not url.path() and url.hasFragment():
            source.scrollToAnchor(url.fragment())
            return
        reader = MarkdownReader(self.document_registry, self)
        try:
            reader.display(Path(current))
            reader.open_link(url)
            if reader.current_path == Path(current) and reader.location.text().startswith("链接不可打开"):
                self.statusBar().showMessage(reader.location.text())
                reader.deleteLater()
                return
        except (OSError, ValueError) as error:
            self.statusBar().showMessage("链接不可打开：" + str(error))
            reader.deleteLater()
            return
        if self.prerequisite_reader:
            self.prerequisite_reader.close()
            self.prerequisite_reader.deleteLater()
        self.prerequisite_reader = reader
        reader.show()
        reader.raise_()

    def _show_lesson_status(self):
        if not self.current_lesson:
            return
        if self.environment:
            problem = self._lesson_problem(self.current_lesson)
            if problem:
                self.session_status.setText("运行条件：" + problem + " 教材仍可阅读。")
                return
        if self.session and (self.session.lesson.profile, self.session.lesson.id) == (
                self.current_lesson.profile, self.current_lesson.id):
            verification = ""
            if self._last_verification and self._last_verification[0] == self.session.token:
                passed = self._last_verification[1].get("passed")
                verification = " · 上次验收通过" if passed is True else " · 上次验收未通过" if passed is False else " · 本课含人工观察检查"
            diagnosis = getattr(self.session, "native_control_diagnosis", {})
            backend = diagnosis.get("backend", "未检测")
            self.session_status.setText("{}{} · 控制：{} · 练习：{}".format(self.session.state, verification, backend, self.session.workspace))
        else:
            target = self.storage.workspace(self.current_lesson)
            self.session_status.setText("尚未开始本课 · 练习目录：" + str(target))

    def _build_commands(self, lesson):
        container = QtWidgets.QWidget()
        container.setStyleSheet("background:white;")
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(22, 20, 22, 20)
        if lesson is None:
            layout.addWidget(QtWidgets.QLabel("选择课程后，这里会显示目标、文件位置和可执行命令。"))
        else:
            text = QtWidgets.QLabel("<b>学习目标</b><br>" + "<br>".join("• " + html.escape(x) for x in lesson.data.get("objectives", [])))
            text.setWordWrap(True)
            layout.addWidget(text)
            path = QtWidgets.QLabel("<b>练习目录</b><br>" + html.escape(str(self.storage.workspace(lesson))))
            path.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            path.setWordWrap(True)
            layout.addWidget(path)
            info = QtWidgets.QLabel("模板更新会在准备实验时检查；已有个人修改会保留，冲突会显示在运行记录中。")
            info.setWordWrap(True)
            layout.addWidget(info)
            for command in lesson.data.get("commands", []):
                group = QtWidgets.QGroupBox(command.get("title", "实验命令"))
                group_layout = QtWidgets.QVBoxLayout(group)
                code = QtWidgets.QLabel(command["command"])
                code.setTextFormat(QtCore.Qt.PlainText)
                code.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                code.setWordWrap(True)
                code.setStyleSheet("background:#edf3f9;padding:10px;font-family:monospace;color:#234567;")
                group_layout.addWidget(code)
                row = QtWidgets.QHBoxLayout()
                copy = QtWidgets.QPushButton("复制命令")
                copy.clicked.connect(lambda unused=False, value=command["command"]: QtWidgets.QApplication.clipboard().setText(value))
                execute = QtWidgets.QPushButton("在本课终端执行")
                execute.clicked.connect(lambda unused=False, item=command: self.run_course_command(item))
                row.addWidget(copy)
                row.addWidget(execute)
                row.addStretch()
                group_layout.addLayout(row)
                layout.addWidget(group)
            note = QtWidgets.QLabel("命令在本课独立环境和个人副本中执行。先阅读命令目的，再运行并解释观察结果。")
            note.setWordWrap(True)
            layout.addWidget(note)
        layout.addStretch()
        previous = self.commands_scroll.takeWidget()
        if previous:
            previous.deleteLater()
        self.commands_scroll.setWidget(container)

    def run_task(self, name, function, callback=None):
        if self._closing:
            return
        self._next_job += 1
        job_id = self._next_job
        self._jobs[job_id] = (name, callback)
        self.statusBar().showMessage(name + "…")
        self._refresh_buttons()
        def work():
            try:
                result = function()
                self.signals.completed.emit(job_id, result)
            except Exception as exc:
                self.signals.failed.emit(job_id, "{}: {}".format(type(exc).__name__, exc))
        threading.Thread(target=work, name="GazeboTeach-" + str(job_id), daemon=True).start()

    def _task_completed(self, job_id, result):
        name, callback = self._jobs.pop(job_id, ("操作", None))
        if self._closing:
            return
        if callback:
            callback(result)
        self._refresh_buttons()
        self._populate_tree()
        self._show_lesson_status()
        self.statusBar().showMessage(name + "完成")

    def _task_failed(self, job_id, error):
        name, callback = self._jobs.pop(job_id, ("操作", None))
        if self._closing:
            return
        self._append_log(name + "失败：" + error)
        self.session_status.setText(name + "失败：" + error)
        self.tabs.setCurrentWidget(self.logs)
        self._refresh_buttons()
        self.statusBar().showMessage(name + "失败，请查看日志")
        QtWidgets.QMessageBox.warning(self, name + "失败", error + "\n\n运行日志中保留了实际执行步骤。")

    def _append_log(self, message):
        self.logs.appendPlainText(str(message))

    def _refresh_buttons(self):
        busy = self.busy or self._closing
        selected = self.current_lesson is not None
        running = self.session is not None and self.session.running
        compatible = selected and self.environment is not None and not self._lesson_problem(self.current_lesson)
        compatible = compatible and not self._environment_dialog_pending
        for name, button in self.buttons.items():
            button.setEnabled(not busy and selected)
        self.buttons["start"].setEnabled(not busy and bool(compatible) and not running)
        for name in ("stop", "pause", "step", "reset", "gui", "terminal", "verify"):
            self.buttons[name].setEnabled(not busy and running)
        if selected and is_modern(self.current_lesson):
            self.buttons["reset"].setEnabled(False)
            self.buttons["reset"].setToolTip("当前 Gazebo Sim 不支持保留时钟的 model-only reset。使用重载项目恢复完整初始状态。")
        else:
            self.buttons["reset"].setToolTip("执行 Gazebo MODEL reset，保留仿真时钟；插件私有状态或关节内部状态可能需要重载项目。")
        self.buttons["restart"].setEnabled(not busy and bool(compatible))
        self.buttons["records"].setEnabled(not busy)
        self.buttons["reference"].setEnabled(not busy and selected and bool(self.current_lesson.data.get("reference")))
        self.buttons["pause"].setText("继续" if self.session and self.session.paused else "暂停")
        self.tree.setEnabled(not busy)
        self.profile_selector.setEnabled(not busy and self.profile_selector.count() > 1)
        self.environment_button.setEnabled(not busy and self.environment_report is not None
                                           and not self._environment_detection_active)

    def _start(self):
        lesson = self.current_lesson
        if self.session:
            self.session.stop(reason="切换或重载实验")
        if self._closing:
            return None
        session = get_backend("native").create_session(self.root, lesson, storage=self.storage, notify=self.signals.log.emit)
        self.session = session
        if self._closing:
            session.stop(reason="应用正在退出")
            return None
        session.start(gui=True, terminals=True)
        if hasattr(self.storage, "template_status"):
            session.log("模板状态：" + json.dumps(self.storage.template_status(lesson), ensure_ascii=False, default=str))
        self._last_health_error = None
        self._last_verification = None
        return session.health()

    def start_lesson(self):
        if (not self.current_lesson or self.busy or self._environment_dialog_pending
                or self.environment is None or self._lesson_problem(self.current_lesson)):
            return
        self.logs.clear()
        self.run_task("准备实验", self._start)

    def stop_lesson(self):
        if self.session and not self.busy:
            self.run_task("结束实验", self.session.stop)

    def restart_lesson(self):
        if (self.current_lesson and not self.busy and not self._environment_dialog_pending
                and self.environment is not None and not self._lesson_problem(self.current_lesson)):
            self.run_task("重载项目", self._start)

    def toggle_pause(self):
        if self.session and not self.busy:
            self.run_task("暂停/继续", lambda: self.session.pause(not self.session.paused))

    def step_lesson(self):
        if self.session and not self.busy:
            self.run_task("单步仿真", self.session.step)

    def reset_models(self):
        if self.session and not self.busy:
            self.run_task("重置模型状态", self.session.reset_models)

    def open_gui(self):
        if self.session and not self.busy:
            self.run_task("打开 Gazebo 窗口", self.session.open_gui)

    def open_terminal(self):
        if self.session and not self.busy:
            self.run_task("打开实验终端", self.session.open_terminal)

    def run_course_command(self, command):
        if self.busy:
            return
        if not self.session or not self.session.running or (self.session.lesson.profile, self.session.lesson.id) != (
                self.current_lesson.profile, self.current_lesson.id):
            QtWidgets.QMessageBox.information(self, "请先开始实验", "开始本课实验后，命令才能在正确的环境中运行。")
            return
        self.run_task("打开命令终端", lambda: self.session.open_terminal(command=command["command"], title=command.get("title")))

    def edit_workspace(self):
        if not self.current_lesson or self.busy:
            return
        lesson = self.current_lesson
        # QDesktopServices is a UI API; run it in the GUI callback, not the worker.
        def prepared(workspace):
            path = workspace / lesson.data.get("world", "world.sdf")
            editor = shutil.which("gedit")
            if editor:
                subprocess.Popen([editor, "--new-window", str(path)], start_new_session=True,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(workspace)))
            self._build_commands(lesson)
        self.run_task("准备练习文件", lambda: self.storage.prepare(lesson), prepared)

    def show_reference(self):
        if not self.current_lesson or self.busy:
            return
        lesson = self.current_lesson
        relative = lesson.data.get("reference")
        if not relative:
            self.reference_view.setHtml(render("# 参考答案\n\n本课尚未提供单独参考文件，请按正文中的观察标准进行练习。"))
            self.tabs.setCurrentWidget(self.reference_view)
            return
        def read_reference():
            path = lesson.file(relative)
            if not path.is_file():
                raise RuntimeError("参考答案文件不存在：" + str(path))
            if path.stat().st_size > 2 * 1024 * 1024:
                raise RuntimeError("参考文件过大，请从课程目录中查看：" + str(path))
            text = path.read_text(encoding="utf-8-sig")
            if path.suffix.lower() not in (".md", ".markdown"):
                text = "# 参考文件：" + path.name + "\n\n```text\n" + text + "\n```"
            return text
        def display(text):
            path = lesson.file(relative)
            self.reference_view.setProperty("markdown_path", str(path))
            self.reference_view.document().setBaseUrl(QtCore.QUrl.fromLocalFile(str(path)))
            self.reference_view.setHtml(render(text))
            self.tabs.setCurrentWidget(self.reference_view)
            self.storage.record(lesson.id, "reference_viewed", profile=lesson.profile, reference=str(relative))
        self.run_task("读取参考答案", read_reference, display)

    def restore_workspace(self):
        if not self.current_lesson or self.busy:
            return
        answer = QtWidgets.QMessageBox.question(
            self, "备份并恢复课程起点", "当前练习将先完整保存到备份目录，再恢复课程模板。\n是否继续？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        lesson = self.current_lesson
        def restore():
            if self.session:
                self.session.stop(reason="备份并恢复课程起点")
            return self.storage.restore(lesson)
        def restored(result):
            workspace, backup = result
            self._append_log("已恢复练习：{}\n原练习备份：{}".format(workspace, backup or "首次创建，无旧练习"))
            self._build_commands(lesson)
        self.run_task("备份并恢复", restore, restored)

    def verify_lesson(self):
        if self.session and not self.busy:
            def verified(result):
                self._last_verification = (self.session.token, result)
                self.tabs.setCurrentWidget(self.logs)
                if result.get("passed") is True:
                    self.session_status.setText("实验验收通过 · 详细观测值见运行日志")
                elif result.get("passed") is False:
                    self.session_status.setText("实验验收未通过 · 请根据日志中的观测值排查")
                else:
                    self.session_status.setText(result.get("message", "请完成本课人工观察检查"))
            self.run_task("验收实验", self.session.verify, verified)

    def show_records(self):
        lines = ["# 实验记录", "", "阅读、运行与验收分别保存。"]
        for record in reversed(self.storage.records(150)):
            lines += ["", "## {} · 课程 {} · {}".format(record.get("time", ""), record.get("lesson", ""), record.get("event", "")),
                      "```json", json.dumps(record.get("details", {}), ensure_ascii=False, indent=2), "```"]
        self.records_view.setHtml(render("\n".join(lines)))
        self.tabs.setCurrentWidget(self.records_view)

    def _poll_session(self):
        if self._closing or not self.session:
            return
        try:
            with self.session.log_path.open("rb") as handle:
                handle.seek(0, os.SEEK_END)
                length = handle.tell()
                handle.seek(max(0, length - 110000))
                text = handle.read().decode("utf-8", "replace")
            if text != self.logs.toPlainText():
                scroll = self.logs.verticalScrollBar()
                follow = scroll.value() >= scroll.maximum() - 4
                self.logs.setPlainText(text)
                if follow:
                    scroll.setValue(scroll.maximum())
            if not self.busy and not self.session.cancelled.is_set():
                health = self.session.health()
                failed = health.get("failed_processes", [])
                if failed:
                    message = "核心进程退出：" + ", ".join(item["name"] for item in failed)
                    self.session_status.setText(message + "。可查看日志并重载项目。")
                    self._refresh_buttons()
                elif self.session.running and not health.get("gui_running", False):
                    self.session_status.setText("服务端运行中，Gazebo 图形客户端未运行；可点击“重开 Gazebo 窗口”。")
        except (OSError, ValueError):
            pass

    def closeEvent(self, event):
        if self._allow_close or (not self.session and not self.busy):
            event.accept()
            return
        event.ignore()
        if self._closing:
            return
        self._closing = True
        self.timer.stop()
        self._refresh_buttons()
        self.statusBar().showMessage("正在结束当前实验，保留练习与记录…")
        def shutdown():
            try:
                if self.session:
                    self.session.stop(reason="关闭桌面应用")
            finally:
                self.signals.closed.emit()
        threading.Thread(target=shutdown, name="GazeboTeach-shutdown", daemon=True).start()

    def _finish_close(self):
        self._allow_close = True
        self.close()


def create_application(root, fixture=None, storage=None):
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    application.setApplicationName("GazeboTeach")
    application.setOrganizationName("GazeboTeach")
    application.setApplicationVersion(__version__)
    application.setFont(QtGui.QFont("Noto Sans CJK SC", 11))
    window = MainWindow(root, fixture=fixture, storage=storage)
    return application, window
