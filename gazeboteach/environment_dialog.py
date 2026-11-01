"""Startup compatibility report and a deliberately non-executing Docker seam."""

import json

from PyQt5 import QtCore, QtWidgets

from .backends import DockerConnectionConfig, backend_descriptors, get_backend
from .markdown import render


STATUS_LABELS = {"ready": "满足已声明依赖", "unsupported": "版本不匹配",
                 "missing_dependencies": "缺少依赖", "incomplete_detection": "检测未完成"}


class EnvironmentDialog(QtWidgets.QDialog):
    def __init__(self, report, docker_probe, root, settings, parent=None):
        super().__init__(parent)
        self.report, self.settings = report, settings
        self.setObjectName("environment_report_dialog")
        self.setWindowTitle("GazeboTeach · 启动环境检测")
        self.setWindowModality(QtCore.Qt.WindowModal)
        self.resize(1040, 740)
        layout = QtWidgets.QVBoxLayout(self)
        counts = report["counts"]
        label = QtWidgets.QLabel("{} 个项目：{} 个满足已声明依赖，{} 个需处理版本、依赖或检测问题。\n"
                                "所有教材均可阅读；满足依赖仍需在真实图形环境验证实验。".format(
                                    report["course_count"], counts["ready"],
                                    report["course_count"] - counts["ready"]))
        label.setWordWrap(True)
        label.setObjectName("environment_summary")
        layout.addWidget(label)
        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs, 1)
        courses = QtWidgets.QWidget()
        content = QtWidgets.QVBoxLayout(courses)
        environment = report["environment"]
        summary = QtWidgets.QLabel("{} · {} · Python {}\nClassic：{}；新版 Gazebo：{}\n"
                                  "ROS 已安装：{}；当前 shell 激活：{}；DISPLAY：{}\n"
                                  "V2 已验证基线：Ubuntu 20.04 / Classic 11.15.1 / ROS Noetic。".format(
                                      environment.get("os"), environment.get("architecture"), environment.get("python"),
                                      environment.get("classic_version") or "未检测到",
                                      json.dumps(environment.get("modern_simulators") or [], ensure_ascii=False),
                                      ", ".join(environment.get("ros_installed") or []) or "未检测到",
                                      environment.get("ros_active") or "未激活", environment.get("display") or "无"))
        summary.setWordWrap(True)
        content.addWidget(summary)
        installed = QtWidgets.QLabel("当前 Classic 可执行文件：{}\n已确认的 Classic 安装：{}".format(
            environment.get("classic_active_executable") or "无",
            "；".join("{} ({})".format(item["version"], item["path"])
                     for item in environment.get("classic_installations") or []) or "未检测到"))
        installed.setWordWrap(True)
        content.addWidget(installed)
        sdk = environment.get("pkg_config") or {}
        moc = environment.get("qt_moc") or {}
        capability = environment.get("control_capability") or {}
        development = QtWidgets.QLabel("Gazebo SDK：{}；头文件编译检查：{}；Qt5 moc：{} {}\n"
                                      "控制能力：{}。{}".format(
                                          sdk.get("gazebo", {}).get("version") or "未检测到 gazebo.pc",
                                          "通过" if (environment.get("sdk") or {}).get("headers_available") else "未通过",
                                          moc.get("path") or "未检测到", moc.get("version") or "",
                                          capability.get("preferred_backend") or "未检测", capability.get("reason") or
                                          "按实际会话构建结果记录后端；环境探测不启动实验。"))
        development.setObjectName("environment_sdk_summary")
        development.setWordWrap(True)
        content.addWidget(development)
        self.table = QtWidgets.QTableWidget(len(report["courses"]), 4)
        self.table.setObjectName("environment_course_table")
        self.table.setHorizontalHeaderLabels(["编号", "课程", "实验状态", "原因 / 下一步"])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        for row, lesson in enumerate(report["courses"]):
            reason = "；".join(item["message"] for item in lesson["reasons"])
            values = [lesson["id"], lesson["title"], STATUS_LABELS[lesson["status"]], reason or "可开始实验并观察真实结果"]
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setToolTip(value)
                self.table.setItem(row, column, item)
        header = self.table.horizontalHeader()
        for column in (0, 2):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Interactive)
        self.table.setColumnWidth(1, 240)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        self.table.resizeRowsToContents()
        content.addWidget(self.table, 1)
        if report["detection"]["errors"] or report["catalog_errors"]:
            errors = QtWidgets.QLabel("检查详情：" + "；".join(str(item) for item in
                                      report["detection"]["errors"] + report["catalog_errors"]))
            errors.setWordWrap(True)
            content.addWidget(errors)
        tabs.addTab(courses, "课程兼容情况")
        docker = QtWidgets.QWidget()
        docker_layout = QtWidgets.QVBoxLayout(docker)
        explanation = QtWidgets.QLabel("Docker 连接接口已预留。当前版本可校验并保存下面的配置，尚未实现容器连接或课程执行。\n"
                                      "需要容器时，请按安装指南手动在兼容容器内启动应用，再由容器内检测环境。")
        explanation.setWordWrap(True)
        docker_layout.addWidget(explanation)
        probe = QtWidgets.QLabel("Docker CLI：{}\n仅检测客户端版本，未连接 daemon、拉取镜像或创建容器。".format(
                                 docker_probe["cli"]["version"] or docker_probe["cli"]["error"] or "未安装"))
        probe.setWordWrap(True)
        docker_layout.addWidget(probe)
        self.docker_configuration = QtWidgets.QPlainTextEdit()
        self.docker_configuration.setObjectName("docker_configuration")
        default = DockerConnectionConfig(image="osrf/ros:noetic-desktop-full", host_workspace=str(root)).to_dict()
        saved = settings.value("docker_configuration", "")
        self.docker_configuration.setPlainText(saved if isinstance(saved, str) and saved else json.dumps(default, indent=2))
        docker_layout.addWidget(self.docker_configuration, 1)
        save = QtWidgets.QPushButton("校验并保存配置（不连接）")
        save.setObjectName("save_docker_configuration")
        save.clicked.connect(self._save_docker)
        docker_layout.addWidget(save)
        self.docker_status = QtWidgets.QLabel("配置与本机课程执行独立；当前执行后端：native。")
        self.docker_status.setWordWrap(True)
        docker_layout.addWidget(self.docker_status)
        tabs.addTab(docker, "Docker 接口（预留）")
        guide = QtWidgets.QTextBrowser()
        guide.setOpenExternalLinks(True)
        path = root / "docs/V2_INSTALL.md"
        guide.setHtml(render(path.read_text(encoding="utf-8") if path.is_file() else
                             "# V2 安装指南\n\n请阅读发行包内 docs/V2_INSTALL.md。"))
        tabs.addTab(guide, "V2 安装指南")
        self.backends = backend_descriptors()
        buttons = QtWidgets.QDialogButtonBox()
        self.retry_button = buttons.addButton("重新检测", QtWidgets.QDialogButtonBox.ActionRole)
        close = buttons.addButton("继续学习", QtWidgets.QDialogButtonBox.AcceptRole)
        close.setObjectName("continue_learning")
        close.clicked.connect(self.accept)
        layout.addWidget(buttons)

    def _save_docker(self):
        try:
            configuration = get_backend("docker").validate_configuration(json.loads(self.docker_configuration.toPlainText()))
            text = json.dumps(configuration.to_dict(), ensure_ascii=False, indent=2)
            self.settings.setValue("docker_configuration", text)
            self.docker_configuration.setPlainText(text)
            self.docker_status.setText("配置校验并保存成功；尚未连接 Docker，也没有启动容器或实验。")
        except (ValueError, TypeError) as error:
            self.docker_status.setText("配置未保存：" + str(error))
