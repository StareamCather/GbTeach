"""Explain native lesson availability using the runtime's compatibility predicate."""

from .catalog import compatibility


STATUSES = ("ready", "unsupported", "missing_dependencies", "incomplete_detection")
_COLLECTIONS = {"profiles", "ros_installed", "plugins", "ros_packages"}
_KNOWN_CHECKS = _COLLECTIONS | {"classic_version", "tools", "display", "modern_simulators", "ros_active",
                                 "os", "architecture", "python", "ros_families", "container_hint", "pkg_config",
                                 "sdk", "qt_moc", "libraries", "native_server", "ros_package_versions", "python_modules"}
_MAPPINGS = {"tools", "pkg_config", "sdk", "qt_moc", "libraries", "native_server", "ros_package_versions", "python_modules"}


def _valid_field(report, name):
    if name not in report:
        return False
    value = report[name]
    if name in _COLLECTIONS:
        return isinstance(value, list) and all(isinstance(item, str) for item in value)
    if name in _MAPPINGS:
        if not isinstance(value, dict):
            return False
        if name == "pkg_config":
            return all(isinstance(item, dict) and isinstance(item.get("version"), str)
                       and type(item.get("available")) is bool for item in value.values())
        if name == "sdk":
            return type(value.get("headers_available")) is bool
        if name in ("qt_moc", "native_server"):
            return type(value.get("available" if name == "qt_moc" else "validated")) is bool
        if name == "python_modules":
            return all(type(item) is bool for item in value.values())
        return all(item is None or isinstance(item, str) for item in value.values()) if name != "tools" else True
    return isinstance(value, str)


def _required_fields(lesson, gui, terminals):
    fields = {"classic_version"} if lesson.profile == "classic11-noetic" else {"profiles"}
    fields.add("tools")
    if lesson.requires_ros:
        fields.add("ros_installed")
    if lesson.data.get("requires_plugins"):
        fields.add("plugins")
    if lesson.data.get("requires_ros_packages"):
        fields.add("ros_packages")
    if lesson.data.get("requires_pkg_config"):
        fields.add("pkg_config")
        if "gazebo" in lesson.data["requires_pkg_config"]:
            fields.add("sdk")
    for requirement, field in (("requires_qt_moc", "qt_moc"), ("requires_libraries", "libraries"),
                               ("requires_native_server", "native_server"),
                               ("requires_ros_package_versions", "ros_package_versions"),
                               ("requires_python_modules", "python_modules")):
        if lesson.data.get(requirement):
            fields.add(field)
    if gui or terminals or lesson.data.get("needs_display", False):
        fields.add("display")
    return fields


def _incomplete_checks(report):
    value = report.get("incomplete_checks", [])
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        return {"invalid_detection_metadata"}
    return set(value)


def _missing_tool_checks(lesson, report, gui, terminals):
    """Do not let catalog's runtime PATH fallback fill holes in a report snapshot."""
    tools = report.get("tools")
    if not isinstance(tools, dict):
        return []  # The whole tools field is already reported as incomplete.
    required = {"gzserver", "gz", "bash"} | set(lesson.data.get("requires_tools", []))
    if gui:
        required.add("gzclient")
    if lesson.requires_ros:
        required.add("roscore")
    missing = ["tools:" + name for name in sorted(required)
               if name not in tools or not (tools[name] is None or isinstance(tools[name], str))]
    if terminals and not any(isinstance(tools.get(name), str) and tools[name]
                             for name in ("gnome-terminal", "xterm")):
        if not all(name in tools and (tools[name] is None or isinstance(tools[name], str))
                   for name in ("gnome-terminal", "xterm")):
            missing.append("tools:terminal")
    return missing


def _known_unsupported(lesson, report, incomplete):
    """Only classify a mismatch when its version/profile probe actually completed."""
    if lesson.profile == "classic11-noetic":
        if "classic_version" not in incomplete and _valid_field(report, "classic_version"):
            version = report["classic_version"]
            if (version and not version.startswith("11.")) or (
                    not version and report.get("modern_simulators") and "modern_simulators" not in incomplete):
                return "本课需要 Gazebo Classic 11，当前模拟器版本不匹配。"
        if (lesson.requires_ros and "ros_installed" not in incomplete
                and _valid_field(report, "ros_installed") and "noetic" not in report["ros_installed"]):
            return "本课需要 ROS Noetic；当前未安装该 ROS 1 发行版。"
    elif ("profiles" not in incomplete and _valid_field(report, "profiles")
          and lesson.profile not in report["profiles"]):
        return "当前环境没有与本课匹配的课程运行配置。"
    if "pkg_config" not in incomplete and _valid_field(report, "pkg_config"):
        for package, version in lesson.data.get("requires_pkg_config", {}).items():
            actual = report.get("pkg_config", {}).get(package, {}).get("version", "")
            if version and actual and actual != version:
                return "本课需要 {} SDK 精确版本 {}；检测到 {}。".format(package, version, actual)
    if "ros_package_versions" not in incomplete and _valid_field(report, "ros_package_versions"):
        for package, version in lesson.data.get("requires_ros_package_versions", {}).items():
            actual = report.get("ros_package_versions", {}).get(package, "")
            if actual and actual != version:
                return "本课需要 Noetic {} 精确版本 {}；检测到 {}。".format(package, version, actual)
    return ""


def lesson_compatibility(lesson, detected, *, gui=True, terminals=True):
    """Return one primary, actionable reason; this function never activates ROS."""
    incomplete = _incomplete_checks(detected)
    fields = _required_fields(lesson, gui, terminals)
    missing = sorted(name for name in fields if name in incomplete or not _valid_field(detected, name))
    mapping_requirements = (("requires_pkg_config", "pkg_config"),
                            ("requires_ros_package_versions", "ros_package_versions"),
                            ("requires_libraries", "libraries"), ("requires_python_modules", "python_modules"))
    for requirement, field in mapping_requirements:
        if _valid_field(detected, field):
            missing.extend(field + ":" + name for name in lesson.data.get(requirement, []) if name not in detected[field])
    missing.extend(_missing_tool_checks(lesson, detected, gui, terminals))
    if {name for name in incomplete - _KNOWN_CHECKS if not name.startswith("tools:")}:
        missing.append("detection_metadata")
    required_tool_checks = set(_missing_tool_checks(lesson, {"tools": {}}, gui, terminals))
    missing.extend(sorted(incomplete & required_tool_checks))
    missing = sorted(set(missing))
    unsupported = _known_unsupported(lesson, detected, incomplete)
    if missing and not unsupported:
        status = "incomplete_detection"
        reasons = [{"code": "incomplete_checks", "fields": missing,
                    "message": "环境检查尚未完成，不能确认本课可运行：" + ", ".join(missing)}]
    else:
        # This is the very same predicate used by LessonSession.start. Do not
        # infer Noetic support from the active shell's ROS_DISTRO or a gz name.
        reason = unsupported or compatibility(lesson, detected, gui=gui, terminals=terminals)
        status = "ready" if reason is None else ("unsupported" if unsupported else "missing_dependencies")
        reasons = [] if reason is None else [{"code": "unsupported_profile" if unsupported else "runtime_requirement",
                                             "message": reason}]
    return {"id": lesson.id, "key": lesson.key, "title": lesson.title,
            "profile": lesson.profile, "requires_ros": lesson.requires_ros,
            "status": status, "compatible": status == "ready", "reading_available": True,
            "reasons": reasons}


def build_compatibility_report(lessons, detected, catalog_errors=(), *, gui=True, terminals=True):
    """Build a JSON-ready report for all installed lessons, without side effects.

    Detectors may supply ``incomplete_checks`` containing existing result field
    names (for example ``ros_packages``), and ``detection_errors`` for detail.
    A failed unrelated probe does not disable an otherwise verified lesson.
    """
    rows = [lesson_compatibility(lesson, detected, gui=gui, terminals=terminals) for lesson in lessons]
    checks = sorted(_incomplete_checks(detected))
    errors = detected.get("detection_errors", [])
    if not isinstance(errors, list):
        errors = [str(errors)]
    missing_fields = sorted({field for row in rows for reason in row["reasons"]
                             for field in reason.get("fields", [])})
    return {"schema_version": 1, "backend": "native", "gui": gui, "terminals": terminals,
            "environment": {name: detected.get(name) for name in
                            ("os", "architecture", "python", "classic_version", "classic_active_executable", "classic_installations", "modern_simulators",
                             "ros_installed", "ros_active", "ros_active_version", "ros_families", "profiles", "display",
                             "pkg_config", "sdk", "qt_moc", "libraries", "native_server", "ros_package_versions", "control_capability")},
            "detection": {"complete": not checks and not missing_fields and not errors,
                          "incomplete_checks": sorted(set(checks + missing_fields)), "errors": list(errors)},
            "catalog_errors": list(catalog_errors), "courses": rows,
            "counts": {status: sum(row["status"] == status for row in rows) for status in STATUSES},
            "course_count": len(rows)}
