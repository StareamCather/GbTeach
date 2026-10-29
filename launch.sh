#!/bin/bash
# GazeboTeach launcher: inherit the actual desktop session; never guess DISPLAY.
set -e
APP_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
cd "$APP_ROOT"

if [ ! -x /usr/bin/python3 ]; then
    printf '%s\n' 'GazeboTeach：缺少系统 Python 3（/usr/bin/python3）。'
    exit 2
fi

if [ "$#" -eq 0 ] || [ "${1:-}" = "--gui-check" ]; then
    if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
        printf '%s\n' 'GazeboTeach：当前不是图形桌面会话。请在 Ubuntu 桌面的终端中运行 ./launch.sh。' '可使用 ./launch.sh --diagnose 检查环境，或 --list 查看已部署课程。'
        exit 2
    fi
    if ! /usr/bin/python3 -c 'from PyQt5 import QtCore, QtWidgets' >/dev/null 2>&1; then
        printf '%s\n' 'GazeboTeach：缺少系统 PyQt5。请根据部署说明准备 python3-pyqt5；启动器不会自动下载或修改系统。'
        exit 2
    fi
fi

exec /usr/bin/python3 -m gazeboteach "$@"
