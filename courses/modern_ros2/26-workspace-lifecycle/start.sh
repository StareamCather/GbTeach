#!/usr/bin/env bash
set -eo pipefail
. ros_ws/install/local_setup.bash
exec ros2 launch gazeboteach_lifecycle lab.launch.py
