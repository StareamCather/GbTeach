#!/usr/bin/env bash
set -euo pipefail
cd ros_ws
colcon build --executor sequential --packages-select gazeboteach_lifecycle
