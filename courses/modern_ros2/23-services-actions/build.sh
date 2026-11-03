#!/usr/bin/env bash
set -euo pipefail
cd ros_ws
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --executor sequential --packages-select gazeboteach_rail_interfaces
