#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="savo_description"
LAUNCH_FILE="display.launch.py"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS 2 environment first."
  exit 1
fi

if ! ros2 pkg prefix "${PACKAGE_NAME}" >/dev/null 2>&1; then
  echo "${PACKAGE_NAME} not found. Build the package and source install/setup.bash first."
  exit 1
fi

ros2 launch "${PACKAGE_NAME}" "${LAUNCH_FILE}"