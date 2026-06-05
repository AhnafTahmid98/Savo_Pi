#!/usr/bin/env bash
set -euo pipefail

PACKAGE_NAME="savo_description"
XACRO_FILE="robot_savo.urdf.xacro"
OUTPUT_FILE="${1:-/tmp/robot_savo.urdf}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS 2 environment first."
  exit 1
fi

if ! command -v xacro >/dev/null 2>&1; then
  echo "xacro command not found. Install ros-jazzy-xacro or source your workspace."
  exit 1
fi

PACKAGE_SHARE="$(ros2 pkg prefix "${PACKAGE_NAME}")/share/${PACKAGE_NAME}"
XACRO_PATH="${PACKAGE_SHARE}/urdf/${XACRO_FILE}"

if [[ ! -f "${XACRO_PATH}" ]]; then
  echo "Xacro file not found: ${XACRO_PATH}"
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_FILE}")"

xacro "${XACRO_PATH}" > "${OUTPUT_FILE}"

echo "Generated URDF: ${OUTPUT_FILE}"