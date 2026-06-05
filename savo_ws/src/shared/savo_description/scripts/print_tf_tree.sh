#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-/tmp/robot_savo_tf}"
OUTPUT_FILE="${OUTPUT_DIR}/frames"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source your ROS 2 environment first."
  exit 1
fi

if ! ros2 run tf2_tools view_frames --help >/dev/null 2>&1; then
  echo "tf2_tools view_frames not available. Install ros-jazzy-tf2-tools."
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

pushd "${OUTPUT_DIR}" >/dev/null
ros2 run tf2_tools view_frames
popd >/dev/null

if [[ -f "${OUTPUT_DIR}/frames.pdf" ]]; then
  echo "TF tree saved: ${OUTPUT_DIR}/frames.pdf"
else
  echo "TF tree command finished, but frames.pdf was not found."
  exit 1
fi