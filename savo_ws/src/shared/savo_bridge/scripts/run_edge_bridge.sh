#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
WORKSPACE="${SAVO_WORKSPACE:-/opt/robot_savo/savo_ws}"
CONFIG_FILE="${SAVO_BRIDGE_CONFIG:-${WORKSPACE}/install/savo_bridge/share/savo_bridge/config/savo_bridge.edge.yaml}"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

exec ros2 launch savo_bridge edge_bridge.launch.py \
  "config_file:=${CONFIG_FILE}"
