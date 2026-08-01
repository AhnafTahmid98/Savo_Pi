#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -euo pipefail

WORKSPACE="${SAVO_WORKSPACE:-$HOME/Savo_Pi/savo_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"

exec ros2 launch savo_supervisor supervisor.launch.py \
  system_state_path:="${SAVO_SUPERVISOR_STATE_PATH:-/var/lib/robot_savo/supervisor/system_state.json}" \
  auto_arm:="${SAVO_SUPERVISOR_AUTO_ARM:-false}"
