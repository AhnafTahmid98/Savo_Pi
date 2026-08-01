#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -euo pipefail

WORKSPACE="${SAVO_WORKSPACE:-$HOME/Savo_Pi/savo_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REASON="${1:-operator_requested_shutdown}"
REQUEST_ID="shutdown-$(date +%s)"
ACTOR_ID="${SAVO_SHUTDOWN_ACTOR:-system_operator}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

REQUEST_PAYLOAD="{command: 3, request_id: '${REQUEST_ID}', actor_id: '${ACTOR_ID}', reason: '${REASON}', expected_generation: 0}"

ros2 service call \
  /savo_supervisor/manage_system_state \
  savo_msgs/srv/ManageSystemState \
  "$REQUEST_PAYLOAD"
