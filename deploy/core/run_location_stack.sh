#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=env_core.sh
source "${SCRIPT_DIR}/env_core.sh"

savo_assert_core_host
savo_require_cmd ros2
savo_require_dir "${SAVO_WS}"

if [[ ! -f "${SAVO_WS}/install/setup.bash" ]]; then
  savo_die "Workspace setup missing: ${SAVO_WS}/install/setup.bash"
fi

savo_source_ros
savo_source_ws

export ROS_HOME="${ROS_HOME:-/var/lib/robot_savo/ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/var/log/robot_savo/location_stack}"
export RCUTILS_LOGGING_BUFFERED_STREAM="1"

LOCATION_STATE_ROOT="${SAVO_LOCATION_STATE_ROOT:-/var/lib/robot_savo/locations}"

mkdir -p   "${LOCATION_STATE_ROOT}"   "${LOCATION_STATE_ROOT}/backups"   "${ROS_HOME}"   "${ROS_LOG_DIR}"

chmod 0750   "${LOCATION_STATE_ROOT}"   "${LOCATION_STATE_ROOT}/backups"   "${ROS_HOME}"   "${ROS_LOG_DIR}"

if [[ ! -w "${LOCATION_STATE_ROOT}" ]]; then
  savo_die "Location state directory is not writable: ${LOCATION_STATE_ROOT}"
fi

START_LOCATIONS="${SAVO_LOCATION_START_LOCATIONS:-true}"
START_SUPERVISOR="${SAVO_LOCATION_START_SUPERVISOR:-true}"
START_HEAD_OBSERVER="${SAVO_LOCATION_START_HEAD_OBSERVER:-true}"
START_HEAD_ACTION="${SAVO_LOCATION_START_HEAD_ACTION:-true}"
START_REGISTRATION="${SAVO_LOCATION_START_REGISTRATION:-true}"
START_NAVIGATION="${SAVO_LOCATION_START_NAVIGATION:-true}"
LOG_LEVEL="${SAVO_LOCATION_LOG_LEVEL:-info}"

savo_log "Starting typed location integration layer"
savo_log "State root: ${LOCATION_STATE_ROOT}"
savo_log "ROS log dir: ${ROS_LOG_DIR}"

exec ros2 launch savo_bringup location_integration.launch.py   "start_locations:=${START_LOCATIONS}"   "start_supervisor:=${START_SUPERVISOR}"   "start_head_observer:=${START_HEAD_OBSERVER}"   "start_head_action:=${START_HEAD_ACTION}"   "start_registration:=${START_REGISTRATION}"   "start_navigation:=${START_NAVIGATION}"   "log_level:=${LOG_LEVEL}"
