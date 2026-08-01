#!/usr/bin/env bash
# Run the production Robot Savo edge bringup.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_edge.sh
source "${SCRIPT_DIR}/env_edge.sh"

ROBOT_MODE="${SAVO_ROBOT_MODE:-safe_idle}"
BRINGUP_PROFILE="${SAVO_BRINGUP_PROFILE:-lidar_only}"

main() {
  savo_assert_edge_host
  savo_require_cmd ros2
  savo_require_dir "${SAVO_WS}"
  savo_source_ros
  savo_source_ws

  export ROS_LOG_DIR="${ROS_LOG_DIR:-${HOME}/.ros/log/robot_savo_edge}"
  mkdir -p "${ROS_LOG_DIR}"

  printf 'Robot Savo launch: role=edge mode=%s profile=%s startup=STOP\n' \
    "${ROBOT_MODE}" "${BRINGUP_PROFILE}"

  exec ros2 launch savo_bringup robot_bringup.launch.py \
    host_role:=edge \
    robot_mode:="${ROBOT_MODE}" \
    bringup_profile:="${BRINGUP_PROFILE}" \
    require_locked_geometry:="${SAVO_REQUIRE_LOCKED_GEOMETRY:-true}" \
    allow_provisional_geometry:="${SAVO_ALLOW_PROVISIONAL_GEOMETRY:-false}" \
    d435_voxel_validated:="${SAVO_D435_VOXEL_VALIDATED:-false}" \
    start_realsense:="${SAVO_START_REALSENSE:-true}" \
    start_vo:="${SAVO_START_VO:-true}" \
    start_obstacle_cloud:="${SAVO_START_OBSTACLE_CLOUD:-false}" \
    start_speech:="${SAVO_START_SPEECH:-false}" \
    start_ui:="${SAVO_START_UI:-false}" \
    start_bridge:="${SAVO_START_BRIDGE:-true}"
}

main "$@"
