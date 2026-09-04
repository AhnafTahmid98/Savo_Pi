#!/usr/bin/env bash
# Run the production Robot Savo core bringup.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_core.sh
source "${SCRIPT_DIR}/env_core.sh"

ROBOT_MODE="${SAVO_ROBOT_MODE:-safe_idle}"
BRINGUP_PROFILE="${SAVO_BRINGUP_PROFILE:-lidar_only}"
MAP_ID="${SAVO_MAP_ID:-robot_savo_map}"
STATE_ROOT="${SAVO_STATE_ROOT:-/var/lib/robot_savo}"
LOG_ROOT="${SAVO_LOG_ROOT:-/var/log/robot_savo}"
MAP_SESSIONS_ROOT="${SAVO_MAP_SESSIONS_ROOT:-${STATE_ROOT}/maps/sessions}"
PRODUCTION_MAP_ROOT="${SAVO_PRODUCTION_MAP_ROOT:-${STATE_ROOT}/maps/production}"
LOCATION_STATE_ROOT="${SAVO_LOCATION_STATE_ROOT:-${STATE_ROOT}/locations}"
SUPERVISOR_STATE_ROOT="${SAVO_SUPERVISOR_STATE_ROOT:-${STATE_ROOT}/supervisor}"
ACTIVE_MAP_CONTRACT="${SAVO_ACTIVE_MAP_CONTRACT:-${PRODUCTION_MAP_ROOT}/active_map.yaml}"

main() {
  savo_assert_core_host
  savo_require_cmd ros2
  savo_require_dir "${SAVO_WS}"
  savo_source_ros
  savo_source_ws

  export ROS_HOME="${ROS_HOME:-${STATE_ROOT}/ros}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-${LOG_ROOT}/core}"

  for directory in \
    "${MAP_SESSIONS_ROOT}" \
    "${PRODUCTION_MAP_ROOT}" \
    "${STATE_ROOT}/maps/release_transactions" \
    "${LOCATION_STATE_ROOT}" \
    "${LOCATION_STATE_ROOT}/releases" \
    "${STATE_ROOT}/localization" \
    "${SUPERVISOR_STATE_ROOT}" \
    "${ROS_HOME}" \
    "${ROS_LOG_DIR}"; do
    [[ -d "${directory}" && -w "${directory}" ]] || \
      savo_die "Runtime directory missing or not writable: ${directory}. Run prepare_runtime_storage.sh with sudo."
  done

  printf 'Robot Savo launch: role=core mode=%s profile=%s startup=STOP\n' \
    "${ROBOT_MODE}" "${BRINGUP_PROFILE}"

  exec ros2 launch savo_bringup robot_bringup.launch.py \
    host_role:=core \
    robot_mode:="${ROBOT_MODE}" \
    bringup_profile:="${BRINGUP_PROFILE}" \
    map_id:="${MAP_ID}" \
    map_output_root:="${MAP_SESSIONS_ROOT}" \
    production_map_root:="${PRODUCTION_MAP_ROOT}" \
    active_map_contract:="${ACTIVE_MAP_CONTRACT}" \
    locations_database_path:="${LOCATION_STATE_ROOT}/locations.db" \
    locations_releases_root:="${LOCATION_STATE_ROOT}/releases" \
    supervisor_state_path:="${SUPERVISOR_STATE_ROOT}/system_state.json" \
    require_locked_geometry:="${SAVO_REQUIRE_LOCKED_GEOMETRY:-true}" \
    allow_provisional_geometry:="${SAVO_ALLOW_PROVISIONAL_GEOMETRY:-false}" \
    d435_voxel_validated:="${SAVO_D435_VOXEL_VALIDATED:-false}" \
    control_startup_mode:="${SAVO_CONTROL_STARTUP_MODE:-STOP}" \
    localization_use_vo:="${SAVO_LOCALIZATION_USE_VO:-true}" \
    start_semantic_interruption:="${SAVO_START_SEMANTIC_INTERRUPTION:-true}" \
    start_speech:=false \
    start_ui:=false
}

main "$@"
