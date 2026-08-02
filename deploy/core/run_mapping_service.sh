#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_core.sh
source "$script_dir/env_core.sh"
[[ "${SAVO_ENABLE_MAPPING_SERVICE:-false}" == true ]] || savo_die \
  "Mapping service is fail-closed; set SAVO_ENABLE_MAPPING_SERVICE=true and create /etc/robot-savo/enable-mapping-service"
[[ "${SAVO_CONTROL_STARTUP_MODE:-STOP}" == STOP ]] || savo_die "Control startup must remain STOP"
savo_source_ros
savo_source_ws
exec ros2 launch savo_bringup manual_mapping.launch.py \
  require_locked_geometry:=true allow_provisional_geometry:=false
