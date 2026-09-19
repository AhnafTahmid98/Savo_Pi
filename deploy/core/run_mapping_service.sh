#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_core.sh
source "$script_dir/env_core.sh"
# shellcheck source=../common/production_geometry.sh
source "${SAVO_ROOT}/deploy/common/production_geometry.sh"
[[ "${SAVO_ENABLE_MAPPING_SERVICE:-false}" == true ]] || savo_die \
  "Mapping service is fail-closed; set SAVO_ENABLE_MAPPING_SERVICE=true and create /etc/robot-savo/enable-mapping-service"
[[ -e "/etc/robot-savo/enable-mapping-service" ]] || savo_die \
  "Mapping service marker is missing: /etc/robot-savo/enable-mapping-service"
[[ "${SAVO_CONTROL_STARTUP_MODE:-STOP}" == STOP ]] || savo_die "Control startup must remain STOP"
savo_assert_production_geometry_environment
savo_assert_core_host
savo_require_cmd python3
savo_require_cmd ros2
savo_require_dir "${SAVO_WS}"
ownership_tool="${SAVO_ROOT}/deploy/common/runtime_ownership.py"
ownership_preflight=(python3 "${ownership_tool}" preflight --owner mapping --ignore-pid "$$")
if [[ -n "${SAVO_SERVICE_UNIT:-}" ]]; then
  ownership_preflight+=(--allow-unit "${SAVO_SERVICE_UNIT}")
fi
"${ownership_preflight[@]}"
savo_source_ros
savo_source_ws
exec python3 "${ownership_tool}" run-core-owner \
  --owner mapping \
  --lock-file /run/robot-savo/core-owner.lock \
  -- ros2 launch savo_bringup manual_mapping.launch.py \
  geometry_profile:="$(savo_production_geometry_profile)" \
  require_locked_geometry:=true allow_provisional_geometry:=false
