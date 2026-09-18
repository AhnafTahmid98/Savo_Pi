#!/usr/bin/env bash
# Run the dedicated autonomous-mapping Core composition without submitting a mission.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_core.sh
source "${SCRIPT_DIR}/env_core.sh"

[[ "${SAVO_CONTROL_STARTUP_MODE:-STOP}" == STOP ]] || \
  savo_die "Control startup must remain STOP"
for argument in "$@"; do
  case "${argument}" in
    control_startup_mode:=*|start_supervisor:=*|supervisor_auto_arm:=*)
      savo_die "Dedicated autonomous runner forbids overriding ${argument%%:=*}"
      ;;
  esac
done
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
  --owner autonomous-mapping \
  --lock-file /run/robot-savo/core-owner.lock \
  -- ros2 launch savo_bringup autonomous_mapping.launch.py \
  "$@" \
  control_startup_mode:="${SAVO_CONTROL_STARTUP_MODE:-STOP}" \
  start_supervisor:=false \
  supervisor_auto_arm:=false
