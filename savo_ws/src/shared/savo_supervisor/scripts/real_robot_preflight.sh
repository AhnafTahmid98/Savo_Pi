#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -euo pipefail

WORKSPACE="${SAVO_WORKSPACE:-$HOME/Savo_Pi/savo_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
TIMEOUT_SEC="${SAVO_PREFLIGHT_TIMEOUT_SEC:-8}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

required_services=(
  /savo_supervisor/manage_system_state
  /savo_supervisor/authorize_operation
  /savo_supervisor/update_map_context
)
required_topics=(
  /savo_supervisor/state_summary
  /savo_supervisor/system_ready
  /savo_supervisor/remote_commands_ready
  /savo_bridge/state
  /savo_bridge/readiness
  /savo_bridge/heartbeat
  /safety/stop
  /safety/slowdown_factor
)

service_list="$(ros2 service list)"
topic_list="$(ros2 topic list)"
failed=0

for service in "${required_services[@]}"; do
  if ! grep -Fxq "$service" <<<"$service_list"; then
    echo "MISSING SERVICE: $service" >&2
    failed=1
  else
    echo "OK SERVICE: $service"
  fi
done

for topic in "${required_topics[@]}"; do
  if ! grep -Fxq "$topic" <<<"$topic_list"; then
    echo "MISSING TOPIC: $topic" >&2
    failed=1
  else
    echo "OK TOPIC: $topic"
  fi
done

if (( failed != 0 )); then
  echo "PHASE_3_REAL_ROBOT_PREFLIGHT_FAILED" >&2
  exit 1
fi

state="$(timeout "$TIMEOUT_SEC" ros2 topic echo --once \
  /savo_supervisor/state_summary std_msgs/msg/String --field data)"

printf '\n===== SUPERVISOR STATE =====\n%s\n' "$state"

python3 - "$state" <<'PY'
import json
import sys

payload = sys.argv[1].strip()
state = json.loads(payload)
system = state.get('system_authority', {})
edge = state.get('edge_capabilities', {})

errors = []
if system.get('system_armed') is not False:
    errors.append('system must be disarmed during preflight')
if system.get('startup_ready') is not True:
    errors.append('startup dependencies are not ready')
if system.get('state') not in {'READY_TO_ARM', 'DISARMED'}:
    errors.append(f"unexpected system state: {system.get('state')}")
if edge.get('bridge_ready') is not True:
    errors.append('bridge is not ready')
if edge.get('core_edge_link_ready') is not True:
    errors.append('core-edge link is not ready')

if errors:
    for error in errors:
        print(f'FAIL: {error}', file=sys.stderr)
    raise SystemExit(1)

print('PHASE_3_REAL_ROBOT_PREFLIGHT_COMPLETE')
PY
