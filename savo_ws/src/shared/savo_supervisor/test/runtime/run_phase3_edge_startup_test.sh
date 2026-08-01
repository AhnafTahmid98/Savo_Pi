#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
set -u

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-227}"
export ROS_LOG_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_phase3_runtime/ros"

cd "$HOME/Savo_Pi/savo_ws" || exit 1

PACKAGE_DIR="$HOME/Savo_Pi/savo_ws/src/shared/savo_supervisor"
RUNTIME_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_phase3_runtime"
STATE_FILE="/tmp/savo_supervisor_phase3_state.json"
mkdir -p "$RUNTIME_DIR" "$ROS_LOG_DIR"
rm -f "$RUNTIME_DIR"/*.log "$RUNTIME_DIR"/*.json "$STATE_FILE"

SUPERVISOR_PID=""
FIXTURE_PID=""
EVENT_PID=""

stop_process()
{
  local pid="${1:-}"
  if [[ -z "$pid" ]]; then
    return 0
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 40); do
    if ! kill -0 "$pid" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup()
{
  trap - EXIT INT TERM
  stop_process "$EVENT_PID"
  stop_process "$SUPERVISOR_PID"
  stop_process "$FIXTURE_PID"
}
trap cleanup EXIT INT TERM

python3 "$PACKAGE_DIR/test/runtime/phase2_fixture.py" \
  > "$RUNTIME_DIR/fixture.log" 2>&1 &
FIXTURE_PID=$!

ros2 launch savo_supervisor supervisor.launch.py \
  auto_arm:=false \
  "system_state_path:=$STATE_FILE" \
  > "$RUNTIME_DIR/supervisor.log" 2>&1 &
SUPERVISOR_PID=$!

timeout 150 ros2 topic echo \
  /savo_supervisor/events \
  std_msgs/msg/String \
  --field data \
  > "$RUNTIME_DIR/events.log" 2>&1 &
EVENT_PID=$!

python3 "$PACKAGE_DIR/test/runtime/phase3_supervisor_probe.py" \
  "$RUNTIME_DIR/snapshots.json" \
  | tee "$RUNTIME_DIR/probe.log"

stop_process "$EVENT_PID"
EVENT_PID=""
stop_process "$SUPERVISOR_PID"
SUPERVISOR_PID=""
stop_process "$FIXTURE_PID"
FIXTURE_PID=""
trap - EXIT INT TERM

printf '\n%s\n' '===== PHASE 3 EVENTS ====='
cat "$RUNTIME_DIR/events.log"
printf '\n%s\n' 'PHASE_3_EDGE_STARTUP_RUNTIME_COMPLETE'
