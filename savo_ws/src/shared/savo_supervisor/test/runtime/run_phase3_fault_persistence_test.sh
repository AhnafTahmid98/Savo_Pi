#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
set -u

unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-228}"
export ROS_LOG_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_phase3_persistence/ros"

cd "$HOME/Savo_Pi/savo_ws" || exit 1

PACKAGE_DIR="$HOME/Savo_Pi/savo_ws/src/shared/savo_supervisor"
RUNTIME_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_phase3_persistence"
STATE_FILE="/tmp/savo_supervisor_phase3_persistence_state.json"
mkdir -p "$RUNTIME_DIR" "$ROS_LOG_DIR"
rm -f "$RUNTIME_DIR"/*.log "$RUNTIME_DIR"/*.json "$STATE_FILE"

SUPERVISOR_PID=""
FIXTURE_PID=""

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
  stop_process "$SUPERVISOR_PID"
  stop_process "$FIXTURE_PID"
}
trap cleanup EXIT INT TERM

start_supervisor()
{
  local log_name="$1"
  ros2 launch savo_supervisor supervisor.launch.py \
    auto_arm:=false \
    "system_state_path:=$STATE_FILE" \
    > "$RUNTIME_DIR/$log_name" 2>&1 &
  SUPERVISOR_PID=$!
}

python3 "$PACKAGE_DIR/test/runtime/phase2_fixture.py" \
  > "$RUNTIME_DIR/fixture.log" 2>&1 &
FIXTURE_PID=$!

start_supervisor supervisor-first.log
python3 "$PACKAGE_DIR/test/runtime/phase3_persistence_probe.py" \
  create "$RUNTIME_DIR/create.json" \
  | tee "$RUNTIME_DIR/create.log"

python3 - "$STATE_FILE" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
state = json.loads(path.read_text())
if state.get('fault_latched') is not True:
    raise SystemExit('persistent state file does not contain fault_latched=true')
print('PASS: persistent state file contains the latched core fault')
PY

stop_process "$SUPERVISOR_PID"
SUPERVISOR_PID=""

timeout 8 ros2 topic pub --once \
  /savo_supervisor/test/control \
  std_msgs/msg/String \
  "{data: 'restore_core'}" \
  > "$RUNTIME_DIR/restore-core.log" 2>&1
sleep 1

start_supervisor supervisor-restarted.log
python3 "$PACKAGE_DIR/test/runtime/phase3_persistence_probe.py" \
  verify "$RUNTIME_DIR/verify.json" \
  | tee "$RUNTIME_DIR/verify.log"

stop_process "$SUPERVISOR_PID"
SUPERVISOR_PID=""
stop_process "$FIXTURE_PID"
FIXTURE_PID=""
trap - EXIT INT TERM

printf '\n%s\n' 'PHASE_3_FAULT_PERSISTENCE_RUNTIME_COMPLETE'
