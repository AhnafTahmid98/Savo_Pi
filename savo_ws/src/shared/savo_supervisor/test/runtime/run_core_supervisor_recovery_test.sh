#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
set -u

cd "$HOME/Savo_Pi/savo_ws" || exit 1

PACKAGE_DIR="$HOME/Savo_Pi/savo_ws/src/shared/savo_supervisor"
RUNTIME_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_phase1_runtime"
mkdir -p "$RUNTIME_DIR"

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
  for _ in $(seq 1 30); do
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
  stop_process "$EVENT_PID"
}
trap cleanup EXIT INT TERM

start_fixture()
{
  local label="$1"
  shift
  stop_process "$FIXTURE_PID"
  FIXTURE_PID=""
  python3 "$PACKAGE_DIR/test/runtime/core_fixture.py" "$@" \
    > "$RUNTIME_DIR/fixture_${label}.log" 2>&1 &
  FIXTURE_PID=$!
}

probe()
{
  local output="$1"
  shift
  python3 "$PACKAGE_DIR/test/runtime/core_supervisor_probe.py" \
    --output "$RUNTIME_DIR/$output" "$@"
}

rm -f "$RUNTIME_DIR"/*.log "$RUNTIME_DIR"/*.json

timeout 90 ros2 topic echo \
  /savo_supervisor/events \
  std_msgs/msg/String \
  --field data \
  > "$RUNTIME_DIR/events.log" 2>&1 &
EVENT_PID=$!

ros2 launch savo_supervisor supervisor.launch.py \
  > "$RUNTIME_DIR/supervisor.log" 2>&1 &
SUPERVISOR_PID=$!

printf '%s\n' '1/5 Missing core contracts must fault after startup grace.'
probe missing.json \
  --timeout 12 \
  --lifecycle FAULTED \
  --health ERROR \
  --capability core_motion_ready \
  --capability-value false

printf '%s\n' '2/5 Healthy six-package core must become fully ready.'
start_fixture healthy
probe healthy.json \
  --timeout 12 \
  --lifecycle RUNNING \
  --health OK \
  --capability can_start_geometric_mapping \
  --capability-value true

printf '%s\n' '3/5 A targeted LiDAR stream loss must fail closed.'
start_fixture no_lidar --drop-component lidar
probe lidar_stale.json \
  --timeout 12 \
  --lifecycle FAULTED \
  --health ERROR \
  --component lidar \
  --component-state STALE \
  --capability core_motion_ready \
  --capability-value false

printf '%s\n' '4/5 LiDAR recovery must restore full readiness.'
start_fixture recovered
probe recovered.json \
  --timeout 12 \
  --lifecycle RUNNING \
  --health OK \
  --component lidar \
  --component-state OK \
  --capability can_start_geometric_mapping \
  --capability-value true

printf '%s\n' '5/5 A known safety stop must block motion without faulting the process.'
start_fixture safety_stop --safety-stop
probe safety_stop.json \
  --timeout 12 \
  --lifecycle RUNNING \
  --health DEGRADED \
  --reason safety_stop_active \
  --capability can_manual_drive \
  --capability-value false

stop_process "$FIXTURE_PID"
FIXTURE_PID=""
stop_process "$SUPERVISOR_PID"
SUPERVISOR_PID=""
stop_process "$EVENT_PID"
EVENT_PID=""
trap - EXIT INT TERM

printf '\n%s\n' '===== HEALTHY STATE ====='
cat "$RUNTIME_DIR/healthy.json"
printf '\n%s\n' '===== LIDAR STALE STATE ====='
cat "$RUNTIME_DIR/lidar_stale.json"
printf '\n%s\n' '===== RECOVERED STATE ====='
cat "$RUNTIME_DIR/recovered.json"
printf '\n%s\n' '===== SAFETY STOP STATE ====='
cat "$RUNTIME_DIR/safety_stop.json"
printf '\n%s\n' '===== EVENTS ====='
cat "$RUNTIME_DIR/events.log"
printf '\n%s\n' 'PHASE_1_CORE_SUPERVISOR_RUNTIME_COMPLETE'
