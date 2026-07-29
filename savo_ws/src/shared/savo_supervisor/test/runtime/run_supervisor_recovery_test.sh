#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary


set -eo pipefail

# ROS setup scripts may inspect variables that are not defined yet.
# Enable nounset only after the environments are loaded.
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
set -u

cd "$HOME/Savo_Pi/savo_ws" || exit 1

PACKAGE_DIR="$HOME/Savo_Pi/savo_ws/src/shared/savo_supervisor"
RUNTIME_DIR="$HOME/Savo_Pi/savo_ws/log/savo_supervisor_runtime"

mkdir -p "$RUNTIME_DIR"

EVENT_LOG="$RUNTIME_DIR/events.log"
LAUNCH_LOG="$RUNTIME_DIR/launch.log"

STATE_MISSING="$RUNTIME_DIR/state_missing.txt"
STATE_HEALTHY="$RUNTIME_DIR/state_healthy.txt"
STATE_STALE="$RUNTIME_DIR/state_stale.txt"
STATE_RECOVERED="$RUNTIME_DIR/state_recovered.txt"

HEARTBEAT_RECOVERED="$RUNTIME_DIR/heartbeat_recovered.txt"
DIAGNOSTICS_RECOVERED="$RUNTIME_DIR/diagnostics_recovered.txt"

rm -f \
  "$EVENT_LOG" \
  "$LAUNCH_LOG" \
  "$STATE_MISSING" \
  "$STATE_HEALTHY" \
  "$STATE_STALE" \
  "$STATE_RECOVERED" \
  "$HEARTBEAT_RECOVERED" \
  "$DIAGNOSTICS_RECOVERED"

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

  # Stop the supervisor first so fixture shutdown does not
  # create another artificial stale transition.
  stop_process "$SUPERVISOR_PID"
  stop_process "$FIXTURE_PID"
  stop_process "$EVENT_PID"

  SUPERVISOR_PID=""
  FIXTURE_PID=""
  EVENT_PID=""
}

trap cleanup EXIT INT TERM

echo "Starting event listener..."

timeout 60 ros2 topic echo \
  /savo_supervisor/events \
  std_msgs/msg/String \
  --field data \
  > "$EVENT_LOG" 2>&1 &

EVENT_PID=$!

sleep 1

echo "Starting supervisor..."

ros2 launch \
  savo_supervisor \
  supervisor.launch.py \
  > "$LAUNCH_LOG" 2>&1 &

SUPERVISOR_PID=$!

echo "Waiting for missing-localization fault..."
sleep 5

timeout 5 ros2 topic echo \
  /savo_supervisor/state_summary \
  --once \
  --field data \
  > "$STATE_MISSING" 2>&1 || true

echo "Starting healthy localization fixture..."

python3 \
  $PACKAGE_DIR/test/runtime/localization_fixture.py \
  > "$RUNTIME_DIR/fixture_first.log" 2>&1 &

FIXTURE_PID=$!

sleep 4

timeout 5 ros2 topic echo \
  /savo_supervisor/state_summary \
  --once \
  --field data \
  > "$STATE_HEALTHY" 2>&1 || true

echo "Stopping fixture to force stale detection..."

stop_process "$FIXTURE_PID"
FIXTURE_PID=""

sleep 4

timeout 5 ros2 topic echo \
  /savo_supervisor/state_summary \
  --once \
  --field data \
  > "$STATE_STALE" 2>&1 || true

echo "Restarting fixture to test recovery..."

python3 \
  $PACKAGE_DIR/test/runtime/localization_fixture.py \
  > "$RUNTIME_DIR/fixture_second.log" 2>&1 &

FIXTURE_PID=$!

sleep 4

timeout 5 ros2 topic echo \
  /savo_supervisor/state_summary \
  --once \
  --field data \
  > "$STATE_RECOVERED" 2>&1 || true

timeout 5 ros2 topic echo \
  /savo_supervisor/heartbeat \
  --once \
  --field data \
  > "$HEARTBEAT_RECOVERED" 2>&1 || true

timeout 5 ros2 topic echo \
  /savo_supervisor/health \
  --once \
  > "$DIAGNOSTICS_RECOVERED" 2>&1 || true

sleep 1

cleanup
trap - EXIT INT TERM

echo
echo "===== 1. MISSING LOCALIZATION ====="
cat "$STATE_MISSING"

echo
echo "===== 2. HEALTHY LOCALIZATION ====="
cat "$STATE_HEALTHY"

echo
echo "===== 3. STALE AFTER FIXTURE STOP ====="
cat "$STATE_STALE"

echo
echo "===== 4. RECOVERED AFTER RESTART ====="
cat "$STATE_RECOVERED"

echo
echo "===== 5. RECOVERED HEARTBEAT ====="
cat "$HEARTBEAT_RECOVERED"

echo
echo "===== 6. TRANSITION EVENTS ====="
cat "$EVENT_LOG"

echo
echo "===== 7. RECOVERED DIAGNOSTICS ====="
cat "$DIAGNOSTICS_RECOVERED"

echo
echo "===== 8. SUPERVISOR LOG ====="
cat "$LAUNCH_LOG"

echo
echo "PHASE_S1A_6B_COMPLETE"
