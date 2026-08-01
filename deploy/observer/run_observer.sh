#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="${REPO_ROOT}/savo_ws"
MODE="${SAVO_OBSERVER_MODE:-full}"
PROFILE="${SAVO_OBSERVER_PROFILE:-standard}"
VIEW="${SAVO_OBSERVER_VIEW:-overview}"
PORT="${SAVO_OBSERVER_PORT:-8765}"
BIND_ADDRESS="${SAVO_OBSERVER_BIND_ADDRESS:-127.0.0.1}"

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"
printf 'Robot SAVO observer: mode=%s profile=%s view=%s bind=%s port=%s\n' \
  "${MODE}" "${PROFILE}" "${VIEW}" "${BIND_ADDRESS}" "${PORT}"
exec ros2 launch savo_observer observer.launch.py \
  mode:="${MODE}" profile:="${PROFILE}" view:="${VIEW}" \
  dashboard_bind_address:="${BIND_ADDRESS}" dashboard_port:="${PORT}" \
  enable_camera_preview:=false enable_pointclouds:=false
