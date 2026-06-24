#!/usr/bin/env bash
# Shared Robot Savo deploy environment.

if [[ "${SAVO_ENV_COMMON_LOADED:-0}" == "1" ]]; then
  return 0 2>/dev/null || exit 0
fi
export SAVO_ENV_COMMON_LOADED=1

export SAVO_ROOT="${SAVO_ROOT:-$HOME/Savo_Pi}"
export SAVO_WS="${SAVO_WS:-$SAVO_ROOT/savo_ws}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Direct core-edge Ethernet link.
export SAVO_CORE_ETH_IP="${SAVO_CORE_ETH_IP:-192.168.50.1}"
export SAVO_EDGE_ETH_IP="${SAVO_EDGE_ETH_IP:-192.168.50.2}"
export SAVO_ETH_IFACE="${SAVO_ETH_IFACE:-eth0}"
export SAVO_WIFI_IFACE="${SAVO_WIFI_IFACE:-wlan0}"

# ROS 2 network defaults.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

savo_log() {
  printf '[SavoDeploy] %s\n' "$*"
}

savo_die() {
  printf '[SavoDeploy][ERROR] %s\n' "$*" >&2
  exit 1
}

savo_require_cmd() {
  command -v "$1" >/dev/null 2>&1 || savo_die "Missing command: $1"
}

savo_require_dir() {
  [[ -d "$1" ]] || savo_die "Missing directory: $1"
}

savo_source_ros() {
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  [[ -f "$setup_file" ]] || savo_die "ROS setup not found: $setup_file"
  # shellcheck disable=SC1090
  source "$setup_file"
}

savo_source_ws() {
  local setup_file="${SAVO_WS}/install/setup.bash"
  if [[ -f "$setup_file" ]]; then
    # shellcheck disable=SC1090
    source "$setup_file"
  else
    savo_log "Workspace setup not found yet: $setup_file"
  fi
}

savo_print_env() {
  cat <<ENV
Robot Savo deploy environment
  SAVO_ROOT          = ${SAVO_ROOT}
  SAVO_WS            = ${SAVO_WS}
  ROS_DISTRO         = ${ROS_DISTRO}
  ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID}
  ROS_LOCALHOST_ONLY = ${ROS_LOCALHOST_ONLY}
  ETH_IFACE          = ${SAVO_ETH_IFACE}
  WIFI_IFACE         = ${SAVO_WIFI_IFACE}
  CORE_ETH_IP        = ${SAVO_CORE_ETH_IP}
  EDGE_ETH_IP        = ${SAVO_EDGE_ETH_IP}
ENV
}
