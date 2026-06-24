#!/usr/bin/env bash
# Robot Savo core deploy environment.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../common/env_common.sh
source "${SCRIPT_DIR}/../common/env_common.sh"

export SAVO_ROLE="core"
export SAVO_THIS_IP="${SAVO_CORE_ETH_IP}"
export SAVO_PEER_IP="${SAVO_EDGE_ETH_IP}"

# Shared packages needed on core.
SAVO_CORE_SHARED_PACKAGES=(
  savo_msgs
  savo_description
  savo_bringup
  savo_dashboard
  savo_perception
  savo_supervisor
)

# Core-owned runtime packages.
SAVO_CORE_PACKAGES=(
  savo_base
  savo_control
  savo_lidar
  savo_localization
  savo_locations
  savo_mapping
  savo_nav
  savo_head
)

# Full core build order: shared first, then core runtime packages.
SAVO_CORE_BUILD_PACKAGES=(
  "${SAVO_CORE_SHARED_PACKAGES[@]}"
  "${SAVO_CORE_PACKAGES[@]}"
)

savo_assert_core_host() {
  local host
  host="$(hostname -s)"

  if [[ "$host" != "core" && "$host" != "savo-core" ]]; then
    savo_die "This script is for savo-core, but current hostname is: $host"
  fi
}

savo_print_core_env() {
  savo_print_env
  cat <<ENV
  SAVO_ROLE             = ${SAVO_ROLE}
  THIS_IP               = ${SAVO_THIS_IP}
  PEER_IP               = ${SAVO_PEER_IP}
  CORE_SHARED_PACKAGES  = ${SAVO_CORE_SHARED_PACKAGES[*]}
  CORE_PACKAGES         = ${SAVO_CORE_PACKAGES[*]}
  CORE_BUILD_PACKAGES   = ${SAVO_CORE_BUILD_PACKAGES[*]}
ENV
}
