#!/usr/bin/env bash
# Robot Savo edge deploy environment.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../common/env_common.sh
source "${SCRIPT_DIR}/../common/env_common.sh"

export SAVO_ROLE="edge"
export SAVO_THIS_IP="${SAVO_EDGE_ETH_IP}"
export SAVO_PEER_IP="${SAVO_CORE_ETH_IP}"

# Shared packages needed on edge.
SAVO_EDGE_SHARED_PACKAGES=(
  savo_msgs
  savo_description
  savo_bringup
  savo_dashboard
  savo_perception
  savo_supervisor
)

# Edge-owned runtime packages.
SAVO_EDGE_PACKAGES=(
  savo_intent
  savo_realsense
  savo_speech
  savo_ui
  savo_vo
)

# Full edge build order: shared first, then edge runtime packages.
SAVO_EDGE_BUILD_PACKAGES=(
  "${SAVO_EDGE_SHARED_PACKAGES[@]}"
  "${SAVO_EDGE_PACKAGES[@]}"
)

savo_assert_edge_host() {
  local host
  host="$(hostname -s)"

  if [[ "$host" != "edge" && "$host" != "savo-edge" ]]; then
    savo_die "This script is for savo-edge, but current hostname is: $host"
  fi
}

savo_print_edge_env() {
  savo_print_env
  cat <<ENV
  SAVO_ROLE             = ${SAVO_ROLE}
  THIS_IP               = ${SAVO_THIS_IP}
  PEER_IP               = ${SAVO_PEER_IP}
  EDGE_SHARED_PACKAGES  = ${SAVO_EDGE_SHARED_PACKAGES[*]}
  EDGE_PACKAGES         = ${SAVO_EDGE_PACKAGES[*]}
  EDGE_BUILD_PACKAGES   = ${SAVO_EDGE_BUILD_PACKAGES[*]}
ENV
}
