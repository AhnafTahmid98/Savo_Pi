#!/usr/bin/env bash
# Robot Savo Ethernet recovery reset.
# Matches the validated manual recovery sequence.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

AUTO_YES=0
CHECK_PEER=0

for arg in "$@"; do
  case "$arg" in
    -y|--yes)
      AUTO_YES=1
      ;;
    --check)
      CHECK_PEER=1
      ;;
    -h|--help)
      cat <<HELP
Usage:
  robot_savo_network_reset.sh [--yes] [--check]

Validated recovery sequence:
  1. eth0 down
  2. wait
  3. eth0 up
  4. wait
  5. flush all neighbor/ARP entries
  6. flush route cache
  7. restart NetworkManager
  8. optional peer ping with --check

Recommended:
  Run this on both Pis:
    ~/Savo_Pi/deploy/common/robot_savo_network_reset.sh --yes

Then validate:
    ~/Savo_Pi/deploy/common/robot_savo_health_check.sh
HELP
      exit 0
      ;;
    *)
      savo_die "Unknown argument: $arg"
      ;;
  esac
done

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

detect_role() {
  local host
  host="$(hostname -s)"

  case "$host" in
    core|savo-core)
      SAVO_ROLE="core"
      SAVO_THIS_IP="${SAVO_CORE_ETH_IP}"
      SAVO_PEER_IP="${SAVO_EDGE_ETH_IP}"
      ;;
    edge|savo-edge)
      SAVO_ROLE="edge"
      SAVO_THIS_IP="${SAVO_EDGE_ETH_IP}"
      SAVO_PEER_IP="${SAVO_CORE_ETH_IP}"
      ;;
    *)
      savo_die "Unknown host role: $host. Expected core/savo-core or edge/savo-edge."
      ;;
  esac
}

confirm_reset() {
  if [[ "$AUTO_YES" == "1" ]]; then
    return 0
  fi

  cat <<WARN
[SavoDeploy] This will reset ${SAVO_ETH_IFACE} and restart NetworkManager.
[SavoDeploy] Use Wi-Fi SSH or local terminal, not Ethernet SSH.

Type RESET to continue:
WARN

  read -r answer
  [[ "$answer" == "RESET" ]] || savo_die "Cancelled"
}

show_status() {
  savo_log "Hostname: $(hostname)"
  savo_log "Role: ${SAVO_ROLE}"
  savo_log "This IP: ${SAVO_THIS_IP}"
  savo_log "Peer IP: ${SAVO_PEER_IP}"

  ip -4 addr show "${SAVO_ETH_IFACE}" || true
  ip route | grep -E "default|192.168.50" || true
}

reset_network_state() {
  savo_log "Bringing ${SAVO_ETH_IFACE} down..."
  ${SUDO} ip link set "${SAVO_ETH_IFACE}" down
  sleep 3

  savo_log "Bringing ${SAVO_ETH_IFACE} up..."
  ${SUDO} ip link set "${SAVO_ETH_IFACE}" up
  sleep 3

  savo_log "Flushing all neighbor/ARP entries..."
  ${SUDO} ip neigh flush all || true

  savo_log "Flushing route cache..."
  ${SUDO} ip route flush cache || true

  savo_log "Restarting NetworkManager..."
  ${SUDO} systemctl restart NetworkManager
  sleep 5
}

check_peer_optional() {
  if [[ "$CHECK_PEER" != "1" ]]; then
    savo_log "Skipping peer ping. Run health check after resetting both Pis."
    return 0
  fi

  savo_log "Testing peer ping: ${SAVO_PEER_IP}"

  if ping -c 5 "${SAVO_PEER_IP}"; then
    savo_log "Peer ping passed"
  else
    savo_log "Peer ping failed. Run this reset on the other Pi too, then run health check."
  fi

  savo_log "Neighbor table:"
  ip neigh show dev "${SAVO_ETH_IFACE}" || true
}

main() {
  detect_role
  show_status
  confirm_reset

  reset_network_state

  show_status
  check_peer_optional

  savo_log "Robot Savo local Ethernet reset completed"
}

main "$@"
