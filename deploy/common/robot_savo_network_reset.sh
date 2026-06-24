#!/usr/bin/env bash
# Robot Savo Ethernet recovery reset.
# Use when core-edge eth0 link is UP but ping/ARP fails.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

AUTO_YES=0

for arg in "$@"; do
  case "$arg" in
    -y|--yes)
      AUTO_YES=1
      ;;
    -h|--help)
      cat <<HELP
Usage:
  robot_savo_network_reset.sh [--yes]

This resets the Robot Savo eth0 direct Ethernet link:
  - eth0 down/up
  - flush neighbor/ARP table
  - flush route cache
  - restart NetworkManager
  - test peer ping

Run on both Pis if the core-edge link becomes stale.
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
[SavoDeploy] If you are connected through Ethernet SSH, your session may disconnect.
[SavoDeploy] Recommended: run over Wi-Fi SSH or local terminal.

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

reset_eth_link() {
  savo_log "Bringing ${SAVO_ETH_IFACE} down..."
  ${SUDO} ip link set "${SAVO_ETH_IFACE}" down

  sleep 3

  savo_log "Bringing ${SAVO_ETH_IFACE} up..."
  ${SUDO} ip link set "${SAVO_ETH_IFACE}" up

  sleep 3
}

flush_kernel_network_state() {
  savo_log "Flushing neighbor/ARP state..."
  ${SUDO} ip neigh flush all || true

  savo_log "Flushing route cache..."
  ${SUDO} ip route flush cache || true
}

restart_network_manager() {
  if systemctl list-unit-files | grep -q "^NetworkManager.service"; then
    savo_log "Restarting NetworkManager..."
    ${SUDO} systemctl restart NetworkManager
    sleep 5
  else
    savo_die "NetworkManager service not found"
  fi
}

reconnect_eth() {
  if command -v nmcli >/dev/null 2>&1; then
    savo_log "Reconnecting ${SAVO_ETH_IFACE} with NetworkManager..."
    ${SUDO} nmcli device connect "${SAVO_ETH_IFACE}" >/dev/null 2>&1 || true
    sleep 2
  fi
}

test_peer() {
  savo_log "Testing peer ping: ${SAVO_PEER_IP}"

  if ping -c 5 -W 2 "${SAVO_PEER_IP}"; then
    savo_log "Peer ping passed"
  else
    savo_die "Peer ping failed after reset"
  fi

  savo_log "Neighbor table:"
  ip neigh show dev "${SAVO_ETH_IFACE}" || true
}

main() {
  detect_role
  show_status
  confirm_reset

  reset_eth_link
  flush_kernel_network_state
  restart_network_manager
  reconnect_eth

  show_status
  test_peer

  savo_log "Robot Savo Ethernet reset completed successfully"
}

main "$@"
