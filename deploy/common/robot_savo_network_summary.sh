#!/usr/bin/env bash
# Robot Savo network summary.
# Read-only status report for core-edge Ethernet and Wi-Fi routing.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

PING_PEER=1

for arg in "$@"; do
  case "$arg" in
    --no-ping)
      PING_PEER=0
      ;;
    -h|--help)
      cat <<HELP
Usage:
  robot_savo_network_summary.sh [--no-ping]

Prints Robot Savo network status without changing anything:
  - role and hostname
  - eth0 / wlan0 IPs
  - routes
  - NetworkManager status
  - Ethernet link speed
  - rp_filter
  - neighbor table
  - optional peer ping
HELP
      exit 0
      ;;
    *)
      savo_die "Unknown argument: $arg"
      ;;
  esac
done

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
      SAVO_ROLE="unknown"
      SAVO_THIS_IP="unknown"
      SAVO_PEER_IP="unknown"
      ;;
  esac
}

section() {
  printf '\n===== %s =====\n' "$*"
}

print_basic() {
  section "Robot Savo network summary"
  echo "Hostname: $(hostname)"
  echo "Role: ${SAVO_ROLE}"
  echo "This Pi IP: ${SAVO_THIS_IP}"
  echo "Peer Pi IP: ${SAVO_PEER_IP}"
  echo "Ethernet interface: ${SAVO_ETH_IFACE}"
  echo "Wi-Fi interface: ${SAVO_WIFI_IFACE}"
}

print_nmcli() {
  section "NetworkManager"
  if command -v nmcli >/dev/null 2>&1; then
    nmcli device status
    echo
    nmcli connection show --active
  else
    echo "nmcli not found"
  fi
}

print_ip_status() {
  section "IP addresses"
  echo "[${SAVO_ETH_IFACE}]"
  ip -4 addr show "${SAVO_ETH_IFACE}" || true

  echo
  echo "[${SAVO_WIFI_IFACE}]"
  ip -4 addr show "${SAVO_WIFI_IFACE}" || true
}

print_routes() {
  section "Routes"
  ip route | grep -E "default|192.168.50|10\.|172\." || ip route || true
}

print_link_speed() {
  section "Ethernet link"
  if command -v ethtool >/dev/null 2>&1; then
    ethtool "${SAVO_ETH_IFACE}" 2>/dev/null | grep -E "Speed|Duplex|Auto-negotiation|Link detected" || true
  else
    echo "ethtool not found"
  fi
}

print_kernel_settings() {
  section "Kernel network settings"
  sysctl "net.ipv4.conf.${SAVO_ETH_IFACE}.rp_filter" 2>/dev/null || true
  sysctl "net.ipv4.conf.all.rp_filter" 2>/dev/null || true
}

print_neighbor_table() {
  section "Neighbor table"
  ip neigh show dev "${SAVO_ETH_IFACE}" || true
}

print_peer_ping() {
  if [[ "${PING_PEER}" != "1" ]]; then
    return 0
  fi

  section "Peer ping"
  if [[ "${SAVO_PEER_IP}" == "unknown" ]]; then
    echo "Peer IP unknown; skipping ping"
    return 0
  fi

  ping -c 5 "${SAVO_PEER_IP}" || true
}

main() {
  detect_role
  print_basic
  print_nmcli
  print_ip_status
  print_routes
  print_link_speed
  print_kernel_settings
  print_neighbor_table
  print_peer_ping
}

main "$@"
