#!/usr/bin/env bash
# Robot Savo core-edge deployment health check.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

FAILURES=0
WARNINGS=0

pass() {
  printf '[PASS] %s\n' "$*"
}

warn() {
  WARNINGS=$((WARNINGS + 1))
  printf '[WARN] %s\n' "$*" >&2
}

fail() {
  FAILURES=$((FAILURES + 1))
  printf '[FAIL] %s\n' "$*" >&2
}

section() {
  printf '\n===== %s =====\n' "$*"
}

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
      fail "Unknown host role: $host. Expected core/savo-core or edge/savo-edge."
      SAVO_ROLE="unknown"
      SAVO_THIS_IP=""
      SAVO_PEER_IP=""
      ;;
  esac
}

check_cmds() {
  section "Required commands"

  for cmd in ip ping hostname sysctl; do
    if command -v "$cmd" >/dev/null 2>&1; then
      pass "Command available: $cmd"
    else
      fail "Missing command: $cmd"
    fi
  done

  if command -v nmcli >/dev/null 2>&1; then
    pass "Command available: nmcli"
  else
    warn "nmcli not found"
  fi

  if command -v ethtool >/dev/null 2>&1; then
    pass "Command available: ethtool"
  else
    warn "ethtool not found"
  fi
}

check_interfaces() {
  section "Network interfaces"

  if ip link show "${SAVO_ETH_IFACE}" >/dev/null 2>&1; then
    pass "Ethernet interface exists: ${SAVO_ETH_IFACE}"
  else
    fail "Ethernet interface missing: ${SAVO_ETH_IFACE}"
    return
  fi

  if ip link show "${SAVO_WIFI_IFACE}" >/dev/null 2>&1; then
    pass "Wi-Fi interface exists: ${SAVO_WIFI_IFACE}"
  else
    warn "Wi-Fi interface missing: ${SAVO_WIFI_IFACE}"
  fi

  ip link show "${SAVO_ETH_IFACE}" | grep -q "LOWER_UP" \
    && pass "${SAVO_ETH_IFACE} physical link is up" \
    || fail "${SAVO_ETH_IFACE} physical link is not LOWER_UP"
}

check_ip_and_routes() {
  section "IP and route"

  echo "[INFO] Host role: ${SAVO_ROLE}"
  echo "[INFO] This Pi IP: ${SAVO_THIS_IP}"
  echo "[INFO] Peer Pi IP: ${SAVO_PEER_IP}"

  if ip -4 addr show "${SAVO_ETH_IFACE}" | grep -q "${SAVO_THIS_IP}/24"; then
    pass "${SAVO_ETH_IFACE} has expected IP: ${SAVO_THIS_IP}/24"
  else
    fail "${SAVO_ETH_IFACE} does not have expected IP: ${SAVO_THIS_IP}/24"
    ip -4 addr show "${SAVO_ETH_IFACE}" || true
  fi

  if ip route | grep -q "192.168.50.0/24 dev ${SAVO_ETH_IFACE}"; then
    pass "Direct Ethernet route exists for 192.168.50.0/24"
  else
    fail "Missing direct Ethernet route for 192.168.50.0/24"
    ip route || true
  fi

  if ip route | grep -q "^default .* dev ${SAVO_WIFI_IFACE}"; then
    pass "Default route uses Wi-Fi: ${SAVO_WIFI_IFACE}"
  else
    warn "Default route is not using ${SAVO_WIFI_IFACE}"
    ip route | grep "^default" || true
  fi
}

check_nmcli() {
  section "NetworkManager"

  if ! command -v nmcli >/dev/null 2>&1; then
    warn "Skipping nmcli checks"
    return
  fi

  nmcli device status || true

  if nmcli device status | grep -E "^${SAVO_ETH_IFACE}[[:space:]]+ethernet[[:space:]]+connected" >/dev/null; then
    pass "NetworkManager sees ${SAVO_ETH_IFACE} as connected"
  else
    fail "NetworkManager does not show ${SAVO_ETH_IFACE} connected"
  fi
}

check_link_speed() {
  section "Ethernet link speed"

  if ! command -v ethtool >/dev/null 2>&1; then
    warn "Skipping ethtool checks"
    return
  fi

  local info
  info="$(sudo ethtool "${SAVO_ETH_IFACE}" 2>/dev/null || true)"
  echo "$info" | grep -E "Speed|Duplex|Auto-negotiation|Link detected" || true

  echo "$info" | grep -q "Speed: 1000Mb/s" \
    && pass "Ethernet speed is 1000Mb/s" \
    || warn "Ethernet speed is not 1000Mb/s"

  echo "$info" | grep -q "Duplex: Full" \
    && pass "Ethernet duplex is Full" \
    || fail "Ethernet duplex is not Full"

  echo "$info" | grep -q "Link detected: yes" \
    && pass "Ethernet link detected" \
    || fail "Ethernet link not detected"
}

check_sysctl() {
  section "Kernel network settings"

  local eth_rp
  eth_rp="$(sysctl -n "net.ipv4.conf.${SAVO_ETH_IFACE}.rp_filter" 2>/dev/null || echo unknown)"

  if [[ "$eth_rp" == "0" ]]; then
    pass "${SAVO_ETH_IFACE} rp_filter is 0"
  else
    warn "${SAVO_ETH_IFACE} rp_filter is ${eth_rp}, expected 0 for Robot Savo dual-interface setup"
  fi
}

check_peer_ping() {
  section "Peer connectivity"

  sudo ip neigh flush dev "${SAVO_ETH_IFACE}" >/dev/null 2>&1 || true

  if ping -c 5 -W 2 "${SAVO_PEER_IP}"; then
    pass "Peer ping passed: ${SAVO_PEER_IP}"
  else
    fail "Peer ping failed: ${SAVO_PEER_IP}"
  fi

  echo "[INFO] Neighbor table:"
  ip neigh show dev "${SAVO_ETH_IFACE}" || true
}

check_ros_workspace() {
  section "ROS workspace"

  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    pass "ROS ${ROS_DISTRO} setup exists"
  else
    warn "ROS ${ROS_DISTRO} setup not found"
  fi

  if [[ -d "${SAVO_WS}" ]]; then
    pass "Workspace exists: ${SAVO_WS}"
  else
    fail "Workspace missing: ${SAVO_WS}"
  fi

  if [[ -f "${SAVO_WS}/install/setup.bash" ]]; then
    pass "Workspace install setup exists"
  else
    warn "Workspace has not been built yet or install/setup.bash is missing"
  fi
}

main() {
  section "Robot Savo health check"

  detect_role
  check_cmds
  check_interfaces
  check_ip_and_routes
  check_nmcli
  check_link_speed
  check_sysctl
  check_peer_ping
  check_ros_workspace

  section "Summary"
  echo "Warnings: ${WARNINGS}"
  echo "Failures: ${FAILURES}"

  if [[ "${FAILURES}" -eq 0 ]]; then
    pass "Robot Savo deploy health check passed"
    exit 0
  fi

  fail "Robot Savo deploy health check failed"
  exit 1
}

main "$@"
