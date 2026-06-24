#!/usr/bin/env bash
# Robot Savo core-edge bandwidth test using iperf3.
# Read-only network performance validation.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

MODE=""
DURATION=10
PORT=5201

for arg in "$@"; do
  case "$arg" in
    --server)
      MODE="server"
      ;;
    --client)
      MODE="client"
      ;;
    --reverse)
      MODE="reverse"
      ;;
    --both)
      MODE="both"
      ;;
    --duration=*)
      DURATION="${arg#*=}"
      ;;
    --port=*)
      PORT="${arg#*=}"
      ;;
    -h|--help)
      cat <<HELP
Usage:
  robot_savo_bandwidth_test.sh --server
  robot_savo_bandwidth_test.sh --client
  robot_savo_bandwidth_test.sh --reverse
  robot_savo_bandwidth_test.sh --both

Options:
  --server          Start iperf3 server on this Pi.
  --client          Test this Pi -> peer Pi.
  --reverse         Test peer Pi -> this Pi using iperf3 reverse mode.
  --both            Run client and reverse tests.
  --duration=N      Test duration in seconds. Default: 10.
  --port=N          iperf3 port. Default: 5201.

Typical Robot Savo test:
  On edge:
    robot_savo_bandwidth_test.sh --server

  On core:
    robot_savo_bandwidth_test.sh --both
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
      savo_die "Unknown host role: $host. Expected core/savo-core or edge/savo-edge."
      ;;
  esac
}

print_header() {
  savo_log "Robot Savo bandwidth test"
  savo_log "Role: ${SAVO_ROLE}"
  savo_log "This IP: ${SAVO_THIS_IP}"
  savo_log "Peer IP: ${SAVO_PEER_IP}"
  savo_log "Duration: ${DURATION}s"
  savo_log "Port: ${PORT}"
}

run_server() {
  savo_log "Starting iperf3 server on ${SAVO_THIS_IP}:${PORT}"
  savo_log "Stop with Ctrl+C"
  iperf3 -s -p "${PORT}"
}

run_client() {
  savo_log "Testing ${SAVO_THIS_IP} -> ${SAVO_PEER_IP}"
  iperf3 -c "${SAVO_PEER_IP}" -p "${PORT}" -t "${DURATION}"
}

run_reverse() {
  savo_log "Testing ${SAVO_PEER_IP} -> ${SAVO_THIS_IP} using reverse mode"
  iperf3 -c "${SAVO_PEER_IP}" -p "${PORT}" -t "${DURATION}" -R
}

main() {
  [[ -n "${MODE}" ]] || savo_die "Choose one mode: --server, --client, --reverse, or --both"

  savo_require_cmd iperf3
  detect_role
  print_header

  case "${MODE}" in
    server)
      run_server
      ;;
    client)
      run_client
      ;;
    reverse)
      run_reverse
      ;;
    both)
      run_client
      echo
      run_reverse
      ;;
    *)
      savo_die "Invalid mode: ${MODE}"
      ;;
  esac
}

main "$@"
