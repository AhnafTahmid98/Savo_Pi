#!/usr/bin/env bash
# Robot Savo rp_filter setup.
# Wi-Fi is used for internet/SSH, eth0 is used for direct core-edge robot communication.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "${SCRIPT_DIR}/env_common.sh"

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

SYSCTL_FILE="/etc/sysctl.d/99-robot-savo-network.conf"

savo_log "Writing network sysctl config: ${SYSCTL_FILE}"

${SUDO} tee "${SYSCTL_FILE}" >/dev/null <<EOF_SYSCTL
# Robot Savo network settings.
# Wi-Fi: internet / SSH
# eth0: direct core-edge robot link

net.ipv4.conf.all.rp_filter=0
net.ipv4.conf.default.rp_filter=0
net.ipv4.conf.${SAVO_ETH_IFACE}.rp_filter=0
net.ipv4.conf.${SAVO_WIFI_IFACE}.rp_filter=0
EOF_SYSCTL

savo_log "Applying sysctl settings..."
${SUDO} sysctl --system >/dev/null

savo_log "Current Robot Savo network values:"
sysctl "net.ipv4.conf.all.rp_filter"
sysctl "net.ipv4.conf.default.rp_filter"
sysctl "net.ipv4.conf.${SAVO_ETH_IFACE}.rp_filter"
sysctl "net.ipv4.conf.${SAVO_WIFI_IFACE}.rp_filter"

savo_log "rp_filter setup completed"
