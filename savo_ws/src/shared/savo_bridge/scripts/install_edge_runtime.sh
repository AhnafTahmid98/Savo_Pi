#!/usr/bin/env bash
set -euo pipefail

START_SERVICE=false
ROBOT_USER="${SUDO_USER:-${USER}}"
WORKSPACE="${HOME}/Savo_Pi/savo_ws"
SHARED_GROUP="savomind-bridge"
SHARED_GID="10001"
SERVICE_NAME="savo_bridge.service"

usage() {
  cat <<'USAGE'
Usage: sudo install_edge_runtime.sh [options]

Options:
  --user USER          Native bridge runtime user
  --workspace PATH     Robot Savo ROS workspace
  --start              Enable and start after installation
  --help               Show this help

Without --start, this script only provisions files and never launches ROS.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --user)
      ROBOT_USER="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE="$2"
      shift 2
      ;;
    --start)
      START_SERVICE=true
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi

if ! id "${ROBOT_USER}" >/dev/null 2>&1; then
  echo "Unknown runtime user: ${ROBOT_USER}" >&2
  exit 1
fi

if getent group "${SHARED_GROUP}" >/dev/null 2>&1; then
  EXISTING_GID="$(getent group "${SHARED_GROUP}" | cut -d: -f3)"
  if [[ "${EXISTING_GID}" != "${SHARED_GID}" ]]; then
    echo "${SHARED_GROUP} exists with GID ${EXISTING_GID}, expected ${SHARED_GID}" >&2
    exit 1
  fi
else
  groupadd --gid "${SHARED_GID}" "${SHARED_GROUP}"
fi

usermod --append --groups "${SHARED_GROUP}" "${ROBOT_USER}"

TEMPLATE="${WORKSPACE}/install/savo_bridge/share/savo_bridge/systemd/savo_bridge.service.in"
TARGET="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${TEMPLATE}" ]]; then
  echo "Missing installed service template: ${TEMPLATE}" >&2
  exit 1
fi

sed \
  -e "s|@ROBOT_USER@|${ROBOT_USER}|g" \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  "${TEMPLATE}" > "${TARGET}"

chmod 0644 "${TARGET}"
systemctl daemon-reload

if [[ "${START_SERVICE}" == true ]]; then
  systemctl enable --now "${SERVICE_NAME}"
else
  echo "Installed ${TARGET}; service was not enabled or started."
fi
