#!/usr/bin/env bash
# Prepare permanent Robot Savo production state and log directories.

set -Eeuo pipefail

OWNER="${SUDO_USER:-${USER:-}}"
GROUP="${OWNER}"
STATE_ROOT="/var/lib/robot_savo"
LOG_ROOT="/var/log/robot_savo"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --owner)
      OWNER="${2:-}"
      shift 2
      ;;
    --group)
      GROUP="${2:-}"
      shift 2
      ;;
    --state-root)
      STATE_ROOT="${2:-}"
      shift 2
      ;;
    --log-root)
      LOG_ROOT="${2:-}"
      shift 2
      ;;
    -h|--help)
      echo "Usage: sudo $0 [--owner USER] [--group GROUP] [--state-root PATH] [--log-root PATH]"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "${OWNER}" || -z "${GROUP}" ]]; then
  echo "Owner and group must be non-empty." >&2
  exit 2
fi

if [[ -z "${STATE_ROOT}" || -z "${LOG_ROOT}" || "${STATE_ROOT}" == "/" || "${LOG_ROOT}" == "/" ]]; then
  echo "State and log roots must be non-empty, non-root paths." >&2
  exit 2
fi

if [[ "${EUID}" -ne 0 && ( "${STATE_ROOT}" == /var/* || "${LOG_ROOT}" == /var/* ) ]]; then
  echo "Run this script with sudo for /var storage roots." >&2
  exit 1
fi

directories=(
  "${STATE_ROOT}"
  "${STATE_ROOT}/maps"
  "${STATE_ROOT}/maps/sessions"
  "${STATE_ROOT}/maps/production"
  "${STATE_ROOT}/maps/production/releases"
  "${STATE_ROOT}/maps/release_transactions"
  "${STATE_ROOT}/locations"
  "${STATE_ROOT}/locations/backups"
  "${STATE_ROOT}/locations/releases"
  "${STATE_ROOT}/supervisor"
  "${STATE_ROOT}/ros"
  "${LOG_ROOT}"
  "${LOG_ROOT}/core"
  "${LOG_ROOT}/edge"
)

if [[ "${EUID}" -eq 0 ]]; then
  install -d -m 0750 -o "${OWNER}" -g "${GROUP}" "${directories[@]}"
else
  current_owner="$(id -un)"
  current_group="$(id -gn)"
  if [[ "${OWNER}" != "${current_owner}" || "${GROUP}" != "${current_group}" ]]; then
    echo "Non-root custom-root setup must use ${current_owner}:${current_group}." >&2
    exit 1
  fi
  install -d -m 0750 "${directories[@]}"
fi

for directory in "${directories[@]}"; do
  [[ -d "${directory}" && -w "${directory}" ]] || {
    echo "Storage directory is missing or not writable: ${directory}" >&2
    exit 1
  }
done

printf 'Robot Savo runtime storage prepared for %s:%s\n' "${OWNER}" "${GROUP}"
