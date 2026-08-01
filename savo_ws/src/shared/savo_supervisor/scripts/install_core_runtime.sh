#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

set -euo pipefail

ROBOT_USER="${1:-${SUDO_USER:-$USER}}"
ROBOT_HOME="$(getent passwd "$ROBOT_USER" | cut -d: -f6)"
WORKSPACE="${2:-${ROBOT_HOME}/Savo_Pi/savo_ws}"
TEMPLATE="${WORKSPACE}/install/savo_supervisor/share/savo_supervisor/systemd/savo-supervisor.service.in"
TARGET="/etc/systemd/system/savo-supervisor.service"

if [[ -z "$ROBOT_HOME" || ! -d "$ROBOT_HOME" ]]; then
  echo "Unable to resolve home directory for user: $ROBOT_USER" >&2
  exit 1
fi
if [[ ! -f "$TEMPLATE" ]]; then
  echo "Supervisor systemd template not found: $TEMPLATE" >&2
  echo "Build and install savo_supervisor first." >&2
  exit 1
fi

TMP_FILE="$(mktemp)"
trap 'rm -f "$TMP_FILE"' EXIT
sed \
  -e "s|@ROBOT_USER@|${ROBOT_USER}|g" \
  -e "s|@ROBOT_HOME@|${ROBOT_HOME}|g" \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  "$TEMPLATE" > "$TMP_FILE"

sudo install -o root -g root -m 0644 "$TMP_FILE" "$TARGET"
sudo systemctl daemon-reload
sudo systemctl enable --now savo-supervisor.service
sudo systemctl --no-pager --full status savo-supervisor.service
