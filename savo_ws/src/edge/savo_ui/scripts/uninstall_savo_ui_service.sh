#!/usr/bin/env bash
set -Eeuo pipefail
sudo systemctl disable --now savo-ui.service 2>/dev/null || true
if [[ -e /etc/systemd/system/savo-ui.service ]]; then
  sudo rm -f /etc/systemd/system/savo-ui.service
fi
sudo systemctl daemon-reload
echo "Standalone UI service removed; edge bringup files were not changed."
