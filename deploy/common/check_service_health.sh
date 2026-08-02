#!/usr/bin/env bash
set -Eeuo pipefail
services=("$@")
((${#services[@]})) || services=(savo_core.service savo_edge.service)
command -v systemctl >/dev/null 2>&1 || { echo "BLOCKED systemd_not_available"; exit 2; }
result=0
for service in "${services[@]}"; do
  if ! systemctl cat "$service" >/dev/null 2>&1; then
    echo "BLOCKED service_not_installed=$service"
    result=2
  elif systemctl is-active --quiet "$service"; then
    echo "PASS service_active=$service"
  else
    echo "FAIL service_inactive=$service state=$(systemctl is-active "$service" 2>/dev/null || true)"
    result=1
  fi
done
exit "$result"
