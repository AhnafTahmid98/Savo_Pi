#!/usr/bin/env bash
set -Eeuo pipefail

[[ "${1:-}" == --standalone ]] || {
  echo "The edge bringup is the production UI authority (SAVO_START_UI=true)." >&2
  echo "Use --standalone only when savo_edge.service is disabled." >&2
  exit 2
}
repo_root="${SAVO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)}"
user="${SAVO_USER:-$(id -un)}"
group="${SAVO_GROUP:-$(id -gn)}"
tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
"$repo_root/deploy/systemd/render_units.sh" --user "$user" --group "$group" \
  --root "$repo_root" --output-dir "$tmp_dir"
if systemctl is-enabled --quiet savo_edge.service 2>/dev/null; then
  echo "Refusing duplicate UI startup: savo_edge.service is enabled" >&2
  exit 2
fi
sudo install -m 0644 "$tmp_dir/savo-ui.service" /etc/systemd/system/savo-ui.service
sudo systemctl daemon-reload
sudo systemctl enable savo-ui.service
echo "Installed standalone UI service; it was not started."
