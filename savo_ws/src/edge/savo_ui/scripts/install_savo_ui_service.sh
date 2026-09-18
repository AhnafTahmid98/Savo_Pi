#!/usr/bin/env bash
set -Eeuo pipefail

[[ "${1:-}" == --standalone ]] || {
  echo "savo-ui-runtime.service is the production UI authority." >&2
  echo "Use --standalone only when Edge and the production UI runtime are inactive." >&2
  exit 2
}
repo_root="${SAVO_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)}"
user="${SAVO_USER:-$(id -un)}"
group="${SAVO_GROUP:-$(id -gn)}"
tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
"$repo_root/deploy/systemd/render_units.sh" --user "$user" --group "$group" \
  --root "$repo_root" --output-dir "$tmp_dir"
for owner in savo_edge.service savo-ui-runtime.service; do
  if systemctl is-enabled --quiet "$owner" 2>/dev/null || \
    systemctl is-active --quiet "$owner" 2>/dev/null
  then
    echo "Refusing duplicate UI ownership: $owner is enabled or active" >&2
    exit 2
  fi
done
sudo install -d -m 0750 /etc/robot-savo
sudo install -m 0640 "$tmp_dir/robot-savo.paths.env" \
  /etc/robot-savo/robot-savo.paths.env
sudo install -m 0644 "$tmp_dir/savo-ui.service" /etc/systemd/system/savo-ui.service
sudo systemctl daemon-reload
echo "Installed standalone UI service; it was not enabled or started."
echo "After confirming no other UI owner is enabled/active, enable or start it explicitly."
