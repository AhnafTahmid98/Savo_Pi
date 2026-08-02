#!/usr/bin/env bash
set -Eeuo pipefail

usage() { echo "Usage: $0 --user USER --group GROUP --root ABS_PATH --output-dir DIR" >&2; }
user=""
group=""
root=""
output_dir=""
while (($#)); do
  case "$1" in
    --user) user="${2:-}"; shift 2 ;;
    --group) group="${2:-}"; shift 2 ;;
    --root) root="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    *) usage; exit 2 ;;
  esac
done
[[ "$user" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || { echo "Invalid user" >&2; exit 2; }
[[ "$group" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || { echo "Invalid group" >&2; exit 2; }
[[ "$root" == /* && "$root" != / ]] || { echo "Root must be a specific absolute path" >&2; exit 2; }
[[ -n "$output_dir" ]] || { usage; exit 2; }
mkdir -p "$output_dir"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="$root/savo_ws"
templates=(
  "$script_dir/savo_core.service"
  "$script_dir/savo_edge.service"
  "$script_dir/savo.service"
  "$script_dir/savo_mapping.service"
  "$root/savo_ws/src/edge/savo_ui/systemd/savo-ui.service"
)
for template in "${templates[@]}"; do
  [[ -f "$template" ]] || continue
  sed -e "s|@SAVO_USER@|$user|g" \
      -e "s|@SAVO_GROUP@|$group|g" \
      -e "s|@SAVO_ROOT@|$root|g" \
      -e "s|@SAVO_WS@|$ws|g" "$template" >"$output_dir/$(basename "$template")"
done
if command -v systemd-analyze >/dev/null 2>&1; then
  systemd-analyze verify "$output_dir"/*.service
fi
echo "Rendered units only; nothing was installed or enabled."
