#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "Usage: $0 --user USER --group GROUP --root ABS_PATH --output-dir DIR [--source-root DIR] [--skip-systemd-verify]" >&2
}
user=""
group=""
root=""
output_dir=""
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_root="$(cd "${script_dir}/../.." && pwd)"
systemd_verify=1
while (($#)); do
  case "$1" in
    --user) user="${2:-}"; shift 2 ;;
    --group) group="${2:-}"; shift 2 ;;
    --root) root="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --source-root) source_root="${2:-}"; shift 2 ;;
    --skip-systemd-verify) systemd_verify=0; shift ;;
    *) usage; exit 2 ;;
  esac
done
[[ "$user" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || { echo "Invalid user" >&2; exit 2; }
[[ "$group" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || { echo "Invalid group" >&2; exit 2; }
[[ "$root" == /* && "$root" != / ]] || { echo "Root must be a specific absolute path" >&2; exit 2; }
[[ "$source_root" == /* && -d "$source_root" ]] || {
  echo "Source root must be an existing absolute path" >&2
  exit 2
}
[[ -n "$output_dir" ]] || { usage; exit 2; }
mkdir -p "$output_dir"
ws="$root/savo_ws"
templates=(
  "$source_root/deploy/systemd/savo_core.service"
  "$source_root/deploy/systemd/savo_edge.service"
  "$source_root/deploy/systemd/savo.service"
  "$source_root/deploy/systemd/savo_mapping.service"
  "$source_root/deploy/systemd/savo-location-stack@.service"
  "$source_root/savo_ws/src/edge/savo_ui/systemd/savo-ui-runtime.service"
  "$source_root/savo_ws/src/edge/savo_ui/systemd/savo-ui.service"
)
for template in "${templates[@]}"; do
  [[ -f "$template" ]] || { echo "Required unit template missing: $template" >&2; exit 1; }
  sed -e "s|@SAVO_USER@|$user|g" \
      -e "s|@SAVO_GROUP@|$group|g" \
      -e "s|@SAVO_ROOT@|$root|g" \
      -e "s|@SAVO_WS@|$ws|g" "$template" >"$output_dir/$(basename "$template")"
done
sed -e "s|@SAVO_ROOT@|$root|g" \
    -e "s|@SAVO_WS@|$ws|g" \
    "$source_root/deploy/systemd/robot-savo.paths.env.in" \
    >"$output_dir/robot-savo.paths.env"
sed -e "s|@SAVO_USER@|$user|g" \
    -e "s|@SAVO_GROUP@|$group|g" \
    "$source_root/deploy/systemd/robot-savo-core-tmpfiles.conf.in" \
    >"$output_dir/robot-savo-core-tmpfiles.conf"
if grep -RE '@SAVO_(USER|GROUP|ROOT|WS)@' "$output_dir" >/dev/null; then
  echo "Unresolved systemd rendering placeholder" >&2
  exit 1
fi
if [[ "$systemd_verify" == 1 ]] && command -v systemd-analyze >/dev/null 2>&1; then
  systemd-analyze verify "$output_dir"/*.service
fi
echo "Rendered units only; nothing was installed or enabled."
