#!/usr/bin/env bash
set -Eeuo pipefail

state_root="${SAVO_STATE_ROOT:-/var/lib/robot_savo}"
config_root="${SAVO_CONFIG_ROOT:-/etc/robot-savo}"
output=""
while (($#)); do
  case "$1" in
    --state-root) state_root="${2:-}"; shift 2 ;;
    --config-root) config_root="${2:-}"; shift 2 ;;
    --output) output="${2:-}"; shift 2 ;;
    *) echo "Usage: $0 [--state-root DIR] [--config-root DIR] --output ARCHIVE" >&2; exit 2 ;;
  esac
done
[[ -n "$output" && "$output" != / && "$output" != "$state_root" ]] || { echo "Unsafe or missing output path" >&2; exit 2; }
[[ -d "$state_root" ]] || { echo "State root missing: $state_root" >&2; exit 2; }
for required in maps locations supervisor; do
  [[ -d "$state_root/$required" ]] || { echo "Required state directory missing: $required" >&2; exit 2; }
done
tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
mkdir -p "$tmp_dir/robot_savo/state" "$tmp_dir/robot_savo/config"
for name in maps locations supervisor; do
  cp -a "$state_root/$name" "$tmp_dir/robot_savo/state/"
done
if [[ -d "$config_root" ]]; then cp -a "$config_root/." "$tmp_dir/robot_savo/config/"; fi
cat >"$tmp_dir/robot_savo/backup-metadata.txt" <<EOF
schema_version=1
created_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
hostname=$(hostname)
state_root=$state_root
config_root=$config_root
EOF
(
  cd "$tmp_dir/robot_savo"
  find state config -type f -print0 | sort -z | xargs -0 sha256sum >SHA256SUMS
)
mkdir -p "$(dirname "$output")"
tar -C "$tmp_dir" --numeric-owner --owner=0 --group=0 -czf "$output" robot_savo
sha256sum "$output" >"$output.sha256"
echo "PASS backup=$output"
