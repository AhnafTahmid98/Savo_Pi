#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
source_state="$tmp_dir/source"
restored_state="$tmp_dir/restored"
source_config="$tmp_dir/config"
restored_config="$tmp_dir/restored-config"
mkdir -p "$source_state"/{maps,locations,supervisor} "$source_config"
printf 'map-data\n' >"$source_state/maps/active.yaml"
printf 'override=true\n' >"$source_config/robot-savo.env"
sqlite3 "$source_state/locations/locations.db" 'CREATE TABLE locations(id TEXT PRIMARY KEY); INSERT INTO locations VALUES("test");'
"$script_dir/backup_robot_state.sh" --state-root "$source_state" --config-root "$source_config" --output "$tmp_dir/state.tar.gz"
"$script_dir/restore_robot_state.sh" --archive "$tmp_dir/state.tar.gz" --state-root "$restored_state" --config-root "$restored_config"
cmp "$source_state/maps/active.yaml" "$restored_state/maps/active.yaml"
[[ "$(sqlite3 "$restored_state/locations/locations.db" 'PRAGMA integrity_check;')" == ok ]]
if "$script_dir/restore_robot_state.sh" --archive "$tmp_dir/state.tar.gz" --state-root "$restored_state" --config-root "$restored_config"; then
  echo "restore unexpectedly overwrote state" >&2
  exit 1
fi
echo "PASS backup_restore_test"
