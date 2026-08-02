#!/usr/bin/env bash
set -Eeuo pipefail

archive=""
state_root="${SAVO_STATE_ROOT:-/var/lib/robot_savo}"
config_root="${SAVO_CONFIG_ROOT:-/etc/robot-savo}"
overwrite=false

sqlite_integrity_check() {
  local database="$1"
  if command -v sqlite3 >/dev/null 2>&1; then
    [[ "$(sqlite3 "$database" 'PRAGMA integrity_check;')" == ok ]]
    return
  fi
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$database" <<'PYSQL'
import sqlite3
import sys
with sqlite3.connect(sys.argv[1]) as connection:
    row = connection.execute("PRAGMA integrity_check").fetchone()
raise SystemExit(0 if row and row[0] == "ok" else 1)
PYSQL
    return
  fi
  echo "sqlite3 CLI or Python sqlite3 module is required to validate $database" >&2
  return 2
}

while (($#)); do
  case "$1" in
    --archive) archive="${2:-}"; shift 2 ;;
    --state-root) state_root="${2:-}"; shift 2 ;;
    --config-root) config_root="${2:-}"; shift 2 ;;
    --overwrite) overwrite=true; shift ;;
    *) echo "Usage: $0 --archive FILE [--state-root DIR] [--config-root DIR] [--overwrite]" >&2; exit 2 ;;
  esac
done
[[ -f "$archive" ]] || { echo "Archive missing: $archive" >&2; exit 2; }
[[ "$state_root" == /* && "$state_root" != / ]] || { echo "Unsafe state root" >&2; exit 2; }
[[ "$config_root" == /* && "$config_root" != / ]] || { echo "Unsafe config root" >&2; exit 2; }
if [[ -f "$archive.sha256" ]]; then (cd "$(dirname "$archive")" && sha256sum -c "$(basename "$archive").sha256"); fi

while IFS= read -r member; do
  [[ "$member" != /* && "$member" != *"../"* && "$member" != ".." ]] || {
    echo "Archive path traversal rejected: $member" >&2
    exit 1
  }
  [[ "$member" == robot_savo || "$member" == robot_savo/* ]] || {
    echo "Unexpected archive root: $member" >&2
    exit 1
  }
done < <(tar -tzf "$archive")

tmp_dir="$(mktemp -d)"
trap 'rm -rf -- "$tmp_dir"' EXIT
tar --no-same-owner --no-same-permissions -xzf "$archive" -C "$tmp_dir"
payload="$tmp_dir/robot_savo"
[[ -f "$payload/SHA256SUMS" ]] || { echo "Missing SHA256SUMS" >&2; exit 1; }
[[ ! -L "$payload" ]] || { echo "Symlink payload rejected" >&2; exit 1; }
if find "$payload" -type l -print -quit | grep -q .; then echo "Archive symlinks rejected" >&2; exit 1; fi
(
  cd "$payload"
  sha256sum -c SHA256SUMS
)
for required in maps locations supervisor; do
  [[ -d "$payload/state/$required" ]] || { echo "Missing payload directory: $required" >&2; exit 1; }
done
while IFS= read -r -d '' database; do
  if ! sqlite_integrity_check "$database"; then
    code=$?
    if [[ "$code" -eq 2 ]]; then
      exit 2
    fi
    echo "SQLite integrity failed: $database" >&2
    exit 1
  fi
done < <(find "$payload/state" -type f -name '*.db' -print0)
while IFS= read -r -d '' sums; do (cd "$(dirname "$sums")" && sha256sum -c "$(basename "$sums")"); done \
  < <(find "$payload/state" -type f -name '*.sha256' -print0)

if [[ -d "$state_root" && -n "$(find "$state_root" -mindepth 1 -print -quit 2>/dev/null)" ]] && ! $overwrite; then
  echo "Refusing to overwrite nonempty state root without --overwrite" >&2
  exit 2
fi
if $overwrite && [[ -d "$state_root" ]]; then
  previous="${state_root}.pre-restore.$(date -u +%Y%m%dT%H%M%SZ)"
  cp -a "$state_root" "$previous"
  echo "Previous state preserved at $previous"
fi
mkdir -p "$state_root" "$config_root"
cp -a "$payload/state/." "$state_root/"
if [[ -d "$payload/config" ]]; then cp -a "$payload/config/." "$config_root/"; fi
expected_user="${SAVO_USER:-$(id -un)}"
expected_group="${SAVO_GROUP:-$(id -gn)}"
if [[ "$(id -u)" -eq 0 ]]; then chown -R "$expected_user:$expected_group" "$state_root" "$config_root"; fi
echo "PASS restored_state=$state_root restored_config=$config_root owner=$expected_user:$expected_group"
