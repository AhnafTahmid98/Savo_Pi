#!/usr/bin/env bash
set -Eeuo pipefail
state_root="${SAVO_STATE_ROOT:-/var/lib/robot_savo}"
minimum_gib="${SAVO_MINIMUM_FREE_GIB:-4}"
while (($#)); do
  case "$1" in
    --state-root) state_root="${2:-}"; shift 2 ;;
    --minimum-gib) minimum_gib="${2:-}"; shift 2 ;;
    *) echo "Usage: $0 [--state-root DIR] [--minimum-gib N]" >&2; exit 2 ;;
  esac
done
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$script_dir/check_disk_space.sh" --path "$state_root" --minimum-gib "$minimum_gib"
result=0
for name in maps locations supervisor; do
  path="$state_root/$name"
  if [[ ! -d "$path" ]]; then echo "BLOCKED missing_directory=$path"; result=2
  elif [[ ! -w "$path" ]]; then echo "FAIL not_writable=$path"; result=1
  else echo "PASS writable=$path"; fi
done
while IFS= read -r -d '' database; do
  if ! command -v sqlite3 >/dev/null 2>&1; then echo "BLOCKED sqlite3_missing database=$database"; result=2; break; fi
  if [[ "$(sqlite3 "$database" 'PRAGMA integrity_check;' 2>/dev/null)" == ok ]]; then echo "PASS sqlite=$database"
  else echo "FAIL sqlite=$database"; result=1; fi
done < <(find "$state_root" -type f -name '*.db' -print0 2>/dev/null)
exit "$result"
