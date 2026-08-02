#!/usr/bin/env bash
set -Eeuo pipefail
path="/var/lib/robot_savo"
minimum_gib=4
while (($#)); do
  case "$1" in
    --path) path="${2:-}"; shift 2 ;;
    --minimum-gib) minimum_gib="${2:-}"; shift 2 ;;
    *) echo "Usage: $0 [--path PATH] [--minimum-gib N]" >&2; exit 2 ;;
  esac
done
[[ "$minimum_gib" =~ ^[0-9]+$ ]] || { echo "minimum GiB must be an integer" >&2; exit 2; }
probe="$path"
while [[ ! -e "$probe" && "$probe" != / ]]; do probe="$(dirname "$probe")"; done
available_kib="$(df -Pk "$probe" | awk 'NR==2 {print $4}')"
required_kib=$((minimum_gib * 1024 * 1024))
if ((available_kib < required_kib)); then
  printf 'FAIL path=%s available_kib=%s required_kib=%s\n' "$path" "$available_kib" "$required_kib"
  exit 1
fi
printf 'PASS path=%s available_kib=%s required_kib=%s\n' "$path" "$available_kib" "$required_kib"
