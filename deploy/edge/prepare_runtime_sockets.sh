#!/usr/bin/env bash
set -Eeuo pipefail

runtime_user="${SUDO_USER:-${USER}}"
shared_group="savomind-bridge"
shared_gid="10001"
install_tmpfiles=false

usage() {
  cat <<'EOF'
Usage: sudo prepare_runtime_sockets.sh [options]

Options:
  --user USER          Native Robot SAVO edge runtime user
  --group GROUP        Shared runtime group (default: savomind-bridge)
  --gid GID            Required group ID (default: 10001)
  --install-tmpfiles   Install boot-persistent tmpfiles configuration
  --help               Show this help

The script creates no sockets and starts no services. It only provisions the
shared group membership and /run/savomind directory used by native savo_speech
and the SavoMind container.
EOF
}

while (($#)); do
  case "$1" in
    --user) runtime_user="${2:-}"; shift 2 ;;
    --group) shared_group="${2:-}"; shift 2 ;;
    --gid) shared_gid="${2:-}"; shift 2 ;;
    --install-tmpfiles) install_tmpfiles=true; shift ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo." >&2; exit 1; }
[[ "$runtime_user" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || {
  echo "Invalid runtime user: $runtime_user" >&2; exit 2;
}
[[ "$shared_group" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || {
  echo "Invalid shared group: $shared_group" >&2; exit 2;
}
[[ "$shared_gid" =~ ^[0-9]+$ ]] || { echo "Invalid GID" >&2; exit 2; }
id "$runtime_user" >/dev/null 2>&1 || {
  echo "Unknown runtime user: $runtime_user" >&2; exit 1;
}

if getent group "$shared_group" >/dev/null 2>&1; then
  actual_gid="$(getent group "$shared_group" | cut -d: -f3)"
  [[ "$actual_gid" == "$shared_gid" ]] || {
    echo "$shared_group uses GID $actual_gid, expected $shared_gid" >&2
    exit 1
  }
else
  groupadd --gid "$shared_gid" "$shared_group"
fi

usermod --append --groups "$shared_group" "$runtime_user"
install -d -m 2770 -o root -g "$shared_group" /run/savomind

if $install_tmpfiles; then
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  source_file="$script_dir/../systemd/robot-savo-tmpfiles.conf"
  target_file="/etc/tmpfiles.d/robot-savo.conf"
  [[ -f "$source_file" ]] || { echo "Missing $source_file" >&2; exit 1; }
  install -m 0644 "$source_file" "$target_file"
  systemd-tmpfiles --create "$target_file"
fi

printf 'Prepared /run/savomind owner=root group=%s mode=2770 user=%s\n' \
  "$shared_group" "$runtime_user"
printf 'Log out and back in before starting native speech if group membership changed.\n'
