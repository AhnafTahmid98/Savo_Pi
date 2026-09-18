#!/usr/bin/env bash
# Install one verified Robot Savo service profile without activating it.
# Run as root for the default /etc destinations. This script deliberately
# does not enable, start, stop, or restart services.

set -Eeuo pipefail

usage() {
  echo "Usage: $0 --profile core|edge|mapping|generic|location --user USER --group GROUP --root ABS_PATH [--source-root DIR] [--unit-dir DIR] [--config-dir DIR] [--tmpfiles-dir DIR] [--skip-daemon-reload]" >&2
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_root="$(cd "${script_dir}/../.." && pwd)"
profile=""
user=""
group=""
root=""
unit_dir="/etc/systemd/system"
config_dir="/etc/robot-savo"
tmpfiles_dir="/etc/tmpfiles.d"
daemon_reload=1

while (($#)); do
  case "$1" in
    --profile) profile="${2:-}"; shift 2 ;;
    --user) user="${2:-}"; shift 2 ;;
    --group) group="${2:-}"; shift 2 ;;
    --root) root="${2:-}"; shift 2 ;;
    --source-root) source_root="${2:-}"; shift 2 ;;
    --unit-dir) unit_dir="${2:-}"; shift 2 ;;
    --config-dir) config_dir="${2:-}"; shift 2 ;;
    --tmpfiles-dir) tmpfiles_dir="${2:-}"; shift 2 ;;
    --skip-daemon-reload) daemon_reload=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage; exit 2 ;;
  esac
done

case "$profile" in
  core) selected=("savo_core.service") ;;
  edge) selected=("savo_edge.service" "savo-ui-runtime.service") ;;
  mapping) selected=("savo_mapping.service") ;;
  generic) selected=("savo.service") ;;
  location) selected=("savo-location-stack@.service") ;;
  *) usage; exit 2 ;;
esac

[[ "$unit_dir" == /* && "$unit_dir" != / ]] || {
  echo "Unit directory must be a specific absolute path" >&2
  exit 2
}
[[ "$config_dir" == /* && "$config_dir" != / ]] || {
  echo "Config directory must be a specific absolute path" >&2
  exit 2
}
[[ "$tmpfiles_dir" == /* && "$tmpfiles_dir" != / ]] || {
  echo "Tmpfiles directory must be a specific absolute path" >&2
  exit 2
}

temporary="$(mktemp -d)"
trap 'rm -rf "$temporary"' EXIT
"${script_dir}/render_units.sh" \
  --user "$user" \
  --group "$group" \
  --root "$root" \
  --source-root "$source_root" \
  --output-dir "$temporary"

install -d -m 0755 "$unit_dir"
install -d -m 0750 "$config_dir"
for unit in "${selected[@]}"; do
  install -m 0644 "$temporary/$unit" "$unit_dir/$unit"
done
install -m 0640 "$temporary/robot-savo.paths.env" "$config_dir/robot-savo.paths.env"

case "$profile" in
  core|mapping|generic)
    install -d -m 0755 "$tmpfiles_dir"
    core_tmpfiles="$tmpfiles_dir/robot-savo-core.conf"
    install -m 0644 "$temporary/robot-savo-core-tmpfiles.conf" "$core_tmpfiles"
    systemd-tmpfiles --create "$core_tmpfiles"
    ;;
esac

if [[ "$daemon_reload" == 1 ]]; then
  systemctl daemon-reload
fi

printf 'Installed profile=%s units=%s\n' "$profile" "${selected[*]}"
echo "No service was enabled or started. Activate the selected owner explicitly after preflight."
