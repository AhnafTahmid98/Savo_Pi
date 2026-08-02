#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  echo "Usage: $0 --role core|edge --env FILE --output-dir DIR" >&2
}

role=""
env_file=""
output_dir=""
while (($#)); do
  case "$1" in
    --role) role="${2:-}"; shift 2 ;;
    --env) env_file="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    *) usage; exit 2 ;;
  esac
done

[[ "$role" == core || "$role" == edge ]] || { usage; exit 2; }
[[ -f "$env_file" ]] || { echo "Missing environment file: $env_file" >&2; exit 2; }
[[ -n "$output_dir" ]] || { usage; exit 2; }

set -a
# shellcheck disable=SC1090
source "$env_file"
set +a

required=(CORE_ETH_IFACE EDGE_ETH_IFACE CORE_WIFI_IFACE EDGE_WIFI_IFACE CORE_ETH_IP EDGE_ETH_IP LINK_PREFIX LINK_SUBNET WIFI_SSID WIFI_PASSWORD)
for name in "${required[@]}"; do
  [[ -n "${!name:-}" ]] || { echo "Missing $name in $env_file" >&2; exit 2; }
done
[[ "$WIFI_SSID" != CHANGE_ME && "$WIFI_PASSWORD" != CHANGE_ME ]] || {
  echo "Refusing to render placeholder Wi-Fi credentials" >&2
  exit 2
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$output_dir"
template="$script_dir/${role}_netplan.yaml"
chrony_template="$script_dir/chrony_${role}.conf"

escape_sed() { printf '%s' "$1" | sed 's/[&|]/\\&/g'; }
render() {
  sed \
    -e "s|@CORE_ETH_IFACE@|$(escape_sed "$CORE_ETH_IFACE")|g" \
    -e "s|@EDGE_ETH_IFACE@|$(escape_sed "$EDGE_ETH_IFACE")|g" \
    -e "s|@CORE_WIFI_IFACE@|$(escape_sed "$CORE_WIFI_IFACE")|g" \
    -e "s|@EDGE_WIFI_IFACE@|$(escape_sed "$EDGE_WIFI_IFACE")|g" \
    -e "s|@CORE_ETH_IP@|$(escape_sed "$CORE_ETH_IP")|g" \
    -e "s|@EDGE_ETH_IP@|$(escape_sed "$EDGE_ETH_IP")|g" \
    -e "s|@LINK_PREFIX@|$(escape_sed "$LINK_PREFIX")|g" \
    -e "s|@LINK_SUBNET@|$(escape_sed "$LINK_SUBNET")|g" \
    -e "s|@WIFI_SSID@|$(escape_sed "$WIFI_SSID")|g" \
    -e "s|@WIFI_PASSWORD@|$(escape_sed "$WIFI_PASSWORD")|g" "$1"
}

render "$template" >"$output_dir/50-robot-savo-${role}.yaml"
render "$chrony_template" >"$output_dir/chrony-${role}.conf"
chmod 600 "$output_dir/50-robot-savo-${role}.yaml"

if command -v netplan >/dev/null 2>&1; then
  root_dir="$(mktemp -d)"
  netplan_error="$root_dir/netplan.error"
  trap 'rm -rf -- "$root_dir"' EXIT
  mkdir -p "$root_dir/etc/netplan"
  cp "$output_dir/50-robot-savo-${role}.yaml" "$root_dir/etc/netplan/"
  if ! netplan generate --root-dir "$root_dir" 2>"$netplan_error"; then
    if grep -qiE 'dbus|failed to connect to bus|failed to communicate' "$netplan_error"; then
      echo "BLOCKED: netplan schema generator requires a system D-Bus in this environment" >&2
    else
      cat "$netplan_error" >&2
      exit 1
    fi
  fi
fi

echo "Rendered configuration only; nothing was applied."
