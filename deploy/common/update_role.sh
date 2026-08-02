#!/usr/bin/env bash
set -Eeuo pipefail

role=""
pull=false
ref="${SAVO_UPDATE_REF:-}"
allow_dirty=false
restart=true
while (($#)); do
  case "$1" in
    --role) role="${2:-}"; shift 2 ;;
    --pull) pull=true; shift ;;
    --ref) ref="${2:-}"; shift 2 ;;
    --allow-dirty) allow_dirty=true; shift ;;
    --no-restart) restart=false; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$role" == core || "$role" == edge ]] || { echo "--role must be core or edge" >&2; exit 2; }
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "$script_dir/env_common.sh"
savo_require_dir "$SAVO_ROOT/.git"
savo_require_dir "$SAVO_WS/src"
"$script_dir/check_disk_space.sh" --path "$SAVO_ROOT" --minimum-gib 8

if [[ -n "$(git -C "$SAVO_ROOT" status --porcelain)" ]] && ! $allow_dirty; then
  savo_die "Repository has local changes; commit/stash them or pass --allow-dirty for a build-only update"
fi
if $pull; then
  $allow_dirty && savo_die "--pull and --allow-dirty cannot be combined"
  savo_require_cmd git
  git -C "$SAVO_ROOT" fetch --prune
  if [[ -n "$ref" ]]; then
    git -C "$SAVO_ROOT" merge --ff-only "$ref"
  else
    git -C "$SAVO_ROOT" pull --ff-only
  fi
fi

service="savo_${role}.service"
was_active=false
if command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet "$service"; then
  was_active=true
  sudo systemctl stop "$service"
fi
restart_on_exit() {
  status=$?
  trap - EXIT
  if $was_active && $restart; then sudo systemctl start "$service" || true; fi
  exit "$status"
}
trap restart_on_exit EXIT

timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -d "$SAVO_WS/install" ]]; then
  cp -a "$SAVO_WS/install" "$SAVO_WS/install.previous.$timestamp"
fi
packages=(savo_bringup)
if [[ "$role" == core ]]; then
  packages+=(savo_base savo_control savo_head savo_lidar savo_localization savo_locations savo_mapping savo_nav savo_perception savo_power savo_supervisor)
else
  packages+=(savo_bridge savo_realsense savo_speech savo_ui savo_vo savo_power)
fi
savo_source_ros
cd "$SAVO_WS"
colcon build --packages-up-to "${packages[@]}" --symlink-install --event-handlers console_direct+
colcon test --packages-select "${packages[@]}" --event-handlers console_direct+ --ctest-args --output-on-failure
colcon test-result --verbose
echo "Update completed for $role; previous install retained as install.previous.$timestamp"
