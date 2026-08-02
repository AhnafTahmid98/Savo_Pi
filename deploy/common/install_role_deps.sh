#!/usr/bin/env bash
set -Eeuo pipefail

role="${SAVO_ROLE:-}"
dry_run=false
while (($#)); do
  case "$1" in
    --role) role="${2:-}"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$role" == core || "$role" == edge ]] || { echo "--role must be core or edge" >&2; exit 2; }

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "$script_dir/env_common.sh"
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] || savo_die "Ubuntu 24.04 is required"
arch="$(dpkg --print-architecture)"
[[ "$arch" == arm64 || "${SAVO_ALLOW_PC_INSTALL:-false}" == true ]] || savo_die \
  "Target installs require arm64 (set SAVO_ALLOW_PC_INSTALL=true only for a development PC)"
savo_require_dir "$SAVO_WS/src"
"$script_dir/check_disk_space.sh" --path "$SAVO_ROOT" --minimum-gib 8

log_dir="${SAVO_DEPLOY_LOG_DIR:-$SAVO_ROOT/log/deploy}"
mkdir -p "$log_dir"
log_file="$log_dir/install-${role}-$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "$log_file") 2>&1

apt_packages=(curl git jq rsync sqlite3 chrony python3-rosdep python3-colcon-common-extensions)
if [[ "$role" == core ]]; then
  apt_packages+=(liblgpio-dev libapriltag-dev)
else
  apt_packages+=(libasound2-dev libpocketsphinx-dev libsphinxbase-dev pocketsphinx-en-us)
fi

if $dry_run; then
  printf 'DRY RUN apt packages:'; printf ' %q' "${apt_packages[@]}"; echo
  echo "DRY RUN rosdep check/install role=$role workspace=$SAVO_WS"
  exit 0
fi

savo_require_cmd sudo
sudo apt-get update
sudo apt-get install -y --no-install-recommends "${apt_packages[@]}"
savo_source_ros
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep check --from-paths "$SAVO_WS/src" --ignore-src || true
rosdep install --from-paths "$SAVO_WS/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO"

packages=(savo_bringup)
if [[ "$role" == core ]]; then
  packages+=(savo_base savo_control savo_head savo_lidar savo_localization savo_locations savo_mapping savo_nav savo_perception savo_power savo_supervisor)
else
  packages+=(savo_bridge savo_realsense savo_speech savo_ui savo_vo savo_power)
fi
cd "$SAVO_WS"
colcon build --packages-up-to "${packages[@]}" --symlink-install --event-handlers console_direct+
if [[ "$role" == edge ]]; then
  sudo "$SAVO_ROOT/deploy/edge/prepare_runtime_sockets.sh" \
    --user "${SUDO_USER:-${USER}}" \
    --install-tmpfiles
fi
echo "Dependency installation and role build completed: $role"
