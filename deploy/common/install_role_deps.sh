#!/usr/bin/env bash
set -Eeuo pipefail

role="${SAVO_ROLE:-}"
dry_run=false

while (($#)); do
  case "$1" in
    --role)
      role="${2:-}"
      shift 2
      ;;
    --dry-run)
      dry_run=true
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

[[ "$role" == core || "$role" == edge ]] || {
  echo "--role must be core or edge" >&2
  exit 2
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=env_common.sh
source "$script_dir/env_common.sh"

source /etc/os-release

[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] || \
  savo_die "Ubuntu 24.04 is required"

arch="$(dpkg --print-architecture)"

[[ "$arch" == arm64 || "${SAVO_ALLOW_PC_INSTALL:-false}" == true ]] || \
  savo_die \
    "Target installs require arm64 (set SAVO_ALLOW_PC_INSTALL=true only for a development PC)"

savo_require_dir "$SAVO_WS/src"

# Use the same authoritative package arrays as the role build scripts.
if [[ "$role" == core ]]; then
  # shellcheck source=../core/env_core.sh
  source "$SAVO_ROOT/deploy/core/env_core.sh"
  packages=("${SAVO_CORE_BUILD_PACKAGES[@]}")
else
  # shellcheck source=../edge/env_edge.sh
  source "$SAVO_ROOT/deploy/edge/env_edge.sh"
  packages=("${SAVO_EDGE_BUILD_PACKAGES[@]}")
fi

log_dir="${SAVO_DEPLOY_LOG_DIR:-$SAVO_ROOT/log/deploy}"
mkdir -p "$log_dir"

log_file="$log_dir/install-${role}-$(date -u +%Y%m%dT%H%M%SZ).log"
exec > >(tee -a "$log_file") 2>&1

apt_packages=(
  curl
  git
  jq
  rsync
  sqlite3
  chrony
  python3-rosdep
  python3-colcon-common-extensions
)

if [[ "$role" == core ]]; then
  apt_packages+=(
    liblgpio-dev
    libapriltag-dev
  )
else
  apt_packages+=(
    libasound2-dev
    libpocketsphinx-dev
    libsphinxbase-dev
    pocketsphinx-en-us
  )
fi

if $dry_run; then
  printf 'DRY RUN apt packages:'
  printf ' %q' "${apt_packages[@]}"
  echo

  printf 'DRY RUN ROS packages for role %s:' "$role"
  printf ' %q' "${packages[@]}"
  echo

  echo "DRY RUN rosdep scope: selected role package directories only"
  echo "DRY RUN colcon build: --packages-select"
  exit 0
fi

# Real installation and building require sufficient free disk space.
"$script_dir/check_disk_space.sh" \
  --path "$SAVO_ROOT" \
  --minimum-gib 8

savo_require_cmd sudo

sudo apt-get update

sudo apt-get install \
  -y \
  --no-install-recommends \
  "${apt_packages[@]}"

savo_source_ros
savo_require_cmd rosdep
savo_require_cmd colcon

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep update

cd "$SAVO_WS"

# Discover all local workspace packages and the source directory belonging
# to each package selected for this computer role.
mapfile -t package_rows < <(colcon list)
mapfile -t all_local_packages < <(colcon list --names-only)

missing_packages=()
role_paths=()

for package in "${packages[@]}"; do
  package_path="$(
    printf '%s\n' "${package_rows[@]}" |
      awk -v requested="$package" '$1 == requested {print $2; exit}'
  )"

  if [[ -z "$package_path" ]]; then
    missing_packages+=("$package")
    continue
  fi

  if [[ "$package_path" != /* ]]; then
    package_path="$SAVO_WS/$package_path"
  fi

  role_paths+=("$package_path")
done

if ((${#missing_packages[@]})); then
  savo_die "Missing $role package(s): ${missing_packages[*]}"
fi

# Skip every package that exists inside this workspace. Those dependencies
# are built by colcon rather than installed as operating-system packages.
local_package_keys="${all_local_packages[*]}"

rosdep check \
  --from-paths "${role_paths[@]}" \
  --ignore-src \
  --skip-keys "$local_package_keys" || true

rosdep install \
  --from-paths "${role_paths[@]}" \
  --ignore-src \
  --skip-keys "$local_package_keys" \
  -r \
  -y \
  --rosdistro "$ROS_DISTRO"

colcon build \
  --packages-select "${packages[@]}" \
  --symlink-install \
  --event-handlers console_direct+

if [[ "$role" == edge ]]; then
  sudo "$SAVO_ROOT/deploy/edge/prepare_runtime_sockets.sh" \
    --user "${SUDO_USER:-${USER}}" \
    --install-tmpfiles
fi

echo "Dependency installation and role build completed: $role"
