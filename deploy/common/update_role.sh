#!/usr/bin/env bash
set -Eeuo pipefail

role=""
pull=false
ref="${SAVO_UPDATE_REF:-}"
allow_dirty=false
restart=true

while (($#)); do
  case "$1" in
    --role)
      role="${2:-}"
      shift 2
      ;;
    --pull)
      pull=true
      shift
      ;;
    --ref)
      ref="${2:-}"
      shift 2
      ;;
    --allow-dirty)
      allow_dirty=true
      shift
      ;;
    --no-restart)
      restart=false
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

savo_require_dir "$SAVO_ROOT/.git"
savo_require_dir "$SAVO_WS/src"
savo_require_cmd colcon

"$script_dir/check_disk_space.sh" \
  --path "$SAVO_ROOT" \
  --minimum-gib 8

if [[ -n "$(git -C "$SAVO_ROOT" status --porcelain)" ]] && ! $allow_dirty; then
  savo_die \
    "Repository has local changes; commit/stash them or pass --allow-dirty for a build-only update"
fi

if $pull; then
  $allow_dirty && \
    savo_die "--pull and --allow-dirty cannot be combined"

  savo_require_cmd git
  git -C "$SAVO_ROOT" fetch --prune

  if [[ -n "$ref" ]]; then
    git -C "$SAVO_ROOT" merge --ff-only "$ref"
  else
    git -C "$SAVO_ROOT" pull --ff-only
  fi
fi

# Use the same authoritative package lists as the normal role build scripts.
if [[ "$role" == core ]]; then
  # shellcheck source=../core/env_core.sh
  source "$SAVO_ROOT/deploy/core/env_core.sh"
  packages=("${SAVO_CORE_BUILD_PACKAGES[@]}")
else
  # shellcheck source=../edge/env_edge.sh
  source "$SAVO_ROOT/deploy/edge/env_edge.sh"
  packages=("${SAVO_EDGE_BUILD_PACKAGES[@]}")
fi

savo_source_ros
cd "$SAVO_WS"

# Confirm that every package required for this role exists before stopping
# the currently running robot service.
mapfile -t available_packages < <(colcon list --names-only)

missing_packages=()

for package in "${packages[@]}"; do
  if ! printf '%s\n' "${available_packages[@]}" | grep -qx "$package"; then
    missing_packages+=("$package")
  fi
done

if ((${#missing_packages[@]})); then
  savo_die "Missing $role package(s): ${missing_packages[*]}"
fi

service="savo_${role}.service"
was_active=false

if command -v systemctl >/dev/null 2>&1 &&
   systemctl is-active --quiet "$service" 2>/dev/null; then
  was_active=true
  sudo systemctl stop "$service"
fi

restart_on_exit() {
  status=$?
  trap - EXIT

  if $was_active && $restart; then
    sudo systemctl start "$service" || true
  fi

  exit "$status"
}

trap restart_on_exit EXIT

# Build and test in an isolated release directory. The active install is not
# modified unless every selected package builds and all tests succeed.
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
release_root="$SAVO_WS/.releases/${role}-${timestamp}-$$"
build_base="$release_root/build"
install_base="$release_root/install"

mkdir -p "$release_root"

savo_log "Staging $role release in $release_root"

colcon build \
  --build-base "$build_base" \
  --install-base "$install_base" \
  --symlink-install \
  --packages-select "${packages[@]}" \
  --event-handlers console_direct+

# Source the staged installation before running its tests.
# shellcheck disable=SC1090
set +u
source "$install_base/setup.bash"
set -u

colcon test \
  --build-base "$build_base" \
  --install-base "$install_base" \
  --packages-select "${packages[@]}" \
  --return-code-on-test-failure \
  --event-handlers console_direct+ \
  --ctest-args --output-on-failure

colcon test-result \
  --test-result-base "$build_base" \
  --verbose

current_install="$SAVO_WS/install"
previous_install="$SAVO_WS/install.previous.$timestamp"

if [[ -e "$current_install" || -L "$current_install" ]]; then
  mv "$current_install" "$previous_install"
fi

# Activate the completed release only after its build and tests pass.
next_link="$SAVO_WS/.install.next.$$"
ln -s "$install_base" "$next_link"

if ! mv -T "$next_link" "$current_install"; then
  rm -f "$next_link"

  if [[ -e "$previous_install" || -L "$previous_install" ]]; then
    mv "$previous_install" "$current_install"
  fi

  savo_die "Failed to activate staged $role release"
fi

if [[ -e "$previous_install" || -L "$previous_install" ]]; then
  savo_log "Previous install retained at $previous_install"
else
  savo_log "No previous install existed"
fi

savo_log "Activated $role release: $install_base"
