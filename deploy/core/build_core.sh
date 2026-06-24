#!/usr/bin/env bash
# Build Robot Savo core-side ROS 2 packages.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_core.sh
source "${SCRIPT_DIR}/env_core.sh"

CLEAN=0
ALLOW_MISSING=0
RUN_TESTS=0

for arg in "$@"; do
  case "$arg" in
    --clean)
      CLEAN=1
      ;;
    --allow-missing)
      ALLOW_MISSING=1
      ;;
    --test|--tests)
      RUN_TESTS=1
      ;;
    -h|--help)
      cat <<HELP
Usage:
  build_core.sh [--clean] [--allow-missing] [--test]

Options:
  --clean          Remove build/install/log before building.
  --allow-missing  Skip packages not currently present in the workspace.
  --test           Run colcon test after build.

Builds core-side Robot Savo packages from env_core.sh.
HELP
      exit 0
      ;;
    *)
      savo_die "Unknown argument: $arg"
      ;;
  esac
done

main() {
  savo_assert_core_host
  savo_require_cmd colcon
  savo_require_dir "${SAVO_WS}"

  savo_log "Starting Robot Savo core build"
  savo_print_core_env

  cd "${SAVO_WS}"

  savo_source_ros

  if [[ "${CLEAN}" == "1" ]]; then
    savo_log "Cleaning workspace build/install/log"
    rm -rf build install log
  fi

  mapfile -t AVAILABLE_PACKAGES < <(colcon list --names-only)

  BUILD_PACKAGES=()
  MISSING_PACKAGES=()

  for pkg in "${SAVO_CORE_BUILD_PACKAGES[@]}"; do
    if printf '%s\n' "${AVAILABLE_PACKAGES[@]}" | grep -qx "${pkg}"; then
      BUILD_PACKAGES+=("${pkg}")
    else
      MISSING_PACKAGES+=("${pkg}")
    fi
  done

  if [[ "${#MISSING_PACKAGES[@]}" -gt 0 ]]; then
    savo_log "Missing core package(s): ${MISSING_PACKAGES[*]}"

    if [[ "${ALLOW_MISSING}" != "1" ]]; then
      savo_die "Missing package(s). Use --allow-missing only during staged development."
    fi
  fi

  if [[ "${#BUILD_PACKAGES[@]}" -eq 0 ]]; then
    savo_die "No core packages found to build"
  fi

  savo_log "Building packages: ${BUILD_PACKAGES[*]}"

  colcon build \
    --symlink-install \
    --packages-select "${BUILD_PACKAGES[@]}"

  # shellcheck disable=SC1091
  source "${SAVO_WS}/install/setup.bash"

  if [[ "${RUN_TESTS}" == "1" ]]; then
    savo_log "Running tests for core packages"
    colcon test --packages-select "${BUILD_PACKAGES[@]}" --event-handlers console_direct+
    colcon test-result --verbose
  fi

  savo_log "Robot Savo core build completed successfully"
}

main "$@"
