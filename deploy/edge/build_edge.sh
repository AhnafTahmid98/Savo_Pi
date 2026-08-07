#!/usr/bin/env bash
# Build Robot Savo edge-side ROS 2 packages.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_edge.sh
source "${SCRIPT_DIR}/env_edge.sh"

CLEAN=0
RUN_TESTS=0

for arg in "$@"; do
  case "$arg" in
    --clean)
      CLEAN=1
      ;;
    --test|--tests)
      RUN_TESTS=1
      ;;
    -h|--help)
      cat <<HELP
Usage: build_edge.sh [--clean] [--test]
HELP
      exit 0
      ;;
    *)
      savo_die "Unknown argument: $arg"
      ;;
  esac
done

main() {
  savo_assert_edge_host
  savo_require_cmd colcon
  savo_require_dir "${SAVO_WS}"

  cd "${SAVO_WS}"
  savo_source_ros

  if [[ "${CLEAN}" == "1" ]]; then
    rm -rf build install log
  fi

  mapfile -t available < <(colcon list --names-only)
  for package in "${SAVO_EDGE_BUILD_PACKAGES[@]}"; do
    printf '%s\n' "${available[@]}" | grep -qx "${package}" || \
      savo_die "Missing edge package: ${package}"
  done

  colcon build \
    --symlink-install \
    --packages-select "${SAVO_EDGE_BUILD_PACKAGES[@]}"

  # Colcon-generated setup scripts may reference optional variables such as
  # COLCON_TRACE. Temporarily disable nounset while sourcing the overlay.
  # shellcheck disable=SC1091
  set +u
  source "${SAVO_WS}/install/setup.bash"
  set -u

  if [[ "${RUN_TESTS}" == "1" ]]; then
    colcon test \
      --packages-select "${SAVO_EDGE_BUILD_PACKAGES[@]}" \
      --return-code-on-test-failure \
      --event-handlers console_direct+
    colcon test-result --verbose
  fi
}

main "$@"
