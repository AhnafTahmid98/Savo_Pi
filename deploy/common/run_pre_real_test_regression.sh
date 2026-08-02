#!/usr/bin/env bash
set -Eeuo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
workspace="${SAVO_WORKSPACE:-$root/savo_ws}"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
clean_affected=false
skip_rosdep=false

usage() {
  cat <<'USAGE'
Usage: run_pre_real_test_regression.sh [options]

Hardware-free build and regression gate for the Robot SAVO packages changed by
pre-real-test closure. The script never launches robot nodes or publishes ROS
commands.

Options:
  --workspace PATH   ROS workspace (default: <repository>/savo_ws)
  --ros-setup PATH   ROS setup script (default: /opt/ros/jazzy/setup.bash)
  --clean-affected   Remove build/install products for affected packages first
  --skip-rosdep      Skip rosdep check (not recommended for final validation)
  --help             Show this help
USAGE
}

while (($#)); do
  case "$1" in
    --workspace) workspace="${2:-}"; shift 2 ;;
    --ros-setup) ros_setup="${2:-}"; shift 2 ;;
    --clean-affected) clean_affected=true; shift ;;
    --skip-rosdep) skip_rosdep=true; shift ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "$ros_setup" ]] || {
  echo "BLOCKED: ROS setup not found: $ros_setup" >&2
  exit 2
}
[[ -d "$workspace/src" ]] || {
  echo "FAIL: workspace source directory missing: $workspace/src" >&2
  exit 1
}
command -v colcon >/dev/null 2>&1 || {
  echo "BLOCKED: colcon is unavailable" >&2
  exit 2
}

# ROS-generated setup scripts may reference variables before defining them.
# Temporarily disable nounset while sourcing, then restore fail-closed mode.
set +u
# shellcheck disable=SC1090
source "$ros_setup"
if [[ -f "$workspace/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$workspace/install/setup.bash"
fi
set -u

packages=(
  savo_msgs
  savo_localization
  savo_bridge
  savo_speech
  savo_ui
  savo_mapping
  savo_nav
  savo_bringup
  savo_observer
)

cd "$root"
bash deploy/common/validate_full_bringup.sh
bash deploy/observer/validate_observer.sh

if [[ -d .git ]]; then
  git diff --check
else
  echo "NOTICE: Git metadata absent; run git diff --check in the live checkout"
fi

if ! $skip_rosdep; then
  command -v rosdep >/dev/null 2>&1 || {
    echo "BLOCKED: rosdep is unavailable" >&2
    exit 2
  }
  rosdep check --from-paths "$workspace/src" --ignore-src
fi

if $clean_affected; then
  for package in "${packages[@]}"; do
    rm -rf "$workspace/build/$package" "$workspace/install/$package"
  done
fi

cd "$workspace"
colcon build \
  --packages-up-to "${packages[@]}" \
  --symlink-install \
  --event-handlers console_direct+

# ROS-generated setup scripts are not guaranteed to be nounset-safe.
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

colcon test \
  --packages-select "${packages[@]}" \
  --event-handlers console_direct+ \
  --ctest-args --output-on-failure

colcon test-result --verbose

for launch_file in \
  robot_bringup.launch.py \
  core_bringup.launch.py \
  edge_bringup.launch.py \
  autonomous_mapping.launch.py \
  saved_map_navigation.launch.py; do
  ros2 launch savo_bringup "$launch_file" --show-args >/dev/null
  printf 'PASS launch arguments: %s\n' "$launch_file"
done

for launch_file in \
  observer.launch.py \
  rviz_observer.launch.py \
  dashboard_observer.launch.py \
  full_observer.launch.py; do
  ros2 launch savo_observer "$launch_file" --show-args >/dev/null
  printf 'PASS launch arguments: %s\n' "$launch_file"
done

cd "$root"
set +e
bash deploy/common/validate_pre_real_test_readiness.sh
readiness_status=$?
set -e
if ((readiness_status == 1)); then
  echo "FAIL: pre-real-test readiness validator reported a repository error" >&2
  exit 1
fi
if ((readiness_status != 0 && readiness_status != 2)); then
  echo "FAIL: unexpected readiness validator exit: $readiness_status" >&2
  exit 1
fi

echo "PASS: Robot SAVO hardware-free pre-real-test regression"
if ((readiness_status == 2)); then
  echo "NOTICE: physical/external blockers remain; do not begin movement yet"
fi
