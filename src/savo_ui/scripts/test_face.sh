#!/usr/bin/env bash
#
# Robot Savo — Face UI test helper
#
# This script publishes sample messages to the savo_ui topics to exercise the
# face UI (INTERACT / NAVIGATE / MAP) without needing the full speech/LLM stack.
#
# Usage (from workspace root):
#   cd ~/Savo_Pi
#
#   # INTERACT demo (default)
#   bash src/savo_ui/scripts/test_face.sh
#   bash src/savo_ui/scripts/test_face.sh interact
#
#   # NAVIGATE demo
#   bash src/savo_ui/scripts/test_face.sh navigate
#
#   # MAP demo
#   bash src/savo_ui/scripts/test_face.sh map
#

# -e : exit on error
# -o pipefail : fail if any command in a pipeline fails
# (no -u here, because ROS setup uses some vars before defining them)
set -eo pipefail

# --------------------------------------------------------------------------- #
# Logging helpers
# --------------------------------------------------------------------------- #
log_info()  { echo "[test_face] [INFO ] $*"; }
log_warn()  { echo "[test_face] [WARN ] $*" >&2; }
log_error() { echo "[test_face] [ERROR] $*" >&2; }

# --------------------------------------------------------------------------- #
# Determine workspace root (assumes script is at src/savo_ui/scripts/)
# --------------------------------------------------------------------------- #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

log_info "Workspace root detected as: ${WS_ROOT}"

# --------------------------------------------------------------------------- #
# Source ROS 2 Jazzy and workspace overlays (best-effort)
# --------------------------------------------------------------------------- #
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/jazzy/setup.bash"
else
  log_warn "/opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?"
fi

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
  log_info "Sourced workspace install/setup.bash"
else
  log_warn "Workspace install/setup.bash not found. Did you run colcon build?"
fi

# Sanity check for ros2 CLI
if ! command -v ros2 >/dev/null 2>&1; then
  log_error "ros2 command not found in PATH. Aborting."
  exit 1
fi

# --------------------------------------------------------------------------- #
# Parse mode argument
# --------------------------------------------------------------------------- #
MODE="${1:-interact}"
MODE_LOWER="$(echo "${MODE}" | tr 'A-Z' 'a-z')"

case "${MODE_LOWER}" in
  interact|i)
    TARGET_MODE="INTERACT"
    ;;
  navigate|nav|n)
    TARGET_MODE="NAVIGATE"
    ;;
  map|m)
    TARGET_MODE="MAP"
    ;;
  *)
    log_error "Unknown mode '${MODE}'. Use: interact | navigate | map"
    exit 1
    ;;
esac

log_info "Selected demo mode: ${TARGET_MODE}"

# --------------------------------------------------------------------------- #
# Ensure display manager node is running
# --------------------------------------------------------------------------- #
if ros2 node list 2>/dev/null | grep -q "/savo_ui_display"; then
  log_info "Found node /savo_ui_display (display_manager_node is running)."
else
  log_warn "Node /savo_ui_display not found."
  log_warn "Start it first in another terminal, for example:"
  log_warn "  cd ${WS_ROOT}"
  log_warn "  source /opt/ros/jazzy/setup.bash"
  log_warn "  source install/setup.bash"
  log_warn "  ros2 run savo_ui display_manager_node \\"
  log_warn "      --ros-args -p screen.width:=800 -p screen.height:=480"
  exit 1
fi

# --------------------------------------------------------------------------- #
# Helper: publish once
# --------------------------------------------------------------------------- #
pub_once() {
  local topic="$1"
  local type="$2"
  local data="$3"
  log_info "Publishing to ${topic}: ${data}"
  ros2 topic pub --once "${topic}" "${type}" "${data}" >/dev/null
}

# --------------------------------------------------------------------------- #
# Publish demo messages per mode
# --------------------------------------------------------------------------- #
case "${TARGET_MODE}" in
  "INTERACT")
    log_info "Running INTERACT face demo..."
    pub_once "/savo_ui/mode"            "std_msgs/String"   "{data: 'INTERACT'}"
    pub_once "/savo_ui/status_text"     "std_msgs/String"   "{data: 'Ready to help you on campus.'}"
    pub_once "/savo_speech/tts_text"    "std_msgs/String"   "{data: 'Hello, I am Robot Savo. How can I guide you today?'}"
    pub_once "/savo_speech/mouth_level" "std_msgs/Float32"  "{data: 0.8}"
    ;;

  "NAVIGATE")
    log_info "Running NAVIGATE face demo..."
    pub_once "/savo_ui/mode"            "std_msgs/String"   "{data: 'NAVIGATE'}"
    pub_once "/savo_ui/status_text"     "std_msgs/String"   "{data: 'Navigation mode ON.'}"
    pub_once "/savo_speech/tts_text"    "std_msgs/String"   "{data: 'Please walk behind me. I will guide you safely to the Info Desk.'}"
    pub_once "/savo_speech/mouth_level" "std_msgs/Float32"  "{data: 0.6}"
    ;;

  "MAP")
    log_info "Running MAP face demo..."
    pub_once "/savo_ui/mode"            "std_msgs/String"   "{data: 'MAP'}"
    pub_once "/savo_ui/status_text"     "std_msgs/String"   "{data: 'Mapping in progress'}"
    pub_once "/savo_speech/tts_text"    "std_msgs/String"   "{data: 'I am now mapping this area. Please keep a little distance while I work.'}"
    pub_once "/savo_speech/mouth_level" "std_msgs/Float32"  "{data: 0.0}"
    ;;
esac

log_info "Done. If the UI is visible, you should see the ${TARGET_MODE} layout now."
