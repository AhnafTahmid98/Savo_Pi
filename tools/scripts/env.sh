#!/usr/bin/env bash
# =============================================================================
# Robot Savo — ROS 2 Jazzy environment setup
#
# Location (in repo):
#   savo_ws/tools/scripts/env.sh
#
# Usage (in every new terminal on the Pi):
#   cd ~/Savo_Pi/savo_ws
#   source tools/scripts/env.sh
#
# This script:
#   - Sources /opt/ros/jazzy
#   - Sources this workspace's install/setup.bash
#   - Fixes PATH so ~/.local/bin tools (pip --user) are usable
#   - Prints a short status line
# =============================================================================

# Internal helper: print error and stop correctly whether sourced or executed
_die() {
  echo "[env.sh] ERROR: $*" 1>&2
  # If sourced, `return` works; if executed, `exit` is used.
  return 1 2>/dev/null || exit 1
}

# -----------------------------------------------------------------------------
# 1. Resolve workspace root (two levels up from this script)
#    tools/scripts/env.sh -> workspace root = ../..
# -----------------------------------------------------------------------------
# BASH_SOURCE[0] works when sourced; $0 is fallback if executed.
SCRIPT_PATH="${BASH_SOURCE[0]:-$0}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Allow overriding from outside if needed:
#   export WS_ROOT=/some/other/path; source env.sh
WS_ROOT="${WS_ROOT_OVERRIDE:-$WS_ROOT}"

if [ ! -d "$WS_ROOT" ]; then
  _die "Workspace root not found at '$WS_ROOT'. Check your repo layout."
fi

# -----------------------------------------------------------------------------
# 2. Source ROS 2 Jazzy base
# -----------------------------------------------------------------------------
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [ -f "$ROS_SETUP" ]; then
  # shellcheck source=/dev/null
  source "$ROS_SETUP"
else
  _die "ROS 2 Jazzy not found at $ROS_SETUP. Is ROS installed?"
fi

# -----------------------------------------------------------------------------
# 3. Source this workspace's overlay (install/setup.bash)
# -----------------------------------------------------------------------------
WS_INSTALL_SETUP="$WS_ROOT/install/setup.bash"

if [ -f "$WS_INSTALL_SETUP" ]; then
  # shellcheck source=/dev/null
  source "$WS_INSTALL_SETUP"
else
  echo "[env.sh] WARNING: $WS_INSTALL_SETUP not found."
  echo "[env.sh]          Run 'colcon build' in $WS_ROOT to create it."
fi

# -----------------------------------------------------------------------------
# 4. Quality-of-life: ensure ~/.local/bin is on PATH (for pip --user tools)
# -----------------------------------------------------------------------------
if [ -d "$HOME/.local/bin" ]; then
  case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *)
      export PATH="$HOME/.local/bin:$PATH"
      ;;
  esac
fi`

# -----------------------------------------------------------------------------
# 5. Optional debug/diagnostics env (safe defaults)
# -----------------------------------------------------------------------------
# Show Python warnings (useful during development; comment out if annoying)
export PYTHONWARNINGS=${PYTHONWARNINGS:-default}

# You can set a fixed ROS_DOMAIN_ID for Robot Savo here if you want:
# export ROS_DOMAIN_ID=42

# -----------------------------------------------------------------------------
# 6. Final status message
# -----------------------------------------------------------------------------
echo "[env.sh] Loaded Robot Savo ROS environment"
echo "[env.sh]   WS_ROOT     = $WS_ROOT"
echo "[env.sh]   ROS_DISTRO  = ${ROS_DISTRO:-unknown}"
