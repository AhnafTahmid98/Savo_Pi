#!/usr/bin/env bash
# =============================================================================
# Robot Savo — ROS 2 Jazzy environment setup
#
# Location (in repo):
#   tools/scripts/env.sh
#
# Usage (in every new terminal on the Pi or PC):
#   cd ~/Savo_Pi
#   source tools/scripts/env.sh
# =============================================================================

_die() {
  echo "[env.sh] ERROR: $*" 1>&2
  return 1 2>/dev/null || exit 1
}

SCRIPT_PATH="${BASH_SOURCE[0]:-$0}"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS_ROOT="${WS_ROOT_OVERRIDE:-$WS_ROOT}"

if [ ! -d "$WS_ROOT" ]; then
  _die "Workspace root not found at '$WS_ROOT'. Check your repo layout."
fi

# 2. Source ROS 2 Jazzy base
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
else
  _die "ROS 2 Jazzy not found at $ROS_SETUP. Is ROS installed?"
fi

# 3. Source this workspace's overlay (install/setup.bash)
WS_INSTALL_SETUP="$WS_ROOT/install/setup.bash"
if [ -f "$WS_INSTALL_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$WS_INSTALL_SETUP"
else
  echo "[env.sh] WARNING: $WS_INSTALL_SETUP not found."
  echo "[env.sh]          Run 'colcon build' in $WS_ROOT to create it."
fi

# -----------------------------------------------------------------------------
# 3.5 ROS networking defaults (Robot Savo)
# -----------------------------------------------------------------------------
# Ensure ROS 2 discovery works across terminals/machines (Pi + PC)
unset ROS_LOCALHOST_ONLY

# Keep all Robot Savo terminals on the same ROS domain unless overridden
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 4. Ensure ~/.local/bin is on PATH
if [ -d "$HOME/.local/bin" ]; then
  case ":$PATH:" in
    *":$HOME/.local/bin:"*) ;;
    *)
      export PATH="$HOME/.local/bin:$PATH"
      ;;
  esac
fi

# 5. Optional debug/diagnostics env
export PYTHONWARNINGS="${PYTHONWARNINGS:-default}"

# 6. Load LLM server configuration (tools/scripts/env_llm.sh)
LLM_ENV="${WS_ROOT}/tools/scripts/env_llm.sh"
if [ -f "$LLM_ENV" ]; then
  # shellcheck source=/dev/null
  . "$LLM_ENV"
else
  if [ -z "${LLM_SERVER_URL:-}" ]; then
    export LLM_SERVER_URL="http://127.0.0.1:8000"
    echo "[env.sh] WARNING: env_llm.sh not found, using fallback LLM_SERVER_URL=${LLM_SERVER_URL}"
  else
    echo "[env.sh] INFO: LLM_SERVER_URL already set externally: ${LLM_SERVER_URL}"
  fi
fi

# 7. Final status message
echo "[env.sh] Loaded Robot Savo ROS environment"
echo "[env.sh]   WS_ROOT            = $WS_ROOT"
echo "[env.sh]   ROS_DISTRO         = ${ROS_DISTRO:-unknown}"
echo "[env.sh]   ROS_DOMAIN_ID      = ${ROS_DOMAIN_ID:-<not set>}"
echo "[env.sh]   ROS_LOCALHOST_ONLY = ${ROS_LOCALHOST_ONLY:-<unset>}"
echo "[env.sh]   LLM_SERVER_URL     = ${LLM_SERVER_URL:-<not set>}"
