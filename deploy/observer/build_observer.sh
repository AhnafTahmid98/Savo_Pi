#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORKSPACE="${REPO_ROOT}/savo_ws"
source /opt/ros/jazzy/setup.bash
cd "${WORKSPACE}"
source install/setup.bash 2>/dev/null || true
colcon build --packages-up-to savo_observer --symlink-install --event-handlers console_direct+
