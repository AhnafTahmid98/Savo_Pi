#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${REPO_ROOT}/savo_ws/install/setup.bash"
exec ros2 run savo_observer check_observer_connection
