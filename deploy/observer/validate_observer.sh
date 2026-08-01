#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PACKAGE="${REPO_ROOT}/savo_ws/src/shared/savo_observer"

required=(
  CMakeLists.txt package.xml README.md LICENSE
  include/savo_observer/observer_contract.hpp
  src/nodes/observer_telemetry_node.cpp
  src/nodes/observer_dashboard_node.cpp
  launch/observer.launch.py
  config/migration_manifest.yaml
  rviz/overview.rviz
  dashboard/web/index.html
)
for path in "${required[@]}"; do
  [[ -s "${PACKAGE}/${path}" ]] || { echo "Missing observer asset: ${path}" >&2; exit 1; }
done

find "${PACKAGE}/launch" "${PACKAGE}/test" -type f -name '*.py' -print0 2>/dev/null | \
  xargs -0 -r python3 -m py_compile
find "${PACKAGE}/config" "${PACKAGE}/dashboard/layouts" "${PACKAGE}/rviz" \
  -type f \( -name '*.yaml' -o -name '*.rviz' \) -print0 | \
  xargs -0 python3 -c 'import sys,yaml; [yaml.safe_load(open(p, encoding="utf-8")) for p in sys.argv[1:]]'
find "${REPO_ROOT}/deploy/observer" "${PACKAGE}/scripts" -type f -print0 | \
  xargs -0 -n1 bash -n

if rg -n 'SetGoal|SetInitialPose|PublishPoint|Teleop' "${PACKAGE}/rviz"; then
  echo 'Unsafe RViz tool found.' >&2
  exit 1
fi
if rg -n 'create_client|create_service|rclcpp_action|/cmd_vel|/goal_pose|/initialpose' \
  "${PACKAGE}/src" "${PACKAGE}/launch"; then
  echo 'Mutation interface found in observer runtime.' >&2
  exit 1
fi

echo 'Robot SAVO observer source validation: PASS'
