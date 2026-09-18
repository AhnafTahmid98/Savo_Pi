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
while IFS= read -r -d '' path; do
  case "$(head -n 1 "${path}")" in
    '#!/usr/bin/env bash') bash -n "${path}" ;;
    '#!/usr/bin/env python3') python3 -m py_compile "${path}" ;;
    *) echo "Unsupported observer script shebang: ${path}" >&2; exit 1 ;;
  esac
done < <(
  find "${REPO_ROOT}/deploy/observer" "${PACKAGE}/scripts" \
    \( -type d \( -name __pycache__ -o -name .pytest_cache -o -name .mypy_cache -o -name .ruff_cache \) -prune \) -o \
    \( -type f ! -name '*.pyc' -print0 \)
)

observer_scan() {
  local pattern="$1"
  shift
  local rc

  if command -v rg >/dev/null 2>&1; then
    if rg -n "${pattern}" "$@"; then
      return 0
    else
      rc=$?
    fi
  else
    if grep -RInE -- "${pattern}" "$@"; then
      return 0
    else
      rc=$?
    fi
  fi

  if [[ ${rc} -eq 1 ]]; then
    return 1
  fi

  echo "Observer source scan failed with exit code ${rc}." >&2
  exit "${rc}"
}

if observer_scan 'SetGoal|SetInitialPose|PublishPoint|Teleop' "${PACKAGE}/rviz"; then
  echo 'Unsafe RViz tool found.' >&2
  exit 1
fi
if observer_scan 'create_client|create_service|rclcpp_action|/cmd_vel|/goal_pose|/initialpose' \
  "${PACKAGE}/src" "${PACKAGE}/launch"; then
  echo 'Mutation interface found in observer runtime.' >&2
  exit 1
fi

echo 'Robot SAVO observer source validation: PASS'
