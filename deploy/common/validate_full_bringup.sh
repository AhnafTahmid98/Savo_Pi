#!/usr/bin/env bash
# Source-level validation for the complete Robot Savo bringup closure.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BRINGUP="${ROOT}/savo_ws/src/shared/savo_bringup"

required_files=(
  "${BRINGUP}/CMakeLists.txt"
  "${BRINGUP}/include/savo_bringup/bringup_contract.hpp"
  "${BRINGUP}/src/bringup_contract.cpp"
  "${BRINGUP}/src/nodes/bringup_readiness_node.cpp"
  "${BRINGUP}/launch/core_bringup.launch.py"
  "${BRINGUP}/launch/edge_bringup.launch.py"
  "${BRINGUP}/launch/robot_bringup.launch.py"
  "${BRINGUP}/launch/manual_mapping.launch.py"
  "${BRINGUP}/launch/saved_map_navigation.launch.py"
  "${BRINGUP}/launch/autonomous_mapping.launch.py"
  "${ROOT}/deploy/core/run_core.sh"
  "${ROOT}/deploy/edge/run_edge.sh"
)

for file in "${required_files[@]}"; do
  [[ -s "${file}" ]] || {
    echo "Missing or empty required file: ${file}" >&2
    exit 1
  }
done

python3 -m compileall -q "${BRINGUP}"
python3 - "${BRINGUP}" <<'PY'
from pathlib import Path
import ast
import sys
import xml.etree.ElementTree as ET
import yaml

root = Path(sys.argv[1])
for path in root.rglob("*.py"):
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
for path in root.rglob("*.yaml"):
    yaml.safe_load(path.read_text(encoding="utf-8"))
ET.parse(root / "package.xml")
PY

bash -n \
  "${ROOT}/deploy/core/env_core.sh" \
  "${ROOT}/deploy/core/run_core.sh" \
  "${ROOT}/deploy/core/prepare_runtime_storage.sh" \
  "${ROOT}/deploy/edge/env_edge.sh" \
  "${ROOT}/deploy/edge/run_edge.sh" \
  "${ROOT}/deploy/edge/build_edge.sh"

if grep -RqE 'savo_dashboard|savo_intent' \
  "${ROOT}/deploy/core/env_core.sh" \
  "${ROOT}/deploy/edge/env_edge.sh"; then
  echo "Invalid retired package name remains in deployment arrays." >&2
  exit 1
fi

grep -q '"start_review_gateway": "true"' \
  "${BRINGUP}/launch/autonomous_mapping.launch.py"
grep -q 'contract_version: 2' "${BRINGUP}/README.md"
grep -q 'require_quality_approval: true' "${BRINGUP}/README.md"
grep -q 'production_navigation.launch.py' \
  "${BRINGUP}/launch/core_bringup.launch.py"
grep -q 'nav2_saved_map_voxel.yaml' \
  "${BRINGUP}/launch/core_bringup.launch.py"
grep -q 'default_value="lidar_only"' \
  "${BRINGUP}/launch/robot_bringup.launch.py"
grep -q '/var/lib/robot_savo/maps/production' \
  "${BRINGUP}/launch/robot_bringup.launch.py"
if grep -Rq '/tmp' "${BRINGUP}/launch" "${BRINGUP}/config"; then
  echo "Production bringup contains a /tmp runtime-state default." >&2
  exit 1
fi

echo "Robot Savo full bringup source validation: PASS"
