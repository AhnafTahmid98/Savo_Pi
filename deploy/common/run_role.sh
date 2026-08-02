#!/usr/bin/env bash
set -Eeuo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_common.sh
source "$script_dir/env_common.sh"
case "${SAVO_ROLE:-}" in
  core) exec "$SAVO_ROOT/deploy/core/run_core.sh" ;;
  edge) exec "$SAVO_ROOT/deploy/edge/run_edge.sh" ;;
  *) savo_die "SAVO_ROLE must be exactly core or edge" ;;
esac
