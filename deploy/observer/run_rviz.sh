#!/usr/bin/env bash
set -Eeuo pipefail

export SAVO_OBSERVER_MODE=rviz
exec "$(dirname "${BASH_SOURCE[0]}")/run_observer.sh" "$@"
