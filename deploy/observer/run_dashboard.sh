#!/usr/bin/env bash
set -Eeuo pipefail

export SAVO_OBSERVER_MODE=dashboard
export SAVO_OBSERVER_PROFILE="${SAVO_OBSERVER_PROFILE:-mobile}"
exec "$(dirname "${BASH_SOURCE[0]}")/run_observer.sh" "$@"
