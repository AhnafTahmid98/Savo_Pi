#!/usr/bin/env bash
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

# Backward-compatible entry point. Phase 1 now validates the complete six-package core.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/run_core_supervisor_recovery_test.sh" "$@"
