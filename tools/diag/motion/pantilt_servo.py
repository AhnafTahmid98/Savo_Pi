#!/usr/bin/env python3
"""Disabled legacy direct-PCA9685 pan/tilt diagnostic.

Robot SAVO head movement must go through the running savo_head controller and
/savo_head/pan_tilt_cmd. Use tools/diag/ui/head_pan_tilt_test.py for read-only
state validation, or its explicit approved-control motion mode.
"""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import time
from datetime import UTC, datetime

from tools.diag.infra.diag_utils import emit_result, parser


def main() -> int:
    args = parser(__doc__, motion=True).parse_args()
    started = time.monotonic()
    return emit_result(
        "legacy_direct_pantilt",
        "BLOCKED",
        "legacy_direct_pca9685_bypass_disabled",
        {"approved_tool": "tools/diag/ui/head_pan_tilt_test.py"},
        output=args.output,
        started=started,
        started_utc=datetime.now(UTC).isoformat(),
    )


if __name__ == "__main__":
    raise SystemExit(main())
