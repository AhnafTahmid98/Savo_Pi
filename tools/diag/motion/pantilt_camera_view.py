#!/usr/bin/env python3
"""Disabled legacy direct-hardware pan/tilt and UDP camera diagnostic.

Use savo_head through ROS for pan/tilt and savo_observer for camera viewing.
This wrapper intentionally performs no I2C, servo, camera, or network action.
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
        "legacy_pantilt_camera_view",
        "BLOCKED",
        "legacy_direct_hardware_bypass_disabled",
        {
            "head_tool": "tools/diag/ui/head_pan_tilt_test.py",
            "camera_tool": "savo_observer sensors view",
        },
        output=args.output,
        started=started,
        started_utc=datetime.now(UTC).isoformat(),
    )


if __name__ == "__main__":
    raise SystemExit(main())
