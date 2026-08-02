#!/usr/bin/env python3
"""Disabled legacy direct-hardware drivetrain diagnostic.

Direct PCA9685/GPIO motor control bypasses Robot SAVO control, perception,
safety gating, watchdogs, and supervisor ownership. Use the approved
`motor_direction_test.py` with wheels raised, or the documented manual-control
hardware test plan.
"""
from __future__ import annotations
import json
import sys

print(json.dumps({
    "status": "BLOCKED",
    "reason": "legacy_direct_motor_bypass_disabled",
    "replacement": "tools/diag/motion/motor_direction_test.py --wheels-raised --allow-motion",
}, indent=2))
raise SystemExit(2)
