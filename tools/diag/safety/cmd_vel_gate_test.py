#!/usr/bin/env python3
"""Verify that the safety-gated velocity remains zero while Robot SAVO is stopped."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import math
import time
from datetime import UTC, datetime
from tools.diag.infra.diag_utils import emit_result, parser
from tools.diag.infra.ros_probe import collect, compact, nested

args = parser(__doc__).parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
try:
    mode = collect('/savo_control/mode_state', 'std_msgs/msg/String', duration_s=min(args.timeout, 2.0),
                   minimum_samples=1, reliable=True)
    velocity = collect('/cmd_vel_safe', 'geometry_msgs/msg/Twist', duration_s=args.timeout,
                       minimum_samples=1, reliable=False)
    details = {'mode': compact(mode), 'safe_velocity': compact(velocity)}
    mode_text = str(mode.samples[-1].get('data', '')) if mode.samples else ''
    nonzero = []
    for sample in velocity.samples:
        values = [nested(sample, 'linear.x', 0.0), nested(sample, 'linear.y', 0.0),
                  nested(sample, 'linear.z', 0.0), nested(sample, 'angular.x', 0.0),
                  nested(sample, 'angular.y', 0.0), nested(sample, 'angular.z', 0.0)]
        if any(abs(float(value)) > 1.0e-4 for value in values):
            nonzero.append(values)
    details['mode_text'] = mode_text
    details['nonzero_sample_count'] = len(nonzero)
    if mode.publisher_count == 0 or velocity.publisher_count == 0:
        status, reason = 'BLOCKED', 'required_control_publisher_missing'
    elif 'STOP' not in mode_text.upper():
        status, reason = 'BLOCKED', 'control_mode_not_stop'
    elif not velocity.samples:
        status, reason = 'FAIL', 'safe_velocity_not_observed'
    elif nonzero:
        status, reason = 'FAIL', 'nonzero_velocity_leaked_while_stopped'
    else:
        status, reason = 'PASS', 'safe_velocity_zero_while_stopped'
except (ImportError, RuntimeError, ValueError) as exc:
    status, reason, details = 'BLOCKED', 'ros_probe_unavailable', {'error': str(exc)}
raise SystemExit(emit_result('cmd_vel_gate', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
