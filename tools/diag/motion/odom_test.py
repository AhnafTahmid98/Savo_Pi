#!/usr/bin/env python3
"""Measure wheel-odometry availability, rate, and stationary drift."""
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
from tools.diag.infra.ros_probe import collect, compact, nested, quaternion_yaw

args = parser(__doc__).parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
try:
    probe = collect('/wheel/odom', 'nav_msgs/msg/Odometry', duration_s=args.timeout,
                    minimum_samples=2, reliable=False, maximum_samples=200)
    details = compact(probe)
    drift_m = yaw_drift = 0.0
    if len(probe.samples) >= 2:
        first, last = probe.samples[0], probe.samples[-1]
        dx = float(nested(last, 'pose.pose.position.x', 0.0)) - float(nested(first, 'pose.pose.position.x', 0.0))
        dy = float(nested(last, 'pose.pose.position.y', 0.0)) - float(nested(first, 'pose.pose.position.y', 0.0))
        drift_m = math.hypot(dx, dy)
        yaw_drift = abs(quaternion_yaw(nested(last, 'pose.pose.orientation', {})) -
                        quaternion_yaw(nested(first, 'pose.pose.orientation', {})))
    details.update({'stationary_translation_drift_m': round(drift_m, 6),
                    'stationary_yaw_drift_rad': round(yaw_drift, 6)})
    if probe.publisher_count == 0:
        status, reason = 'BLOCKED', 'wheel_odometry_publisher_missing'
    elif probe.sample_count < 2:
        status, reason = 'FAIL', 'insufficient_wheel_odometry_samples'
    elif probe.rate_hz < 5.0:
        status, reason = 'FAIL', 'wheel_odometry_rate_too_low'
    elif drift_m > 0.03 or yaw_drift > 0.08:
        status, reason = 'FAIL', 'stationary_odometry_drift_excessive'
    else:
        status, reason = 'PASS', 'wheel_odometry_stream_healthy'
except (ImportError, RuntimeError, ValueError) as exc:
    status, reason, details = 'BLOCKED', 'ros_probe_unavailable', {'error': str(exc)}
raise SystemExit(emit_result('odometry', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
