#!/usr/bin/env python3
"""Record odometry during an operator-controlled measured motion; never commands motion."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import argparse
import math
import time
from datetime import UTC, datetime
from tools.diag.infra.diag_utils import emit_result, parser
from tools.diag.infra.ros_probe import collect, compact, nested, quaternion_yaw

value = parser(__doc__, motion=True)
value.add_argument('--expected-distance-m', type=float, default=0.0)
value.add_argument('--expected-yaw-rad', type=float, default=0.0)
args = value.parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
if not args.allow_motion:
    raise SystemExit(emit_result('odom_calibration', 'BLOCKED', 'motion_not_authorized_use_allow_motion', {},
        output=args.output, started=started, started_utc=started_utc))
if args.expected_distance_m <= 0.0 and abs(args.expected_yaw_rad) <= 0.0:
    raise SystemExit(emit_result('odom_calibration', 'BLOCKED', 'expected_motion_not_specified', {},
        output=args.output, started=started, started_utc=started_utc))
try:
    probe = collect('/wheel/odom', 'nav_msgs/msg/Odometry', duration_s=args.timeout,
                    minimum_samples=2, reliable=False, maximum_samples=500)
    details = compact(probe)
    if len(probe.samples) < 2:
        status, reason = ('BLOCKED', 'wheel_odometry_publisher_missing') if probe.publisher_count == 0 else ('FAIL', 'insufficient_samples')
    else:
        first, last = probe.samples[0], probe.samples[-1]
        dx = float(nested(last, 'pose.pose.position.x', 0.0)) - float(nested(first, 'pose.pose.position.x', 0.0))
        dy = float(nested(last, 'pose.pose.position.y', 0.0)) - float(nested(first, 'pose.pose.position.y', 0.0))
        measured_distance = math.hypot(dx, dy)
        measured_yaw = quaternion_yaw(nested(last, 'pose.pose.orientation', {})) - quaternion_yaw(nested(first, 'pose.pose.orientation', {}))
        details.update({'expected_distance_m': args.expected_distance_m, 'measured_distance_m': measured_distance,
                        'expected_yaw_rad': args.expected_yaw_rad, 'measured_yaw_rad': measured_yaw})
        errors = []
        if args.expected_distance_m > 0.0:
            errors.append(abs(measured_distance - args.expected_distance_m) / args.expected_distance_m)
        if abs(args.expected_yaw_rad) > 0.0:
            errors.append(abs(measured_yaw - args.expected_yaw_rad) / abs(args.expected_yaw_rad))
        details['relative_error'] = max(errors) if errors else None
        status, reason = ('PASS', 'operator_motion_recorded') if max(errors) <= 0.25 else ('FAIL', 'odometry_calibration_error_exceeds_25_percent')
except (ImportError, RuntimeError, ValueError) as exc:
    status, reason, details = 'BLOCKED', 'ros_probe_unavailable', {'error': str(exc)}
raise SystemExit(emit_result('odom_calibration', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
