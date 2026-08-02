#!/usr/bin/env python3
"""Observe bounded typed AprilTag detections from the head camera."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import time
from datetime import UTC, datetime
from tools.diag.infra.diag_utils import emit_result, parser
from tools.diag.infra.ros_probe import collect, compact

value = parser(__doc__)
value.add_argument('--minimum-samples', type=int, default=1)
args = value.parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
try:
    probe = collect('/savo_head/apriltag_detections', 'savo_msgs/msg/AprilTagObservation',
                    duration_s=args.timeout, minimum_samples=max(1, args.minimum_samples),
                    reliable=False)
    details = compact(probe)
    if probe.publisher_count == 0:
        status, reason = 'BLOCKED', 'apriltag_detector_not_running'
    elif probe.sample_count < args.minimum_samples:
        status, reason = 'BLOCKED', 'no_apriltag_visible_during_window'
    else:
        status, reason = 'PASS', 'apriltag_observation_received'
except (ImportError, RuntimeError, ValueError) as exc:
    status, reason, details = 'BLOCKED', 'ros_probe_unavailable', {'error': str(exc)}
raise SystemExit(emit_result('apriltag', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
