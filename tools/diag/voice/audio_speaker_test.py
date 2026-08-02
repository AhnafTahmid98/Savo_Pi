#!/usr/bin/env python3
"""Observe one approved playback-completion event; this tool never requests speech."""
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

value=parser(__doc__); value.add_argument('--expect-playback',action='store_true'); args=value.parse_args()
started=time.monotonic(); started_utc=datetime.now(UTC).isoformat()
try:
    probe=collect('/savo_speech/playback/finished','std_msgs/msg/String',duration_s=args.timeout,
                  minimum_samples=1,reliable=True)
    details=compact(probe)
    if probe.publisher_count == 0: status,reason='BLOCKED','playback_event_publisher_missing'
    elif not probe.samples: status,reason=('FAIL','approved_playback_not_completed') if args.expect_playback else ('BLOCKED','no_playback_during_window')
    else: status,reason='PASS','playback_completion_observed'
except (ImportError,RuntimeError,ValueError) as exc:
    status,reason,details='BLOCKED','ros_probe_unavailable',{'error':str(exc)}
raise SystemExit(emit_result('audio_speaker',status,reason,details,output=args.output,started=started,started_utc=started_utc))
