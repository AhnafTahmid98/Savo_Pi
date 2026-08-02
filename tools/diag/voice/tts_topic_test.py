#!/usr/bin/env python3
"""Observe one response text from the approved SavoMind speech round trip."""
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
args=parser(__doc__).parse_args(); started=time.monotonic(); started_utc=datetime.now(UTC).isoformat()
try:
    probe=collect('/savo_speech/response','std_msgs/msg/String',duration_s=args.timeout,minimum_samples=1,reliable=True)
    details=compact(probe)
    if probe.publisher_count == 0: status,reason='BLOCKED','response_publisher_missing'
    elif not probe.samples: status,reason='BLOCKED','no_response_during_window'
    elif not str(probe.samples[-1].get('data','')).strip(): status,reason='FAIL','empty_response'
    else: status,reason='PASS','response_observed'
except (ImportError,RuntimeError,ValueError) as exc: status,reason,details='BLOCKED','ros_probe_unavailable',{'error':str(exc)}
raise SystemExit(emit_result('tts_topic',status,reason,details,output=args.output,started=started,started_utc=started_utc))
