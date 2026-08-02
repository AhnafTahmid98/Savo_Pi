#!/usr/bin/env python3
"""Verify that the edge UI node is present and reporting diagnostics."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import subprocess
import time
from datetime import UTC, datetime
from tools.diag.infra.diag_utils import command_available, emit_result, parser

args = parser(__doc__).parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
details: dict[str, object] = {}
if not command_available('ros2'):
    status, reason = 'BLOCKED', 'ros2_cli_missing'
else:
    nodes = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True,
                           timeout=args.timeout, check=False)
    details['nodes'] = nodes.stdout.splitlines()
    ui_nodes = [name for name in details['nodes'] if 'savo_ui' in name]
    details['ui_nodes'] = ui_nodes
    if nodes.returncode != 0:
        status, reason = 'FAIL', 'ros_node_list_failed'
    elif not ui_nodes:
        status, reason = 'BLOCKED', 'savo_ui_node_not_running'
    else:
        status, reason = 'PASS', 'savo_ui_node_running'
raise SystemExit(emit_result('screen_ui', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
