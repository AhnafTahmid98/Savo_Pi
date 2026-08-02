#!/usr/bin/env python3
"""Verify required Robot SAVO TF links without publishing transforms."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
import time
from datetime import UTC, datetime
from tools.diag.infra.diag_utils import emit_result, parser

args = parser(__doc__).parse_args()
started = time.monotonic(); started_utc = datetime.now(UTC).isoformat()
frames = [
    ('map', 'odom', False),
    ('odom', 'base_footprint', True),
    ('base_footprint', 'base_link', True),
    ('base_link', 'laser', True),
]
details: dict[str, object] = {'links': []}
try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init(args=None)
    node = Node('robot_savo_tf_tree_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    del listener
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    required_missing = []
    optional_missing = []
    for parent, child, required in frames:
        try:
            transform = buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
            details['links'].append({'parent': parent, 'child': child, 'required': required,
                                     'available': True, 'stamp': {'sec': transform.header.stamp.sec,
                                     'nanosec': transform.header.stamp.nanosec}})
        except Exception as exc:  # tf2 exception classes differ across releases
            details['links'].append({'parent': parent, 'child': child, 'required': required,
                                     'available': False, 'error': str(exc)})
            (required_missing if required else optional_missing).append(f'{parent}->{child}')
    node.destroy_node(); rclpy.shutdown()
    details['required_missing'] = required_missing
    details['optional_missing'] = optional_missing
    if required_missing:
        status, reason = 'FAIL', 'required_tf_chain_missing'
    else:
        status, reason = 'PASS', 'required_tf_chain_available'
except (ImportError, RuntimeError, ValueError) as exc:
    status, reason, details = 'BLOCKED', 'tf2_runtime_unavailable', {'error': str(exc)}
raise SystemExit(emit_result('tf_tree', status, reason, details, output=args.output,
                             started=started, started_utc=started_utc))
