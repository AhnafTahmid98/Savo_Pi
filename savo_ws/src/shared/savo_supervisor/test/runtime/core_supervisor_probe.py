#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Wait for and assert a Robot Savo supervisor state-summary condition."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _bool_text(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {'true', '1', 'yes'}:
        return True
    if normalized in {'false', '0', 'no'}:
        return False
    raise argparse.ArgumentTypeError('expected true or false')


def _arguments(argv: Sequence[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--lifecycle')
    parser.add_argument('--health')
    parser.add_argument('--reason')
    parser.add_argument('--component')
    parser.add_argument('--component-state')
    parser.add_argument('--capability')
    parser.add_argument('--capability-value', type=_bool_text)
    parser.add_argument('--output')
    return parser.parse_known_args(argv)


class SupervisorProbe(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__('savo_supervisor_core_probe')
        self._options = options
        self.matched: dict[str, object] | None = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = self.create_subscription(
            String,
            '/savo_supervisor/state_summary',
            self._on_state,
            qos,
        )

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not self._matches(state):
            return
        self.matched = state

    def _matches(self, state: dict[str, object]) -> bool:
        options = self._options
        if options.lifecycle and state.get('lifecycle') != options.lifecycle:
            return False
        if options.health and state.get('health') != options.health:
            return False
        if options.reason and state.get('reason_code') != options.reason:
            return False
        if options.component:
            components = state.get('components')
            if not isinstance(components, dict):
                return False
            component = components.get(options.component)
            if not isinstance(component, dict):
                return False
            if options.component_state and component.get('state') != options.component_state:
                return False
        if options.capability:
            capabilities = state.get('capabilities')
            if not isinstance(capabilities, dict):
                return False
            if options.capability_value is None:
                raise ValueError('--capability-value is required with --capability')
            if capabilities.get(options.capability) is not options.capability_value:
                return False
        return True


def main(argv: Sequence[str] | None = None) -> int:
    options, ros_args = _arguments(argv)
    if options.timeout <= 0.0:
        raise ValueError('timeout must be positive')
    if options.component_state and not options.component:
        raise ValueError('--component-state requires --component')
    if options.capability_value is not None and not options.capability:
        raise ValueError('--capability-value requires --capability')

    rclpy.init(args=ros_args)
    node = SupervisorProbe(options)
    deadline = time.monotonic() + options.timeout
    try:
        while node.matched is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.matched is None:
            node.get_logger().error('Timed out waiting for expected supervisor state')
            return 1
        serialized = json.dumps(node.matched, indent=2, sort_keys=True)
        print(serialized)
        if options.output:
            Path(options.output).write_text(serialized + '\n', encoding='utf-8')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
