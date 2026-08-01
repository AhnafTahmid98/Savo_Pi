#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Create or verify a persistent Phase 3 supervisor fault latch."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.srv import ManageSystemState
from std_msgs.msg import String


_STATE_TOPIC = '/savo_supervisor/state_summary'
_CONTROL_TOPIC = '/savo_supervisor/test/control'


class PersistenceProbe(Node):
    """Drive and inspect persistent supervisor state."""

    def __init__(self) -> None:
        super().__init__('savo_supervisor_phase3_persistence_probe')
        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        normal_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state: dict[str, object] | None = None
        self._state_subscription = self.create_subscription(
            String, _STATE_TOPIC, self._on_state, retained_qos)
        self._control = self.create_publisher(
            String, _CONTROL_TOPIC, normal_qos)
        self._system = self.create_client(
            ManageSystemState, '/savo_supervisor/manage_system_state')

    def _on_state(self, message: String) -> None:
        try:
            self.state = json.loads(message.data)
        except json.JSONDecodeError:
            return

    def wait_state(
        self,
        predicate: Callable[[dict[str, object]], bool],
        label: str,
        timeout: float = 25.0,
    ) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state is not None and predicate(self.state):
                print(f'PASS: {label}')
                print(json.dumps(self.state, indent=2, sort_keys=True))
                return self.state
        raise RuntimeError(f'timed out waiting for {label}; last={self.state!r}')

    def publish_control(self, command: str) -> None:
        message = String()
        message.data = command
        for _ in range(3):
            self._control.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)

    def call_system(
        self,
        command: int,
        request_id: str,
        reason: str,
        expected_generation: int = 0,
    ) -> ManageSystemState.Response:
        if not self._system.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('manage_system_state service unavailable')
        request = ManageSystemState.Request()
        request.command = command
        request.request_id = request_id
        request.actor_id = 'system_operator_phase3_persistence_probe'
        request.reason = reason
        request.expected_generation = expected_generation
        future = self._system.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('manage_system_state response unavailable')
        return future.result()


def _system_state(expected: str):
    def predicate(state: dict[str, object]) -> bool:
        system = state.get('system_authority')
        return isinstance(system, dict) and system.get('state') == expected
    return predicate


def _system_armed(state: dict[str, object]) -> bool:
    system = state.get('system_authority')
    return (
        isinstance(system, dict)
        and system.get('system_armed') is True
        and system.get('state') in {'ARMED', 'ARMED_DEGRADED'}
    )


def create_latch(output: Path) -> int:
    rclpy.init()
    node = PersistenceProbe()
    try:
        snapshots: dict[str, object] = {}
        snapshots['ready'] = node.wait_state(
            _system_state('READY_TO_ARM'),
            'system reaches READY_TO_ARM before persistence test',
        )
        arm = node.call_system(
            ManageSystemState.Request.COMMAND_ARM,
            'persist-arm-1',
            'arm_before_fault_persistence_test',
        )
        if not arm.accepted:
            raise RuntimeError(f'arm failed: {arm.reason}')
        node.wait_state(_system_armed, 'system arms before injected fault')
        node.publish_control('drop_core:lidar')
        snapshots['latched'] = node.wait_state(
            _system_state('FAULT_LATCHED'),
            'required core fault creates persistent latch',
        )
        output.write_text(json.dumps(snapshots, indent=2, sort_keys=True))
        print('PHASE_3_FAULT_LATCH_CREATED')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def verify_latch(output: Path) -> int:
    rclpy.init()
    node = PersistenceProbe()
    try:
        snapshots: dict[str, object] = {}
        snapshots['restored'] = node.wait_state(
            lambda state: (
                _system_state('FAULT_LATCHED')(state)
                and state.get('system_authority', {}).get('startup_ready') is True
                and state.get('system_authority', {}).get('system_armed') is False
            ),
            'fault latch survives supervisor process restart',
        )
        generation = int(
            snapshots['restored']['system_authority']['system_generation'])
        clear = node.call_system(
            ManageSystemState.Request.COMMAND_CLEAR_FAULT_LATCH,
            'persist-clear-1',
            'operator_verified_recovery_after_restart',
            generation,
        )
        if not clear.accepted:
            raise RuntimeError(f'fault clear failed: {clear.reason}')
        snapshots['cleared'] = node.wait_state(
            _system_state('READY_TO_ARM'),
            'explicit clear returns restarted supervisor to READY_TO_ARM',
        )
        output.write_text(json.dumps(snapshots, indent=2, sort_keys=True))
        print('PHASE_3_FAULT_PERSISTENCE_RUNTIME_COMPLETE')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if len(sys.argv) != 3 or sys.argv[1] not in {'create', 'verify'}:
        print(
            'usage: phase3_persistence_probe.py {create|verify} OUTPUT_JSON',
            file=sys.stderr,
        )
        return 2
    output = Path(sys.argv[2])
    return create_latch(output) if sys.argv[1] == 'create' else verify_latch(output)


if __name__ == '__main__':
    raise SystemExit(main())
