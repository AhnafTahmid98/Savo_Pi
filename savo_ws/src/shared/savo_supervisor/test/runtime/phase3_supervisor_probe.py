#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise Phase 3 edge, startup, latch and shutdown authority."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.srv import AuthorizeOperation, ManageSystemState
from std_msgs.msg import String


_STATE_TOPIC = '/savo_supervisor/state_summary'
_CONTROL_TOPIC = '/savo_supervisor/test/control'


class Phase3Probe(Node):
    """Drive system and mission authority through production-style faults."""

    def __init__(self) -> None:
        super().__init__('savo_supervisor_phase3_probe')
        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state: dict[str, object] | None = None
        self.history: list[dict[str, object]] = []
        self._state_subscription = self.create_subscription(
            String, _STATE_TOPIC, self._on_state, retained_qos)
        self._control = self.create_publisher(
            String, _CONTROL_TOPIC, status_qos)
        self._system = self.create_client(
            ManageSystemState, '/savo_supervisor/manage_system_state')
        self._authority = self.create_client(
            AuthorizeOperation, '/savo_supervisor/authorize_operation')

    def _on_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.state = state
        self.history.append(state)

    def publish_control(self, command: str) -> None:
        message = String()
        message.data = command
        for _ in range(3):
            self._control.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_state(
        self,
        predicate: Callable[[dict[str, object]], bool],
        label: str,
        timeout: float = 20.0,
    ) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state is not None and predicate(self.state):
                print(f'PASS: {label}')
                print(json.dumps(self.state, indent=2, sort_keys=True))
                return self.state
        raise RuntimeError(f'timed out waiting for {label}; last={self.state!r}')

    def call_system(
        self,
        command: int,
        *,
        request_id: str,
        reason: str,
        expected_generation: int = 0,
        actor_id: str = 'system_operator_phase3_probe',
    ) -> ManageSystemState.Response:
        if not self._system.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('manage_system_state service unavailable')
        request = ManageSystemState.Request()
        request.command = command
        request.request_id = request_id
        request.actor_id = actor_id
        request.reason = reason
        request.expected_generation = expected_generation
        future = self._system.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('manage_system_state response unavailable')
        response = future.result()
        print(
            'SYSTEM:', command, response.accepted, response.reason,
            response.system_state, response.system_generation,
        )
        return response

    def call_manual_authority(
        self,
        command: int,
        *,
        request_id: str = 'remote-manual-1',
        expected_generation: int = 0,
    ) -> AuthorizeOperation.Response:
        if not self._authority.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('authorize_operation service unavailable')
        request = AuthorizeOperation.Request()
        request.command = command
        request.operation = AuthorizeOperation.Request.OP_ENTER_MANUAL_CONTROL
        request.request_id = request_id
        request.actor_id = 'savo_bridge_runtime'
        request.motion_required = True
        request.expected_generation = expected_generation
        future = self._authority.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('authorize_operation response unavailable')
        response = future.result()
        print(
            'AUTH:', command, response.authorized, response.reason,
            response.operation_state, response.authority_generation,
        )
        return response


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


def _mission_state(expected: str):
    def predicate(state: dict[str, object]) -> bool:
        mission = state.get('mission')
        return isinstance(mission, dict) and mission.get('operation_state') == expected
    return predicate


def run(output: Path | None) -> int:
    rclpy.init()
    node = Phase3Probe()
    snapshots: dict[str, object] = {}
    try:
        snapshots['ready_to_arm'] = node.wait_state(
            lambda state: (
                _system_state('READY_TO_ARM')(state)
                and state.get('system_authority', {}).get('startup_ready') is True
                and state.get('edge_capabilities', {}).get('bridge_ready') is True
                and state.get('operating_mode') == 'STOP'
            ),
            'healthy core and required edge path reach READY_TO_ARM',
        )

        denied = node.call_manual_authority(
            AuthorizeOperation.Request.COMMAND_ACQUIRE)
        if denied.authorized or denied.reason != 'system_not_armed':
            raise RuntimeError('remote operation was not blocked before system arm')

        unauthorized_arm = node.call_system(
            ManageSystemState.Request.COMMAND_ARM,
            request_id='unauthorized-arm-1',
            reason='untrusted_arm_attempt',
            actor_id='untrusted_component',
        )
        if unauthorized_arm.accepted or (
            unauthorized_arm.reason != 'system_actor_not_authorized'
        ):
            raise RuntimeError('untrusted system actor was not rejected')

        armed = node.call_system(
            ManageSystemState.Request.COMMAND_ARM,
            request_id='arm-1',
            reason='operator_arm_for_runtime_test',
        )
        if not armed.accepted:
            raise RuntimeError(f'arm denied: {armed.reason}')
        snapshots['armed'] = node.wait_state(
            lambda state: (
                _system_armed(state)
                and state.get('system_authority', {}).get(
                    'remote_commands_ready') is True
                and state.get('operating_mode') == 'IDLE'
            ),
            'explicit arm enables remote command authority',
        )

        node.publish_control('drop_speech')
        snapshots['optional_edge_degraded'] = node.wait_state(
            lambda state: (
                _system_state('ARMED_DEGRADED')(state)
                and state.get('edge_capabilities', {}).get(
                    'speech_ready') is False
                and state.get('system_authority', {}).get(
                    'remote_commands_ready') is True
            ),
            'optional speech loss degrades without disabling bridge commands',
        )
        node.publish_control('restore_speech')
        node.wait_state(
            _system_armed,
            'optional speech recovery restores nominal armed state',
        )

        acquired = node.call_manual_authority(
            AuthorizeOperation.Request.COMMAND_ACQUIRE)
        if not acquired.authorized:
            raise RuntimeError(f'remote manual authority denied: {acquired.reason}')
        snapshots['remote_active'] = node.wait_state(
            lambda state: (
                _mission_state('ACTIVE')(state)
                and state.get('operating_mode') == 'MANUAL'
            ),
            'remote-origin manual lease becomes active',
        )

        node.publish_control('drop_bridge')
        snapshots['bridge_revoked'] = node.wait_state(
            lambda state: (
                _system_state('ARMED_DEGRADED')(state)
                and _mission_state('REVOKED')(state)
                and state.get('system_authority', {}).get(
                    'remote_commands_ready') is False
            ),
            'bridge loss revokes remote mission without faulting local core',
        )
        revoked_generation = int(
            snapshots['bridge_revoked']['mission']['authority_generation'])

        node.publish_control('restore_bridge')
        snapshots['bridge_recovered'] = node.wait_state(
            lambda state: (
                _system_armed(state)
                and _mission_state('REVOKED')(state)
                and state.get('system_authority', {}).get(
                    'remote_commands_ready') is True
            ),
            'bridge recovery does not automatically resume remote motion',
        )

        resumed = node.call_manual_authority(
            AuthorizeOperation.Request.COMMAND_RESUME,
            expected_generation=revoked_generation,
        )
        if not resumed.authorized:
            raise RuntimeError(f'remote resume denied: {resumed.reason}')
        node.wait_state(
            _mission_state('ACTIVE'),
            'explicit remote resume restores authority',
        )

        released = node.call_manual_authority(
            AuthorizeOperation.Request.COMMAND_RELEASE,
            expected_generation=resumed.authority_generation,
        )
        if not released.authorized:
            raise RuntimeError(f'remote release denied: {released.reason}')
        node.wait_state(_mission_state('IDLE'), 'remote authority released')

        node.publish_control('drop_core:lidar')
        snapshots['fault_latched'] = node.wait_state(
            lambda state: (
                _system_state('FAULT_LATCHED')(state)
                and state.get('operating_mode') == 'ERROR'
                and state.get('system_authority', {}).get('system_armed') is False
            ),
            'critical core fault latches and disarms the system',
            timeout=25.0,
        )

        node.publish_control('restore_core')
        snapshots['fault_persists'] = node.wait_state(
            lambda state: (
                _system_state('FAULT_LATCHED')(state)
                and state.get('system_authority', {}).get('startup_ready') is True
            ),
            'core recovery leaves persistent fault latch for operator review',
            timeout=25.0,
        )
        latch_generation = int(
            snapshots['fault_persists']['system_authority']['system_generation'])

        cleared = node.call_system(
            ManageSystemState.Request.COMMAND_CLEAR_FAULT_LATCH,
            request_id='clear-fault-1',
            reason='operator_verified_core_recovery',
            expected_generation=latch_generation,
        )
        if not cleared.accepted:
            raise RuntimeError(f'fault clear denied: {cleared.reason}')
        node.wait_state(
            _system_state('READY_TO_ARM'),
            'operator fault clear returns system to READY_TO_ARM',
        )

        rearmed = node.call_system(
            ManageSystemState.Request.COMMAND_ARM,
            request_id='arm-2',
            reason='operator_rearm_after_recovery',
            expected_generation=cleared.system_generation,
        )
        if not rearmed.accepted:
            raise RuntimeError(f'rearm denied: {rearmed.reason}')
        node.wait_state(_system_armed, 'system rearmed after fault review')

        shutdown = node.call_system(
            ManageSystemState.Request.COMMAND_BEGIN_SHUTDOWN,
            request_id='shutdown-1',
            reason='controlled_test_shutdown',
            expected_generation=rearmed.system_generation,
        )
        if not shutdown.accepted:
            raise RuntimeError(f'shutdown request denied: {shutdown.reason}')
        snapshots['shutdown'] = node.wait_state(
            lambda state: (
                _system_state('SHUTDOWN_REQUESTED')(state)
                and state.get('operating_mode') == 'SHUTTING_DOWN'
                and state.get('ready') is False
            ),
            'controlled shutdown revokes readiness and publishes shutdown intent',
        )

        if output is not None:
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(snapshots, indent=2, sort_keys=True))
        print('PHASE_3_EDGE_STARTUP_RUNTIME_COMPLETE')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    destination = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    raise SystemExit(run(destination))
