#!/usr/bin/env python3
# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise Phase 2 stateful mission authority against deterministic fixtures."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.srv import AuthorizeOperation, UpdateMapContext
from std_msgs.msg import String


_STATE_TOPIC = '/savo_supervisor/state_summary'
_CONTROL_TOPIC = '/savo_supervisor/test/control'


class Phase2Probe(Node):
    """Drive service requests and assert state transitions."""

    def __init__(self) -> None:
        super().__init__('savo_supervisor_phase2_probe')
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
        self._control = self.create_publisher(String, _CONTROL_TOPIC, status_qos)
        self._authority = self.create_client(
            AuthorizeOperation, '/savo_supervisor/authorize_operation')
        self._map_context = self.create_client(
            UpdateMapContext, '/savo_supervisor/update_map_context')

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
        timeout: float = 15.0,
    ) -> dict[str, object]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.state is not None and predicate(self.state):
                print(f'PASS: {label}')
                print(json.dumps(self.state, indent=2, sort_keys=True))
                return self.state
        raise RuntimeError(f'timed out waiting for {label}; last={self.state!r}')

    def call_authority(
        self,
        *,
        command: int,
        operation: int,
        request_id: str,
        actor_id: str = 'phase2_probe',
        map_id: str = '',
        map_revision: int = 0,
        map_release_id: str = '',
        require_semantic: bool = False,
        motion_required: bool = False,
        expected_generation: int = 0,
    ) -> AuthorizeOperation.Response:
        if not self._authority.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('authorize_operation service unavailable')
        request = AuthorizeOperation.Request()
        request.command = command
        request.operation = operation
        request.request_id = request_id
        request.actor_id = actor_id
        request.map_id = map_id
        request.map_revision = map_revision
        request.map_release_id = map_release_id
        request.require_semantic = require_semantic
        request.motion_required = motion_required
        request.expected_generation = expected_generation
        future = self._authority.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('authorize_operation response unavailable')
        response = future.result()
        print(
            'AUTH:', command, operation, response.authorized,
            response.reason, response.operation_state,
            response.authority_generation,
        )
        return response

    def set_saved_map(self) -> UpdateMapContext.Response:
        if not self._map_context.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('update_map_context service unavailable')
        request = UpdateMapContext.Request()
        request.command = UpdateMapContext.Request.COMMAND_SET_SAVED_RELEASE
        request.request_id = 'set-release-1'
        request.actor_id = 'phase2_probe'
        request.map_id = 'floor_2'
        request.map_revision = 1
        request.map_release_id = 'release-1'
        request.mapping_session_id = 'mission-1'
        request.approved = True
        future = self._map_context.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('update_map_context response unavailable')
        response = future.result()
        print('MAP:', response.updated, response.reason, response.context_generation)
        return response


def _mission_capability(name: str, value: bool):
    def predicate(state: dict[str, object]) -> bool:
        capabilities = state.get('mission_capabilities')
        return isinstance(capabilities, dict) and capabilities.get(name) is value
    return predicate


def _authority_state(operation: str, operation_state: str, mode: str | None = None):
    def predicate(state: dict[str, object]) -> bool:
        mission = state.get('mission')
        if not isinstance(mission, dict):
            return False
        if mission.get('active_operation') != operation:
            return False
        if mission.get('operation_state') != operation_state:
            return False
        return mode is None or state.get('operating_mode') == mode
    return predicate


def run(output: Path | None) -> int:
    rclpy.init()
    node = Phase2Probe()
    snapshots: dict[str, object] = {}
    try:
        snapshots['ready'] = node.wait_state(
            _mission_capability('can_start_autonomous_mapping', True),
            'full semantic autonomous mapping readiness',
            timeout=20.0,
        )

        acquired = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_ACQUIRE,
            operation=AuthorizeOperation.Request.OP_START_AUTONOMOUS_MAPPING,
            request_id='mission-1',
            map_id='floor_2',
            map_revision=1,
            require_semantic=True,
            motion_required=True,
        )
        if not acquired.authorized:
            raise RuntimeError(f'autonomous mapping acquisition denied: {acquired.reason}')
        snapshots['mapping_active'] = node.wait_state(
            _authority_state('AUTONOMOUS_MAPPING', 'ACTIVE', 'MAPPING'),
            'autonomous mapping ownership active',
        )

        conflict = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_ACQUIRE,
            operation=AuthorizeOperation.Request.OP_ENTER_MANUAL_CONTROL,
            request_id='manual-conflict',
            motion_required=True,
        )
        if conflict.authorized or conflict.result_code != (
            AuthorizeOperation.Response.RESULT_OPERATION_CONFLICT
        ):
            raise RuntimeError('conflicting major operation was not rejected')

        node.publish_control('drop_navigation')
        snapshots['mapping_revoked'] = node.wait_state(
            _authority_state('AUTONOMOUS_MAPPING', 'REVOKED', 'RECOVERY'),
            'navigation loss revokes mapping authority',
        )
        revoked_generation = int(
            snapshots['mapping_revoked']['mission']['authority_generation'])

        node.publish_control('restore_navigation')
        snapshots['still_revoked'] = node.wait_state(
            lambda state: (
                _authority_state('AUTONOMOUS_MAPPING', 'REVOKED', 'RECOVERY')(state)
                and _mission_capability('can_start_autonomous_mapping', True)(state)
            ),
            'recovery does not automatically resume authority',
        )

        resumed = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_RESUME,
            operation=AuthorizeOperation.Request.OP_START_AUTONOMOUS_MAPPING,
            request_id='mission-1',
            map_id='floor_2',
            map_revision=1,
            require_semantic=True,
            motion_required=True,
            expected_generation=revoked_generation,
        )
        if not resumed.authorized:
            raise RuntimeError(f'explicit mapping resume denied: {resumed.reason}')
        snapshots['mapping_resumed'] = node.wait_state(
            _authority_state('AUTONOMOUS_MAPPING', 'ACTIVE', 'MAPPING'),
            'explicit resume restores mapping authority',
        )

        released = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_RELEASE,
            operation=AuthorizeOperation.Request.OP_START_AUTONOMOUS_MAPPING,
            request_id='mission-1',
            map_id='floor_2',
            map_revision=1,
            expected_generation=resumed.authority_generation,
        )
        if not released.authorized:
            raise RuntimeError(f'mapping release denied: {released.reason}')
        node.wait_state(
            _authority_state('NONE', 'IDLE', 'IDLE'),
            'mapping ownership released',
        )

        map_response = node.set_saved_map()
        if not map_response.updated:
            raise RuntimeError(f'saved map context denied: {map_response.reason}')
        snapshots['saved_map_ready'] = node.wait_state(
            _mission_capability('can_navigate', True),
            'approved saved release enables navigation',
        )

        nav_acquired = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_ACQUIRE,
            operation=AuthorizeOperation.Request.OP_NAVIGATE_TO_LOCATION,
            request_id='nav-1',
            map_id='floor_2',
            map_revision=1,
            map_release_id='release-1',
            motion_required=True,
        )
        if not nav_acquired.authorized:
            raise RuntimeError(f'navigation acquisition denied: {nav_acquired.reason}')
        snapshots['navigation_active'] = node.wait_state(
            _authority_state('NAVIGATE_TO_LOCATION', 'ACTIVE', 'NAVIGATE'),
            'navigation ownership active',
        )

        node.publish_control('safety_stop')
        snapshots['navigation_revoked'] = node.wait_state(
            _authority_state('NAVIGATE_TO_LOCATION', 'REVOKED', 'ESTOP'),
            'safety stop revokes navigation authority',
        )
        nav_revoked_generation = int(
            snapshots['navigation_revoked']['mission']['authority_generation'])

        node.publish_control('safety_clear')
        snapshots['navigation_still_revoked'] = node.wait_state(
            _authority_state('NAVIGATE_TO_LOCATION', 'REVOKED', 'RECOVERY'),
            'safety recovery still requires explicit decision',
        )

        nav_release = node.call_authority(
            command=AuthorizeOperation.Request.COMMAND_RELEASE,
            operation=AuthorizeOperation.Request.OP_NAVIGATE_TO_LOCATION,
            request_id='nav-1',
            map_id='floor_2',
            map_revision=1,
            map_release_id='release-1',
            expected_generation=nav_revoked_generation,
        )
        if not nav_release.authorized:
            raise RuntimeError(f'navigation release denied: {nav_release.reason}')
        snapshots['complete'] = node.wait_state(
            _authority_state('NONE', 'IDLE', 'IDLE'),
            'navigation ownership released',
        )

        if output is not None:
            output.write_text(
                json.dumps(snapshots, indent=2, sort_keys=True) + '\n',
                encoding='utf-8',
            )
        print('PHASE_2_MISSION_AUTHORITY_RUNTIME_COMPLETE')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    return run(output)


if __name__ == '__main__':
    raise SystemExit(main())
