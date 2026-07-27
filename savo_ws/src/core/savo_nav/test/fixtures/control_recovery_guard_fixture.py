# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise the Phase 7A control/recovery guard."""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


class GuardFixture(Node):
    """Publish control evidence and observe the guard."""

    def __init__(self):
        """Create fixture publishers and subscriptions."""
        super().__init__('control_recovery_guard_fixture')

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.mode_publisher = self.create_publisher(
            String,
            '/savo_control/mode_state',
            retained_qos,
        )

        self.mode_reason_publisher = self.create_publisher(
            String,
            '/savo_control/mode_reason',
            event_qos,
        )

        self.control_status_publisher = self.create_publisher(
            String,
            '/savo_control/control_status',
            event_qos,
        )

        self.recovery_active_publisher = self.create_publisher(
            Bool,
            '/savo_control/recovery_active',
            event_qos,
        )

        self.recovery_state_publisher = self.create_publisher(
            String,
            '/savo_control/recovery_state',
            event_qos,
        )

        self.recovery_status_publisher = self.create_publisher(
            String,
            '/savo_control/recovery_status',
            event_qos,
        )

        self.allowed = None
        self.reason = None
        self.status = None

        self.allowed_subscription = self.create_subscription(
            Bool,
            '/savo_nav/control_recovery/allowed',
            self.on_allowed,
            retained_qos,
        )

        self.reason_subscription = self.create_subscription(
            String,
            '/savo_nav/control_recovery/reason',
            self.on_reason,
            retained_qos,
        )

        self.status_subscription = self.create_subscription(
            String,
            '/savo_nav/control_recovery/status',
            self.on_status,
            retained_qos,
        )

    def on_allowed(self, message):
        """Store the latest permission state."""
        self.allowed = message.data

    def on_reason(self, message):
        """Store the latest reason."""
        self.reason = message.data

    def on_status(self, message):
        """Store the latest status."""
        self.status = message.data

    def publish_evidence(self, mode, recovery_active):
        """Publish one complete control/recovery snapshot."""
        mode_message = String()
        mode_message.data = mode
        self.mode_publisher.publish(mode_message)

        mode_reason = String()
        mode_reason.data = 'REQUESTED'
        self.mode_reason_publisher.publish(mode_reason)

        control_status = String()
        control_status.data = (
            f'mode={mode};reason=REQUESTED'
        )
        self.control_status_publisher.publish(
            control_status
        )

        recovery_message = Bool()
        recovery_message.data = recovery_active
        self.recovery_active_publisher.publish(
            recovery_message
        )

        recovery_state = String()
        recovery_state.data = (
            'active'
            if recovery_active
            else 'idle'
        )
        self.recovery_state_publisher.publish(
            recovery_state
        )

        recovery_status = String()
        recovery_status.data = (
            f'active={str(recovery_active).lower()}'
        )
        self.recovery_status_publisher.publish(
            recovery_status
        )

    def wait_for_decision(
        self,
        expected_allowed,
        expected_reason,
        timeout_seconds,
        publish_callback=None,
    ):
        """Wait for a matching retained guard decision."""
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            if publish_callback is not None:
                publish_callback()

            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.allowed == expected_allowed
                and self.reason == expected_reason
            ):
                return

        raise RuntimeError(
            'Guard decision mismatch: '
            f'allowed={self.allowed}, '
            f'reason={self.reason}, '
            f'expected_allowed={expected_allowed}, '
            f'expected_reason={expected_reason}'
        )


def main():
    """Run all Phase 7A fixture scenarios."""
    rclpy.init()

    node = GuardFixture()

    try:
        node.wait_for_decision(
            False,
            'control_mode_unobserved',
            3.0,
        )

        print('Unknown-state blocking: PASSED')

        node.wait_for_decision(
            True,
            'control_recovery_ready',
            3.0,
            lambda: node.publish_evidence(
                'NAV',
                False,
            ),
        )

        print('NAV mode with clear recovery: PASSED')

        node.wait_for_decision(
            False,
            'control_mode_not_nav',
            3.0,
            lambda: node.publish_evidence(
                'STOP',
                False,
            ),
        )

        print('STOP mode blocking: PASSED')

        node.wait_for_decision(
            False,
            'recovery_active',
            3.0,
            lambda: node.publish_evidence(
                'NAV',
                True,
            ),
        )

        print('Active recovery blocking: PASSED')

        node.wait_for_decision(
            True,
            'control_recovery_ready',
            3.0,
            lambda: node.publish_evidence(
                'NAV',
                False,
            ),
        )

        print('Recovery-clear restoration: PASSED')

        stale_deadline = time.monotonic() + 1.2

        while time.monotonic() < stale_deadline:
            recovery_message = Bool()
            recovery_message.data = False

            node.recovery_active_publisher.publish(
                recovery_message
            )

            rclpy.spin_once(node, timeout_sec=0.05)

        node.wait_for_decision(
            False,
            'control_mode_stale',
            3.0,
        )

        print('Stale control-mode blocking: PASSED')

        node.wait_for_decision(
            True,
            'control_recovery_ready',
            3.0,
            lambda: node.publish_evidence(
                'NAV',
                False,
            ),
        )

        print('Fresh-state restoration: PASSED')

        if node.status is None:
            raise RuntimeError(
                'Guard status was never published.'
            )

        print('Guard observer status: PASSED')

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
