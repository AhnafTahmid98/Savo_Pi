# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise Phase 7B admission and interruption behavior."""

import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String


class GateFixture(Node):
    """Drive public goals and publish guard evidence."""

    def __init__(self):
        """Create clients, publishers, and observers."""
        super().__init__('goal_admission_gate_fixture')

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.allowed_publisher = self.create_publisher(
            Bool,
            '/savo_nav/control_recovery/allowed',
            retained_qos,
        )

        self.reason_publisher = self.create_publisher(
            String,
            '/savo_nav/control_recovery/reason',
            retained_qos,
        )

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            '/savo_nav/navigation/navigate_to_pose',
        )

        self.exploration_client = ActionClient(
            self,
            NavigateToPose,
            '/savo_nav/exploration/navigate_to_pose',
        )

        self.cancel_count = None
        self.gate_state = None
        self.gate_reason = None

        self.cancel_count_subscription = self.create_subscription(
            Int32,
            '/test/internal_gateway/cancel_count',
            self.on_cancel_count,
            retained_qos,
        )

        self.state_subscription = self.create_subscription(
            String,
            '/savo_nav/goal_admission/state',
            self.on_gate_state,
            retained_qos,
        )

        self.reason_subscription = self.create_subscription(
            String,
            '/savo_nav/goal_admission/reason',
            self.on_gate_reason,
            retained_qos,
        )

    def on_cancel_count(self, message):
        """Store the internal cancel count."""
        self.cancel_count = message.data

    def on_gate_state(self, message):
        """Store the gate state."""
        self.gate_state = message.data

    def on_gate_reason(self, message):
        """Store the gate reason."""
        self.gate_reason = message.data

    def publish_guard(self, allowed, reason):
        """Publish one retained guard snapshot."""
        allowed_message = Bool()
        allowed_message.data = allowed
        self.allowed_publisher.publish(allowed_message)

        reason_message = String()
        reason_message.data = reason
        self.reason_publisher.publish(reason_message)

    def publish_guard_for(self, allowed, reason, seconds):
        """Publish guard evidence while spinning."""
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            self.publish_guard(allowed, reason)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_future(self, future, timeout_seconds):
        """Wait for a future without relying on return enums."""
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_seconds,
        )

        if not future.done():
            raise RuntimeError('Timed out waiting for future.')

        return future.result()

    def wait_servers(self):
        """Wait for both public action servers."""
        if not self.navigation_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError('Navigation gate server missing.')

        if not self.exploration_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError('Exploration gate server missing.')

    def make_goal(self, x):
        """Create a valid planar map goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.orientation.w = 1.0
        return goal

    def send_goal(self, client, x):
        """Send one goal and return its handle."""
        future = client.send_goal_async(self.make_goal(x))
        return self.wait_future(future, 5.0)

    def wait_result(self, goal_handle, timeout_seconds=8.0):
        """Wait for one action result."""
        future = goal_handle.get_result_async()
        return self.wait_future(future, timeout_seconds)

    def wait_cancel_count(self, expected, timeout_seconds=3.0):
        """Wait for the retained internal cancellation count."""
        deadline = time.monotonic() + timeout_seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.cancel_count == expected:
                return

        raise RuntimeError(
            'Cancel count mismatch: '
            f'actual={self.cancel_count}, expected={expected}'
        )


def main():
    """Run all Phase 7B scenarios."""
    rclpy.init()
    node = GateFixture()

    try:
        node.wait_servers()

        node.publish_guard_for(False, 'control_mode_not_nav', 0.5)
        blocked = node.send_goal(node.navigation_client, 1.0)

        if blocked.accepted:
            raise RuntimeError('Blocked goal was accepted.')

        print('Blocked-goal admission rejection: PASSED')

        node.publish_guard_for(True, 'control_recovery_ready', 0.5)
        success_goal = node.send_goal(node.navigation_client, 1.0)

        if not success_goal.accepted:
            raise RuntimeError('Ready navigation goal was rejected.')

        success_result = node.wait_result(success_goal)

        if success_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Ready navigation goal did not succeed.')

        print('Ready-goal forwarding: PASSED')

        baseline = node.cancel_count or 0
        node.publish_guard_for(True, 'control_recovery_ready', 0.3)
        recovery_goal = node.send_goal(node.navigation_client, 2.0)

        if not recovery_goal.accepted:
            raise RuntimeError('Recovery-interruption goal was rejected.')

        node.publish_guard_for(False, 'recovery_active', 1.0)
        recovery_result = node.wait_result(recovery_goal)

        if recovery_result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError(
                'Guard interruption did not abort the public goal.'
            )

        if recovery_result.result.error_msg != 'recovery_active':
            raise RuntimeError(
                'Guard interruption reason was not preserved.'
            )

        node.wait_cancel_count(baseline + 1)
        node.publish_guard_for(False, 'recovery_active', 0.5)
        node.wait_cancel_count(baseline + 1)

        print('Recovery-active interruption: PASSED')
        print('One-shot internal cancellation: PASSED')

        node.publish_guard_for(True, 'control_recovery_ready', 0.4)
        stale_goal = node.send_goal(node.navigation_client, 2.0)

        if not stale_goal.accepted:
            raise RuntimeError('Stale-watch goal was rejected.')

        stale_result = node.wait_result(stale_goal)

        if stale_result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError('Stale guard did not interrupt the goal.')

        if (
            stale_result.result.error_msg
            != 'control_recovery_guard_stale'
        ):
            raise RuntimeError('Stale interruption reason mismatch.')

        node.wait_cancel_count(baseline + 2)
        print('Guard-staleness interruption: PASSED')

        stale_rejection = node.send_goal(
            node.navigation_client,
            1.0,
        )

        if stale_rejection.accepted:
            raise RuntimeError('Goal was accepted with stale guard.')

        print('Stale-guard admission rejection: PASSED')

        node.publish_guard_for(True, 'control_recovery_ready', 0.5)
        external_cancel_goal = node.send_goal(
            node.navigation_client,
            2.0,
        )

        if not external_cancel_goal.accepted:
            raise RuntimeError('External-cancel goal was rejected.')

        cancel_future = external_cancel_goal.cancel_goal_async()
        node.wait_future(cancel_future, 5.0)
        external_cancel_result = node.wait_result(external_cancel_goal)

        if (
            external_cancel_result.status
            != GoalStatus.STATUS_CANCELED
        ):
            raise RuntimeError('External cancellation was not preserved.')

        node.wait_cancel_count(baseline + 3)
        print('External cancellation forwarding: PASSED')

        node.publish_guard_for(True, 'control_recovery_ready', 0.4)
        exploration_goal = node.send_goal(
            node.exploration_client,
            3.0,
        )

        if not exploration_goal.accepted:
            raise RuntimeError('Exploration goal was rejected.')

        exploration_result = node.wait_result(exploration_goal)

        if exploration_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Exploration goal did not succeed.')

        print('Exploration forwarding through gate: PASSED')

        if node.gate_state is None or node.gate_reason is None:
            raise RuntimeError('Gate observer topics were not received.')

        print('Goal-admission observer topics: PASSED')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
