# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise success, busy rejection and cancellation forwarding."""

import sys
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class GatewayFixtureClient(Node):
    """Drive the two public Savo navigation action servers."""

    def __init__(self):
        """Create publishers and action clients."""
        super().__init__('goal_gateway_fixture_client')

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.readiness_publisher = self.create_publisher(
            String,
            '/savo_nav/readiness',
            state_qos,
        )

        self.reason_publisher = self.create_publisher(
            String,
            '/savo_nav/readiness_reason',
            state_qos,
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

    def publish_ready(self):
        """Publish a retained ready state."""
        state = String()
        state.data = 'ready'

        reason = String()
        reason.data = (
            'all_required_navigation_dependencies_ready'
        )

        self.readiness_publisher.publish(state)
        self.reason_publisher.publish(reason)

    def make_goal(self, x_value, y_value):
        """Construct a valid map-frame Nav2 goal."""
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x_value
        goal.pose.pose.position.y = y_value
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        goal.behavior_tree = ''

        return goal

    def wait_for_servers(self):
        """Wait for both public gateway action servers."""
        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            navigation_ready = (
                self.navigation_client.server_is_ready()
            )

            exploration_ready = (
                self.exploration_client.server_is_ready()
            )

            if navigation_ready and exploration_ready:
                return

            self.publish_ready()
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(
            'Public Savo action servers were unavailable.'
        )


def wait_future(node, future, timeout_seconds):
    """Spin until a future completes or timeout expires."""
    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=timeout_seconds,
    )

    if not future.done():
        raise RuntimeError('Timed out waiting for a future.')

    return future.result()


def main():
    """Run the Phase 6 gateway fixture."""
    rclpy.init()

    node = GatewayFixtureClient()

    try:
        node.publish_ready()
        node.wait_for_servers()

        for _ in range(5):
            node.publish_ready()
            rclpy.spin_once(node, timeout_sec=0.1)

        first_future = (
            node.navigation_client.send_goal_async(
                node.make_goal(1.0, 0.0)
            )
        )

        first_handle = wait_future(
            node,
            first_future,
            5.0,
        )

        if not first_handle.accepted:
            raise RuntimeError(
                'Normal navigation goal was rejected.'
            )

        busy_future = (
            node.exploration_client.send_goal_async(
                node.make_goal(2.0, 0.0)
            )
        )

        busy_handle = wait_future(
            node,
            busy_future,
            5.0,
        )

        if busy_handle.accepted:
            raise RuntimeError(
                'Exploration goal replaced an active goal.'
            )

        first_result = wait_future(
            node,
            first_handle.get_result_async(),
            10.0,
        )

        if first_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                'Forwarded normal goal did not succeed.'
            )

        second_future = (
            node.exploration_client.send_goal_async(
                node.make_goal(0.0, 1.0)
            )
        )

        second_handle = wait_future(
            node,
            second_future,
            5.0,
        )

        if not second_handle.accepted:
            raise RuntimeError(
                'Exploration goal was not accepted.'
            )

        end_time = (
            node.get_clock().now()
            + Duration(seconds=0.25)
        )

        while node.get_clock().now() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)

        cancel_response = wait_future(
            node,
            second_handle.cancel_goal_async(),
            5.0,
        )

        if not cancel_response.goals_canceling:
            raise RuntimeError(
                'External cancellation was not accepted.'
            )

        second_result = wait_future(
            node,
            second_handle.get_result_async(),
            10.0,
        )

        if second_result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError(
                'Canceled goal did not finish as canceled.'
            )

        print('Normal goal forwarding: PASSED')
        print('Busy-goal rejection: PASSED')
        print('Exploration goal forwarding: PASSED')
        print('Cancellation forwarding: PASSED')
        print('Cancellation acknowledgement: PASSED')

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
