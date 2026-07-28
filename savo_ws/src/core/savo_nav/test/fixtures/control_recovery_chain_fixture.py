# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise the Phase 7C control, guard, and admission chain."""

import argparse
from pathlib import Path
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


PUBLIC_ACTION = '/savo_nav/navigation/navigate_to_pose'


class ControlRecoveryChainFixture(Node):
    """Publish raw evidence and observe the guarded public action."""

    def __init__(self):
        """Create raw-evidence publishers and chain observers."""
        super().__init__('control_recovery_chain_fixture')

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
        )

        self.mode_state_publisher = self.create_publisher(
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

        self.guard_allowed = None
        self.guard_reason = None
        self.gate_state = None
        self.gate_reason = None
        self.cancel_count = None

        self.guard_allowed_subscription = self.create_subscription(
            Bool,
            '/savo_nav/control_recovery/allowed',
            self.on_guard_allowed,
            retained_qos,
        )
        self.guard_reason_subscription = self.create_subscription(
            String,
            '/savo_nav/control_recovery/reason',
            self.on_guard_reason,
            retained_qos,
        )
        self.gate_state_subscription = self.create_subscription(
            String,
            '/savo_nav/goal_admission/state',
            self.on_gate_state,
            retained_qos,
        )
        self.gate_reason_subscription = self.create_subscription(
            String,
            '/savo_nav/goal_admission/reason',
            self.on_gate_reason,
            retained_qos,
        )
        self.cancel_count_subscription = self.create_subscription(
            Int32,
            '/test/internal_gateway/cancel_count',
            self.on_cancel_count,
            retained_qos,
        )

        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            PUBLIC_ACTION,
        )

    def on_guard_allowed(self, message):
        """Store the latest guard permission."""
        self.guard_allowed = message.data

    def on_guard_reason(self, message):
        """Store the latest deterministic guard reason."""
        self.guard_reason = message.data

    def on_gate_state(self, message):
        """Store the latest admission-gate state."""
        self.gate_state = message.data

    def on_gate_reason(self, message):
        """Store the latest admission-gate reason."""
        self.gate_reason = message.data

    def on_cancel_count(self, message):
        """Store the hidden gateway cancellation count."""
        self.cancel_count = message.data

    def publish_raw_evidence(self, mode, recovery_active):
        """Publish one complete read-only control/recovery snapshot."""
        mode_message = String()
        mode_message.data = mode
        self.mode_state_publisher.publish(mode_message)

        mode_reason = String()
        mode_reason.data = f'phase7c_mode_{mode.lower()}'
        self.mode_reason_publisher.publish(mode_reason)

        control_status = String()
        control_status.data = f'mode={mode};source=phase7c_fixture'
        self.control_status_publisher.publish(control_status)

        recovery_message = Bool()
        recovery_message.data = recovery_active
        self.recovery_active_publisher.publish(recovery_message)

        recovery_state = String()
        recovery_state.data = (
            'active' if recovery_active else 'clear'
        )
        self.recovery_state_publisher.publish(recovery_state)

        recovery_status = String()
        recovery_status.data = (
            f'active={str(recovery_active).lower()};'
            'source=phase7c_fixture'
        )
        self.recovery_status_publisher.publish(recovery_status)

    def publish_for(self, mode, recovery_active, seconds):
        """Publish fresh raw evidence while servicing callbacks."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publish_raw_evidence(mode, recovery_active)
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_for(self, seconds):
        """Service callbacks without publishing raw evidence."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout_seconds, description):
        """Wait for a predicate while servicing ROS callbacks."""
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f'Timed out waiting for {description}.')

    def wait_future(self, future, timeout_seconds, description):
        """Wait for an action future and return its result."""
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_seconds,
        )
        if not future.done():
            raise RuntimeError(f'Timed out waiting for {description}.')
        result = future.result()
        if result is None:
            raise RuntimeError(f'{description} returned no result.')
        return result

    def wait_public_server(self):
        """Wait for the public navigation action server."""
        if not self.navigation_client.wait_for_server(
            timeout_sec=8.0
        ):
            raise RuntimeError('Public navigation action is unavailable.')

    def make_goal(self, x_coordinate):
        """Create one valid map-frame navigation goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x_coordinate
        goal.pose.pose.orientation.w = 1.0
        return goal

    def send_goal(self, x_coordinate):
        """Send one public navigation goal."""
        future = self.navigation_client.send_goal_async(
            self.make_goal(x_coordinate)
        )
        return self.wait_future(
            future,
            8.0,
            'goal response',
        )

    def wait_result(self, goal_handle, timeout_seconds=10.0):
        """Wait for a public action result."""
        return self.wait_future(
            goal_handle.get_result_async(),
            timeout_seconds,
            'goal result',
        )

    def assert_rejected(self):
        """Send a normal goal and require fail-closed rejection."""
        goal_handle = self.send_goal(1.0)
        if goal_handle.accepted:
            raise RuntimeError('Fail-closed goal was accepted.')

    def assert_success(self):
        """Send a normal goal and require successful forwarding."""
        goal_handle = self.send_goal(1.0)
        if not goal_handle.accepted:
            raise RuntimeError('Allowed goal was rejected.')
        result = self.wait_result(goal_handle)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'Expected success, received status {result.status}.'
            )

    def start_held_goal(self):
        """Start a hidden-gateway goal that waits for cancellation."""
        goal_handle = self.send_goal(2.0)
        if not goal_handle.accepted:
            raise RuntimeError('Held goal was rejected.')
        return goal_handle

    def require_abort(self, goal_handle, expected_reason=None):
        """Require an aborted goal with an optional exact reason."""
        result = self.wait_result(goal_handle)
        if result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError(
                f'Expected abort, received status {result.status}.'
            )
        actual_reason = result.result.error_msg
        if (
            expected_reason is not None
            and actual_reason != expected_reason
        ):
            raise RuntimeError(
                'Abort reason mismatch: '
                f'actual={actual_reason}, expected={expected_reason}'
            )
        return actual_reason

    def wait_cancel_count(self, expected, timeout_seconds=5.0):
        """Wait for an exact hidden-gateway cancel count."""
        self.wait_until(
            lambda: self.cancel_count == expected,
            timeout_seconds,
            f'cancel count {expected}',
        )

    def scenario_startup_blocked(self):
        """Validate public-server availability and fail-closed startup."""
        self.wait_public_server()
        self.assert_rejected()

    def scenario_success(self):
        """Validate fresh NAV evidence permits a successful goal."""
        self.wait_public_server()
        self.publish_for('NAV', False, 0.9)
        self.wait_until(
            lambda: self.guard_allowed is True,
            3.0,
            'guard permission',
        )
        self.assert_success()

    def scenario_full_chain(self):
        """Validate raw evidence, interruption, and restoration."""
        self.wait_public_server()

        self.publish_for('STOP', False, 0.9)
        self.wait_until(
            lambda: (
                self.guard_allowed is False
                and self.guard_reason == 'control_mode_not_nav'
            ),
            3.0,
            'STOP-mode guard rejection',
        )
        self.assert_rejected()
        print('Raw STOP-mode admission blocking: PASSED')

        self.publish_for('NAV', False, 0.9)
        self.wait_until(
            lambda: self.guard_allowed is True,
            3.0,
            'NAV-mode guard permission',
        )
        self.assert_success()
        print('Raw NAV-mode admission: PASSED')

        baseline = self.cancel_count or 0
        self.publish_for('NAV', False, 0.5)
        held_goal = self.start_held_goal()
        self.publish_for('NAV', False, 0.3)
        result_future = held_goal.get_result_async()
        self.publish_for('STOP', False, 1.0)
        result = self.wait_future(
            result_future,
            8.0,
            'STOP interruption result',
        )
        if result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError('STOP did not abort the active goal.')
        if result.result.error_msg != 'control_mode_not_nav':
            raise RuntimeError('STOP interruption reason mismatch.')
        self.wait_cancel_count(baseline + 1)
        print('STOP-mode active-goal interruption: PASSED')

        self.publish_for('STOP', False, 0.8)
        self.wait_cancel_count(baseline + 1)
        print('STOP interruption one-shot cancellation: PASSED')

        self.publish_for('NAV', False, 0.9)
        held_goal = self.start_held_goal()
        self.publish_for('NAV', False, 0.3)
        result_future = held_goal.get_result_async()
        self.publish_for('NAV', True, 1.0)
        result = self.wait_future(
            result_future,
            8.0,
            'recovery interruption result',
        )
        if result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError('Recovery did not abort the active goal.')
        if result.result.error_msg != 'recovery_active':
            raise RuntimeError('Recovery interruption reason mismatch.')
        self.wait_cancel_count(baseline + 2)
        print('Recovery-active goal interruption: PASSED')

        self.publish_for('NAV', True, 0.8)
        self.wait_cancel_count(baseline + 2)
        print('Recovery interruption one-shot cancellation: PASSED')

        self.publish_for('NAV', False, 0.9)
        held_goal = self.start_held_goal()
        self.publish_for('NAV', False, 0.3)
        stale_reason = self.require_abort(
            held_goal,
            expected_reason=None,
        )
        if 'stale' not in stale_reason:
            raise RuntimeError(
                f'Stale interruption reason missing: {stale_reason}'
            )
        self.wait_cancel_count(baseline + 3)
        print('Raw control-evidence staleness interruption: PASSED')

        self.publish_for('NAV', False, 0.9)
        self.wait_until(
            lambda: self.guard_allowed is True,
            3.0,
            'fresh-evidence restoration',
        )
        self.assert_success()
        if (
            self.guard_reason is None
            or self.gate_state is None
            or self.gate_reason is None
        ):
            raise RuntimeError('Guard or gate observer topic missing.')
        print('Fresh-evidence restoration: PASSED')

    def scenario_guard_loss(self, ready_file):
        """Validate guard-process loss and restart recovery."""
        self.wait_public_server()
        self.publish_for('NAV', False, 0.9)
        self.wait_until(
            lambda: self.guard_allowed is True,
            3.0,
            'guard permission before process loss',
        )
        held_goal = self.start_held_goal()
        self.publish_for('NAV', False, 0.3)
        ready_file.write_text('active\n', encoding='utf-8')

        result_future = held_goal.get_result_async()
        deadline = time.monotonic() + 10.0
        while not result_future.done() and time.monotonic() < deadline:
            self.publish_raw_evidence('NAV', False)
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result_future.done():
            raise RuntimeError('Guard loss did not terminate the goal.')
        result = result_future.result()
        if result.status != GoalStatus.STATUS_ABORTED:
            raise RuntimeError('Guard loss did not abort the goal.')
        if (
            result.result.error_msg
            != 'control_recovery_guard_stale'
        ):
            raise RuntimeError('Guard-loss reason mismatch.')
        print('Guard-process loss interruption: PASSED')

        restart_file = Path(f'{ready_file}.restart')
        restart_file.write_text('restart\n', encoding='utf-8')
        self.publish_for('NAV', False, 1.4)
        self.wait_until(
            lambda: self.guard_allowed is True,
            5.0,
            'guard restart permission',
        )
        self.assert_success()
        print('Guard-process restart recovery: PASSED')

    def scenario_downstream_unavailable(self, ready_file):
        """Validate hidden-gateway absence and restart recovery."""
        self.wait_public_server()
        self.publish_for('NAV', False, 0.9)
        self.wait_until(
            lambda: self.guard_allowed is True,
            3.0,
            'guard permission without downstream',
        )

        goal_handle = self.send_goal(1.0)
        if not goal_handle.accepted:
            raise RuntimeError(
                'Gate unexpectedly rejected the downstream-loss goal.'
            )
        self.require_abort(
            goal_handle,
            'internal_goal_gateway_unavailable',
        )
        print('Downstream gateway absence containment: PASSED')

        if ready_file is None:
            return

        ready_file.write_text('restart\n', encoding='utf-8')
        self.publish_for('NAV', False, 1.4)
        self.assert_success()
        print('Hidden gateway restart recovery: PASSED')


def parse_arguments():
    """Parse the requested Phase 7C runtime scenario."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'scenario',
        choices=(
            'startup-blocked',
            'full-chain',
            'guard-loss',
            'downstream-unavailable',
            'success',
        ),
    )
    parser.add_argument('--ready-file', type=Path)
    return parser.parse_args()


def main():
    """Run one requested Phase 7C runtime scenario."""
    arguments = parse_arguments()
    rclpy.init()
    node = ControlRecoveryChainFixture()

    try:
        if arguments.scenario == 'startup-blocked':
            node.scenario_startup_blocked()
        elif arguments.scenario == 'full-chain':
            node.scenario_full_chain()
        elif arguments.scenario == 'guard-loss':
            if arguments.ready_file is None:
                raise RuntimeError('guard-loss requires --ready-file.')
            node.scenario_guard_loss(arguments.ready_file)
        elif arguments.scenario == 'downstream-unavailable':
            node.scenario_downstream_unavailable(
                arguments.ready_file
            )
        elif arguments.scenario == 'success':
            node.scenario_success()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
