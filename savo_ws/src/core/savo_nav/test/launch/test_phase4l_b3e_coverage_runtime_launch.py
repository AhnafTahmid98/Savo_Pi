# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Exercise the complete isolated Coverage action runtime."""

import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
import launch
import launch_ros.actions
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from savo_msgs.action import ExecuteCoveragePath
from std_msgs.msg import Bool
from std_msgs.msg import String


PUBLIC_COVERAGE = '/test/coverage'
INTERNAL_COVERAGE = '/test/internal/coverage'
PUBLIC_NAVIGATION = '/test/navigation'
INTERNAL_NAVIGATION = '/test/internal/navigation'
PUBLIC_EXPLORATION = '/test/exploration'
INTERNAL_EXPLORATION = '/test/internal/exploration'
FOLLOW_PATH = '/test/follow_path'
FOLLOW_PATH_MODE = '/test/follow_path_mode'

GUARD_ALLOWED = '/test/guard/allowed'
GUARD_REASON = '/test/guard/reason'


@pytest.mark.launch_test
def generate_test_description():
    """Launch only the isolated fake Coverage chain."""
    fake_follow_path = launch_ros.actions.Node(
        package='savo_nav',
        executable='fake_follow_path_server_node',
        name='fake_follow_path_server_node',
        output='screen',
        parameters=[
            {
                'action_name': FOLLOW_PATH,
                'mode_topic': FOLLOW_PATH_MODE,
            }
        ],
    )

    gateway = launch_ros.actions.Node(
        package='savo_nav',
        executable='goal_gateway_node',
        name='goal_gateway_node',
        output='screen',
        parameters=[
            {
                'navigation_action_name': INTERNAL_NAVIGATION,
                'exploration_action_name': INTERNAL_EXPLORATION,
                'nav2_action_name': '/test/navigate_to_pose',
                'coverage_action_name': INTERNAL_COVERAGE,
                'nav2_follow_path_action_name': FOLLOW_PATH,
                'map_mode': 'saved_map',
                'active_map_id': 'coverage_test_map',
                'map_revision': 1,
                'recent_history_capacity': 32,
                'allow_behavior_tree_override': False,
                'execution_timeout_seconds': 5.0,
                'feedback_stale_timeout_seconds': 0.8,
                'coverage_maximum_execution_timeout_seconds': 6.0,
                'coverage_cancel_timeout_seconds': 0.6,
                'max_abs_coordinate_m': 50.0,
                'allow_degraded_readiness': False,
                'coverage_controller_id': '',
                'coverage_goal_checker_id': '',
                'coverage_progress_checker_id': '',
            }
        ],
    )

    gate = launch_ros.actions.Node(
        package='savo_nav',
        executable='goal_admission_gate_node',
        name='goal_admission_gate_node',
        output='screen',
        parameters=[
            {
                'publish_hz': 50.0,
                'guard_timeout_seconds': 2.0,
                'internal_server_timeout_seconds': 2.0,
                'control_recovery_allowed_topic': GUARD_ALLOWED,
                'control_recovery_reason_topic': GUARD_REASON,
                'public_navigation_action': PUBLIC_NAVIGATION,
                'public_exploration_action': PUBLIC_EXPLORATION,
                'internal_navigation_action': INTERNAL_NAVIGATION,
                'internal_exploration_action': INTERNAL_EXPLORATION,
                'public_coverage_action': PUBLIC_COVERAGE,
                'internal_coverage_action': INTERNAL_COVERAGE,
                'state_topic': '/test/gate/state',
                'reason_topic': '/test/gate/reason',
                'status_topic': '/test/gate/status',
            }
        ],
    )

    return launch.LaunchDescription(
        [
            fake_follow_path,
            gateway,
            gate,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestCoverageRuntime(unittest.TestCase):
    """Run success, cancellation and quarantine scenarios."""

    @classmethod
    def setUpClass(cls):
        """Create one test client and prerequisite publishers."""
        rclpy.init()

        cls.node = rclpy.create_node(
            'phase4l_b3e_coverage_test_client'
        )

        retained_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        cls.guard_allowed_publisher = (
            cls.node.create_publisher(
                Bool,
                GUARD_ALLOWED,
                retained_qos,
            )
        )

        cls.guard_reason_publisher = (
            cls.node.create_publisher(
                String,
                GUARD_REASON,
                retained_qos,
            )
        )

        cls.readiness_publisher = (
            cls.node.create_publisher(
                String,
                '/savo_nav/readiness',
                retained_qos,
            )
        )

        cls.readiness_reason_publisher = (
            cls.node.create_publisher(
                String,
                '/savo_nav/readiness_reason',
                retained_qos,
            )
        )

        cls.backend_mode_publisher = (
            cls.node.create_publisher(
                String,
                FOLLOW_PATH_MODE,
                retained_qos,
            )
        )

        cls.public_coverage_client = ActionClient(
            cls.node,
            ExecuteCoveragePath,
            PUBLIC_COVERAGE,
        )

        cls.internal_coverage_client = ActionClient(
            cls.node,
            ExecuteCoveragePath,
            INTERNAL_COVERAGE,
        )

        cls.navigation_client = ActionClient(
            cls.node,
            NavigateToPose,
            PUBLIC_NAVIGATION,
        )

        cls.prerequisite_timer = cls.node.create_timer(
            0.1,
            cls._publish_prerequisites,
        )

        cls.executor = MultiThreadedExecutor(
            num_threads=4
        )

        cls.executor.add_node(cls.node)

        cls.executor_thread = threading.Thread(
            target=cls.executor.spin,
            daemon=True,
        )

        cls.executor_thread.start()

    @classmethod
    def tearDownClass(cls):
        """Stop the isolated test client."""
        cls.prerequisite_timer.cancel()

        cls.executor.shutdown(
            timeout_sec=2.0
        )

        cls.executor_thread.join(
            timeout=2.0
        )

        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _publish_prerequisites(cls):
        """Publish a continuously fresh admission state."""
        cls.guard_allowed_publisher.publish(
            Bool(data=True)
        )

        cls.guard_reason_publisher.publish(
            String(data='control_recovery_ready')
        )

        cls.readiness_publisher.publish(
            String(data='ready')
        )

        cls.readiness_reason_publisher.publish(
            String(data='coverage_test_ready')
        )

    def _wait_future(self, future, timeout):
        """Wait for a future serviced by the executor thread."""
        deadline = time.monotonic() + timeout

        while (
            not future.done() and
            time.monotonic() < deadline
        ):
            time.sleep(0.02)

        self.assertTrue(
            future.done(),
            'Timed out waiting for ROS future',
        )

        return future.result()

    def _wait_condition(self, condition, timeout):
        """Wait for a callback-driven condition."""
        deadline = time.monotonic() + timeout

        while (
            not condition() and
            time.monotonic() < deadline
        ):
            time.sleep(0.02)

        self.assertTrue(
            condition(),
            'Timed out waiting for test condition',
        )

    @staticmethod
    def _duration(seconds):
        """Convert floating seconds to a ROS duration."""
        whole_seconds = int(seconds)

        nanoseconds = int(
            (seconds - whole_seconds) * 1_000_000_000
        )

        return Duration(
            sec=whole_seconds,
            nanosec=nanoseconds,
        )

    def _set_backend_mode(self, mode):
        """Select one private fake-backend scenario."""
        message = String(data=mode)

        for _ in range(3):
            self.backend_mode_publisher.publish(message)
            time.sleep(0.05)

    def _coverage_goal(
        self,
        mission_id,
        mode,
        execution_timeout=5.0,
    ):
        """Create one valid six-metre Coverage path."""
        self._set_backend_mode(mode)

        goal = ExecuteCoveragePath.Goal()

        goal.contract_version = (
            ExecuteCoveragePath.Goal.CONTRACT_VERSION
        )

        goal.mission_id = mission_id

        goal.execution_timeout = self._duration(
            execution_timeout
        )

        goal.path = Path()
        goal.path.header.frame_id = 'map'

        for index in range(4):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(index * 2)
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            goal.path.poses.append(pose)

        return goal

    def _send_goal(
        self,
        client,
        goal,
        feedback_callback=None,
    ):
        """Send an action goal and return its goal handle."""
        future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback,
        )

        return self._wait_future(
            future,
            5.0,
        )

    def _wait_result(
        self,
        goal_handle,
        timeout=8.0,
    ):
        """Wait for an action terminal result."""
        return self._wait_future(
            goal_handle.get_result_async(),
            timeout,
        )

    def test_complete_coverage_runtime_matrix(self):
        """Exercise B3E1, B3E2 and B3E3."""
        self.assertTrue(
            self.public_coverage_client.wait_for_server(
                timeout_sec=10.0
            )
        )

        self.assertTrue(
            self.internal_coverage_client.wait_for_server(
                timeout_sec=10.0
            )
        )

        self.assertTrue(
            self.navigation_client.wait_for_server(
                timeout_sec=10.0
            )
        )

        time.sleep(0.8)

        # ------------------------------------------------------
        # E2: end-to-end success and feedback
        # ------------------------------------------------------

        completion_ratios = []

        def success_feedback(message):
            completion_ratios.append(
                message.feedback.completion_ratio
            )

        success_handle = self._send_goal(
            self.public_coverage_client,
            self._coverage_goal(
                'coverage-success',
                'test_success',
            ),
            success_feedback,
        )

        self.assertTrue(success_handle.accepted)

        success = self._wait_result(success_handle)

        self.assertEqual(
            success.status,
            GoalStatus.STATUS_SUCCEEDED,
        )

        self.assertTrue(success.result.success)

        self.assertEqual(
            success.result.result_code,
            ExecuteCoveragePath.Result.RESULT_SUCCEEDED,
        )

        self.assertGreaterEqual(
            len(completion_ratios),
            2,
        )

        self.assertEqual(
            completion_ratios,
            sorted(completion_ratios),
        )

        time.sleep(0.2)

        # ------------------------------------------------------
        # E2: backend rejection propagation
        # ------------------------------------------------------

        rejected_handle = self._send_goal(
            self.public_coverage_client,
            self._coverage_goal(
                'coverage-backend-rejection',
                'test_reject',
            ),
        )

        self.assertTrue(rejected_handle.accepted)

        rejected = self._wait_result(
            rejected_handle
        )

        self.assertEqual(
            rejected.status,
            GoalStatus.STATUS_ABORTED,
        )

        self.assertEqual(
            rejected.result.result_code,
            ExecuteCoveragePath.Result.RESULT_BACKEND_REJECTED,
        )

        time.sleep(0.2)

        # ------------------------------------------------------
        # E2: shared gate slot and normal cancellation
        # ------------------------------------------------------

        hold_feedback = []

        def active_feedback(message):
            hold_feedback.append(
                message.feedback.remaining_distance_m
            )

        hold_handle = self._send_goal(
            self.public_coverage_client,
            self._coverage_goal(
                'coverage-shared-slot',
                'test_hold',
            ),
            active_feedback,
        )

        self.assertTrue(hold_handle.accepted)

        self._wait_condition(
            lambda: len(hold_feedback) > 0,
            3.0,
        )

        navigation_goal = NavigateToPose.Goal()
        navigation_goal.pose.header.frame_id = 'map'
        navigation_goal.pose.pose.orientation.w = 1.0

        blocked_navigation = self._send_goal(
            self.navigation_client,
            navigation_goal,
        )

        self.assertFalse(
            blocked_navigation.accepted
        )

        cancel_response = self._wait_future(
            hold_handle.cancel_goal_async(),
            3.0,
        )

        self.assertGreaterEqual(
            len(cancel_response.goals_canceling),
            1,
        )

        canceled = self._wait_result(
            hold_handle
        )

        self.assertEqual(
            canceled.status,
            GoalStatus.STATUS_CANCELED,
        )

        self.assertEqual(
            canceled.result.result_code,
            ExecuteCoveragePath.Result.RESULT_CANCELED,
        )

        time.sleep(0.3)

        # ------------------------------------------------------
        # E3: execution timeout with fresh feedback
        # ------------------------------------------------------

        timeout_handle = self._send_goal(
            self.internal_coverage_client,
            self._coverage_goal(
                'coverage-execution-timeout',
                'test_execution_timeout',
                execution_timeout=1.0,
            ),
        )

        self.assertTrue(timeout_handle.accepted)

        timed_out = self._wait_result(
            timeout_handle
        )

        self.assertEqual(
            timed_out.status,
            GoalStatus.STATUS_ABORTED,
        )

        self.assertEqual(
            timed_out.result.result_code,
            ExecuteCoveragePath.Result.RESULT_TIMED_OUT,
        )

        time.sleep(0.3)

        # ------------------------------------------------------
        # E3: stale feedback watchdog
        # ------------------------------------------------------

        stale_handle = self._send_goal(
            self.internal_coverage_client,
            self._coverage_goal(
                'coverage-feedback-stale',
                'test_stale',
                execution_timeout=5.0,
            ),
        )

        self.assertTrue(stale_handle.accepted)

        stale = self._wait_result(
            stale_handle
        )

        self.assertEqual(
            stale.status,
            GoalStatus.STATUS_ABORTED,
        )

        self.assertEqual(
            stale.result.result_code,
            ExecuteCoveragePath.Result.RESULT_FEEDBACK_STALE,
        )

        time.sleep(0.3)

        # ------------------------------------------------------
        # E3: cancel timeout and late-result quarantine
        # ------------------------------------------------------

        late_handle = self._send_goal(
            self.internal_coverage_client,
            self._coverage_goal(
                'coverage-late-success',
                'test_late_success',
                execution_timeout=5.0,
            ),
        )

        self.assertTrue(late_handle.accepted)

        time.sleep(0.2)

        late_cancel = self._wait_future(
            late_handle.cancel_goal_async(),
            3.0,
        )

        self.assertGreaterEqual(
            len(late_cancel.goals_canceling),
            1,
        )

        quarantined_terminal = self._wait_result(
            late_handle,
            timeout=4.0,
        )

        self.assertEqual(
            quarantined_terminal.status,
            GoalStatus.STATUS_ABORTED,
        )

        self.assertFalse(
            quarantined_terminal.result.success
        )

        busy_handle = self._send_goal(
            self.internal_coverage_client,
            self._coverage_goal(
                'coverage-quarantine-blocked',
                'test_success',
            ),
        )

        self.assertFalse(busy_handle.accepted)

        # The fake backend returns late success after 1.5 seconds.
        # The gateway must then release its retained ownership.
        time.sleep(1.5)

        released_handle = self._send_goal(
            self.internal_coverage_client,
            self._coverage_goal(
                'coverage-after-quarantine',
                'test_success',
            ),
        )

        self.assertTrue(released_handle.accepted)

        released = self._wait_result(
            released_handle
        )

        self.assertEqual(
            released.status,
            GoalStatus.STATUS_SUCCEEDED,
        )

        self.assertTrue(released.result.success)
