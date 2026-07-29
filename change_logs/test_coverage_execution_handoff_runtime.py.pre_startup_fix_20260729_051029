#!/usr/bin/env python3

"""Isolated runtime validation for the Coverage execution handoff."""

import json
import os
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.action import ExecuteCoveragePath
from std_msgs.msg import String
from std_srvs.srv import Trigger


WAIT_TIMEOUT_SEC = 10.0


def wait_until(predicate, timeout=WAIT_TIMEOUT_SEC):
    """Wait until a condition becomes true."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def state_qos():
    """Return the retained reliable state policy."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def status_qos():
    """Return the reliable volatile status policy."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def installed_handoff_executable():
    """Locate the installed production handoff executable."""
    configured = os.environ.get(
        'COVERAGE_HANDOFF_EXECUTABLE',
        '',
    )
    if configured and os.path.isfile(configured):
        return configured

    for prefix in os.environ.get(
        'AMENT_PREFIX_PATH',
        '',
    ).split(os.pathsep):
        candidate = os.path.join(
            prefix,
            'lib',
            'savo_mapping',
            'coverage_execution_handoff_node',
        )
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate

    raise AssertionError(
        'installed coverage_execution_handoff_node was not found'
    )


def make_path(frame_id, offset=0.0):
    """Create a small valid planar Coverage path."""
    path = Path()
    path.header.frame_id = frame_id
    for x_value in (offset, offset + 1.0, offset + 2.0):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x_value
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


class RuntimeHarness:
    """Own the test action server and installed handoff process."""

    def __init__(self, *, start_action_server=True):
        """Start an isolated graph with unique endpoint names."""
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        self.frame_id = f'coverage_map_{suffix}'
        self.plan_topic = f'/test/coverage_plan_{suffix}'
        self.action_name = f'/test/coverage_action_{suffix}'
        self.state_topic = f'/test/coverage_state_{suffix}'
        self.status_topic = f'/test/coverage_status_{suffix}'
        self.feedback_topic = f'/test/coverage_feedback_{suffix}'
        self.approve_service = f'/test/coverage_approve_{suffix}'
        self.cancel_service = f'/test/coverage_cancel_{suffix}'
        self.reset_service = f'/test/coverage_reset_{suffix}'

        self.node = rclpy.create_node(
            f'coverage_handoff_fixture_{suffix}'
        )
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.callback_group = ReentrantCallbackGroup()

        self.mode = 'succeed'
        self.goal_count = 0
        self.received_goals = []
        self.statuses = []
        self.states = []

        self.action_server = None
        if start_action_server:
            self.action_server = ActionServer(
                self.node,
                ExecuteCoveragePath,
                self.action_name,
                execute_callback=self.execute_goal,
                goal_callback=self.accept_goal,
                cancel_callback=self.accept_cancel,
                callback_group=self.callback_group,
            )
        self.path_publisher = self.node.create_publisher(
            Path,
            self.plan_topic,
            state_qos(),
        )
        self.status_subscription = self.node.create_subscription(
            String,
            self.status_topic,
            self.receive_status,
            status_qos(),
        )
        self.state_subscription = self.node.create_subscription(
            String,
            self.state_topic,
            self.receive_state,
            state_qos(),
        )
        self.approve_client = self.node.create_client(
            Trigger,
            self.approve_service,
        )
        self.cancel_client = self.node.create_client(
            Trigger,
            self.cancel_service,
        )
        self.reset_client = self.node.create_client(
            Trigger,
            self.reset_service,
        )

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

        command = [
            installed_handoff_executable(),
            '--ros-args',
            '-p', f'expected_frame:={self.frame_id}',
            '-p', f'plan_topic:={self.plan_topic}',
            '-p', f'action_name:={self.action_name}',
            '-p', f'state_topic:={self.state_topic}',
            '-p', f'status_topic:={self.status_topic}',
            '-p', f'feedback_topic:={self.feedback_topic}',
            '-p', f'approve_service:={self.approve_service}',
            '-p', f'cancel_service:={self.cancel_service}',
            '-p', f'reset_service:={self.reset_service}',
            '-p', 'server_wait_timeout_sec:=1.0',
            '-p', 'goal_response_timeout_sec:=1.0',
            '-p', 'cancel_timeout_sec:=1.0',
            '-p', 'watchdog_period_ms:=20',
        ]
        environment = os.environ.copy()
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        environment['ROS_LOCALHOST_ONLY'] = '1'
        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self.process_output = ''

        assert wait_until(
            lambda: self.process.poll() is not None
            or (
                self.approve_client.service_is_ready()
                and bool(self.statuses)
            )
        ), self.diagnostics()
        assert self.process.poll() is None, self.diagnostics()

    def accept_goal(self, goal_request):
        """Accept every valid request from the production handoff."""
        del goal_request
        return GoalResponse.ACCEPT

    def accept_cancel(self, goal_handle):
        """Acknowledge cancellation for the active fixture goal."""
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle):
        """Return success or wait for cancellation according to mode."""
        self.goal_count += 1
        self.received_goals.append(goal_handle.request)

        feedback = ExecuteCoveragePath.Feedback()
        feedback.state = feedback.STATE_EXECUTING
        feedback.state_text = 'executing'
        feedback.reason = 'fixture_executing'
        feedback.current_waypoint = 1
        feedback.completed_waypoints = 1
        feedback.total_waypoints = len(goal_handle.request.path.poses)
        feedback.completion_ratio = 1.0 / feedback.total_waypoints
        feedback.remaining_distance_m = 1.0
        goal_handle.publish_feedback(feedback)

        if self.mode == 'cancel':
            assert wait_until(
                lambda: goal_handle.is_cancel_requested,
                timeout=5.0,
            )
            goal_handle.canceled()
            result = ExecuteCoveragePath.Result()
            result.success = False
            result.result_code = result.RESULT_CANCELED
            result.terminal_state = 'canceled'
            result.reason = 'fixture_canceled'
            result.completed_waypoints = 1
            result.total_waypoints = len(goal_handle.request.path.poses)
            result.completion_ratio = feedback.completion_ratio
            result.remaining_distance_m = 1.0
            return result

        time.sleep(0.05)
        goal_handle.succeed()
        result = ExecuteCoveragePath.Result()
        result.success = True
        result.result_code = result.RESULT_SUCCEEDED
        result.terminal_state = 'succeeded'
        result.reason = 'fixture_succeeded'
        result.completed_waypoints = len(goal_handle.request.path.poses)
        result.total_waypoints = len(goal_handle.request.path.poses)
        result.completion_ratio = 1.0
        result.remaining_distance_m = 0.0
        return result

    def receive_status(self, message):
        """Decode a handoff status snapshot."""
        try:
            self.statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    def receive_state(self, message):
        """Record a stable state transition."""
        self.states.append(message.data)

    def call(self, client):
        """Call one Trigger service and return its response."""
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        assert wait_until(future.done), self.diagnostics()
        return future.result()

    def latest_status(self):
        """Return the latest decoded status."""
        return self.statuses[-1] if self.statuses else {}

    def diagnostics(self):
        """Return bounded failure context."""
        return (
            f'process={self.process.poll()} '
            f'goals={self.goal_count} '
            f'states={self.states[-8:]!r} '
            f'statuses={self.statuses[-3:]!r} '
            f'output={self.process_output!r}'
        )

    def stop(self):
        """Stop the child process and fixture graph."""
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process_output = self.process.communicate(
                    timeout=5.0
                )[0]
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process_output = self.process.communicate()[0]
        if self.action_server is not None:
            self.action_server.destroy()
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize one ROS context for this isolated test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_explicit_approval_success_and_cancel():
    """Validate B3F staging, approval, feedback, result and cancel."""
    harness = RuntimeHarness()
    try:
        harness.path_publisher.publish(
            make_path(harness.frame_id)
        )
        assert wait_until(
            lambda: harness.latest_status().get('candidate_valid')
        ), harness.diagnostics()

        time.sleep(0.25)
        assert harness.goal_count == 0, harness.diagnostics()
        assert 'plan_available' in harness.states

        approval = harness.call(harness.approve_client)
        assert approval.success, approval.message
        assert approval.message.startswith('coverage-')
        assert wait_until(
            lambda: harness.goal_count == 1
            and harness.latest_status().get('state') == 'succeeded'
        ), harness.diagnostics()
        assert harness.received_goals[0].mission_id == approval.message
        assert len(harness.received_goals[0].path.poses) == 3
        assert harness.latest_status()['result_code'] == 0

        reset = harness.call(harness.reset_client)
        assert reset.success, reset.message

        harness.mode = 'cancel'
        harness.path_publisher.publish(
            make_path(harness.frame_id, offset=10.0)
        )
        assert wait_until(
            lambda: harness.latest_status().get(
                'candidate_generation', 0
            ) >= 2
        ), harness.diagnostics()
        approval = harness.call(harness.approve_client)
        assert approval.success, approval.message
        assert wait_until(
            lambda: harness.goal_count == 2
            and harness.latest_status().get('goal_accepted')
        ), harness.diagnostics()

        cancellation = harness.call(harness.cancel_client)
        assert cancellation.success, cancellation.message
        assert wait_until(
            lambda: harness.latest_status().get('state') == 'canceled'
        ), harness.diagnostics()
        assert harness.latest_status()['result_code'] == 7
    finally:
        harness.stop()


def test_backend_unavailable_times_out_without_motion():
    """Reject approval safely when the guarded action server is absent."""
    harness = RuntimeHarness(start_action_server=False)
    try:
        missing_plan = harness.call(harness.approve_client)
        assert not missing_plan.success
        assert missing_plan.message == 'coverage_handoff_no_valid_plan'

        harness.path_publisher.publish(
            make_path(harness.frame_id)
        )
        assert wait_until(
            lambda: harness.latest_status().get('candidate_valid')
        ), harness.diagnostics()
        approval = harness.call(harness.approve_client)
        assert approval.success, approval.message
        assert wait_until(
            lambda: harness.latest_status().get('state') == 'timed_out'
        ), harness.diagnostics()
        assert harness.goal_count == 0
        assert (
            harness.latest_status().get('reason')
            == 'savo_nav_coverage_action_unavailable'
        )
    finally:
        harness.stop()
