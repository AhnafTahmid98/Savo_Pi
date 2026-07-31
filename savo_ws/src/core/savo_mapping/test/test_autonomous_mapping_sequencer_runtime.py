# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Isolated runtime validation for the AM-5 autonomous mission prelude."""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from savo_msgs.action import RunAutonomousMapping
from savo_msgs.msg import AutonomousMappingStatus
from savo_msgs.srv import ControlAutonomousMapping
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def retained_qos():
    """Return reliable transient-local state QoS."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def command_qos():
    """Return reliable volatile command QoS."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def wait_until(predicate, timeout=10.0, interval=0.02):
    """Wait until a predicate succeeds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def installed_executable():
    """Return the installed production orchestrator executable."""
    path = Path(os.environ.get('AUTONOMOUS_ORCHESTRATOR_EXECUTABLE', ''))
    assert path.is_file() and os.access(path, os.X_OK), path
    return str(path)


class SequencerHarness:
    """Run the production AM-5 prelude against controlled ROS fixtures."""

    def __init__(self):
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        prefix = f'/test/am5_{suffix}'

        self.map_frame = f'map_{suffix}'
        self.base_frame = f'base_{suffix}'
        self.action_name = f'{prefix}/run'
        self.control_service = f'{prefix}/control'
        self.status_topic = f'{prefix}/status'
        self.mode_topic = f'{prefix}/mode'
        self.exploration_mode_topic = f'{prefix}/exploration_mode'
        self.workflow_phase_topic = f'{prefix}/workflow_phase'
        self.session_state_topic = f'{prefix}/session_state'
        self.readiness_topic = f'{prefix}/readiness'
        self.safety_stop_topic = f'{prefix}/safety_stop'
        self.runtime_authority_topic = f'{prefix}/runtime_authority'
        self.handoff_state_topic = f'{prefix}/handoff_state'
        self.frontier_status_topic = f'{prefix}/frontier_status'
        self.mode_command_topic = f'{prefix}/mode_cmd'
        self.start_session_topic = f'{prefix}/start_session_cmd'
        self.cancel_session_topic = f'{prefix}/cancel_session_cmd'
        self.handoff_cancel_service = f'{prefix}/handoff_cancel'
        self.map_save_service = f'{prefix}/map_save'
        self.scan_state_topic = f'{prefix}/scan360/state'
        self.scan_start_service = f'{prefix}/scan360/start'
        self.scan_cancel_service = f'{prefix}/scan360/cancel'
        self.head_state_topic = f'{prefix}/head/scan_state'
        self.head_start_service = f'{prefix}/head/start_scan'
        self.head_pause_service = f'{prefix}/head/pause_scan'
        self.head_resume_service = f'{prefix}/head/resume_scan'

        self.node = rclpy.create_node(f'am5_fixture_{suffix}')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.mode_commands = []
        self.start_session_commands = []
        self.cancel_session_commands = []
        self.statuses = []
        self.scan_start_count = 0
        self.scan_cancel_count = 0
        self.head_start_count = 0
        self.head_pause_count = 0
        self.head_resume_count = 0

        self.mode_pub = self.node.create_publisher(
            String, self.mode_topic, retained_qos()
        )
        self.exploration_mode_pub = self.node.create_publisher(
            String, self.exploration_mode_topic, retained_qos()
        )
        self.workflow_phase_pub = self.node.create_publisher(
            String, self.workflow_phase_topic, retained_qos()
        )
        self.session_state_pub = self.node.create_publisher(
            String, self.session_state_topic, retained_qos()
        )
        self.readiness_pub = self.node.create_publisher(
            String, self.readiness_topic, retained_qos()
        )
        self.safety_stop_pub = self.node.create_publisher(
            Bool, self.safety_stop_topic, command_qos()
        )
        self.runtime_authority_pub = self.node.create_publisher(
            Bool, self.runtime_authority_topic, retained_qos()
        )
        self.handoff_state_pub = self.node.create_publisher(
            String, self.handoff_state_topic, retained_qos()
        )
        self.scan_state_pub = self.node.create_publisher(
            String, self.scan_state_topic, retained_qos()
        )
        self.head_state_pub = self.node.create_publisher(
            String, self.head_state_topic, command_qos()
        )

        self.mode_command_sub = self.node.create_subscription(
            String,
            self.mode_command_topic,
            lambda msg: self.mode_commands.append(msg.data),
            command_qos(),
        )
        self.start_session_sub = self.node.create_subscription(
            String,
            self.start_session_topic,
            lambda msg: self.start_session_commands.append(msg.data),
            command_qos(),
        )
        self.cancel_session_sub = self.node.create_subscription(
            String,
            self.cancel_session_topic,
            lambda msg: self.cancel_session_commands.append(msg.data),
            command_qos(),
        )
        self.status_sub = self.node.create_subscription(
            AutonomousMappingStatus,
            self.status_topic,
            lambda msg: self.statuses.append(msg),
            retained_qos(),
        )

        self.handoff_cancel_server = self.node.create_service(
            Trigger,
            self.handoff_cancel_service,
            self.accept_trigger,
            callback_group=self.group,
        )
        self.map_save_server = self.node.create_service(
            Trigger,
            self.map_save_service,
            self.accept_trigger,
            callback_group=self.group,
        )
        self.scan_start_server = self.node.create_service(
            Trigger,
            self.scan_start_service,
            self.handle_scan_start,
            callback_group=self.group,
        )
        self.scan_cancel_server = self.node.create_service(
            Trigger,
            self.scan_cancel_service,
            self.handle_scan_cancel,
            callback_group=self.group,
        )
        self.head_start_server = self.node.create_service(
            Trigger,
            self.head_start_service,
            self.handle_head_start,
            callback_group=self.group,
        )
        self.head_pause_server = self.node.create_service(
            Trigger,
            self.head_pause_service,
            self.handle_head_pause,
            callback_group=self.group,
        )
        self.head_resume_server = self.node.create_service(
            Trigger,
            self.head_resume_service,
            self.handle_head_resume,
            callback_group=self.group,
        )

        self.action_client = ActionClient(
            self.node,
            RunAutonomousMapping,
            self.action_name,
            callback_group=self.group,
        )
        self.control_client = self.node.create_client(
            ControlAutonomousMapping,
            self.control_service,
            callback_group=self.group,
        )

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

        command = [
            installed_executable(),
            '--ros-args',
            '-p', f'action_name:={self.action_name}',
            '-p', f'control_service:={self.control_service}',
            '-p', f'status_topic:={self.status_topic}',
            '-p', f'mode_topic:={self.mode_topic}',
            '-p', f'exploration_mode_topic:={self.exploration_mode_topic}',
            '-p', f'workflow_phase_topic:={self.workflow_phase_topic}',
            '-p', f'session_state_topic:={self.session_state_topic}',
            '-p', f'readiness_topic:={self.readiness_topic}',
            '-p', f'safety_stop_topic:={self.safety_stop_topic}',
            '-p', f'runtime_authority_topic:={self.runtime_authority_topic}',
            '-p', f'handoff_state_topic:={self.handoff_state_topic}',
            '-p', f'frontier_status_topic:={self.frontier_status_topic}',
            '-p', f'mode_command_topic:={self.mode_command_topic}',
            '-p', f'start_session_command_topic:={self.start_session_topic}',
            '-p', f'cancel_session_command_topic:={self.cancel_session_topic}',
            '-p', f'handoff_cancel_service:={self.handoff_cancel_service}',
            '-p', f'save.map_session_service:={self.map_save_service}',
            '-p', f'sequence.scan360_state_topic:={self.scan_state_topic}',
            '-p', f'sequence.scan360_start_service:={self.scan_start_service}',
            '-p', f'sequence.scan360_cancel_service:={self.scan_cancel_service}',
            '-p', f'sequence.head_scan_state_topic:={self.head_state_topic}',
            '-p', f'sequence.head_scan_start_service:={self.head_start_service}',
            '-p', f'sequence.head_scan_pause_service:={self.head_pause_service}',
            '-p', f'sequence.head_scan_resume_service:={self.head_resume_service}',
            '-p', f'sequence.start_pose_target_frame:={self.map_frame}',
            '-p', f'sequence.start_pose_source_frame:={self.base_frame}',
            '-p', 'sequence.start_pose_lookup_timeout_s:=0.5',
            '-p', 'sequence.start_pose_stale_timeout_s:=1.0',
            '-p', 'sequence.start_pose_operation_timeout_s:=4.0',
            '-p', 'sequence.scan360_operation_timeout_s:=4.0',
            '-p', 'sequence.head_scan_operation_timeout_s:=4.0',
            '-p', 'evaluation_period_ms:=50',
            '-p', 'command_retry_period_ms:=100',
            '-p', 'default_mission_timeout_s:=0.0',
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

        assert wait_until(
            lambda: self.process.poll() is not None
            or (
                self.action_client.server_is_ready()
                and self.control_client.service_is_ready()
            )
        ), self.diagnostics()
        assert self.process.poll() is None, self.diagnostics()

    @staticmethod
    def string_message(value):
        """Create a String message."""
        message = String()
        message.data = value
        return message

    @staticmethod
    def bool_message(value):
        """Create a Bool message."""
        message = Bool()
        message.data = value
        return message

    def accept_trigger(self, _request, response):
        """Accept a generic fixture trigger."""
        response.success = True
        response.message = 'accepted'
        return response

    def delayed_publish(self, publisher, value, delay=0.10):
        """Publish a terminal component state after a service response."""
        timer = threading.Timer(
            delay,
            lambda: publisher.publish(self.string_message(value)),
        )
        timer.daemon = True
        timer.start()

    def handle_scan_start(self, _request, response):
        """Accept and complete one Scan360 operation."""
        self.scan_start_count += 1
        response.success = True
        response.message = 'scan360_started'
        self.delayed_publish(self.scan_state_pub, 'complete')
        return response

    def handle_scan_cancel(self, _request, response):
        """Accept one Scan360 cancellation."""
        self.scan_cancel_count += 1
        response.success = True
        response.message = 'scan360_cancel_requested'
        self.delayed_publish(self.scan_state_pub, 'canceled')
        return response

    def handle_head_start(self, _request, response):
        """Accept and complete one head scan."""
        self.head_start_count += 1
        response.success = True
        response.message = 'head_scan_started'
        self.delayed_publish(
            self.head_state_pub,
            'state=done;phase=complete;reason=fixture_complete',
        )
        return response

    def handle_head_pause(self, _request, response):
        """Accept one head-scan pause."""
        self.head_pause_count += 1
        response.success = True
        response.message = 'head_scan_paused'
        self.delayed_publish(
            self.head_state_pub,
            'state=paused;phase=paused;reason=fixture_pause',
        )
        return response

    def handle_head_resume(self, _request, response):
        """Accept one head-scan resume."""
        self.head_resume_count += 1
        response.success = True
        response.message = 'head_scan_resumed'
        self.delayed_publish(
            self.head_state_pub,
            'state=done;phase=complete;reason=fixture_resume_complete',
        )
        return response

    def publish_transform(self):
        """Publish one fresh map-to-base transform."""
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = 1.25
        transform.transform.translation.y = -0.50
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def publish_initial_state(self):
        """Publish a safe monitor-only idle state and fresh TF repeatedly."""
        for _ in range(10):
            self.publish_transform()
            self.mode_pub.publish(self.string_message('monitor_only'))
            self.exploration_mode_pub.publish(self.string_message('idle'))
            self.workflow_phase_pub.publish(self.string_message('idle'))
            self.session_state_pub.publish(self.string_message('idle'))
            self.readiness_pub.publish(self.string_message('ready'))
            self.safety_stop_pub.publish(self.bool_message(False))
            self.runtime_authority_pub.publish(self.bool_message(False))
            self.handoff_state_pub.publish(self.string_message('idle'))
            time.sleep(0.05)

    def publish_monitor_state(self):
        """Publish monitor-only state."""
        self.publish_transform()
        self.mode_pub.publish(self.string_message('monitor_only'))
        self.exploration_mode_pub.publish(self.string_message('idle'))
        self.workflow_phase_pub.publish(self.string_message('idle'))
        self.runtime_authority_pub.publish(self.bool_message(False))

    def publish_scan_state(self):
        """Publish the Scan360 workflow state."""
        self.publish_transform()
        self.mode_pub.publish(self.string_message('autonomous'))
        self.exploration_mode_pub.publish(self.string_message('scan360'))
        self.workflow_phase_pub.publish(self.string_message('scan360'))
        self.runtime_authority_pub.publish(self.bool_message(False))

    def publish_frontier_state(self):
        """Publish the active frontier workflow state."""
        self.publish_transform()
        self.mode_pub.publish(self.string_message('autonomous'))
        self.exploration_mode_pub.publish(self.string_message('frontier'))
        self.workflow_phase_pub.publish(self.string_message('exploring'))
        self.runtime_authority_pub.publish(self.bool_message(True))
        self.handoff_state_pub.publish(self.string_message('idle'))

    def send_goal(self):
        """Send one frontier mission goal."""
        assert self.action_client.wait_for_server(timeout_sec=5.0)
        goal = RunAutonomousMapping.Goal()
        goal.contract_version = RunAutonomousMapping.Goal.CONTRACT_VERSION
        goal.mission_id = 'mission_am5_runtime'
        goal.actor_id = 'operator_am5'
        goal.map_id = 'campus_main'
        goal.map_revision = 1
        goal.strategy = RunAutonomousMapping.Goal.STRATEGY_FRONTIER
        goal.auto_save = False
        goal.require_quality_approval = False
        goal.mission_timeout.sec = 0
        goal.mission_timeout.nanosec = 0
        future = self.action_client.send_goal_async(goal)
        assert wait_until(future.done), self.diagnostics()
        handle = future.result()
        assert handle.accepted, self.diagnostics()
        return handle

    def control(self, command, reason):
        """Send one typed mission-control request."""
        assert self.control_client.wait_for_service(timeout_sec=5.0)
        request = ControlAutonomousMapping.Request()
        request.contract_version = (
            ControlAutonomousMapping.Request.CONTRACT_VERSION
        )
        request.mission_id = 'mission_am5_runtime'
        request.actor_id = 'operator_am5'
        request.command = command
        request.reason = reason
        future = self.control_client.call_async(request)
        assert wait_until(future.done), self.diagnostics()
        return future.result()

    def latest_state(self):
        """Return the latest typed mission state."""
        if not self.statuses:
            return None
        return self.statuses[-1].state

    def diagnostics(self):
        """Return process and fixture diagnostics."""
        output = ''
        if self.process.poll() is not None and self.process.stdout:
            output = self.process.stdout.read()
        return (
            f'process={self.process.poll()} modes={self.mode_commands[-12:]} '
            f'starts={self.start_session_commands[-5:]} '
            f'cancels={self.cancel_session_commands[-5:]} '
            f'scan_starts={self.scan_start_count} '
            f'head_starts={self.head_start_count} '
            f'states={[status.state_text for status in self.statuses[-12:]]} '
            f'output={output}'
        )

    def close(self):
        """Stop the production process and fixture executor."""
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5.0)

        self.executor.shutdown(timeout_sec=2.0)
        self.action_client.destroy()
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)


@pytest.fixture(autouse=True, scope='module')
def ros_context():
    """Initialize one isolated ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_initial_sequence_and_conditional_scan_resume_frontier():
    """Validate the AM-5 prelude and one guarded conditional rescan."""
    harness = SequencerHarness()
    try:
        harness.publish_initial_state()
        goal_handle = harness.send_goal()
        result_future = goal_handle.get_result_async()

        assert wait_until(
            lambda: 'mission_am5_runtime' in harness.start_session_commands
        ), harness.diagnostics()

        for _ in range(5):
            harness.publish_transform()
            harness.session_state_pub.publish(
                harness.string_message('active')
            )
            time.sleep(0.05)

        assert wait_until(
            lambda: 'autonomous:scan360' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_scan_state()

        assert wait_until(
            lambda: harness.scan_start_count == 1
        ), harness.diagnostics()
        assert wait_until(
            lambda: 'monitor_only' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_monitor_state()

        assert wait_until(
            lambda: harness.head_start_count == 1
        ), harness.diagnostics()
        assert wait_until(
            lambda: 'autonomous:frontier' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_frontier_state()

        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), harness.diagnostics()

        status = harness.statuses[-1]
        assert status.start_pose_capture_complete
        assert status.start_pose_valid
        assert status.start_pose_map.header.frame_id == harness.map_frame
        assert status.initial_scan360_complete
        assert status.initial_scan360_succeeded
        assert status.initial_head_scan_complete
        assert status.initial_head_scan_succeeded

        previous_monitor_count = harness.mode_commands.count('monitor_only')
        previous_scan_mode_count = harness.mode_commands.count(
            'autonomous:scan360'
        )
        previous_frontier_count = harness.mode_commands.count(
            'autonomous:frontier'
        )

        response = harness.control(
            ControlAutonomousMapping.Request.COMMAND_REQUEST_SCAN360,
            'map_growth_stalled',
        )
        assert response.accepted

        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > previous_monitor_count
        ), harness.diagnostics()
        harness.publish_monitor_state()

        assert wait_until(
            lambda: harness.mode_commands.count('autonomous:scan360')
            > previous_scan_mode_count
        ), harness.diagnostics()
        harness.publish_scan_state()
        assert wait_until(
            lambda: harness.scan_start_count == 2
        ), harness.diagnostics()

        assert wait_until(
            lambda: harness.mode_commands.count('autonomous:frontier')
            > previous_frontier_count
        ), harness.diagnostics()
        harness.publish_frontier_state()

        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
            and harness.statuses[-1].conditional_scan360_completed == 1
        ), harness.diagnostics()

        previous_monitor_count = harness.mode_commands.count('monitor_only')
        response = harness.control(
            ControlAutonomousMapping.Request.COMMAND_CANCEL,
            'runtime_cleanup',
        )
        assert response.accepted

        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > previous_monitor_count
        ), harness.diagnostics()
        harness.publish_monitor_state()

        assert wait_until(
            lambda: 'mission_am5_runtime'
            in harness.cancel_session_commands
        ), harness.diagnostics()
        harness.session_state_pub.publish(
            harness.string_message('cancelled')
        )
        assert wait_until(result_future.done), harness.diagnostics()
        assert not result_future.result().result.success
    finally:
        harness.close()
