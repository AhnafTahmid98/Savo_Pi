# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Isolated runtime validation for the AM-3 mission orchestrator."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

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
from savo_msgs.msg import FrontierExplorationStatus
from savo_msgs.srv import ControlAutonomousMapping
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger


def retained_qos():
    """Return the retained reliable state QoS."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def command_qos():
    """Return the reliable volatile command QoS."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def wait_until(predicate, timeout=8.0, interval=0.02):
    """Wait until a predicate succeeds."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


def installed_executable():
    """Return the installed AM-3 orchestrator executable."""
    path = Path(os.environ.get('AUTONOMOUS_ORCHESTRATOR_EXECUTABLE', ''))
    assert path.is_file() and os.access(path, os.X_OK), path
    return str(path)


class RuntimeHarness:
    """Run the production orchestrator against controlled state fixtures."""

    def __init__(self):
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        prefix = f'/test/am3_{suffix}'

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

        self.node = rclpy.create_node(f'am3_fixture_{suffix}')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)

        self.mode_commands = []
        self.start_session_commands = []
        self.cancel_session_commands = []
        self.statuses = []
        self.handoff_cancel_count = 0
        self.map_save_count = 0
        self.save_success = True
        self.executor_error = None
        self.pending_goal_responses = set()
        self.temporary_root = tempfile.TemporaryDirectory(
            prefix='savo_am3_runtime_'
        )

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
        self.frontier_status_pub = self.node.create_publisher(
            FrontierExplorationStatus,
            self.frontier_status_topic,
            retained_qos(),
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
            self.handle_handoff_cancel,
            callback_group=self.group,
        )
        self.map_save_server = self.node.create_service(
            Trigger,
            self.map_save_service,
            self.handle_map_save,
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
            target=self._spin_executor,
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
            '-p', 'sequence.require_start_pose_capture:=false',
            '-p', 'sequence.require_initial_scan360:=false',
            '-p', 'sequence.require_initial_head_scan:=false',
            '-p', 'coverage.enabled:=false',
            '-p', 'coverage.required:=false',
            '-p', f'mode_command_topic:={self.mode_command_topic}',
            '-p', f'start_session_command_topic:={self.start_session_topic}',
            '-p', f'cancel_session_command_topic:={self.cancel_session_topic}',
            '-p', f'handoff_cancel_service:={self.handoff_cancel_service}',
            '-p', f'save.map_session_service:={self.map_save_service}',
            '-p', 'save.operation_timeout_s:=3.0',
            '-p', 'save.expected_frame:=map',
            '-p', 'evaluation_period_ms:=50',
            '-p', 'command_retry_period_ms:=100',
            '-p', 'default_mission_timeout_s:=0.0',
            '-p', 'completion.minimum_exhaustion_observations:=3',
            '-p', 'completion.minimum_stable_duration_s:=0.15',
            '-p', 'completion.frontier_status_timeout_s:=2.0',
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

    def _spin_executor(self):
        """Run the fixture executor while retaining unexpected exits."""
        try:
            self.executor.spin()
        except BaseException as error:
            self.executor_error = repr(error)
            raise

    def _assert_runtime_healthy(self, stage):
        """Fail with the owner of a broken action handshake."""
        if self.process.poll() is not None:
            pytest.fail(
                f'production_process_exited stage={stage} '
                f'{self.diagnostics()}'
            )
        if not self.spin_thread.is_alive():
            pytest.fail(
                f'fixture_executor_stopped stage={stage} '
                f'{self.diagnostics()}'
            )

    def _wait_for_action_server(self):
        """Perform an explicit action readiness handshake before each goal."""
        self._assert_runtime_healthy('before_action_server_handshake')
        ready = self.action_client.wait_for_server(timeout_sec=3.0)
        self._assert_runtime_healthy('after_action_server_handshake')
        if not ready or not self.action_client.server_is_ready():
            pytest.fail(f'action_server_unavailable {self.diagnostics()}')

    def handle_handoff_cancel(self, _request, response):
        """Accept guarded exploration-goal cancellation."""
        self.handoff_cancel_count += 1
        response.success = True
        response.message = 'handoff_cancel_accepted'
        return response

    def create_valid_session(self, map_id):
        """Create one saved-map session accepted by the production verifier."""
        session = Path(self.temporary_root.name) / map_id
        session.mkdir(parents=True, exist_ok=True)
        base = session / map_id
        (base.with_suffix('.pgm')).write_bytes(b'P5\n1 1\n255\nx')
        (base.with_suffix('.posegraph')).write_text(
            'posegraph', encoding='utf-8'
        )
        (base.with_suffix('.data')).write_text('data', encoding='utf-8')
        (base.with_suffix('.yaml')).write_text(
            f'image: {map_id}.pgm\n'
            'mode: trinary\n'
            'resolution: 0.05\n'
            'origin: [0.0, 0.0, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n',
            encoding='utf-8',
        )
        (session / 'manifest.yaml').write_text(
            'schema_version: 1\n'
            f'map_id: "{map_id}"\n'
            'frame_id: "map"\n'
            f'session_directory: "{session}"\n'
            'occupancy_grid:\n'
            f'  yaml: "{base}.yaml"\n'
            f'  image: "{base}.pgm"\n'
            'pose_graph:\n'
            f'  posegraph: "{base}.posegraph"\n'
            f'  data: "{base}.data"\n'
            'map_quality:\n'
            '  structurally_valid: true\n'
            '  evaluated: false\n'
            'navigation_handoff_ready: false\n',
            encoding='utf-8',
        )
        return session

    def handle_map_save(self, _request, response):
        """Emulate the public map-session save boundary."""
        self.map_save_count += 1
        if not self.save_success:
            response.success = False
            response.message = 'simulated_map_save_failure'
            return response

        session = self.create_valid_session('campus_main')
        response.success = True
        response.message = f'map_session_saved:{session}'
        return response

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

    @staticmethod
    def frontier_message(sequence, planning_status):
        """Create one typed frontier-planning observation."""
        message = FrontierExplorationStatus()
        message.contract_version = FrontierExplorationStatus.CONTRACT_VERSION
        message.state = planning_status
        message.reason = planning_status
        message.configured_enabled = True
        message.runtime_authority_received = True
        message.runtime_enabled = True
        message.enabled = True
        message.map_received = True
        message.map_generation = 10
        message.planned_map_generation = 10
        message.plan_sequence = sequence
        message.planning_status = planning_status
        message.planning_reason = planning_status
        message.handoff_state = 'idle'
        message.goal_pending = False
        if planning_status == 'no_frontiers':
            message.exhaustion_kind = (
                FrontierExplorationStatus.EXHAUSTION_NO_FRONTIERS
            )
            message.detected_frontiers = 0
            message.reachable_frontiers = 0
        else:
            message.exhaustion_kind = (
                FrontierExplorationStatus.EXHAUSTION_NONE
            )
            message.detected_frontiers = 1
            message.reachable_frontiers = 1
        return message

    def publish_initial_state(self):
        """Publish a safe monitor-only idle mapping state."""
        for _ in range(3):
            self.mode_pub.publish(self.string_message('monitor_only'))
            self.exploration_mode_pub.publish(self.string_message('idle'))
            self.workflow_phase_pub.publish(self.string_message('idle'))
            self.session_state_pub.publish(self.string_message('idle'))
            self.readiness_pub.publish(self.string_message('ready'))
            self.safety_stop_pub.publish(self.bool_message(False))
            self.runtime_authority_pub.publish(self.bool_message(False))
            self.handoff_state_pub.publish(self.string_message('idle'))
            time.sleep(0.05)

    def publish_exploring_state(self):
        """Publish the authorized frontier workflow state."""
        self.session_state_pub.publish(self.string_message('active'))
        self.mode_pub.publish(self.string_message('autonomous'))
        self.exploration_mode_pub.publish(self.string_message('frontier'))
        self.workflow_phase_pub.publish(self.string_message('exploring'))
        self.runtime_authority_pub.publish(self.bool_message(True))
        self.handoff_state_pub.publish(self.string_message('idle'))

    def publish_monitor_state(self):
        """Publish the safe monitor-only state after goal cancellation."""
        self.mode_pub.publish(self.string_message('monitor_only'))
        self.exploration_mode_pub.publish(self.string_message('idle'))
        self.workflow_phase_pub.publish(self.string_message('idle'))
        self.runtime_authority_pub.publish(self.bool_message(False))

    def send_goal(self, auto_save=True):
        """Start one typed frontier mission."""
        goal = RunAutonomousMapping.Goal()
        goal.contract_version = RunAutonomousMapping.Goal.CONTRACT_VERSION
        goal.mission_id = 'mission-am3-runtime'
        goal.actor_id = 'runtime-operator'
        goal.map_id = 'campus_main'
        goal.map_revision = 1
        goal.strategy = RunAutonomousMapping.Goal.STRATEGY_FRONTIER
        goal.auto_save = auto_save
        goal.require_quality_approval = True

        self._wait_for_action_server()
        future = self.action_client.send_goal_async(goal)
        self.pending_goal_responses.add(future)
        try:
            completed = wait_until(
                lambda: (
                    future.done()
                    or self.process.poll() is not None
                    or not self.spin_thread.is_alive()
                )
            )
            if self.process.poll() is not None:
                future.cancel()
                pytest.fail(
                    'production_process_exited while_waiting_for_goal_response '
                    f'{self.diagnostics()}'
                )
            if not self.spin_thread.is_alive():
                future.cancel()
                pytest.fail(
                    'fixture_executor_stopped while_waiting_for_goal_response '
                    f'{self.diagnostics()}'
                )
            if not completed or not future.done():
                server_ready_now = self.action_client.server_is_ready()
                future.cancel()
                pytest.fail(
                    'action_server_ready_but_no_goal_response '
                    f'server_ready_at_submit=True '
                    f'server_ready_now={server_ready_now} '
                    f'{self.diagnostics()}'
                )
            error = future.exception()
            if error is not None:
                pytest.fail(
                    f'action_goal_response_error={error!r} '
                    f'{self.diagnostics()}'
                )
            handle = future.result()
            if handle is None or not handle.accepted:
                pytest.fail(f'action_goal_rejected {self.diagnostics()}')
            return handle
        finally:
            self.pending_goal_responses.discard(future)

    def control(self, command, reason):
        """Call the typed mission control service."""
        request = ControlAutonomousMapping.Request()
        request.contract_version = (
            ControlAutonomousMapping.Request.CONTRACT_VERSION
        )
        request.mission_id = 'mission-am3-runtime'
        request.actor_id = 'runtime-operator'
        request.command = command
        request.reason = reason

        future = self.control_client.call_async(request)
        assert wait_until(future.done), self.diagnostics()
        return future.result()

    def latest_state(self):
        """Return the latest typed mission state."""
        return self.statuses[-1].state if self.statuses else None

    def diagnostics(self):
        """Return process and observed state diagnostics."""
        output = ''
        if self.process.poll() is not None and self.process.stdout:
            output = self.process.stdout.read()
        return (
            f'poll={self.process.poll()} modes={self.mode_commands} '
            f'action_server_ready={self.action_client.server_is_ready()} '
            f'executor_alive={self.spin_thread.is_alive()} '
            f'executor_error={self.executor_error} '
            f'pending_goal_responses={len(self.pending_goal_responses)} '
            f'starts={self.start_session_commands} '
            f'cancels={self.cancel_session_commands} '
            f'handoff_cancels={self.handoff_cancel_count} '
            f'map_saves={self.map_save_count} '
            f'states={[msg.state_text for msg in self.statuses[-8:]]} '
            f'completion={[(
                msg.completion_candidate,
                msg.completion_confirmed,
            ) for msg in self.statuses[-8:]]} '
            f'output={output}'
        )

    def close(self):
        """Stop the production process and fixture executor."""
        for future in tuple(self.pending_goal_responses):
            future.cancel()
        self.pending_goal_responses.clear()

        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5.0)

        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.action_client.destroy()
        self.node.destroy_node()
        self.temporary_root.cleanup()


@pytest.fixture(autouse=True, scope='module')
def ros_context():
    """Initialize one isolated ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_start_pause_resume_and_cancel_lifecycle():
    """Validate the complete AM-1 control lifecycle remains intact."""
    harness = RuntimeHarness()
    try:
        harness.publish_initial_state()
        goal_handle = harness.send_goal()
        result_future = goal_handle.get_result_async()

        assert wait_until(
            lambda: 'mission-am3-runtime' in harness.start_session_commands
        ), harness.diagnostics()

        harness.session_state_pub.publish(
            harness.string_message('active')
        )
        assert wait_until(
            lambda: 'autonomous:frontier' in harness.mode_commands
        ), harness.diagnostics()

        harness.publish_exploring_state()
        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), harness.diagnostics()

        harness.handoff_state_pub.publish(
            harness.string_message('executing')
        )
        response = harness.control(
            ControlAutonomousMapping.Request.COMMAND_PAUSE,
            'runtime_pause',
        )
        assert response.accepted
        assert wait_until(
            lambda: harness.handoff_cancel_count >= 1
        ), harness.diagnostics()

        harness.handoff_state_pub.publish(
            harness.string_message('canceled')
        )
        previous_monitor_count = harness.mode_commands.count('monitor_only')
        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > previous_monitor_count
        ), harness.diagnostics()

        harness.publish_monitor_state()
        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_PAUSED
        ), harness.diagnostics()

        response = harness.control(
            ControlAutonomousMapping.Request.COMMAND_RESUME,
            'runtime_resume',
        )
        assert response.accepted
        previous_frontier_count = harness.mode_commands.count(
            'autonomous:frontier'
        )
        assert wait_until(
            lambda: harness.mode_commands.count('autonomous:frontier')
            > previous_frontier_count
        ), harness.diagnostics()

        harness.publish_exploring_state()
        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), harness.diagnostics()

        response = harness.control(
            ControlAutonomousMapping.Request.COMMAND_CANCEL,
            'runtime_cancel',
        )
        assert response.accepted

        previous_monitor_count = harness.mode_commands.count('monitor_only')
        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > previous_monitor_count
        ), harness.diagnostics()

        harness.publish_monitor_state()
        assert wait_until(
            lambda: 'mission-am3-runtime'
            in harness.cancel_session_commands
        ), harness.diagnostics()

        harness.session_state_pub.publish(
            harness.string_message('cancelled')
        )
        assert wait_until(result_future.done), harness.diagnostics()

        wrapped_result = result_future.result()
        assert not wrapped_result.result.success
        assert wrapped_result.result.result_code == (
            RunAutonomousMapping.Result.RESULT_CANCELED
        )
        assert wrapped_result.result.final_status.state == (
            AutonomousMappingStatus.STATE_CANCELED
        )
    finally:
        harness.close()


def test_stable_frontier_exhaustion_completes_manual_save_mission():
    """Confirm AM-2 completion only after stable typed observations."""
    harness = RuntimeHarness()
    try:
        harness.publish_initial_state()
        goal_handle = harness.send_goal(auto_save=False)
        result_future = goal_handle.get_result_async()

        assert wait_until(
            lambda: 'mission-am3-runtime' in harness.start_session_commands
        ), harness.diagnostics()

        harness.publish_exploring_state()
        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), harness.diagnostics()

        harness.frontier_status_pub.publish(
            harness.frontier_message(1, 'no_frontiers')
        )
        time.sleep(0.10)
        harness.frontier_status_pub.publish(
            harness.frontier_message(2, 'no_frontiers')
        )
        time.sleep(0.10)
        harness.frontier_status_pub.publish(
            harness.frontier_message(3, 'no_frontiers')
        )

        assert wait_until(
            lambda: any(
                status.state
                == AutonomousMappingStatus.STATE_COMPLETION_PENDING
                and status.completion_confirmed
                for status in harness.statuses
            )
        ), harness.diagnostics()

        previous_monitor_count = harness.mode_commands.count('monitor_only')
        assert wait_until(
            lambda: harness.mode_commands.count('monitor_only')
            > previous_monitor_count
        ), harness.diagnostics()

        harness.publish_monitor_state()
        assert wait_until(result_future.done), harness.diagnostics()

        wrapped_result = result_future.result()
        assert wrapped_result.result.success
        assert wrapped_result.result.result_code == (
            RunAutonomousMapping.Result.RESULT_SUCCEEDED
        )
        assert wrapped_result.result.final_status.state == (
            AutonomousMappingStatus.STATE_COMPLETED
        )
        assert not wrapped_result.result.map_saved
    finally:
        harness.close()


def test_auto_save_rejects_a_map_that_fails_the_requested_quality_gate():
    """Confirm the requested quality gate remains fail closed before AM-8."""
    harness = RuntimeHarness()
    try:
        harness.publish_initial_state()
        goal_handle = harness.send_goal(auto_save=True)
        result_future = goal_handle.get_result_async()

        assert wait_until(
            lambda: 'mission-am3-runtime' in harness.start_session_commands
        ), harness.diagnostics()
        harness.publish_exploring_state()
        assert wait_until(
            lambda: harness.latest_state()
            == AutonomousMappingStatus.STATE_EXPLORING
        ), harness.diagnostics()

        for sequence in (1, 2, 3):
            harness.frontier_status_pub.publish(
                harness.frontier_message(sequence, 'no_frontiers')
            )
            time.sleep(0.10)

        assert wait_until(
            lambda: 'monitor_only' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_monitor_state()

        assert wait_until(
            lambda: harness.map_save_count == 1
        ), harness.diagnostics()
        assert wait_until(result_future.done), harness.diagnostics()

        wrapped_result = result_future.result().result
        assert not wrapped_result.success
        assert wrapped_result.result_code == (
            RunAutonomousMapping.Result.RESULT_QUALITY_REJECTED
        )
        assert wrapped_result.map_saved
        assert wrapped_result.final_status.map_saved
        assert not wrapped_result.final_status.map_verified
        assert wrapped_result.final_status.verification_reason.startswith(
            'quality_rejected:'
        )
        assert any(
            status.state == AutonomousMappingStatus.STATE_SAVING
            for status in harness.statuses
        )
        assert any(
            status.state == AutonomousMappingStatus.STATE_VERIFYING
            for status in harness.statuses
        )
    finally:
        harness.close()


def test_auto_save_failure_returns_typed_save_failed_result():
    """Confirm a public save-service failure cannot report mission success."""
    harness = RuntimeHarness()
    harness.save_success = False
    try:
        harness.publish_initial_state()
        goal_handle = harness.send_goal(auto_save=True)
        result_future = goal_handle.get_result_async()
        assert wait_until(
            lambda: 'mission-am3-runtime' in harness.start_session_commands
        ), harness.diagnostics()
        harness.publish_exploring_state()

        for sequence in (1, 2, 3):
            harness.frontier_status_pub.publish(
                harness.frontier_message(sequence, 'no_frontiers')
            )
            time.sleep(0.10)

        assert wait_until(
            lambda: 'monitor_only' in harness.mode_commands
        ), harness.diagnostics()
        harness.publish_monitor_state()
        assert wait_until(result_future.done), harness.diagnostics()

        wrapped_result = result_future.result().result
        assert not wrapped_result.success
        assert wrapped_result.result_code == (
            RunAutonomousMapping.Result.RESULT_SAVE_FAILED
        )
        assert not wrapped_result.map_saved
        assert wrapped_result.final_status.state == (
            AutonomousMappingStatus.STATE_FAILED
        )
    finally:
        harness.close()
