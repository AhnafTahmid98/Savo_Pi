"""Isolated Phase 4L-B3G runtime validation."""

import json
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def state_qos():
    """Return the retained state QoS used by production nodes."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def status_qos():
    """Return the volatile reliable status QoS."""
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
    """Return the installed orchestrator executable."""
    path = os.environ.get('COVERAGE_ORCHESTRATOR_EXECUTABLE', '')
    assert path and os.path.isfile(path), path
    return path


class RuntimeHarness:
    """Run the orchestrator against fake supervisor and B3F services."""

    def __init__(self):
        suffix = f'{os.getpid()}_{time.monotonic_ns()}'
        self.supervisor_topic = f'/test/supervisor_{suffix}'
        self.handoff_state_topic = f'/test/handoff_state_{suffix}'
        self.handoff_status_topic = f'/test/handoff_status_{suffix}'
        self.handoff_feedback_topic = f'/test/handoff_feedback_{suffix}'
        self.operation_state_topic = f'/test/operation_state_{suffix}'
        self.operation_status_topic = f'/test/operation_status_{suffix}'
        self.operation_events_topic = f'/test/operation_events_{suffix}'
        self.public_approve = f'/test/operation_approve_{suffix}'
        self.public_cancel = f'/test/operation_cancel_{suffix}'
        self.public_reset = f'/test/operation_reset_{suffix}'
        self.internal_approve = f'/test/internal_approve_{suffix}'
        self.internal_cancel = f'/test/internal_cancel_{suffix}'
        self.internal_reset = f'/test/internal_reset_{suffix}'

        self.node = rclpy.create_node(f'b3g_fixture_{suffix}')
        self.group = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)

        self.approve_count = 0
        self.cancel_count = 0
        self.reset_count = 0
        self.states = []
        self.statuses = []

        self.supervisor_pub = self.node.create_publisher(
            String, self.supervisor_topic, state_qos()
        )
        self.handoff_state_pub = self.node.create_publisher(
            String, self.handoff_state_topic, state_qos()
        )
        self.handoff_status_pub = self.node.create_publisher(
            String, self.handoff_status_topic, state_qos()
        )

        self.internal_approve_server = self.node.create_service(
            Trigger,
            self.internal_approve,
            self.handle_internal_approve,
            callback_group=self.group,
        )
        self.internal_cancel_server = self.node.create_service(
            Trigger,
            self.internal_cancel,
            self.handle_internal_cancel,
            callback_group=self.group,
        )
        self.internal_reset_server = self.node.create_service(
            Trigger,
            self.internal_reset,
            self.handle_internal_reset,
            callback_group=self.group,
        )

        self.state_subscription = self.node.create_subscription(
            String,
            self.operation_state_topic,
            lambda msg: self.states.append(msg.data),
            state_qos(),
        )
        self.status_subscription = self.node.create_subscription(
            String,
            self.operation_status_topic,
            lambda msg: self.statuses.append(msg.data),
            status_qos(),
        )

        self.approve_client = self.node.create_client(
            Trigger, self.public_approve, callback_group=self.group
        )
        self.cancel_client = self.node.create_client(
            Trigger, self.public_cancel, callback_group=self.group
        )

        self.spin_thread = threading.Thread(
            target=self.executor.spin, daemon=True
        )
        self.spin_thread.start()

        command = [
            installed_executable(),
            '--ros-args',
            '-p', f'supervisor_state_topic:={self.supervisor_topic}',
            '-p', f'handoff_state_topic:={self.handoff_state_topic}',
            '-p', f'handoff_status_topic:={self.handoff_status_topic}',
            '-p', f'handoff_feedback_topic:={self.handoff_feedback_topic}',
            '-p', f'state_topic:={self.operation_state_topic}',
            '-p', f'status_topic:={self.operation_status_topic}',
            '-p', f'events_topic:={self.operation_events_topic}',
            '-p', f'approve_service:={self.public_approve}',
            '-p', f'cancel_service:={self.public_cancel}',
            '-p', f'reset_service:={self.public_reset}',
            '-p', f'internal_approve_service:={self.internal_approve}',
            '-p', f'internal_cancel_service:={self.internal_cancel}',
            '-p', f'internal_reset_service:={self.internal_reset}',
            '-p', 'supervisor_timeout_sec:=2.0',
            '-p', 'maximum_candidate_age_sec:=20.0',
            '-p', 'internal_service_timeout_sec:=1.0',
            '-p', 'supervisor_loss_cancel_delay_sec:=0.1',
            '-p', 'watchdog_period_ms:=20',
            '-p', 'publish_period_ms:=100',
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
            or self.approve_client.service_is_ready()
        ), self.diagnostics()
        assert self.process.poll() is None, self.diagnostics()

    def handle_internal_approve(self, _request, response):
        self.approve_count += 1
        response.success = True
        response.message = 'coverage-7-123456789'
        return response

    def handle_internal_cancel(self, _request, response):
        self.cancel_count += 1
        response.success = True
        response.message = 'coverage_handoff_cancel_requested'
        return response

    def handle_internal_reset(self, _request, response):
        self.reset_count += 1
        response.success = True
        response.message = 'coverage_handoff_reset'
        return response

    def publish_supervisor(self, *, ready=True, health='OK'):
        message = String()
        message.data = json.dumps(
            {
                'schema_version': 1,
                'node': 'savo_supervisor',
                'lifecycle': 'RUNNING' if ready else 'FAULTED',
                'operating_mode': 'STOP',
                'health': health,
                'safety': 'UNKNOWN',
                'ready': ready,
                'degraded': health == 'DEGRADED',
                'reason_code': (
                    'supervisor_operational'
                    if ready
                    else 'required_component_unavailable'
                ),
            }
        )
        self.supervisor_pub.publish(message)

    def publish_handoff(self, state='plan_available'):
        state_message = String()
        state_message.data = state
        self.handoff_state_pub.publish(state_message)

        status_message = String()
        status_message.data = json.dumps(
            {
                'enabled': True,
                'state': state,
                'reason': 'idle',
                'candidate_valid': True,
                'candidate_generation': 7,
                'candidate_age_sec': 0.0,
                'mission_id': (
                    'coverage-7-123456789'
                    if state != 'plan_available'
                    else ''
                ),
                'terminal_state': '',
                'result_reason': '',
            }
        )
        self.handoff_status_pub.publish(status_message)

    def call(self, client):
        assert client.wait_for_service(timeout_sec=2.0)
        future = client.call_async(Trigger.Request())
        assert wait_until(future.done, timeout=4.0), self.diagnostics()
        return future.result()

    def diagnostics(self):
        output = ''
        if self.process.poll() is not None and self.process.stdout:
            output = self.process.stdout.read()
        return (
            f'poll={self.process.poll()} approves={self.approve_count} '
            f'cancels={self.cancel_count} states={self.states} '
            f'statuses={self.statuses[-2:]} output={output}'
        )

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5.0)
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)


@pytest.fixture(autouse=True, scope='module')
def ros_context():
    """Initialize one isolated ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_two_key_approval_and_supervisor_loss_cancel():
    """Require operator call plus supervisor authorization."""
    harness = RuntimeHarness()
    try:
        harness.publish_supervisor(ready=True)
        harness.publish_handoff('plan_available')
        assert wait_until(
            lambda: 'ready_for_approval' in harness.states
        ), harness.diagnostics()
        time.sleep(0.2)
        assert harness.approve_count == 0

        response = harness.call(harness.approve_client)
        assert response.success, response.message
        assert response.message == 'coverage-7-123456789'
        assert harness.approve_count == 1

        harness.publish_handoff('executing')
        harness.publish_supervisor(ready=False, health='ERROR')
        assert wait_until(
            lambda: harness.cancel_count == 1,
            timeout=4.0,
        ), harness.diagnostics()
    finally:
        harness.close()


def test_unready_supervisor_blocks_operator_approval():
    """A public approval call cannot bypass the supervisor gate."""
    harness = RuntimeHarness()
    try:
        harness.publish_supervisor(ready=False, health='ERROR')
        harness.publish_handoff('plan_available')
        assert wait_until(
            lambda: 'blocked_by_supervisor' in harness.states
        ), harness.diagnostics()

        response = harness.call(harness.approve_client)
        assert not response.success
        assert 'supervisor' in response.message
        assert harness.approve_count == 0
    finally:
        harness.close()
