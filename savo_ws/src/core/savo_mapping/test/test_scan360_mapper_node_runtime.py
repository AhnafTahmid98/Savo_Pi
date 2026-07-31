import json
import math
import os
import signal
import subprocess
import threading
import time

from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from savo_msgs.action import RotateToHeading
from std_msgs.msg import String
from std_srvs.srv import Trigger


TIMEOUT = 8.0
COUNTER = 0


def unique_suffix():
    global COUNTER
    COUNTER += 1
    return f'{os.getpid()}_{COUNTER}'


def wait_until(predicate, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def installed_mapper_executable():
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        candidate = os.path.join(
            prefix,
            'lib',
            'savo_mapping',
            'scan360_mapper_node',
        )
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    raise AssertionError('installed scan360_mapper_node was not found')


class FixtureServer:

    def __init__(self, node, endpoint, mode='success', cancel_delay=0.0):
        self.mode = mode
        self.cancel_delay = cancel_delay
        self.goal_count = 0
        self.cancel_count = 0
        self.goal_times = []
        self.targets = []
        self.active = threading.Event()
        self.cancel_seen = threading.Event()
        self.server = ActionServer(
            node,
            RotateToHeading,
            endpoint,
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal(self, request):
        self.goal_count += 1
        self.goal_times.append(time.monotonic())
        self.targets.append(request.target_yaw_rad)
        if self.mode == 'reject':
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        self.cancel_count += 1
        self.cancel_seen.set()
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        self.active.set()
        result = RotateToHeading.Result()

        if self.mode == 'abort':
            result.success = False
            result.final_yaw_rad = goal_handle.request.target_yaw_rad
            result.final_error_rad = 0.5
            result.reason = 'fixture_abort'
            goal_handle.abort()
            return result

        if self.mode == 'hold':
            deadline = time.monotonic() + TIMEOUT
            while (
                not goal_handle.is_cancel_requested
                and time.monotonic() < deadline
            ):
                time.sleep(0.01)

            if goal_handle.is_cancel_requested:
                time.sleep(self.cancel_delay)
                result.success = False
                result.final_yaw_rad = goal_handle.request.target_yaw_rad
                result.final_error_rad = 0.1
                result.reason = 'fixture_canceled'
                goal_handle.canceled()
                return result

            result.success = False
            result.reason = 'fixture_hold_timeout'
            goal_handle.abort()
            return result

        result.success = True
        result.final_yaw_rad = goal_handle.request.target_yaw_rad
        result.final_error_rad = 0.0
        result.reason = 'goal_reached'
        goal_handle.succeed()
        return result

    def destroy(self):
        self.server.destroy()


class RuntimeHarness:

    def __init__(
        self,
        *,
        auto_start=True,
        enabled=True,
        mode='success',
        settle=0.05,
        tick=0.02,
        yaw_timeout=0.5,
        create_server=True,
        cancel_delay=0.0,
        extra_parameters=None,
    ):
        suffix = unique_suffix()
        self.endpoint = f'/fixture/scan360_mapper_{suffix}'
        self.odom_topic = f'/fixture/scan360_odom_{suffix}'
        self.status_topic = f'/fixture/scan360_status_{suffix}'
        self.start_service = f'/fixture/scan360_start_{suffix}'
        self.cancel_service = f'/fixture/scan360_cancel_{suffix}'
        self.node = rclpy.create_node(f'scan360_mapper_fixture_{suffix}')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.server = None

        if create_server:
            self.server = FixtureServer(
                self.node,
                self.endpoint,
                mode,
                cancel_delay,
            )

        self.statuses = []
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_subscription = self.node.create_subscription(
            String,
            self.status_topic,
            self._status,
            status_qos,
        )
        self.odom_publisher = self.node.create_publisher(
            Odometry,
            self.odom_topic,
            10,
        )
        self.start_client = self.node.create_client(
            Trigger,
            self.start_service,
            callback_group=ReentrantCallbackGroup(),
        )
        self.cancel_client = self.node.create_client(
            Trigger,
            self.cancel_service,
            callback_group=ReentrantCallbackGroup(),
        )
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

        parameters = {
            'enabled': str(enabled).lower(),
            'auto_start': str(auto_start).lower(),
            'action_name': self.endpoint,
            'odom_topic': self.odom_topic,
            'odom_frame': 'odom',
            'status_topic': self.status_topic,
            'state_topic': f'/fixture/scan360_state_{suffix}',
            'start_service': self.start_service,
            'cancel_service': self.cancel_service,
            'sweep_angle_rad': str(math.pi),
            'step_angle_rad': str(math.pi / 2.0),
            'settle_duration_sec': str(settle),
            'rotation_max_duration_sec': '2.0',
            'tick_period_sec': str(tick),
            'yaw_stale_timeout_sec': str(yaw_timeout),
            'server_wait_timeout_sec': '1.0',
            'goal_response_timeout_sec': '1.0',
            'feedback_stale_timeout_sec': '4.0',
            'cancel_timeout_sec': '1.0',
            'execution_grace_timeout_sec': '0.2',
        }
        parameters.update(extra_parameters or {})

        command = [
            installed_mapper_executable(),
            '--ros-args',
        ]
        for name, value in parameters.items():
            command.extend(('-p', f'{name}:={value}'))

        environment = os.environ.copy()
        environment['ROS_LOCALHOST_ONLY'] = '1'
        environment['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '217')
        environment['PYTHONDONTWRITEBYTECODE'] = '1'

        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        assert wait_until(
            lambda: self.process.poll() is not None or bool(self.statuses)
        ), self.output()
        assert self.process.poll() is None, self.output()

    def _status(self, message):
        try:
            self.statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    def latest_status(self):
        return self.statuses[-1] if self.statuses else {}

    def wait_status(self, predicate, timeout=TIMEOUT):
        assert wait_until(
            lambda: bool(self.statuses) and predicate(self.latest_status()),
            timeout,
        ), self.output()
        return self.latest_status()

    def publish_odom(self, frame='odom', quaternion=None, repeat=8):
        quaternion = quaternion or (0.0, 0.0, 0.0, 1.0)
        message = Odometry()
        message.header.frame_id = frame
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = quaternion
        for _ in range(repeat):
            self.odom_publisher.publish(message)
            time.sleep(0.02)

    def call_trigger(self, client):
        assert client.wait_for_service(timeout_sec=5.0), self.output()
        future = client.call_async(Trigger.Request())
        assert wait_until(future.done), self.output()
        return future.result()

    def output(self):
        if self.process.poll() is None or self.process.stdout is None:
            return ''
        return self.process.stdout.read()

    def stop(self, expect_success=True):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
                pytest.fail('scan360 mapper subprocess did not stop')

        output = self.output()
        if expect_success:
            assert self.process.returncode == 0, output

        if self.server is not None:
            self.server.destroy()
            self.server = None
        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        assert not self.spin_thread.is_alive()
        return output


@pytest.fixture(scope='module', autouse=True)
def ros_context(tmp_path_factory):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID')
    previous_log_dir = os.environ.get('ROS_LOG_DIR')
    os.environ['ROS_LOG_DIR'] = str(
        tmp_path_factory.mktemp('scan360_mapper_ros_logs')
    )
    rclpy.init()
    yield
    rclpy.shutdown()
    if previous_log_dir is None:
        os.environ.pop('ROS_LOG_DIR', None)
    else:
        os.environ['ROS_LOG_DIR'] = previous_log_dir


def test_safe_start_modes_and_read_only_graph():
    for auto_start, enabled in ((False, True), (True, False)):
        harness = RuntimeHarness(
            auto_start=auto_start,
            enabled=enabled,
        )
        try:
            harness.publish_odom()
            time.sleep(0.4)
            assert harness.server.goal_count == 0
            graph_topics = {
                name for name, _types in harness.node.get_topic_names_and_types()
            }
            velocity_root = 'cmd' + '_vel'
            assert '/' + velocity_root not in graph_topics
            assert '/' + velocity_root + '_auto' not in graph_topics
        finally:
            harness.stop()


def test_odom_validation_and_freshness_gate():
    harness = RuntimeHarness(
        tick=0.5,
        yaw_timeout=0.05,
    )
    try:
        time.sleep(0.55)
        assert harness.server.goal_count == 0

        harness.publish_odom(frame='map')
        harness.wait_status(
            lambda status: status.get('reason') == 'odom_frame_mismatch'
        )
        assert harness.server.goal_count == 0

        harness.publish_odom(
            quaternion=(float('nan'), 0.0, 0.0, 1.0)
        )
        harness.wait_status(
            lambda status: status.get('reason')
            == 'odom_quaternion_not_finite'
        )
        assert harness.server.goal_count == 0

        harness.publish_odom(quaternion=(0.0, 0.0, 0.0, 0.0))
        harness.wait_status(
            lambda status: status.get('reason')
            == 'odom_quaternion_norm_too_small'
        )
        assert harness.server.goal_count == 0

        harness.publish_odom(repeat=1)
        harness.wait_status(lambda status: status.get('odom_valid') is True)
        time.sleep(0.35)
        assert harness.server.goal_count == 0

        deadline = time.monotonic() + TIMEOUT
        while harness.server.goal_count == 0 and time.monotonic() < deadline:
            harness.publish_odom(repeat=1)
        assert harness.server.goal_count == 1
        assert harness.endpoint.startswith('/fixture/')
    finally:
        harness.stop()


def test_multiple_targets_settle_and_complete_once():
    settle = 0.16
    harness = RuntimeHarness(settle=settle)
    try:
        harness.publish_odom()
        status = harness.wait_status(
            lambda value: value.get('state') == 'complete'
        )
        assert status['terminal'] is True
        assert status['target_count'] == 2
        assert harness.server.goal_count == 2
        assert len(harness.server.goal_times) == 2
        assert (
            harness.server.goal_times[1]
            - harness.server.goal_times[0]
        ) >= settle * 0.8
        count = harness.server.goal_count
        time.sleep(0.35)
        assert harness.server.goal_count == count
    finally:
        harness.stop()


@pytest.mark.parametrize('mode', ('reject', 'abort'))
def test_terminal_action_failures_send_no_followup(mode):
    harness = RuntimeHarness(mode=mode)
    try:
        harness.publish_odom()
        status = harness.wait_status(
            lambda value: value.get('state') == 'failed'
        )
        assert status['terminal'] is True
        assert harness.server.goal_count == 1
        time.sleep(0.3)
        assert harness.server.goal_count == 1
    finally:
        harness.stop()


def test_explicit_start_service_supports_bounded_restart():
    harness = RuntimeHarness(auto_start=False)
    try:
        harness.publish_odom()
        response = harness.call_trigger(harness.start_client)
        assert response.success, response.message
        harness.wait_status(lambda value: value.get('state') == 'complete')
        assert harness.server.goal_count == 2

        harness.publish_odom()
        response = harness.call_trigger(harness.start_client)
        assert response.success, response.message
        assert wait_until(lambda: harness.server.goal_count == 4)
        harness.wait_status(
            lambda value: value.get('state') == 'complete'
            and value.get('target_count') == 2
        )
    finally:
        harness.stop()


def test_invalid_configuration_fails_before_goal():
    suffix = unique_suffix()
    endpoint = f'/fixture/scan360_mapper_{suffix}'
    environment = os.environ.copy()
    environment['ROS_LOCALHOST_ONLY'] = '1'
    environment['ROS_DOMAIN_ID'] = os.environ['ROS_DOMAIN_ID']
    result = subprocess.run(
        [
            installed_mapper_executable(),
            '--ros-args',
            '-p',
            f'action_name:={endpoint}',
            '-p',
            'auto_start:=true',
            '-p',
            'step_angle_rad:=0.0',
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    assert result.returncode != 0
    assert 'step_angle_rad' in result.stdout + result.stderr


def test_shutdown_waits_for_fixture_cancel_acknowledgement():
    cancel_delay = 0.18
    harness = RuntimeHarness(
        mode='hold',
        cancel_delay=cancel_delay,
    )
    stopped = False
    try:
        harness.publish_odom()
        assert wait_until(lambda: harness.server.active.is_set())
        started = time.monotonic()
        harness.process.send_signal(signal.SIGINT)
        assert wait_until(lambda: harness.server.cancel_seen.is_set())
        harness.process.wait(timeout=4.0)
        elapsed = time.monotonic() - started
        assert harness.process.returncode == 0, harness.output()
        assert harness.server.cancel_count >= 1
        assert elapsed >= cancel_delay * 0.75
        assert elapsed < 3.0
        stopped = True
    finally:
        harness.stop(expect_success=stopped)
