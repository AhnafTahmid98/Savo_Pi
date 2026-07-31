#!/usr/bin/env python3

"""Isolated runtime coverage for the production coverage planning node."""

import json
import math
import os
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
import pytest
import rclpy
from rclpy.action import get_action_client_names_and_types_by_node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


PROCESS_TIMEOUT_SEC = 8.0
WAIT_TIMEOUT_SEC = 8.0
COUNTER = 0


def unique_suffix():
    """Return a process-local suffix for every fixture graph."""
    global COUNTER
    COUNTER += 1
    return f'{os.getpid()}_{COUNTER}'


def wait_until(predicate, timeout=WAIT_TIMEOUT_SEC):
    """Evaluate a predicate until it succeeds or its monotonic bound ends."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def installed_mapper_executable():
    """Locate the coverage mapper installed by the current workspace."""
    configured = os.environ.get(
        'COVERAGE_MAPPER_EXECUTABLE',
        '',
    )
    if configured:
        if os.path.isfile(configured) and os.access(configured, os.X_OK):
            return configured

    for prefix in os.environ.get(
        'AMENT_PREFIX_PATH',
        '',
    ).split(os.pathsep):
        candidate = os.path.join(
            prefix,
            'lib',
            'savo_mapping',
            'coverage_mapper_node',
        )
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate

    raise AssertionError('installed coverage_mapper_node was not found')


def state_qos():
    """Return the package's reliable retained-state policy."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def status_qos():
    """Return the package's reliable volatile-status policy."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class RuntimeHarness:
    """Own one installed mapper and its isolated ROS fixture graph."""

    def __init__(
        self,
        *,
        enabled=True,
        auto_plan=True,
        plan_once=True,
        replan_on_map_update=False,
        tick_period_sec=0.05,
        map_stale_timeout_sec=2.0,
        tf_lookup_timeout_sec=0.05,
        tf_stale_timeout_sec=0.50,
        extra_parameters=None,
    ):
        """Create fixture interfaces and start the installed mapper."""
        suffix = unique_suffix()
        self.map_topic = f'/fixture/coverage_map_{suffix}'
        self.path_topic = f'/fixture/coverage_path_{suffix}'
        self.status_topic = f'/fixture/coverage_status_{suffix}'
        self.state_topic = f'/fixture/coverage_state_{suffix}'
        self.map_frame = f'fixture_map_{suffix}'
        self.base_frame = f'fixture_base_{suffix}'

        self.node = rclpy.create_node(
            f'coverage_mapper_fixture_{suffix}'
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)

        self.paths = []
        self.statuses = []
        self.states = []

        self.map_publisher = self.node.create_publisher(
            OccupancyGrid,
            self.map_topic,
            state_qos(),
        )
        self.path_subscription = self.node.create_subscription(
            Path,
            self.path_topic,
            self._receive_path,
            state_qos(),
        )
        self.status_subscription = self.node.create_subscription(
            String,
            self.status_topic,
            self._receive_status,
            status_qos(),
        )
        self.state_subscription = self.node.create_subscription(
            String,
            self.state_topic,
            self._receive_state,
            state_qos(),
        )
        self.tf_broadcaster = TransformBroadcaster(self.node)

        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()

        self.tf_stop_event = None
        self.tf_thread = None
        self.tf_pose_lock = threading.Lock()
        self.tf_x_m = 0.5
        self.tf_y_m = 0.5
        self.tf_yaw_rad = 0.0

        parameters = {
            'enabled': enabled,
            'auto_plan': auto_plan,
            'plan_once': plan_once,
            'replan_on_map_update': replan_on_map_update,
            'map_topic': self.map_topic,
            'map_frame': self.map_frame,
            'base_frame': self.base_frame,
            'path_topic': self.path_topic,
            'status_topic': self.status_topic,
            'state_topic': self.state_topic,
            'tick_period_sec': tick_period_sec,
            'map_stale_timeout_sec': map_stale_timeout_sec,
            'tf_lookup_timeout_sec': tf_lookup_timeout_sec,
            'tf_stale_timeout_sec': tf_stale_timeout_sec,
            'free_threshold': 0,
            'occupied_threshold': 65,
            'allow_unknown': False,
            'inflation_radius_m': 0.0,
            'connectivity': 'four',
            'sweep_axis': 'rows',
            'track_spacing_m': 1.0,
            'minimum_segment_length_m': 0.0,
            'maximum_waypoints': 10000,
        }
        parameters.update(extra_parameters or {})

        command = [
            installed_mapper_executable(),
            '--ros-args',
        ]
        for name, value in parameters.items():
            if isinstance(value, bool):
                parameter_value = str(value).lower()
            else:
                parameter_value = str(value)
            command.extend(('-p', f'{name}:={parameter_value}'))

        environment = os.environ.copy()
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        environment['ROS_LOCALHOST_ONLY'] = '1'
        environment['ROS_DOMAIN_ID'] = os.environ.get(
            'ROS_DOMAIN_ID',
            '221',
        )

        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self.process_output = ''

        try:
            assert wait_until(
                lambda: (
                    self.process.poll() is not None
                    or bool(self.statuses)
                )
            ), self.diagnostics()
            assert self.process.poll() is None, self.diagnostics()
        except BaseException:
            self.stop(
                expect_success=False,
                allow_forced_shutdown=True,
            )
            raise

    def _receive_path(self, message):
        """Record a published read-only coverage plan."""
        self.paths.append(message)

    def _receive_status(self, message):
        """Decode and record a mapper status snapshot."""
        try:
            self.statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    def _receive_state(self, message):
        """Record a stable mapper state transition."""
        self.states.append(message.data)

    def diagnostics(self):
        """Return bounded diagnostic context without blocking on live output."""
        status_tail = self.statuses[-3:]
        process_state = self.process.poll()
        return (
            f'process_state={process_state} '
            f'status_tail={status_tail!r} '
            f'output={self.process_output!r}'
        )

    def latest_status(self):
        """Return the newest decoded status or an empty mapping."""
        return self.statuses[-1] if self.statuses else {}

    def wait_status(self, predicate, timeout=WAIT_TIMEOUT_SEC):
        """Wait for a status satisfying a caller-provided predicate."""
        assert wait_until(
            lambda: (
                bool(self.statuses)
                and predicate(self.latest_status())
            ),
            timeout,
        ), self.diagnostics()
        return self.latest_status()

    def wait_state_reason(self, state, reason, detail=None):
        """Wait until the exact state and reason appear in status history."""
        matched = [None]

        def observed():
            for status in reversed(list(self.statuses)):
                if (
                    status.get('state') != state
                    or status.get('reason') != reason
                ):
                    continue
                if detail is not None and detail not in str(
                    status.get('detail', '')
                ):
                    continue
                matched[0] = status
                return True
            return False

        assert wait_until(
            observed,
            WAIT_TIMEOUT_SEC,
        ), self.diagnostics()
        return matched[0]

    def wait_path_count(self, count, timeout=WAIT_TIMEOUT_SEC):
        """Wait until exactly the requested number of plans is observed."""
        assert wait_until(
            lambda: len(self.paths) >= count,
            timeout,
        ), self.diagnostics()
        assert len(self.paths) == count, self.diagnostics()
        return self.paths[-1]

    def make_map(
        self,
        *,
        width=5,
        height=5,
        resolution=1.0,
        data=None,
        frame=None,
        origin=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
    ):
        """Build one occupancy fixture without contacting a map producer."""
        message = OccupancyGrid()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = (
            self.map_frame if frame is None else frame
        )
        message.info.map_load_time = message.header.stamp
        message.info.width = width
        message.info.height = height
        message.info.resolution = resolution
        (
            message.info.origin.position.x,
            message.info.origin.position.y,
            message.info.origin.position.z,
        ) = origin
        (
            message.info.origin.orientation.x,
            message.info.origin.orientation.y,
            message.info.origin.orientation.z,
            message.info.origin.orientation.w,
        ) = orientation
        if data is None:
            data = [0] * (width * height)
        message.data = data
        return message

    def publish_map(self, message, repeat=5):
        """Publish one semantic map repeatedly for reliable discovery."""
        for _ in range(repeat):
            self.map_publisher.publish(message)
            time.sleep(0.03)

    def set_tf_pose(self, x_m, y_m, yaw_rad=0.0):
        """Change the pose sent by the active fixture TF stream."""
        with self.tf_pose_lock:
            self.tf_x_m = x_m
            self.tf_y_m = y_m
            self.tf_yaw_rad = yaw_rad

    def start_tf(self, x_m=0.5, y_m=0.5, yaw_rad=0.0):
        """Start a fresh transform stream for this fixture's frames."""
        self.stop_tf()
        self.set_tf_pose(x_m, y_m, yaw_rad)
        self.tf_stop_event = threading.Event()
        self.tf_thread = threading.Thread(
            target=self._broadcast_tf,
            daemon=True,
        )
        self.tf_thread.start()
        time.sleep(0.35)

    def _broadcast_tf(self):
        """Publish current fixture pose until bounded teardown requests stop."""
        while not self.tf_stop_event.is_set():
            with self.tf_pose_lock:
                x_m = self.tf_x_m
                y_m = self.tf_y_m
                yaw_rad = self.tf_yaw_rad

            transform = TransformStamped()
            transform.header.stamp = (
                self.node.get_clock().now().to_msg()
            )
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = x_m
            transform.transform.translation.y = y_m
            transform.transform.rotation.z = math.sin(
                yaw_rad * 0.5
            )
            transform.transform.rotation.w = math.cos(
                yaw_rad * 0.5
            )
            self.tf_broadcaster.sendTransform(transform)
            self.tf_stop_event.wait(0.02)

    def stop_tf(self):
        """Stop and join the fixture transform stream."""
        if self.tf_stop_event is not None:
            self.tf_stop_event.set()
        if self.tf_thread is not None:
            self.tf_thread.join(timeout=2.0)
            assert not self.tf_thread.is_alive()
        self.tf_stop_event = None
        self.tf_thread = None

    def assert_path_count_stable(self, duration_sec=0.35):
        """Assert timer activity does not duplicate the current plan."""
        path_count = len(self.paths)
        time.sleep(duration_sec)
        assert len(self.paths) == path_count, self.diagnostics()

    def stop(
        self,
        expect_success=True,
        allow_forced_shutdown=False,
    ):
        """Stop every fixture process and executor within fixed bounds."""
        self.stop_tf()
        forced_shutdown = False

        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

            try:
                self.process.wait(timeout=PROCESS_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                forced_shutdown = True
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=2.0)

        if self.process.stdout is not None:
            self.process_output = self.process.stdout.read()

        self.executor.shutdown(timeout_sec=2.0)
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()

        assert not self.spin_thread.is_alive()
        if not allow_forced_shutdown:
            assert not forced_shutdown, (
                'coverage mapper subprocess exceeded its shutdown bound\n'
                + self.process_output
            )
        if expect_success:
            assert self.process.returncode == 0, self.process_output
        return self.process_output


@pytest.fixture(scope='module', autouse=True)
def isolated_ros_context(tmp_path_factory):
    """Provide one isolated ROS context for all runtime scenarios."""
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '221'

    previous_log_dir = os.environ.get('ROS_LOG_DIR')
    os.environ['ROS_LOG_DIR'] = str(
        tmp_path_factory.mktemp('coverage_mapper_ros_logs')
    )

    if not rclpy.ok():
        rclpy.init()

    yield

    if rclpy.ok():
        rclpy.shutdown()

    if previous_log_dir is None:
        os.environ.pop('ROS_LOG_DIR', None)
    else:
        os.environ['ROS_LOG_DIR'] = previous_log_dir


def test_safe_activation_and_input_gates():
    """Verify startup gates cannot publish a plan without explicit inputs."""
    scenarios = (
        (
            {'auto_plan': False},
            'ready',
            'coverage_node_auto_plan_disabled',
            True,
            True,
        ),
        (
            {'enabled': False},
            'disabled',
            'coverage_node_disabled',
            True,
            True,
        ),
        (
            {},
            'waiting_for_map',
            'coverage_node_waiting_for_map',
            False,
            True,
        ),
        (
            {},
            'waiting_for_pose',
            'coverage_node_waiting_for_pose',
            True,
            False,
        ),
    )

    for overrides, state, reason, send_map, send_tf in scenarios:
        harness = RuntimeHarness(**overrides)
        try:
            if send_tf:
                harness.start_tf()
            if send_map:
                harness.publish_map(harness.make_map())

            harness.wait_state_reason(state, reason)
            harness.assert_path_count_stable()
            assert harness.process.poll() is None
        finally:
            harness.stop()


def test_invalid_maps_are_rejected_without_output():
    """Reject malformed map messages while preserving read-only behavior."""
    harness = RuntimeHarness()
    try:
        harness.start_tf()

        wrong_frame = harness.make_map(
            frame=f'fixture_wrong_{unique_suffix()}'
        )
        harness.publish_map(wrong_frame)
        harness.wait_state_reason(
            'map_invalid',
            'coverage_node_map_frame_mismatch',
        )
        assert not harness.paths

        invalid_cases = (
            (
                harness.make_map(width=0, height=5, data=[]),
                'dimensions',
            ),
            (
                harness.make_map(resolution=0.0),
                'resolution',
            ),
            (
                harness.make_map(data=[0] * 24),
                'data_size',
            ),
            (
                harness.make_map(
                    origin=(float('nan'), 0.0, 0.0)
                ),
                'origin',
            ),
        )

        for message, detail in invalid_cases:
            path_count = len(harness.paths)
            harness.publish_map(message)
            harness.wait_state_reason(
                'map_invalid',
                'coverage_node_map_invalid',
                detail,
            )
            assert len(harness.paths) == path_count

        harness.assert_path_count_stable()
    finally:
        harness.stop()


def test_stale_map_blocks_planning():
    """Use steady reception age to prevent planning from an expired map."""
    harness = RuntimeHarness(
        map_stale_timeout_sec=0.12,
        tf_stale_timeout_sec=1.0,
    )
    try:
        harness.publish_map(harness.make_map())
        time.sleep(0.25)
        harness.start_tf()

        status = harness.wait_state_reason(
            'map_stale',
            'coverage_node_map_stale',
        )
        assert status['map_valid'] is True
        assert status['map_fresh'] is False
        harness.assert_path_count_stable()
    finally:
        harness.stop()


def test_valid_plan_geometry_metrics_and_read_only_graph():
    """Validate successful plan output, metrics, and graph ownership."""
    harness = RuntimeHarness()
    try:
        width = 7
        height = 5
        data = [0] * (width * height)
        central_obstacle = (2, 2)
        data[central_obstacle[1] * width + central_obstacle[0]] = 100
        for row in range(height):
            data[row * width + 5] = 100

        harness.start_tf(0.5, 0.5)
        fixture_map = harness.make_map(
            width=width,
            height=height,
            data=data,
        )
        harness.publish_map(fixture_map)
        path = harness.wait_path_count(1)
        status = harness.wait_state_reason(
            'plan_ready',
            'coverage_node_plan_ready',
        )

        required_status = {
            'enabled',
            'auto_plan',
            'state',
            'reason',
            'map_valid',
            'map_fresh',
            'pose_valid',
            'pose_fresh',
            'waypoint_count',
            'reachable_cell_count',
            'covered_cell_count',
            'coverage_ratio',
            'estimated_path_length_m',
            'map_sequence',
            'plan_sequence',
            'terminal',
        }
        assert required_status <= status.keys()
        assert status['enabled'] is True
        assert status['auto_plan'] is True
        assert status['map_valid'] is True
        assert status['map_fresh'] is True
        assert status['pose_valid'] is True
        assert status['pose_fresh'] is True
        assert status['waypoint_count'] == len(path.poses)
        assert status['reachable_cell_count'] > 0
        assert status['covered_cell_count'] > 0
        assert 0.0 < status['coverage_ratio'] <= 1.0
        assert math.isfinite(status['estimated_path_length_m'])
        assert status['estimated_path_length_m'] >= 0.0
        assert status['plan_sequence'] == 1

        assert path.header.frame_id == harness.map_frame
        assert path.poses
        path_stamp = (
            path.header.stamp.sec,
            path.header.stamp.nanosec,
        )
        assert path_stamp != (0, 0)

        visited_cells = set()
        for pose_stamped in path.poses:
            assert pose_stamped.header.frame_id == harness.map_frame
            assert (
                pose_stamped.header.stamp.sec,
                pose_stamped.header.stamp.nanosec,
            ) == path_stamp

            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation
            assert all(
                math.isfinite(value)
                for value in (
                    position.x,
                    position.y,
                    position.z,
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )
            norm = math.sqrt(
                orientation.x * orientation.x
                + orientation.y * orientation.y
                + orientation.z * orientation.z
                + orientation.w * orientation.w
            )
            assert norm == pytest.approx(1.0, abs=1.0e-9)

            column = math.floor(position.x)
            row = math.floor(position.y)
            visited_cells.add((column, row))

        assert central_obstacle not in visited_cells
        assert all(column < 6 for column, _row in visited_cells)

        harness.publish_map(fixture_map)
        harness.assert_path_count_stable()

        assert wait_until(
            lambda: (
                'coverage_mapper_node',
                '/',
            ) in harness.node.get_node_names_and_namespaces()
        )
        action_clients = get_action_client_names_and_types_by_node(
            harness.node,
            'coverage_mapper_node',
            '/',
        )
        assert action_clients == []

        publishers = (
            harness.node.get_publisher_names_and_types_by_node(
                'coverage_mapper_node',
                '/',
            )
        )
        prohibited_message_type = (
            'geometry_msgs/msg/' + 'Tw' + 'ist'
        )
        for _topic, message_types in publishers:
            assert prohibited_message_type not in message_types

        velocity_root = 'cmd' + '_vel'
        control_root = '/' + 'savo_' + 'control'
        prohibited_topics = (
            '/' + velocity_root,
            '/' + velocity_root + '_auto',
            '/' + velocity_root + '_safe',
            '/' + velocity_root + '_recovery',
            control_root + '/' + 'mode_' + 'cmd',
            control_root + '/' + 'recovery_' + 'request',
        )
        for topic in prohibited_topics:
            node_publishers = [
                endpoint
                for endpoint in (
                    harness.node.get_publishers_info_by_topic(topic)
                )
                if endpoint.node_name == 'coverage_mapper_node'
            ]
            assert not node_publishers
    finally:
        harness.stop()


def test_changed_map_replans_once_and_invalid_update_is_contained():
    """Permit one changed-map replan and contain later invalid input."""
    harness = RuntimeHarness(
        plan_once=False,
        replan_on_map_update=True,
    )
    try:
        harness.start_tf()
        initial_map = harness.make_map(
            width=4,
            height=3,
            data=[0] * 12,
        )
        harness.publish_map(initial_map)
        harness.wait_path_count(1)
        harness.publish_map(initial_map)
        harness.assert_path_count_stable()

        changed_data = [0] * 12
        changed_data[5] = 100
        changed_map = harness.make_map(
            width=4,
            height=3,
            data=changed_data,
        )
        harness.publish_map(changed_map)
        harness.wait_path_count(2)
        matching_status = [None]

        def observed_second_plan():
            for candidate in reversed(list(harness.statuses)):
                if (
                    candidate.get('state') == 'plan_ready'
                    and candidate.get('reason')
                    == 'coverage_node_plan_ready'
                    and candidate.get('plan_sequence') == 2
                ):
                    matching_status[0] = candidate
                    return True
            return False

        assert wait_until(
            observed_second_plan
        ), harness.diagnostics()
        status = matching_status[0]
        accepted_sequence = status['map_sequence']

        harness.publish_map(changed_map)
        harness.assert_path_count_stable()

        invalid_update = harness.make_map(
            width=4,
            height=3,
            data=[0] * 11,
        )
        harness.publish_map(invalid_update)
        invalid_status = harness.wait_state_reason(
            'map_invalid',
            'coverage_node_map_invalid',
            'data_size',
        )
        assert invalid_status['map_sequence'] == accepted_sequence
        harness.assert_path_count_stable()
        assert len(harness.paths) == 2
    finally:
        harness.stop()


@pytest.mark.parametrize(
    (
        'pose',
        'blocked_start',
        'reason',
    ),
    (
        (
            (0.5, 0.5),
            True,
            'coverage_node_start_blocked',
        ),
        (
            (-0.5, 0.5),
            False,
            'coverage_node_start_out_of_bounds',
        ),
    ),
)
def test_invalid_robot_start_produces_no_plan(
    pose,
    blocked_start,
    reason,
):
    """Reject blocked and out-of-grid fixture robot starts."""
    harness = RuntimeHarness()
    try:
        data = [0] * 9
        if blocked_start:
            data[0] = 100

        harness.start_tf(*pose)
        harness.publish_map(
            harness.make_map(
                width=3,
                height=3,
                data=data,
            )
        )
        harness.wait_state_reason(
            'plan_failed',
            reason,
        )
        harness.assert_path_count_stable()
    finally:
        harness.stop()


def test_stale_tf_blocks_planning():
    """Reject a cached fixture transform after its freshness limit."""
    harness = RuntimeHarness(
        map_stale_timeout_sec=2.0,
        tf_stale_timeout_sec=0.10,
    )
    try:
        harness.start_tf()
        harness.stop_tf()
        time.sleep(0.20)
        harness.publish_map(harness.make_map())

        status = harness.wait_state_reason(
            'pose_stale',
            'coverage_node_pose_stale',
        )
        assert status['pose_valid'] is False
        assert status['pose_fresh'] is False
        assert 'stale' in str(status.get('detail', ''))
        harness.assert_path_count_stable()
    finally:
        harness.stop()


def test_invalid_configuration_exits_before_runtime_output():
    """Ensure invalid node configuration fails before any plan is possible."""
    suffix = unique_suffix()
    fixture_prefix = f'/fixture/coverage_invalid_{suffix}'
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['ROS_LOCALHOST_ONLY'] = '1'
    environment['ROS_DOMAIN_ID'] = os.environ['ROS_DOMAIN_ID']

    result = subprocess.run(
        [
            installed_mapper_executable(),
            '--ros-args',
            '-p',
            'enabled:=true',
            '-p',
            'auto_plan:=true',
            '-p',
            f'map_topic:={fixture_prefix}_map',
            '-p',
            f'path_topic:={fixture_prefix}_path',
            '-p',
            f'status_topic:={fixture_prefix}_status',
            '-p',
            f'state_topic:={fixture_prefix}_state',
            '-p',
            f'map_frame:=fixture_map_invalid_{suffix}',
            '-p',
            f'base_frame:=fixture_base_invalid_{suffix}',
            '-p',
            'tick_period_sec:=0.0',
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
        start_new_session=True,
    )
    output = result.stdout + result.stderr
    assert result.returncode != 0
    assert 'coverage_node_tick_period_invalid' in output
