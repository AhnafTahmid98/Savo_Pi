#!/usr/bin/env python3

"""Installed-launch runtime checks for Coverage Mapping deployment assets."""

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


WAIT_TIMEOUT_SEC = 12.0


def state_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def status_qos():
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def wait_until(predicate, timeout=WAIT_TIMEOUT_SEC):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class LaunchHarness:
    def __init__(self, tmp_path, *, auto_plan):
        suffix = f'{os.getpid()}_{int(auto_plan)}'
        self.map_topic = f'/fixture/coverage_launch_map_{suffix}'
        self.path_topic = f'/fixture/coverage_launch_path_{suffix}'
        self.status_topic = f'/fixture/coverage_launch_status_{suffix}'
        self.state_topic = f'/fixture/coverage_launch_state_{suffix}'
        self.map_frame = f'fixture_map_{suffix}'
        self.base_frame = f'fixture_base_{suffix}'
        self.paths = []
        self.statuses = []
        self.states = []

        self.node = rclpy.create_node(f'coverage_launch_fixture_{suffix}')
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        self.map_publisher = self.node.create_publisher(
            OccupancyGrid,
            self.map_topic,
            state_qos(),
        )
        self.path_subscription = self.node.create_subscription(
            Path,
            self.path_topic,
            self.paths.append,
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
            lambda message: self.states.append(message.data),
            state_qos(),
        )
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.stop_event = threading.Event()
        self.spin_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.spin_thread.start()
        self.tf_thread = None

        environment = os.environ.copy()
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        environment['ROS_LOCALHOST_ONLY'] = '1'
        environment['ROS_DOMAIN_ID'] = os.environ.get('ROS_DOMAIN_ID', '222')
        environment['ROS_LOG_DIR'] = str(tmp_path / f'ros_logs_{suffix}')
        os.makedirs(environment['ROS_LOG_DIR'], exist_ok=True)

        command = [
            'ros2',
            'launch',
            'savo_mapping',
            'coverage_mapping.launch.xml',
            'use_real_robot_profile:=false',
            'enabled:=true',
            f'auto_plan:={str(auto_plan).lower()}',
            'plan_once:=true',
            'replan_on_map_update:=false',
            f'map_topic:={self.map_topic}',
            f'map_frame:={self.map_frame}',
            f'base_frame:={self.base_frame}',
            f'path_topic:={self.path_topic}',
            f'status_topic:={self.status_topic}',
            f'state_topic:={self.state_topic}',
            'use_sim_time:=false',
        ]
        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        assert wait_until(
            lambda: self.process.poll() is not None or bool(self.statuses)
        ), self.diagnostics()
        assert self.process.poll() is None, self.diagnostics()

    def _receive_status(self, message):
        try:
            self.statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    def diagnostics(self):
        if self.process.poll() is None or self.process.stdout is None:
            return 'Coverage launch is running without expected interfaces'
        return self.process.stdout.read()

    def start_tf(self):
        def broadcast():
            while not self.stop_event.is_set():
                message = TransformStamped()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.header.frame_id = self.map_frame
                message.child_frame_id = self.base_frame
                message.transform.translation.x = 0.5
                message.transform.translation.y = 0.5
                message.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(message)
                self.stop_event.wait(0.05)

        self.tf_thread = threading.Thread(target=broadcast, daemon=True)
        self.tf_thread.start()

    def publish_map(self):
        message = OccupancyGrid()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.info.resolution = 1.0
        message.info.width = 5
        message.info.height = 4
        message.info.origin.orientation.w = 1.0
        message.data = [0] * 20
        for _ in range(4):
            self.map_publisher.publish(message)
            time.sleep(0.08)

    def assert_no_authority(self):
        assert wait_until(
            lambda: ('coverage_mapper_node', '/') in (
                self.node.get_node_names_and_namespaces()
            )
        )
        action_clients = get_action_client_names_and_types_by_node(
            self.node,
            'coverage_mapper_node',
            '/',
        )
        assert action_clients == []
        prohibited_type = 'geometry_msgs/msg/' + 'Tw' + 'ist'
        publishers = self.node.get_publisher_names_and_types_by_node(
            'coverage_mapper_node',
            '/',
        )
        for _topic, message_types in publishers:
            assert prohibited_type not in message_types
        velocity_root = 'cmd' + '_vel'
        for topic in ('/' + velocity_root, '/' + velocity_root + '_auto'):
            node_publishers = [
                endpoint
                for endpoint in self.node.get_publishers_info_by_topic(topic)
                if endpoint.node_name == 'coverage_mapper_node'
            ]
            assert node_publishers == []

    def stop(self):
        self.stop_event.set()
        if self.tf_thread is not None:
            self.tf_thread.join(timeout=2.0)
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=2.0)
                pytest.fail('Coverage launch did not terminate within bound')
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        self.spin_thread.join(timeout=2.0)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    yield
    rclpy.shutdown()


def test_installed_launch_safe_start_is_observer_only(tmp_path):
    harness = LaunchHarness(tmp_path, auto_plan=False)
    try:
        assert wait_until(lambda: bool(harness.statuses))
        assert harness.paths == []
        assert harness.statuses[-1]['auto_plan'] is False
        assert harness.statuses[-1]['state'] == 'waiting_for_map'
        assert (
            harness.statuses[-1]['reason']
            == 'coverage_node_waiting_for_map'
        )
        harness.assert_no_authority()
    finally:
        harness.stop()


def test_installed_launch_can_publish_read_only_fixture_plan(tmp_path):
    harness = LaunchHarness(tmp_path, auto_plan=True)
    try:
        harness.start_tf()
        harness.publish_map()
        assert wait_until(lambda: bool(harness.paths)), harness.diagnostics()
        path = harness.paths[-1]
        assert path.header.frame_id == harness.map_frame
        assert path.poses
        for pose in path.poses:
            position = pose.pose.position
            orientation = pose.pose.orientation
            assert math.isfinite(position.x)
            assert math.isfinite(position.y)
            norm = math.sqrt(
                orientation.x * orientation.x
                + orientation.y * orientation.y
                + orientation.z * orientation.z
                + orientation.w * orientation.w
            )
            assert norm == pytest.approx(1.0, abs=1.0e-9)
        assert wait_until(
            lambda: any(
                status.get('state') == 'plan_ready'
                for status in harness.statuses
            )
        )
        harness.assert_no_authority()
    finally:
        harness.stop()
