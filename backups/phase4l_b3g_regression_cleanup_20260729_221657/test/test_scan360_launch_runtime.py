import json
import os
import signal
import subprocess
import time

import pytest
import rclpy
from std_msgs.msg import String


TIMEOUT = 12.0


def wait_until(predicate, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(CONTEXT_NODE, timeout_sec=0.05)
        if predicate():
            return True
    return predicate()


CONTEXT_NODE = None


@pytest.fixture
def launched_scan360(tmp_path):
    global CONTEXT_NODE

    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    domain_id = 180 + os.getpid() % 40
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_LOG_DIR'] = str(tmp_path / 'ros_logs')
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)

    rclpy.init()
    CONTEXT_NODE = rclpy.create_node(
        f'scan360_launch_observer_{os.getpid()}'
    )
    statuses = []

    def receive_status(message):
        try:
            statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    subscription = CONTEXT_NODE.create_subscription(
        String,
        '/savo_mapping/scan360/status',
        receive_status,
        10,
    )
    fixture_endpoint = f'/fixture/scan360_launch_{os.getpid()}'
    environment = os.environ.copy()
    command = [
        'ros2',
        'launch',
        'savo_mapping',
        'scan360_mapping.launch.xml',
        f'action_name:={fixture_endpoint}',
        'auto_start:=false',
        'enabled:=true',
        'use_real_robot_profile:=false',
        'use_sim_time:=false',
    ]
    process = subprocess.Popen(
        command,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    yield process, statuses, fixture_endpoint

    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=6.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
            pytest.fail('Scan360 launch did not terminate within its bound')

    CONTEXT_NODE.destroy_subscription(subscription)
    CONTEXT_NODE.destroy_node()
    CONTEXT_NODE = None
    rclpy.shutdown()


def test_installed_launch_starts_observer_only_node(launched_scan360):
    process, statuses, fixture_endpoint = launched_scan360

    def started():
        node_names = CONTEXT_NODE.get_node_names_and_namespaces()
        return (
            process.poll() is not None
            or ('scan360_mapper_node', '/') in node_names
        )

    assert wait_until(started), 'installed Scan360 launch did not start'
    assert process.poll() is None, (
        process.stdout.read() if process.stdout else 'launch exited'
    )
    assert wait_until(lambda: bool(statuses)), (
        'Scan360 status interface did not publish'
    )
    assert statuses[-1]['state'] == 'waiting_for_odom'
    assert statuses[-1]['terminal'] is False

    topic_names = {
        name for name, _types in CONTEXT_NODE.get_topic_names_and_types()
    }
    assert '/savo_mapping/scan360/status' in topic_names
    assert '/savo_mapping/scan360/state' in topic_names

    velocity_root = 'cmd' + '_vel'
    for topic in (
        '/' + velocity_root,
        '/' + velocity_root + '_auto',
    ):
        assert not CONTEXT_NODE.get_publishers_info_by_topic(topic), (
            f'Scan360 launch unexpectedly exposes publisher {topic}'
        )

    assert fixture_endpoint.startswith('/fixture/')
    assert all(
        status.get('action_client_state') == 'idle'
        for status in statuses
    ), 'auto_start=false must not dispatch an action goal'
