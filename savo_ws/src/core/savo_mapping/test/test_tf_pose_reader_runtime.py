import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from tf2_ros import TransformBroadcaster


PROCESS_TIMEOUT_SEC = 5.0


def unique_frames():
    suffix = f'{os.getpid()}_{uuid.uuid4().hex[:10]}'
    return (
        f'fixture_map_{suffix}',
        f'fixture_base_{suffix}',
    )


def installed_fixture_executable():
    configured = os.environ.get(
        'TF_POSE_READER_FIXTURE_EXECUTABLE',
        '',
    )
    if configured:
        candidate = Path(configured)
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)

    for prefix in os.environ.get(
        'AMENT_PREFIX_PATH',
        '',
    ).split(os.pathsep):
        candidate = (
            Path(prefix)
            / 'lib'
            / 'savo_mapping'
            / 'tf_pose_reader_fixture'
        )
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)

    raise AssertionError(
        'installed tf_pose_reader_fixture was not found'
    )


def fixture_command(
    target_frame,
    source_frame,
    *,
    lookup_timeout_sec=0.20,
    stale_timeout_sec=1.00,
    wait_sec=0.50,
):
    return [
        installed_fixture_executable(),
        '--target-frame',
        target_frame,
        '--source-frame',
        source_frame,
        '--lookup-timeout-sec',
        str(lookup_timeout_sec),
        '--stale-timeout-sec',
        str(stale_timeout_sec),
        '--wait-sec',
        str(wait_sec),
    ]


def fixture_environment():
    environment = os.environ.copy()
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    environment['ROS_LOCALHOST_ONLY'] = '1'
    environment['ROS_DOMAIN_ID'] = '220'
    return environment


def parse_result(output):
    line = next(
        (
            candidate
            for candidate in output.splitlines()
            if candidate.startswith('tf_pose_result ')
        ),
        '',
    )
    assert line, output

    return dict(
        token.split('=', 1)
        for token in line.split()[1:]
    )


def complete_process(process):
    try:
        output, _ = process.communicate(
            timeout=PROCESS_TIMEOUT_SEC
        )
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate(timeout=2.0)
        pytest.fail(
            'TF pose reader fixture exceeded its process bound:\n'
            + output
        )

    assert process.returncode == 0, output
    return parse_result(output), output


def start_process(command):
    return subprocess.Popen(
        command,
        env=fixture_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


class FixtureTransformStream:

    def __init__(self, target_frame, source_frame):
        suffix = uuid.uuid4().hex[:10]
        self.node = rclpy.create_node(
            f'tf_pose_fixture_stream_{suffix}'
        )
        self.broadcaster = TransformBroadcaster(self.node)
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            daemon=True,
        )

    def _run(self):
        while not self.stop_event.is_set():
            transform = TransformStamped()
            transform.header.stamp = (
                self.node.get_clock().now().to_msg()
            )
            transform.header.frame_id = self.target_frame
            transform.child_frame_id = self.source_frame
            transform.transform.translation.x = 1.25
            transform.transform.translation.y = -0.75
            transform.transform.translation.z = 0.065
            transform.transform.rotation.z = math.sin(
                math.pi / 8.0
            )
            transform.transform.rotation.w = math.cos(
                math.pi / 8.0
            )
            self.broadcaster.sendTransform(transform)
            self.stop_event.wait(0.02)

    def start(self):
        self.thread.start()
        time.sleep(0.35)

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        assert not self.thread.is_alive()
        self.node.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def isolated_ros_context():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '220'

    if not rclpy.ok():
        rclpy.init()

    yield

    if rclpy.ok():
        rclpy.shutdown()


def test_actual_reader_observes_valid_fixture_transform():
    target_frame, source_frame = unique_frames()
    stream = FixtureTransformStream(
        target_frame,
        source_frame,
    )
    stream.start()

    try:
        process = start_process(
            fixture_command(
                target_frame,
                source_frame,
                stale_timeout_sec=0.50,
                wait_sec=0.60,
            )
        )
        result, output = complete_process(process)
    finally:
        stream.stop()

    assert result['valid'] == '1', output
    assert result['fresh'] == '1', output
    assert result['reason'] == 'tf_pose_ready', output
    assert result['target'] == target_frame
    assert result['source'] == source_frame
    assert float(result['x_m']) == pytest.approx(1.25)
    assert float(result['y_m']) == pytest.approx(-0.75)
    assert float(result['yaw_rad']) == pytest.approx(
        math.pi / 4.0,
        abs=1.0e-6,
    )


def test_missing_transform_is_unavailable_and_bounded():
    target_frame, source_frame = unique_frames()
    started_at = time.monotonic()

    process = start_process(
        fixture_command(
            target_frame,
            source_frame,
            lookup_timeout_sec=0.12,
            stale_timeout_sec=0.50,
            wait_sec=0.05,
        )
    )
    result, output = complete_process(process)
    wall_duration_sec = time.monotonic() - started_at

    assert result['valid'] == '0', output
    assert result['fresh'] == '0', output
    assert result['reason'] == (
        'tf_pose_transform_unavailable'
    ), output
    assert 0.0 <= float(
        result['lookup_duration_sec']
    ) < 0.40
    assert wall_duration_sec < 2.0


def test_received_transform_becomes_stale():
    target_frame, source_frame = unique_frames()
    stream = FixtureTransformStream(
        target_frame,
        source_frame,
    )
    stream.start()

    process = start_process(
        fixture_command(
            target_frame,
            source_frame,
            lookup_timeout_sec=0.15,
            stale_timeout_sec=0.20,
            wait_sec=1.10,
        )
    )

    time.sleep(0.35)
    stream.stop()
    result, output = complete_process(process)

    assert result['valid'] == '0', output
    assert result['fresh'] == '0', output
    assert result['reason'] == (
        'tf_pose_transform_stale'
    ), output
    assert float(result['age_sec']) > 0.20


def test_fixture_exits_cleanly_after_each_lookup():
    target_frame, source_frame = unique_frames()
    process = start_process(
        fixture_command(
            target_frame,
            source_frame,
            lookup_timeout_sec=0.02,
            wait_sec=0.0,
        )
    )

    result, output = complete_process(process)

    assert process.returncode == 0, output
    assert result['reason'] == (
        'tf_pose_transform_unavailable'
    )
