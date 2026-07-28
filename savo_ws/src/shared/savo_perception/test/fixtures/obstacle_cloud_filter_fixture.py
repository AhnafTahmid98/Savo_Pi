# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Synthetic PointCloud2 runtime fixtures for the Phase 8A filter."""

import argparse
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


RAW_TOPIC = '/camera/camera/depth/color/points'
OUTPUT_TOPIC = '/savo_perception/obstacles/points'
HEALTH_TOPIC = '/savo_perception/obstacle_cloud/health'
STATUS_TOPIC = '/savo_perception/obstacle_cloud/status'
HEARTBEAT_TOPIC = '/savo_perception/obstacle_cloud/heartbeat'
VALID_FRAME = 'phase8a_camera_frame'


class ObstacleCloudFixture(Node):
    """Publish synthetic clouds and observe the production filter."""

    def __init__(self):
        """Create fixture publishers, subscribers, and static transform."""
        super().__init__('phase8a_obstacle_cloud_fixture')
        self.publisher = self.create_publisher(
            PointCloud2,
            RAW_TOPIC,
            qos_profile_sensor_data,
        )
        self.output_messages = []
        self.health = None
        self.status = {}
        self.heartbeat = None

        self.create_subscription(
            PointCloud2,
            OUTPUT_TOPIC,
            self._on_output,
            qos_profile_sensor_data,
        )
        self.create_subscription(Bool, HEALTH_TOPIC, self._on_health, 10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(
            String,
            HEARTBEAT_TOPIC,
            self._on_heartbeat,
            10,
        )

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = VALID_FRAME
        transform.transform.translation.x = 0.40
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def _on_output(self, message):
        """Record a filtered output cloud."""
        self.output_messages.append(message)

    def _on_health(self, message):
        """Record the latest health state."""
        self.health = message.data

    def _on_status(self, message):
        """Decode the deterministic JSON status."""
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            self.status = {}

    def _on_heartbeat(self, message):
        """Record the latest heartbeat."""
        self.heartbeat = message.data

    def cloud(self, frame_id, points, stamp=None):
        """Build an XYZ-only FLOAT32 unorganized PointCloud2."""
        message = PointCloud2()
        message.header.frame_id = frame_id
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b''.join(
            struct.pack('<fff', *point)
            for point in points
        )
        message.is_dense = False
        return message

    def spin_until(self, predicate, timeout_sec=5.0):
        """Spin until a predicate succeeds or the deadline expires."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def publish_until(self, message, predicate, timeout_sec=5.0):
        """Publish repeatedly until discovery and processing complete."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.10)
            if predicate():
                return True
        return predicate()


def decode_points(message):
    """Decode an XYZ FLOAT32 PointCloud2 into tuples."""
    return [
        struct.unpack_from('<fff', message.data, index * message.point_step)
        for index in range(message.width)
    ]


def has_point(points, expected, tolerance=1.0e-4):
    """Return whether a point exists within a finite tolerance."""
    return any(
        all(
            math.isclose(actual, target, abs_tol=tolerance)
            for actual, target in zip(point, expected)
        )
        for point in points
    )


def valid_cloud(fixture):
    """Create the shared valid obstacle cloud."""
    return fixture.cloud(
        VALID_FRAME,
        [
            (0.60, 0.00, 0.30),
            (float('nan'), 0.00, 0.30),
            (float('inf'), 0.00, 0.30),
            (-0.25, 0.00, 0.30),
            (3.10, 0.00, 0.30),
            (0.60, 0.00, 0.01),
            (0.60, 0.00, 1.70),
            (-0.15, 0.00, 0.30),
            (0.80, 0.00, 0.30),
            (0.81, 0.00, 0.30),
            (0.90, 0.00, 0.30),
        ],
    )


def wait_for_ready_status_contract(
    fixture,
    timeout_seconds=5.0,
):
    """Wait for the processed-cloud ready status contract."""
    deadline = time.monotonic() + timeout_seconds
    last_status = {}

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(fixture, timeout_sec=0.05)
        last_status = dict(fixture.status)

        if (
            last_status.get('state') == 'ready'
            and last_status.get('semantics') == 'obstacle_only'
            and last_status.get('clearing_supported') is False
        ):
            return last_status

    raise AssertionError(
        'Timed out waiting for ready obstacle-cloud status. '
        f'Last status: {last_status!r}'
    )


def run_filter(fixture):
    """Validate filtering, transform, schema, and obstacle semantics."""
    message = valid_cloud(fixture)
    stamp = message.header.stamp
    assert fixture.publish_until(
        message,
        lambda: fixture.health is True and bool(fixture.output_messages),
    )

    output = fixture.output_messages[-1]
    points = decode_points(output)
    assert output.header.stamp == stamp
    assert output.header.frame_id == 'base_link'
    assert output.height == 1
    assert [field.name for field in output.fields] == ['x', 'y', 'z']
    assert all(
        field.datatype == PointField.FLOAT32
        for field in output.fields
    )
    assert has_point(points, (1.00, 0.00, 0.30))
    assert has_point(points, (1.20, 0.00, 0.30))
    assert has_point(points, (1.30, 0.00, 0.30))
    assert len(points) == 3
    ready_status = wait_for_ready_status_contract(fixture)

    assert ready_status['semantics'] == 'obstacle_only'
    assert ready_status['clearing_supported'] is False
    assert ready_status['input_points'] == 11
    assert ready_status['finite_points'] == 9
    assert ready_status['range_rejected'] == 2
    assert ready_status['height_rejected'] == 2
    assert ready_status['self_rejected'] == 1
    assert ready_status['voxel_rejected'] == 1
    assert fixture.heartbeat is not None or fixture.spin_until(
        lambda: fixture.heartbeat is not None,
        timeout_sec=2.0,
    )

    print('Synthetic obstacle filtering: PASSED')
    print('PointCloud2 timestamp preservation: PASSED')
    print('PointCloud2 frame transformation: PASSED')
    print('Floor and height filtering: PASSED')
    print('Robot self-filtering: PASSED')
    print('Range filtering: PASSED')
    print('Finite-point filtering: PASSED')
    print('Voxel downsampling: PASSED')
    print('Obstacle-only output contract: PASSED')


def run_bad_frame(fixture):
    """Validate missing-transform containment and recovery."""
    initial_output_count = len(fixture.output_messages)
    bad_cloud = fixture.cloud(
        'phase8a_missing_frame',
        [(1.0, 0.0, 0.3)],
    )

    assert fixture.publish_until(
        bad_cloud,
        lambda: (
            fixture.health is False
            and fixture.status.get('state') == 'transform_unavailable'
            and fixture.status.get('transform_failures', 0) > 0
        ),
    )
    assert len(fixture.output_messages) == initial_output_count
    print('Missing-transform containment: PASSED')

    good_cloud = valid_cloud(fixture)
    assert fixture.publish_until(
        good_cloud,
        lambda: (
            fixture.health is True
            and fixture.status.get('state') == 'ready'
            and len(fixture.output_messages) > initial_output_count
        ),
    )
    print('Transform restoration: PASSED')


def wait_for_output_quiet(
    fixture,
    quiet_seconds=0.35,
    timeout_seconds=3.0,
):
    """Drain delayed outputs and wait for a quiet interval."""
    deadline = time.monotonic() + timeout_seconds
    last_count = len(fixture.output_messages)
    quiet_since = time.monotonic()

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(fixture, timeout_sec=0.05)

        current_count = len(fixture.output_messages)
        current_time = time.monotonic()

        if current_count != last_count:
            last_count = current_count
            quiet_since = current_time
        elif current_time - quiet_since >= quiet_seconds:
            return current_count

    raise AssertionError(
        'Timed out waiting for the filtered-output stream '
        'to become quiet.'
    )


def run_stale_recovery(fixture):
    """Validate stale detection without stale-cloud republication."""
    cloud = valid_cloud(fixture)
    assert fixture.publish_until(
        cloud,
        lambda: fixture.health is True and bool(fixture.output_messages),
    )

    assert fixture.spin_until(
        lambda: (
            fixture.health is False
            and fixture.status.get('state') == 'stale'
        ),
        timeout_sec=3.0,
    )
    settled_output_count = wait_for_output_quiet(fixture)

    verification_deadline = time.monotonic() + 0.50

    while (
        rclpy.ok()
        and time.monotonic() < verification_deadline
    ):
        rclpy.spin_once(fixture, timeout_sec=0.05)

    assert (
        len(fixture.output_messages)
        == settled_output_count
    )
    print('PointCloud2 staleness detection: PASSED')

    assert fixture.publish_until(
        valid_cloud(fixture),
        lambda: (
            fixture.health is True
            and fixture.status.get('state') == 'ready'
            and len(fixture.output_messages) > settled_output_count
        ),
    )
    print('PointCloud2 freshness restoration: PASSED')


def parse_arguments():
    """Parse the required positional runtime scenario."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'scenario',
        choices=('filter', 'bad-frame', 'stale-recovery'),
    )
    return parser.parse_args()


def main():
    """Run one bounded fixture scenario."""
    arguments = parse_arguments()
    rclpy.init()
    fixture = ObstacleCloudFixture()

    try:
        if arguments.scenario == 'filter':
            run_filter(fixture)
        elif arguments.scenario == 'bad-frame':
            run_bad_frame(fixture)
        else:
            run_stale_recovery(fixture)
    finally:
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
