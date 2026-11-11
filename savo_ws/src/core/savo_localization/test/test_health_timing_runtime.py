"""Verify timing telemetry and unchanged freshness decisions with real ROS callbacks."""

import json
import os
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix  # noqa: E402, I100, I201
from nav_msgs.msg import Odometry  # noqa: E402, I100, I201
from rclpy.qos import qos_profile_sensor_data  # noqa: E402, I100, I201
from std_msgs.msg import String  # noqa: E402, I100, I201


def test_health_timing_tracks_callbacks_and_completed_evaluations(tmp_path):
    """Pausing input must still cause stale health; completed timing identifies its cycle."""
    old_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
    rclpy.init()
    node = rclpy.create_node('health_timing_test')
    reports = []
    subscription = node.create_subscription(
        String, '/savo_localization/health', lambda msg: reports.append(json.loads(msg.data)), 10)
    publisher = node.create_publisher(Odometry, '/odometry/filtered', qos_profile_sensor_data)
    publishing = True

    def publish():
        if not publishing:
            return
        message = Odometry()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.orientation.w = 1.0
        publisher.publish(message)

    timer = node.create_timer(0.04, publish)
    executable = (Path(get_package_prefix('savo_localization')) /
                  'lib/savo_localization/localization_health_node')
    command = [str(executable), '--ros-args']
    for key in ['use_imu', 'use_wheel_odom', 'require_odom_to_base_tf', 'require_base_to_imu_tf']:
        command += ['-p', f'{key}:=false']
    command += ['-p', 'startup_grace_s:=0.1']
    log_path = tmp_path / 'health.log'

    def wait_for(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
            assert process.poll() is None, log_path.read_text()
        pytest.fail(f'timeout; reports={reports[-1:]}; log={log_path.read_text()}')

    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: reports and reports[-1]['ready'])
            report = reports[-1]
            timing = report['timing']
            assert timing['completed_evaluation']['sequence'] == report['evaluation_sequence'] - 1
            assert timing['completed_evaluation']['total']['wall_s'] > 0
            callback = timing['callbacks']['filtered_odom']
            assert callback['count'] >= 3
            assert callback['last_start_monotonic_ns'] <= report['evaluation_start_monotonic_ns']
            assert callback['header_stamp_ns'] > 0
            assert callback['header_age_at_callback_s'] >= 0
            assert timing['callbacks']['imu_state']['source_to_callback_s'] is None
            publishing = False
            wait_for(lambda: reports[-1]['state'] == 'STALE')
            assert not reports[-1]['ready']
            assert 'filtered_odom_stale' in reports[-1]['reasons']
            publishing = True
            wait_for(lambda: reports[-1]['ready'])
            assert reports[-1]['timing']['callbacks']['filtered_odom']['max_gap_s'] > 0.5
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_timer(timer)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
            if old_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = old_domain
