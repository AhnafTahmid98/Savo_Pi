"""Exercise dependency confirmation and recovery ordering through real ROS callbacks."""

import json
import os
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix  # noqa: E402, I100, I201
from rclpy.qos import qos_profile_sensor_data  # noqa: E402, I100, I201
from std_msgs.msg import Bool, String  # noqa: E402, I100, I201


@pytest.mark.parametrize('fault', ['confirmed', 'safety_stop'])
def test_startup_trace_preserves_fail_closed_ordering(fault, tmp_path):
    """Cached observations latch; safety has no grace; late recovery cannot rearm."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(140 + os.getpid() % 40))
    old_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    rclpy.init()
    node = rclpy.create_node('startup_trace_test')
    traces = []
    subscription = node.create_subscription(
        String, '/savo_bringup/core/startup_trace',
        lambda msg: traces.append(json.loads(msg.data)), qos_profile_sensor_data)
    topics = {
        '/savo_base/base_state': 'ready',
        '/savo_control/mode_state': 'STOP',
        '/savo_perception/safety_state': 'ready',
        '/savo_localization/imu_state': 'ready',
        '/savo_localization/wheel_odom_state': 'ready',
    }
    publishers = {topic: node.create_publisher(String, topic, 10) for topic in topics}
    health = node.create_publisher(String, '/savo_localization/health', 10)
    stop = node.create_publisher(Bool, '/safety/stop', 10)
    stopped = False

    def publish_dependencies():
        for topic, value in topics.items():
            publishers[topic].publish(String(data=value))
        stop.publish(Bool(data=stopped))

    timer = node.create_timer(0.05, publish_dependencies)
    executable = (Path(get_package_prefix('savo_bringup')) /
                  'lib/savo_bringup/bringup_readiness_node')
    parameters = {
        'startup.enabled': True, 'robot_mode': 'autonomous_mapping',
        'require_geometry': False, 'require_base': True,
        'require_control': True, 'require_safety': True,
        'require_localization': True, 'require_head': True,
        'startup.observation_freshness_s': 20.0,
        'startup.core.transient_ekf_rate_recovery_s': 12.0,
    }
    for stage in ['infrastructure', 'hardware', 'motion_safety',
                  'sensor_stabilization', 'localization']:
        parameters[f'startup.core.{stage}.stable_ready_s'] = 0.0
    parameters['startup.core.head.startup_timeout_s'] = 30.0
    command = [str(executable), '--ros-args']
    for key, value in parameters.items():
        command += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']

    def wait_for(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
            assert process.poll() is None, log_path.read_text()
        pytest.fail(f'timeout; last trace={traces[-1:]}; log={log_path.read_text()}')

    log_path = tmp_path / 'coordinator.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: health.get_subscription_count() > 0)
            health.publish(String(data='ready'))
            wait_for(lambda: any(t['stage'] == 'head' for t in traces))
            start = len(traces)
            if fault == 'safety_stop':
                stopped = True
            else:
                health.publish(String(data='state=stale;ready=false'))
            wait_for(lambda: any(t['consecutive_loss_samples'] == 1 for t in traces[start:]))
            first = next(t for t in traces[start:] if t['consecutive_loss_samples'] == 1)
            assert first['blocking'] and not first['decision_ready']
            assert not first['decision_failed']
            assert not first['recovery_holding']
            wait_for(lambda: any(t['confirmed_loss'] for t in traces[start:]))
            confirmed = next(t for t in traces[start:] if t['confirmed_loss'])
            assert confirmed['decision_failed']
            assert confirmed['consecutive_loss_samples'] == 2
            if fault == 'confirmed':
                assert (first['observations']['localization']['receive_sequence'] ==
                        confirmed['observations']['localization']['receive_sequence'])
            stopped = False
            health.publish(String(data='ready'))
            wait_for(lambda: any(t['dependencies_recovered'] for t in traces[start:]))
            recovered = next(t for t in traces[start:] if t['dependencies_recovered'])
            assert recovered['decision_failed']
            assert not recovered['decision_ready']
            assert recovered['first_failure_evaluation'] == confirmed['evaluation_sequence']
            receipt = recovered['observations']['localization']
            assert receipt['receive_event_order'] < recovered['evaluation_event_order']
            assert receipt['receive_monotonic_ns'] <= recovered['evaluation_start_monotonic_ns']
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
