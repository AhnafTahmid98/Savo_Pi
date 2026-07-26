# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_readiness_node_is_hardware_independent():
    source = (
        ROOT
        / 'src/nodes/navigation_readiness_node.cpp'
    ).read_text(encoding='utf-8').lower()

    forbidden_hardware_terms = (
        '/dev/',
        'gpiochip',
        'wiringpi',
        'pigpio',
        'pca9685',
        'tca9548',
        'vl53l1x',
        'hc-sr04',
        'i2c-dev',
        'librealsense',
        'rplidar_driver',
    )

    for term in forbidden_hardware_terms:
        assert term not in source


def test_readiness_node_does_not_publish_velocity():
    source = (
        ROOT
        / 'src/nodes/navigation_readiness_node.cpp'
    ).read_text(encoding='utf-8')

    assert '"/cmd_vel"' not in source
    assert '"/cmd_vel_safe"' not in source
    assert '"/cmd_vel_recovery"' not in source
    assert '"/cmd_vel_nav"' not in source


def test_readiness_node_subscribes_to_package_outputs():
    source = (
        ROOT
        / 'src/nodes/navigation_readiness_node.cpp'
    ).read_text(encoding='utf-8')

    required_message_types = (
        'nav_msgs::msg::OccupancyGrid',
        'nav_msgs::msg::Odometry',
        'sensor_msgs::msg::LaserScan',
        'sensor_msgs::msg::PointCloud2',
        'tf2_msgs::msg::TFMessage',
        'std_msgs::msg::Bool',
        'std_msgs::msg::Float32',
        'std_msgs::msg::String',
    )

    for message_type in required_message_types:
        assert message_type in source


def test_phase3_readiness_configuration():
    path = ROOT / 'config/readiness.yaml'

    with path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    parameters = config[
        'navigation_readiness_node'
    ]['ros__parameters']

    assert parameters['require_pointcloud'] is True
    assert parameters['require_control_mode'] is True
    assert parameters['require_safety_state'] is True

    assert parameters['map_topic'] == '/map'

    assert parameters['odometry_topic'] == (
        '/odometry/filtered'
    )

    assert parameters['scan_topic'] == '/scan'

    assert parameters['pointcloud_topic'] == (
        '/savo_perception/obstacles/points'
    )

    assert parameters['control_mode_topic'] == (
        '/savo_control/mode_state'
    )

    assert parameters['safety_stop_topic'] == (
        '/safety/stop'
    )

    assert parameters['nav2_action_name'] == (
        '/navigate_to_pose'
    )


def test_target_tf_chain_is_preserved():
    source = (
        ROOT
        / 'src/nodes/navigation_readiness_node.cpp'
    ).read_text(encoding='utf-8')

    assert 'frames::kMap' in source
    assert 'frames::kOdom' in source
    assert 'frames::kBaseFootprint' in source
    assert 'frames::kBaseLink' in source
