# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate the guarded Phase 8 sensor integration contract."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SENSOR_CONTRACT = ROOT / 'config/costmap_sensors.yaml'
NAV2_CONFIG = ROOT / 'config/nav2/saved_map.yaml'
PROFILE = ROOT / 'config/profiles/saved_map.yaml'
RAW_TOPIC = '/camera/camera/depth/color/points'


def load_yaml(path):
    """Load a YAML mapping from one package path."""
    with path.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    assert isinstance(document, dict), path
    return document


def costmap_parameters(document, costmap_name):
    """Return one Nav2 costmap parameter mapping."""
    parameters = document[costmap_name][costmap_name][
        'ros__parameters'
    ]
    assert isinstance(parameters, dict)
    return parameters


def test_sensor_contract_records_guarded_hardware_stage():
    """Require the implemented producer and pending hardware gate."""
    assert SENSOR_CONTRACT.is_file()
    assert SENSOR_CONTRACT.stat().st_size > 0
    contract = load_yaml(SENSOR_CONTRACT)['costmap_sensors']
    assert (
        contract['integration_status']
        == 'ready_for_guarded_real_d435_validation'
    )


def test_lidar_contract_matches_verified_pipeline():
    """Require the verified hard LiDAR dependency."""
    lidar = load_yaml(SENSOR_CONTRACT)['costmap_sensors']['lidar']
    assert lidar['topic'] == '/scan'
    assert lidar['message_type'] == 'sensor_msgs/msg/LaserScan'
    assert lidar['required_for_navigation'] is True
    assert lidar['global_costmap'] is True
    assert lidar['local_costmap'] is True
    assert lidar['marking'] is True
    assert lidar['clearing'] is True
    assert lidar['inf_is_valid'] is True
    assert lidar['publisher_verified'] is True
    assert 'positive infinity' in lidar['infinity_evidence']


def test_realsense_contract_is_implemented_but_fail_closed():
    """Record the producer without enabling an unvalidated Nav2 layer."""
    realsense = load_yaml(SENSOR_CONTRACT)[
        'costmap_sensors'
    ]['realsense']
    assert (
        realsense['filtered_topic']
        == '/savo_perception/obstacles/points'
    )
    assert realsense['raw_topic'] == RAW_TOPIC
    assert (
        realsense['message_type']
        == 'sensor_msgs/msg/PointCloud2'
    )
    assert realsense['producer_implemented'] is True
    assert realsense['producer_contract_verified'] is True
    assert realsense['producer_hardware_validated'] is False
    assert realsense['clearing'] is False
    assert realsense['raw_topic_allowed_in_nav2'] is False
    assert 'Real D435' in realsense['evidence']


def test_raw_camera_cloud_is_absent_from_costmap_trees():
    """Ensure neither production costmap consumes the raw cloud."""
    nav2 = load_yaml(NAV2_CONFIG)
    for costmap_name in ('global_costmap', 'local_costmap'):
        parameters = costmap_parameters(nav2, costmap_name)
        serialized = yaml.safe_dump(parameters, sort_keys=True)
        assert RAW_TOPIC not in serialized


def test_existing_costmaps_retain_safe_baseline():
    """Require current LiDAR/static/inflation and robot geometry."""
    nav2 = load_yaml(NAV2_CONFIG)
    global_parameters = costmap_parameters(nav2, 'global_costmap')
    local_parameters = costmap_parameters(nav2, 'local_costmap')

    assert global_parameters['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]
    assert local_parameters['plugins'] == [
        'obstacle_layer',
        'inflation_layer',
    ]
    assert global_parameters['obstacle_layer']['scan'][
        'topic'
    ] == '/scan'
    assert local_parameters['obstacle_layer']['scan'][
        'topic'
    ] == '/scan'
    expected_footprint = (
        '[[0.165, 0.120], [0.165, -0.120], '
        '[-0.165, -0.120], [-0.165, 0.120]]'
    )
    assert global_parameters['footprint'] == expected_footprint
    assert local_parameters['footprint'] == expected_footprint
    assert global_parameters['footprint_padding'] > 0.0
    assert local_parameters['footprint_padding'] > 0.0


def test_ownership_boundaries_are_explicit():
    """Require all non-navigation authorities to remain false."""
    ownership = load_yaml(SENSOR_CONTRACT)[
        'costmap_sensors'
    ]['ownership']
    assert ownership == {
        'sensor_production': False,
        'control_mode': False,
        'recovery_execution': False,
        'velocity': False,
        'hardware': False,
    }


def test_profile_records_pending_real_d435_validation():
    """Keep the optional voxel layer disabled until hardware validation."""
    profile = load_yaml(PROFILE)
    assert (
        profile['phase8_costmap_sensor_integration_enabled']
        is False
    )
    assert (
        profile['phase8_blocked_reason']
        == 'real_d435_hardware_validation_pending'
    )
    assert profile['filtered_realsense_producer_verified'] is True
    assert profile['filtered_realsense_hardware_validated'] is False
    assert profile['raw_realsense_cloud_for_nav2'] is False
    assert profile['lidar_required_for_navigation'] is True
    assert profile['realsense_required_for_navigation'] is False
    assert profile['savo_nav_sensor_production_authority'] is False
    assert profile['savo_nav_velocity_authority'] is False
