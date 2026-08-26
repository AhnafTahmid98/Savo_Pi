# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate the guarded Phase 8 sensor integration contract."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SENSOR_CONTRACT = ROOT / 'config/costmap_sensors.yaml'
NAV2_CONFIG = ROOT / 'config/nav2_saved_map.yaml'
LIVE_CONFIG = ROOT / 'config/nav2_live_mapping.yaml'
VOXEL_CONFIG = ROOT / 'config/nav2_saved_map_voxel.yaml'
LIVE_VOXEL_CONFIG = ROOT / 'config/nav2_live_mapping_voxel.yaml'
VOXEL_READINESS = ROOT / 'config/readiness_realsense_voxel.yaml'
PROFILE = ROOT / 'config/profiles/saved_map.yaml'
VOXEL_PROFILE = ROOT / 'config/profiles/saved_map_realsense_voxel.yaml'
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
    """Record stationary acceptance without opening production gates."""
    assert SENSOR_CONTRACT.is_file()
    assert SENSOR_CONTRACT.stat().st_size > 0
    contract = load_yaml(SENSOR_CONTRACT)['costmap_sensors']
    assert (
        contract['integration_status']
        == 'stationary_d435_voxel_acceptance_complete_production_gated'
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
    assert realsense['producer_stationary_hardware_accepted'] is True
    assert realsense['stationary_voxel_hardware_accepted'] is True
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
    assert 'robot_radius' not in global_parameters
    assert 'robot_radius' not in local_parameters
    assert global_parameters['footprint'].startswith('[[0.165, 0.120]')
    assert local_parameters['footprint'] == global_parameters['footprint']
    assert global_parameters['footprint_padding'] == 0.02
    assert local_parameters['footprint_padding'] == 0.02


def test_observation_sources_use_ros_string_parameter_type():
    """Require scalar source IDs in every production costmap profile."""
    profiles = (
        (LIVE_CONFIG, False),
        (NAV2_CONFIG, False),
        (LIVE_VOXEL_CONFIG, True),
        (VOXEL_CONFIG, True),
    )

    for config_path, has_voxel_layer in profiles:
        nav2 = load_yaml(config_path)

        for costmap_name in ('global_costmap', 'local_costmap'):
            costmap = costmap_parameters(nav2, costmap_name)
            obstacle = costmap['obstacle_layer']
            sources = obstacle['observation_sources']
            assert type(sources) is str
            assert not isinstance(sources, list)
            assert sources == 'scan'
            assert isinstance(obstacle['scan'], dict)

        if has_voxel_layer:
            voxel = costmap_parameters(nav2, 'local_costmap')[
                'voxel_layer'
            ]
            sources = voxel['observation_sources']
            assert type(sources) is str
            assert not isinstance(sources, list)
            assert sources == 'filtered_obstacles'
            assert isinstance(voxel['filtered_obstacles'], dict)


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


def test_profile_records_scoped_real_d435_validation():
    """Record stationary acceptance while keeping full validation gated."""
    profile = load_yaml(PROFILE)['saved_map_profile']
    validation = profile['validation']
    ownership = profile['ownership']
    assert validation['voxel_profile_source_complete'] is True
    assert validation['voxel_profile_stationary_hardware_accepted'] is True
    assert validation['voxel_profile_activation_enabled'] is False
    assert (
        validation['voxel_profile_activation_gate']
        == 'complete_floor_scene_motion_and_stability_validation_pending'
    )
    assert validation['filtered_realsense_producer_verified'] is True
    assert validation[
        'filtered_realsense_stationary_hardware_accepted'
    ] is True
    assert validation['filtered_realsense_hardware_validated'] is False
    assert validation['raw_realsense_cloud_for_nav2'] is False
    assert profile['footprint_source'] == (
        'savo_nav_conservative_collision_envelope'
    )
    assert profile['footprint_status'] == (
        'provisional_pending_full_collision_survey'
    )
    assert profile['lidar_required_for_navigation'] is True
    assert profile['realsense_required_for_navigation'] is False
    assert ownership['savo_nav_sensor_production_authority'] is False
    assert ownership['savo_nav_velocity_authority'] is False


def test_live_and_saved_voxel_profiles_preserve_sensor_roles():
    """Keep LiDAR clearing and filtered-D435 local-only marking."""
    for config_path in (LIVE_VOXEL_CONFIG, VOXEL_CONFIG):
        nav2 = load_yaml(config_path)
        local = costmap_parameters(nav2, 'local_costmap')
        global_costmap = costmap_parameters(nav2, 'global_costmap')

        assert local['plugins'] == [
            'obstacle_layer',
            'voxel_layer',
            'inflation_layer',
        ]
        assert global_costmap['obstacle_layer']['scan']['topic'] == '/scan'
        assert global_costmap['obstacle_layer']['scan']['marking'] is True
        assert global_costmap['obstacle_layer']['scan']['clearing'] is True
        assert local['obstacle_layer']['scan']['topic'] == '/scan'
        assert local['obstacle_layer']['scan']['marking'] is True
        assert local['obstacle_layer']['scan']['clearing'] is True

        voxel = local['voxel_layer']
        assert (
            voxel['plugin']
            == 'nav2_costmap_2d/NonPersistentVoxelLayer'
        )
        assert voxel['publish_voxel_map'] is False
        assert voxel['observation_sources'] == 'filtered_obstacles'
        assert type(voxel['observation_sources']) is str
        source = voxel['filtered_obstacles']
        assert source['topic'] == '/savo_perception/obstacles/points'
        assert source['data_type'] == 'PointCloud2'
        assert source['marking'] is True
        assert source['clearing'] is False
        assert source['observation_persistence'] == 1.0
        assert source['expected_update_rate'] == 0.0
        assert RAW_TOPIC not in yaml.safe_dump(nav2)
        assert 'voxel_layer' not in global_costmap['plugins']


def test_voxel_grid_covers_the_accepted_obstacle_height():
    """Fit the full D435 height contract within Nav2's 16-bit grid."""
    for config_path in (LIVE_VOXEL_CONFIG, VOXEL_CONFIG):
        local = costmap_parameters(
            load_yaml(config_path), 'local_costmap'
        )
        voxel = local['voxel_layer']
        source = voxel['filtered_obstacles']
        assert voxel['z_voxels'] <= 16
        coverage = (
            voxel['origin_z']
            + voxel['z_resolution'] * voxel['z_voxels']
        )
        assert coverage >= source['max_obstacle_height']


def test_voxel_readiness_keeps_filtered_cloud_optional():
    """Missing or stale filtered D435 data must not block navigation."""
    params = load_yaml(VOXEL_READINESS)[
        'navigation_readiness_node'
    ]['ros__parameters']
    assert params['require_pointcloud'] is False
    assert (
        params['pointcloud_topic']
        == '/savo_perception/obstacles/points'
    )
    assert params['pointcloud_timeout_seconds'] > 0.0


def test_optional_voxel_runtime_dependency_is_declared():
    """Install the released Jazzy plugin used for stale-safe helper marks."""
    package_xml = (ROOT / 'package.xml').read_text(encoding='utf-8')
    dependency = '<exec_depend>nonpersistent_voxel_layer</exec_depend>'
    assert dependency in package_xml


def test_guarded_voxel_companion_profile_is_source_complete():
    """Keep the saved-map voxel activation contract hardware-gated."""
    profile = load_yaml(VOXEL_PROFILE)[
        'saved_map_realsense_voxel_profile'
    ]
    assert profile['software_completion'] == 'complete'
    assert profile['stationary_hardware_acceptance_completed'] is True
    assert profile['production_validation_complete'] is False
    assert profile['activation_enabled'] is False
    assert profile['realsense_required_for_navigation'] is False
    assert profile['lidar_required_for_navigation'] is True


def test_readme_records_stationary_voxel_acceptance_and_open_gates():
    """Document scoped hardware evidence without claiming production lock."""
    readme = (ROOT / 'README.md').read_text(encoding='utf-8')
    assert '2026-08-26 stationary D435 voxel hardware acceptance' in readme
    assert 'stationary on a table' in readme
    assert 'nonpersistent stale mark expiration' in readme
    assert 'This is not D435 ray clearing' in readme
    assert 'LiDAR remains the required authoritative clearing source' in readme
    assert '`d435_voxel_validated=false`' in readme
    assert '`activation_enabled=false`' in readme
    assert 'does not complete floor-level' in readme
    assert 'open RealSense startup/USB' in readme
