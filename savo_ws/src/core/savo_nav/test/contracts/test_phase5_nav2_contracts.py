# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate the Phase 5 saved-map Nav2 contracts."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]

NAV2_CONFIG = (
    ROOT
    / 'config'
    / 'nav2'
    / 'saved_map.yaml'
)

PROFILE_CONFIG = (
    ROOT
    / 'config'
    / 'profiles'
    / 'saved_map.yaml'
)

LAUNCH_FILE = (
    ROOT
    / 'launch'
    / 'saved_map_navigation.launch.py'
)


def load_yaml(path):
    """Load and return a YAML document."""
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_phase5_files_exist():
    """Verify the Phase 5 files are present."""
    required = (
        NAV2_CONFIG,
        PROFILE_CONFIG,
        LAUNCH_FILE,
        (
            ROOT
            / 'test'
            / 'fixtures'
            / 'maps'
            / 'phase5_empty_map.yaml'
        ),
        (
            ROOT
            / 'test'
            / 'fixtures'
            / 'maps'
            / 'phase5_empty_map.pgm'
        ),
    )

    for path in required:
        assert path.is_file(), str(path)
        assert path.stat().st_size > 0, str(path)


def test_saved_map_tf_ownership():
    """Verify AMCL and localization frame ownership."""
    config = load_yaml(NAV2_CONFIG)

    amcl = config['amcl']['ros__parameters']

    assert amcl['global_frame_id'] == 'map'
    assert amcl['odom_frame_id'] == 'odom'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['tf_broadcast'] is True

    profile = load_yaml(PROFILE_CONFIG)[
        'saved_map_profile'
    ]

    assert profile['map_to_odom_authority'] == 'amcl'

    assert profile[
        'odom_to_base_footprint_authority'
    ] == 'savo_localization'


def test_amcl_is_configured_for_mecanum():
    """Verify the omnidirectional AMCL model."""
    config = load_yaml(NAV2_CONFIG)

    amcl = config['amcl']['ros__parameters']

    assert amcl['robot_model_type'] == (
        'nav2_amcl::OmniMotionModel'
    )

    assert amcl['scan_topic'] == '/scan'


def test_controller_is_holonomic_dwb():
    """Verify DWB can command x, y and yaw."""
    config = load_yaml(NAV2_CONFIG)

    controller = config[
        'controller_server'
    ]['ros__parameters']

    follow_path = controller['FollowPath']

    assert follow_path['plugin'] == (
        'dwb_core::DWBLocalPlanner'
    )

    assert follow_path['max_vel_x'] > 0.0
    assert follow_path['max_vel_y'] > 0.0
    assert follow_path['max_vel_theta'] > 0.0

    assert follow_path['min_vel_x'] < 0.0
    assert follow_path['min_vel_y'] < 0.0

    assert controller['odom_topic'] == (
        '/odometry/filtered'
    )

    assert controller['enable_stamped_cmd_vel'] is False


def test_velocity_output_is_only_cmd_vel_nav():
    """Verify Nav2 velocity producers are remapped."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "('cmd_vel', '/cmd_vel_nav')" in source

    assert source.count(
        'nav_velocity_remappings'
    ) >= 3

    forbidden_targets = (
        "('cmd_vel', '/cmd_vel')",
        "('cmd_vel', '/cmd_vel_safe')",
        "('cmd_vel', '/cmd_vel_recovery')",
    )

    for target in forbidden_targets:
        assert target not in source


def test_lifecycle_autostart_is_disabled():
    """Verify launch and profiles default to no activation."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'autostart'" in source
    assert "default_value='false'" in source

    config = load_yaml(NAV2_CONFIG)

    localization = config[
        'lifecycle_manager_localization'
    ]['ros__parameters']

    navigation = config[
        'lifecycle_manager_navigation'
    ]['ros__parameters']

    assert localization['autostart'] is False
    assert navigation['autostart'] is False

    profile = load_yaml(PROFILE_CONFIG)[
        'saved_map_profile'
    ]

    assert profile['lifecycle_autostart'] is False
    assert profile['hardware_test_enabled'] is False


def test_baseline_costmaps_are_lidar_only():
    """Verify Phase 5 does not claim Phase 8 work."""
    config = load_yaml(NAV2_CONFIG)

    global_costmap = config[
        'global_costmap'
    ]['global_costmap']['ros__parameters']

    local_costmap = config[
        'local_costmap'
    ]['local_costmap']['ros__parameters']

    assert global_costmap['plugins'] == [
        'static_layer',
        'obstacle_layer',
        'inflation_layer',
    ]

    assert local_costmap['plugins'] == [
        'obstacle_layer',
        'inflation_layer',
    ]

    assert global_costmap[
        'obstacle_layer'
    ]['observation_sources'] == ['scan']

    assert local_costmap[
        'obstacle_layer'
    ]['observation_sources'] == ['scan']

    serialized = NAV2_CONFIG.read_text(
        encoding='utf-8'
    ).lower()

    assert 'voxellayer' not in serialized
    assert 'pointcloud2' not in serialized

    profile = load_yaml(PROFILE_CONFIG)[
        'saved_map_profile'
    ]

    assert profile['voxel_layer_enabled'] is False

    assert profile[
        'filtered_pointcloud_enabled'
    ] is False


def test_required_nav2_servers_are_launched():
    """Verify the expected Nav2 nodes are present."""
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    required = (
        "package='nav2_map_server'",
        "package='nav2_amcl'",
        "package='nav2_controller'",
        "package='nav2_planner'",
        "package='nav2_behaviors'",
        "package='nav2_bt_navigator'",
        "package='nav2_waypoint_follower'",
        "package='nav2_lifecycle_manager'",
    )

    for contract in required:
        assert contract in source


def test_launch_does_not_start_rviz():
    """Verify neither Pi is assigned RViz ownership."""
    source = LAUNCH_FILE.read_text(
        encoding='utf-8'
    ).lower()

    assert 'rviz2' not in source
    assert "package='rviz2'" not in source


def test_fixture_map_is_valid():
    """Verify the dry-run map metadata."""
    fixture = load_yaml(
        ROOT
        / 'test'
        / 'fixtures'
        / 'maps'
        / 'phase5_empty_map.yaml'
    )

    assert fixture['image'] == 'phase5_empty_map.pgm'
    assert fixture['resolution'] == 0.05
    assert fixture['mode'] == 'trinary'
    assert fixture['occupied_thresh'] > fixture['free_thresh']
