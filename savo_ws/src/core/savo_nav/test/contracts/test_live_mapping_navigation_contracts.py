# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate guarded Nav2 bringup while SLAM Toolbox owns map-to-odom."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'launch/live_mapping_navigation.launch.py'
PROFILE = ROOT / 'config/profiles/live_mapping_real_robot.yaml'
READINESS = ROOT / 'config/readiness.yaml'
STARTUP_READINESS = ROOT / 'config/startup_readiness.yaml'
PRODUCTION_NAV2 = ROOT / 'config/nav2_live_mapping.yaml'
DEGRADED_NAV2 = (
    ROOT / 'config/nav2_live_mapping_core_lidar_degraded.yaml'
)


def _flatten(document, prefix=''):
    """Return leaf values keyed by their dotted YAML path."""
    flattened = {}
    if isinstance(document, dict):
        for key, value in document.items():
            child = f'{prefix}.{key}' if prefix else str(key)
            flattened.update(_flatten(value, child))
    elif isinstance(document, list):
        for index, value in enumerate(document):
            child = f'{prefix}[{index}]'
            flattened.update(_flatten(value, child))
    else:
        flattened[prefix] = document
    return flattened


def test_live_mapping_launch_is_installed_source():
    """Require the package-owned live mapping launch file."""
    assert LAUNCH.is_file()
    text = LAUNCH.read_text(encoding='utf-8')
    assert "package='nav2_controller'" in text
    assert "package='nav2_planner'" in text
    assert "package='nav2_bt_navigator'" in text
    assert "package='nav2_lifecycle_manager'" in text


def test_live_mapping_launch_excludes_saved_map_localization():
    """Prevent AMCL and map_server from competing with SLAM Toolbox."""
    text = LAUNCH.read_text(encoding='utf-8')
    assert "package='nav2_amcl'" not in text
    assert "package='nav2_map_server'" not in text
    assert "'map_mode': 'live_mapping'" in text
    assert "'autostart',\n                default_value='true'" in text


def test_live_mapping_launch_preserves_guarded_velocity_chain():
    """Require Nav2 output and public-action guard layers."""
    text = LAUNCH.read_text(encoding='utf-8')
    assert "('cmd_vel', '/cmd_vel_nav')" in text
    assert "executable='goal_gateway_node'" in text
    assert "executable='goal_admission_gate_node'" in text
    assert "executable='control_recovery_guard_node'" in text
    assert '/savo_nav/_internal/exploration/navigate_to_pose' in text


def test_live_mapping_separates_startup_evidence_from_runtime_admission():
    """Nav2 can prove lifecycle/action health while control remains STOP."""
    launch = LAUNCH.read_text(encoding='utf-8')
    startup = yaml.safe_load(STARTUP_READINESS.read_text(encoding='utf-8'))
    runtime = yaml.safe_load(READINESS.read_text(encoding='utf-8'))

    assert "executable='nav2_startup_readiness_node'" in launch
    assert 'condition=IfCondition(start_startup_readiness)' in launch
    assert startup['nav2_startup_readiness_node']['ros__parameters'][
        'required_lifecycle_nodes'
    ] == [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    runtime_parameters = runtime['navigation_readiness_node'][
        'ros__parameters'
    ]
    assert runtime_parameters['require_control_mode'] is True
    assert 'control_mode' not in startup[
        'nav2_startup_readiness_node'
    ]['ros__parameters']


def test_lidar_only_readiness_matches_active_costmap_baseline():
    """Do not block the LiDAR-only first hardware test on D435 data."""
    document = yaml.safe_load(READINESS.read_text(encoding='utf-8'))
    parameters = document['navigation_readiness_node']['ros__parameters']
    assert parameters['require_pointcloud'] is False
    assert parameters['require_control_mode'] is True
    assert parameters['require_safety_state'] is True


def test_live_mapping_profile_records_tf_authority():
    """Freeze the no-AMCL live mapping deployment contract."""
    document = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    profile = document['live_mapping_real_robot_profile']
    assert profile['mode'] == 'live_mapping'
    assert profile['map_to_odom_authority'] == 'slam_toolbox'
    assert profile['odom_to_base_footprint_authority'] == 'savo_localization'
    assert profile['lifecycle_autostart'] is True
    assert profile['hardware_test_enabled'] is True
    assert profile['realsense_required_for_navigation'] is False
    assert profile['direct_velocity_authority'] is False


def test_degraded_nav2_changes_only_approved_controller_tuning():
    """Keep production fixed while adding bounded degraded holonomic tuning."""
    production_document = yaml.safe_load(
        PRODUCTION_NAV2.read_text(encoding='utf-8')
    )
    degraded_document = yaml.safe_load(
        DEGRADED_NAV2.read_text(encoding='utf-8')
    )
    production = _flatten(production_document)
    degraded = _flatten(degraded_document)
    allowed_differences = {
        'controller_server.ros__parameters.FollowPath.max_vel_theta',
        'controller_server.ros__parameters.FollowPath.acc_lim_theta',
        'controller_server.ros__parameters.FollowPath.decel_lim_theta',
        'controller_server.ros__parameters.FollowPath.critics[7]',
        'controller_server.ros__parameters.FollowPath.Twirling.scale',
    }
    actual_differences = {
        key
        for key in production.keys() | degraded.keys()
        if production.get(key) != degraded.get(key)
    }

    assert actual_differences == allowed_differences
    assert production[
        'controller_server.ros__parameters.FollowPath.max_vel_theta'
    ] == 0.55
    assert production[
        'controller_server.ros__parameters.FollowPath.acc_lim_theta'
    ] == 1.00
    assert production[
        'controller_server.ros__parameters.FollowPath.decel_lim_theta'
    ] == -1.00
    assert degraded[
        'controller_server.ros__parameters.FollowPath.max_vel_theta'
    ] == 0.30
    assert degraded[
        'controller_server.ros__parameters.FollowPath.acc_lim_theta'
    ] == 0.50
    assert degraded[
        'controller_server.ros__parameters.FollowPath.decel_lim_theta'
    ] == -0.50
    assert degraded[
        'controller_server.ros__parameters.FollowPath.max_vel_theta'
    ] > 0.0

    production_follow_path = production_document[
        'controller_server'
    ]['ros__parameters']['FollowPath']
    degraded_follow_path = degraded_document[
        'controller_server'
    ]['ros__parameters']['FollowPath']

    assert 'Twirling' not in production_follow_path['critics']
    assert degraded_follow_path['critics'] == [
        *production_follow_path['critics'],
        'Twirling',
    ]
    assert degraded_follow_path['Twirling.scale'] == 10.0


def test_degraded_holonomic_tuning_discourages_twirling_without_forcing_motion():
    """Penalize travel-time spin while preserving STOP and normal turning."""
    document = yaml.safe_load(DEGRADED_NAV2.read_text(encoding='utf-8'))
    controller = document['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    goal_checker = controller['general_goal_checker']

    assert follow_path['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert follow_path['min_speed_xy'] == 0.0
    assert follow_path['min_speed_theta'] == 0.0
    assert follow_path['min_vel_x'] < 0.0 < follow_path['max_vel_x']
    assert follow_path['min_vel_y'] < 0.0 < follow_path['max_vel_y']
    assert follow_path['max_vel_theta'] == 0.30
    assert follow_path['acc_lim_theta'] == 0.50
    assert follow_path['decel_lim_theta'] == -0.50
    assert follow_path['critics'][-1] == 'Twirling'
    assert follow_path['Twirling.scale'] == 10.0
    assert 'RotateToGoal' in follow_path['critics']
    assert follow_path['RotateToGoal.scale'] > 0.0
    assert goal_checker['yaw_goal_tolerance'] > 0.0
