# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate guarded Nav2 bringup while SLAM Toolbox owns map-to-odom."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'launch/live_mapping_navigation.launch.py'
PROFILE = ROOT / 'config/profiles/live_mapping_real_robot.yaml'
READINESS = ROOT / 'config/readiness.yaml'


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
