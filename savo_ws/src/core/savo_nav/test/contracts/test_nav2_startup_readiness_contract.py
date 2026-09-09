# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_startup_readiness_is_diagnostic_only():
    """The legacy probe remains available but is not a production gate."""
    launch = (ROOT / 'launch/live_mapping_navigation.launch.py').read_text()
    assert "executable='nav2_startup_readiness_node'" not in launch
    assert 'navigation_readiness_node' in launch
    source = (ROOT / 'src/nodes/nav2_startup_readiness_node.cpp').read_text()
    assert 'async_send_request' in source
    assert 'action_server_is_ready' in source
    assert 'async_send_goal' not in source
    assert 'map_required\\\":false' in source


def test_startup_tf_uses_real_localization_contract():
    """Startup TF evidence follows the production localization ownership."""
    config = (ROOT / 'config/startup_readiness.yaml').read_text()
    assert 'odom_frame: odom' in config
    assert 'base_frame: base_footprint' in config


def test_startup_gate_requires_lifecycle_and_action_evidence():
    """Startup requires every configured lifecycle and action endpoint."""
    config = (ROOT / 'config/startup_readiness.yaml').read_text()
    for node in (
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ):
        assert f'- {node}' in config
    assert 'navigate_to_pose_action: /navigate_to_pose' in config
    assert 'follow_waypoints_action: /follow_waypoints' in config
