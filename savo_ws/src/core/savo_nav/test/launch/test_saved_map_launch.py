# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate construction of the saved-map launch description."""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[2]

LAUNCH_PATH = (
    ROOT
    / 'launch'
    / 'saved_map_navigation.launch.py'
)


def load_launch_module():
    """Load the launch file as a Python module."""
    specification = importlib.util.spec_from_file_location(
        'savo_nav_saved_map_launch',
        LAUNCH_PATH,
    )

    assert specification is not None
    assert specification.loader is not None

    module = importlib.util.module_from_spec(
        specification
    )

    specification.loader.exec_module(module)

    return module


def test_launch_description_constructs():
    """Verify the complete saved-map node set, including startup readiness."""
    module = load_launch_module()

    description = module.generate_launch_description()

    assert isinstance(description, LaunchDescription)

    nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
    ]

    context = LaunchContext()

    def resolve(substitutions):
        return perform_substitutions(context, substitutions)

    identities = {
        (
            resolve(node.package),
            resolve(node.executable),
            resolve(node.node_name),
        )
        for node in nodes
    }
    assert identities == {
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_amcl', 'amcl', 'amcl'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        (
            'nav2_lifecycle_manager',
            'lifecycle_manager',
            'lifecycle_manager_localization',
        ),
        (
            'nav2_lifecycle_manager',
            'lifecycle_manager',
            'lifecycle_manager_navigation',
        ),
        ('savo_nav', 'goal_gateway_node', 'goal_gateway_node'),
        ('savo_nav', 'navigation_readiness_node', 'navigation_readiness_node'),
        (
            'savo_nav',
            'nav2_startup_readiness_node',
            'nav2_startup_readiness_node',
        ),
        (
            'savo_nav',
            'control_recovery_guard_node',
            'control_recovery_guard_node',
        ),
        ('savo_nav', 'goal_admission_gate_node', 'goal_admission_gate_node'),
    }


def test_launch_file_has_no_default_map_fixture():
    """Verify production launch requires a map."""
    source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument(\n                'map'" in source
    assert "default_value=''" in source

    assert 'phase5_empty_map.yaml' not in source


def test_launch_contains_goal_gateway():
    """Verify the public gateway is integrated."""
    source = LAUNCH_PATH.read_text(encoding='utf-8')

    assert "executable='goal_gateway_node'" in source
    assert "'start_goal_gateway'" in source
    assert "'active_map_id': map_id" in source


def test_launch_does_not_start_rviz():
    """Verify neither Pi owns RViz."""
    source = LAUNCH_PATH.read_text(
        encoding='utf-8'
    ).lower()

    assert "package='rviz2'" not in source
