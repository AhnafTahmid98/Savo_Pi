# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate construction of the saved-map launch description."""

from collections import Counter
import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.utilities import normalize_to_list_of_substitutions
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
    """Verify the complete saved-map node set without a startup gate."""
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
        return perform_substitutions(
            context,
            normalize_to_list_of_substitutions(substitutions),
        )

    identities = Counter(
        (
            resolve(node.node_package),
            resolve(node.node_executable),
        )
        for node in nodes
    )
    expected_identities = Counter(
        {
            ('nav2_map_server', 'map_server'): 1,
            ('nav2_amcl', 'amcl'): 1,
            ('nav2_controller', 'controller_server'): 1,
            ('nav2_planner', 'planner_server'): 1,
            ('nav2_behaviors', 'behavior_server'): 1,
            ('nav2_bt_navigator', 'bt_navigator'): 1,
            ('nav2_waypoint_follower', 'waypoint_follower'): 1,
            ('nav2_lifecycle_manager', 'lifecycle_manager'): 2,
            ('savo_nav', 'goal_gateway_node'): 1,
            ('savo_nav', 'navigation_readiness_node'): 1,
            ('savo_nav', 'control_recovery_guard_node'): 1,
            ('savo_nav', 'goal_admission_gate_node'): 1,
        }
    )

    assert sum(identities.values()) == 13
    assert identities == expected_identities


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
