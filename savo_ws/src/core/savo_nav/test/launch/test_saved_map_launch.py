# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate construction of the saved-map launch description."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
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
    """Verify launch-description construction."""
    module = load_launch_module()

    description = module.generate_launch_description()

    assert isinstance(description, LaunchDescription)

    nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
    ]

    assert len(nodes) == 11


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
