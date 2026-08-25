# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate Robot SAVO package-owned guarded behavior trees."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
GUARDED_TREE = ROOT / 'behavior_trees/navigate_to_pose.xml'
EXPLORATION_TREE = ROOT / 'behavior_trees/exploration_navigation.xml'
SAVED_LAUNCH = ROOT / 'launch/saved_map_navigation.launch.py'
LIVE_LAUNCH = ROOT / 'launch/live_mapping_navigation.launch.py'
GATEWAY_SOURCE = ROOT / 'src/nodes/goal_gateway_node.cpp'
GATEWAY_CONFIG = ROOT / 'config/goal_gateway.yaml'
NAV2_CONFIG = ROOT / 'config/nav2_saved_map.yaml'


def _tags(path):
    """Return all XML element tag names in one tree."""
    root = ET.parse(path).getroot()
    return {element.tag for element in root.iter()}


def test_package_owns_normal_and_exploration_trees():
    """Require both installed behavior-tree definitions."""
    for path in (GUARDED_TREE, EXPLORATION_TREE):
        assert path.is_file(), path
        assert path.stat().st_size > 0, path
        assert ET.parse(path).getroot().tag == 'root'


def test_trees_replan_and_use_only_non_motion_recovery():
    """Keep motion-producing recovery under savo_control ownership."""
    forbidden = {'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop'}
    for path in (GUARDED_TREE, EXPLORATION_TREE):
        tags = _tags(path)
        assert {'ComputePathToPose', 'FollowPath', 'ClearEntireCostmap'} <= tags
        assert 'Wait' in tags
        assert forbidden.isdisjoint(tags)


def test_nav2_behavior_server_exposes_wait_only():
    """Prevent built-in Nav2 motion behaviors from becoming executable."""
    document = yaml.safe_load(NAV2_CONFIG.read_text(encoding='utf-8'))
    parameters = document['behavior_server']['ros__parameters']
    assert parameters['behavior_plugins'] == ['wait']
    assert parameters['wait']['plugin'] == 'nav2_behaviors::Wait'
    for key in ('spin', 'backup', 'drive_on_heading'):
        assert key not in parameters


def test_launches_install_package_trees_into_bt_navigator_and_gateway():
    """Require deterministic tree selection for each guarded goal source."""
    for launch in (SAVED_LAUNCH, LIVE_LAUNCH):
        source = launch.read_text(encoding='utf-8')
        assert 'navigate_to_pose.xml' in source
        assert 'exploration_navigation.xml' in source
        assert 'default_nav_to_pose_bt_xml' in source
        assert "'navigation_behavior_tree'" in source
        assert "'exploration_behavior_tree'" in source


def test_gateway_rejects_external_tree_override_and_selects_by_source():
    """Keep callers from bypassing Robot SAVO's guarded trees."""
    config = yaml.safe_load(GATEWAY_CONFIG.read_text(encoding='utf-8'))
    parameters = config['goal_gateway_node']['ros__parameters']
    assert parameters['allow_behavior_tree_override'] is False

    source = GATEWAY_SOURCE.read_text(encoding='utf-8')
    assert 'goal_rejected_behavior_tree_override' in source
    assert 'navigation_behavior_tree_' in source
    assert 'exploration_behavior_tree_' in source
    assert 'nav2_goal.behavior_tree' in source


def test_requested_production_layout_is_canonical():
    """Keep the exact pre-bringup file contract stable."""
    required = {
        ROOT / 'behavior_trees/navigate_to_pose.xml',
        ROOT / 'behavior_trees/exploration_navigation.xml',
        ROOT / 'config/nav2_saved_map.yaml',
        ROOT / 'config/nav2_live_mapping.yaml',
        ROOT / 'config/nav2_live_mapping_voxel.yaml',
        ROOT / 'config/nav2_saved_map_voxel.yaml',
    }
    for path in required:
        assert path.is_file(), path
        assert path.stat().st_size > 0, path

    assert not (ROOT / 'config/nav2').exists()
