#!/usr/bin/env python3

"""Static contracts for Phase 4L-B3F Coverage execution handoff."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MAPPER = PACKAGE_ROOT / 'src/nodes/coverage_mapper_node.cpp'
HANDOFF = PACKAGE_ROOT / 'src/nodes/coverage_execution_handoff_node.cpp'
HEADER = PACKAGE_ROOT / 'include/savo_mapping/coverage_execution_handoff.hpp'
CMAKE = PACKAGE_ROOT / 'CMakeLists.txt'
CONFIG = PACKAGE_ROOT / 'config/coverage_execution_handoff.yaml'
LAUNCH = PACKAGE_ROOT / 'launch/coverage_execution_handoff.launch.xml'
COVERAGE_LAUNCH = PACKAGE_ROOT / 'launch/coverage_mapping.launch.xml'


def read(path):
    """Return UTF-8 source text."""
    return path.read_text(encoding='utf-8')


def test_mapper_remains_planner_only():
    """Publishing a planned path cannot dispatch an action goal."""
    source = read(MAPPER)
    assert 'async_send_goal' not in source
    assert 'create_client<' not in source
    assert 'ExecuteCoveragePath' not in source


def test_handoff_requires_explicit_approval():
    """Only the dedicated handoff owns the Coverage action client."""
    source = read(HANDOFF)
    assert 'coverage_execution_handoff_node' in source
    assert 'handle_approve' in source
    assert 'async_send_goal' in source
    assert 'approve_service_name_' in source
    assert 'handle_plan' in source
    assert 'send_active_goal' not in source.split('handle_plan(', 1)[1].split(
        'handle_approve(', 1
    )[0]


def test_handoff_uses_guarded_savo_nav_action():
    """The mapping node must never call Nav2 FollowPath directly."""
    header = read(HEADER)
    source = read(HANDOFF)
    assert '/savo_nav/coverage/execute_path' in header
    assert 'savo_msgs::action::ExecuteCoveragePath' in source
    assert 'nav2_msgs::action::FollowPath' not in source
    assert '/follow_path' not in source
    assert 'cmd_vel' not in source


def test_handoff_snapshots_candidate_before_dispatch():
    """The active mission owns an immutable copy of the approved path."""
    source = read(HANDOFF)
    assert 'active_path_ = *candidate_path_' in source
    assert 'goal.path = *active_path_' in source
    assert 'active_candidate_generation_' in source
    assert 'make_coverage_mission_id' in source


def test_handoff_retains_ownership_on_uncertain_cancel():
    """Timeout reporting must not release an unconfirmed backend goal."""
    source = read(HANDOFF)
    assert 'retaining mission ownership' in source
    assert 'cancel_timeout_reported_' in source
    assert 'response_timeout_reported_' in source
    assert 'async_cancel_all_goals' not in source


def test_deployment_assets_are_wired():
    """The new production node, config and launch files are installed."""
    cmake = read(CMAKE)
    config = read(CONFIG)
    launch = read(LAUNCH)
    coverage_launch = read(COVERAGE_LAUNCH)
    assert 'coverage_execution_handoff_node' in cmake
    assert 'coverage_execution_handoff.yaml' in cmake
    assert 'coverage_execution_handoff.launch.xml' in cmake
    assert 'approve_service:' in config
    assert 'execution_timeout_sec: 0.0' in config
    assert 'coverage_execution_handoff_node' in launch
    assert 'start_execution_handoff' in coverage_launch
    assert 'coverage_execution_handoff.launch.xml' in coverage_launch
