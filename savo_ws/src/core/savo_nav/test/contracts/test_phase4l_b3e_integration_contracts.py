# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Verify Phase 4L-B3E isolated integration coverage."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def read(relative_path):
    """Read one package file."""
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_fake_follow_path_fixture_exists():
    """Verify the fake Nav2 action server modes."""
    source = read(
        'test/fixtures/fake_follow_path_server_node.cpp'
    )

    required = (
        'nav2_msgs::action::FollowPath',
        'test_success',
        'test_reject',
        'test_execution_timeout',
        'test_stale',
        'test_late_success',
    )

    for token in required:
        assert token in source


def test_launch_uses_private_test_endpoints():
    """Verify the launch test is isolated."""
    source = read(
        'test/launch/'
        'test_phase4l_b3e_coverage_runtime_launch.py'
    )

    assert '/test/coverage' in source
    assert '/test/internal/coverage' in source
    assert '/test/follow_path' in source
    assert 'fake_follow_path_server_node' in source


def test_launch_never_starts_robot_control():
    """Verify motor and control nodes are absent."""
    source = read(
        'test/launch/'
        'test_phase4l_b3e_coverage_runtime_launch.py'
    )

    forbidden = (
        'savo_base',
        'savo_control',
        'base_driver',
        'controller_server',
        'cmd_vel',
    )

    for token in forbidden:
        assert token not in source


def test_gateway_protects_active_runtime_state():
    """Verify rejected concurrent goals cannot reset state."""
    source = read(
        'src/ros/coverage_follow_path_gateway.cpp'
    )

    assert 'if (active_)' in source

    assert (
        'coverage_rejected_goal_gateway_busy'
        in source
    )


def test_fake_backend_mode_is_test_only():
    """Verify scenario control stays outside production actions."""
    fixture = read(
        'test/fixtures/fake_follow_path_server_node.cpp'
    )

    launch_source = read(
        'test/launch/'
        'test_phase4l_b3e_coverage_runtime_launch.py'
    )

    gateway = read(
        'src/ros/coverage_follow_path_gateway.cpp'
    )

    assert '/test/follow_path_mode' in fixture
    assert 'FOLLOW_PATH_MODE' in launch_source
    assert 'backend_mode_publisher' in launch_source

    assert 'backend_goal.controller_id = controller_id_' in gateway

    assert (
        'goal_handle->get_goal()->controller_id'
        not in gateway
    )


def test_integration_covers_required_terminals():
    """Verify the runtime matrix covers B3E2 and B3E3."""
    source = read(
        'test/launch/'
        'test_phase4l_b3e_coverage_runtime_launch.py'
    )

    required = (
        'RESULT_SUCCEEDED',
        'RESULT_BACKEND_REJECTED',
        'RESULT_CANCELED',
        'RESULT_TIMED_OUT',
        'RESULT_FEEDBACK_STALE',
        'coverage-quarantine-blocked',
        'coverage-after-quarantine',
    )

    for token in required:
        assert token in source
