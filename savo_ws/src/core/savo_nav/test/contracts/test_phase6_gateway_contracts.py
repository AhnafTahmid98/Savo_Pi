# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate Phase 6 action-gateway contracts."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    """Read a package file."""
    return (ROOT / relative).read_text(
        encoding='utf-8'
    )


def test_phase6_files_exist():
    """Verify Phase 6 source and fixture files."""
    required = (
        'include/savo_nav/goal_gateway.hpp',
        'src/core/goal_gateway.cpp',
        'src/nodes/goal_gateway_node.cpp',
        'config/goal_gateway.yaml',
        'test/unit/test_goal_gateway.cpp',
        'test/fixtures/fake_nav2_server_node.cpp',
        'test/fixtures/goal_gateway_client.py',
    )

    for relative in required:
        path = ROOT / relative

        assert path.is_file(), relative
        assert path.stat().st_size > 0, relative


def test_public_and_internal_action_names():
    """Verify the three action endpoints."""
    action_names = read(
        'include/savo_nav/action_names.hpp'
    )

    assert (
        '/savo_nav/navigation/navigate_to_pose'
        in action_names
    )

    assert (
        '/savo_nav/exploration/navigate_to_pose'
        in action_names
    )

    assert '/navigate_to_pose' in action_names


def test_gateway_forwards_to_nav2():
    """Verify asynchronous Nav2 goal forwarding."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    assert 'async_send_goal' in source
    assert 'async_cancel_goal' in source
    assert 'OnNav2Feedback' in source
    assert 'OnNav2Result' in source


def test_single_gateway_owns_both_sources():
    """Verify both public servers share one arbiter."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    assert 'navigation_server_' in source
    assert 'exploration_server_' in source
    assert 'std::unique_ptr<savo_nav::GoalGateway>' in source

    assert source.count('gateway_->Admit') == 1


def test_cancel_acknowledgement_precedes_release():
    """Verify cancel result drives ownership release."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    assert 'AcknowledgeCancellation' in source

    assert (
        'ResultCode::CANCELED'
        in source
    )

    assert 'async_cancel_goal' in source


def test_behavior_tree_override_is_disabled():
    """Verify arbitrary BT paths are rejected by default."""
    with (
        ROOT / 'config/goal_gateway.yaml'
    ).open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    parameters = config[
        'goal_gateway_node'
    ]['ros__parameters']

    assert (
        parameters[
            'allow_behavior_tree_override'
        ]
        is False
    )


def test_gateway_has_no_hardware_access():
    """Verify no hardware implementation leaks."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    ).lower()

    forbidden = (
        '/dev/',
        'gpiochip',
        'pca9685',
        'tca9548',
        'vl53l1x',
        'hc-sr04',
        'librealsense',
        'rplidar',
        'i2c-dev',
    )

    for term in forbidden:
        assert term not in source


def test_gateway_never_publishes_velocity():
    """Verify gateway does not own velocity topics."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    forbidden = (
        '/cmd_vel',
        '/cmd_vel_nav',
        '/cmd_vel_safe',
        '/cmd_vel_recovery',
    )

    for topic in forbidden:
        assert topic not in source


def test_gateway_requires_readiness():
    """Verify admission consumes readiness state."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    assert 'kReadiness' in source
    assert 'kReadinessReason' in source
    assert 'goal_acceptance_allowed' in source


def test_watchdogs_request_nav2_cancellation():
    """Verify execution and feedback watchdogs."""
    source = read(
        'src/nodes/goal_gateway_node.cpp'
    )

    assert 'nav2_execution_timeout' in source
    assert 'nav2_feedback_stale' in source
    assert 'CheckWatchdogs' in source
