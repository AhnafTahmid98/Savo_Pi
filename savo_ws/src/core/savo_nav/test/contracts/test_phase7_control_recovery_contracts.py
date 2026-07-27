# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate Phase 7A control/recovery guard contracts."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    """Read one package file."""
    return (ROOT / relative).read_text(
        encoding='utf-8'
    )


def test_phase7a_files_exist():
    """Verify all Phase 7A runtime files exist."""
    required = (
        'include/savo_nav/control_mode_client.hpp',
        'include/savo_nav/control_recovery_guard.hpp',
        'include/savo_nav/recovery_bridge.hpp',
        'src/core/control_mode_client.cpp',
        'src/core/control_recovery_guard.cpp',
        'src/core/recovery_bridge.cpp',
        'src/nodes/control_recovery_guard_node.cpp',
        'config/control_recovery_guard.yaml',
        'test/unit/test_control_recovery.cpp',
    )

    for relative in required:
        path = ROOT / relative

        assert path.is_file(), relative
        assert path.stat().st_size > 0, relative


def test_existing_control_topics_are_consumed():
    """Verify the current savo_control interfaces."""
    source = read(
        'src/nodes/control_recovery_guard_node.cpp'
    )

    header = read(
        'include/savo_nav/topic_names.hpp'
    )

    required = (
        '/savo_control/mode_state',
        '/savo_control/mode_reason',
        '/savo_control/control_status',
        '/savo_control/recovery_active',
        '/savo_control/recovery_state',
        '/savo_control/recovery_status',
    )

    for topic in required:
        assert topic in header

    assert 'create_subscription' in source

    assert 'kControlModeState' in header
    assert 'kControlRecoveryState' in header

    assert header.count('kControlModeState') == 1
    assert header.count('kRecoveryState') == 1
    assert header.count('kControlRecoveryState') == 1


def test_guard_outputs_are_observer_contracts():
    """Verify the three navigation guard outputs."""
    header = read(
        'include/savo_nav/topic_names.hpp'
    )

    required = (
        '/savo_nav/control_recovery/allowed',
        '/savo_nav/control_recovery/reason',
        '/savo_nav/control_recovery/status',
    )

    for topic in required:
        assert topic in header


def test_nav_does_not_request_control_mode():
    """Verify supervisor and savo_control retain authority."""
    source = read(
        'src/nodes/control_recovery_guard_node.cpp'
    )

    forbidden = (
        '/savo_control/mode_cmd',
        '/savo_control/recovery_request',
        'mode_cmd_publisher',
        'recovery_request_publisher',
    )

    for item in forbidden:
        assert item not in source


def test_guard_never_publishes_velocity():
    """Verify the guard has no movement output."""
    source = read(
        'src/nodes/control_recovery_guard_node.cpp'
    )

    forbidden = (
        '/cmd_vel',
        '/cmd_vel_nav',
        '/cmd_vel_safe',
        '/cmd_vel_recovery',
        'geometry_msgs',
    )

    for item in forbidden:
        assert item not in source


def test_guard_defaults_to_blocked():
    """Verify unknown state cannot authorize navigation."""
    source = read(
        'include/savo_nav/'
        'control_recovery_guard.hpp'
    )

    assert 'navigation_allowed{false}' in source
    assert 'cancel_active_goal{true}' in source


def test_guard_configuration_matches_contract():
    """Verify configuration uses authoritative topics."""
    path = ROOT / 'config/control_recovery_guard.yaml'

    with path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    parameters = config[
        'control_recovery_guard_node'
    ]['ros__parameters']

    assert parameters[
        'control_mode_state_topic'
    ] == '/savo_control/mode_state'

    assert parameters[
        'recovery_active_topic'
    ] == '/savo_control/recovery_active'

    assert parameters[
        'navigation_allowed_topic'
    ] == '/savo_nav/control_recovery/allowed'

    assert (
        parameters['control_mode_timeout_seconds']
        > 0.0
    )

    assert parameters['recovery_timeout_seconds'] > 0.0
