# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate Phase 7B goal admission contracts."""

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    """Read one package file."""
    return (ROOT / relative).read_text(encoding='utf-8')


def test_phase7b_files_exist():
    """Verify Phase 7B production and test files."""
    required = (
        'include/savo_nav/goal_admission_policy.hpp',
        'src/core/goal_admission_policy.cpp',
        'src/nodes/goal_admission_gate_node.cpp',
        'config/goal_admission_gate.yaml',
        'test/unit/test_goal_admission_policy.cpp',
        'test/fixtures/fake_guarded_gateway_server_node.cpp',
        'test/fixtures/goal_admission_gate_fixture.py',
    )

    for relative in required:
        path = ROOT / relative
        assert path.is_file(), relative
        assert path.stat().st_size > 0, relative


def test_public_and_internal_actions_are_separate():
    """Verify public actions forward only through hidden internals."""
    header = read('include/savo_nav/action_names.hpp')

    assert '/savo_nav/navigation/navigate_to_pose' in header
    assert '/savo_nav/exploration/navigate_to_pose' in header

    assert (
        '/savo_nav/_internal/navigation/navigate_to_pose'
        in header
    )

    assert (
        '/savo_nav/_internal/exploration/navigate_to_pose'
        in header
    )


def test_gate_is_fail_closed_and_one_shot():
    """Verify unknown state blocks and cancellation is one-shot."""
    policy = read('include/savo_nav/goal_admission_policy.hpp')
    node = read('src/nodes/goal_admission_gate_node.cpp')

    assert 'guard_observed{false}' in policy
    assert 'guard_allowed{false}' in policy
    assert 'accept_new_goal{false}' in policy

    assert 'internal_cancel_sent_' in node
    assert 'control_recovery_guard_stale' in node
    assert 'RequestInternalCancel' in node


def test_gate_does_not_gain_motion_or_control_authority():
    """Verify the gate cannot command control, recovery, or velocity."""
    source = read('src/nodes/goal_admission_gate_node.cpp')

    forbidden = (
        '/savo_control/mode_cmd',
        '/savo_control/recovery_request',
        '/cmd_vel',
        '/cmd_vel_nav',
        '/cmd_vel_safe',
        '/cmd_vel_recovery',
        'geometry_msgs',
    )

    for item in forbidden:
        assert item not in source


def test_gate_observer_topics_are_declared():
    """Verify admission state is externally observable."""
    header = read('include/savo_nav/topic_names.hpp')

    required = (
        '/savo_nav/goal_admission/state',
        '/savo_nav/goal_admission/reason',
        '/savo_nav/goal_admission/status',
    )

    for topic in required:
        assert topic in header


def test_guarded_launch_contains_gate_and_remaps():
    """Verify production launch places the gate in front."""
    launch_path = ROOT / 'launch/saved_map_navigation.launch.py'
    launch_text = launch_path.read_text(encoding='utf-8')

    ast.parse(launch_text)

    assert 'goal_admission_gate_node' in launch_text
    assert (
        '/savo_nav/_internal/navigation/navigate_to_pose'
        in launch_text
    )

    assert (
        '/savo_nav/_internal/exploration/navigate_to_pose'
        in launch_text
    )


def test_saved_map_profile_enables_guarded_gateway():
    """Verify the saved-map profile records the new topology."""
    path = ROOT / 'config/profiles/saved_map.yaml'

    with path.open('r', encoding='utf-8') as stream:
        profile = yaml.safe_load(stream)

    saved = profile['saved_map_profile']
    assert saved['goal_gateway_enabled'] is True
    assert saved['goal_admission_gate_enabled'] is True
    assert saved['direct_public_goal_gateway_enabled'] is False
    assert saved['internal_gateway_actions_hidden'] is True
