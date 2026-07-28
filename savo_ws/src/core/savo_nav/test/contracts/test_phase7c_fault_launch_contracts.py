# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Validate Phase 7C fault and launch-chain contracts."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    """Read one package file as UTF-8."""
    return (ROOT / relative).read_text(encoding='utf-8')


def node_executables():
    """Return literal Node executable values from production launch."""
    source = read('launch/saved_map_navigation.launch.py')
    tree = ast.parse(source)
    executables = []

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        function_name = None
        if isinstance(node.func, ast.Name):
            function_name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            function_name = node.func.attr
        if function_name != 'Node':
            continue
        for keyword in node.keywords:
            if (
                keyword.arg == 'executable'
                and isinstance(keyword.value, ast.Constant)
                and isinstance(keyword.value.value, str)
            ):
                executables.append(keyword.value.value)

    return executables


def test_phase7c_files_exist_and_are_nonempty():
    """Require both Phase 7C test files."""
    required = (
        'test/fixtures/control_recovery_chain_fixture.py',
        'test/contracts/test_phase7c_fault_launch_contracts.py',
    )
    for relative in required:
        path = ROOT / relative
        assert path.is_file(), relative
        assert path.stat().st_size > 0, relative


def test_launch_contains_exactly_one_guarded_chain():
    """Require exactly one guard, gate, and hidden gateway."""
    executables = node_executables()
    assert executables.count('control_recovery_guard_node') == 1
    assert executables.count('goal_admission_gate_node') == 1
    assert executables.count('goal_gateway_node') == 1


def test_launch_retains_public_and_hidden_actions():
    """Require stable public actions and hidden gateway remaps."""
    source = read('launch/saved_map_navigation.launch.py')
    required = (
        '/savo_nav/navigation/navigate_to_pose',
        '/savo_nav/exploration/navigate_to_pose',
        '/savo_nav/_internal/navigation/navigate_to_pose',
        '/savo_nav/_internal/exploration/navigate_to_pose',
    )
    for action_name in required:
        assert action_name in source


def test_guard_source_is_fail_closed():
    """Require unobserved and stale handling in the guard chain."""
    sources = (
        read('src/nodes/control_recovery_guard_node.cpp')
        + read('src/core/control_mode_client.cpp')
        + read('src/core/recovery_bridge.cpp')
    )
    assert 'control_mode_unobserved' in sources
    assert 'control_mode_stale' in sources
    assert 'recovery_state_unobserved' in sources
    assert 'recovery_state_stale' in sources


def test_gate_contains_required_fault_reasons_and_one_shot_state():
    """Require deterministic admission and interruption state."""
    source = read('src/nodes/goal_admission_gate_node.cpp')
    required = (
        'control_recovery_guard_unobserved',
        'control_recovery_guard_stale',
        'internal_goal_gateway_unavailable',
        'internal_cancel_sent_',
    )
    for item in required:
        assert item in source


def test_fixture_has_no_command_or_output_authority():
    """Forbid control, recovery, and velocity outputs."""
    source = read(
        'test/fixtures/control_recovery_chain_fixture.py'
    )
    forbidden = (
        '/savo_control/mode_' + 'cmd',
        '/savo_control/recovery_' + 'request',
        '/' + 'cmd_vel',
        '/' + 'cmd_vel_nav',
        '/' + 'cmd_vel_safe',
        '/' + 'cmd_vel_recovery',
    )
    for topic in forbidden:
        assert topic not in source
