# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Static contracts for the Phase 1 six-package supervisor integration."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_node_observes_all_phase1_components_and_direct_safety() -> None:
    source = (ROOT / 'src/supervisor_node.cpp').read_text(encoding='utf-8')

    for token in (
        'base_status_',
        'control_status_',
        'perception_status_',
        'lidar_status_',
        'localization_status_',
        'power_status_',
        'ParseBaseState',
        'ParseControlStatus',
        'ParsePerceptionHealth',
        'ParseLidarState',
        'ParsePowerStatus',
        'safety_stop_tracker_',
        'safety_slowdown_tracker_',
    ):
        assert token in source

    for forbidden in (
        'geometry_msgs/msg/twist',
        'create_publisher<geometry_msgs',
        '/cmd_vel',
        'NavigateToPose',
    ):
        assert forbidden not in source


def test_default_policy_uses_existing_package_contracts() -> None:
    policy = (ROOT / 'src/supervisor_policy.cpp').read_text(encoding='utf-8')
    config = (ROOT / 'config/supervisor.yaml').read_text(encoding='utf-8')

    for topic in (
        '/savo_base/base_state',
        '/savo_control/control_status',
        '/savo_control/twist_mux/status',
        '/savo_control/cmd_vel_shaper/status',
        '/savo_perception/range_health',
        '/savo_perception/safety_state',
        '/savo_perception/heartbeat',
        '/savo_lidar/state',
        '/savo_lidar/heartbeat',
        '/savo_localization/health',
        '/savo_localization/state_summary',
        '/savo_localization/heartbeat',
        '/savo_power/health',
        '/savo_power/status',
        '/safety/stop',
        '/safety/slowdown_factor',
    ):
        assert topic in policy or topic in config


def test_phase1_capability_contract_and_runtime_fixture_exist() -> None:
    state_header = (ROOT / 'include/savo_supervisor/supervisor_state.hpp').read_text(
        encoding='utf-8')
    policy = (ROOT / 'src/supervisor_policy.cpp').read_text(encoding='utf-8')
    fixture = ROOT / 'test/runtime/core_fixture.py'
    probe = ROOT / 'test/runtime/core_supervisor_probe.py'
    runtime = ROOT / 'test/runtime/run_core_supervisor_recovery_test.sh'

    for capability in (
        'core_health_ready',
        'core_safety_ready',
        'core_motion_ready',
        'can_manual_drive',
        'can_rotate',
        'can_start_geometric_mapping',
    ):
        assert capability in state_header
        assert capability in policy

    assert fixture.is_file()
    assert probe.is_file()
    assert runtime.is_file()
    assert 'drop-component' in fixture.read_text(encoding='utf-8')
    assert 'safety_stop_active' in runtime.read_text(encoding='utf-8')
