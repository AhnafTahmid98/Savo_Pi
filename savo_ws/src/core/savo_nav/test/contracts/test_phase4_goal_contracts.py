# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative):
    return (ROOT / relative).read_text(
        encoding='utf-8',
    )


def test_phase4_files_exist():
    required = (
        'include/savo_nav/goal_validator.hpp',
        'include/savo_nav/goal_arbiter.hpp',
        'src/core/goal_validator.cpp',
        'src/core/goal_arbiter.cpp',
        'test/unit/test_goal_validator.cpp',
        'test/unit/test_goal_arbiter.cpp',
    )

    for relative in required:
        path = ROOT / relative

        assert path.is_file()
        assert path.stat().st_size > 0


def test_phase4_does_not_forward_to_nav2():
    source = (
        read('src/core/goal_validator.cpp')
        + read('src/core/goal_arbiter.cpp')
    )

    forbidden = (
        'send_goal_async',
        'async_cancel_goal',
        'NavigateToPose',
        'rclcpp_action',
        'create_client',
        'create_server',
    )

    for term in forbidden:
        assert term not in source


def test_phase4_has_no_hardware_ownership():
    source = (
        read('src/core/goal_validator.cpp')
        + read('src/core/goal_arbiter.cpp')
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


def test_phase4_does_not_publish_velocity():
    source = (
        read('src/core/goal_validator.cpp')
        + read('src/core/goal_arbiter.cpp')
    )

    forbidden = (
        '/cmd_vel',
        '/cmd_vel_safe',
        '/cmd_vel_recovery',
        '/cmd_vel_nav',
    )

    for topic in forbidden:
        assert topic not in source


def test_single_active_goal_contract_is_explicit():
    header = read('include/savo_nav/goal_arbiter.hpp')
    source = read('src/core/goal_arbiter.cpp')

    assert 'std::optional<GoalContext>' in header
    assert 'active_goal_' in header
    assert 'active_goal_preemption_forbidden' in source
    assert 'goal_ownership_acquired' in source


def test_cancellation_acknowledgement_is_explicit():
    header = read('include/savo_nav/goal_arbiter.hpp')
    source = read('src/core/goal_arbiter.cpp')

    assert 'RequestCancel' in header
    assert 'AcknowledgeCancellation' in header

    assert (
        'cancel_request_recorded_waiting_for_'
        'acknowledgement'
    ) in source

    assert 'cancel_acknowledged_goal_released' in source


def test_duplicate_and_sequence_protection_exist():
    header = read('include/savo_nav/goal_arbiter.hpp')
    source = read('src/core/goal_arbiter.cpp')

    assert 'recent_history_' in header
    assert 'last_sequence_by_source_' in header
    assert 'goal_id_was_already_seen' in source

    assert (
        'goal_sequence_is_not_newer_for_source'
    ) in source


def test_goal_validation_uses_readiness_and_map():
    header = read('include/savo_nav/goal_validator.hpp')
    source = read('src/core/goal_validator.cpp')

    assert 'NavigationReadinessResult' in header
    assert 'MapContext' in header
    assert 'GoalPose2D' in header

    assert (
        'navigation_readiness_rejects_goal'
    ) in source

    assert (
        'goal_map_id_does_not_match_active_map'
    ) in source
