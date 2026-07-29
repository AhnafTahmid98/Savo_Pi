# Copyright 2026 Ahnaf Tahmid
# SPDX-License-Identifier: LicenseRef-Proprietary

"""Verify Phase 4L-B3D Coverage runtime wiring."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def read(relative_path):
    """Read one package file."""
    return (
        PACKAGE_ROOT / relative_path
    ).read_text(encoding='utf-8')


def test_gate_owns_coverage_proxy():
    """Ensure the existing gate owns Coverage admission."""
    source = read('src/nodes/goal_admission_gate_node.cpp')

    assert 'CoverageAdmissionProxy' in source
    assert 'public_coverage_action' in source
    assert 'internal_coverage_action' in source
    assert 'slot_reserved_' in source
    assert 'active_goal_' in source


def test_gateway_owns_coverage_follow_path():
    """Ensure the existing gateway owns Coverage execution."""
    source = read('src/nodes/goal_gateway_node.cpp')

    assert 'CoverageFollowPathGateway' in source
    assert 'coverage_action_name' in source
    assert 'nav2_follow_path_action_name' in source
    assert 'gateway_' in source


def test_follow_path_backend_is_used():
    """Ensure the helper forwards one validated path."""
    source = read(
        'src/ros/coverage_follow_path_gateway.cpp'
    )

    assert 'nav2_msgs::action::FollowPath' in read(
        'include/savo_nav/coverage_follow_path_gateway.hpp'
    )

    assert 'backend_goal.path' in source
    assert 'distance_to_goal' in source
    assert 'async_send_goal' in source
    assert 'async_cancel_goal' in source


def test_cancellation_retains_backend_ownership():
    """Ensure cancel timeout does not clear the session."""
    source = read(
        'src/ros/coverage_follow_path_gateway.cpp'
    )

    watchdog = source.index(
        'void CoverageFollowPathGateway::CheckWatchdogs'
    )

    backend_result = source.index(
        'void CoverageFollowPathGateway::OnBackendResult'
    )

    assert 'public_terminal_sent_' in source
    assert 'coverage_cancel_ack_timeout_quarantined' in source

    assert backend_result < watchdog or watchdog < backend_result

    cancel_timeout_section = source[
        watchdog:
        source.index(
            'void CoverageFollowPathGateway::ForwardBackendCancel'
        )
    ]

    assert 'ClearLocked();' not in cancel_timeout_section


def test_runtime_helpers_never_publish_velocity():
    """Ensure Coverage runtime has no velocity authority."""
    combined = '\n'.join(
        [
            read('src/ros/coverage_admission_proxy.cpp'),
            read('src/ros/coverage_follow_path_gateway.cpp'),
        ]
    )

    forbidden = (
        'cmd_vel',
        'geometry_msgs::msg::Twist',
        '/follow_path/_action/send_goal',
    )

    for token in forbidden:
        assert token not in combined


def test_cmake_builds_runtime_helpers():
    """Ensure both helpers are built and linked."""
    cmake = read('CMakeLists.txt')

    assert 'src/ros/coverage_admission_proxy.cpp' in cmake

    assert (
        'src/ros/coverage_follow_path_gateway.cpp'
        in cmake
    )

    assert 'savo_nav_coverage_ros_adapter' in cmake
