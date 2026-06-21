#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only test for the ROS runtime recovery-test manager node.

The node imports rclpy at runtime, so source-level PC tests should compile it
instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_recovery_test_manager_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "recovery_test_manager_node.py"

    py_compile.compile(str(path), doraise=True)


def test_recovery_test_helpers_compile_and_import_without_ros():
    path = ROOT / "savo_control" / "nodes" / "recovery_test_helpers.py"
    py_compile.compile(str(path), doraise=True)

    from savo_control.nodes.recovery_test_helpers import (
        RecoveryTestLimits,
        RecoveryTestState,
        default_recovery_tests,
        start_active_test,
        step_active_test,
    )

    tests = default_recovery_tests(RecoveryTestLimits())
    active = start_active_test(tests["backup_short"], now_s=1.0)
    result = step_active_test(active, now_s=1.5, max_duration_s=8.0)

    assert result.state == RecoveryTestState.RUNNING
    assert result.command.vx < 0.0
    assert result.request_recovery is True
    assert result.finished is False


def test_current_runtime_nodes_compile():
    for relative_path in [
        "savo_control/nodes/control_status_node.py",
        "savo_control/nodes/recovery_status_node.py",
        "savo_control/nodes/control_dashboard_node.py",
        "savo_control/nodes/distance_pid_test_node.py",
        "savo_control/nodes/keyboard_teleop_node.py",
        "savo_control/nodes/auto_test_manager_node.py",
        "savo_control/nodes/recovery_test_manager_node.py",
    ]:
        path = ROOT / relative_path
        py_compile.compile(str(path), doraise=True)
