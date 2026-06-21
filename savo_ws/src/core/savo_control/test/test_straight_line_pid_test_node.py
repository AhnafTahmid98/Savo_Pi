#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only test for the ROS runtime straight-line PID test node.

The node imports rclpy at runtime, so source-level PC tests should compile it
instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_straight_line_pid_test_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "straight_line_pid_test_node.py"

    py_compile.compile(str(path), doraise=True)


def test_straight_line_helpers_compile_and_import_without_ros():
    path = ROOT / "savo_control" / "nodes" / "straight_line_test_helpers.py"
    py_compile.compile(str(path), doraise=True)

    from savo_control.nodes.straight_line_test_helpers import (
        Pose2D,
        StraightLineConfig,
        StraightLineState,
        start_run,
        step_straight_line,
    )

    cfg = StraightLineConfig(target_distance_m=1.0).sanitized()
    run = start_run(pose=Pose2D(0.0, 0.0, 0.0), config=cfg, now_s=1.0)

    result = step_straight_line(
        run=run,
        pose=Pose2D(0.5, 0.1, 0.05),
        config=cfg,
        now_s=2.0,
    )

    assert result.state == StraightLineState.RUNNING
    assert result.command.vx > 0.0
    assert result.remaining_m > 0.0
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
        "savo_control/nodes/straight_line_pid_test_node.py",
    ]:
        path = ROOT / relative_path
        py_compile.compile(str(path), doraise=True)
