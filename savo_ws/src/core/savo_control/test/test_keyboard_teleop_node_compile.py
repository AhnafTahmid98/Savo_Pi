#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only test for the ROS runtime keyboard teleop node.

The node imports rclpy and terminal/ROS runtime modules, so source-level PC
tests should compile it instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_keyboard_teleop_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "keyboard_teleop_node.py"

    py_compile.compile(str(path), doraise=True)


def test_keyboard_teleop_helpers_compile_and_import_without_ros():
    path = ROOT / "savo_control" / "nodes" / "keyboard_teleop_helpers.py"
    py_compile.compile(str(path), doraise=True)

    from savo_control.nodes.keyboard_teleop_helpers import (
        TeleopLimits,
        TeleopSpeeds,
        apply_teleop_key,
    )

    result = apply_teleop_key(
        "w",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
        stamp_sec=1.0,
    )

    assert result.handled is True
    assert result.command is not None
    assert result.command.vx == 0.12
    assert result.command.vy == 0.0
    assert result.command.wz == 0.0
    assert result.command.source == "keyboard_teleop"
    assert result.command.stamp_sec == 1.0


def test_current_runtime_nodes_compile():
    for relative_path in [
        "savo_control/nodes/control_status_node.py",
        "savo_control/nodes/recovery_status_node.py",
        "savo_control/nodes/control_dashboard_node.py",
        "savo_control/nodes/distance_pid_test_node.py",
        "savo_control/nodes/keyboard_teleop_node.py",
    ]:
        path = ROOT / relative_path
        py_compile.compile(str(path), doraise=True)
