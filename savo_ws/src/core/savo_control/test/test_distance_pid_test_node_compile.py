#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only test for the ROS runtime distance PID test node.

The node imports rclpy at runtime, so source-level PC tests should compile it
instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_distance_pid_test_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "distance_pid_test_node.py"

    py_compile.compile(str(path), doraise=True)


def test_control_runtime_nodes_compile():
    for relative_path in [
        "savo_control/nodes/control_status_node.py",
        "savo_control/nodes/recovery_status_node.py",
        "savo_control/nodes/control_dashboard_node.py",
        "savo_control/nodes/distance_pid_test_node.py",
    ]:
        path = ROOT / relative_path
        py_compile.compile(str(path), doraise=True)
