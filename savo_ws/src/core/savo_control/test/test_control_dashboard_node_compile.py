#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only tests for the ROS runtime control dashboard node.

The dashboard node imports rclpy at runtime, so source-level PC tests should
compile it instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_control_dashboard_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "control_dashboard_node.py"

    py_compile.compile(str(path), doraise=True)


def test_status_runtime_nodes_compile():
    for relative_path in [
        "savo_control/nodes/control_status_node.py",
        "savo_control/nodes/recovery_status_node.py",
        "savo_control/nodes/control_dashboard_node.py",
    ]:
        path = ROOT / relative_path
        py_compile.compile(str(path), doraise=True)


def test_control_status_helpers_stay_ros_free():
    from savo_control.nodes.control_status_helpers import (
        BoolSample,
        CommandSample,
        OdomSample,
        ScalarSample,
        TextSample,
    )
    from savo_control.models import TwistCommand

    cmd = CommandSample(
        command=TwistCommand(vx=0.1, source="safe"),
        stamp_s=1.0,
    )

    assert cmd.fresh(now_s=1.2, timeout_s=0.5) is True
    assert cmd.moving() is True

    assert BoolSample(value=False, stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
    assert ScalarSample(value=0.75, stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
    assert TextSample(value="AUTO", stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
    assert OdomSample(linear_speed=0.1, angular_speed=0.2, stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
