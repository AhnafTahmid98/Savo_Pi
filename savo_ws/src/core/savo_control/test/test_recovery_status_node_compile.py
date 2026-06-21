#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compile-only tests for ROS runtime recovery status node.

The node imports rclpy at runtime, so source-level PC tests should compile it
instead of importing it directly.
"""

from __future__ import annotations

from pathlib import Path
import py_compile


ROOT = Path(__file__).resolve().parents[1]


def test_recovery_status_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "recovery_status_node.py"

    py_compile.compile(str(path), doraise=True)


def test_control_status_node_compiles():
    path = ROOT / "savo_control" / "nodes" / "control_status_node.py"

    py_compile.compile(str(path), doraise=True)


def test_control_status_helpers_import_without_ros():
    from savo_control.nodes.control_status_helpers import (
        BoolSample,
        CommandSample,
        OdomSample,
        ScalarSample,
        TextSample,
    )
    from savo_control.models import TwistCommand

    cmd = CommandSample(
        command=TwistCommand(vx=0.1, source="manual"),
        stamp_s=1.0,
    )

    assert cmd.fresh(now_s=1.2, timeout_s=0.5) is True
    assert cmd.moving() is True

    assert BoolSample(value=True, stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
    assert ScalarSample(value=0.5, stamp_s=1.0).fresh(
        now_s=2.0,
        timeout_s=0.5,
    ) is False
    assert TextSample(value="AUTO", stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
    assert OdomSample(linear_speed=0.1, angular_speed=0.2, stamp_s=1.0).fresh(
        now_s=1.2,
        timeout_s=0.5,
    ) is True
