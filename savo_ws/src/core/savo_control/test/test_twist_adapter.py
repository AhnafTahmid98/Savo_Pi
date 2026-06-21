#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for TwistCommand adapters."""

from __future__ import annotations

from dataclasses import dataclass

import pytest

from savo_control.adapters import (
    copy_to_ros_msg,
    twist_from_mapping,
    twist_from_ros_msg,
    twist_from_sequence,
    twist_to_dict,
    zero_ros_twist,
)
from savo_control.models import TwistCommand


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeTwist:
    linear: Vector3
    angular: Vector3


def make_fake_twist() -> FakeTwist:
    return FakeTwist(linear=Vector3(), angular=Vector3())


def test_twist_from_mapping_uses_short_keys():
    cmd = twist_from_mapping(
        {
            "vx": 0.10,
            "vy": -0.20,
            "wz": 0.30,
            "source": "mapping",
            "stamp_sec": 12.5,
        }
    )

    assert cmd == TwistCommand(
        vx=0.10,
        vy=-0.20,
        wz=0.30,
        source="mapping",
        stamp_sec=12.5,
    )


def test_twist_from_mapping_uses_ros_style_keys():
    cmd = twist_from_mapping(
        {
            "linear_x": 0.15,
            "linear_y": -0.05,
            "angular_z": 0.25,
        },
        source="ros_style",
        stamp_sec=1.5,
    )

    assert cmd.vx == 0.15
    assert cmd.vy == -0.05
    assert cmd.wz == 0.25
    assert cmd.source == "ros_style"
    assert cmd.stamp_sec == 1.5


def test_twist_from_mapping_sanitizes_bad_values():
    cmd = twist_from_mapping(
        {
            "vx": float("nan"),
            "vy": 0.2,
            "wz": float("inf"),
        }
    )

    assert cmd.vx == 0.0
    assert cmd.vy == 0.2
    assert cmd.wz == 0.0


def test_twist_from_sequence():
    cmd = twist_from_sequence([0.1, -0.2, 0.3], source="seq", stamp_sec=2.0)

    assert cmd == TwistCommand(vx=0.1, vy=-0.2, wz=0.3, source="seq", stamp_sec=2.0)


def test_twist_from_sequence_rejects_wrong_length():
    with pytest.raises(ValueError):
        twist_from_sequence([0.1, 0.2])

    with pytest.raises(ValueError):
        twist_from_sequence([0.1, 0.2, 0.3, 0.4])


def test_copy_to_ros_msg_sets_only_valid_twist_fields():
    cmd = TwistCommand(vx=0.2, vy=-0.1, wz=0.4, source="test")
    msg = make_fake_twist()

    result = copy_to_ros_msg(cmd, msg)

    assert result is msg
    assert msg.linear.x == 0.2
    assert msg.linear.y == -0.1
    assert msg.linear.z == 0.0
    assert msg.angular.x == 0.0
    assert msg.angular.y == 0.0
    assert msg.angular.z == 0.4


def test_copy_to_ros_msg_sanitizes_non_finite_values():
    cmd = TwistCommand(vx=float("nan"), vy=float("inf"), wz=-float("inf"))
    msg = make_fake_twist()

    copy_to_ros_msg(cmd, msg)

    assert msg.linear.x == 0.0
    assert msg.linear.y == 0.0
    assert msg.angular.z == 0.0


def test_twist_from_ros_msg():
    msg = make_fake_twist()
    msg.linear.x = 0.12
    msg.linear.y = -0.03
    msg.linear.z = 99.0
    msg.angular.x = 11.0
    msg.angular.y = 12.0
    msg.angular.z = -0.45

    cmd = twist_from_ros_msg(msg, source="ros", stamp_sec=4.0)

    assert cmd.vx == 0.12
    assert cmd.vy == -0.03
    assert cmd.wz == -0.45
    assert cmd.source == "ros"
    assert cmd.stamp_sec == 4.0


def test_twist_to_dict():
    cmd = TwistCommand(vx=0.1, vy=0.2, wz=0.3, source="dict", stamp_sec=5.0)

    assert twist_to_dict(cmd) == {
        "vx": 0.1,
        "vy": 0.2,
        "wz": 0.3,
        "source": "dict",
        "stamp_sec": 5.0,
    }


def test_zero_ros_twist_with_fake_msg_type():
    msg = zero_ros_twist(msg_type=make_fake_twist)

    assert msg.linear.x == 0.0
    assert msg.linear.y == 0.0
    assert msg.linear.z == 0.0
    assert msg.angular.x == 0.0
    assert msg.angular.y == 0.0
    assert msg.angular.z == 0.0
