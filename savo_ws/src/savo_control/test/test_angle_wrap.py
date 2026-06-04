#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / test_angle_wrap.py
==============================================

Unit tests for angle helper utilities.

These tests validate:
  - radians/degrees conversion
  - angle normalization
  - shortest angular distance
  - angle reached tolerance
  - yaw extraction from quaternion
  - yaw-only quaternion generation

These tests are ROS-independent and do not require robot hardware.
"""

from __future__ import annotations

import math

import pytest

from savo_control.utils.angles import (
    TAU,
    angle_reached_rad,
    angular_direction_text,
    clamp_angle_error_rad,
    deg_to_rad,
    format_angle_rad,
    normalize_angle_0_2pi_rad,
    normalize_angle_rad,
    quaternion_xyzw_from_yaw,
    rad_to_deg,
    shortest_angular_distance_rad,
    yaw_from_quaternion_xyzw,
    yaw_from_ros_quaternion,
)


class DummyQuaternion:
    """Small object that behaves like geometry_msgs/msg/Quaternion for tests."""

    def __init__(self, x: float, y: float, z: float, w: float) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def assert_angle_close(actual: float, expected: float, tol: float = 1.0e-9) -> None:
    """Compare two angles using wrapped angular distance."""
    error = normalize_angle_rad(actual - expected)
    assert abs(error) <= tol, f"actual={actual}, expected={expected}, error={error}"


def test_tau_constant() -> None:
    assert TAU == pytest.approx(2.0 * math.pi)


def test_rad_deg_conversion() -> None:
    assert rad_to_deg(math.pi) == pytest.approx(180.0)
    assert rad_to_deg(math.pi / 2.0) == pytest.approx(90.0)
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert deg_to_rad(90.0) == pytest.approx(math.pi / 2.0)


def test_invalid_rad_deg_conversion_returns_zero() -> None:
    assert rad_to_deg(float("nan")) == 0.0
    assert rad_to_deg(float("inf")) == 0.0
    assert deg_to_rad(float("nan")) == 0.0
    assert deg_to_rad(float("inf")) == 0.0


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 0.0),
        (math.pi / 2.0, math.pi / 2.0),
        (-math.pi / 2.0, -math.pi / 2.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),
        (2.0 * math.pi, 0.0),
        (-2.0 * math.pi, 0.0),
        (3.0 * math.pi, math.pi),
        (-3.0 * math.pi, math.pi),
        (5.0 * math.pi / 2.0, math.pi / 2.0),
        (-5.0 * math.pi / 2.0, -math.pi / 2.0),
    ],
)
def test_normalize_angle_rad(angle: float, expected: float) -> None:
    assert normalize_angle_rad(angle) == pytest.approx(expected)


def test_normalize_angle_rad_rejects_invalid_values() -> None:
    assert normalize_angle_rad(float("nan")) == 0.0
    assert normalize_angle_rad(float("inf")) == 0.0
    assert normalize_angle_rad(float("-inf")) == 0.0


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (2.0 * math.pi, 0.0),
        (-math.pi / 2.0, 1.5 * math.pi),
        (-2.0 * math.pi, 0.0),
        (5.0 * math.pi / 2.0, math.pi / 2.0),
    ],
)
def test_normalize_angle_0_2pi_rad(angle: float, expected: float) -> None:
    assert normalize_angle_0_2pi_rad(angle) == pytest.approx(expected)


def test_shortest_angular_distance_simple() -> None:
    assert shortest_angular_distance_rad(0.0, math.pi / 2.0) == pytest.approx(
        math.pi / 2.0
    )
    assert shortest_angular_distance_rad(0.0, -math.pi / 2.0) == pytest.approx(
        -math.pi / 2.0
    )
    assert shortest_angular_distance_rad(math.pi / 2.0, 0.0) == pytest.approx(
        -math.pi / 2.0
    )


def test_shortest_angular_distance_wraparound_positive() -> None:
    # From +170 deg to -170 deg should rotate +20 deg through +pi boundary.
    from_angle = deg_to_rad(170.0)
    to_angle = deg_to_rad(-170.0)

    result = shortest_angular_distance_rad(from_angle, to_angle)

    assert result == pytest.approx(deg_to_rad(20.0))


def test_shortest_angular_distance_wraparound_negative() -> None:
    # From -170 deg to +170 deg should rotate -20 deg through -pi boundary.
    from_angle = deg_to_rad(-170.0)
    to_angle = deg_to_rad(170.0)

    result = shortest_angular_distance_rad(from_angle, to_angle)

    assert result == pytest.approx(deg_to_rad(-20.0))


def test_angle_reached_rad() -> None:
    assert angle_reached_rad(
        current_rad=deg_to_rad(10.0),
        target_rad=deg_to_rad(12.0),
        tolerance_rad=deg_to_rad(3.0),
    )

    assert not angle_reached_rad(
        current_rad=deg_to_rad(10.0),
        target_rad=deg_to_rad(20.0),
        tolerance_rad=deg_to_rad(3.0),
    )


def test_angle_reached_wraparound() -> None:
    assert angle_reached_rad(
        current_rad=deg_to_rad(179.0),
        target_rad=deg_to_rad(-179.0),
        tolerance_rad=deg_to_rad(3.0),
    )


def test_clamp_angle_error_rad() -> None:
    assert clamp_angle_error_rad(deg_to_rad(90.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(30.0)
    )

    assert clamp_angle_error_rad(deg_to_rad(-90.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(-30.0)
    )

    assert clamp_angle_error_rad(deg_to_rad(10.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(10.0)
    )


def test_angular_direction_text() -> None:
    assert angular_direction_text(0.0) == "ZERO"
    assert angular_direction_text(0.1) == "CCW"
    assert angular_direction_text(-0.1) == "CW"


def test_quaternion_from_yaw_and_back_zero() -> None:
    x, y, z, w = quaternion_xyzw_from_yaw(0.0)

    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(0.0)
    assert w == pytest.approx(1.0)

    yaw = yaw_from_quaternion_xyzw(x, y, z, w)
    assert yaw is not None
    assert_angle_close(yaw, 0.0)


def test_quaternion_from_yaw_and_back_positive() -> None:
    target_yaw = math.pi / 2.0
    x, y, z, w = quaternion_xyzw_from_yaw(target_yaw)

    yaw = yaw_from_quaternion_xyzw(x, y, z, w)

    assert yaw is not None
    assert_angle_close(yaw, target_yaw)


def test_quaternion_from_yaw_and_back_negative() -> None:
    target_yaw = -math.pi / 2.0
    x, y, z, w = quaternion_xyzw_from_yaw(target_yaw)

    yaw = yaw_from_quaternion_xyzw(x, y, z, w)

    assert yaw is not None
    assert_angle_close(yaw, target_yaw)


def test_quaternion_from_yaw_wraps_input() -> None:
    target_yaw = 3.0 * math.pi
    x, y, z, w = quaternion_xyzw_from_yaw(target_yaw)

    yaw = yaw_from_quaternion_xyzw(x, y, z, w)

    assert yaw is not None
    assert_angle_close(yaw, math.pi)


def test_yaw_from_quaternion_normalizes_non_unit_quaternion() -> None:
    # Quaternion for yaw 90 deg, scaled by 2.
    target_yaw = math.pi / 2.0
    x, y, z, w = quaternion_xyzw_from_yaw(target_yaw)

    yaw = yaw_from_quaternion_xyzw(2.0 * x, 2.0 * y, 2.0 * z, 2.0 * w)

    assert yaw is not None
    assert_angle_close(yaw, target_yaw)


def test_yaw_from_quaternion_rejects_invalid_zero_norm() -> None:
    yaw = yaw_from_quaternion_xyzw(0.0, 0.0, 0.0, 0.0)
    assert yaw is None


def test_yaw_from_quaternion_rejects_nan() -> None:
    yaw = yaw_from_quaternion_xyzw(0.0, 0.0, float("nan"), 1.0)
    assert yaw is None


def test_yaw_from_ros_quaternion() -> None:
    target_yaw = deg_to_rad(45.0)
    x, y, z, w = quaternion_xyzw_from_yaw(target_yaw)

    q = DummyQuaternion(x=x, y=y, z=z, w=w)

    yaw = yaw_from_ros_quaternion(q)

    assert yaw is not None
    assert_angle_close(yaw, target_yaw)


def test_yaw_from_ros_quaternion_none() -> None:
    assert yaw_from_ros_quaternion(None) is None


def test_format_angle_rad_contains_units() -> None:
    text = format_angle_rad(math.pi / 2.0)

    assert "rad" in text
    assert "deg" in text
    assert "+1.571" in text
    assert "+90.0" in text