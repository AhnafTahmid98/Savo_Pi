#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for odometry math helpers used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.math.odom_math import (
    Pose2D,
    Twist2D,
    distance_2d_m,
    pose_is_finite,
    twist_is_finite,
    twist_magnitude,
)


def test_pose2d_defaults() -> None:
    pose = Pose2D()

    assert pose.x_m == pytest.approx(0.0)
    assert pose.y_m == pytest.approx(0.0)
    assert pose.yaw_rad == pytest.approx(0.0)


def test_twist2d_defaults() -> None:
    twist = Twist2D()

    assert twist.vx_mps == pytest.approx(0.0)
    assert twist.vy_mps == pytest.approx(0.0)
    assert twist.omega_rad_s == pytest.approx(0.0)


def test_distance_2d_m_zero_for_same_pose() -> None:
    pose = Pose2D(x_m=1.0, y_m=2.0, yaw_rad=0.5)

    assert distance_2d_m(pose, pose) == pytest.approx(0.0)


def test_distance_2d_m_uses_xy_only() -> None:
    first = Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0)
    second = Pose2D(x_m=3.0, y_m=4.0, yaw_rad=math.pi)

    assert distance_2d_m(first, second) == pytest.approx(5.0)


def test_distance_2d_m_is_symmetric() -> None:
    first = Pose2D(x_m=-1.0, y_m=2.0, yaw_rad=0.0)
    second = Pose2D(x_m=4.0, y_m=-2.0, yaw_rad=1.0)

    assert distance_2d_m(first, second) == pytest.approx(distance_2d_m(second, first))


def test_twist_magnitude_zero() -> None:
    linear_speed, angular_speed = twist_magnitude(Twist2D())

    assert linear_speed == pytest.approx(0.0)
    assert angular_speed == pytest.approx(0.0)


def test_twist_magnitude_linear_speed() -> None:
    twist = Twist2D(vx_mps=3.0, vy_mps=4.0, omega_rad_s=0.5)

    linear_speed, angular_speed = twist_magnitude(twist)

    assert linear_speed == pytest.approx(5.0)
    assert angular_speed == pytest.approx(0.5)


def test_twist_magnitude_angular_speed_is_absolute() -> None:
    twist = Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=-1.25)

    linear_speed, angular_speed = twist_magnitude(twist)

    assert linear_speed == pytest.approx(0.0)
    assert angular_speed == pytest.approx(1.25)


def test_pose_is_finite_true() -> None:
    pose = Pose2D(x_m=1.0, y_m=-2.0, yaw_rad=0.25)

    assert pose_is_finite(pose)


def test_pose_is_finite_false_for_nan_x() -> None:
    pose = Pose2D(x_m=math.nan, y_m=0.0, yaw_rad=0.0)

    assert not pose_is_finite(pose)


def test_pose_is_finite_false_for_inf_y() -> None:
    pose = Pose2D(x_m=0.0, y_m=math.inf, yaw_rad=0.0)

    assert not pose_is_finite(pose)


def test_pose_is_finite_false_for_nan_yaw() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, yaw_rad=math.nan)

    assert not pose_is_finite(pose)


def test_twist_is_finite_true() -> None:
    twist = Twist2D(vx_mps=0.1, vy_mps=-0.2, omega_rad_s=0.3)

    assert twist_is_finite(twist)


def test_twist_is_finite_false_for_nan_vx() -> None:
    twist = Twist2D(vx_mps=math.nan, vy_mps=0.0, omega_rad_s=0.0)

    assert not twist_is_finite(twist)


def test_twist_is_finite_false_for_inf_vy() -> None:
    twist = Twist2D(vx_mps=0.0, vy_mps=math.inf, omega_rad_s=0.0)

    assert not twist_is_finite(twist)


def test_twist_is_finite_false_for_nan_omega() -> None:
    twist = Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=math.nan)

    assert not twist_is_finite(twist)


def test_robot_savo_stationary_odom_values_are_valid() -> None:
    pose = Pose2D(x_m=0.0, y_m=0.0, yaw_rad=0.0)
    twist = Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=0.0)

    linear_speed, angular_speed = twist_magnitude(twist)

    assert pose_is_finite(pose)
    assert twist_is_finite(twist)
    assert distance_2d_m(pose, pose) == pytest.approx(0.0)
    assert linear_speed == pytest.approx(0.0)
    assert angular_speed == pytest.approx(0.0)


def test_robot_savo_mecanum_side_motion_twist_is_valid() -> None:
    twist = Twist2D(vx_mps=0.0, vy_mps=0.20, omega_rad_s=0.0)

    linear_speed, angular_speed = twist_magnitude(twist)

    assert twist_is_finite(twist)
    assert linear_speed == pytest.approx(0.20)
    assert angular_speed == pytest.approx(0.0)


def test_robot_savo_rotation_twist_is_valid() -> None:
    twist = Twist2D(vx_mps=0.0, vy_mps=0.0, omega_rad_s=1.0)

    linear_speed, angular_speed = twist_magnitude(twist)

    assert twist_is_finite(twist)
    assert linear_speed == pytest.approx(0.0)
    assert angular_speed == pytest.approx(1.0)