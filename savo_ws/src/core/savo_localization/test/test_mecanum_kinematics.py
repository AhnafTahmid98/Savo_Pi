#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for mecanum kinematics used by Robot Savo localization."""

from __future__ import annotations

import pytest

from savo_localization.math.mecanum_kinematics import (
    BodyVelocity,
    MecanumGeometry,
    WheelSpeeds,
    body_velocity_from_wheel_speeds,
    inverse_kinematics,
    wheel_speeds_from_body_velocity,
)


def test_mecanum_geometry_valid_defaults() -> None:
    geometry = MecanumGeometry()

    assert geometry.wheelbase_m > 0.0
    assert geometry.track_m > 0.0
    assert geometry.radius_sum_m > 0.0


def test_mecanum_geometry_rejects_bad_values() -> None:
    with pytest.raises(ValueError):
        MecanumGeometry(wheelbase_m=0.0)

    with pytest.raises(ValueError):
        MecanumGeometry(track_m=0.0)


def test_forward_motion_from_equal_wheel_speeds() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=0.20,
            fr_mps=0.20,
            rl_mps=0.20,
            rr_mps=0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(0.20)
    assert velocity.vy_mps == pytest.approx(0.0)
    assert velocity.omega_rad_s == pytest.approx(0.0)


def test_reverse_motion_from_equal_negative_wheel_speeds() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=-0.20,
            fr_mps=-0.20,
            rl_mps=-0.20,
            rr_mps=-0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(-0.20)
    assert velocity.vy_mps == pytest.approx(0.0)
    assert velocity.omega_rad_s == pytest.approx(0.0)


def test_strafe_left_pattern() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=-0.20,
            fr_mps=0.20,
            rl_mps=0.20,
            rr_mps=-0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(0.0)
    assert velocity.vy_mps == pytest.approx(0.20)
    assert velocity.omega_rad_s == pytest.approx(0.0)


def test_strafe_right_pattern() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=0.20,
            fr_mps=-0.20,
            rl_mps=-0.20,
            rr_mps=0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(0.0)
    assert velocity.vy_mps == pytest.approx(-0.20)
    assert velocity.omega_rad_s == pytest.approx(0.0)


def test_rotate_ccw_pattern() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)
    radius_sum = geometry.radius_sum_m

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=-0.20,
            fr_mps=0.20,
            rl_mps=-0.20,
            rr_mps=0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(0.0)
    assert velocity.vy_mps == pytest.approx(0.0)
    assert velocity.omega_rad_s == pytest.approx(0.20 / radius_sum)


def test_rotate_cw_pattern() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)
    radius_sum = geometry.radius_sum_m

    velocity = body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=0.20,
            fr_mps=-0.20,
            rl_mps=0.20,
            rr_mps=-0.20,
        ),
        geometry,
    )

    assert velocity.vx_mps == pytest.approx(0.0)
    assert velocity.vy_mps == pytest.approx(0.0)
    assert velocity.omega_rad_s == pytest.approx(-0.20 / radius_sum)


def test_inverse_kinematics_forward() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    wheels = wheel_speeds_from_body_velocity(
        BodyVelocity(vx_mps=0.20, vy_mps=0.0, omega_rad_s=0.0),
        geometry,
    )

    assert wheels.fl_mps == pytest.approx(0.20)
    assert wheels.fr_mps == pytest.approx(0.20)
    assert wheels.rl_mps == pytest.approx(0.20)
    assert wheels.rr_mps == pytest.approx(0.20)


def test_inverse_kinematics_strafe_left() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    wheels = wheel_speeds_from_body_velocity(
        BodyVelocity(vx_mps=0.0, vy_mps=0.20, omega_rad_s=0.0),
        geometry,
    )

    assert wheels.fl_mps == pytest.approx(-0.20)
    assert wheels.fr_mps == pytest.approx(0.20)
    assert wheels.rl_mps == pytest.approx(0.20)
    assert wheels.rr_mps == pytest.approx(-0.20)


def test_inverse_kinematics_rotate_ccw() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)
    omega = 1.0
    rotation_component = geometry.radius_sum_m * omega

    wheels = wheel_speeds_from_body_velocity(
        BodyVelocity(vx_mps=0.0, vy_mps=0.0, omega_rad_s=omega),
        geometry,
    )

    assert wheels.fl_mps == pytest.approx(-rotation_component)
    assert wheels.fr_mps == pytest.approx(rotation_component)
    assert wheels.rl_mps == pytest.approx(-rotation_component)
    assert wheels.rr_mps == pytest.approx(rotation_component)


def test_forward_inverse_round_trip() -> None:
    geometry = MecanumGeometry(wheelbase_m=0.165, track_m=0.165)

    original = BodyVelocity(vx_mps=0.18, vy_mps=-0.07, omega_rad_s=0.35)

    wheels = inverse_kinematics(original, geometry)
    recovered = body_velocity_from_wheel_speeds(wheels, geometry)

    assert recovered.vx_mps == pytest.approx(original.vx_mps)
    assert recovered.vy_mps == pytest.approx(original.vy_mps)
    assert recovered.omega_rad_s == pytest.approx(original.omega_rad_s)