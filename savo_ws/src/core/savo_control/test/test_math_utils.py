#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for shared math utilities."""

from __future__ import annotations

from math import inf, isclose, nan, pi

from savo_control.utils import (
    apply_deadband,
    clamp,
    clamp_abs,
    finite_or_zero,
    hypot2,
    lerp,
    near_zero,
    safe_divide,
    shortest_angle_error_rad,
    sign,
    within_tolerance,
    wrap_angle_rad,
)


def test_finite_or_zero():
    assert finite_or_zero(1.25) == 1.25
    assert finite_or_zero(-0.5) == -0.5
    assert finite_or_zero(nan) == 0.0
    assert finite_or_zero(inf) == 0.0
    assert finite_or_zero(-inf) == 0.0


def test_clamp_normal_range():
    assert clamp(0.5, -1.0, 1.0) == 0.5
    assert clamp(2.0, -1.0, 1.0) == 1.0
    assert clamp(-2.0, -1.0, 1.0) == -1.0


def test_clamp_accepts_reversed_limits():
    assert clamp(2.0, 1.0, -1.0) == 1.0
    assert clamp(-2.0, 1.0, -1.0) == -1.0
    assert clamp(0.5, 1.0, -1.0) == 0.5


def test_clamp_abs():
    assert clamp_abs(0.5, 1.0) == 0.5
    assert clamp_abs(2.0, 1.0) == 1.0
    assert clamp_abs(-2.0, 1.0) == -1.0
    assert clamp_abs(2.0, -1.0) == 1.0


def test_apply_deadband():
    assert apply_deadband(0.004, 0.005) == 0.0
    assert apply_deadband(-0.004, 0.005) == 0.0
    assert apply_deadband(0.006, 0.005) == 0.006
    assert apply_deadband(-0.006, 0.005) == -0.006


def test_apply_deadband_accepts_negative_threshold():
    assert apply_deadband(0.004, -0.005) == 0.0
    assert apply_deadband(0.006, -0.005) == 0.006


def test_sign():
    assert sign(2.0) == 1
    assert sign(-2.0) == -1
    assert sign(0.0) == 0
    assert sign(0.0, zero=1) == 1
    assert sign(0.0, zero=-1) == -1


def test_wrap_angle_rad_basic_values():
    assert isclose(wrap_angle_rad(0.0), 0.0, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(pi), pi, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(-pi), pi, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(3.0 * pi), pi, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(-3.0 * pi), pi, abs_tol=1e-9)


def test_wrap_angle_rad_keeps_result_in_range():
    for angle in [
        -10.0 * pi,
        -2.5 * pi,
        -1.5 * pi,
        -0.5 * pi,
        0.5 * pi,
        1.5 * pi,
        2.5 * pi,
        10.0 * pi,
    ]:
        wrapped = wrap_angle_rad(angle)
        assert -pi <= wrapped <= pi


def test_shortest_angle_error_rad():
    assert isclose(shortest_angle_error_rad(pi, 0.0), pi, abs_tol=1e-9)
    assert isclose(shortest_angle_error_rad(0.0, pi), pi, abs_tol=1e-9)
    assert isclose(shortest_angle_error_rad(pi / 2.0, 0.0), pi / 2.0, abs_tol=1e-9)
    assert isclose(shortest_angle_error_rad(0.0, pi / 2.0), -pi / 2.0, abs_tol=1e-9)


def test_near_zero():
    assert near_zero(0.0) is True
    assert near_zero(1e-10) is True
    assert near_zero(-1e-10) is True
    assert near_zero(1e-4, eps=1e-5) is False


def test_within_tolerance():
    assert within_tolerance(0.60, 0.60, 0.04) is True
    assert within_tolerance(0.56, 0.60, 0.04) is True
    assert within_tolerance(0.64, 0.60, 0.04) is True
    assert within_tolerance(0.55, 0.60, 0.04) is False
    assert within_tolerance(0.65, 0.60, 0.04) is False


def test_within_tolerance_accepts_negative_tolerance():
    assert within_tolerance(0.64, 0.60, -0.04) is True
    assert within_tolerance(0.65, 0.60, -0.04) is False


def test_hypot2():
    assert hypot2(3.0, 4.0) == 5.0
    assert hypot2(nan, 4.0) == 4.0
    assert hypot2(3.0, inf) == 3.0


def test_safe_divide():
    assert safe_divide(4.0, 2.0) == 2.0
    assert safe_divide(-4.0, 2.0) == -2.0
    assert safe_divide(4.0, 0.0) == 0.0
    assert safe_divide(4.0, 0.0, default=-1.0) == -1.0
    assert safe_divide(nan, 2.0, default=-1.0) == -1.0
    assert safe_divide(4.0, inf, default=-1.0) == -1.0


def test_lerp():
    assert lerp(0.0, 10.0, 0.0) == 0.0
    assert lerp(0.0, 10.0, 0.25) == 2.5
    assert lerp(0.0, 10.0, 1.0) == 10.0


def test_lerp_clamps_alpha():
    assert lerp(0.0, 10.0, -1.0) == 0.0
    assert lerp(0.0, 10.0, 2.0) == 10.0
