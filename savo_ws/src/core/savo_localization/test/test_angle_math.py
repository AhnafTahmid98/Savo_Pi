#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for angle helpers used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.math.angle_math import (
    angle_close_rad,
    clamp_angle_rad,
    deg_to_rad,
    rad_to_deg,
    shortest_angle_delta_rad,
    wrap_angle_rad,
)


def test_deg_to_rad() -> None:
    assert deg_to_rad(0.0) == pytest.approx(0.0)
    assert deg_to_rad(90.0) == pytest.approx(math.pi / 2.0)
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert deg_to_rad(-90.0) == pytest.approx(-math.pi / 2.0)


def test_rad_to_deg() -> None:
    assert rad_to_deg(0.0) == pytest.approx(0.0)
    assert rad_to_deg(math.pi / 2.0) == pytest.approx(90.0)
    assert rad_to_deg(math.pi) == pytest.approx(180.0)
    assert rad_to_deg(-math.pi / 2.0) == pytest.approx(-90.0)


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (2.0 * math.pi, 0.0),
        (-2.0 * math.pi, 0.0),
        (3.0 * math.pi, -math.pi),
        (-3.0 * math.pi, -math.pi),
        (math.pi / 2.0, math.pi / 2.0),
        (-math.pi / 2.0, -math.pi / 2.0),
    ],
)
def test_wrap_angle_rad(value: float, expected: float) -> None:
    assert wrap_angle_rad(value) == pytest.approx(expected)


def test_wrap_angle_rad_handles_multiple_turns() -> None:
    assert wrap_angle_rad(10.0 * math.pi) == pytest.approx(0.0)
    assert wrap_angle_rad(-10.0 * math.pi) == pytest.approx(0.0)
    assert wrap_angle_rad(10.5 * math.pi) == pytest.approx(math.pi / 2.0)


def test_shortest_angle_delta_rad_same_angle() -> None:
    assert shortest_angle_delta_rad(0.0, 0.0) == pytest.approx(0.0)
    assert shortest_angle_delta_rad(math.pi / 2.0, math.pi / 2.0) == pytest.approx(0.0)


def test_shortest_angle_delta_rad_positive_turn() -> None:
    current = deg_to_rad(10.0)
    target = deg_to_rad(40.0)

    assert shortest_angle_delta_rad(current, target) == pytest.approx(deg_to_rad(30.0))


def test_shortest_angle_delta_rad_negative_turn() -> None:
    current = deg_to_rad(40.0)
    target = deg_to_rad(10.0)

    assert shortest_angle_delta_rad(current, target) == pytest.approx(deg_to_rad(-30.0))


def test_shortest_angle_delta_rad_wraps_across_positive_boundary() -> None:
    current = deg_to_rad(170.0)
    target = deg_to_rad(-170.0)

    assert shortest_angle_delta_rad(current, target) == pytest.approx(deg_to_rad(20.0))


def test_shortest_angle_delta_rad_wraps_across_negative_boundary() -> None:
    current = deg_to_rad(-170.0)
    target = deg_to_rad(170.0)

    assert shortest_angle_delta_rad(current, target) == pytest.approx(deg_to_rad(-20.0))


def test_clamp_angle_rad_within_limit() -> None:
    assert clamp_angle_rad(deg_to_rad(10.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(10.0)
    )


def test_clamp_angle_rad_positive_limit() -> None:
    assert clamp_angle_rad(deg_to_rad(60.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(30.0)
    )


def test_clamp_angle_rad_negative_limit() -> None:
    assert clamp_angle_rad(deg_to_rad(-60.0), deg_to_rad(30.0)) == pytest.approx(
        deg_to_rad(-30.0)
    )


def test_clamp_angle_rad_rejects_negative_limit() -> None:
    with pytest.raises(ValueError):
        clamp_angle_rad(0.1, -0.1)


def test_angle_close_rad_true_for_small_error() -> None:
    assert angle_close_rad(
        deg_to_rad(179.0),
        deg_to_rad(-179.0),
        tolerance_rad=deg_to_rad(3.0),
    )


def test_angle_close_rad_false_for_large_error() -> None:
    assert not angle_close_rad(
        deg_to_rad(90.0),
        deg_to_rad(120.0),
        tolerance_rad=deg_to_rad(5.0),
    )


def test_angle_close_rad_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError):
        angle_close_rad(0.0, 0.0, tolerance_rad=-0.1)
