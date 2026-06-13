#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for encoder math helpers used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.math.encoder_math import (
    counts_to_revolutions,
    direction_from_delta,
    direction_symbol,
    wheel_circumference_m,
)


def test_wheel_circumference_m_robot_savo_wheel() -> None:
    assert wheel_circumference_m(0.065) == pytest.approx(math.pi * 0.065)


def test_wheel_circumference_m_large_wheel() -> None:
    assert wheel_circumference_m(0.100) == pytest.approx(math.pi * 0.100)


def test_counts_to_revolutions_zero() -> None:
    assert counts_to_revolutions(0, 80) == pytest.approx(0.0)


def test_counts_to_revolutions_one_forward_wheel_turn() -> None:
    assert counts_to_revolutions(80, 80) == pytest.approx(1.0)


def test_counts_to_revolutions_one_reverse_wheel_turn() -> None:
    assert counts_to_revolutions(-80, 80) == pytest.approx(-1.0)


def test_counts_to_revolutions_half_turn() -> None:
    assert counts_to_revolutions(40, 80) == pytest.approx(0.5)
    assert counts_to_revolutions(-40, 80) == pytest.approx(-0.5)


def test_counts_to_revolutions_robot_savo_encoder_default() -> None:
    cpr = 20
    decoding = 4
    counts_per_wheel_rev = cpr * decoding

    assert counts_per_wheel_rev == 80
    assert counts_to_revolutions(20, counts_per_wheel_rev) == pytest.approx(0.25)
    assert counts_to_revolutions(40, counts_per_wheel_rev) == pytest.approx(0.50)
    assert counts_to_revolutions(80, counts_per_wheel_rev) == pytest.approx(1.00)


def test_direction_from_delta_forward() -> None:
    assert direction_from_delta(1) == 1
    assert direction_from_delta(25) == 1


def test_direction_from_delta_reverse() -> None:
    assert direction_from_delta(-1) == -1
    assert direction_from_delta(-25) == -1


def test_direction_from_delta_stopped() -> None:
    assert direction_from_delta(0) == 0


def test_direction_symbol_forward() -> None:
    assert direction_symbol(1) == "+"
    assert direction_symbol(99) == "+"


def test_direction_symbol_reverse() -> None:
    assert direction_symbol(-1) == "-"
    assert direction_symbol(-99) == "-"


def test_direction_symbol_stopped() -> None:
    assert direction_symbol(0) == "0"


def test_one_encoder_count_distance_matches_wheel_geometry() -> None:
    wheel_diameter_m = 0.065
    counts_per_wheel_rev = 80

    circumference_m = wheel_circumference_m(wheel_diameter_m)
    metres_per_count = circumference_m / counts_per_wheel_rev

    assert metres_per_count == pytest.approx((math.pi * 0.065) / 80.0)
    assert metres_per_count > 0.0


def test_full_revolution_distance_matches_circumference() -> None:
    wheel_diameter_m = 0.065
    counts_per_wheel_rev = 80

    revolutions = counts_to_revolutions(80, counts_per_wheel_rev)
    distance_m = revolutions * wheel_circumference_m(wheel_diameter_m)

    assert distance_m == pytest.approx(math.pi * wheel_diameter_m)