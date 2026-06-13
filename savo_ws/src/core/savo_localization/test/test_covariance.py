#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for covariance helpers used by Robot Savo localization."""

from __future__ import annotations

import math

import pytest

from savo_localization.math.covariance import (
    covariance3_from_diagonal,
    covariance6_from_diagonal,
    covariance_from_diagonal,
    covariance_is_finite,
    diagonal_from_covariance,
    scale_covariance,
)


def test_covariance3_from_diagonal() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)

    assert len(covariance) == 9

    assert covariance[0] == pytest.approx(0.1)
    assert covariance[4] == pytest.approx(0.2)
    assert covariance[8] == pytest.approx(0.3)

    assert covariance[1] == pytest.approx(0.0)
    assert covariance[2] == pytest.approx(0.0)
    assert covariance[3] == pytest.approx(0.0)
    assert covariance[5] == pytest.approx(0.0)
    assert covariance[6] == pytest.approx(0.0)
    assert covariance[7] == pytest.approx(0.0)


def test_covariance6_from_diagonal() -> None:
    covariance = covariance6_from_diagonal(
        x=0.1,
        y=0.2,
        z=999.0,
        roll=999.0,
        pitch=999.0,
        yaw=0.3,
    )

    assert len(covariance) == 36

    assert covariance[0] == pytest.approx(0.1)
    assert covariance[7] == pytest.approx(0.2)
    assert covariance[14] == pytest.approx(999.0)
    assert covariance[21] == pytest.approx(999.0)
    assert covariance[28] == pytest.approx(999.0)
    assert covariance[35] == pytest.approx(0.3)

    off_diagonal = [
        value
        for index, value in enumerate(covariance)
        if index not in (0, 7, 14, 21, 28, 35)
    ]

    assert all(value == pytest.approx(0.0) for value in off_diagonal)


def test_covariance_from_diagonal_generic_2x2() -> None:
    covariance = covariance_from_diagonal([0.5, 0.8])

    assert len(covariance) == 4
    assert covariance == pytest.approx(
        [
            0.5,
            0.0,
            0.0,
            0.8,
        ]
    )


def test_covariance_from_diagonal_generic_3x3() -> None:
    covariance = covariance_from_diagonal([1.0, 2.0, 3.0])

    assert len(covariance) == 9
    assert covariance == pytest.approx(
        [
            1.0,
            0.0,
            0.0,
            0.0,
            2.0,
            0.0,
            0.0,
            0.0,
            3.0,
        ]
    )


def test_diagonal_from_covariance_3x3() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)

    assert diagonal_from_covariance(covariance, size=3) == pytest.approx(
        [0.1, 0.2, 0.3]
    )


def test_diagonal_from_covariance_6x6() -> None:
    covariance = covariance6_from_diagonal(
        x=0.1,
        y=0.2,
        z=999.0,
        roll=999.0,
        pitch=999.0,
        yaw=0.3,
    )

    assert diagonal_from_covariance(covariance, size=6) == pytest.approx(
        [0.1, 0.2, 999.0, 999.0, 999.0, 0.3]
    )


def test_diagonal_from_covariance_rejects_wrong_length() -> None:
    with pytest.raises(ValueError):
        diagonal_from_covariance([1.0, 0.0, 0.0], size=2)


def test_diagonal_from_covariance_rejects_bad_size() -> None:
    with pytest.raises(ValueError):
        diagonal_from_covariance([1.0], size=0)


def test_scale_covariance() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)
    scaled = scale_covariance(covariance, scale=2.0)

    assert scaled == pytest.approx(
        [
            0.2,
            0.0,
            0.0,
            0.0,
            0.4,
            0.0,
            0.0,
            0.0,
            0.6,
        ]
    )


def test_scale_covariance_with_zero_scale() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)
    scaled = scale_covariance(covariance, scale=0.0)

    assert all(value == pytest.approx(0.0) for value in scaled)


def test_scale_covariance_rejects_negative_scale() -> None:
    with pytest.raises(ValueError):
        scale_covariance([1.0, 0.0, 0.0, 1.0], scale=-1.0)


def test_covariance_is_finite_true() -> None:
    covariance = covariance6_from_diagonal(
        x=0.1,
        y=0.2,
        z=999.0,
        roll=999.0,
        pitch=999.0,
        yaw=0.3,
    )

    assert covariance_is_finite(covariance)


def test_covariance_is_finite_false_for_nan() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)
    covariance[4] = math.nan

    assert not covariance_is_finite(covariance)


def test_covariance_is_finite_false_for_inf() -> None:
    covariance = covariance3_from_diagonal(0.1, 0.2, 0.3)
    covariance[8] = math.inf

    assert not covariance_is_finite(covariance)


def test_robot_savo_wheel_odom_covariance_values() -> None:
    pose_covariance = covariance6_from_diagonal(
        x=0.05,
        y=0.10,
        z=999.0,
        roll=999.0,
        pitch=999.0,
        yaw=0.10,
    )

    twist_covariance = covariance6_from_diagonal(
        x=0.05,
        y=0.10,
        z=999.0,
        roll=999.0,
        pitch=999.0,
        yaw=0.10,
    )

    assert len(pose_covariance) == 36
    assert len(twist_covariance) == 36

    assert diagonal_from_covariance(pose_covariance, size=6) == pytest.approx(
        [0.05, 0.10, 999.0, 999.0, 999.0, 0.10]
    )

    assert diagonal_from_covariance(twist_covariance, size=6) == pytest.approx(
        [0.05, 0.10, 999.0, 999.0, 999.0, 0.10]
    )