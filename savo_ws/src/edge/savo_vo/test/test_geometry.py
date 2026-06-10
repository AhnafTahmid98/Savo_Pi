"""Tests for geometry helpers."""

import pytest

from savo_vo.utils.geometry import (
    heading_from_delta_rad,
    normalize_angle_rad,
    planar_distance_m,
    rotate_point_2d,
    transform_point_2d,
    yaw_delta_rad,
)


def test_normalize_angle_keeps_angle_inside_range() -> None:
    assert normalize_angle_rad(0.5) == pytest.approx(0.5)
    assert normalize_angle_rad(-0.5) == pytest.approx(-0.5)


def test_normalize_angle_wraps_positive_angle() -> None:
    assert normalize_angle_rad(3.5) == pytest.approx(-2.7831853071795862)


def test_normalize_angle_wraps_negative_angle() -> None:
    assert normalize_angle_rad(-3.5) == pytest.approx(2.7831853071795862)


def test_planar_distance_uses_euclidean_distance() -> None:
    assert planar_distance_m(
        first_x_m=0.0,
        first_y_m=0.0,
        second_x_m=3.0,
        second_y_m=4.0,
    ) == pytest.approx(5.0)


def test_yaw_delta_uses_wrapped_difference() -> None:
    assert yaw_delta_rad(
        first_yaw_rad=3.10,
        second_yaw_rad=-3.10,
    ) == pytest.approx(0.08318530717958605)


def test_heading_from_delta() -> None:
    assert heading_from_delta_rad(1.0, 0.0) == pytest.approx(0.0)
    assert heading_from_delta_rad(0.0, 1.0) == pytest.approx(1.5707963267948966)


def test_rotate_point_2d_by_zero() -> None:
    x_m, y_m = rotate_point_2d(
        x_m=1.0,
        y_m=2.0,
        yaw_rad=0.0,
    )

    assert x_m == pytest.approx(1.0)
    assert y_m == pytest.approx(2.0)


def test_rotate_point_2d_by_ninety_degrees() -> None:
    x_m, y_m = rotate_point_2d(
        x_m=1.0,
        y_m=0.0,
        yaw_rad=1.5707963267948966,
    )

    assert x_m == pytest.approx(0.0, abs=1e-9)
    assert y_m == pytest.approx(1.0)


def test_transform_point_2d_rotates_then_translates() -> None:
    x_m, y_m = transform_point_2d(
        x_m=1.0,
        y_m=0.0,
        translation_x_m=2.0,
        translation_y_m=3.0,
        yaw_rad=1.5707963267948966,
    )

    assert x_m == pytest.approx(2.0, abs=1e-9)
    assert y_m == pytest.approx(4.0)