# -*- coding: utf-8 -*-

import math

import pytest

from savo_lidar.filters import (
    angle_in_sector,
    degrees_to_radians,
    normalize_angle_deg,
    radians_to_degrees,
    scan_angle_deg,
    sector_indices,
)


def test_normalize_angle_keeps_zero():
    assert normalize_angle_deg(0.0) == 0.0


def test_normalize_angle_keeps_positive_inside_range():
    assert normalize_angle_deg(90.0) == 90.0


def test_normalize_angle_keeps_negative_inside_range():
    assert normalize_angle_deg(-90.0) == -90.0


def test_normalize_angle_wraps_above_180():
    assert normalize_angle_deg(190.0) == -170.0
    assert normalize_angle_deg(360.0) == 0.0
    assert normalize_angle_deg(540.0) == 180.0


def test_normalize_angle_wraps_below_minus_180():
    assert normalize_angle_deg(-190.0) == 170.0
    assert normalize_angle_deg(-360.0) == 0.0
    assert normalize_angle_deg(-540.0) == 180.0


def test_angle_in_normal_sector():
    assert angle_in_sector(0.0, -45.0, 45.0)
    assert angle_in_sector(-45.0, -45.0, 45.0)
    assert angle_in_sector(45.0, -45.0, 45.0)
    assert not angle_in_sector(90.0, -45.0, 45.0)


def test_angle_in_wrapped_sector():
    assert angle_in_sector(170.0, 135.0, -135.0)
    assert angle_in_sector(-170.0, 135.0, -135.0)
    assert angle_in_sector(180.0, 135.0, -135.0)
    assert not angle_in_sector(0.0, 135.0, -135.0)


def test_angle_in_sector_normalizes_inputs():
    assert angle_in_sector(370.0, -45.0, 45.0)
    assert angle_in_sector(-350.0, -45.0, 45.0)


def test_degrees_to_radians():
    assert degrees_to_radians(180.0) == pytest.approx(math.pi)
    assert degrees_to_radians(90.0) == pytest.approx(math.pi / 2.0)


def test_radians_to_degrees():
    assert radians_to_degrees(math.pi) == pytest.approx(180.0)
    assert radians_to_degrees(math.pi / 2.0) == pytest.approx(90.0)


def test_scan_angle_deg_for_front_index():
    angle = scan_angle_deg(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        index=180,
    )

    assert angle == pytest.approx(0.0)


def test_scan_angle_deg_for_left_index():
    angle = scan_angle_deg(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        index=270,
    )

    assert angle == pytest.approx(90.0)


def test_scan_angle_deg_wraps_output():
    angle = scan_angle_deg(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        index=360,
    )

    assert angle == pytest.approx(180.0)


def test_sector_indices_returns_empty_for_zero_points():
    indices = sector_indices(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        point_count=0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
    )

    assert indices == []


def test_sector_indices_rejects_zero_angle_increment():
    with pytest.raises(ValueError):
        sector_indices(
            angle_min_rad=-math.pi,
            angle_increment_rad=0.0,
            point_count=360,
            sector_min_deg=-45.0,
            sector_max_deg=45.0,
        )


def test_sector_indices_extracts_front_sector():
    indices = sector_indices(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        point_count=360,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
    )

    assert len(indices) == 91
    assert 180 in indices
    assert min(indices) == 135
    assert max(indices) == 225


def test_sector_indices_extracts_left_sector():
    indices = sector_indices(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        point_count=360,
        sector_min_deg=45.0,
        sector_max_deg=135.0,
    )

    assert len(indices) == 91
    assert 270 in indices
    assert min(indices) == 225
    assert max(indices) == 315


def test_sector_indices_extracts_wrapped_back_sector():
    indices = sector_indices(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        point_count=360,
        sector_min_deg=135.0,
        sector_max_deg=-135.0,
    )

    assert len(indices) == 91
    assert 0 in indices
    assert 359 in indices
