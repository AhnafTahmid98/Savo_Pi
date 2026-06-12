import math

import pytest

from savo_lidar.filters.angle_filter import (
    angle_in_sector,
    degrees_to_radians,
    normalize_angle_deg,
    radians_to_degrees,
    scan_angle_deg,
    sector_indices,
)


def test_normalize_angle_deg_keeps_angle_inside_robot_range():
    assert normalize_angle_deg(0.0) == 0.0
    assert normalize_angle_deg(180.0) == 180.0
    assert normalize_angle_deg(181.0) == -179.0
    assert normalize_angle_deg(-181.0) == 179.0
    assert normalize_angle_deg(540.0) == 180.0


def test_angle_in_sector_accepts_normal_front_sector():
    assert angle_in_sector(0.0, -45.0, 45.0)
    assert angle_in_sector(-45.0, -45.0, 45.0)
    assert angle_in_sector(45.0, -45.0, 45.0)
    assert not angle_in_sector(90.0, -45.0, 45.0)


def test_angle_in_sector_accepts_wraparound_rear_sector():
    assert angle_in_sector(170.0, 135.0, -135.0)
    assert angle_in_sector(-170.0, 135.0, -135.0)
    assert not angle_in_sector(0.0, 135.0, -135.0)


def test_degrees_and_radians_conversion_round_trip():
    angle_deg = 90.0
    angle_rad = degrees_to_radians(angle_deg)

    assert math.isclose(angle_rad, math.pi / 2.0)
    assert math.isclose(radians_to_degrees(angle_rad), angle_deg)


def test_scan_angle_deg_uses_angle_min_and_increment():
    angle = scan_angle_deg(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 2.0,
        index=2,
    )

    assert math.isclose(angle, 0.0)


def test_sector_indices_selects_front_points():
    indices = sector_indices(
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        point_count=8,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
    )

    assert indices == [3, 4, 5]


def test_sector_indices_returns_empty_for_no_points():
    assert (
        sector_indices(
            angle_min_rad=-math.pi,
            angle_increment_rad=math.radians(1.0),
            point_count=0,
            sector_min_deg=-45.0,
            sector_max_deg=45.0,
        )
        == []
    )


def test_sector_indices_rejects_zero_angle_increment():
    with pytest.raises(ValueError):
        sector_indices(
            angle_min_rad=-math.pi,
            angle_increment_rad=0.0,
            point_count=10,
            sector_min_deg=-45.0,
            sector_max_deg=45.0,
        )