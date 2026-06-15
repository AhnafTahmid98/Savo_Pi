# -*- coding: utf-8 -*-

import math

import pytest

from savo_lidar.filters import (
    extract_front_sector,
    extract_sector_ranges,
    extract_sector_summary,
)


def test_extract_sector_ranges_returns_front_values():
    ranges = [1.0] * 360

    selected = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert len(selected) == 91
    assert all(value == 1.0 for value in selected)


def test_extract_sector_ranges_replaces_invalid_values_with_inf():
    ranges = [1.0, 1.0, 1.0, "bad", 2.0, 13.0, 1.0, 1.0]

    selected = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert len(selected) == 3
    assert math.isinf(selected[0])
    assert selected[1] == 2.0
    assert math.isinf(selected[2])


def test_extract_sector_ranges_handles_wrapped_back_sector():
    ranges = [1.0] * 360

    selected = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=135.0,
        sector_max_deg=-135.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert len(selected) == 91


def test_extract_sector_ranges_returns_empty_for_empty_ranges():
    selected = extract_sector_ranges(
        ranges=[],
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert selected == []


def test_extract_sector_ranges_rejects_zero_angle_increment():
    with pytest.raises(ValueError):
        extract_sector_ranges(
            ranges=[1.0] * 10,
            angle_min_rad=-math.pi,
            angle_increment_rad=0.0,
            sector_min_deg=-45.0,
            sector_max_deg=45.0,
            min_range_m=0.15,
            max_range_m=12.0,
        )


def test_extract_sector_summary_reports_clear_sector():
    ranges = [1.0] * 360

    summary = extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.valid_points == 91
    assert summary.nearest_range_m == 1.0
    assert not summary.blocked
    assert summary.clear
    assert summary.message == "sector clear"


def test_extract_sector_summary_reports_blocked_sector():
    ranges = [1.0] * 360
    ranges[180] = 0.30

    summary = extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.nearest_range_m == 0.30
    assert summary.blocked
    assert not summary.clear
    assert summary.message == "sector blocked"


def test_extract_sector_summary_ignores_invalid_ranges():
    ranges = [float("inf")] * 360
    ranges[180] = 2.0

    summary = extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.total_points == 1
    assert summary.valid_points == 1
    assert summary.nearest_range_m == 2.0
    assert not summary.blocked
    assert summary.message == "sector clear"


def test_extract_front_sector_uses_default_front_angles():
    ranges = [1.0] * 360
    ranges[180] = 0.40

    summary = extract_front_sector(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.angle_min_deg == -45.0
    assert summary.angle_max_deg == 45.0
    assert summary.nearest_range_m == 0.40
    assert summary.blocked


def test_extract_front_sector_supports_custom_front_angles():
    ranges = [1.0] * 360
    ranges[180] = 0.40

    summary = extract_front_sector(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.pi / 180.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
        sector_min_deg=-20.0,
        sector_max_deg=20.0,
    )

    assert summary.name == "front"
    assert summary.angle_min_deg == -20.0
    assert summary.angle_max_deg == 20.0
    assert summary.nearest_range_m == 0.40
    assert summary.blocked
