import math

from savo_lidar.filters.sector_extractor import (
    extract_front_sector,
    extract_sector_ranges,
    extract_sector_summary,
)


def test_extract_sector_ranges_selects_front_ranges():
    ranges = [1.0] * 8

    selected = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert selected == [1.0, 1.0, 1.0]


def test_extract_sector_ranges_replaces_invalid_values_with_inf():
    ranges = [1.0, 1.0, 1.0, 0.10, 2.0, 13.0, 1.0, 1.0]

    selected = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
    )

    assert selected[0] == float("inf")
    assert selected[1] == 2.0
    assert selected[2] == float("inf")


def test_extract_sector_summary_reports_clear_sector():
    ranges = [2.0] * 8

    summary = extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.valid_points == 3
    assert summary.nearest_range_m == 2.0
    assert not summary.blocked
    assert summary.message == "sector clear"


def test_extract_sector_summary_reports_blocked_sector():
    ranges = [2.0, 2.0, 2.0, 0.40, 0.45, 2.0, 2.0, 2.0]

    summary = extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        sector_min_deg=-45.0,
        sector_max_deg=45.0,
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.blocked
    assert summary.nearest_range_m == 0.40
    assert summary.message == "sector blocked"


def test_extract_front_sector_uses_default_front_angles():
    ranges = [3.0] * 8

    summary = extract_front_sector(
        ranges=ranges,
        angle_min_rad=-math.pi,
        angle_increment_rad=math.radians(45.0),
        min_range_m=0.15,
        max_range_m=12.0,
        blocked_distance_m=0.50,
    )

    assert summary.name == "front"
    assert summary.angle_min_deg == -45.0
    assert summary.angle_max_deg == 45.0
    assert summary.valid_points == 3