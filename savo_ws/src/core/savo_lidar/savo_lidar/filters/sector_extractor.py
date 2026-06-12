"""Extract angle sectors from ROS LaserScan-style range data."""

from __future__ import annotations

import math
from collections.abc import Sequence

from savo_lidar.filters.angle_filter import sector_indices
from savo_lidar.filters.range_filter import is_valid_range
from savo_lidar.models.sector_scan import SectorScan, make_sector_scan


def extract_sector_ranges(
    *,
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    sector_min_deg: float,
    sector_max_deg: float,
    min_range_m: float,
    max_range_m: float,
) -> list[float]:
    indices = sector_indices(
        angle_min_rad=angle_min_rad,
        angle_increment_rad=angle_increment_rad,
        point_count=len(ranges),
        sector_min_deg=sector_min_deg,
        sector_max_deg=sector_max_deg,
    )

    selected: list[float] = []

    for index in indices:
        value = float(ranges[index])

        if is_valid_range(value, min_range_m, max_range_m):
            selected.append(value)
        else:
            selected.append(float("inf"))

    return selected


def extract_sector_summary(
    *,
    name: str,
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    sector_min_deg: float,
    sector_max_deg: float,
    min_range_m: float,
    max_range_m: float,
    blocked_distance_m: float,
) -> SectorScan:
    sector_ranges = extract_sector_ranges(
        ranges=ranges,
        angle_min_rad=angle_min_rad,
        angle_increment_rad=angle_increment_rad,
        sector_min_deg=sector_min_deg,
        sector_max_deg=sector_max_deg,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
    )

    clean_ranges = [
        value
        for value in sector_ranges
        if math.isfinite(value) and min_range_m <= value <= max_range_m
    ]

    return make_sector_scan(
        name=name,
        angle_min_deg=sector_min_deg,
        angle_max_deg=sector_max_deg,
        ranges_m=clean_ranges,
        blocked_distance_m=blocked_distance_m,
    )


def extract_front_sector(
    *,
    ranges: Sequence[float],
    angle_min_rad: float,
    angle_increment_rad: float,
    min_range_m: float,
    max_range_m: float,
    blocked_distance_m: float,
    sector_min_deg: float = -45.0,
    sector_max_deg: float = 45.0,
) -> SectorScan:
    return extract_sector_summary(
        name="front",
        ranges=ranges,
        angle_min_rad=angle_min_rad,
        angle_increment_rad=angle_increment_rad,
        sector_min_deg=sector_min_deg,
        sector_max_deg=sector_max_deg,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        blocked_distance_m=blocked_distance_m,
    )