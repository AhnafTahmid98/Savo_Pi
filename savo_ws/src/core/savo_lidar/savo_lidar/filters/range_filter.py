# -*- coding: utf-8 -*-
"""Range filtering for LaserScan data."""

from __future__ import annotations

import math
from collections.abc import Iterable


def is_valid_range(value: float, min_range_m: float, max_range_m: float) -> bool:
    try:
        range_m = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(range_m) and float(min_range_m) <= range_m <= float(max_range_m)


def filter_range_value(
    value: float,
    *,
    min_range_m: float,
    max_range_m: float,
    invalid_value: float = float("inf"),
) -> float:
    if is_valid_range(value, min_range_m, max_range_m):
        return float(value)

    return float(invalid_value)


def filter_ranges(
    ranges: Iterable[float],
    *,
    min_range_m: float,
    max_range_m: float,
    invalid_value: float = float("inf"),
) -> list[float]:
    return [
        filter_range_value(
            value,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            invalid_value=invalid_value,
        )
        for value in ranges
    ]


def count_valid_ranges(
    ranges: Iterable[float],
    *,
    min_range_m: float,
    max_range_m: float,
) -> int:
    return sum(
        1
        for value in ranges
        if is_valid_range(value, min_range_m, max_range_m)
    )


def valid_range_ratio(
    ranges: Iterable[float],
    *,
    min_range_m: float,
    max_range_m: float,
) -> float:
    values = list(ranges)

    if not values:
        return 0.0

    valid_count = count_valid_ranges(
        values,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
    )

    return valid_count / float(len(values))


def range_stats(
    ranges: Iterable[float],
    *,
    min_range_m: float,
    max_range_m: float,
) -> tuple[int, int, float, float | None, float | None, float | None]:
    values = list(ranges)
    valid_ranges = [
        float(value)
        for value in values
        if is_valid_range(value, min_range_m, max_range_m)
    ]

    total_points = len(values)
    valid_points = len(valid_ranges)
    ratio = 0.0 if total_points == 0 else valid_points / float(total_points)

    if not valid_ranges:
        return total_points, valid_points, ratio, None, None, None

    min_seen = min(valid_ranges)
    max_seen = max(valid_ranges)
    mean_seen = sum(valid_ranges) / float(valid_points)

    return total_points, valid_points, ratio, min_seen, max_seen, mean_seen


__all__ = [
    "count_valid_ranges",
    "filter_range_value",
    "filter_ranges",
    "is_valid_range",
    "range_stats",
    "valid_range_ratio",
]
