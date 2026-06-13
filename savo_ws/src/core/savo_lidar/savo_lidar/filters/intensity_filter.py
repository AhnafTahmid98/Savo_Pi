# -*- coding: utf-8 -*-
"""Intensity helpers for LaserScan data."""

from __future__ import annotations

import math
from collections.abc import Iterable


def has_intensity_data(intensities: Iterable[float] | None) -> bool:
    if intensities is None:
        return False

    return any(_is_valid_intensity(value) for value in intensities)


def filter_intensity_value(
    value: float,
    *,
    invalid_value: float = 0.0,
) -> float:
    if _is_valid_intensity(value):
        return float(value)

    return float(invalid_value)


def filter_intensities(
    intensities: Iterable[float],
    *,
    invalid_value: float = 0.0,
) -> list[float]:
    return [
        filter_intensity_value(value, invalid_value=invalid_value)
        for value in intensities
    ]


def intensity_stats(
    intensities: Iterable[float],
) -> tuple[int, int, float, float | None, float | None, float | None]:
    values = list(intensities)
    valid_values = [
        float(value)
        for value in values
        if _is_valid_intensity(value)
    ]

    total_points = len(values)
    valid_points = len(valid_values)
    valid_ratio = 0.0 if total_points == 0 else valid_points / float(total_points)

    if not valid_values:
        return total_points, valid_points, valid_ratio, None, None, None

    min_seen = min(valid_values)
    max_seen = max(valid_values)
    mean_seen = sum(valid_values) / float(valid_points)

    return total_points, valid_points, valid_ratio, min_seen, max_seen, mean_seen


def _is_valid_intensity(value: object) -> bool:
    try:
        intensity = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(intensity) and intensity >= 0.0


__all__ = [
    "filter_intensities",
    "filter_intensity_value",
    "has_intensity_data",
    "intensity_stats",
]
