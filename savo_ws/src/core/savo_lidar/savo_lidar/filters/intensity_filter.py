"""Intensity helpers for LaserScan data."""

from __future__ import annotations

import math
from collections.abc import Iterable


def has_intensity_data(intensities: Iterable[float]) -> bool:
    return any(math.isfinite(float(value)) and float(value) > 0.0 for value in intensities)


def filter_intensity_value(
    value: float,
    *,
    min_intensity: float,
    invalid_value: float = 0.0,
) -> float:
    value = float(value)

    if math.isfinite(value) and value >= float(min_intensity):
        return value

    return float(invalid_value)


def filter_intensities(
    intensities: Iterable[float],
    *,
    min_intensity: float,
    invalid_value: float = 0.0,
) -> list[float]:
    return [
        filter_intensity_value(
            value,
            min_intensity=min_intensity,
            invalid_value=invalid_value,
        )
        for value in intensities
    ]


def intensity_stats(intensities: Iterable[float]) -> tuple[int, int, float | None, float | None, float | None]:
    values = [float(value) for value in intensities]
    valid = [value for value in values if math.isfinite(value) and value > 0.0]

    total_points = len(values)
    valid_points = len(valid)

    if not valid:
        return total_points, valid_points, None, None, None

    min_value = min(valid)
    max_value = max(valid)
    mean_value = sum(valid) / float(valid_points)

    return total_points, valid_points, min_value, max_value, mean_value