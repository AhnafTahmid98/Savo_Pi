# -*- coding: utf-8 -*-
"""Range quality checks for LaserScan data."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Iterable

from savo_lidar.constants import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_QUALITY_ERROR_VALID_RATIO,
    DEFAULT_QUALITY_WARN_VALID_RATIO,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)


@dataclass(frozen=True)
class RangeQualityCheckResult:
    total_points: int
    valid_points: int
    invalid_points: int
    valid_ratio: float

    min_range_m: float | None
    max_range_m: float | None
    mean_range_m: float | None

    warn_ratio: float
    error_ratio: float

    ok: bool
    status: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def check_range_quality(
    ranges_m: Iterable[float],
    *,
    min_range_m: float = DEFAULT_MIN_RANGE_M,
    max_range_m: float = DEFAULT_MAX_RANGE_M,
    warn_ratio: float = DEFAULT_QUALITY_WARN_VALID_RATIO,
    error_ratio: float = DEFAULT_QUALITY_ERROR_VALID_RATIO,
) -> RangeQualityCheckResult:
    ranges = list(ranges_m)
    total_points = len(ranges)

    min_range_m = float(min_range_m)
    max_range_m = float(max_range_m)
    warn_ratio = _clamp_ratio(warn_ratio)
    error_ratio = _clamp_ratio(error_ratio)

    if error_ratio > warn_ratio:
        error_ratio = warn_ratio

    valid_ranges = [
        float(value)
        for value in ranges
        if _is_valid_range(value, min_range_m=min_range_m, max_range_m=max_range_m)
    ]

    valid_points = len(valid_ranges)
    invalid_points = total_points - valid_points

    valid_ratio = 0.0
    if total_points > 0:
        valid_ratio = valid_points / float(total_points)

    min_seen: float | None = None
    max_seen: float | None = None
    mean_seen: float | None = None

    if valid_ranges:
        min_seen = min(valid_ranges)
        max_seen = max(valid_ranges)
        mean_seen = sum(valid_ranges) / float(valid_points)

    if total_points == 0:
        status = STATUS_ERROR
        message = "no range samples"
    elif valid_ratio < error_ratio:
        status = STATUS_ERROR
        message = "too few valid range samples"
    elif valid_ratio < warn_ratio:
        status = STATUS_WARN
        message = "range quality is low"
    else:
        status = STATUS_OK
        message = "range quality is good"

    return RangeQualityCheckResult(
        total_points=total_points,
        valid_points=valid_points,
        invalid_points=invalid_points,
        valid_ratio=valid_ratio,
        min_range_m=min_seen,
        max_range_m=max_seen,
        mean_range_m=mean_seen,
        warn_ratio=warn_ratio,
        error_ratio=error_ratio,
        ok=status == STATUS_OK,
        status=status,
        message=message,
    )


def _is_valid_range(
    value: object,
    *,
    min_range_m: float,
    max_range_m: float,
) -> bool:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(v) and min_range_m <= v <= max_range_m


def _clamp_ratio(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


__all__ = [
    "RangeQualityCheckResult",
    "check_range_quality",
]
