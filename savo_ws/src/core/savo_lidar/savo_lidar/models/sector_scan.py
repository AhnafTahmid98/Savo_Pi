# -*- coding: utf-8 -*-
"""Angle-sector summary for LiDAR diagnostics."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class SectorScan:
    name: str
    angle_min_deg: float
    angle_max_deg: float

    total_points: int = 0
    valid_points: int = 0
    valid_ratio: float = 0.0

    nearest_range_m: float | None = None
    mean_range_m: float | None = None

    blocked: bool = False
    message: str = ""

    @property
    def has_valid_data(self) -> bool:
        return self.valid_points > 0

    @property
    def clear(self) -> bool:
        return self.has_valid_data and not self.blocked

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def make_sector_scan(
    *,
    name: str,
    angle_min_deg: float,
    angle_max_deg: float,
    ranges_m: Iterable[float],
    blocked_distance_m: float,
) -> SectorScan:
    ranges = list(ranges_m)
    total_points = len(ranges)

    valid_ranges = [_as_float(value) for value in ranges if _is_valid_range(value)]

    valid_points = len(valid_ranges)
    valid_ratio = 0.0
    if total_points > 0:
        valid_ratio = valid_points / float(total_points)

    nearest_range_m: float | None = None
    mean_range_m: float | None = None
    blocked = False

    if valid_ranges:
        nearest_range_m = min(valid_ranges)
        mean_range_m = sum(valid_ranges) / float(valid_points)
        blocked = nearest_range_m <= float(blocked_distance_m)

    if total_points == 0:
        message = "sector has no scan points"
    elif not valid_ranges:
        message = "sector has no valid ranges"
    elif blocked:
        message = "sector blocked"
    else:
        message = "sector clear"

    return SectorScan(
        name=str(name),
        angle_min_deg=float(angle_min_deg),
        angle_max_deg=float(angle_max_deg),
        total_points=total_points,
        valid_points=valid_points,
        valid_ratio=valid_ratio,
        nearest_range_m=nearest_range_m,
        mean_range_m=mean_range_m,
        blocked=blocked,
        message=message,
    )


def _is_valid_range(value: object) -> bool:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(v) and v > 0.0


def _as_float(value: object) -> float:
    return float(value)


__all__ = [
    "SectorScan",
    "make_sector_scan",
]
