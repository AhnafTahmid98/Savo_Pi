"""Scan quality summary for LiDAR diagnostics and dashboards."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import (
    DEFAULT_QUALITY_ERROR_VALID_RATIO,
    DEFAULT_QUALITY_WARN_VALID_RATIO,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)


@dataclass(frozen=True)
class ScanQuality:
    total_points: int = 0
    valid_points: int = 0
    valid_ratio: float = 0.0
    min_range_m: float | None = None
    max_range_m: float | None = None
    mean_range_m: float | None = None
    scan_rate_hz: float = 0.0
    status: str = STATUS_ERROR
    message: str = "no scan received"

    @property
    def ok(self) -> bool:
        return self.status == STATUS_OK

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def quality_status(
    valid_ratio: float,
    *,
    warn_ratio: float = DEFAULT_QUALITY_WARN_VALID_RATIO,
    error_ratio: float = DEFAULT_QUALITY_ERROR_VALID_RATIO,
) -> tuple[str, str]:
    valid_ratio = float(valid_ratio)

    if valid_ratio < float(error_ratio):
        return STATUS_ERROR, "too few valid LiDAR points"

    if valid_ratio < float(warn_ratio):
        return STATUS_WARN, "LiDAR scan quality is low"

    return STATUS_OK, "LiDAR scan quality is good"


def make_scan_quality(
    *,
    total_points: int,
    valid_points: int,
    min_range_m: float | None,
    max_range_m: float | None,
    mean_range_m: float | None,
    scan_rate_hz: float,
    warn_ratio: float = DEFAULT_QUALITY_WARN_VALID_RATIO,
    error_ratio: float = DEFAULT_QUALITY_ERROR_VALID_RATIO,
) -> ScanQuality:
    total_points = max(0, int(total_points))
    valid_points = max(0, int(valid_points))

    if total_points == 0:
        valid_ratio = 0.0
    else:
        valid_ratio = min(1.0, valid_points / float(total_points))

    status, message = quality_status(
        valid_ratio,
        warn_ratio=warn_ratio,
        error_ratio=error_ratio,
    )

    return ScanQuality(
        total_points=total_points,
        valid_points=valid_points,
        valid_ratio=valid_ratio,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        mean_range_m=mean_range_m,
        scan_rate_hz=float(scan_rate_hz),
        status=status,
        message=message,
    )