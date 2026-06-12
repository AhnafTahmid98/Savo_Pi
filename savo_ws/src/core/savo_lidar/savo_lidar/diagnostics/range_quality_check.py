"""Range-quality diagnostics for LiDAR scan data."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.filters.range_filter import range_stats
from savo_lidar.models.scan_quality import ScanQuality, make_scan_quality


@dataclass(frozen=True)
class RangeQualityCheckResult:
    total_points: int
    valid_points: int
    valid_ratio: float
    min_range_m: float | None
    max_range_m: float | None
    mean_range_m: float | None
    scan_rate_hz: float
    status: str
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def check_range_quality(
    *,
    ranges: list[float],
    min_range_m: float,
    max_range_m: float,
    scan_rate_hz: float = 0.0,
    warn_ratio: float = 0.60,
    error_ratio: float = 0.30,
) -> RangeQualityCheckResult:
    (
        total_points,
        valid_points,
        _valid_ratio,
        min_seen_m,
        max_seen_m,
        mean_seen_m,
    ) = range_stats(
        ranges,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
    )

    quality: ScanQuality = make_scan_quality(
        total_points=total_points,
        valid_points=valid_points,
        min_range_m=min_seen_m,
        max_range_m=max_seen_m,
        mean_range_m=mean_seen_m,
        scan_rate_hz=scan_rate_hz,
        warn_ratio=warn_ratio,
        error_ratio=error_ratio,
    )

    return RangeQualityCheckResult(
        total_points=quality.total_points,
        valid_points=quality.valid_points,
        valid_ratio=quality.valid_ratio,
        min_range_m=quality.min_range_m,
        max_range_m=quality.max_range_m,
        mean_range_m=quality.mean_range_m,
        scan_rate_hz=quality.scan_rate_hz,
        status=quality.status,
        ok=quality.ok,
        message=quality.message,
    )