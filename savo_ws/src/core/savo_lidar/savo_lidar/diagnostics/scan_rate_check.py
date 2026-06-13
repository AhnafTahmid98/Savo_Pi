# -*- coding: utf-8 -*-
"""Scan-rate diagnostics for LiDAR data streams."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import DEFAULT_SCAN_RATE_HZ
from savo_lidar.filters.scan_rate_filter import (
    ScanRateStatus,
    check_scan_rate,
)


@dataclass(frozen=True)
class ScanRateCheckResult:
    rate_hz: float
    expected_hz: float
    min_allowed_hz: float
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_scan_rate_check(
    *,
    measured_rate_hz: float,
    expected_rate_hz: float = DEFAULT_SCAN_RATE_HZ,
    tolerance_ratio: float = 0.50,
) -> ScanRateCheckResult:
    status: ScanRateStatus = check_scan_rate(
        rate_hz=measured_rate_hz,
        expected_hz=expected_rate_hz,
        tolerance_ratio=tolerance_ratio,
    )

    return ScanRateCheckResult(
        rate_hz=status.rate_hz,
        expected_hz=status.expected_hz,
        min_allowed_hz=status.min_allowed_hz,
        ok=status.ok,
        message=status.message,
    )


__all__ = [
    "ScanRateCheckResult",
    "run_scan_rate_check",
]
