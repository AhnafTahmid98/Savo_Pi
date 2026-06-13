# -*- coding: utf-8 -*-
"""Health policy for LiDAR hardware and scan-stream state."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import (
    DEFAULT_QUALITY_ERROR_VALID_RATIO,
    DEFAULT_QUALITY_WARN_VALID_RATIO,
    SCAN_RATE_MIN_HZ,
    STATUS_ERROR,
    STATUS_OFFLINE,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)


@dataclass(frozen=True)
class LidarHealthDecision:
    status: str
    hardware_ok: bool
    scan_ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LidarHealthPolicy:
    min_valid_ratio_warn: float = DEFAULT_QUALITY_WARN_VALID_RATIO
    min_valid_ratio_error: float = DEFAULT_QUALITY_ERROR_VALID_RATIO
    min_scan_rate_hz: float = SCAN_RATE_MIN_HZ

    def __post_init__(self) -> None:
        if not 0.0 <= self.min_valid_ratio_error <= 1.0:
            raise ValueError(
                "min_valid_ratio_error must be between 0.0 and 1.0, "
                f"got {self.min_valid_ratio_error}"
            )

        if not 0.0 <= self.min_valid_ratio_warn <= 1.0:
            raise ValueError(
                "min_valid_ratio_warn must be between 0.0 and 1.0, "
                f"got {self.min_valid_ratio_warn}"
            )

        if self.min_valid_ratio_error > self.min_valid_ratio_warn:
            raise ValueError("min_valid_ratio_error cannot be greater than min_valid_ratio_warn")

        if self.min_scan_rate_hz < 0.0:
            raise ValueError(f"min_scan_rate_hz cannot be negative, got {self.min_scan_rate_hz}")

    def evaluate(
        self,
        *,
        hardware_ok: bool,
        driver_running: bool,
        stale: bool,
        scan_rate_hz: float,
        valid_ratio: float,
        fault_latched: bool = False,
        fault_reason: str = "",
    ) -> LidarHealthDecision:
        valid_ratio = max(0.0, min(1.0, float(valid_ratio)))
        scan_rate_hz = max(0.0, float(scan_rate_hz))

        if fault_latched:
            return LidarHealthDecision(
                status=STATUS_ERROR,
                hardware_ok=bool(hardware_ok),
                scan_ok=False,
                message=str(fault_reason).strip() or "LiDAR fault latched",
            )

        if not hardware_ok:
            return LidarHealthDecision(
                status=STATUS_OFFLINE,
                hardware_ok=False,
                scan_ok=False,
                message="LiDAR hardware offline",
            )

        if not driver_running:
            return LidarHealthDecision(
                status=STATUS_OFFLINE,
                hardware_ok=True,
                scan_ok=False,
                message="LiDAR driver is not running",
            )

        if stale:
            return LidarHealthDecision(
                status=STATUS_STALE,
                hardware_ok=True,
                scan_ok=False,
                message="LiDAR scan stream stale",
            )

        if valid_ratio < self.min_valid_ratio_error:
            return LidarHealthDecision(
                status=STATUS_ERROR,
                hardware_ok=True,
                scan_ok=False,
                message="LiDAR scan has too few valid points",
            )

        if valid_ratio < self.min_valid_ratio_warn:
            return LidarHealthDecision(
                status=STATUS_WARN,
                hardware_ok=True,
                scan_ok=True,
                message="LiDAR scan quality is low",
            )

        if self.min_scan_rate_hz > 0.0 and scan_rate_hz < self.min_scan_rate_hz:
            return LidarHealthDecision(
                status=STATUS_WARN,
                hardware_ok=True,
                scan_ok=True,
                message="LiDAR scan rate is below expected range",
            )

        return LidarHealthDecision(
            status=STATUS_OK,
            hardware_ok=True,
            scan_ok=True,
            message="LiDAR healthy",
        )


__all__ = [
    "LidarHealthDecision",
    "LidarHealthPolicy",
]
