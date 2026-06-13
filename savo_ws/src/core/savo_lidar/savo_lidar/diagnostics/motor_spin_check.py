# -*- coding: utf-8 -*-
"""Motor and scan-stream check for RPLIDAR diagnostics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import SCAN_RATE_MIN_HZ


@dataclass(frozen=True)
class MotorSpinCheckResult:
    driver_running: bool
    scan_count: int
    scan_rate_hz: float
    min_scan_rate_hz: float
    ok: bool
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def check_motor_spin(
    *,
    driver_running: bool,
    scan_count: int,
    scan_rate_hz: float,
    min_scan_rate_hz: float = SCAN_RATE_MIN_HZ,
) -> MotorSpinCheckResult:
    scan_count = max(0, int(scan_count))
    scan_rate_hz = max(0.0, float(scan_rate_hz))
    min_scan_rate_hz = max(0.0, float(min_scan_rate_hz))

    if not driver_running:
        return MotorSpinCheckResult(
            driver_running=False,
            scan_count=scan_count,
            scan_rate_hz=scan_rate_hz,
            min_scan_rate_hz=min_scan_rate_hz,
            ok=False,
            message="LiDAR driver is not running",
        )

    if scan_count <= 0:
        return MotorSpinCheckResult(
            driver_running=True,
            scan_count=0,
            scan_rate_hz=scan_rate_hz,
            min_scan_rate_hz=min_scan_rate_hz,
            ok=False,
            message="LiDAR motor may not be spinning; no scans received",
        )

    if min_scan_rate_hz > 0.0 and scan_rate_hz < min_scan_rate_hz:
        return MotorSpinCheckResult(
            driver_running=True,
            scan_count=scan_count,
            scan_rate_hz=scan_rate_hz,
            min_scan_rate_hz=min_scan_rate_hz,
            ok=False,
            message="LiDAR scan stream is slower than expected",
        )

    return MotorSpinCheckResult(
        driver_running=True,
        scan_count=scan_count,
        scan_rate_hz=scan_rate_hz,
        min_scan_rate_hz=min_scan_rate_hz,
        ok=True,
        message="LiDAR scan stream looks healthy",
    )


__all__ = [
    "MotorSpinCheckResult",
    "check_motor_spin",
]
