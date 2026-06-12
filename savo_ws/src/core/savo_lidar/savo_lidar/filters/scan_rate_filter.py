"""Scan-rate checks for LiDAR diagnostics and watchdog logic."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ScanRateStatus:
    rate_hz: float
    expected_hz: float
    min_allowed_hz: float
    ok: bool
    message: str


def minimum_allowed_rate(expected_hz: float, tolerance_ratio: float = 0.50) -> float:
    expected_hz = float(expected_hz)
    tolerance_ratio = float(tolerance_ratio)

    if expected_hz <= 0.0:
        raise ValueError(f"expected_hz must be > 0.0, got {expected_hz}")

    if not 0.0 <= tolerance_ratio <= 1.0:
        raise ValueError(f"tolerance_ratio must be between 0.0 and 1.0, got {tolerance_ratio}")

    return expected_hz * tolerance_ratio


def check_scan_rate(
    *,
    rate_hz: float,
    expected_hz: float,
    tolerance_ratio: float = 0.50,
) -> ScanRateStatus:
    rate_hz = max(0.0, float(rate_hz))
    expected_hz = float(expected_hz)
    min_allowed_hz = minimum_allowed_rate(expected_hz, tolerance_ratio)

    if rate_hz <= 0.0:
        return ScanRateStatus(
            rate_hz=rate_hz,
            expected_hz=expected_hz,
            min_allowed_hz=min_allowed_hz,
            ok=False,
            message="no scan rate measured",
        )

    if rate_hz < min_allowed_hz:
        return ScanRateStatus(
            rate_hz=rate_hz,
            expected_hz=expected_hz,
            min_allowed_hz=min_allowed_hz,
            ok=False,
            message="LiDAR scan rate below expected range",
        )

    return ScanRateStatus(
        rate_hz=rate_hz,
        expected_hz=expected_hz,
        min_allowed_hz=min_allowed_hz,
        ok=True,
        message="LiDAR scan rate healthy",
    )


def rate_to_period_s(rate_hz: float) -> float:
    rate_hz = float(rate_hz)

    if rate_hz <= 0.0:
        raise ValueError(f"rate_hz must be > 0.0, got {rate_hz}")

    return 1.0 / rate_hz