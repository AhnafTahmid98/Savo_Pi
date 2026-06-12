"""Policy for turning stale LiDAR scans into clear health states."""

from __future__ import annotations

from dataclasses import dataclass

from savo_lidar.constants import STATUS_OK, STATUS_STALE, STATUS_WARN


@dataclass(frozen=True)
class StaleScanDecision:
    stale: bool
    status: str
    scan_ok: bool
    message: str


@dataclass(frozen=True)
class StaleScanPolicy:
    warn_before_stale_ratio: float = 0.75

    def __post_init__(self) -> None:
        if not 0.0 <= self.warn_before_stale_ratio <= 1.0:
            raise ValueError(
                "warn_before_stale_ratio must be between 0.0 and 1.0, "
                f"got {self.warn_before_stale_ratio}"
            )

    def evaluate(
        self,
        *,
        last_scan_age_s: float | None,
        timeout_s: float,
    ) -> StaleScanDecision:
        if timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0.0, got {timeout_s}")

        if last_scan_age_s is None:
            return StaleScanDecision(
                stale=True,
                status=STATUS_STALE,
                scan_ok=False,
                message="no LiDAR scan received",
            )

        age_s = max(0.0, float(last_scan_age_s))

        if age_s > float(timeout_s):
            return StaleScanDecision(
                stale=True,
                status=STATUS_STALE,
                scan_ok=False,
                message=f"LiDAR scan stale | age_s={age_s:.3f}",
            )

        warn_age_s = float(timeout_s) * self.warn_before_stale_ratio
        if age_s >= warn_age_s:
            return StaleScanDecision(
                stale=False,
                status=STATUS_WARN,
                scan_ok=True,
                message=f"LiDAR scan age near timeout | age_s={age_s:.3f}",
            )

        return StaleScanDecision(
            stale=False,
            status=STATUS_OK,
            scan_ok=True,
            message=f"LiDAR scan fresh | age_s={age_s:.3f}",
        )