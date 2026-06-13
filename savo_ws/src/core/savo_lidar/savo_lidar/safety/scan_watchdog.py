# -*- coding: utf-8 -*-
"""Watchdog for stale LiDAR scan streams."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_lidar.utils.timing import elapsed_s, is_stale, monotonic_now_s


@dataclass
class ScanWatchdog:
    timeout_s: float
    last_scan_s: float | None = None
    scan_count: int = 0

    def __post_init__(self) -> None:
        if self.timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0.0, got {self.timeout_s}")

    def mark_scan(self, stamp_s: float | None = None) -> None:
        if stamp_s is None:
            stamp_s = monotonic_now_s()

        self.last_scan_s = float(stamp_s)
        self.scan_count += 1

    def stale(self, now_s: float | None = None) -> bool:
        return is_stale(self.last_scan_s, self.timeout_s, now_s=now_s)

    def age_s(self, now_s: float | None = None) -> float | None:
        if self.last_scan_s is None:
            return None

        return elapsed_s(self.last_scan_s, now_s=now_s)

    def reset(self) -> None:
        self.last_scan_s = None
        self.scan_count = 0

    def status_message(self, now_s: float | None = None) -> str:
        age = self.age_s(now_s=now_s)

        if age is None:
            return "no scan received"

        if self.stale(now_s=now_s):
            return f"scan stream stale | age_s={age:.3f}"

        return f"scan stream healthy | age_s={age:.3f}"

    def to_dict(self, now_s: float | None = None) -> dict[str, Any]:
        age = self.age_s(now_s=now_s)

        return {
            "timeout_s": self.timeout_s,
            "last_scan_s": self.last_scan_s,
            "scan_count": self.scan_count,
            "age_s": age,
            "stale": self.stale(now_s=now_s),
            "message": self.status_message(now_s=now_s),
        }


__all__ = [
    "ScanWatchdog",
]
