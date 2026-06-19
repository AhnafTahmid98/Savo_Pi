#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Timing helpers for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


# =============================================================================
# Clock helpers
# =============================================================================
def now_s() -> float:
    return time.time()


def monotonic_s() -> float:
    return time.monotonic()


def age_s(timestamp_s: Optional[float], now: Optional[float] = None) -> Optional[float]:
    if timestamp_s is None:
        return None

    current = now_s() if now is None else float(now)
    return max(0.0, current - float(timestamp_s))


def is_stale(
    timestamp_s: Optional[float],
    timeout_s: float,
    now: Optional[float] = None,
) -> bool:
    sample_age = age_s(timestamp_s, now=now)

    if sample_age is None:
        return True

    return sample_age > max(0.0, float(timeout_s))


# =============================================================================
# Rate tracking
# =============================================================================
@dataclass
class RateTracker:
    window_s: float = 5.0
    _first_stamp_s: Optional[float] = None
    _last_stamp_s: Optional[float] = None
    _count: int = 0

    def tick(self, stamp_s: Optional[float] = None) -> None:
        stamp = monotonic_s() if stamp_s is None else float(stamp_s)

        if self._first_stamp_s is None:
            self._first_stamp_s = stamp

        self._last_stamp_s = stamp
        self._count += 1

        self._trim_if_needed(stamp)

    @property
    def count(self) -> int:
        return self._count

    @property
    def last_stamp_s(self) -> Optional[float]:
        return self._last_stamp_s

    @property
    def rate_hz(self) -> float:
        if self._first_stamp_s is None or self._last_stamp_s is None:
            return 0.0

        elapsed = self._last_stamp_s - self._first_stamp_s

        if elapsed <= 0.0:
            return 0.0

        return max(0.0, (self._count - 1) / elapsed)

    def reset(self) -> None:
        self._first_stamp_s = None
        self._last_stamp_s = None
        self._count = 0

    def _trim_if_needed(self, current_stamp_s: float) -> None:
        if self._first_stamp_s is None:
            return

        if current_stamp_s - self._first_stamp_s <= self.window_s:
            return

        self._first_stamp_s = current_stamp_s
        self._count = 1


# =============================================================================
# Stale monitor
# =============================================================================
@dataclass
class StaleMonitor:
    timeout_s: float
    last_update_s: Optional[float] = None

    def update(self, stamp_s: Optional[float] = None) -> None:
        self.last_update_s = now_s() if stamp_s is None else float(stamp_s)

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        return age_s(self.last_update_s, now=now)

    def stale(self, now: Optional[float] = None) -> bool:
        return is_stale(self.last_update_s, self.timeout_s, now=now)

    def reset(self) -> None:
        self.last_update_s = None


# =============================================================================
# Simple timer
# =============================================================================
@dataclass
class SimpleTimer:
    started_s: float = field(default_factory=monotonic_s)

    def reset(self) -> None:
        self.started_s = monotonic_s()

    def elapsed_s(self) -> float:
        return max(0.0, monotonic_s() - self.started_s)

    def expired(self, timeout_s: float) -> bool:
        return self.elapsed_s() >= max(0.0, float(timeout_s))


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    tracker = RateTracker()
    tracker.tick(0.0)
    tracker.tick(0.5)
    tracker.tick(1.0)

    monitor = StaleMonitor(timeout_s=0.5)
    monitor.update(stamp_s=10.0)

    print(f"rate_hz={tracker.rate_hz:.2f}")
    print(f"age_s={monitor.age_s(now=10.25):.2f}")
    print(f"stale={monitor.stale(now=11.0)}")


if __name__ == "__main__":
    main()


__all__ = [
    "now_s",
    "monotonic_s",
    "age_s",
    "is_stale",
    "RateTracker",
    "StaleMonitor",
    "SimpleTimer",
    "main",
]