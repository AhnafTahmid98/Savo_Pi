"""Timing helpers for watchdogs, rate checks, and diagnostics."""

from __future__ import annotations

import time
from dataclasses import dataclass


def monotonic_now_s() -> float:
    return time.monotonic()


def wall_time_s() -> float:
    return time.time()


def elapsed_s(start_s: float, now_s: float | None = None) -> float:
    if now_s is None:
        now_s = monotonic_now_s()

    return max(0.0, float(now_s) - float(start_s))


def is_stale(last_update_s: float | None, timeout_s: float, now_s: float | None = None) -> bool:
    if last_update_s is None:
        return True

    if timeout_s <= 0.0:
        return False

    if now_s is None:
        now_s = monotonic_now_s()

    return elapsed_s(last_update_s, now_s) > float(timeout_s)


@dataclass
class RateTracker:
    min_dt_s: float = 1e-6
    _last_stamp_s: float | None = None
    _rate_hz: float = 0.0

    def tick(self, stamp_s: float | None = None) -> float:
        if stamp_s is None:
            stamp_s = monotonic_now_s()

        if self._last_stamp_s is None:
            self._last_stamp_s = float(stamp_s)
            self._rate_hz = 0.0
            return self._rate_hz

        dt_s = max(float(stamp_s) - self._last_stamp_s, self.min_dt_s)
        self._last_stamp_s = float(stamp_s)
        self._rate_hz = 1.0 / dt_s

        return self._rate_hz

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    @property
    def last_stamp_s(self) -> float | None:
        return self._last_stamp_s