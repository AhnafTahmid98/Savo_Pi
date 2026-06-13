#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Timing helpers for localization diagnostics and CLI tools. No ROS imports."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field


def monotonic_now_s() -> float:
    return time.monotonic()


def wall_now_s() -> float:
    return time.time()


def elapsed_s(start_s: float, *, now_s: float | None = None) -> float:
    if now_s is None:
        now_s = monotonic_now_s()

    return max(0.0, float(now_s) - float(start_s))


def age_s(stamp_s: float | None, *, now_s: float | None = None) -> float | None:
    if stamp_s is None:
        return None

    return elapsed_s(stamp_s, now_s=now_s)


def is_stale(
    stamp_s: float | None,
    timeout_s: float,
    *,
    now_s: float | None = None,
) -> bool:
    if timeout_s <= 0.0:
        raise ValueError(f"timeout_s must be > 0.0, got {timeout_s}")

    if stamp_s is None:
        return True

    return elapsed_s(stamp_s, now_s=now_s) > float(timeout_s)


def sleep_until_next_period(start_s: float, period_s: float) -> float:
    if period_s <= 0.0:
        raise ValueError(f"period_s must be > 0.0, got {period_s}")

    remaining_s = float(period_s) - elapsed_s(start_s)

    if remaining_s > 0.0:
        time.sleep(remaining_s)

    return max(0.0, remaining_s)


def hz_to_period_s(rate_hz: float) -> float:
    rate_hz = float(rate_hz)

    if rate_hz <= 0.0:
        raise ValueError(f"rate_hz must be > 0.0, got {rate_hz}")

    return 1.0 / rate_hz


def period_s_to_hz(period_s: float) -> float:
    period_s = float(period_s)

    if period_s <= 0.0:
        return 0.0

    return 1.0 / period_s


@dataclass
class RateTracker:
    window_size: int = 20
    _last_tick_s: float | None = None
    _periods_s: deque[float] = field(default_factory=deque)

    def __post_init__(self) -> None:
        if self.window_size <= 0:
            raise ValueError(f"window_size must be > 0, got {self.window_size}")

    def tick(self, now_s: float | None = None) -> float:
        if now_s is None:
            now_s = monotonic_now_s()

        now_s = float(now_s)

        if self._last_tick_s is None:
            self._last_tick_s = now_s
            return 0.0

        period_s = max(0.0, now_s - self._last_tick_s)
        self._last_tick_s = now_s

        self._periods_s.append(period_s)

        while len(self._periods_s) > self.window_size:
            self._periods_s.popleft()

        return self.rate_hz

    @property
    def rate_hz(self) -> float:
        if not self._periods_s:
            return 0.0

        mean_period_s = sum(self._periods_s) / float(len(self._periods_s))
        return period_s_to_hz(mean_period_s)

    @property
    def last_period_s(self) -> float | None:
        if not self._periods_s:
            return None

        return self._periods_s[-1]

    def reset(self) -> None:
        self._last_tick_s = None
        self._periods_s.clear()


@dataclass
class TimeoutWatch:
    timeout_s: float
    last_update_s: float | None = None

    def __post_init__(self) -> None:
        if self.timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0.0, got {self.timeout_s}")

    def mark(self, stamp_s: float | None = None) -> None:
        if stamp_s is None:
            stamp_s = monotonic_now_s()

        self.last_update_s = float(stamp_s)

    def stale(self, now_s: float | None = None) -> bool:
        return is_stale(self.last_update_s, self.timeout_s, now_s=now_s)

    def age_s(self, now_s: float | None = None) -> float | None:
        return age_s(self.last_update_s, now_s=now_s)

    def reset(self) -> None:
        self.last_update_s = None