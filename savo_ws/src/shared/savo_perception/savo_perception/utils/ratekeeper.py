#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Loop-rate helper for Python fallback nodes and CLI tools."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from savo_perception.utils.clamp import safe_rate_hz


def monotonic_s() -> float:
    return time.monotonic()


@dataclass
class RateKeeper:
    rate_hz: float
    min_hz: float = 0.1
    _period_s: float = field(init=False)
    _last_start_s: float = field(default_factory=monotonic_s)
    _last_sleep_s: float = 0.0
    _overrun_count: int = 0
    _tick_count: int = 0

    def __post_init__(self) -> None:
        self.set_rate(self.rate_hz)

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def last_sleep_s(self) -> float:
        return self._last_sleep_s

    @property
    def overrun_count(self) -> int:
        return self._overrun_count

    @property
    def tick_count(self) -> int:
        return self._tick_count

    def set_rate(self, rate_hz: float) -> None:
        self.rate_hz = safe_rate_hz(rate_hz, default=self.min_hz, min_hz=self.min_hz)
        self._period_s = 1.0 / self.rate_hz

    def mark_start(self) -> float:
        self._last_start_s = monotonic_s()
        return self._last_start_s

    def elapsed_s(self) -> float:
        return max(0.0, monotonic_s() - self._last_start_s)

    def remaining_s(self) -> float:
        return self._period_s - self.elapsed_s()

    def sleep(self) -> float:
        remaining = self.remaining_s()

        if remaining > 0.0:
            time.sleep(remaining)
            self._last_sleep_s = remaining
        else:
            self._last_sleep_s = 0.0
            self._overrun_count += 1

        self._tick_count += 1
        return self._last_sleep_s

    def step(self) -> float:
        slept = self.sleep()
        self.mark_start()
        return slept

    def reset_stats(self) -> None:
        self._last_sleep_s = 0.0
        self._overrun_count = 0
        self._tick_count = 0

    def to_dict(self) -> dict:
        return {
            "rate_hz": self.rate_hz,
            "period_s": self.period_s,
            "last_sleep_s": self.last_sleep_s,
            "overrun_count": self.overrun_count,
            "tick_count": self.tick_count,
        }


__all__ = [
    "RateKeeper",
    "monotonic_s",
]