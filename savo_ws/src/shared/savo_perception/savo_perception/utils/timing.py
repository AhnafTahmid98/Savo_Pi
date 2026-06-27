#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Timing helpers for perception fallback nodes and diagnostics."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

from savo_perception.utils.clamp import safe_rate_hz


def now_mono_s() -> float:
    return time.monotonic()


def now_wall_s() -> float:
    return time.time()


def elapsed_s(start_mono_s: float, now_s: Optional[float] = None) -> float:
    t = now_mono_s() if now_s is None else float(now_s)
    return max(0.0, t - float(start_mono_s))


def age_s(stamp_mono_s: float, now_s: Optional[float] = None) -> float:
    return elapsed_s(stamp_mono_s, now_s)


def is_stale(
    stamp_mono_s: Optional[float],
    *,
    timeout_s: float,
    now_s: Optional[float] = None,
) -> bool:
    if stamp_mono_s is None:
        return True

    return age_s(float(stamp_mono_s), now_s) > float(timeout_s)


def rate_to_period_s(rate_hz: float, *, min_hz: float = 0.1) -> float:
    hz = safe_rate_hz(rate_hz, default=min_hz, min_hz=min_hz)
    return 1.0 / hz


def sleep_remaining(loop_start_mono_s: float, period_s: float) -> float:
    remaining = float(period_s) - elapsed_s(loop_start_mono_s)

    if remaining > 0.0:
        time.sleep(remaining)
        return remaining

    return 0.0


@dataclass
class RateKeeper:
    rate_hz: float
    min_hz: float = 0.1
    last_start_mono_s: float = field(default_factory=now_mono_s)

    @property
    def period_s(self) -> float:
        return rate_to_period_s(self.rate_hz, min_hz=self.min_hz)

    def mark_start(self) -> float:
        self.last_start_mono_s = now_mono_s()
        return self.last_start_mono_s

    def sleep(self) -> float:
        return sleep_remaining(self.last_start_mono_s, self.period_s)


@dataclass
class LoopStats:
    name: str
    count: int = 0
    start_mono_s: float = field(default_factory=now_mono_s)
    last_mono_s: float = field(default_factory=now_mono_s)

    def tick(self, now_s: Optional[float] = None) -> None:
        t = now_mono_s() if now_s is None else float(now_s)
        self.count += 1
        self.last_mono_s = t

    @property
    def elapsed_s(self) -> float:
        return elapsed_s(self.start_mono_s)

    @property
    def hz(self) -> float:
        duration = self.elapsed_s
        if duration <= 0.0:
            return 0.0
        return float(self.count) / duration

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "count": self.count,
            "elapsed_s": self.elapsed_s,
            "hz": self.hz,
            "last_age_s": age_s(self.last_mono_s),
        }


@dataclass
class FreshnessTracker:
    name: str
    stale_timeout_s: float
    last_update_mono_s: Optional[float] = None
    update_count: int = 0

    def update(self, stamp_mono_s: Optional[float] = None) -> None:
        self.last_update_mono_s = now_mono_s() if stamp_mono_s is None else float(stamp_mono_s)
        self.update_count += 1

    @property
    def age_s(self) -> Optional[float]:
        if self.last_update_mono_s is None:
            return None
        return age_s(self.last_update_mono_s)

    @property
    def stale(self) -> bool:
        return is_stale(
            self.last_update_mono_s,
            timeout_s=self.stale_timeout_s,
        )

    @property
    def fresh(self) -> bool:
        return not self.stale

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "fresh": self.fresh,
            "stale": self.stale,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "update_count": self.update_count,
        }


__all__ = [
    "now_mono_s",
    "now_wall_s",
    "elapsed_s",
    "age_s",
    "is_stale",
    "rate_to_period_s",
    "sleep_remaining",
    "RateKeeper",
    "LoopStats",
    "FreshnessTracker",
]