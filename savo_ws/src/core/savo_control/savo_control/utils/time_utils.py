# -*- coding: utf-8 -*-

"""Time helpers for Python fallback nodes, CLI tools, and tests."""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic


@dataclass(frozen=True)
class TimeCheck:
    age_s: float
    timeout_s: float

    @property
    def stale(self) -> bool:
        return self.age_s > self.timeout_s

    @property
    def fresh(self) -> bool:
        return not self.stale

    def to_dict(self) -> dict:
        return {
            "age_s": self.age_s,
            "timeout_s": self.timeout_s,
            "fresh": self.fresh,
            "stale": self.stale,
        }


def now_s() -> float:
    return monotonic()


def age_s(stamp_s: float, *, now: float | None = None) -> float:
    current = now_s() if now is None else now
    return current - stamp_s


def is_stale(stamp_s: float, timeout_s: float, *, now: float | None = None) -> bool:
    return age_s(stamp_s, now=now) > timeout_s


def is_fresh(stamp_s: float, timeout_s: float, *, now: float | None = None) -> bool:
    return not is_stale(stamp_s, timeout_s, now=now)


def check_age(stamp_s: float, timeout_s: float, *, now: float | None = None) -> TimeCheck:
    return TimeCheck(
        age_s=age_s(stamp_s, now=now),
        timeout_s=timeout_s,
    )


def clamp_dt(dt_s: float, *, min_dt_s: float, max_dt_s: float) -> float:
    if dt_s < min_dt_s:
        return min_dt_s
    if dt_s > max_dt_s:
        return max_dt_s
    return dt_s


def safe_rate_period_s(rate_hz: float, *, fallback_hz: float = 10.0) -> float:
    rate = rate_hz if rate_hz > 0.0 else fallback_hz
    if rate <= 0.0:
        rate = 10.0
    return 1.0 / rate


__all__ = [
    "TimeCheck",
    "age_s",
    "check_age",
    "clamp_dt",
    "is_fresh",
    "is_stale",
    "now_s",
    "safe_rate_period_s",
]
