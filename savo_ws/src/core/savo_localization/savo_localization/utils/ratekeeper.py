#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Loop-rate helper for localization diagnostics and CLI tools. No ROS imports."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from savo_localization.utils.timing import hz_to_period_s, monotonic_now_s


@dataclass
class RateKeeper:
    rate_hz: float
    warn_if_late: bool = False
    name: str = "ratekeeper"

    _period_s: float = field(init=False)
    _next_tick_s: float = field(init=False)
    _late_count: int = field(default=0, init=False)
    _tick_count: int = field(default=0, init=False)
    _last_late_s: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self._period_s = hz_to_period_s(self.rate_hz)
        self._next_tick_s = monotonic_now_s() + self._period_s

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def tick_count(self) -> int:
        return self._tick_count

    @property
    def late_count(self) -> int:
        return self._late_count

    @property
    def last_late_s(self) -> float:
        return self._last_late_s

    def reset(self) -> None:
        self._next_tick_s = monotonic_now_s() + self._period_s
        self._late_count = 0
        self._tick_count = 0
        self._last_late_s = 0.0

    def sleep(self) -> float:
        now_s = monotonic_now_s()
        remaining_s = self._next_tick_s - now_s

        if remaining_s > 0.0:
            time.sleep(remaining_s)
            self._last_late_s = 0.0
        else:
            self._late_count += 1
            self._last_late_s = abs(remaining_s)

            if self.warn_if_late:
                print(
                    f"[{self.name}] loop late by {self._last_late_s:.4f}s",
                    flush=True,
                )

        self._tick_count += 1
        self._next_tick_s += self._period_s

        now_after_s = monotonic_now_s()
        if self._next_tick_s < now_after_s - self._period_s:
            self._next_tick_s = now_after_s + self._period_s

        return self._last_late_s

    def status_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "rate_hz": float(self.rate_hz),
            "period_s": self._period_s,
            "tick_count": self._tick_count,
            "late_count": self._late_count,
            "last_late_s": self._last_late_s,
        }
