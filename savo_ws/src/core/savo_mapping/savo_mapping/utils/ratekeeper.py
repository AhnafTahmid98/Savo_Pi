#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small ratekeeper helper for Python fallback nodes. No ROS imports."""

from __future__ import annotations

import time
from dataclasses import dataclass, field


# =============================================================================
# Ratekeeper
# =============================================================================
@dataclass
class Ratekeeper:
    hz: float
    _period_s: float = field(init=False)
    _next_tick_s: float = field(init=False)

    def __post_init__(self) -> None:
        if self.hz <= 0.0:
            raise ValueError("Ratekeeper frequency must be > 0 Hz.")

        self._period_s = 1.0 / float(self.hz)
        self._next_tick_s = time.monotonic() + self._period_s

    @property
    def period_s(self) -> float:
        return self._period_s

    def reset(self) -> None:
        self._next_tick_s = time.monotonic() + self._period_s

    def sleep(self) -> None:
        now = time.monotonic()
        delay_s = self._next_tick_s - now

        if delay_s > 0.0:
            time.sleep(delay_s)
            self._next_tick_s += self._period_s
            return

        self._next_tick_s = now + self._period_s

    def should_run(self) -> bool:
        return time.monotonic() >= self._next_tick_s

    def mark_run(self) -> None:
        now = time.monotonic()

        if now > self._next_tick_s + self._period_s:
            self._next_tick_s = now + self._period_s
        else:
            self._next_tick_s += self._period_s


# =============================================================================
# Loop budget
# =============================================================================
@dataclass
class LoopBudget:
    target_hz: float
    started_s: float = field(default_factory=time.monotonic)

    @property
    def period_s(self) -> float:
        if self.target_hz <= 0.0:
            raise ValueError("Loop budget frequency must be > 0 Hz.")

        return 1.0 / float(self.target_hz)

    def elapsed_s(self) -> float:
        return max(0.0, time.monotonic() - self.started_s)

    def remaining_s(self) -> float:
        return max(0.0, self.period_s - self.elapsed_s())

    def overrun_s(self) -> float:
        return max(0.0, self.elapsed_s() - self.period_s)

    def reset(self) -> None:
        self.started_s = time.monotonic()


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    rk = Ratekeeper(10.0)
    budget = LoopBudget(10.0)

    print(f"period_s={rk.period_s:.3f}")
    print(f"remaining_s={budget.remaining_s():.3f}")
    print(f"overrun_s={budget.overrun_s():.3f}")


if __name__ == "__main__":
    main()


__all__ = [
    "Ratekeeper",
    "LoopBudget",
    "main",
]