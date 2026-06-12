"""Loop-rate helper for nodes that need steady periodic work."""

from __future__ import annotations

import time
from dataclasses import dataclass

from savo_lidar.utils.timing import monotonic_now_s


@dataclass
class RateKeeper:
    rate_hz: float
    warn_on_overrun: bool = False

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be > 0.0, got {self.rate_hz}")

        self._period_s = 1.0 / float(self.rate_hz)
        self._next_wake_s = monotonic_now_s() + self._period_s
        self._last_overrun_s = 0.0

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def last_overrun_s(self) -> float:
        return self._last_overrun_s

    def reset(self) -> None:
        self._next_wake_s = monotonic_now_s() + self._period_s
        self._last_overrun_s = 0.0

    def sleep(self) -> bool:
        now_s = monotonic_now_s()
        sleep_s = self._next_wake_s - now_s

        if sleep_s > 0.0:
            time.sleep(sleep_s)
            self._last_overrun_s = 0.0
            self._next_wake_s += self._period_s
            return True

        self._last_overrun_s = abs(sleep_s)
        self._next_wake_s = now_s + self._period_s
        return False