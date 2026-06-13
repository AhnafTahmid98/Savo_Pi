# -*- coding: utf-8 -*-
"""Loop-rate helper for ROS nodes and CLI tools."""

from __future__ import annotations

import time

from savo_lidar.utils.clamp import positive_or_default


class RateKeeper:
    def __init__(self, rate_hz: float) -> None:
        self.rate_hz = positive_or_default(rate_hz, 1.0)
        self.period_s = 1.0 / self.rate_hz
        self._next_s = time.monotonic() + self.period_s

    def sleep(self) -> None:
        now_s = time.monotonic()
        remaining_s = self._next_s - now_s

        if remaining_s > 0.0:
            time.sleep(remaining_s)

        self._next_s += self.period_s

        if self._next_s < time.monotonic():
            self._next_s = time.monotonic() + self.period_s

    def reset(self) -> None:
        self._next_s = time.monotonic() + self.period_s

    def set_rate(self, rate_hz: float) -> None:
        self.rate_hz = positive_or_default(rate_hz, self.rate_hz)
        self.period_s = 1.0 / self.rate_hz
        self.reset()


__all__ = [
    "RateKeeper",
]
