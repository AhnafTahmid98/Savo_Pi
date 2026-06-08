from __future__ import annotations

from collections import deque
from time import monotonic


class RateTracker:
    def __init__(self, window_size: int = 30) -> None:
        self._stamps: deque[float] = deque(maxlen=max(2, window_size))

    def tick(self, stamp_s: float | None = None) -> None:
        self._stamps.append(monotonic() if stamp_s is None else stamp_s)

    @property
    def seen(self) -> bool:
        return bool(self._stamps)

    @property
    def last_age_s(self) -> float:
        if not self._stamps:
            return float("inf")
        return max(0.0, monotonic() - self._stamps[-1])

    @property
    def rate_hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0

        duration_s = self._stamps[-1] - self._stamps[0]
        if duration_s <= 0.0:
            return 0.0

        return (len(self._stamps) - 1) / duration_s


def is_stale(last_age_s: float, stale_timeout_s: float) -> bool:
    return last_age_s > stale_timeout_s