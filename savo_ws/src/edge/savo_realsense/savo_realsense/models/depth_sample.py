from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DepthSample:
    distance_m: float
    valid: bool
    source: str
    stamp_s: float | None = None

    @property
    def is_close_range(self) -> bool:
        return self.valid and self.distance_m < 0.60

    @property
    def is_stop_range(self) -> bool:
        return self.valid and self.distance_m < 0.28