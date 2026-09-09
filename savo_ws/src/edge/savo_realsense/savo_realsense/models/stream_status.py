# Copyright 2026 Ahnaf Tahmid
from dataclasses import dataclass


@dataclass(frozen=True)
class StreamStatus:
    topic: str
    seen: bool
    stale: bool
    rate_hz: float
    expected_hz: float
    last_age_s: float

    @property
    def ok(self) -> bool:
        return (
            self.seen
            and not self.stale
            and self.rate_quality != "BELOW_MINIMUM"
        )

    @property
    def rate_thresholds_hz(self) -> tuple[float, float, float]:
        """Return production thresholds for this configured stream."""
        if "point" in self.topic.lower():
            return 3.0, 5.0, 7.0
        return 8.0, 12.0, 14.0

    @property
    def rate_quality(self) -> str:
        """Classify measured producer rate independently from freshness."""
        minimum, good, excellent = self.rate_thresholds_hz
        if self.rate_hz < minimum:
            return "BELOW_MINIMUM"
        if self.rate_hz < good:
            return "MINIMUM"
        if self.rate_hz < excellent:
            return "GOOD"
        return "EXCELLENT"

    @property
    def below_expected_rate(self) -> bool:
        return self.rate_quality == "BELOW_MINIMUM"
