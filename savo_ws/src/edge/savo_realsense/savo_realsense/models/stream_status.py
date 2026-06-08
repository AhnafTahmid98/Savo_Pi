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
        return self.seen and not self.stale and self.rate_hz > 0.0

    @property
    def below_expected_rate(self) -> bool:
        if self.expected_hz <= 0.0:
            return False
        return self.rate_hz < self.expected_hz * 0.5