"""VO runtime status model."""

from dataclasses import dataclass
from enum import Enum


class VOState(str, Enum):
    """High-level visual odometry state."""

    OK = "ok"
    DEGRADED = "degraded"
    LOST = "lost"
    STALE = "stale"
    ERROR = "error"


@dataclass(frozen=True)
class VOStatus:
    state: VOState
    message: str = ""
    tracking_quality: float = 0.0
    feature_count: int = 0
    age_s: float = 0.0

    @property
    def is_usable(self) -> bool:
        return self.state in {VOState.OK, VOState.DEGRADED}

    @property
    def is_healthy(self) -> bool:
        return self.state == VOState.OK

    @classmethod
    def ok(
        cls,
        message: str = "visual odometry is healthy",
        tracking_quality: float = 1.0,
        feature_count: int = 0,
        age_s: float = 0.0,
    ) -> "VOStatus":
        return cls(
            state=VOState.OK,
            message=message,
            tracking_quality=tracking_quality,
            feature_count=feature_count,
            age_s=age_s,
        )

    @classmethod
    def degraded(
        cls,
        message: str,
        tracking_quality: float = 0.0,
        feature_count: int = 0,
        age_s: float = 0.0,
    ) -> "VOStatus":
        return cls(
            state=VOState.DEGRADED,
            message=message,
            tracking_quality=tracking_quality,
            feature_count=feature_count,
            age_s=age_s,
        )

    @classmethod
    def lost(cls, message: str = "visual odometry tracking lost") -> "VOStatus":
        return cls(state=VOState.LOST, message=message)

    @classmethod
    def stale(cls, age_s: float, message: str = "visual odometry data is stale") -> "VOStatus":
        return cls(state=VOState.STALE, message=message, age_s=age_s)

    @classmethod
    def error(cls, message: str) -> "VOStatus":
        return cls(state=VOState.ERROR, message=message)