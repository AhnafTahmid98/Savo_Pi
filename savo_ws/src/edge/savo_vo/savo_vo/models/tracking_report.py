"""Feature tracking summary for visual odometry."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TrackingReport:
    feature_count: int
    matched_count: int
    inlier_count: int
    tracking_quality: float
    message: str = ""

    @property
    def has_features(self) -> bool:
        return self.feature_count > 0

    @property
    def has_matches(self) -> bool:
        return self.matched_count > 0

    @property
    def has_inliers(self) -> bool:
        return self.inlier_count > 0

    @property
    def inlier_ratio(self) -> float:
        if self.matched_count <= 0:
            return 0.0
        return self.inlier_count / self.matched_count

    @property
    def match_ratio(self) -> float:
        if self.feature_count <= 0:
            return 0.0
        return self.matched_count / self.feature_count

    @property
    def is_usable(self) -> bool:
        return self.tracking_quality > 0.0 and self.inlier_count > 0

    @classmethod
    def empty(cls, message: str = "no tracking data") -> "TrackingReport":
        return cls(
            feature_count=0,
            matched_count=0,
            inlier_count=0,
            tracking_quality=0.0,
            message=message,
        )