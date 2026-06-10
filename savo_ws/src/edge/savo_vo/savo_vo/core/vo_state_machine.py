"""State selection logic for visual odometry health."""

from dataclasses import dataclass

from savo_vo.models.tracking_report import TrackingReport
from savo_vo.models.vo_status import VOState, VOStatus


@dataclass(frozen=True)
class VOStateThresholds:
    min_tracking_quality: float = 0.35
    min_inliers: int = 8
    stale_timeout_s: float = 0.30


def evaluate_vo_state(
    tracking: TrackingReport,
    age_s: float,
    thresholds: VOStateThresholds,
) -> VOStatus:
    if age_s > thresholds.stale_timeout_s:
        return VOStatus.stale(
            age_s=age_s,
            message=f"visual odometry is stale: age={age_s:.3f}s",
        )

    if not tracking.has_features:
        return VOStatus.lost("visual odometry has no tracked features")

    if tracking.inlier_count < thresholds.min_inliers:
        return VOStatus.lost(
            f"visual odometry has too few inliers: {tracking.inlier_count}"
        )

    if tracking.tracking_quality < thresholds.min_tracking_quality:
        return VOStatus.degraded(
            message=(
                "visual odometry tracking quality is low: "
                f"{tracking.tracking_quality:.2f}"
            ),
            tracking_quality=tracking.tracking_quality,
            feature_count=tracking.feature_count,
            age_s=age_s,
        )

    return VOStatus.ok(
        tracking_quality=tracking.tracking_quality,
        feature_count=tracking.feature_count,
        age_s=age_s,
    )


def should_publish_odom(status: VOStatus) -> bool:
    return status.state in {VOState.OK, VOState.DEGRADED}