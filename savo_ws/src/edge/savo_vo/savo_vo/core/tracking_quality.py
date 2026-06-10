"""Tracking quality scoring for visual odometry."""

from savo_vo.models.tracking_report import TrackingReport


def clamp_quality(value: float) -> float:
    return max(0.0, min(1.0, value))


def compute_tracking_quality(
    feature_count: int,
    matched_count: int,
    inlier_count: int,
    good_features_target: int,
) -> float:
    if good_features_target <= 0:
        raise ValueError("good_features_target must be positive")

    if feature_count <= 0 or matched_count <= 0 or inlier_count <= 0:
        return 0.0

    feature_score = min(1.0, feature_count / good_features_target)
    match_score = matched_count / feature_count
    inlier_score = inlier_count / matched_count

    return clamp_quality(
        0.35 * feature_score
        + 0.30 * match_score
        + 0.35 * inlier_score
    )


def build_tracking_report(
    feature_count: int,
    matched_count: int,
    inlier_count: int,
    good_features_target: int,
) -> TrackingReport:
    quality = compute_tracking_quality(
        feature_count=feature_count,
        matched_count=matched_count,
        inlier_count=inlier_count,
        good_features_target=good_features_target,
    )

    if quality <= 0.0:
        message = "tracking has no usable features"
    elif quality < 0.35:
        message = "tracking quality is poor"
    elif quality < 0.65:
        message = "tracking quality is acceptable"
    else:
        message = "tracking quality is good"

    return TrackingReport(
        feature_count=max(0, feature_count),
        matched_count=max(0, matched_count),
        inlier_count=max(0, inlier_count),
        tracking_quality=quality,
        message=message,
    )


def has_enough_features(feature_count: int, min_features: int) -> bool:
    if min_features < 0:
        raise ValueError("min_features must be non-negative")

    return feature_count >= min_features


def is_tracking_usable(
    report: TrackingReport,
    min_tracking_quality: float,
    min_inliers: int = 8,
) -> bool:
    if min_tracking_quality < 0.0:
        raise ValueError("min_tracking_quality must be non-negative")

    return (
        report.tracking_quality >= min_tracking_quality
        and report.inlier_count >= min_inliers
    )