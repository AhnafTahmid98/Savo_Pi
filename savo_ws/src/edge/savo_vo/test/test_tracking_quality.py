"""Tests for visual tracking quality scoring."""

import pytest

from savo_vo.core.tracking_quality import (
    build_tracking_report,
    clamp_quality,
    compute_tracking_quality,
    has_enough_features,
    is_tracking_usable,
)


def test_clamp_quality_limits_values_to_probability_range() -> None:
    assert clamp_quality(-0.5) == 0.0
    assert clamp_quality(0.4) == 0.4
    assert clamp_quality(1.5) == 1.0


def test_compute_tracking_quality_returns_zero_for_missing_data() -> None:
    assert compute_tracking_quality(
        feature_count=0,
        matched_count=0,
        inlier_count=0,
        good_features_target=300,
    ) == 0.0

    assert compute_tracking_quality(
        feature_count=100,
        matched_count=0,
        inlier_count=0,
        good_features_target=300,
    ) == 0.0

    assert compute_tracking_quality(
        feature_count=100,
        matched_count=50,
        inlier_count=0,
        good_features_target=300,
    ) == 0.0


def test_compute_tracking_quality_rejects_invalid_feature_target() -> None:
    with pytest.raises(ValueError):
        compute_tracking_quality(
            feature_count=100,
            matched_count=50,
            inlier_count=25,
            good_features_target=0,
        )


def test_compute_tracking_quality_scores_good_tracking() -> None:
    quality = compute_tracking_quality(
        feature_count=300,
        matched_count=240,
        inlier_count=200,
        good_features_target=300,
    )

    assert quality == pytest.approx(0.881666, rel=1e-4)


def test_build_tracking_report_returns_clear_good_message() -> None:
    report = build_tracking_report(
        feature_count=300,
        matched_count=240,
        inlier_count=200,
        good_features_target=300,
    )

    assert report.feature_count == 300
    assert report.matched_count == 240
    assert report.inlier_count == 200
    assert report.tracking_quality > 0.65
    assert report.message == "tracking quality is good"


def test_build_tracking_report_returns_poor_message_for_weak_tracking() -> None:
    report = build_tracking_report(
        feature_count=50,
        matched_count=10,
        inlier_count=2,
        good_features_target=300,
    )

    assert report.tracking_quality < 0.35
    assert report.message == "tracking quality is poor"


def test_has_enough_features_uses_threshold() -> None:
    assert has_enough_features(feature_count=80, min_features=80)
    assert not has_enough_features(feature_count=79, min_features=80)


def test_has_enough_features_rejects_negative_threshold() -> None:
    with pytest.raises(ValueError):
        has_enough_features(feature_count=80, min_features=-1)


def test_is_tracking_usable_requires_quality_and_inliers() -> None:
    report = build_tracking_report(
        feature_count=300,
        matched_count=240,
        inlier_count=200,
        good_features_target=300,
    )

    assert is_tracking_usable(
        report=report,
        min_tracking_quality=0.35,
        min_inliers=8,
    )


def test_is_tracking_usable_rejects_low_quality() -> None:
    report = build_tracking_report(
        feature_count=20,
        matched_count=4,
        inlier_count=2,
        good_features_target=300,
    )

    assert report.tracking_quality < 0.35
    assert not is_tracking_usable(
        report=report,
        min_tracking_quality=0.35,
        min_inliers=2,
    )


def test_is_tracking_usable_rejects_low_inlier_count() -> None:
    report = build_tracking_report(
        feature_count=300,
        matched_count=240,
        inlier_count=4,
        good_features_target=300,
    )

    assert not is_tracking_usable(
        report=report,
        min_tracking_quality=0.35,
        min_inliers=8,
    )


def test_is_tracking_usable_rejects_negative_quality_threshold() -> None:
    report = build_tracking_report(
        feature_count=300,
        matched_count=240,
        inlier_count=200,
        good_features_target=300,
    )

    with pytest.raises(ValueError):
        is_tracking_usable(
            report=report,
            min_tracking_quality=-0.1,
            min_inliers=8,
        )
