"""Tests for visual feature tracking reports."""

from savo_vo.models.tracking_report import TrackingReport


def test_empty_tracking_report_is_not_usable() -> None:
    report = TrackingReport.empty()

    assert report.feature_count == 0
    assert report.matched_count == 0
    assert report.inlier_count == 0
    assert report.tracking_quality == 0.0
    assert not report.has_features
    assert not report.has_matches
    assert not report.has_inliers
    assert not report.is_usable


def test_tracking_report_ratios_are_computed_correctly() -> None:
    report = TrackingReport(
        feature_count=100,
        matched_count=50,
        inlier_count=25,
        tracking_quality=0.75,
    )

    assert report.has_features
    assert report.has_matches
    assert report.has_inliers
    assert report.match_ratio == 0.5
    assert report.inlier_ratio == 0.5
    assert report.is_usable


def test_tracking_report_handles_zero_matches_safely() -> None:
    report = TrackingReport(
        feature_count=100,
        matched_count=0,
        inlier_count=0,
        tracking_quality=0.0,
    )

    assert report.match_ratio == 0.0
    assert report.inlier_ratio == 0.0
    assert not report.is_usable


def test_tracking_report_handles_zero_features_safely() -> None:
    report = TrackingReport(
        feature_count=0,
        matched_count=0,
        inlier_count=0,
        tracking_quality=0.0,
    )

    assert report.match_ratio == 0.0
    assert report.inlier_ratio == 0.0
    assert not report.is_usable


def test_tracking_report_requires_positive_quality_and_inliers_to_be_usable() -> None:
    no_quality = TrackingReport(
        feature_count=100,
        matched_count=50,
        inlier_count=25,
        tracking_quality=0.0,
    )
    no_inliers = TrackingReport(
        feature_count=100,
        matched_count=50,
        inlier_count=0,
        tracking_quality=0.8,
    )

    assert not no_quality.is_usable
    assert not no_inliers.is_usable