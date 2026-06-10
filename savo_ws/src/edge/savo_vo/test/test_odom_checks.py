"""Tests for visual odometry sample validation."""

from math import inf, nan

from savo_vo.core.odom_checks import (
    is_finite_number,
    is_sample_numeric,
    is_sample_publishable,
    is_valid_planar_pose,
    is_valid_planar_velocity,
    reject_reason,
)
from savo_vo.models.odometry_quality import OdometryQuality
from savo_vo.models.tracking_report import TrackingReport
from savo_vo.models.vo_sample import VOSample
from savo_vo.models.vo_status import VOStatus


def make_valid_sample() -> VOSample:
    tracking = TrackingReport(
        feature_count=300,
        matched_count=240,
        inlier_count=200,
        tracking_quality=0.85,
    )

    return VOSample(
        timestamp_s=10.0,
        x_m=1.0,
        y_m=2.0,
        z_m=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.2,
        vx_mps=0.1,
        vy_mps=0.0,
        vz_mps=0.0,
        yaw_rate_radps=0.05,
        tracking=tracking,
        quality=OdometryQuality.good(tracking_quality=0.85),
        status=VOStatus.ok(tracking_quality=0.85, feature_count=300),
    )


def test_is_finite_number_rejects_nan_and_inf() -> None:
    assert is_finite_number(1.0)
    assert not is_finite_number(nan)
    assert not is_finite_number(inf)


def test_valid_planar_pose_accepts_finite_values() -> None:
    assert is_valid_planar_pose(x_m=1.0, y_m=2.0, yaw_rad=0.5)


def test_valid_planar_pose_rejects_non_finite_values() -> None:
    assert not is_valid_planar_pose(x_m=nan, y_m=2.0, yaw_rad=0.5)
    assert not is_valid_planar_pose(x_m=1.0, y_m=inf, yaw_rad=0.5)
    assert not is_valid_planar_pose(x_m=1.0, y_m=2.0, yaw_rad=nan)


def test_valid_planar_velocity_accepts_finite_values() -> None:
    assert is_valid_planar_velocity(
        vx_mps=0.1,
        vy_mps=0.0,
        yaw_rate_radps=0.05,
    )


def test_valid_planar_velocity_rejects_non_finite_values() -> None:
    assert not is_valid_planar_velocity(
        vx_mps=nan,
        vy_mps=0.0,
        yaw_rate_radps=0.05,
    )
    assert not is_valid_planar_velocity(
        vx_mps=0.1,
        vy_mps=inf,
        yaw_rate_radps=0.05,
    )
    assert not is_valid_planar_velocity(
        vx_mps=0.1,
        vy_mps=0.0,
        yaw_rate_radps=nan,
    )


def test_valid_sample_is_numeric_and_publishable() -> None:
    sample = make_valid_sample()

    assert is_sample_numeric(sample)
    assert is_sample_publishable(sample)
    assert reject_reason(sample) == ""


def test_sample_with_nan_is_not_numeric_or_publishable() -> None:
    sample = make_valid_sample()
    bad_sample = VOSample(
        **{
            **sample.__dict__,
            "x_m": nan,
        }
    )

    assert not is_sample_numeric(bad_sample)
    assert not is_sample_publishable(bad_sample)
    assert reject_reason(bad_sample) == "visual odometry sample contains non-finite values"


def test_sample_with_lost_status_is_not_publishable() -> None:
    sample = make_valid_sample()
    bad_sample = VOSample(
        **{
            **sample.__dict__,
            "status": VOStatus.lost(),
        }
    )

    assert not is_sample_publishable(bad_sample)
    assert reject_reason(bad_sample) == "visual odometry status is not usable: lost"


def test_sample_with_invalid_quality_is_not_publishable() -> None:
    sample = make_valid_sample()
    bad_sample = VOSample(
        **{
            **sample.__dict__,
            "quality": OdometryQuality.invalid(),
        }
    )

    assert not is_sample_publishable(bad_sample)
    assert reject_reason(bad_sample) == "visual odometry quality is not valid: invalid"


def test_sample_with_unusable_tracking_is_not_publishable() -> None:
    sample = make_valid_sample()
    bad_sample = VOSample(
        **{
            **sample.__dict__,
            "tracking": TrackingReport.empty(),
        }
    )

    assert not is_sample_publishable(bad_sample)
    assert reject_reason(bad_sample) == "visual odometry tracking is not usable"