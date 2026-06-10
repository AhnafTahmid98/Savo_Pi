"""Tests for visual odometry covariance helpers."""

from savo_vo.core.covariance_builder import (
    HIGH_VARIANCE,
    PlanarCovariance,
    build_planar_covariance,
    covariance_for_tracking_quality,
)
from savo_vo.models.odometry_quality import OdometryQuality, OdometryQualityLevel


def test_planar_covariance_pose_array_has_expected_diagonal_values() -> None:
    covariance = PlanarCovariance(
        x_variance=0.1,
        y_variance=0.2,
        z_variance=0.3,
        roll_variance=0.4,
        pitch_variance=0.5,
        yaw_variance=0.6,
    )

    values = covariance.as_pose_covariance()

    assert len(values) == 36
    assert values[0] == 0.1
    assert values[7] == 0.2
    assert values[14] == 0.3
    assert values[21] == 0.4
    assert values[28] == 0.5
    assert values[35] == 0.6


def test_planar_covariance_twist_array_has_expected_diagonal_values() -> None:
    covariance = PlanarCovariance(
        x_variance=0.1,
        y_variance=0.2,
        z_variance=0.3,
        roll_variance=0.4,
        pitch_variance=0.5,
        yaw_variance=0.6,
    )

    values = covariance.as_twist_covariance()

    assert len(values) == 36
    assert values[0] == 0.1
    assert values[7] == 0.2
    assert values[14] == 0.3
    assert values[21] == 0.4
    assert values[28] == 0.5
    assert values[35] == 0.6


def test_build_planar_covariance_disables_z_roll_and_pitch_by_default() -> None:
    quality = OdometryQuality.good(tracking_quality=0.9)
    covariance = build_planar_covariance(quality)

    assert covariance.x_variance == quality.position_variance
    assert covariance.y_variance == quality.position_variance
    assert covariance.yaw_variance == quality.yaw_variance
    assert covariance.z_variance == HIGH_VARIANCE
    assert covariance.roll_variance == HIGH_VARIANCE
    assert covariance.pitch_variance == HIGH_VARIANCE


def test_build_planar_covariance_can_keep_z_roll_and_pitch_enabled() -> None:
    quality = OdometryQuality.ok(tracking_quality=0.6)
    covariance = build_planar_covariance(
        quality=quality,
        disable_z=False,
        disable_roll_pitch=False,
    )

    assert covariance.z_variance == quality.position_variance
    assert covariance.roll_variance == quality.yaw_variance
    assert covariance.pitch_variance == quality.yaw_variance


def test_build_planar_covariance_returns_high_variance_for_invalid_quality() -> None:
    covariance = build_planar_covariance(OdometryQuality.invalid())

    assert covariance.x_variance == HIGH_VARIANCE
    assert covariance.y_variance == HIGH_VARIANCE
    assert covariance.z_variance == HIGH_VARIANCE
    assert covariance.roll_variance == HIGH_VARIANCE
    assert covariance.pitch_variance == HIGH_VARIANCE
    assert covariance.yaw_variance == HIGH_VARIANCE


def test_covariance_for_tracking_quality_good() -> None:
    quality = covariance_for_tracking_quality(0.90)

    assert quality.level == OdometryQualityLevel.GOOD
    assert quality.tracking_quality == 0.90


def test_covariance_for_tracking_quality_ok() -> None:
    quality = covariance_for_tracking_quality(0.60)

    assert quality.level == OdometryQualityLevel.OK
    assert quality.tracking_quality == 0.60


def test_covariance_for_tracking_quality_poor() -> None:
    quality = covariance_for_tracking_quality(0.20)

    assert quality.level == OdometryQualityLevel.POOR
    assert quality.tracking_quality == 0.20


def test_covariance_for_tracking_quality_invalid() -> None:
    quality = covariance_for_tracking_quality(0.0)

    assert quality.level == OdometryQualityLevel.INVALID
    assert quality.tracking_quality == 0.0