"""Covariance helpers for visual odometry output."""

from dataclasses import dataclass

from savo_vo.models.odometry_quality import OdometryQuality, OdometryQualityLevel


@dataclass(frozen=True)
class PlanarCovariance:
    x_variance: float
    y_variance: float
    z_variance: float
    roll_variance: float
    pitch_variance: float
    yaw_variance: float

    def as_pose_covariance(self) -> list[float]:
        covariance = [0.0] * 36
        covariance[0] = self.x_variance
        covariance[7] = self.y_variance
        covariance[14] = self.z_variance
        covariance[21] = self.roll_variance
        covariance[28] = self.pitch_variance
        covariance[35] = self.yaw_variance
        return covariance

    def as_twist_covariance(self) -> list[float]:
        covariance = [0.0] * 36
        covariance[0] = self.x_variance
        covariance[7] = self.y_variance
        covariance[14] = self.z_variance
        covariance[21] = self.roll_variance
        covariance[28] = self.pitch_variance
        covariance[35] = self.yaw_variance
        return covariance


HIGH_VARIANCE = 999.0


def build_planar_covariance(
    quality: OdometryQuality,
    disable_z: bool = True,
    disable_roll_pitch: bool = True,
) -> PlanarCovariance:
    if quality.level == OdometryQualityLevel.INVALID:
        return PlanarCovariance(
            x_variance=HIGH_VARIANCE,
            y_variance=HIGH_VARIANCE,
            z_variance=HIGH_VARIANCE,
            roll_variance=HIGH_VARIANCE,
            pitch_variance=HIGH_VARIANCE,
            yaw_variance=HIGH_VARIANCE,
        )

    z_variance = HIGH_VARIANCE if disable_z else quality.position_variance
    roll_pitch_variance = HIGH_VARIANCE if disable_roll_pitch else quality.yaw_variance

    return PlanarCovariance(
        x_variance=quality.position_variance,
        y_variance=quality.position_variance,
        z_variance=z_variance,
        roll_variance=roll_pitch_variance,
        pitch_variance=roll_pitch_variance,
        yaw_variance=quality.yaw_variance,
    )


def covariance_for_tracking_quality(tracking_quality: float) -> OdometryQuality:
    if tracking_quality >= 0.75:
        return OdometryQuality.good(tracking_quality=tracking_quality)

    if tracking_quality >= 0.45:
        return OdometryQuality.ok(tracking_quality=tracking_quality)

    if tracking_quality > 0.0:
        return OdometryQuality.poor(tracking_quality=tracking_quality)

    return OdometryQuality.invalid(message="tracking quality is zero")