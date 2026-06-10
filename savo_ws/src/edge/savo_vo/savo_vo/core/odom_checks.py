"""Validation helpers for visual odometry samples."""

from math import isfinite

from savo_vo.models.vo_sample import VOSample


def is_finite_number(value: float) -> bool:
    return isfinite(value)


def is_valid_planar_pose(x_m: float, y_m: float, yaw_rad: float) -> bool:
    return (
        is_finite_number(x_m)
        and is_finite_number(y_m)
        and is_finite_number(yaw_rad)
    )


def is_valid_planar_velocity(
    vx_mps: float,
    vy_mps: float,
    yaw_rate_radps: float,
) -> bool:
    return (
        is_finite_number(vx_mps)
        and is_finite_number(vy_mps)
        and is_finite_number(yaw_rate_radps)
    )


def is_sample_numeric(sample: VOSample) -> bool:
    values = [
        sample.timestamp_s,
        sample.x_m,
        sample.y_m,
        sample.z_m,
        sample.roll_rad,
        sample.pitch_rad,
        sample.yaw_rad,
        sample.vx_mps,
        sample.vy_mps,
        sample.vz_mps,
        sample.yaw_rate_radps,
    ]

    return all(is_finite_number(value) for value in values)


def is_sample_publishable(sample: VOSample) -> bool:
    return (
        sample.is_valid
        and is_sample_numeric(sample)
        and is_valid_planar_pose(sample.x_m, sample.y_m, sample.yaw_rad)
        and is_valid_planar_velocity(
            sample.vx_mps,
            sample.vy_mps,
            sample.yaw_rate_radps,
        )
    )


def reject_reason(sample: VOSample) -> str:
    if not is_sample_numeric(sample):
        return "visual odometry sample contains non-finite values"

    if not sample.status.is_usable:
        return f"visual odometry status is not usable: {sample.status.state.value}"

    if not sample.quality.is_valid:
        return f"visual odometry quality is not valid: {sample.quality.level.value}"

    if not sample.tracking.is_usable:
        return "visual odometry tracking is not usable"

    if not is_valid_planar_pose(sample.x_m, sample.y_m, sample.yaw_rad):
        return "visual odometry planar pose is invalid"

    if not is_valid_planar_velocity(
        sample.vx_mps,
        sample.vy_mps,
        sample.yaw_rate_radps,
    ):
        return "visual odometry planar velocity is invalid"

    return ""