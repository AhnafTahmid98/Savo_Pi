#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Covariance helpers for IMU, odometry, and EKF diagnostics. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


COVARIANCE_3X3_SIZE = 9
COVARIANCE_6X6_SIZE = 36
UNKNOWN_COVARIANCE_SENTINEL = -1.0


@dataclass(frozen=True)
class DiagonalCovariance3:
    x: float
    y: float
    z: float

    def validate(self) -> None:
        _require_non_negative(self.x, "x")
        _require_non_negative(self.y, "y")
        _require_non_negative(self.z, "z")

    def to_ros_array(self) -> list[float]:
        self.validate()

        return [
            float(self.x), 0.0, 0.0,
            0.0, float(self.y), 0.0,
            0.0, 0.0, float(self.z),
        ]


@dataclass(frozen=True)
class DiagonalCovariance6:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def validate(self) -> None:
        _require_non_negative(self.x, "x")
        _require_non_negative(self.y, "y")
        _require_non_negative(self.z, "z")
        _require_non_negative(self.roll, "roll")
        _require_non_negative(self.pitch, "pitch")
        _require_non_negative(self.yaw, "yaw")

    def to_ros_array(self) -> list[float]:
        self.validate()

        return [
            float(self.x), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, float(self.y), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, float(self.z), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, float(self.roll), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, float(self.pitch), 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(self.yaw),
        ]


def covariance_3x3(
    *,
    x: float,
    y: float,
    z: float,
) -> list[float]:
    return DiagonalCovariance3(x=x, y=y, z=z).to_ros_array()


def covariance_6x6(
    *,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[float]:
    return DiagonalCovariance6(
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    ).to_ros_array()


def unknown_covariance_3x3() -> list[float]:
    values = [0.0] * COVARIANCE_3X3_SIZE
    values[0] = UNKNOWN_COVARIANCE_SENTINEL
    return values


def unknown_covariance_6x6() -> list[float]:
    values = [0.0] * COVARIANCE_6X6_SIZE
    values[0] = UNKNOWN_COVARIANCE_SENTINEL
    return values


def zero_covariance_3x3() -> list[float]:
    return [0.0] * COVARIANCE_3X3_SIZE


def zero_covariance_6x6() -> list[float]:
    return [0.0] * COVARIANCE_6X6_SIZE


def scale_covariance(values: Iterable[float], scale: float) -> list[float]:
    scale = float(scale)

    if scale < 0.0:
        raise ValueError(f"scale must be >= 0.0, got {scale}")

    return [float(value) * scale for value in values]


def validate_covariance_size(values: Iterable[float], expected_size: int) -> None:
    values_list = list(values)

    if len(values_list) != int(expected_size):
        raise ValueError(
            f"covariance must contain {expected_size} values, "
            f"got {len(values_list)}"
        )


def validate_covariance_3x3(values: Iterable[float]) -> None:
    validate_covariance_size(values, COVARIANCE_3X3_SIZE)


def validate_covariance_6x6(values: Iterable[float]) -> None:
    validate_covariance_size(values, COVARIANCE_6X6_SIZE)


def diagonal_from_covariance_3x3(values: Iterable[float]) -> tuple[float, float, float]:
    values_list = list(values)
    validate_covariance_3x3(values_list)

    return (
        float(values_list[0]),
        float(values_list[4]),
        float(values_list[8]),
    )


def diagonal_from_covariance_6x6(
    values: Iterable[float],
) -> tuple[float, float, float, float, float, float]:
    values_list = list(values)
    validate_covariance_6x6(values_list)

    return (
        float(values_list[0]),
        float(values_list[7]),
        float(values_list[14]),
        float(values_list[21]),
        float(values_list[28]),
        float(values_list[35]),
    )


def covariance_has_unknown_marker(values: Iterable[float]) -> bool:
    values_list = list(values)

    if not values_list:
        return False

    return float(values_list[0]) == UNKNOWN_COVARIANCE_SENTINEL


def covariance_is_zero(values: Iterable[float]) -> bool:
    return all(float(value) == 0.0 for value in values)


def replace_unknown_with_zero(values: Iterable[float]) -> list[float]:
    values_list = [float(value) for value in values]

    if covariance_has_unknown_marker(values_list):
        values_list[0] = 0.0

    return values_list


def make_imu_orientation_covariance(
    *,
    roll_variance: float = 0.05,
    pitch_variance: float = 0.05,
    yaw_variance: float = 0.10,
    orientation_available: bool = True,
) -> list[float]:
    if not orientation_available:
        return unknown_covariance_3x3()

    return covariance_3x3(
        x=roll_variance,
        y=pitch_variance,
        z=yaw_variance,
    )


def make_imu_angular_velocity_covariance(
    *,
    x_variance: float = 0.02,
    y_variance: float = 0.02,
    z_variance: float = 0.02,
) -> list[float]:
    return covariance_3x3(
        x=x_variance,
        y=y_variance,
        z=z_variance,
    )


def make_imu_linear_acceleration_covariance(
    *,
    x_variance: float = 0.10,
    y_variance: float = 0.10,
    z_variance: float = 0.10,
) -> list[float]:
    return covariance_3x3(
        x=x_variance,
        y=y_variance,
        z=z_variance,
    )


def make_wheel_odom_pose_covariance(
    *,
    x_variance: float = 0.05,
    y_variance: float = 0.10,
    z_variance: float = 999.0,
    roll_variance: float = 999.0,
    pitch_variance: float = 999.0,
    yaw_variance: float = 0.10,
) -> list[float]:
    return covariance_6x6(
        x=x_variance,
        y=y_variance,
        z=z_variance,
        roll=roll_variance,
        pitch=pitch_variance,
        yaw=yaw_variance,
    )


def make_wheel_odom_twist_covariance(
    *,
    vx_variance: float = 0.05,
    vy_variance: float = 0.10,
    vz_variance: float = 999.0,
    wx_variance: float = 999.0,
    wy_variance: float = 999.0,
    wz_variance: float = 0.10,
) -> list[float]:
    return covariance_6x6(
        x=vx_variance,
        y=vy_variance,
        z=vz_variance,
        roll=wx_variance,
        pitch=wy_variance,
        yaw=wz_variance,
    )


def _require_non_negative(value: float, name: str) -> None:
    if float(value) < 0.0:
        raise ValueError(f"{name} covariance must be >= 0.0, got {value}")