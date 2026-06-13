#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Covariance helpers for IMU, wheel odometry, and EKF diagnostics."""

from __future__ import annotations

import math
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

    def to_list(self) -> list[float]:
        return covariance3_from_diagonal(self.x, self.y, self.z)


@dataclass(frozen=True)
class DiagonalCovariance6:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def to_list(self) -> list[float]:
        return covariance6_from_diagonal(
            x=self.x,
            y=self.y,
            z=self.z,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
        )


def covariance_from_diagonal(diagonal: Iterable[float]) -> list[float]:
    values = [float(value) for value in diagonal]

    if not values:
        raise ValueError("diagonal cannot be empty")

    size = len(values)
    covariance = [0.0] * (size * size)

    for index, value in enumerate(values):
        covariance[index * size + index] = value

    return covariance


def covariance3_from_diagonal(x: float, y: float, z: float) -> list[float]:
    return covariance_from_diagonal([x, y, z])


def covariance6_from_diagonal(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[float]:
    return covariance_from_diagonal([x, y, z, roll, pitch, yaw])


def covariance_3x3(x: float, y: float, z: float) -> list[float]:
    return covariance3_from_diagonal(x, y, z)


def covariance_6x6(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[float]:
    return covariance6_from_diagonal(
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def diagonal_from_covariance(
    covariance: Iterable[float],
    *,
    size: int,
) -> list[float]:
    if size <= 0:
        raise ValueError("size must be > 0")

    values = [float(value) for value in covariance]
    expected_len = size * size

    if len(values) != expected_len:
        raise ValueError(
            f"covariance length must be {expected_len}, got {len(values)}"
        )

    return [values[index * size + index] for index in range(size)]


def diagonal_from_covariance_3x3(covariance: Iterable[float]) -> list[float]:
    return diagonal_from_covariance(covariance, size=3)


def diagonal_from_covariance_6x6(covariance: Iterable[float]) -> list[float]:
    return diagonal_from_covariance(covariance, size=6)


def validate_covariance_size(
    covariance: Iterable[float],
    *,
    expected_size: int,
) -> list[float]:
    values = [float(value) for value in covariance]

    if len(values) != expected_size:
        raise ValueError(
            f"covariance length must be {expected_size}, got {len(values)}"
        )

    return values


def validate_covariance_3x3(covariance: Iterable[float]) -> list[float]:
    return validate_covariance_size(
        covariance,
        expected_size=COVARIANCE_3X3_SIZE,
    )


def validate_covariance_6x6(covariance: Iterable[float]) -> list[float]:
    return validate_covariance_size(
        covariance,
        expected_size=COVARIANCE_6X6_SIZE,
    )


def covariance_is_finite(covariance: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in covariance)


def covariance_is_zero(
    covariance: Iterable[float],
    *,
    tolerance: float = 1e-12,
) -> bool:
    if tolerance < 0.0:
        raise ValueError("tolerance must be >= 0.0")

    return all(abs(float(value)) <= tolerance for value in covariance)


def covariance_has_unknown_marker(covariance: Iterable[float]) -> bool:
    values = list(covariance)
    return bool(values) and float(values[0]) == UNKNOWN_COVARIANCE_SENTINEL


def replace_unknown_with_zero(covariance: Iterable[float]) -> list[float]:
    values = [float(value) for value in covariance]

    if values and values[0] == UNKNOWN_COVARIANCE_SENTINEL:
        values[0] = 0.0

    return values


def scale_covariance(
    covariance: Iterable[float],
    scale: float,
) -> list[float]:
    if scale < 0.0:
        raise ValueError("scale must be >= 0.0")

    return [float(value) * float(scale) for value in covariance]


def zero_covariance_3x3() -> list[float]:
    return [0.0] * COVARIANCE_3X3_SIZE


def zero_covariance_6x6() -> list[float]:
    return [0.0] * COVARIANCE_6X6_SIZE


def unknown_covariance_3x3() -> list[float]:
    values = zero_covariance_3x3()
    values[0] = UNKNOWN_COVARIANCE_SENTINEL
    return values


def unknown_covariance_6x6() -> list[float]:
    values = zero_covariance_6x6()
    values[0] = UNKNOWN_COVARIANCE_SENTINEL
    return values


def make_imu_orientation_covariance(
    roll: float = 0.05,
    pitch: float = 0.05,
    yaw: float = 0.10,
) -> list[float]:
    return covariance3_from_diagonal(roll, pitch, yaw)


def make_imu_angular_velocity_covariance(
    x: float = 0.02,
    y: float = 0.02,
    z: float = 0.02,
) -> list[float]:
    return covariance3_from_diagonal(x, y, z)


def make_imu_linear_acceleration_covariance(
    x: float = 0.10,
    y: float = 0.10,
    z: float = 0.10,
) -> list[float]:
    return covariance3_from_diagonal(x, y, z)


def make_wheel_odom_pose_covariance(
    x: float = 0.05,
    y: float = 0.10,
    z: float = 999.0,
    roll: float = 999.0,
    pitch: float = 999.0,
    yaw: float = 0.10,
) -> list[float]:
    return covariance6_from_diagonal(
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def make_wheel_odom_twist_covariance(
    vx: float = 0.05,
    vy: float = 0.10,
    vz: float = 999.0,
    wx: float = 999.0,
    wy: float = 999.0,
    wz: float = 0.10,
) -> list[float]:
    return covariance6_from_diagonal(
        x=vx,
        y=vy,
        z=vz,
        roll=wx,
        pitch=wy,
        yaw=wz,
    )


__all__ = [
    "COVARIANCE_3X3_SIZE",
    "COVARIANCE_6X6_SIZE",
    "UNKNOWN_COVARIANCE_SENTINEL",
    "DiagonalCovariance3",
    "DiagonalCovariance6",
    "covariance_from_diagonal",
    "covariance3_from_diagonal",
    "covariance6_from_diagonal",
    "covariance_3x3",
    "covariance_6x6",
    "diagonal_from_covariance",
    "diagonal_from_covariance_3x3",
    "diagonal_from_covariance_6x6",
    "validate_covariance_size",
    "validate_covariance_3x3",
    "validate_covariance_6x6",
    "covariance_is_finite",
    "covariance_is_zero",
    "covariance_has_unknown_marker",
    "replace_unknown_with_zero",
    "scale_covariance",
    "zero_covariance_3x3",
    "zero_covariance_6x6",
    "unknown_covariance_3x3",
    "unknown_covariance_6x6",
    "make_imu_orientation_covariance",
    "make_imu_angular_velocity_covariance",
    "make_imu_linear_acceleration_covariance",
    "make_wheel_odom_pose_covariance",
    "make_wheel_odom_twist_covariance",
]
