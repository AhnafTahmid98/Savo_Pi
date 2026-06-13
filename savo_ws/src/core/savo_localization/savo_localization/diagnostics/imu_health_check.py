#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU health checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Iterable

from savo_localization.constants import (
    DEFAULT_GRAVITY_ERROR_ERROR_MPS2,
    DEFAULT_GRAVITY_ERROR_WARN_MPS2,
    DEFAULT_GYRO_Z_BIAS_ERROR_DPS,
    DEFAULT_GYRO_Z_BIAS_WARN_DPS,
    DEFAULT_IMU_TEMP_MAX_C,
    DEFAULT_IMU_TEMP_MIN_C,
    DEFAULT_IMU_TEMP_WARN_HIGH_C,
    DEFAULT_IMU_TEMP_WARN_LOW_C,
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    IMU_HEALTH_GRADE_B,
    IMU_HEALTH_GRADE_C,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.models.imu_state import (
    ImuCalibration,
    ImuHealthState,
    ImuRawSample,
)


@dataclass(frozen=True)
class ImuStats:
    sample_count: int = 0

    gravity_mean_mps2: float | None = None
    gravity_std_mps2: float | None = None

    gyro_z_mean_dps: float | None = None
    gyro_z_std_dps: float | None = None

    accel_x_mean_mps2: float | None = None
    accel_y_mean_mps2: float | None = None
    accel_z_mean_mps2: float | None = None

    temp_mean_c: float | None = None
    temp_min_c: float | None = None
    temp_max_c: float | None = None

    moving_sample_count: int = 0
    max_vibe_score: float = 0.0

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class ImuHealthCheckResult:
    status: str
    grade: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)
    stats: ImuStats = field(default_factory=ImuStats)

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "grade": self.grade,
            "ok": self.ok,
            "message": self.message,
            "reasons": list(self.reasons),
            "stats": self.stats.to_dict(),
        }

    def to_health_state(self) -> ImuHealthState:
        health = ImuHealthState(
            status=self.status,
            grade=self.grade,
            message=self.message,
            reasons=list(self.reasons),
        )

        health.hardware_ok = self.status in (STATUS_OK, STATUS_WARN)
        health.data_ok = self.status in (STATUS_OK, STATUS_WARN)
        health.temperature_ok = not any("temperature" in reason.lower() for reason in self.reasons)

        health.gravity_norm_mps2 = self.stats.gravity_mean_mps2
        health.gyro_z_bias_dps = self.stats.gyro_z_mean_dps
        health.temp_c = self.stats.temp_mean_c
        health.moving = self.stats.moving_sample_count > 0
        health.vibe_score = self.stats.max_vibe_score

        return health


def check_imu_sample(
    sample: ImuRawSample,
    *,
    require_motion_ready: bool = True,
    require_mag_ready: bool = False,
) -> ImuHealthCheckResult:
    return check_imu_samples(
        [sample],
        calibration=sample.calibration,
        sys_status=sample.sys_status,
        sys_error=sample.sys_error,
        require_motion_ready=require_motion_ready,
        require_mag_ready=require_mag_ready,
    )


def check_imu_samples(
    samples: Iterable[ImuRawSample],
    *,
    calibration: ImuCalibration | None = None,
    sys_status: int | None = None,
    sys_error: int | None = None,
    require_motion_ready: bool = True,
    require_mag_ready: bool = False,
) -> ImuHealthCheckResult:
    sample_list = list(samples)

    if not sample_list:
        return ImuHealthCheckResult(
            status=STATUS_ERROR,
            grade=IMU_HEALTH_GRADE_C,
            ok=False,
            message="no IMU samples available",
            reasons=["sample list is empty"],
        )

    stats = compute_imu_stats(sample_list)

    last_sample = sample_list[-1]
    calibration = calibration or last_sample.calibration
    sys_status = last_sample.sys_status if sys_status is None else int(sys_status)
    sys_error = last_sample.sys_error if sys_error is None else int(sys_error)

    major: list[str] = []
    minor: list[str] = []

    if sys_status not in (0, 5):
        minor.append(f"unexpected SYS_STATUS={sys_status}")

    if sys_error != 0:
        major.append(f"SYS_ERR={sys_error}")

    _check_temperature(stats, major=major, minor=minor)
    _check_gravity(stats, major=major, minor=minor)
    _check_gyro_z_bias(stats, major=major, minor=minor)
    _check_calibration(
        calibration,
        major=major,
        minor=minor,
        require_motion_ready=require_motion_ready,
        require_mag_ready=require_mag_ready,
    )

    if major:
        return ImuHealthCheckResult(
            status=STATUS_ERROR,
            grade=IMU_HEALTH_GRADE_C,
            ok=False,
            message="IMU health check failed",
            reasons=[f"MAJOR: {reason}" for reason in major]
            + [f"MINOR: {reason}" for reason in minor],
            stats=stats,
        )

    if minor:
        return ImuHealthCheckResult(
            status=STATUS_WARN,
            grade=IMU_HEALTH_GRADE_B,
            ok=True,
            message="IMU usable with health notes",
            reasons=[f"MINOR: {reason}" for reason in minor],
            stats=stats,
        )

    return ImuHealthCheckResult(
        status=STATUS_OK,
        grade=IMU_HEALTH_GRADE_A,
        ok=True,
        message="IMU healthy",
        reasons=[],
        stats=stats,
    )


def compute_imu_stats(samples: Iterable[ImuRawSample]) -> ImuStats:
    sample_list = list(samples)

    gravity_values = [sample.gravity_norm_mps2 for sample in sample_list]
    gyro_z_values = [sample.gyro_dps.z for sample in sample_list]
    accel_x_values = [sample.accel_mps2.x for sample in sample_list]
    accel_y_values = [sample.accel_mps2.y for sample in sample_list]
    accel_z_values = [sample.accel_mps2.z for sample in sample_list]
    temp_values = [
        float(sample.temp_c)
        for sample in sample_list
        if sample.temp_c is not None and math.isfinite(float(sample.temp_c))
    ]

    moving_count = sum(1 for sample in sample_list if sample.moving)
    max_vibe = max((float(sample.vibe_score) for sample in sample_list), default=0.0)

    return ImuStats(
        sample_count=len(sample_list),
        gravity_mean_mps2=_mean_or_none(gravity_values),
        gravity_std_mps2=_std_or_none(gravity_values),
        gyro_z_mean_dps=_mean_or_none(gyro_z_values),
        gyro_z_std_dps=_std_or_none(gyro_z_values),
        accel_x_mean_mps2=_mean_or_none(accel_x_values),
        accel_y_mean_mps2=_mean_or_none(accel_y_values),
        accel_z_mean_mps2=_mean_or_none(accel_z_values),
        temp_mean_c=_mean_or_none(temp_values),
        temp_min_c=min(temp_values) if temp_values else None,
        temp_max_c=max(temp_values) if temp_values else None,
        moving_sample_count=moving_count,
        max_vibe_score=max_vibe,
    )


def grade_from_reasons(reasons: Iterable[str]) -> str:
    reason_list = list(reasons)

    if any(reason.startswith("MAJOR:") for reason in reason_list):
        return IMU_HEALTH_GRADE_C

    if reason_list:
        return IMU_HEALTH_GRADE_B

    return IMU_HEALTH_GRADE_A


def status_from_grade(grade: str) -> str:
    if grade == IMU_HEALTH_GRADE_A:
        return STATUS_OK

    if grade == IMU_HEALTH_GRADE_B:
        return STATUS_WARN

    return STATUS_ERROR


def _check_temperature(
    stats: ImuStats,
    *,
    major: list[str],
    minor: list[str],
) -> None:
    if stats.temp_mean_c is None:
        minor.append("temperature not available")
        return

    temp = stats.temp_mean_c

    if temp < DEFAULT_IMU_TEMP_MIN_C or temp > DEFAULT_IMU_TEMP_MAX_C:
        major.append(f"temperature out of range: {temp:.1f}C")
        return

    if temp < DEFAULT_IMU_TEMP_WARN_LOW_C or temp > DEFAULT_IMU_TEMP_WARN_HIGH_C:
        minor.append(f"temperature near limits: {temp:.1f}C")


def _check_gravity(
    stats: ImuStats,
    *,
    major: list[str],
    minor: list[str],
) -> None:
    if stats.gravity_mean_mps2 is None:
        major.append("gravity norm not available")
        return

    gravity_error = abs(float(stats.gravity_mean_mps2) - GRAVITY_MPS2)

    if gravity_error > DEFAULT_GRAVITY_ERROR_ERROR_MPS2:
        major.append(
            f"|g| mean {stats.gravity_mean_mps2:.2f} far from {GRAVITY_MPS2:.2f}"
        )
        return

    if gravity_error > DEFAULT_GRAVITY_ERROR_WARN_MPS2:
        minor.append(
            f"|g| mean {stats.gravity_mean_mps2:.2f} slightly off {GRAVITY_MPS2:.2f}"
        )


def _check_gyro_z_bias(
    stats: ImuStats,
    *,
    major: list[str],
    minor: list[str],
) -> None:
    if stats.gyro_z_mean_dps is None:
        major.append("gyro Z bias not available")
        return

    bias = abs(float(stats.gyro_z_mean_dps))

    if bias > DEFAULT_GYRO_Z_BIAS_ERROR_DPS:
        major.append(f"gyro Z bias {stats.gyro_z_mean_dps:.2f} dps high")
        return

    if bias > DEFAULT_GYRO_Z_BIAS_WARN_DPS:
        minor.append(f"gyro Z bias {stats.gyro_z_mean_dps:.2f} dps elevated")


def _check_calibration(
    calibration: ImuCalibration,
    *,
    major: list[str],
    minor: list[str],
    require_motion_ready: bool,
    require_mag_ready: bool,
) -> None:
    try:
        calibration.validate()
    except ValueError as exc:
        major.append(str(exc))
        return

    if require_motion_ready:
        if calibration.gyro < 2:
            minor.append(f"gyro calibration low ({calibration.gyro}/3)")

        if calibration.accel < 2:
            minor.append(f"accel calibration low ({calibration.accel}/3)")

    if require_mag_ready and calibration.mag < 2:
        minor.append(f"mag calibration low ({calibration.mag}/3)")


def _mean_or_none(values: Iterable[float]) -> float | None:
    cleaned = _finite_values(values)

    if not cleaned:
        return None

    return sum(cleaned) / float(len(cleaned))


def _std_or_none(values: Iterable[float]) -> float | None:
    cleaned = _finite_values(values)

    if len(cleaned) < 2:
        return 0.0 if cleaned else None

    mean = sum(cleaned) / float(len(cleaned))
    variance = sum((value - mean) ** 2 for value in cleaned) / float(len(cleaned) - 1)

    return math.sqrt(variance)


def _finite_values(values: Iterable[float]) -> list[float]:
    cleaned: list[float] = []

    for value in values:
        number = float(value)
        if math.isfinite(number):
            cleaned.append(number)

    return cleaned