#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU health checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from statistics import mean, pstdev
from typing import Iterable

from savo_localization.constants import (
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    IMU_HEALTH_GRADE_B,
    IMU_HEALTH_GRADE_C,
    IMU_HEALTH_GRADE_F,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.models.imu_state import ImuRawSample


@dataclass
class ImuStats:
    sample_count: int = 0
    accel_norm_mean_mps2: float = 0.0
    accel_norm_std_mps2: float = 0.0
    gyro_norm_mean_dps: float = 0.0
    gyro_norm_std_dps: float = 0.0
    temperature_mean_c: float | None = None

    def to_dict(self) -> dict[str, float | int]:
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
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "stats": self.stats.to_dict(),
        }


def compute_imu_stats(samples: Iterable[ImuRawSample]) -> ImuStats:
    sample_list = list(samples)

    if not sample_list:
        return ImuStats()

    accel_norms = [float(sample.gravity_norm_mps2) for sample in sample_list]
    gyro_norms = [float(sample.gyro_dps.norm) for sample in sample_list]
    temperatures = [
        float(sample.temp_c)
        for sample in sample_list
        if sample.temp_c is not None
    ]

    return ImuStats(
        sample_count=len(sample_list),
        accel_norm_mean_mps2=mean(accel_norms),
        accel_norm_std_mps2=pstdev(accel_norms) if len(accel_norms) > 1 else 0.0,
        gyro_norm_mean_dps=mean(gyro_norms),
        gyro_norm_std_dps=pstdev(gyro_norms) if len(gyro_norms) > 1 else 0.0,
        temperature_mean_c=mean(temperatures) if temperatures else 0.0,
    )


def grade_from_reasons(reasons: Iterable[str]) -> str:
    reason_list = list(reasons)

    if not reason_list:
        return IMU_HEALTH_GRADE_A

    reason_text = " ".join(reason.lower() for reason in reason_list)

    if (
        len(reason_list) >= 4
        or "system error" in reason_text
        or "chip check failed" in reason_text
        or "no imu samples" in reason_text
        or "no samples" in reason_text
    ):
        return IMU_HEALTH_GRADE_F

    if len(reason_list) >= 2:
        return IMU_HEALTH_GRADE_C

    return IMU_HEALTH_GRADE_B


def status_from_grade(grade: str) -> str:
    if grade == IMU_HEALTH_GRADE_A:
        return STATUS_OK

    if grade in (IMU_HEALTH_GRADE_B, IMU_HEALTH_GRADE_C):
        return STATUS_WARN

    return STATUS_ERROR


def check_imu_sample(sample: ImuRawSample) -> ImuHealthCheckResult:
    return check_imu_samples([sample])


def check_imu_samples(samples: Iterable[ImuRawSample]) -> ImuHealthCheckResult:
    sample_list = list(samples)
    stats = compute_imu_stats(sample_list)

    if not sample_list:
        return ImuHealthCheckResult(
            status=STATUS_ERROR,
            grade=IMU_HEALTH_GRADE_F,
            ok=False,
            message="IMU health check failed",
            reasons=["no IMU samples available"],
            stats=stats,
        )

    reasons: list[str] = []

    for sample in sample_list:
        if int(sample.sys_error) != 0:
            reasons.append(f"system error: sys_error={sample.sys_error}")

        if not sample.calibration.motion_ready:
            reasons.append("calibration not motion ready")

        if sample.temp_c is not None and float(sample.temp_c) >= 85.0:
            reasons.append(f"temperature high: {float(sample.temp_c):.1f}C")

        gravity_error = abs(float(sample.gravity_norm_mps2) - GRAVITY_MPS2)
        if gravity_error > 3.0:
            reasons.append(
                f"gravity norm outside expected range: {sample.gravity_norm_mps2:.2f}"
            )

    if stats.accel_norm_std_mps2 > 1.0:
        reasons.append(
            f"acceleration noise high: std={stats.accel_norm_std_mps2:.3f}"
        )

    if stats.gyro_norm_std_dps > 5.0:
        reasons.append(
            f"gyro noise high: std={stats.gyro_norm_std_dps:.3f}"
        )

    grade = grade_from_reasons(reasons)
    status = status_from_grade(grade)
    ok = status != STATUS_ERROR

    if status == STATUS_OK:
        message = "IMU healthy"
    elif status == STATUS_WARN:
        message = "IMU usable with health notes"
    else:
        message = "IMU health check failed"

    return ImuHealthCheckResult(
        status=status,
        grade=grade,
        ok=ok,
        message=message,
        reasons=_unique_preserve_order(reasons),
        stats=stats,
    )


def _unique_preserve_order(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []

    for value in values:
        if value in seen:
            continue

        seen.add(value)
        result.append(value)

    return result


__all__ = [
    "ImuStats",
    "ImuHealthCheckResult",
    "compute_imu_stats",
    "grade_from_reasons",
    "status_from_grade",
    "check_imu_sample",
    "check_imu_samples",
]
