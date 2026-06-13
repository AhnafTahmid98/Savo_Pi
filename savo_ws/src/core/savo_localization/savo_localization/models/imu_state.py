#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU state models for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from savo_localization.constants import (
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    IMU_MODEL_BNO055,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)


@dataclass
class ImuVector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def norm(self) -> float:
        return math.sqrt(
            float(self.x) * float(self.x)
            + float(self.y) * float(self.y)
            + float(self.z) * float(self.z)
        )

    @property
    def magnitude(self) -> float:
        return self.norm

    def to_dict(self) -> dict[str, float]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "norm": self.norm,
        }


@dataclass
class ImuEuler:
    yaw_deg: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    available: bool = False

    def to_dict(self) -> dict[str, object]:
        return {
            "yaw_deg": float(self.yaw_deg),
            "roll_deg": float(self.roll_deg),
            "pitch_deg": float(self.pitch_deg),
            "available": bool(self.available),
        }


@dataclass
class ImuCalibration:
    system: int = 0
    gyro: int = 0
    accel: int = 0
    mag: int = 0


    @property
    def valid(self) -> bool:
        return all(0 <= int(value) <= 3 for value in (self.system, self.gyro, self.accel, self.mag))

    def validate(self) -> None:
        for name, value in self.to_tuple_named().items():
            if int(value) < 0 or int(value) > 3:
                raise ValueError(f"{name} calibration must be between 0 and 3")

    @property
    def motion_ready(self) -> bool:
        return self.valid and self.gyro >= 2 and self.accel >= 2

    @property
    def fully_calibrated(self) -> bool:
        return self.valid and (
            self.system == 3
            and self.gyro == 3
            and self.accel == 3
            and self.mag == 3
        )

    def to_dict(self) -> dict[str, int]:
        return {
            "system": int(self.system),
            "gyro": int(self.gyro),
            "accel": int(self.accel),
            "mag": int(self.mag),
            "motion_ready": self.motion_ready,
            "fully_calibrated": self.fully_calibrated,
        }

    def to_tuple_named(self) -> dict[str, int]:
        return {
            "system": int(self.system),
            "gyro": int(self.gyro),
            "accel": int(self.accel),
            "mag": int(self.mag),
        }


@dataclass
class ImuHealthState:
    status: str = STATUS_UNKNOWN
    grade: str = IMU_HEALTH_GRADE_A
    message: str = "IMU state unknown"
    reasons: list[str] = field(default_factory=list)

    hardware_ok: bool = False
    data_ok: bool = False
    temperature_ok: bool = False
    calibration_ok: bool = False

    @property
    def ready(self) -> bool:
        return (
            self.status == STATUS_OK
            and self.hardware_ok
            and self.data_ok
            and self.temperature_ok
        )

    @property
    def ok(self) -> bool:
        return self.ready

    def mark_ok(self, message: str = "IMU healthy") -> None:
        self.status = STATUS_OK
        self.grade = IMU_HEALTH_GRADE_A
        self.message = message
        self.reasons.clear()
        self.hardware_ok = True
        self.data_ok = True
        self.temperature_ok = True
        self.calibration_ok = True

    def mark_warn(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_WARN
        self.message = message
        self.reasons = list(reasons or [])
        self.hardware_ok = True
        self.data_ok = True

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_ERROR
        self.message = message
        self.reasons = list(reasons or [])
        self.hardware_ok = False
        self.data_ok = False

    def mark_stale(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_STALE
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "grade": self.grade,
            "message": self.message,
            "reasons": list(self.reasons),
            "hardware_ok": self.hardware_ok,
            "data_ok": self.data_ok,
            "temperature_ok": self.temperature_ok,
            "calibration_ok": self.calibration_ok,
            "ready": self.ready,
            "ok": self.ok,
        }


@dataclass
class ImuRawSample:
    stamp_s: float = 0.0
    accel_mps2: ImuVector3 = field(default_factory=ImuVector3)
    gyro_dps: ImuVector3 = field(default_factory=ImuVector3)
    mag_ut: ImuVector3 = field(default_factory=ImuVector3)
    euler_deg: ImuEuler = field(default_factory=ImuEuler)
    calibration: ImuCalibration = field(default_factory=ImuCalibration)

    temp_c: float | None = None
    sys_status: int = 0
    sys_error: int = 0
    moving: bool | None = None

    def __post_init__(self) -> None:
        if self.moving is None:
            self.moving = imu_is_moving(
                gyro_dps=self.gyro_dps,
                accel_mps2=self.accel_mps2,
            )

    @property
    def accel_norm_mps2(self) -> float:
        return self.accel_mps2.norm

    @property
    def gravity_norm_mps2(self) -> float:
        return self.accel_norm_mps2

    @property
    def gyro_norm_dps(self) -> float:
        return self.gyro_dps.norm

    @property
    def gravity_error_mps2(self) -> float:
        return abs(self.accel_norm_mps2 - GRAVITY_MPS2)

    @property
    def healthy_system(self) -> bool:
        return int(self.sys_error) == 0

    @property
    def vibe_score(self) -> float:
        return float(self.gyro_norm_dps) + float(self.gravity_error_mps2)

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "accel_mps2": self.accel_mps2.to_dict(),
            "gyro_dps": self.gyro_dps.to_dict(),
            "mag_ut": self.mag_ut.to_dict(),
            "euler_deg": self.euler_deg.to_dict(),
            "calibration": self.calibration.to_dict(),
            "temp_c": self.temp_c,
            "sys_status": int(self.sys_status),
            "sys_error": int(self.sys_error),
            "moving": bool(self.moving),
            "accel_norm_mps2": self.accel_norm_mps2,
            "gravity_norm_mps2": self.gravity_norm_mps2,
            "gyro_norm_dps": self.gyro_norm_dps,
            "gravity_error_mps2": self.gravity_error_mps2,
            "healthy_system": self.healthy_system,
            "vibe_score": self.vibe_score,
        }


@dataclass
class ImuState:
    model: str = IMU_MODEL_BNO055

    sample_count: int = 0
    last_sample: ImuRawSample | None = None
    last_sample_age_s: float | None = None

    health: ImuHealthState = field(default_factory=ImuHealthState)

    @property
    def chip_ok(self) -> bool:
        return self.last_sample is not None and self.last_sample.healthy_system

    @property
    def sys_status(self) -> int:
        return int(self.last_sample.sys_status) if self.last_sample is not None else 0

    @property
    def sys_error(self) -> int:
        return int(self.last_sample.sys_error) if self.last_sample is not None else 0

    @property
    def temp_c(self) -> float | None:
        return self.last_sample.temp_c if self.last_sample is not None else None

    @property
    def calibration(self) -> ImuCalibration:
        return (
            self.last_sample.calibration
            if self.last_sample is not None
            else ImuCalibration()
        )

    @property
    def ready(self) -> bool:
        return self.health.ready

    def mark_sample(
        self,
        sample: ImuRawSample,
        *,
        now_s: float | None = None,
    ) -> None:
        self.last_sample = sample
        self.sample_count += 1

        if now_s is None:
            self.last_sample_age_s = 0.0
        else:
            self.last_sample_age_s = max(0.0, float(now_s) - float(sample.stamp_s))

        self.health.hardware_ok = sample.healthy_system
        self.health.data_ok = True
        self.health.temperature_ok = (
            sample.temp_c is None
            or (-40.0 <= float(sample.temp_c) <= 85.0)
        )
        self.health.calibration_ok = sample.calibration.motion_ready

        if (
            sample.healthy_system
            and self.health.temperature_ok
            and sample.calibration.motion_ready
        ):
            self.health.mark_ok()
        elif not sample.healthy_system:
            self.health.mark_error(
                "IMU system error",
                [f"sys_error={sample.sys_error}"],
            )
        else:
            reasons: list[str] = []
            if not sample.calibration.motion_ready:
                reasons.append("IMU calibration not motion ready")
            if not self.health.temperature_ok:
                reasons.append("IMU temperature outside expected range")
            self.health.mark_warn("IMU usable with health notes", reasons)

    def mark_stale(self, age_s: float) -> None:
        self.last_sample_age_s = float(age_s)
        self.health.mark_stale(
            "IMU state stale",
            [f"last sample age_s={float(age_s):.3f}"],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "ready": self.ready,
            "chip_ok": self.chip_ok,
            "sys_status": self.sys_status,
            "sys_error": self.sys_error,
            "temp_c": self.temp_c,
            "calibration": self.calibration.to_dict(),
            "sample_count": int(self.sample_count),
            "last_sample_age_s": self.last_sample_age_s,
            "health": self.health.to_dict(),
            "last_sample": (
                self.last_sample.to_dict()
                if self.last_sample is not None
                else None
            ),
        }


def make_imu_vector3(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> ImuVector3:
    return ImuVector3(x=x, y=y, z=z)


def make_imu_calibration(
    *,
    system: int = 0,
    gyro: int = 0,
    accel: int = 0,
    mag: int = 0,
) -> ImuCalibration:
    return ImuCalibration(
        system=system,
        gyro=gyro,
        accel=accel,
        mag=mag,
    )


def make_imu_sample(
    *,
    stamp_s: float = 0.0,
    accel_mps2: ImuVector3 | None = None,
    gyro_dps: ImuVector3 | None = None,
    mag_ut: ImuVector3 | None = None,
    euler_deg: ImuEuler | None = None,
    calibration: ImuCalibration | None = None,
    temp_c: float | None = None,
    sys_status: int = 0,
    sys_error: int = 0,
    moving: bool | None = None,
) -> ImuRawSample:
    return ImuRawSample(
        stamp_s=stamp_s,
        accel_mps2=accel_mps2 or ImuVector3(),
        gyro_dps=gyro_dps or ImuVector3(),
        mag_ut=mag_ut or ImuVector3(),
        euler_deg=euler_deg or ImuEuler(),
        calibration=calibration or ImuCalibration(),
        temp_c=temp_c,
        sys_status=sys_status,
        sys_error=sys_error,
        moving=moving,
    )


def imu_is_moving(
    *,
    gyro_dps: ImuVector3,
    accel_mps2: ImuVector3,
    gyro_threshold_dps: float = 5.0,
    accel_delta_threshold_mps2: float = 1.0,
) -> bool:
    if gyro_dps.norm < gyro_threshold_dps and accel_mps2.norm == 0.0:
        return False

    accel_delta = math.sqrt(
        accel_mps2.x * accel_mps2.x
        + accel_mps2.y * accel_mps2.y
        + (accel_mps2.z - GRAVITY_MPS2) * (accel_mps2.z - GRAVITY_MPS2)
    )

    return (
        gyro_dps.norm >= gyro_threshold_dps
        or accel_delta >= accel_delta_threshold_mps2
    )


__all__ = [
    "ImuVector3",
    "ImuEuler",
    "ImuCalibration",
    "ImuHealthState",
    "ImuRawSample",
    "ImuState",
    "make_imu_vector3",
    "make_imu_calibration",
    "make_imu_sample",
    "imu_is_moving",
]
