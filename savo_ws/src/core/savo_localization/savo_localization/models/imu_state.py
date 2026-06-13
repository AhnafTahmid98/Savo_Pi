#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU state models for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    BNO055_CHIP_ID,
    BNO055_DEFAULT_ADDRESS,
    BNO055_DEFAULT_I2C_BUS,
    BNO055_DEFAULT_MODE,
    DEFAULT_IMU_ACCEL_STD_MOVING_MPS2,
    DEFAULT_IMU_GYRO_RMS_MOVING_DPS,
    FRAME_IMU,
    GRAVITY_MPS2,
    IMU_HEALTH_GRADE_A,
    IMU_HEALTH_GRADE_B,
    IMU_HEALTH_GRADE_C,
    IMU_MODEL_BNO055,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_UNKNOWN,
    STATUS_WARN,
)


@dataclass(frozen=True)
class ImuVector3:
    x: float
    y: float
    z: float

    @property
    def magnitude(self) -> float:
        return math.sqrt(
            float(self.x) * float(self.x)
            + float(self.y) * float(self.y)
            + float(self.z) * float(self.z)
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "magnitude": self.magnitude,
        }


@dataclass(frozen=True)
class ImuEuler:
    yaw_deg: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None

    @property
    def available(self) -> bool:
        return (
            self.yaw_deg is not None
            and self.roll_deg is not None
            and self.pitch_deg is not None
        )

    def to_dict(self) -> dict[str, float | bool | None]:
        return {
            "available": self.available,
            "yaw_deg": self.yaw_deg,
            "roll_deg": self.roll_deg,
            "pitch_deg": self.pitch_deg,
        }


@dataclass(frozen=True)
class ImuCalibration:
    system: int = 0
    gyro: int = 0
    accel: int = 0
    mag: int = 0

    def validate(self) -> None:
        for name, value in self.to_tuple_named().items():
            if not 0 <= int(value) <= 3:
                raise ValueError(f"{name} calibration must be 0..3, got {value}")

    @property
    def fully_calibrated(self) -> bool:
        return all(value >= 3 for value in self.to_tuple())

    @property
    def motion_ready(self) -> bool:
        return self.gyro >= 2 and self.accel >= 2

    def to_tuple(self) -> tuple[int, int, int, int]:
        return (
            int(self.system),
            int(self.gyro),
            int(self.accel),
            int(self.mag),
        )

    def to_tuple_named(self) -> dict[str, int]:
        return {
            "system": int(self.system),
            "gyro": int(self.gyro),
            "accel": int(self.accel),
            "mag": int(self.mag),
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            **self.to_tuple_named(),
            "fully_calibrated": self.fully_calibrated,
            "motion_ready": self.motion_ready,
        }


@dataclass(frozen=True)
class ImuRawSample:
    stamp_s: float
    accel_mps2: ImuVector3
    gyro_dps: ImuVector3
    euler_deg: ImuEuler = field(default_factory=ImuEuler)
    mag_ut: ImuVector3 | None = None
    temp_c: float | None = None
    calibration: ImuCalibration = field(default_factory=ImuCalibration)
    sys_status: int = 0
    sys_error: int = 0
    moving: bool = False
    vibe_score: float = 0.0

    @property
    def gravity_norm_mps2(self) -> float:
        return self.accel_mps2.magnitude

    @property
    def gyro_norm_dps(self) -> float:
        return self.gyro_dps.magnitude

    @property
    def mag_norm_ut(self) -> float | None:
        if self.mag_ut is None:
            return None

        return self.mag_ut.magnitude

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "accel_mps2": self.accel_mps2.to_dict(),
            "gyro_dps": self.gyro_dps.to_dict(),
            "euler_deg": self.euler_deg.to_dict(),
            "mag_ut": self.mag_ut.to_dict() if self.mag_ut else None,
            "temp_c": self.temp_c,
            "calibration": self.calibration.to_dict(),
            "sys_status": int(self.sys_status),
            "sys_error": int(self.sys_error),
            "moving": bool(self.moving),
            "vibe_score": float(self.vibe_score),
            "gravity_norm_mps2": self.gravity_norm_mps2,
            "gyro_norm_dps": self.gyro_norm_dps,
            "mag_norm_ut": self.mag_norm_ut,
        }


@dataclass
class ImuHealthState:
    status: str = STATUS_UNKNOWN
    grade: str = IMU_HEALTH_GRADE_C
    message: str = "IMU state unknown"
    reasons: list[str] = field(default_factory=list)

    chip_ok: bool = False
    hardware_ok: bool = False
    calibration_ok: bool = False
    motion_ready: bool = False
    data_ok: bool = False
    temperature_ok: bool = False

    gravity_norm_mps2: float | None = None
    gyro_z_bias_dps: float | None = None
    temp_c: float | None = None
    moving: bool = False
    vibe_score: float = 0.0

    def mark_ok(self, message: str = "IMU healthy") -> None:
        self.status = STATUS_OK
        self.grade = IMU_HEALTH_GRADE_A
        self.message = message
        self.reasons.clear()
        self.hardware_ok = True
        self.data_ok = True

    def mark_warn(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_WARN
        self.grade = IMU_HEALTH_GRADE_B
        self.message = message
        self.reasons = list(reasons or [])

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_ERROR
        self.grade = IMU_HEALTH_GRADE_C
        self.message = message
        self.reasons = list(reasons or [])

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ImuState:
    model: str = IMU_MODEL_BNO055
    frame_id: str = FRAME_IMU

    i2c_bus: int = BNO055_DEFAULT_I2C_BUS
    i2c_address: int = BNO055_DEFAULT_ADDRESS
    mode: str = BNO055_DEFAULT_MODE

    chip_id: int | None = None
    sys_status: int = 0
    sys_error: int = 0
    calibration: ImuCalibration = field(default_factory=ImuCalibration)

    sample_count: int = 0
    last_sample: ImuRawSample | None = None
    last_sample_age_s: float | None = None

    health: ImuHealthState = field(default_factory=ImuHealthState)

    def mark_chip(self, chip_id: int) -> None:
        self.chip_id = int(chip_id)
        self.health.chip_ok = self.chip_id == BNO055_CHIP_ID
        self.health.hardware_ok = self.health.chip_ok

        if not self.health.chip_ok:
            self.health.mark_error(
                f"Unexpected IMU chip id: 0x{self.chip_id:02X}",
                reasons=[f"expected 0x{BNO055_CHIP_ID:02X}"],
            )

    def mark_sample(self, sample: ImuRawSample) -> None:
        self.last_sample = sample
        self.sample_count += 1

        self.sys_status = sample.sys_status
        self.sys_error = sample.sys_error
        self.calibration = sample.calibration

        self.health.data_ok = True
        self.health.motion_ready = sample.calibration.motion_ready
        self.health.gravity_norm_mps2 = sample.gravity_norm_mps2
        self.health.temp_c = sample.temp_c
        self.health.moving = sample.moving
        self.health.vibe_score = sample.vibe_score

        if sample.sys_error != 0:
            self.health.mark_error(
                f"IMU system error: {sample.sys_error}",
                reasons=[f"SYS_ERR={sample.sys_error}"],
            )
            return

        gravity_error = abs(sample.gravity_norm_mps2 - GRAVITY_MPS2)

        if gravity_error > 0.8:
            self.health.mark_error(
                "IMU gravity norm is far from expected value",
                reasons=[f"|g|={sample.gravity_norm_mps2:.3f} m/s²"],
            )
            return

        reasons: list[str] = []

        if gravity_error > 0.4:
            reasons.append(f"|g| slightly off: {sample.gravity_norm_mps2:.3f} m/s²")

        if sample.calibration.gyro < 2:
            reasons.append(f"gyro calibration low: {sample.calibration.gyro}/3")

        if sample.calibration.accel < 2:
            reasons.append(f"accel calibration low: {sample.calibration.accel}/3")

        if reasons:
            self.health.mark_warn("IMU usable with calibration notes", reasons=reasons)
            self.health.hardware_ok = True
            self.health.data_ok = True
            return

        self.health.mark_ok()

    def mark_stale(self, age_s: float | None) -> None:
        self.last_sample_age_s = age_s
        self.health.mark_warn(
            "IMU sample stale",
            reasons=[
                "no sample age available" if age_s is None else f"age_s={age_s:.3f}"
            ],
        )

    @property
    def i2c_address_hex(self) -> str:
        return hex(int(self.i2c_address))

    @property
    def chip_ok(self) -> bool:
        return self.chip_id == BNO055_CHIP_ID

    @property
    def ready(self) -> bool:
        return self.health.hardware_ok and self.health.data_ok

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "frame_id": self.frame_id,
            "i2c_bus": int(self.i2c_bus),
            "i2c_address": int(self.i2c_address),
            "i2c_address_hex": self.i2c_address_hex,
            "mode": self.mode,
            "chip_id": self.chip_id,
            "chip_ok": self.chip_ok,
            "sys_status": int(self.sys_status),
            "sys_error": int(self.sys_error),
            "calibration": self.calibration.to_dict(),
            "sample_count": int(self.sample_count),
            "last_sample_age_s": self.last_sample_age_s,
            "last_sample": self.last_sample.to_dict() if self.last_sample else None,
            "health": self.health.to_dict(),
            "ready": self.ready,
        }


def make_imu_vector3(x: float, y: float, z: float) -> ImuVector3:
    return ImuVector3(float(x), float(y), float(z))


def make_imu_calibration(
    system: int,
    gyro: int,
    accel: int,
    mag: int,
) -> ImuCalibration:
    calibration = ImuCalibration(
        system=int(system),
        gyro=int(gyro),
        accel=int(accel),
        mag=int(mag),
    )
    calibration.validate()
    return calibration


def make_imu_sample(
    *,
    stamp_s: float,
    accel_xyz: tuple[float, float, float],
    gyro_xyz: tuple[float, float, float],
    euler_ypr: tuple[float | None, float | None, float | None] = (None, None, None),
    mag_xyz: tuple[float, float, float] | None = None,
    temp_c: float | None = None,
    calibration: ImuCalibration | None = None,
    sys_status: int = 0,
    sys_error: int = 0,
    moving: bool = False,
    vibe_score: float = 0.0,
) -> ImuRawSample:
    yaw, roll, pitch = euler_ypr

    return ImuRawSample(
        stamp_s=float(stamp_s),
        accel_mps2=make_imu_vector3(*accel_xyz),
        gyro_dps=make_imu_vector3(*gyro_xyz),
        euler_deg=ImuEuler(yaw_deg=yaw, roll_deg=roll, pitch_deg=pitch),
        mag_ut=make_imu_vector3(*mag_xyz) if mag_xyz is not None else None,
        temp_c=temp_c,
        calibration=calibration or ImuCalibration(),
        sys_status=int(sys_status),
        sys_error=int(sys_error),
        moving=bool(moving),
        vibe_score=float(vibe_score),
    )


def imu_is_moving(
    *,
    gyro_rms_dps: float,
    accel_std_mps2: float,
    gyro_threshold_dps: float = DEFAULT_IMU_GYRO_RMS_MOVING_DPS,
    accel_threshold_mps2: float = DEFAULT_IMU_ACCEL_STD_MOVING_MPS2,
) -> bool:
    return (
        float(gyro_rms_dps) > float(gyro_threshold_dps)
        or float(accel_std_mps2) > float(accel_threshold_mps2)
    )