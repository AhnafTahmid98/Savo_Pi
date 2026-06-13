#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""BNO055 calibration checks for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    IMU_HEALTH_GRADE_A,
    IMU_HEALTH_GRADE_B,
    IMU_HEALTH_GRADE_C,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_localization.models.imu_state import ImuCalibration


CALIB_MIN = 0
CALIB_MAX = 3
DEFAULT_READY_LEVEL = 2
FULLY_CALIBRATED_LEVEL = 3


@dataclass(frozen=True)
class ImuCalibrationCheckResult:
    status: str
    grade: str
    ok: bool
    message: str
    reasons: list[str] = field(default_factory=list)

    calibration: ImuCalibration = field(default_factory=ImuCalibration)
    require_system: bool = False
    require_gyro: bool = True
    require_accel: bool = True
    require_mag: bool = False
    ready_level: int = DEFAULT_READY_LEVEL

    @property
    def fully_calibrated(self) -> bool:
        return self.calibration.fully_calibrated

    @property
    def motion_ready(self) -> bool:
        return self.calibration.motion_ready

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "grade": self.grade,
            "ok": bool(self.ok),
            "message": self.message,
            "reasons": list(self.reasons),
            "calibration": self.calibration.to_dict(),
            "require_system": bool(self.require_system),
            "require_gyro": bool(self.require_gyro),
            "require_accel": bool(self.require_accel),
            "require_mag": bool(self.require_mag),
            "ready_level": int(self.ready_level),
            "fully_calibrated": self.fully_calibrated,
            "motion_ready": self.motion_ready,
        }


def check_imu_calibration(
    calibration: ImuCalibration,
    *,
    require_system: bool = False,
    require_gyro: bool = True,
    require_accel: bool = True,
    require_mag: bool = False,
    ready_level: int = DEFAULT_READY_LEVEL,
) -> ImuCalibrationCheckResult:
    _validate_ready_level(ready_level)

    try:
        calibration.validate()
    except ValueError as exc:
        return ImuCalibrationCheckResult(
            status=STATUS_ERROR,
            grade=IMU_HEALTH_GRADE_C,
            ok=False,
            message="IMU calibration values are invalid",
            reasons=[str(exc)],
            calibration=calibration,
            require_system=require_system,
            require_gyro=require_gyro,
            require_accel=require_accel,
            require_mag=require_mag,
            ready_level=ready_level,
        )

    required = _required_fields(
        require_system=require_system,
        require_gyro=require_gyro,
        require_accel=require_accel,
        require_mag=require_mag,
    )

    reasons: list[str] = []

    values = calibration.to_tuple_named()
    for name in required:
        value = values[name]
        if value < ready_level:
            reasons.append(
                f"{name} calibration low ({value}/{CALIB_MAX}), "
                f"required >= {ready_level}"
            )

    if reasons:
        return ImuCalibrationCheckResult(
            status=STATUS_WARN,
            grade=IMU_HEALTH_GRADE_B,
            ok=False,
            message="IMU calibration not ready",
            reasons=reasons,
            calibration=calibration,
            require_system=require_system,
            require_gyro=require_gyro,
            require_accel=require_accel,
            require_mag=require_mag,
            ready_level=ready_level,
        )

    if calibration.fully_calibrated:
        return ImuCalibrationCheckResult(
            status=STATUS_OK,
            grade=IMU_HEALTH_GRADE_A,
            ok=True,
            message="IMU fully calibrated",
            reasons=[],
            calibration=calibration,
            require_system=require_system,
            require_gyro=require_gyro,
            require_accel=require_accel,
            require_mag=require_mag,
            ready_level=ready_level,
        )

    return ImuCalibrationCheckResult(
        status=STATUS_OK,
        grade=IMU_HEALTH_GRADE_A,
        ok=True,
        message="IMU calibration ready",
        reasons=[],
        calibration=calibration,
        require_system=require_system,
        require_gyro=require_gyro,
        require_accel=require_accel,
        require_mag=require_mag,
        ready_level=ready_level,
    )


def check_motion_calibration(
    calibration: ImuCalibration,
    *,
    ready_level: int = DEFAULT_READY_LEVEL,
) -> ImuCalibrationCheckResult:
    return check_imu_calibration(
        calibration,
        require_system=False,
        require_gyro=True,
        require_accel=True,
        require_mag=False,
        ready_level=ready_level,
    )


def check_ndof_calibration(
    calibration: ImuCalibration,
    *,
    ready_level: int = DEFAULT_READY_LEVEL,
) -> ImuCalibrationCheckResult:
    return check_imu_calibration(
        calibration,
        require_system=False,
        require_gyro=True,
        require_accel=True,
        require_mag=True,
        ready_level=ready_level,
    )


def check_full_calibration(
    calibration: ImuCalibration,
) -> ImuCalibrationCheckResult:
    return check_imu_calibration(
        calibration,
        require_system=True,
        require_gyro=True,
        require_accel=True,
        require_mag=True,
        ready_level=FULLY_CALIBRATED_LEVEL,
    )


def calibration_tuple_to_model(
    calibration: tuple[int, int, int, int],
) -> ImuCalibration:
    if len(calibration) != 4:
        raise ValueError(
            "calibration tuple must contain exactly 4 values: "
            "(system, gyro, accel, mag)"
        )

    system, gyro, accel, mag = calibration

    result = ImuCalibration(
        system=int(system),
        gyro=int(gyro),
        accel=int(accel),
        mag=int(mag),
    )
    result.validate()
    return result


def calibration_from_dict(data: dict[str, Any]) -> ImuCalibration:
    result = ImuCalibration(
        system=int(data.get("system", data.get("sys", 0))),
        gyro=int(data.get("gyro", data.get("gyr", 0))),
        accel=int(data.get("accel", data.get("acc", 0))),
        mag=int(data.get("mag", 0)),
    )
    result.validate()
    return result


def calibration_level_name(level: int) -> str:
    level = int(level)

    if level <= 0:
        return "uncalibrated"

    if level == 1:
        return "low"

    if level == 2:
        return "usable"

    return "fully calibrated"


def calibration_summary(calibration: ImuCalibration) -> str:
    calibration.validate()

    values = calibration.to_tuple_named()

    return (
        f"sys={values['system']}/3 "
        f"gyro={values['gyro']}/3 "
        f"accel={values['accel']}/3 "
        f"mag={values['mag']}/3"
    )


def calibration_status_dict(calibration: ImuCalibration) -> dict[str, Any]:
    calibration.validate()

    values = calibration.to_tuple_named()

    return {
        "system": values["system"],
        "gyro": values["gyro"],
        "accel": values["accel"],
        "mag": values["mag"],
        "system_label": calibration_level_name(values["system"]),
        "gyro_label": calibration_level_name(values["gyro"]),
        "accel_label": calibration_level_name(values["accel"]),
        "mag_label": calibration_level_name(values["mag"]),
        "motion_ready": calibration.motion_ready,
        "fully_calibrated": calibration.fully_calibrated,
        "summary": calibration_summary(calibration),
    }


def calibration_is_motion_ready(
    calibration: ImuCalibration,
    *,
    ready_level: int = DEFAULT_READY_LEVEL,
) -> bool:
    _validate_ready_level(ready_level)
    calibration.validate()

    return calibration.gyro >= ready_level and calibration.accel >= ready_level


def calibration_is_ndof_ready(
    calibration: ImuCalibration,
    *,
    ready_level: int = DEFAULT_READY_LEVEL,
) -> bool:
    _validate_ready_level(ready_level)
    calibration.validate()

    return (
        calibration.gyro >= ready_level
        and calibration.accel >= ready_level
        and calibration.mag >= ready_level
    )


def calibration_is_fully_ready(calibration: ImuCalibration) -> bool:
    calibration.validate()
    return calibration.fully_calibrated


def _validate_ready_level(level: int) -> None:
    if not CALIB_MIN <= int(level) <= CALIB_MAX:
        raise ValueError(f"ready_level must be {CALIB_MIN}..{CALIB_MAX}, got {level}")


def _required_fields(
    *,
    require_system: bool,
    require_gyro: bool,
    require_accel: bool,
    require_mag: bool,
) -> tuple[str, ...]:
    fields: list[str] = []

    if require_system:
        fields.append("system")

    if require_gyro:
        fields.append("gyro")

    if require_accel:
        fields.append("accel")

    if require_mag:
        fields.append("mag")

    return tuple(fields)