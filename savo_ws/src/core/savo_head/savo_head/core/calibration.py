# -*- coding: utf-8 -*-

"""Servo calibration helpers for Robot Savo head fallback tools."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Final

from savo_head.constants import (
    FREENOVE_LOGICAL_TO_PCA9685_CHANNEL,
    PAN_CENTER_DEG_DEFAULT,
    PAN_LOGICAL_CHANNEL_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
    PCA9685_PULSE_MAX_US,
    PCA9685_PULSE_MIN_US,
    PCA9685_SERVO_PERIOD_US,
    PCA9685_TICKS_PER_CYCLE,
    SERVO_ANGLE_MAX_DEG,
    SERVO_ANGLE_MIN_DEG,
    SERVO_ERROR_DEFAULT_DEG,
    TILT_CENTER_DEG_DEFAULT,
    TILT_LOGICAL_CHANNEL_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
)


SERVO_DIRECTION_NORMAL: Final[str] = "normal"
SERVO_DIRECTION_REVERSED: Final[str] = "reversed"


@dataclass(frozen=True)
class ServoChannelCalibration:
    logical_channel: str
    pca9685_channel: int
    min_deg: int
    center_deg: int
    max_deg: int
    error_deg: int = SERVO_ERROR_DEFAULT_DEG
    direction: str = SERVO_DIRECTION_NORMAL

    def normalized(self) -> "ServoChannelCalibration":
        logical = str(self.logical_channel)

        if logical not in FREENOVE_LOGICAL_TO_PCA9685_CHANNEL:
            valid = ", ".join(sorted(FREENOVE_LOGICAL_TO_PCA9685_CHANNEL))
            raise ValueError(f"invalid logical channel {logical!r}; valid: {valid}")

        min_deg = max(SERVO_ANGLE_MIN_DEG, min(SERVO_ANGLE_MAX_DEG, int(self.min_deg)))
        max_deg = max(SERVO_ANGLE_MIN_DEG, min(SERVO_ANGLE_MAX_DEG, int(self.max_deg)))

        if min_deg > max_deg:
            min_deg, max_deg = max_deg, min_deg

        center = max(min_deg, min(max_deg, int(self.center_deg)))
        direction = (
            SERVO_DIRECTION_REVERSED
            if self.direction == SERVO_DIRECTION_REVERSED
            else SERVO_DIRECTION_NORMAL
        )

        return replace(
            self,
            logical_channel=logical,
            pca9685_channel=FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[logical],
            min_deg=min_deg,
            center_deg=center,
            max_deg=max_deg,
            error_deg=int(self.error_deg),
            direction=direction,
        )

    def clamp_angle(self, angle_deg: int) -> int:
        item = self.normalized()
        return max(item.min_deg, min(item.max_deg, int(angle_deg)))

    def validation_errors(self) -> list[str]:
        errors: list[str] = []

        if str(self.logical_channel) not in FREENOVE_LOGICAL_TO_PCA9685_CHANNEL:
            errors.append(f"invalid logical_channel: {self.logical_channel!r}")

        if int(self.pca9685_channel) != FREENOVE_LOGICAL_TO_PCA9685_CHANNEL.get(
            str(self.logical_channel),
            int(self.pca9685_channel),
        ):
            errors.append("pca9685_channel does not match Freenove logical mapping")

        if int(self.min_deg) < SERVO_ANGLE_MIN_DEG:
            errors.append("min_deg is below servo angle minimum")

        if int(self.max_deg) > SERVO_ANGLE_MAX_DEG:
            errors.append("max_deg is above servo angle maximum")

        if int(self.min_deg) >= int(self.max_deg):
            errors.append("min_deg must be lower than max_deg")

        if not int(self.min_deg) <= int(self.center_deg) <= int(self.max_deg):
            errors.append("center_deg must be inside min/max range")

        if self.direction not in (SERVO_DIRECTION_NORMAL, SERVO_DIRECTION_REVERSED):
            errors.append(f"invalid direction: {self.direction!r}")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()


@dataclass(frozen=True)
class HeadServoCalibration:
    pan: ServoChannelCalibration = ServoChannelCalibration(
        logical_channel=PAN_LOGICAL_CHANNEL_DEFAULT,
        pca9685_channel=FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[PAN_LOGICAL_CHANNEL_DEFAULT],
        min_deg=PAN_MIN_DEG_DEFAULT,
        center_deg=PAN_CENTER_DEG_DEFAULT,
        max_deg=PAN_MAX_DEG_DEFAULT,
    )
    tilt: ServoChannelCalibration = ServoChannelCalibration(
        logical_channel=TILT_LOGICAL_CHANNEL_DEFAULT,
        pca9685_channel=FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[TILT_LOGICAL_CHANNEL_DEFAULT],
        min_deg=TILT_MIN_DEG_DEFAULT,
        center_deg=TILT_CENTER_DEG_DEFAULT,
        max_deg=TILT_MAX_DEG_DEFAULT,
    )

    def normalized(self) -> "HeadServoCalibration":
        return HeadServoCalibration(
            pan=self.pan.normalized(),
            tilt=self.tilt.normalized(),
        )

    def validation_errors(self) -> list[str]:
        errors: list[str] = []
        errors.extend(f"pan: {item}" for item in self.pan.validation_errors())
        errors.extend(f"tilt: {item}" for item in self.tilt.validation_errors())
        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()


DEFAULT_SERVO_CALIBRATION: Final[HeadServoCalibration] = HeadServoCalibration()


def clamp_pulse_us(pulse_us: float) -> float:
    value = float(pulse_us)
    if value < PCA9685_PULSE_MIN_US:
        return PCA9685_PULSE_MIN_US
    if value > PCA9685_PULSE_MAX_US:
        return PCA9685_PULSE_MAX_US
    return value


def angle_to_pulse_us(
    angle_deg: int,
    *,
    error_deg: int = SERVO_ERROR_DEFAULT_DEG,
    direction: str = SERVO_DIRECTION_NORMAL,
) -> float:
    angle = int(angle_deg)

    if direction == SERVO_DIRECTION_REVERSED:
        pulse = 2500.0 - float((angle + int(error_deg)) / 0.09)
    else:
        pulse = 500.0 + float((angle + int(error_deg)) / 0.09)

    return clamp_pulse_us(pulse)


def pulse_us_to_ticks(pulse_us: float) -> int:
    pulse = clamp_pulse_us(pulse_us)
    ticks = pulse * float(PCA9685_TICKS_PER_CYCLE) / float(PCA9685_SERVO_PERIOD_US)
    return int(ticks)


def angle_to_ticks(
    angle_deg: int,
    *,
    error_deg: int = SERVO_ERROR_DEFAULT_DEG,
    direction: str = SERVO_DIRECTION_NORMAL,
) -> int:
    return pulse_us_to_ticks(
        angle_to_pulse_us(
            angle_deg,
            error_deg=error_deg,
            direction=direction,
        )
    )


def logical_channel_to_pca9685_channel(logical_channel: str) -> int:
    key = str(logical_channel)
    if key not in FREENOVE_LOGICAL_TO_PCA9685_CHANNEL:
        valid = ", ".join(sorted(FREENOVE_LOGICAL_TO_PCA9685_CHANNEL))
        raise ValueError(f"invalid logical channel {logical_channel!r}; valid: {valid}")
    return FREENOVE_LOGICAL_TO_PCA9685_CHANNEL[key]


def calibration_for_axis(axis: str, calibration: HeadServoCalibration | None = None) -> ServoChannelCalibration:
    cal = (calibration or DEFAULT_SERVO_CALIBRATION).normalized()
    axis_norm = str(axis).strip().lower()

    if axis_norm == "pan":
        return cal.pan
    if axis_norm == "tilt":
        return cal.tilt

    raise ValueError(f"unknown head axis: {axis!r}")


def angle_to_servo_output(
    axis: str,
    angle_deg: int,
    calibration: HeadServoCalibration | None = None,
) -> tuple[int, float, int]:
    item = calibration_for_axis(axis, calibration)
    angle = item.clamp_angle(angle_deg)
    pulse_us = angle_to_pulse_us(
        angle,
        error_deg=item.error_deg,
        direction=item.direction,
    )
    ticks = pulse_us_to_ticks(pulse_us)
    return item.pca9685_channel, pulse_us, ticks


__all__ = [
    "SERVO_DIRECTION_NORMAL",
    "SERVO_DIRECTION_REVERSED",
    "ServoChannelCalibration",
    "HeadServoCalibration",
    "DEFAULT_SERVO_CALIBRATION",
    "clamp_pulse_us",
    "angle_to_pulse_us",
    "pulse_us_to_ticks",
    "angle_to_ticks",
    "logical_channel_to_pca9685_channel",
    "calibration_for_axis",
    "angle_to_servo_output",
]
