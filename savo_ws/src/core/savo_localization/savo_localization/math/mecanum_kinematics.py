#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mecanum kinematics helpers for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass

from savo_localization.constants import (
    DEFAULT_TRACK_M,
    DEFAULT_WHEELBASE_M,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
)


@dataclass(frozen=True)
class MecanumGeometry:
    wheelbase_m: float = DEFAULT_WHEELBASE_M
    track_m: float = DEFAULT_TRACK_M

    def validate(self) -> None:
        if self.wheelbase_m <= 0.0:
            raise ValueError(f"wheelbase_m must be > 0.0, got {self.wheelbase_m}")

        if self.track_m <= 0.0:
            raise ValueError(f"track_m must be > 0.0, got {self.track_m}")

    @property
    def radius_sum_m(self) -> float:
        return radius_sum_m(
            wheelbase_m=self.wheelbase_m,
            track_m=self.track_m,
        )


@dataclass(frozen=True)
class WheelSpeeds:
    fl_mps: float
    fr_mps: float
    rl_mps: float
    rr_mps: float

    def to_dict(self) -> dict[str, float]:
        return {
            WHEEL_FL: self.fl_mps,
            WHEEL_FR: self.fr_mps,
            WHEEL_RL: self.rl_mps,
            WHEEL_RR: self.rr_mps,
        }


@dataclass(frozen=True)
class BodyVelocity:
    vx_mps: float
    vy_mps: float
    omega_rad_s: float

    def to_dict(self) -> dict[str, float]:
        return {
            "vx_mps": self.vx_mps,
            "vy_mps": self.vy_mps,
            "omega_rad_s": self.omega_rad_s,
        }


def radius_sum_m(*, wheelbase_m: float, track_m: float) -> float:
    wheelbase_m = float(wheelbase_m)
    track_m = float(track_m)

    if wheelbase_m <= 0.0:
        raise ValueError(f"wheelbase_m must be > 0.0, got {wheelbase_m}")

    if track_m <= 0.0:
        raise ValueError(f"track_m must be > 0.0, got {track_m}")

    return wheelbase_m + track_m


def forward_kinematics(
    *,
    fl_mps: float,
    fr_mps: float,
    rl_mps: float,
    rr_mps: float,
    wheelbase_m: float = DEFAULT_WHEELBASE_M,
    track_m: float = DEFAULT_TRACK_M,
) -> BodyVelocity:
    radius_sum = radius_sum_m(
        wheelbase_m=wheelbase_m,
        track_m=track_m,
    )

    fl = float(fl_mps)
    fr = float(fr_mps)
    rl = float(rl_mps)
    rr = float(rr_mps)

    vx_mps = (fl + fr + rl + rr) / 4.0
    vy_mps = (-fl + fr + rl - rr) / 4.0
    omega_rad_s = (-fl + fr - rl + rr) / (4.0 * radius_sum)

    return BodyVelocity(
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        omega_rad_s=omega_rad_s,
    )


def inverse_kinematics(
    *,
    vx_mps: float,
    vy_mps: float,
    omega_rad_s: float,
    wheelbase_m: float = DEFAULT_WHEELBASE_M,
    track_m: float = DEFAULT_TRACK_M,
) -> WheelSpeeds:
    radius_sum = radius_sum_m(
        wheelbase_m=wheelbase_m,
        track_m=track_m,
    )

    vx = float(vx_mps)
    vy = float(vy_mps)
    omega = float(omega_rad_s)

    rotation = radius_sum * omega

    return WheelSpeeds(
        fl_mps=vx - vy - rotation,
        fr_mps=vx + vy + rotation,
        rl_mps=vx + vy - rotation,
        rr_mps=vx - vy + rotation,
    )


def forward_kinematics_from_dict(
    wheel_speeds_mps: dict[str, float],
    *,
    wheelbase_m: float = DEFAULT_WHEELBASE_M,
    track_m: float = DEFAULT_TRACK_M,
) -> BodyVelocity:
    return forward_kinematics(
        fl_mps=float(wheel_speeds_mps.get(WHEEL_FL, 0.0)),
        fr_mps=float(wheel_speeds_mps.get(WHEEL_FR, 0.0)),
        rl_mps=float(wheel_speeds_mps.get(WHEEL_RL, 0.0)),
        rr_mps=float(wheel_speeds_mps.get(WHEEL_RR, 0.0)),
        wheelbase_m=wheelbase_m,
        track_m=track_m,
    )


def wheel_speeds_from_body_velocity(
    body_velocity: BodyVelocity,
    *,
    wheelbase_m: float = DEFAULT_WHEELBASE_M,
    track_m: float = DEFAULT_TRACK_M,
) -> WheelSpeeds:
    return inverse_kinematics(
        vx_mps=body_velocity.vx_mps,
        vy_mps=body_velocity.vy_mps,
        omega_rad_s=body_velocity.omega_rad_s,
        wheelbase_m=wheelbase_m,
        track_m=track_m,
    )


def body_velocity_is_near_zero(
    body_velocity: BodyVelocity,
    *,
    linear_tolerance_mps: float = 1e-6,
    angular_tolerance_rad_s: float = 1e-6,
) -> bool:
    if linear_tolerance_mps < 0.0:
        raise ValueError(
            f"linear_tolerance_mps must be >= 0.0, got {linear_tolerance_mps}"
        )

    if angular_tolerance_rad_s < 0.0:
        raise ValueError(
            f"angular_tolerance_rad_s must be >= 0.0, got {angular_tolerance_rad_s}"
        )

    return (
        abs(body_velocity.vx_mps) <= linear_tolerance_mps
        and abs(body_velocity.vy_mps) <= linear_tolerance_mps
        and abs(body_velocity.omega_rad_s) <= angular_tolerance_rad_s
    )


def scale_wheel_speeds(
    wheel_speeds: WheelSpeeds,
    scale: float,
) -> WheelSpeeds:
    scale = float(scale)

    return WheelSpeeds(
        fl_mps=wheel_speeds.fl_mps * scale,
        fr_mps=wheel_speeds.fr_mps * scale,
        rl_mps=wheel_speeds.rl_mps * scale,
        rr_mps=wheel_speeds.rr_mps * scale,
    )


def max_abs_wheel_speed(wheel_speeds: WheelSpeeds) -> float:
    return max(
        abs(wheel_speeds.fl_mps),
        abs(wheel_speeds.fr_mps),
        abs(wheel_speeds.rl_mps),
        abs(wheel_speeds.rr_mps),
    )


def normalize_wheel_speeds(
    wheel_speeds: WheelSpeeds,
    *,
    max_allowed_mps: float,
) -> WheelSpeeds:
    max_allowed_mps = abs(float(max_allowed_mps))

    if max_allowed_mps <= 0.0:
        raise ValueError(f"max_allowed_mps must be > 0.0, got {max_allowed_mps}")

    current_max = max_abs_wheel_speed(wheel_speeds)

    if current_max <= max_allowed_mps or current_max <= 0.0:
        return wheel_speeds

    return scale_wheel_speeds(
        wheel_speeds,
        scale=max_allowed_mps / current_max,
    )


def wheel_speed_tuple(wheel_speeds: WheelSpeeds) -> tuple[float, float, float, float]:
    return (
        wheel_speeds.fl_mps,
        wheel_speeds.fr_mps,
        wheel_speeds.rl_mps,
        wheel_speeds.rr_mps,
    )


def body_velocity_tuple(body_velocity: BodyVelocity) -> tuple[float, float, float]:
    return (
        body_velocity.vx_mps,
        body_velocity.vy_mps,
        body_velocity.omega_rad_s,
    )