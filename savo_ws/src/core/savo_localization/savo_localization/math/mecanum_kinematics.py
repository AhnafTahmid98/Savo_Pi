#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mecanum kinematics helpers for Robot Savo localization."""

from __future__ import annotations

import math
from dataclasses import dataclass

from savo_localization.constants import (
    DEFAULT_TRACK_M,
    DEFAULT_WHEELBASE_M,
)


@dataclass(frozen=True)
class MecanumGeometry:
    wheelbase_m: float = DEFAULT_WHEELBASE_M
    track_m: float = DEFAULT_TRACK_M

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be > 0.0")

        if self.track_m <= 0.0:
            raise ValueError("track_m must be > 0.0")

    @property
    def radius_sum_m(self) -> float:
        return radius_sum_m(self.wheelbase_m, self.track_m)


@dataclass(frozen=True)
class WheelSpeeds:
    fl_mps: float = 0.0
    fr_mps: float = 0.0
    rl_mps: float = 0.0
    rr_mps: float = 0.0


@dataclass(frozen=True)
class BodyVelocity:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rad_s: float = 0.0


def radius_sum_m(wheelbase_m: float, track_m: float) -> float:
    wheelbase = float(wheelbase_m)
    track = float(track_m)

    if wheelbase <= 0.0:
        raise ValueError("wheelbase_m must be > 0.0")

    if track <= 0.0:
        raise ValueError("track_m must be > 0.0")

    return (wheelbase + track) / 2.0


def body_velocity_from_wheel_speeds(
    wheels: WheelSpeeds,
    geometry: MecanumGeometry | None = None,
) -> BodyVelocity:
    geometry = geometry or MecanumGeometry()
    r_sum = geometry.radius_sum_m

    vx = (
        wheels.fl_mps
        + wheels.fr_mps
        + wheels.rl_mps
        + wheels.rr_mps
    ) / 4.0

    vy = (
        -wheels.fl_mps
        + wheels.fr_mps
        + wheels.rl_mps
        - wheels.rr_mps
    ) / 4.0

    omega = (
        -wheels.fl_mps
        + wheels.fr_mps
        - wheels.rl_mps
        + wheels.rr_mps
    ) / (4.0 * r_sum)

    return BodyVelocity(
        vx_mps=vx,
        vy_mps=vy,
        omega_rad_s=omega,
    )


def forward_kinematics(
    wheels: WheelSpeeds,
    geometry: MecanumGeometry | None = None,
) -> BodyVelocity:
    return body_velocity_from_wheel_speeds(wheels, geometry)


def wheel_speeds_from_body_velocity(
    velocity: BodyVelocity,
    geometry: MecanumGeometry | None = None,
) -> WheelSpeeds:
    geometry = geometry or MecanumGeometry()
    r_sum = geometry.radius_sum_m
    rotation = r_sum * velocity.omega_rad_s

    return WheelSpeeds(
        fl_mps=velocity.vx_mps - velocity.vy_mps - rotation,
        fr_mps=velocity.vx_mps + velocity.vy_mps + rotation,
        rl_mps=velocity.vx_mps + velocity.vy_mps - rotation,
        rr_mps=velocity.vx_mps - velocity.vy_mps + rotation,
    )


def inverse_kinematics(
    velocity: BodyVelocity,
    geometry: MecanumGeometry | None = None,
) -> WheelSpeeds:
    return wheel_speeds_from_body_velocity(velocity, geometry)


def forward_kinematics_from_dict(
    wheel_speeds: dict[str, float],
    geometry: MecanumGeometry | None = None,
) -> BodyVelocity:
    return body_velocity_from_wheel_speeds(
        WheelSpeeds(
            fl_mps=float(wheel_speeds.get("FL", wheel_speeds.get("fl", 0.0))),
            fr_mps=float(wheel_speeds.get("FR", wheel_speeds.get("fr", 0.0))),
            rl_mps=float(wheel_speeds.get("RL", wheel_speeds.get("rl", 0.0))),
            rr_mps=float(wheel_speeds.get("RR", wheel_speeds.get("rr", 0.0))),
        ),
        geometry,
    )


def wheel_speed_tuple(wheels: WheelSpeeds) -> tuple[float, float, float, float]:
    return (
        wheels.fl_mps,
        wheels.fr_mps,
        wheels.rl_mps,
        wheels.rr_mps,
    )


def body_velocity_tuple(velocity: BodyVelocity) -> tuple[float, float, float]:
    return (
        velocity.vx_mps,
        velocity.vy_mps,
        velocity.omega_rad_s,
    )


def max_abs_wheel_speed(wheels: WheelSpeeds) -> float:
    return max(abs(value) for value in wheel_speed_tuple(wheels))


def scale_wheel_speeds(wheels: WheelSpeeds, scale: float) -> WheelSpeeds:
    return WheelSpeeds(
        fl_mps=wheels.fl_mps * scale,
        fr_mps=wheels.fr_mps * scale,
        rl_mps=wheels.rl_mps * scale,
        rr_mps=wheels.rr_mps * scale,
    )


def normalize_wheel_speeds(
    wheels: WheelSpeeds,
    max_speed_mps: float,
) -> WheelSpeeds:
    if max_speed_mps <= 0.0:
        raise ValueError("max_speed_mps must be > 0.0")

    current_max = max_abs_wheel_speed(wheels)

    if current_max <= max_speed_mps:
        return wheels

    return scale_wheel_speeds(wheels, max_speed_mps / current_max)


def body_velocity_is_near_zero(
    velocity: BodyVelocity,
    *,
    linear_tolerance_mps: float = 1e-6,
    angular_tolerance_rad_s: float = 1e-6,
) -> bool:
    if linear_tolerance_mps < 0.0:
        raise ValueError("linear_tolerance_mps must be >= 0.0")

    if angular_tolerance_rad_s < 0.0:
        raise ValueError("angular_tolerance_rad_s must be >= 0.0")

    linear = math.hypot(velocity.vx_mps, velocity.vy_mps)

    return (
        linear <= linear_tolerance_mps
        and abs(velocity.omega_rad_s) <= angular_tolerance_rad_s
    )


__all__ = [
    "BodyVelocity",
    "MecanumGeometry",
    "WheelSpeeds",
    "radius_sum_m",
    "body_velocity_from_wheel_speeds",
    "forward_kinematics",
    "forward_kinematics_from_dict",
    "wheel_speeds_from_body_velocity",
    "inverse_kinematics",
    "wheel_speed_tuple",
    "body_velocity_tuple",
    "max_abs_wheel_speed",
    "scale_wheel_speeds",
    "normalize_wheel_speeds",
    "body_velocity_is_near_zero",
]
