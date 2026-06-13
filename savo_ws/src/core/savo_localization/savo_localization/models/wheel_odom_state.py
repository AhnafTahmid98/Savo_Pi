#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wheel odometry state models for Robot Savo. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_ODOM,
    ODOM_MODEL_MECANUM_4ENC,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
)
from savo_localization.math.mecanum_kinematics import WheelSpeeds
from savo_localization.math.angle_math import shortest_angle_delta_rad
from savo_localization.math.odom_math import (
    Pose2D,
    Twist2D,
    distance_2d_m,
    pose_is_finite,
    twist_is_finite,
)
from savo_localization.utils.frames import require_frame_pair


@dataclass
class WheelOdomSample:
    stamp_s: float = 0.0
    dt_s: float = 0.0

    pose: Pose2D = field(default_factory=Pose2D)
    twist: Twist2D = field(default_factory=Twist2D)
    wheel_speeds: WheelSpeeds = field(default_factory=WheelSpeeds)

    odom_frame_id: str = FRAME_ODOM
    base_frame_id: str = FRAME_BASE_LINK

    active_wheel_count: int = 0
    encoder_sample_count: int = 0

    @property
    def frame_ok(self) -> bool:
        try:
            require_frame_pair(self.odom_frame_id, self.base_frame_id)
        except ValueError:
            return False

        return True

    @property
    def pose_finite(self) -> bool:
        return pose_is_finite(self.pose)

    @property
    def twist_finite(self) -> bool:
        return twist_is_finite(self.twist)

    @property
    def finite_ok(self) -> bool:
        return self.pose_finite and self.twist_finite

    @property
    def valid(self) -> bool:
        return self.frame_ok and self.finite_ok

    @property
    def linear_speed_mps(self) -> float:
        return math.hypot(float(self.twist.vx_mps), float(self.twist.vy_mps))

    @property
    def angular_speed_rad_s(self) -> float:
        return abs(float(self.twist.omega_rad_s))

    @property
    def finite(self) -> bool:
        return self.finite_ok

    @property
    def moving(self) -> bool:
        return (
            self.linear_speed_mps > 1e-9
            or self.angular_speed_rad_s > 1e-9
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "dt_s": float(self.dt_s),
            "pose": {
                "x_m": float(self.pose.x_m),
                "y_m": float(self.pose.y_m),
                "yaw_rad": float(self.pose.yaw_rad),
            },
            "twist": {
                "vx_mps": float(self.twist.vx_mps),
                "vy_mps": float(self.twist.vy_mps),
                "omega_rad_s": float(self.twist.omega_rad_s),
            },
            "wheel_speeds": {
                "fl_mps": float(self.wheel_speeds.fl_mps),
                "fr_mps": float(self.wheel_speeds.fr_mps),
                "rl_mps": float(self.wheel_speeds.rl_mps),
                "rr_mps": float(self.wheel_speeds.rr_mps),
            },
            "odom_frame_id": self.odom_frame_id,
            "base_frame_id": self.base_frame_id,
            "active_wheel_count": int(self.active_wheel_count),
            "encoder_sample_count": int(self.encoder_sample_count),
            "frame_ok": self.frame_ok,
            "pose_finite": self.pose_finite,
            "twist_finite": self.twist_finite,
            "finite_ok": self.finite_ok,
            "finite": self.finite,
            "valid": self.valid,
            "moving": self.moving,
            "linear_speed_mps": self.linear_speed_mps,
            "angular_speed_rad_s": self.angular_speed_rad_s,
        }


@dataclass
class WheelOdomHealthState:
    status: str = STATUS_UNKNOWN
    message: str = "wheel odometry state unknown"
    reasons: list[str] = field(default_factory=list)

    hardware_ok: bool = False
    data_ok: bool = False
    rate_ok: bool = False
    frame_ok: bool = False
    finite_ok: bool = False

    sample_count: int = 0
    active_wheel_count: int = 0
    rate_hz: float = 0.0
    last_age_s: float | None = None

    @property
    def ready(self) -> bool:
        return (
            self.status == STATUS_OK
            and self.hardware_ok
            and self.data_ok
            and self.rate_ok
            and self.frame_ok
            and self.finite_ok
            and self.sample_count > 0
        )

    @property
    def ok(self) -> bool:
        return self.ready

    def mark_ok(self, message: str = "wheel odometry healthy") -> None:
        self.status = STATUS_OK
        self.message = message
        self.reasons.clear()
        self.hardware_ok = True
        self.data_ok = True
        self.rate_ok = True
        self.frame_ok = True
        self.finite_ok = True

    def mark_warn(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_WARN
        self.message = message
        self.reasons = list(reasons or [])

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_ERROR
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = False

    def mark_stale(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_STALE
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "message": self.message,
            "reasons": list(self.reasons),
            "hardware_ok": self.hardware_ok,
            "data_ok": self.data_ok,
            "rate_ok": self.rate_ok,
            "frame_ok": self.frame_ok,
            "finite_ok": self.finite_ok,
            "sample_count": int(self.sample_count),
            "active_wheel_count": int(self.active_wheel_count),
            "rate_hz": float(self.rate_hz),
            "last_age_s": self.last_age_s,
            "ready": self.ready,
            "ok": self.ok,
        }


@dataclass
class WheelOdomState:
    model: str = ODOM_MODEL_MECANUM_4ENC

    sample_count: int = 0
    last_sample: WheelOdomSample | None = None
    last_sample_age_s: float | None = None

    total_distance_m: float = 0.0
    total_rotation_rad: float = 0.0

    health: WheelOdomHealthState = field(default_factory=WheelOdomHealthState)

    @property
    def ready(self) -> bool:
        return self.health.ready

    def mark_sample(
        self,
        sample: WheelOdomSample,
        *,
        now_s: float | None = None,
    ) -> None:
        if self.last_sample is not None:
            self.total_distance_m += distance_2d_m(
                self.last_sample.pose,
                sample.pose,
            )
            self.total_rotation_rad += abs(
                shortest_angle_delta_rad(
                    self.last_sample.pose.yaw_rad,
                    sample.pose.yaw_rad,
                )
            )

        self.last_sample = sample
        self.sample_count += 1

        if now_s is None:
            self.last_sample_age_s = 0.0
        else:
            self.last_sample_age_s = max(0.0, float(now_s) - float(sample.stamp_s))

        self.health.sample_count = self.sample_count
        self.health.active_wheel_count = sample.active_wheel_count
        self.health.last_age_s = self.last_sample_age_s
        self.health.rate_hz = 1.0 / sample.dt_s if sample.dt_s > 0.0 else 0.0
        self.health.hardware_ok = True
        self.health.data_ok = True
        self.health.rate_ok = sample.dt_s > 0.0
        self.health.frame_ok = sample.frame_ok
        self.health.finite_ok = sample.finite_ok

        if sample.valid and sample.dt_s > 0.0:
            self.health.mark_ok()
        else:
            reasons: list[str] = []
            if sample.dt_s <= 0.0:
                reasons.append("dt_s must be > 0.0")
            if not sample.frame_ok:
                reasons.append("wheel odom frame ids are invalid")
            if not sample.finite_ok:
                reasons.append("wheel odom pose or twist is non-finite")
            self.health.mark_error("wheel odometry sample not valid", reasons)

        self.health.sample_count = self.sample_count
        self.health.active_wheel_count = sample.active_wheel_count
        self.health.last_age_s = self.last_sample_age_s
        self.health.rate_hz = 1.0 / sample.dt_s if sample.dt_s > 0.0 else 0.0

    def mark_stale(self, age_s: float) -> None:
        self.last_sample_age_s = float(age_s)
        self.health.last_age_s = float(age_s)
        self.health.mark_stale(
            "wheel odometry state stale",
            [f"last sample age_s={float(age_s):.3f}"],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "ready": self.ready,
            "sample_count": int(self.sample_count),
            "last_sample_age_s": self.last_sample_age_s,
            "total_distance_m": float(self.total_distance_m),
            "total_rotation_rad": float(self.total_rotation_rad),
            "health": self.health.to_dict(),
            "last_sample": (
                self.last_sample.to_dict()
                if self.last_sample is not None
                else None
            ),
        }



def make_zero_wheel_speeds() -> WheelSpeeds:
    return WheelSpeeds(
        fl_mps=0.0,
        fr_mps=0.0,
        rl_mps=0.0,
        rr_mps=0.0,
    )


def wheel_speeds_from_dict(data: dict[str, float] | None = None) -> WheelSpeeds:
    data = data or {}

    return WheelSpeeds(
        fl_mps=float(data.get("fl_mps", data.get("FL", data.get("fl", 0.0)))),
        fr_mps=float(data.get("fr_mps", data.get("FR", data.get("fr", 0.0)))),
        rl_mps=float(data.get("rl_mps", data.get("RL", data.get("rl", 0.0)))),
        rr_mps=float(data.get("rr_mps", data.get("RR", data.get("rr", 0.0)))),
    )


def wheel_speeds_to_dict(wheel_speeds: WheelSpeeds) -> dict[str, float]:
    return {
        "fl_mps": float(wheel_speeds.fl_mps),
        "fr_mps": float(wheel_speeds.fr_mps),
        "rl_mps": float(wheel_speeds.rl_mps),
        "rr_mps": float(wheel_speeds.rr_mps),
    }


def make_wheel_odom_sample(
    *,
    stamp_s: float,
    dt_s: float,
    pose: Pose2D,
    twist: Twist2D,
    wheel_speeds: WheelSpeeds | None = None,
    odom_frame_id: str = FRAME_ODOM,
    base_frame_id: str = FRAME_BASE_LINK,
    active_wheel_count: int = 0,
    encoder_sample_count: int = 0,
) -> WheelOdomSample:
    if dt_s <= 0.0:
        raise ValueError("dt_s must be > 0.0")

    require_frame_pair(odom_frame_id, base_frame_id)

    return WheelOdomSample(
        stamp_s=stamp_s,
        dt_s=dt_s,
        pose=pose,
        twist=twist,
        wheel_speeds=wheel_speeds or WheelSpeeds(),
        odom_frame_id=odom_frame_id,
        base_frame_id=base_frame_id,
        active_wheel_count=active_wheel_count,
        encoder_sample_count=encoder_sample_count,
    )


__all__ = [
    "WheelOdomSample",
    "WheelOdomHealthState",
    "WheelOdomState",
    "make_zero_wheel_speeds",
    "wheel_speeds_to_dict",
    "wheel_speeds_from_dict",
    "make_wheel_odom_sample",
]
