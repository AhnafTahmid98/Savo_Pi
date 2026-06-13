#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wheel odometry state models for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

from savo_localization.constants import (
    DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S,
    DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS,
    DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    DEFAULT_WHEEL_ODOM_TOPIC,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    ODOM_MODEL_MECANUM_4ENC,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNKNOWN,
    STATUS_WARN,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.math.mecanum_kinematics import WheelSpeeds
from savo_localization.math.odom_math import (
    Pose2D,
    Twist2D,
    integrate_pose_2d,
    pose_is_finite,
    twist_is_finite,
    twist_magnitude,
    zero_pose,
    zero_twist,
)


@dataclass(frozen=True)
class WheelOdomSample:
    stamp_s: float
    dt_s: float

    pose: Pose2D
    twist: Twist2D
    wheel_speeds: WheelSpeeds

    odom_frame_id: str = FRAME_ODOM
    base_frame_id: str = FRAME_BASE_LINK

    active_wheel_count: int = 0
    encoder_sample_count: int = 0

    pose_covariance: list[float] = field(default_factory=list)
    twist_covariance: list[float] = field(default_factory=list)

    @property
    def linear_speed_mps(self) -> float:
        linear_mps, _ = twist_magnitude(self.twist)
        return linear_mps

    @property
    def angular_speed_rad_s(self) -> float:
        _, angular_rad_s = twist_magnitude(self.twist)
        return angular_rad_s

    @property
    def moving(self) -> bool:
        return self.linear_speed_mps > 0.0 or self.angular_speed_rad_s > 0.0

    @property
    def finite(self) -> bool:
        return pose_is_finite(self.pose) and twist_is_finite(self.twist)

    def to_dict(self) -> dict[str, Any]:
        return {
            "stamp_s": float(self.stamp_s),
            "dt_s": float(self.dt_s),
            "odom_frame_id": self.odom_frame_id,
            "base_frame_id": self.base_frame_id,
            "pose": self.pose.to_dict(),
            "twist": self.twist.to_dict(),
            "wheel_speeds": self.wheel_speeds.to_dict(),
            "linear_speed_mps": self.linear_speed_mps,
            "angular_speed_rad_s": self.angular_speed_rad_s,
            "moving": self.moving,
            "finite": self.finite,
            "active_wheel_count": int(self.active_wheel_count),
            "encoder_sample_count": int(self.encoder_sample_count),
            "pose_covariance": list(self.pose_covariance),
            "twist_covariance": list(self.twist_covariance),
        }


@dataclass
class WheelOdomHealthState:
    status: str = STATUS_UNKNOWN
    message: str = "wheel odometry state unknown"
    reasons: list[str] = field(default_factory=list)

    data_ok: bool = False
    pose_ok: bool = False
    twist_ok: bool = False
    finite_ok: bool = False
    speed_ok: bool = False
    frame_ok: bool = False

    sample_count: int = 0
    last_sample_age_s: float | None = None

    linear_speed_mps: float = 0.0
    angular_speed_rad_s: float = 0.0
    active_wheel_count: int = 0

    def mark_ok(self, message: str = "wheel odometry healthy") -> None:
        self.status = STATUS_OK
        self.message = message
        self.reasons.clear()

        self.data_ok = True
        self.pose_ok = True
        self.twist_ok = True
        self.finite_ok = True
        self.speed_ok = True
        self.frame_ok = True

    def mark_warn(self, message: str, reasons: list[str] | None = None) -> None:
        self.status = STATUS_WARN
        self.message = message
        self.reasons = list(reasons or [])
        self.data_ok = True

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
        return asdict(self)


@dataclass
class WheelOdomState:
    model: str = ODOM_MODEL_MECANUM_4ENC

    wheel_odom_topic: str = DEFAULT_WHEEL_ODOM_TOPIC
    wheel_odom_state_topic: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC

    odom_frame_id: str = FRAME_ODOM
    base_frame_id: str = FRAME_BASE_LINK

    pose: Pose2D = field(default_factory=zero_pose)
    twist: Twist2D = field(default_factory=zero_twist)

    wheel_speeds: WheelSpeeds = field(
        default_factory=lambda: WheelSpeeds(
            fl_mps=0.0,
            fr_mps=0.0,
            rl_mps=0.0,
            rr_mps=0.0,
        )
    )

    sample_count: int = 0
    encoder_sample_count: int = 0

    last_sample: WheelOdomSample | None = None
    last_sample_age_s: float | None = None

    total_distance_m: float = 0.0
    total_rotation_rad: float = 0.0

    health: WheelOdomHealthState = field(default_factory=WheelOdomHealthState)

    @property
    def ready(self) -> bool:
        return self.health.data_ok and self.health.finite_ok

    @property
    def linear_speed_mps(self) -> float:
        linear_mps, _ = twist_magnitude(self.twist)
        return linear_mps

    @property
    def angular_speed_rad_s(self) -> float:
        _, angular_rad_s = twist_magnitude(self.twist)
        return angular_rad_s

    @property
    def moving(self) -> bool:
        return self.linear_speed_mps > 0.0 or self.angular_speed_rad_s > 0.0

    def reset_pose(
        self,
        *,
        x_m: float = 0.0,
        y_m: float = 0.0,
        yaw_rad: float = 0.0,
    ) -> None:
        self.pose = Pose2D(
            x_m=float(x_m),
            y_m=float(y_m),
            yaw_rad=float(yaw_rad),
        ).normalized()
        self.twist = zero_twist()
        self.total_distance_m = 0.0
        self.total_rotation_rad = 0.0
        self.last_sample = None
        self.last_sample_age_s = None

    def integrate(
        self,
        *,
        twist: Twist2D,
        wheel_speeds: WheelSpeeds,
        dt_s: float,
        stamp_s: float,
        active_wheel_count: int = 0,
        encoder_sample_count: int = 0,
        pose_covariance: list[float] | None = None,
        twist_covariance: list[float] | None = None,
    ) -> WheelOdomSample:
        dt = max(float(dt_s), 0.0)
        previous_pose = self.pose

        self.pose = integrate_pose_2d(
            pose=self.pose,
            twist=twist,
            dt_s=dt,
        )
        self.twist = twist
        self.wheel_speeds = wheel_speeds

        linear_speed, angular_speed = twist_magnitude(twist)
        self.total_distance_m += linear_speed * dt
        self.total_rotation_rad += angular_speed * dt

        sample = WheelOdomSample(
            stamp_s=float(stamp_s),
            dt_s=dt,
            pose=self.pose,
            twist=self.twist,
            wheel_speeds=self.wheel_speeds,
            odom_frame_id=self.odom_frame_id,
            base_frame_id=self.base_frame_id,
            active_wheel_count=int(active_wheel_count),
            encoder_sample_count=int(encoder_sample_count),
            pose_covariance=list(pose_covariance or []),
            twist_covariance=list(twist_covariance or []),
        )

        self.mark_sample(sample)

        if not pose_is_finite(previous_pose):
            self.health.mark_error(
                "previous wheel odometry pose was not finite",
                reasons=["pose integration started from invalid state"],
            )

        return sample

    def mark_sample(self, sample: WheelOdomSample) -> None:
        self.last_sample = sample
        self.sample_count += 1
        self.encoder_sample_count = int(sample.encoder_sample_count)
        self.last_sample_age_s = 0.0

        self.pose = sample.pose
        self.twist = sample.twist
        self.wheel_speeds = sample.wheel_speeds

        self.health.sample_count = self.sample_count
        self.health.last_sample_age_s = self.last_sample_age_s
        self.health.linear_speed_mps = sample.linear_speed_mps
        self.health.angular_speed_rad_s = sample.angular_speed_rad_s
        self.health.active_wheel_count = int(sample.active_wheel_count)

        reasons: list[str] = []

        self.health.finite_ok = sample.finite
        self.health.pose_ok = pose_is_finite(sample.pose)
        self.health.twist_ok = twist_is_finite(sample.twist)
        self.health.frame_ok = (
            bool(sample.odom_frame_id.strip())
            and bool(sample.base_frame_id.strip())
            and sample.odom_frame_id != sample.base_frame_id
        )

        if not self.health.finite_ok:
            reasons.append("pose or twist contains non-finite values")

        if not self.health.frame_ok:
            reasons.append("wheel odometry frame ids are invalid")

        speed_ok = (
            sample.linear_speed_mps <= DEFAULT_MAX_ODOM_LINEAR_SPEED_MPS
            and sample.angular_speed_rad_s <= DEFAULT_MAX_ODOM_ANGULAR_SPEED_RAD_S
        )
        self.health.speed_ok = speed_ok

        if not speed_ok:
            reasons.append(
                "wheel odometry speed outside sanity limits: "
                f"linear={sample.linear_speed_mps:.3f} m/s, "
                f"angular={sample.angular_speed_rad_s:.3f} rad/s"
            )

        if sample.dt_s <= 0.0:
            reasons.append(f"invalid odometry dt_s={sample.dt_s:.6f}")

        if reasons:
            self.health.mark_warn("wheel odometry usable with notes", reasons=reasons)
            return

        self.health.mark_ok()

    def mark_stale(self, age_s: float | None) -> None:
        self.last_sample_age_s = age_s
        self.health.last_sample_age_s = age_s
        self.health.mark_stale(
            "wheel odometry sample stale",
            reasons=[
                "no sample age available" if age_s is None else f"age_s={age_s:.3f}"
            ],
        )

    def mark_error(self, message: str, reasons: list[str] | None = None) -> None:
        self.health.mark_error(message, reasons=reasons)

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "wheel_odom_topic": self.wheel_odom_topic,
            "wheel_odom_state_topic": self.wheel_odom_state_topic,
            "odom_frame_id": self.odom_frame_id,
            "base_frame_id": self.base_frame_id,
            "pose": self.pose.to_dict(),
            "twist": self.twist.to_dict(),
            "wheel_speeds": self.wheel_speeds.to_dict(),
            "linear_speed_mps": self.linear_speed_mps,
            "angular_speed_rad_s": self.angular_speed_rad_s,
            "moving": self.moving,
            "sample_count": int(self.sample_count),
            "encoder_sample_count": int(self.encoder_sample_count),
            "last_sample_age_s": self.last_sample_age_s,
            "total_distance_m": float(self.total_distance_m),
            "total_rotation_rad": float(self.total_rotation_rad),
            "last_sample": self.last_sample.to_dict() if self.last_sample else None,
            "health": self.health.to_dict(),
            "ready": self.ready,
        }


def make_wheel_odom_sample(
    *,
    stamp_s: float,
    dt_s: float,
    pose: Pose2D,
    twist: Twist2D,
    wheel_speeds: WheelSpeeds,
    odom_frame_id: str = FRAME_ODOM,
    base_frame_id: str = FRAME_BASE_LINK,
    active_wheel_count: int = 0,
    encoder_sample_count: int = 0,
    pose_covariance: list[float] | None = None,
    twist_covariance: list[float] | None = None,
) -> WheelOdomSample:
    if dt_s < 0.0:
        raise ValueError(f"dt_s must be >= 0.0, got {dt_s}")

    if not math.isfinite(float(stamp_s)):
        raise ValueError(f"stamp_s must be finite, got {stamp_s}")

    return WheelOdomSample(
        stamp_s=float(stamp_s),
        dt_s=float(dt_s),
        pose=pose.normalized(),
        twist=twist,
        wheel_speeds=wheel_speeds,
        odom_frame_id=str(odom_frame_id),
        base_frame_id=str(base_frame_id),
        active_wheel_count=int(active_wheel_count),
        encoder_sample_count=int(encoder_sample_count),
        pose_covariance=list(pose_covariance or []),
        twist_covariance=list(twist_covariance or []),
    )


def make_zero_wheel_speeds() -> WheelSpeeds:
    return WheelSpeeds(
        fl_mps=0.0,
        fr_mps=0.0,
        rl_mps=0.0,
        rr_mps=0.0,
    )


def wheel_speeds_from_dict(values: dict[str, float]) -> WheelSpeeds:
    return WheelSpeeds(
        fl_mps=float(values.get(WHEEL_FL, 0.0)),
        fr_mps=float(values.get(WHEEL_FR, 0.0)),
        rl_mps=float(values.get(WHEEL_RL, 0.0)),
        rr_mps=float(values.get(WHEEL_RR, 0.0)),
    )