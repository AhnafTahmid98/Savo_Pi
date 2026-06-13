#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Odometry integration helpers for Robot Savo localization. No ROS imports."""

from __future__ import annotations

import math
from dataclasses import dataclass

from savo_localization.math.angle_math import normalize_yaw_rad


@dataclass(frozen=True)
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def normalized(self) -> "Pose2D":
        return Pose2D(
            x_m=float(self.x_m),
            y_m=float(self.y_m),
            yaw_rad=normalize_yaw_rad(self.yaw_rad),
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "x_m": float(self.x_m),
            "y_m": float(self.y_m),
            "yaw_rad": normalize_yaw_rad(self.yaw_rad),
        }


@dataclass(frozen=True)
class Twist2D:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rad_s: float = 0.0

    def to_dict(self) -> dict[str, float]:
        return {
            "vx_mps": float(self.vx_mps),
            "vy_mps": float(self.vy_mps),
            "omega_rad_s": float(self.omega_rad_s),
        }


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def to_dict(self) -> dict[str, float]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "w": float(self.w),
        }


def integrate_pose_2d(
    pose: Pose2D,
    twist: Twist2D,
    dt_s: float,
) -> Pose2D:
    dt = max(0.0, float(dt_s))

    if dt == 0.0:
        return pose.normalized()

    yaw = normalize_yaw_rad(pose.yaw_rad)

    world_vx, world_vy = body_velocity_to_world(
        vx_mps=twist.vx_mps,
        vy_mps=twist.vy_mps,
        yaw_rad=yaw,
    )

    return Pose2D(
        x_m=float(pose.x_m) + world_vx * dt,
        y_m=float(pose.y_m) + world_vy * dt,
        yaw_rad=normalize_yaw_rad(yaw + float(twist.omega_rad_s) * dt),
    )


def body_velocity_to_world(
    *,
    vx_mps: float,
    vy_mps: float,
    yaw_rad: float,
) -> tuple[float, float]:
    yaw = float(yaw_rad)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    vx = float(vx_mps)
    vy = float(vy_mps)

    world_vx = vx * cos_yaw - vy * sin_yaw
    world_vy = vx * sin_yaw + vy * cos_yaw

    return world_vx, world_vy


def world_velocity_to_body(
    *,
    world_vx_mps: float,
    world_vy_mps: float,
    yaw_rad: float,
) -> tuple[float, float]:
    yaw = float(yaw_rad)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    vx = float(world_vx_mps)
    vy = float(world_vy_mps)

    body_vx = vx * cos_yaw + vy * sin_yaw
    body_vy = -vx * sin_yaw + vy * cos_yaw

    return body_vx, body_vy


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    half_yaw = normalize_yaw_rad(yaw_rad) * 0.5

    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def quaternion_to_yaw_rad(
    *,
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    x = float(x)
    y = float(y)
    z = float(z)
    w = float(w)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return normalize_yaw_rad(math.atan2(siny_cosp, cosy_cosp))


def distance_2d_m(
    pose_a: Pose2D,
    pose_b: Pose2D,
) -> float:
    dx = float(pose_b.x_m) - float(pose_a.x_m)
    dy = float(pose_b.y_m) - float(pose_a.y_m)

    return math.hypot(dx, dy)


def pose_delta_2d(
    start: Pose2D,
    end: Pose2D,
) -> Pose2D:
    return Pose2D(
        x_m=float(end.x_m) - float(start.x_m),
        y_m=float(end.y_m) - float(start.y_m),
        yaw_rad=normalize_yaw_rad(float(end.yaw_rad) - float(start.yaw_rad)),
    )


def twist_magnitude(
    twist: Twist2D,
) -> tuple[float, float]:
    linear_mps = math.hypot(float(twist.vx_mps), float(twist.vy_mps))
    angular_rad_s = abs(float(twist.omega_rad_s))

    return linear_mps, angular_rad_s


def pose_is_finite(pose: Pose2D) -> bool:
    return (
        math.isfinite(float(pose.x_m))
        and math.isfinite(float(pose.y_m))
        and math.isfinite(float(pose.yaw_rad))
    )


def twist_is_finite(twist: Twist2D) -> bool:
    return (
        math.isfinite(float(twist.vx_mps))
        and math.isfinite(float(twist.vy_mps))
        and math.isfinite(float(twist.omega_rad_s))
    )


def require_finite_pose(pose: Pose2D) -> None:
    if not pose_is_finite(pose):
        raise ValueError(f"Pose contains non-finite values: {pose}")


def require_finite_twist(twist: Twist2D) -> None:
    if not twist_is_finite(twist):
        raise ValueError(f"Twist contains non-finite values: {twist}")


def clamp_twist(
    twist: Twist2D,
    *,
    max_linear_mps: float,
    max_angular_rad_s: float,
) -> Twist2D:
    max_linear = abs(float(max_linear_mps))
    max_angular = abs(float(max_angular_rad_s))

    if max_linear <= 0.0:
        raise ValueError(f"max_linear_mps must be > 0.0, got {max_linear_mps}")

    if max_angular <= 0.0:
        raise ValueError(f"max_angular_rad_s must be > 0.0, got {max_angular_rad_s}")

    linear_mps, angular_rad_s = twist_magnitude(twist)

    scale = 1.0
    if linear_mps > max_linear:
        scale = min(scale, max_linear / linear_mps)

    omega = float(twist.omega_rad_s)
    if angular_rad_s > max_angular:
        omega = math.copysign(max_angular, omega)

    return Twist2D(
        vx_mps=float(twist.vx_mps) * scale,
        vy_mps=float(twist.vy_mps) * scale,
        omega_rad_s=omega,
    )


def zero_pose() -> Pose2D:
    return Pose2D()


def zero_twist() -> Twist2D:
    return Twist2D()