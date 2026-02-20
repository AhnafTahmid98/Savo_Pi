#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_localization/utils/odom_math.py
------------------------------------------------
Professional odometry math utilities.

This module is intentionally ROS-agnostic (no rclpy imports) so it can be used by:
- C++ wheel_odom_node (as reference / parity tests)
- Python wheel_odom_fallback_node
- Unit tests

Current assumption (locked for localization):
- Wheel odometry is computed from rear encoders as a differential-drive model:
    v = (v_r + v_l)/2
    w = (v_r - v_l)/track_width
- Pose is integrated in the odom frame.
- EKF (robot_localization) is responsible for publishing TF (odom->base_link).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


# ---------------------------
# Basic helpers
# ---------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x to [lo, hi]."""
    return lo if x < lo else hi if x > hi else x


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    # Keep +pi instead of -pi (optional, but helpful for continuity)
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


def safe_div(num: float, den: float, default: float = 0.0) -> float:
    """Safe divide with fallback default when den is 0."""
    return default if abs(den) < 1e-12 else (num / den)


# ---------------------------
# Core data structures
# ---------------------------

@dataclass
class Pose2D:
    """2D pose in odom frame."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # rad


@dataclass
class Twist2D:
    """2D body twist (base_link frame convention)."""
    v: float = 0.0    # m/s (forward)
    w: float = 0.0    # rad/s (yaw rate)


# ---------------------------
# Encoder + wheel conversions
# ---------------------------

def ticks_to_distance_m(
    ticks: float,
    wheel_diameter_m: float,
    ticks_per_rev: float,
    gear_ratio: float = 1.0,
    decoding: float = 1.0,
) -> float:
    """
    Convert encoder ticks to traveled distance in meters.

    Args:
        ticks: encoder tick delta (can be negative)
        wheel_diameter_m: wheel diameter (m)
        ticks_per_rev: encoder counts per motor revolution (CPR)
        gear_ratio: wheel revs per motor rev (or motor->wheel ratio).
                    If your CPR is at the wheel already, keep gear_ratio=1.0.
        decoding: e.g. 1, 2, 4 depending on quadrature decoding multiplier.

    Returns:
        Distance (m).
    """
    circumference = math.pi * wheel_diameter_m
    effective_ticks_per_wheel_rev = ticks_per_rev * decoding * safe_div(1.0, gear_ratio, default=1.0)
    # If gear_ratio is motor_rev_per_wheel_rev, you might instead multiply.
    # Keep your system consistent; for Robot Savo we treat gear_ratio as:
    #   ticks_per_wheel_rev = CPR * decoding * gear_ratio
    # If that's your case, replace the line above with:
    #   effective_ticks_per_wheel_rev = ticks_per_rev * decoding * gear_ratio
    revs = safe_div(ticks, effective_ticks_per_wheel_rev, default=0.0)
    return revs * circumference


def distance_to_ticks(
    dist_m: float,
    wheel_diameter_m: float,
    ticks_per_rev: float,
    gear_ratio: float = 1.0,
    decoding: float = 1.0,
) -> float:
    """Inverse of ticks_to_distance_m (approx)."""
    circumference = math.pi * wheel_diameter_m
    revs = safe_div(dist_m, circumference, default=0.0)
    effective_ticks_per_wheel_rev = ticks_per_rev * decoding * safe_div(1.0, gear_ratio, default=1.0)
    return revs * effective_ticks_per_wheel_rev


# ---------------------------
# Differential-drive kinematics
# ---------------------------

def diff_drive_twist_from_wheel_vel(
    v_l: float,
    v_r: float,
    track_width_m: float,
) -> Twist2D:
    """
    Convert left/right wheel linear velocities (m/s) to base twist.

    v = (v_r + v_l)/2
    w = (v_r - v_l)/track_width
    """
    if track_width_m <= 0.0:
        raise ValueError("track_width_m must be > 0")
    v = 0.5 * (v_r + v_l)
    w = (v_r - v_l) / track_width_m
    return Twist2D(v=v, w=w)


def diff_drive_wheel_vel_from_twist(
    v: float,
    w: float,
    track_width_m: float,
) -> Tuple[float, float]:
    """
    Convert base twist to left/right wheel linear velocities (m/s).

    v_l = v - w*track/2
    v_r = v + w*track/2
    """
    if track_width_m <= 0.0:
        raise ValueError("track_width_m must be > 0")
    half = 0.5 * track_width_m
    v_l = v - w * half
    v_r = v + w * half
    return v_l, v_r


# ---------------------------
# Pose integration
# ---------------------------

def integrate_pose_2d(
    pose: Pose2D,
    twist: Twist2D,
    dt: float,
) -> Pose2D:
    """
    Integrate pose forward by dt using a stable midpoint method.

    Assumes:
      - twist.v is forward velocity in base_link
      - twist.w is yaw rate (rad/s)
      - pose is in odom frame

    Uses:
      yaw_mid = yaw + 0.5*w*dt
      x += v*dt*cos(yaw_mid)
      y += v*dt*sin(yaw_mid)
      yaw += w*dt
    """
    if dt <= 0.0:
        return Pose2D(pose.x, pose.y, pose.yaw)

    wdt = twist.w * dt
    yaw_mid = pose.yaw + 0.5 * wdt

    dx = twist.v * dt * math.cos(yaw_mid)
    dy = twist.v * dt * math.sin(yaw_mid)

    return Pose2D(
        x=pose.x + dx,
        y=pose.y + dy,
        yaw=wrap_to_pi(pose.yaw + wdt),
    )


# ---------------------------
# Quaternion helpers (2D yaw only)
# ---------------------------

def yaw_to_quat_xyzw(yaw_rad: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw (rad) to quaternion (x,y,z,w) assuming roll=pitch=0.
    """
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_xyzw_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Convert quaternion (x,y,z,w) to yaw (rad).
    Assumes quaternion is valid; works generally even if roll/pitch exist,
    but intended for planar motion.
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return wrap_to_pi(math.atan2(siny_cosp, cosy_cosp))


# ---------------------------
# Covariance helpers (simple, conservative defaults)
# ---------------------------

def make_odom_covariance_6x6(
    pos_xy_var: float = 1e-3,
    yaw_var: float = 1e-2,
    lin_vel_var: float = 1e-2,
    yaw_rate_var: float = 1e-1,
) -> Tuple[float, ...]:
    """
    Create a 6x6 covariance tuple (row-major) for nav_msgs/Odometry.

    Layout corresponds to [x, y, z, roll, pitch, yaw].
    For a planar robot, we set z/roll/pitch with large variance.

    Returns:
        36 floats, row-major.
    """
    big = 1e3  # large uncertainty for unused DOF
    cov = [0.0] * 36

    def set_diag(i: int, var: float) -> None:
        cov[i * 6 + i] = max(var, 0.0)

    # pose covariance
    set_diag(0, pos_xy_var)  # x
    set_diag(1, pos_xy_var)  # y
    set_diag(2, big)         # z
    set_diag(3, big)         # roll
    set_diag(4, big)         # pitch
    set_diag(5, yaw_var)     # yaw

    return tuple(cov)


def make_twist_covariance_6x6(
    lin_vel_var: float = 1e-2,
    yaw_rate_var: float = 1e-1,
) -> Tuple[float, ...]:
    """
    Create a 6x6 covariance tuple (row-major) for geometry_msgs/TwistWithCovariance.

    Layout corresponds to [vx, vy, vz, wx, wy, wz].
    Planar diff-drive: vx and wz are used; others get large variance.
    """
    big = 1e3
    cov = [0.0] * 36

    def set_diag(i: int, var: float) -> None:
        cov[i * 6 + i] = max(var, 0.0)

    set_diag(0, lin_vel_var)  # vx
    set_diag(1, big)          # vy
    set_diag(2, big)          # vz
    set_diag(3, big)          # wx
    set_diag(4, big)          # wy
    set_diag(5, yaw_rate_var) # wz

    return tuple(cov)


# ---------------------------
# Sanity checks
# ---------------------------

def is_finite_pose(p: Pose2D) -> bool:
    return all(math.isfinite(v) for v in (p.x, p.y, p.yaw))


def is_finite_twist(t: Twist2D) -> bool:
    return all(math.isfinite(v) for v in (t.v, t.w))