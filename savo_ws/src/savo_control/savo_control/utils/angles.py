#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / utils / angles.py
=============================================

Small ROS-independent angle helpers for Robot Savo control nodes.

Purpose
-------
Shared angle/yaw utilities for:

    - heading PID
    - rotate-to-heading tests
    - straight-line heading hold
    - diagnostics/dashboard formatting

Architecture
------------
This file is pure Python utility code.

It does NOT:
    - create ROS nodes
    - publish topics
    - subscribe to topics
    - command hardware
    - touch /cmd_vel_safe

All functions are deterministic and safe to unit test.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


TAU = 2.0 * math.pi


def is_finite_number(value: float) -> bool:
    """Return True if value is a finite float."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def normalize_angle_rad(angle: float) -> float:
    """
    Normalize angle to [-pi, pi).

    Example:
        normalize_angle_rad(3.5)  -> about -2.78
        normalize_angle_rad(-3.5) -> about  2.78
    """
    if not is_finite_number(angle):
        return 0.0

    wrapped = (float(angle) + math.pi) % TAU - math.pi

    # Keep +pi instead of returning -pi for nicer boundary behavior.
    if wrapped == -math.pi:
        return math.pi

    return wrapped


def normalize_angle_0_2pi_rad(angle: float) -> float:
    """Normalize angle to [0, 2*pi)."""
    if not is_finite_number(angle):
        return 0.0
    return float(angle) % TAU


def shortest_angular_distance_rad(from_angle: float, to_angle: float) -> float:
    """
    Return shortest signed angle from from_angle to to_angle.

    Positive result means rotate counter-clockwise by ROS yaw convention.
    Negative result means rotate clockwise.
    """
    return normalize_angle_rad(float(to_angle) - float(from_angle))


def angle_reached_rad(
    current_rad: float,
    target_rad: float,
    tolerance_rad: float,
) -> bool:
    """Return True if current angle is within tolerance of target angle."""
    if tolerance_rad < 0.0:
        tolerance_rad = abs(tolerance_rad)

    error = shortest_angular_distance_rad(current_rad, target_rad)
    return abs(error) <= tolerance_rad


def clamp_angle_error_rad(error_rad: float, max_abs_error_rad: float) -> float:
    """Normalize and clamp an angular error."""
    error = normalize_angle_rad(error_rad)
    limit = abs(float(max_abs_error_rad))

    if limit <= 0.0:
        return 0.0

    return max(-limit, min(limit, error))


def rad_to_deg(angle_rad: float) -> float:
    """Convert radians to degrees."""
    if not is_finite_number(angle_rad):
        return 0.0
    return math.degrees(float(angle_rad))


def deg_to_rad(angle_deg: float) -> float:
    """Convert degrees to radians."""
    if not is_finite_number(angle_deg):
        return 0.0
    return math.radians(float(angle_deg))


def yaw_from_quaternion_xyzw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[float]:
    """
    Extract yaw from quaternion x, y, z, w.

    Returns:
        yaw in radians, or None if quaternion is invalid.
    """
    try:
        x = float(x)
        y = float(y)
        z = float(z)
        w = float(w)
    except (TypeError, ValueError):
        return None

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9 or not math.isfinite(norm):
        return None

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_ros_quaternion(q) -> Optional[float]:
    """
    Extract yaw from a ROS geometry_msgs Quaternion-like object.

    Expected fields:
        q.x, q.y, q.z, q.w
    """
    if q is None:
        return None

    return yaw_from_quaternion_xyzw(q.x, q.y, q.z, q.w)


def quaternion_xyzw_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    """
    Create quaternion x, y, z, w from yaw only.

    Roll = 0, pitch = 0.
    """
    yaw = normalize_angle_rad(yaw_rad)
    half = 0.5 * yaw

    x = 0.0
    y = 0.0
    z = math.sin(half)
    w = math.cos(half)

    return x, y, z, w


def angular_direction_text(error_rad: float) -> str:
    """
    Human-readable direction for yaw error.

    Returns:
        "CCW", "CW", or "ZERO"
    """
    error = normalize_angle_rad(error_rad)

    if abs(error) < 1.0e-9:
        return "ZERO"

    return "CCW" if error > 0.0 else "CW"


def format_angle_rad(angle_rad: float) -> str:
    """Format angle as rad + deg string."""
    angle = normalize_angle_rad(angle_rad)
    return f"{angle:+.3f} rad ({rad_to_deg(angle):+.1f} deg)"