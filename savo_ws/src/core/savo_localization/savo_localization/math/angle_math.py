#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Angle helpers for 2D localization and yaw control."""

from __future__ import annotations

import math


PI = math.pi
TWO_PI = 2.0 * math.pi
TAU = TWO_PI


def deg_to_rad(value_deg: float) -> float:
    return float(value_deg) * PI / 180.0


def rad_to_deg(value_rad: float) -> float:
    return float(value_rad) * 180.0 / PI


def normalize_angle_rad(angle_rad: float) -> float:
    return (float(angle_rad) + PI) % TWO_PI - PI


def wrap_angle_rad(angle_rad: float) -> float:
    return normalize_angle_rad(angle_rad)


def normalize_yaw_rad(yaw_rad: float) -> float:
    return normalize_angle_rad(yaw_rad)


def normalize_angle_deg(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def normalize_yaw_deg(yaw_deg: float) -> float:
    return normalize_angle_deg(yaw_deg)


def shortest_angle_delta_rad(current_rad: float, target_rad: float) -> float:
    return normalize_angle_rad(float(target_rad) - float(current_rad))


def shortest_angle_delta_deg(current_deg: float, target_deg: float) -> float:
    return normalize_angle_deg(float(target_deg) - float(current_deg))


def angle_error_rad(current_rad: float, target_rad: float) -> float:
    return shortest_angle_delta_rad(current_rad, target_rad)


def angle_error_deg(current_deg: float, target_deg: float) -> float:
    return shortest_angle_delta_deg(current_deg, target_deg)


def clamp_angle_rad(angle_rad: float, limit_rad: float) -> float:
    if limit_rad < 0.0:
        raise ValueError("limit_rad must be >= 0.0")

    limit = float(limit_rad)
    return max(-limit, min(limit, normalize_angle_rad(angle_rad)))


def is_angle_close_rad(
    current_rad: float,
    target_rad: float,
    tolerance_rad: float,
) -> bool:
    if tolerance_rad < 0.0:
        raise ValueError("tolerance_rad must be >= 0.0")

    return abs(shortest_angle_delta_rad(current_rad, target_rad)) <= tolerance_rad


def angle_close_rad(
    first_rad: float,
    second_rad: float,
    *,
    tolerance_rad: float,
) -> bool:
    return is_angle_close_rad(first_rad, second_rad, tolerance_rad)


def is_angle_close_deg(
    current_deg: float,
    target_deg: float,
    tolerance_deg: float,
) -> bool:
    if tolerance_deg < 0.0:
        raise ValueError("tolerance_deg must be >= 0.0")

    return abs(shortest_angle_delta_deg(current_deg, target_deg)) <= tolerance_deg


def integrate_yaw_rad(
    yaw_rad: float,
    yaw_rate_rad_s: float,
    dt_s: float,
) -> float:
    if dt_s < 0.0:
        raise ValueError("dt_s must be >= 0.0")

    return normalize_yaw_rad(float(yaw_rad) + float(yaw_rate_rad_s) * float(dt_s))


def integrate_yaw_deg(
    yaw_deg: float,
    yaw_rate_dps: float,
    dt_s: float,
) -> float:
    if dt_s < 0.0:
        raise ValueError("dt_s must be >= 0.0")

    return normalize_yaw_deg(float(yaw_deg) + float(yaw_rate_dps) * float(dt_s))


def yaw_rate_dps_to_rad_s(value_dps: float) -> float:
    return deg_to_rad(value_dps)


def yaw_rate_rad_s_to_dps(value_rad_s: float) -> float:
    return rad_to_deg(value_rad_s)


def heading_to_unit_vector(heading_rad: float) -> tuple[float, float]:
    return (
        math.cos(float(heading_rad)),
        math.sin(float(heading_rad)),
    )


def unit_vector_to_heading_rad(x: float, y: float) -> float:
    x_value = float(x)
    y_value = float(y)

    if x_value == 0.0 and y_value == 0.0:
        raise ValueError("unit vector cannot be zero")

    return normalize_angle_rad(math.atan2(y_value, x_value))


__all__ = [
    "PI",
    "TWO_PI",
    "TAU",
    "deg_to_rad",
    "rad_to_deg",
    "normalize_angle_rad",
    "wrap_angle_rad",
    "normalize_yaw_rad",
    "normalize_angle_deg",
    "normalize_yaw_deg",
    "shortest_angle_delta_rad",
    "shortest_angle_delta_deg",
    "angle_error_rad",
    "angle_error_deg",
    "clamp_angle_rad",
    "is_angle_close_rad",
    "angle_close_rad",
    "is_angle_close_deg",
    "integrate_yaw_rad",
    "integrate_yaw_deg",
    "yaw_rate_dps_to_rad_s",
    "yaw_rate_rad_s_to_dps",
    "heading_to_unit_vector",
    "unit_vector_to_heading_rad",
]
