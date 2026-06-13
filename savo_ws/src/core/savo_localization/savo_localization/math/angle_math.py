#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Angle helpers for localization math. No ROS imports."""

from __future__ import annotations

import math


PI = math.pi
TWO_PI = 2.0 * math.pi


def deg_to_rad(angle_deg: float) -> float:
    return math.radians(float(angle_deg))


def rad_to_deg(angle_rad: float) -> float:
    return math.degrees(float(angle_rad))


def normalize_angle_rad(angle_rad: float) -> float:
    angle = math.fmod(float(angle_rad) + PI, TWO_PI)

    if angle < 0.0:
        angle += TWO_PI

    return angle - PI


def normalize_angle_deg(angle_deg: float) -> float:
    angle = math.fmod(float(angle_deg) + 180.0, 360.0)

    if angle < 0.0:
        angle += 360.0

    return angle - 180.0


def normalize_yaw_rad(yaw_rad: float) -> float:
    return normalize_angle_rad(yaw_rad)


def normalize_yaw_deg(yaw_deg: float) -> float:
    return normalize_angle_deg(yaw_deg)


def shortest_angle_delta_rad(from_angle_rad: float, to_angle_rad: float) -> float:
    return normalize_angle_rad(float(to_angle_rad) - float(from_angle_rad))


def shortest_angle_delta_deg(from_angle_deg: float, to_angle_deg: float) -> float:
    return normalize_angle_deg(float(to_angle_deg) - float(from_angle_deg))


def angle_error_rad(target_rad: float, current_rad: float) -> float:
    return shortest_angle_delta_rad(current_rad, target_rad)


def angle_error_deg(target_deg: float, current_deg: float) -> float:
    return shortest_angle_delta_deg(current_deg, target_deg)


def integrate_yaw_rad(
    yaw_rad: float,
    angular_velocity_rad_s: float,
    dt_s: float,
) -> float:
    return normalize_yaw_rad(
        float(yaw_rad) + float(angular_velocity_rad_s) * max(0.0, float(dt_s))
    )


def integrate_yaw_deg(
    yaw_deg: float,
    angular_velocity_deg_s: float,
    dt_s: float,
) -> float:
    return normalize_yaw_deg(
        float(yaw_deg) + float(angular_velocity_deg_s) * max(0.0, float(dt_s))
    )


def heading_to_unit_vector(yaw_rad: float) -> tuple[float, float]:
    yaw = float(yaw_rad)
    return math.cos(yaw), math.sin(yaw)


def unit_vector_to_heading_rad(x: float, y: float) -> float:
    if float(x) == 0.0 and float(y) == 0.0:
        raise ValueError("Cannot compute heading from zero vector.")

    return normalize_yaw_rad(math.atan2(float(y), float(x)))


def is_angle_close_rad(
    angle_a_rad: float,
    angle_b_rad: float,
    *,
    tolerance_rad: float,
) -> bool:
    if tolerance_rad < 0.0:
        raise ValueError(f"tolerance_rad must be >= 0.0, got {tolerance_rad}")

    return abs(shortest_angle_delta_rad(angle_a_rad, angle_b_rad)) <= tolerance_rad


def is_angle_close_deg(
    angle_a_deg: float,
    angle_b_deg: float,
    *,
    tolerance_deg: float,
) -> bool:
    if tolerance_deg < 0.0:
        raise ValueError(f"tolerance_deg must be >= 0.0, got {tolerance_deg}")

    return abs(shortest_angle_delta_deg(angle_a_deg, angle_b_deg)) <= tolerance_deg


def yaw_rate_dps_to_rad_s(yaw_rate_dps: float) -> float:
    return deg_to_rad(yaw_rate_dps)


def yaw_rate_rad_s_to_dps(yaw_rate_rad_s: float) -> float:
    return rad_to_deg(yaw_rate_rad_s)