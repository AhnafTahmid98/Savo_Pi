# -*- coding: utf-8 -*-
"""Angle helpers for LaserScan sector selection."""

from __future__ import annotations

import math


def normalize_angle_deg(angle_deg: float) -> float:
    angle = float(angle_deg)

    while angle > 180.0:
        angle -= 360.0

    while angle <= -180.0:
        angle += 360.0

    return angle


def angle_in_sector(
    angle_deg: float,
    sector_min_deg: float,
    sector_max_deg: float,
) -> bool:
    angle = normalize_angle_deg(angle_deg)
    sector_min = normalize_angle_deg(sector_min_deg)
    sector_max = normalize_angle_deg(sector_max_deg)

    if sector_min <= sector_max:
        return sector_min <= angle <= sector_max

    # rear sectors usually wrap across ±180°
    return angle >= sector_min or angle <= sector_max


def radians_to_degrees(angle_rad: float) -> float:
    return math.degrees(float(angle_rad))


def degrees_to_radians(angle_deg: float) -> float:
    return math.radians(float(angle_deg))


def scan_angle_deg(
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    index: int,
) -> float:
    angle_rad = float(angle_min_rad) + float(index) * float(angle_increment_rad)
    return normalize_angle_deg(radians_to_degrees(angle_rad))


def sector_indices(
    *,
    angle_min_rad: float,
    angle_increment_rad: float,
    point_count: int,
    sector_min_deg: float,
    sector_max_deg: float,
) -> list[int]:
    if point_count <= 0:
        return []

    if angle_increment_rad == 0.0:
        raise ValueError("angle_increment_rad cannot be zero")

    indices: list[int] = []

    for index in range(int(point_count)):
        angle_deg = scan_angle_deg(
            angle_min_rad=angle_min_rad,
            angle_increment_rad=angle_increment_rad,
            index=index,
        )

        if angle_in_sector(angle_deg, sector_min_deg, sector_max_deg):
            indices.append(index)

    return indices


__all__ = [
    "angle_in_sector",
    "degrees_to_radians",
    "normalize_angle_deg",
    "radians_to_degrees",
    "scan_angle_deg",
    "sector_indices",
]
