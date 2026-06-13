# -*- coding: utf-8 -*-
"""ROS message adapters for LiDAR nodes."""

from __future__ import annotations

import math
from collections.abc import Sequence

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def make_string_msg(data: str) -> String:
    msg = String()
    msg.data = str(data)
    return msg


def copy_laser_scan(scan: LaserScan) -> LaserScan:
    copied = LaserScan()

    copied.header = scan.header
    copied.angle_min = scan.angle_min
    copied.angle_max = scan.angle_max
    copied.angle_increment = scan.angle_increment
    copied.time_increment = scan.time_increment
    copied.scan_time = scan.scan_time
    copied.range_min = scan.range_min
    copied.range_max = scan.range_max
    copied.ranges = list(scan.ranges)
    copied.intensities = list(scan.intensities)

    return copied


def valid_range_count(ranges: Sequence[float], range_min: float, range_max: float) -> int:
    return sum(
        1
        for value in ranges
        if _is_valid_range(value, range_min=range_min, range_max=range_max)
    )


def valid_range_ratio(ranges: Sequence[float], range_min: float, range_max: float) -> float:
    if not ranges:
        return 0.0

    return valid_range_count(ranges, range_min, range_max) / float(len(ranges))


def clamp_laser_ranges(scan: LaserScan, range_min: float, range_max: float) -> LaserScan:
    filtered = copy_laser_scan(scan)
    filtered.range_min = float(range_min)
    filtered.range_max = float(range_max)

    filtered.ranges = [
        _range_or_inf(value, range_min=range_min, range_max=range_max)
        for value in filtered.ranges
    ]

    return filtered


def scan_angle_deg(scan: LaserScan, index: int) -> float:
    angle_rad = scan.angle_min + float(index) * scan.angle_increment
    return math.degrees(angle_rad)


def scan_index_for_angle_deg(scan: LaserScan, angle_deg: float) -> int:
    if scan.angle_increment == 0.0:
        raise ValueError("LaserScan angle_increment cannot be zero")

    if not scan.ranges:
        return 0

    angle_rad = math.radians(float(angle_deg))
    index = round((angle_rad - scan.angle_min) / scan.angle_increment)

    return max(0, min(int(index), len(scan.ranges) - 1))


def _range_or_inf(value: object, *, range_min: float, range_max: float) -> float:
    if not _is_valid_range(value, range_min=range_min, range_max=range_max):
        return float("inf")

    return float(value)


def _is_valid_range(value: object, *, range_min: float, range_max: float) -> bool:
    try:
        range_m = float(value)
    except (TypeError, ValueError):
        return False

    return math.isfinite(range_m) and float(range_min) <= range_m <= float(range_max)


__all__ = [
    "clamp_laser_ranges",
    "copy_laser_scan",
    "make_string_msg",
    "scan_angle_deg",
    "scan_index_for_angle_deg",
    "valid_range_count",
    "valid_range_ratio",
]
