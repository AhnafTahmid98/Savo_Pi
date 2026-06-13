# -*- coding: utf-8 -*-
"""Synthetic LiDAR driver for PC tests and dryrun bringup."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from savo_lidar.constants import (
    DEFAULT_FRAME_ID,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
)


@dataclass
class DryrunScan:
    stamp_s: float
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: list[float]
    intensities: list[float] = field(default_factory=list)


class DryrunLidarDriver:
    def __init__(
        self,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
        min_range_m: float = DEFAULT_MIN_RANGE_M,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
        point_count: int = 360,
        base_distance_m: float = 2.0,
        obstacle_distance_m: float = 0.7,
        obstacle_center_deg: float = 0.0,
        obstacle_width_deg: float = 30.0,
    ) -> None:
        if point_count < 4:
            raise ValueError(f"point_count must be >= 4, got {point_count}")

        if min_range_m <= 0.0:
            raise ValueError(f"min_range_m must be > 0.0, got {min_range_m}")

        if max_range_m <= min_range_m:
            raise ValueError(
                f"max_range_m must be greater than min_range_m, got {max_range_m}"
            )

        self.frame_id = str(frame_id)
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.point_count = int(point_count)
        self.base_distance_m = float(base_distance_m)
        self.obstacle_distance_m = float(obstacle_distance_m)
        self.obstacle_center_deg = float(obstacle_center_deg)
        self.obstacle_width_deg = float(obstacle_width_deg)

        self._running = False
        self._scan_count = 0
        self._angle_min_rad = -math.pi
        self._angle_max_rad = math.pi
        self._angle_increment_rad = (
            self._angle_max_rad - self._angle_min_rad
        ) / float(self.point_count)

    @property
    def running(self) -> bool:
        return self._running

    @property
    def scan_count(self) -> int:
        return self._scan_count

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    def read_scan(self) -> DryrunScan:
        if not self._running:
            raise RuntimeError("Dryrun LiDAR driver is not running.")

        self._scan_count += 1

        ranges: list[float] = []
        intensities: list[float] = []

        for index in range(self.point_count):
            angle_rad = self._angle_min_rad + index * self._angle_increment_rad
            angle_deg = math.degrees(angle_rad)

            distance_m = self._distance_for_angle(angle_deg)
            ranges.append(distance_m)
            intensities.append(1.0)

        return DryrunScan(
            stamp_s=time.monotonic(),
            angle_min_rad=self._angle_min_rad,
            angle_max_rad=self._angle_max_rad,
            angle_increment_rad=self._angle_increment_rad,
            range_min_m=self.min_range_m,
            range_max_m=self.max_range_m,
            ranges=ranges,
            intensities=intensities,
        )

    def _distance_for_angle(self, angle_deg: float) -> float:
        delta_deg = _shortest_angle_delta_deg(angle_deg, self.obstacle_center_deg)
        half_width_deg = self.obstacle_width_deg / 2.0

        if abs(delta_deg) <= half_width_deg:
            return _clamp_range(
                self.obstacle_distance_m,
                self.min_range_m,
                self.max_range_m,
            )

        # small wave makes the dryrun scan easier to recognize in RViz
        wave_m = 0.15 * math.sin(math.radians(angle_deg * 3.0))
        return _clamp_range(
            self.base_distance_m + wave_m,
            self.min_range_m,
            self.max_range_m,
        )


def _shortest_angle_delta_deg(angle_a: float, angle_b: float) -> float:
    return (float(angle_a) - float(angle_b) + 180.0) % 360.0 - 180.0


def _clamp_range(value: float, minimum: float, maximum: float) -> float:
    return max(float(minimum), min(float(maximum), float(value)))


__all__ = [
    "DryrunLidarDriver",
    "DryrunScan",
]
