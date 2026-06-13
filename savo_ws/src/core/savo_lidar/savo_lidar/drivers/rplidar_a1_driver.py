# -*- coding: utf-8 -*-
"""RPLIDAR A1 Python driver wrapper."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from savo_lidar.constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_SERIAL_PORT,
)
from savo_lidar.drivers.lidar_exceptions import (
    LidarConnectionError,
    LidarNotReadyError,
    LidarReadError,
)
from savo_lidar.drivers.serial_port import require_serial_port


@dataclass
class RplidarScan:
    stamp_s: float
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: list[float]
    intensities: list[float] = field(default_factory=list)


class RplidarA1Driver:
    def __init__(
        self,
        *,
        serial_port: str = DEFAULT_SERIAL_PORT,
        baudrate: int = DEFAULT_BAUDRATE,
        min_range_m: float = DEFAULT_MIN_RANGE_M,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
        angle_compensate: bool = True,
        allow_port_fallback: bool = True,
    ) -> None:
        self.serial_port = str(serial_port)
        self.baudrate = int(baudrate)
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.angle_compensate = bool(angle_compensate)
        self.allow_port_fallback = bool(allow_port_fallback)

        self._lidar: Any | None = None
        self._scan_iter: Any | None = None
        self._running = False
        self._scan_count = 0

        self._validate_config()

    @property
    def running(self) -> bool:
        return self._running

    @property
    def scan_count(self) -> int:
        return self._scan_count

    def start(self) -> None:
        port = require_serial_port(
            self.serial_port,
            allow_fallback=self.allow_port_fallback,
        )

        try:
            from rplidar import RPLidar
        except ImportError as exc:
            raise LidarConnectionError(
                "Python package 'rplidar' is not installed. Install it on the Pi for backend=real."
            ) from exc

        try:
            self._lidar = RPLidar(port, baudrate=self.baudrate)

            if hasattr(self._lidar, "start_motor"):
                self._lidar.start_motor()

            self._scan_iter = self._lidar.iter_scans()
            self.serial_port = port
            self._running = True
        except Exception as exc:
            self._running = False
            self._lidar = None
            self._scan_iter = None
            raise LidarConnectionError(
                f"Failed to start RPLIDAR A1 on {port} at {self.baudrate} baud: {exc}"
            ) from exc

    def stop(self) -> None:
        self._running = False

        if self._lidar is None:
            return

        try:
            self._lidar.stop()
        except Exception:
            pass

        try:
            self._lidar.stop_motor()
        except Exception:
            pass

        try:
            self._lidar.disconnect()
        except Exception:
            pass

        self._lidar = None
        self._scan_iter = None

    def read_scan(self) -> RplidarScan:
        if not self._running or self._scan_iter is None:
            raise LidarNotReadyError("RPLIDAR driver is not running")

        try:
            raw_scan = next(self._scan_iter)
        except StopIteration as exc:
            raise LidarReadError("RPLIDAR scan iterator stopped") from exc
        except Exception as exc:
            raise LidarReadError(f"Failed to read RPLIDAR scan: {exc}") from exc

        self._scan_count += 1
        return self._convert_raw_scan(raw_scan)

    def get_info(self) -> dict[str, Any]:
        if self._lidar is None:
            return {}

        try:
            return dict(self._lidar.get_info())
        except Exception:
            return {}

    def get_health(self) -> dict[str, Any]:
        if self._lidar is None:
            return {}

        try:
            return dict(self._lidar.get_health())
        except Exception:
            return {}

    def _validate_config(self) -> None:
        if self.baudrate <= 0:
            raise ValueError(f"baudrate must be > 0, got {self.baudrate}")

        if self.min_range_m <= 0.0:
            raise ValueError(f"min_range_m must be > 0.0, got {self.min_range_m}")

        if self.max_range_m <= self.min_range_m:
            raise ValueError(
                f"max_range_m must be greater than min_range_m, got {self.max_range_m}"
            )

    def _convert_raw_scan(self, raw_scan: list[tuple[float, float, float]]) -> RplidarScan:
        if self.angle_compensate:
            ranges, intensities = _angle_compensated_scan(
                raw_scan,
                min_range_m=self.min_range_m,
                max_range_m=self.max_range_m,
            )
        else:
            ranges, intensities = _ordered_scan_points(
                raw_scan,
                min_range_m=self.min_range_m,
                max_range_m=self.max_range_m,
            )

        point_count = len(ranges)
        if point_count <= 0:
            raise LidarReadError("RPLIDAR returned an empty scan")

        angle_min_rad = -math.pi
        angle_max_rad = math.pi
        angle_increment_rad = (angle_max_rad - angle_min_rad) / float(point_count)

        return RplidarScan(
            stamp_s=time.monotonic(),
            angle_min_rad=angle_min_rad,
            angle_max_rad=angle_max_rad,
            angle_increment_rad=angle_increment_rad,
            range_min_m=self.min_range_m,
            range_max_m=self.max_range_m,
            ranges=ranges,
            intensities=intensities,
        )


def _angle_compensated_scan(
    raw_scan: list[tuple[float, float, float]],
    *,
    min_range_m: float,
    max_range_m: float,
    point_count: int = 360,
) -> tuple[list[float], list[float]]:
    ranges = [float("inf")] * point_count
    intensities = [0.0] * point_count

    for quality, angle_deg, distance_mm in raw_scan:
        index = int(round(float(angle_deg))) % point_count
        distance_m = float(distance_mm) / 1000.0

        if not _valid_distance(distance_m, min_range_m, max_range_m):
            continue

        current = ranges[index]
        if not math.isfinite(current) or distance_m < current:
            ranges[index] = distance_m
            intensities[index] = float(quality)

    return ranges, intensities


def _ordered_scan_points(
    raw_scan: list[tuple[float, float, float]],
    *,
    min_range_m: float,
    max_range_m: float,
) -> tuple[list[float], list[float]]:
    ordered = sorted(raw_scan, key=lambda item: float(item[1]))

    ranges: list[float] = []
    intensities: list[float] = []

    for quality, _angle_deg, distance_mm in ordered:
        distance_m = float(distance_mm) / 1000.0

        if _valid_distance(distance_m, min_range_m, max_range_m):
            ranges.append(distance_m)
            intensities.append(float(quality))
        else:
            ranges.append(float("inf"))
            intensities.append(0.0)

    return ranges, intensities


def _valid_distance(distance_m: float, min_range_m: float, max_range_m: float) -> bool:
    return math.isfinite(distance_m) and min_range_m <= distance_m <= max_range_m


__all__ = [
    "RplidarA1Driver",
    "RplidarScan",
]
