# -*- coding: utf-8 -*-
"""Driver factory for real and dryrun LiDAR backends."""

from __future__ import annotations

from typing import Protocol

from savo_lidar.constants import BACKEND_DRYRUN, BACKEND_REAL
from savo_lidar.drivers.dryrun_lidar_driver import DryrunLidarDriver
from savo_lidar.drivers.lidar_exceptions import LidarConfigurationError
from savo_lidar.drivers.rplidar_a1_driver import RplidarA1Driver
from savo_lidar.models.lidar_config import LidarDriverConfig


class LidarDriver(Protocol):
    @property
    def running(self) -> bool: ...

    @property
    def scan_count(self) -> int: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def read_scan(self) -> object: ...


def create_lidar_driver(config: LidarDriverConfig) -> LidarDriver:
    config.validate()

    if config.backend == BACKEND_DRYRUN:
        return DryrunLidarDriver(
            frame_id=config.frame_id,
            min_range_m=config.min_range_m,
            max_range_m=config.max_range_m,
        )

    if config.backend == BACKEND_REAL:
        return RplidarA1Driver(
            serial_port=config.serial_port,
            baudrate=config.baudrate,
            min_range_m=config.min_range_m,
            max_range_m=config.max_range_m,
            angle_compensate=config.angle_compensate,
        )

    raise LidarConfigurationError(f"Unsupported LiDAR backend: {config.backend}")


__all__ = [
    "LidarDriver",
    "create_lidar_driver",
]
