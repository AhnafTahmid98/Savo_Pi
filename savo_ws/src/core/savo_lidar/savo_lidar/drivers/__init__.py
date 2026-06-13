# -*- coding: utf-8 -*-
"""LiDAR driver implementations and serial helpers."""

from __future__ import annotations

from .driver_factory import LidarDriver, create_lidar_driver
from .dryrun_lidar_driver import DryrunLidarDriver, DryrunScan
from .lidar_exceptions import (
    LidarConfigurationError,
    LidarConnectionError,
    LidarDriverError,
    LidarNotReadyError,
    LidarReadError,
    LidarTimeoutError,
)
from .rplidar_a1_driver import RplidarA1Driver, RplidarScan
from .rplidar_process import RplidarProcess, RplidarProcessState
from .serial_port import (
    SerialPortInfo,
    available_serial_ports,
    choose_serial_port,
    inspect_serial_port,
    require_serial_port,
)

__all__ = [
    "DryrunLidarDriver",
    "DryrunScan",
    "LidarConfigurationError",
    "LidarConnectionError",
    "LidarDriver",
    "LidarDriverError",
    "LidarNotReadyError",
    "LidarReadError",
    "LidarTimeoutError",
    "RplidarA1Driver",
    "RplidarProcess",
    "RplidarProcessState",
    "RplidarScan",
    "SerialPortInfo",
    "available_serial_ports",
    "choose_serial_port",
    "create_lidar_driver",
    "inspect_serial_port",
    "require_serial_port",
]
