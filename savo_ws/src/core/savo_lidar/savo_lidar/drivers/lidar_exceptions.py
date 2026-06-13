# -*- coding: utf-8 -*-
"""Driver exceptions for LiDAR hardware access."""

from __future__ import annotations


class LidarDriverError(RuntimeError):
    """Base error for LiDAR driver failures."""


class LidarConnectionError(LidarDriverError):
    """LiDAR serial connection failed."""


class LidarReadError(LidarDriverError):
    """LiDAR scan read failed."""


class LidarConfigurationError(LidarDriverError):
    """LiDAR driver configuration is invalid."""


class LidarTimeoutError(LidarDriverError):
    """LiDAR response timed out."""


class LidarNotReadyError(LidarDriverError):
    """LiDAR driver is not ready to provide scans."""


__all__ = [
    "LidarConfigurationError",
    "LidarConnectionError",
    "LidarDriverError",
    "LidarNotReadyError",
    "LidarReadError",
    "LidarTimeoutError",
]
