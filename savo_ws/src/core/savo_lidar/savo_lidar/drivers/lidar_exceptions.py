"""Driver exceptions for Robot Savo LiDAR hardware access."""

from __future__ import annotations


class LidarDriverError(RuntimeError):
    """Base error for LiDAR driver failures."""


class LidarConnectionError(LidarDriverError):
    """Raised when the LiDAR serial connection cannot be opened."""


class LidarReadError(LidarDriverError):
    """Raised when scan data cannot be read reliably."""


class LidarConfigurationError(LidarDriverError):
    """Raised when driver configuration is invalid."""


class LidarTimeoutError(LidarDriverError):
    """Raised when the LiDAR does not respond within the expected time."""


class LidarNotReadyError(LidarDriverError):
    """Raised when a scan is requested before the driver is ready."""