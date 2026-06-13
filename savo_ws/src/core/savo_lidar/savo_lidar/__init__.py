# -*- coding: utf-8 -*-
"""Package root. Exports version, constants, and top-level helpers. No ROS imports."""

from __future__ import annotations

try:
    from .version import (
        __version__,
        VERSION,
        get_version,
        get_package_version_info,
    )
except Exception:
    __version__ = "0.0.0"
    VERSION = __version__

    def get_version() -> str:
        return __version__

    def get_package_version_info() -> dict:
        return {
            "package": "savo_lidar",
            "version": __version__,
            "version_tuple": (0, 0, 0),
        }


try:
    from .constants import (
        DEFAULTS,
        LidarDefaults,
        get_lidar_defaults,
        # Identity
        PACKAGE_NAME,
        ROBOT_NAME,
        DEFAULT_LIDAR_MODEL,
        LIDAR_MODEL_RPLIDAR_A1,
        # Nodes
        NODE_NAME_LIDAR_DRIVER,
        NODE_NAME_LIDAR_PY_DRIVER,
        NODE_NAME_LIDAR_HEALTH,
        NODE_NAME_LIDAR_FILTER,
        NODE_NAME_LIDAR_WATCHDOG,
        NODE_NAME_LIDAR_STATE_PUBLISHER,
        # Frames
        DEFAULT_FRAME_ID,
        DEFAULT_BASE_FRAME_ID,
        # Topics
        TOPIC_SCAN,
        TOPIC_SCAN_FILTERED,
        TOPIC_SAVO_LIDAR_HEALTH,
        TOPIC_SAVO_LIDAR_STATE,
        TOPIC_SAVO_LIDAR_SCAN_QUALITY,
        TOPIC_SAVO_LIDAR_HEARTBEAT,
        TOPIC_SAVO_LIDAR_WATCHDOG_STATE,
        TOPIC_SAVO_LIDAR_SECTOR_SCAN,
        # Hardware
        DEFAULT_SERIAL_PORT,
        DEFAULT_BAUDRATE,
        DEFAULT_SCAN_MODE,
        DEFAULT_BACKEND,
        BACKEND_REAL,
        BACKEND_DRYRUN,
        # Helpers
        is_supported_backend,
        is_supported_lidar_model,
        clamp_range_m,
    )
except Exception:
    DEFAULTS = None
    LidarDefaults = None

    def get_lidar_defaults():
        return None

    PACKAGE_NAME = "savo_lidar"
    ROBOT_NAME = "Robot Savo"

    DEFAULT_LIDAR_MODEL = "rplidar_a1"
    LIDAR_MODEL_RPLIDAR_A1 = "rplidar_a1"

    NODE_NAME_LIDAR_DRIVER = "lidar_driver_node"
    NODE_NAME_LIDAR_PY_DRIVER = "lidar_py_driver_node"
    NODE_NAME_LIDAR_HEALTH = "lidar_health_node"
    NODE_NAME_LIDAR_FILTER = "lidar_filter_node"
    NODE_NAME_LIDAR_WATCHDOG = "lidar_watchdog_node"
    NODE_NAME_LIDAR_STATE_PUBLISHER = "lidar_state_publisher_node"

    DEFAULT_FRAME_ID = "laser"
    DEFAULT_BASE_FRAME_ID = "base_link"

    TOPIC_SCAN = "/scan"
    TOPIC_SCAN_FILTERED = "/scan_filtered"
    TOPIC_SAVO_LIDAR_HEALTH = "/savo_lidar/health"
    TOPIC_SAVO_LIDAR_STATE = "/savo_lidar/state"
    TOPIC_SAVO_LIDAR_SCAN_QUALITY = "/savo_lidar/scan_quality"
    TOPIC_SAVO_LIDAR_HEARTBEAT = "/savo_lidar/heartbeat"
    TOPIC_SAVO_LIDAR_WATCHDOG_STATE = "/savo_lidar/watchdog_state"
    TOPIC_SAVO_LIDAR_SECTOR_SCAN = "/savo_lidar/sector_scan"

    DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
    DEFAULT_BAUDRATE = 115200
    DEFAULT_SCAN_MODE = "standard"
    DEFAULT_BACKEND = "real"
    BACKEND_REAL = "real"
    BACKEND_DRYRUN = "dryrun"

    def is_supported_backend(backend: str) -> bool:
        return backend in (BACKEND_REAL, BACKEND_DRYRUN)

    def is_supported_lidar_model(model: str) -> bool:
        return model == LIDAR_MODEL_RPLIDAR_A1

    def clamp_range_m(value: float) -> float:
        v = float(value)

        if v < 0.15:
            return 0.15

        if v > 12.0:
            return 12.0

        return v


def get_package_info() -> dict:
    """Lightweight package metadata — safe to call without ROS."""
    return {
        "package": PACKAGE_NAME,
        "robot_name": ROBOT_NAME,
        "version": get_version(),
        "lidar_model": DEFAULT_LIDAR_MODEL,
        "node_defaults": {
            "driver": NODE_NAME_LIDAR_DRIVER,
            "python_driver": NODE_NAME_LIDAR_PY_DRIVER,
            "health": NODE_NAME_LIDAR_HEALTH,
            "filter": NODE_NAME_LIDAR_FILTER,
            "watchdog": NODE_NAME_LIDAR_WATCHDOG,
            "state_publisher": NODE_NAME_LIDAR_STATE_PUBLISHER,
        },
        "frames": {
            "lidar": DEFAULT_FRAME_ID,
            "base": DEFAULT_BASE_FRAME_ID,
        },
        "topics": {
            "scan": TOPIC_SCAN,
            "scan_filtered": TOPIC_SCAN_FILTERED,
            "health": TOPIC_SAVO_LIDAR_HEALTH,
            "state": TOPIC_SAVO_LIDAR_STATE,
            "scan_quality": TOPIC_SAVO_LIDAR_SCAN_QUALITY,
            "heartbeat": TOPIC_SAVO_LIDAR_HEARTBEAT,
            "watchdog_state": TOPIC_SAVO_LIDAR_WATCHDOG_STATE,
            "sector_scan": TOPIC_SAVO_LIDAR_SECTOR_SCAN,
        },
        "hardware_defaults": {
            "serial_port": DEFAULT_SERIAL_PORT,
            "baudrate": DEFAULT_BAUDRATE,
            "scan_mode": DEFAULT_SCAN_MODE,
            "backend": DEFAULT_BACKEND,
        },
        "has_structured_defaults": DEFAULTS is not None,
    }


__all__ = [
    # version
    "__version__",
    "VERSION",
    "get_version",
    "get_package_version_info",
    # constants/defaults
    "DEFAULTS",
    "LidarDefaults",
    "get_lidar_defaults",
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "DEFAULT_LIDAR_MODEL",
    "LIDAR_MODEL_RPLIDAR_A1",
    "NODE_NAME_LIDAR_DRIVER",
    "NODE_NAME_LIDAR_PY_DRIVER",
    "NODE_NAME_LIDAR_HEALTH",
    "NODE_NAME_LIDAR_FILTER",
    "NODE_NAME_LIDAR_WATCHDOG",
    "NODE_NAME_LIDAR_STATE_PUBLISHER",
    "DEFAULT_FRAME_ID",
    "DEFAULT_BASE_FRAME_ID",
    "TOPIC_SCAN",
    "TOPIC_SCAN_FILTERED",
    "TOPIC_SAVO_LIDAR_HEALTH",
    "TOPIC_SAVO_LIDAR_STATE",
    "TOPIC_SAVO_LIDAR_SCAN_QUALITY",
    "TOPIC_SAVO_LIDAR_HEARTBEAT",
    "TOPIC_SAVO_LIDAR_WATCHDOG_STATE",
    "TOPIC_SAVO_LIDAR_SECTOR_SCAN",
    "DEFAULT_SERIAL_PORT",
    "DEFAULT_BAUDRATE",
    "DEFAULT_SCAN_MODE",
    "DEFAULT_BACKEND",
    "BACKEND_REAL",
    "BACKEND_DRYRUN",
    "is_supported_backend",
    "is_supported_lidar_model",
    "clamp_range_m",
    # metadata helper
    "get_package_info",
]