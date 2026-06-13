# -*- coding: utf-8 -*-

"""Package-wide constants. No ROS imports — safe to import anywhere."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


# =============================================================================
# Package / Identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_lidar"
ROBOT_NAME: Final[str] = "Robot Savo"

DEFAULT_LIDAR_MODEL: Final[str] = "rplidar_a1"
LIDAR_MODEL_RPLIDAR_A1: Final[str] = "rplidar_a1"


# =============================================================================
# Node Names
# =============================================================================
NODE_NAME_LIDAR_DRIVER: Final[str] = "lidar_driver_node"
NODE_NAME_LIDAR_PY_DRIVER: Final[str] = "lidar_py_driver_node"
NODE_NAME_LIDAR_HEALTH: Final[str] = "lidar_health_node"
NODE_NAME_LIDAR_FILTER: Final[str] = "lidar_filter_node"
NODE_NAME_LIDAR_WATCHDOG: Final[str] = "lidar_watchdog_node"
NODE_NAME_LIDAR_STATE_PUBLISHER: Final[str] = "lidar_state_publisher_node"


# =============================================================================
# Frame Names
# =============================================================================
DEFAULT_FRAME_ID: Final[str] = "laser"
DEFAULT_BASE_FRAME_ID: Final[str] = "base_link"


# =============================================================================
# Topic Names
# =============================================================================
DEFAULT_SCAN_TOPIC: Final[str] = "/scan"
DEFAULT_FILTERED_SCAN_TOPIC: Final[str] = "/scan_filtered"

DEFAULT_HEALTH_TOPIC: Final[str] = "/savo_lidar/health"
DEFAULT_STATE_TOPIC: Final[str] = "/savo_lidar/state"
DEFAULT_SCAN_QUALITY_TOPIC: Final[str] = "/savo_lidar/scan_quality"
DEFAULT_HEARTBEAT_TOPIC: Final[str] = "/savo_lidar/heartbeat"
DEFAULT_WATCHDOG_STATE_TOPIC: Final[str] = "/savo_lidar/watchdog_state"
DEFAULT_SECTOR_TOPIC: Final[str] = "/savo_lidar/sector_scan"

TOPIC_SCAN: Final[str] = DEFAULT_SCAN_TOPIC
TOPIC_SCAN_FILTERED: Final[str] = DEFAULT_FILTERED_SCAN_TOPIC
TOPIC_SAVO_LIDAR_HEALTH: Final[str] = DEFAULT_HEALTH_TOPIC
TOPIC_SAVO_LIDAR_STATE: Final[str] = DEFAULT_STATE_TOPIC
TOPIC_SAVO_LIDAR_SCAN_QUALITY: Final[str] = DEFAULT_SCAN_QUALITY_TOPIC
TOPIC_SAVO_LIDAR_HEARTBEAT: Final[str] = DEFAULT_HEARTBEAT_TOPIC
TOPIC_SAVO_LIDAR_WATCHDOG_STATE: Final[str] = DEFAULT_WATCHDOG_STATE_TOPIC
TOPIC_SAVO_LIDAR_SECTOR_SCAN: Final[str] = DEFAULT_SECTOR_TOPIC


# =============================================================================
# Serial / Hardware Defaults
# =============================================================================
DEFAULT_SERIAL_PORT: Final[str] = "/dev/ttyUSB0"
DEFAULT_BAUDRATE: Final[int] = 115200
DEFAULT_SCAN_MODE: Final[str] = "standard"

SERIAL_TIMEOUT_S_DEFAULT: Final[float] = 1.0
SERIAL_RECONNECT_DELAY_S_DEFAULT: Final[float] = 1.0
SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT: Final[float] = 5.0

MOTOR_START_SETTLE_S_DEFAULT: Final[float] = 2.0
MOTOR_STOP_TIMEOUT_S_DEFAULT: Final[float] = 1.0


# =============================================================================
# Scan Geometry / Range Defaults
# =============================================================================
DEFAULT_MIN_RANGE_M: Final[float] = 0.15
DEFAULT_MAX_RANGE_M: Final[float] = 12.0

DEFAULT_ANGLE_MIN_RAD: Final[float] = -3.141592653589793
DEFAULT_ANGLE_MAX_RAD: Final[float] = 3.141592653589793

DEFAULT_FRONT_SECTOR_MIN_DEG: Final[float] = -45.0
DEFAULT_FRONT_SECTOR_MAX_DEG: Final[float] = 45.0

DEFAULT_LEFT_SECTOR_MIN_DEG: Final[float] = 45.0
DEFAULT_LEFT_SECTOR_MAX_DEG: Final[float] = 135.0

DEFAULT_RIGHT_SECTOR_MIN_DEG: Final[float] = -135.0
DEFAULT_RIGHT_SECTOR_MAX_DEG: Final[float] = -45.0

DEFAULT_BACK_SECTOR_MIN_DEG: Final[float] = 135.0
DEFAULT_BACK_SECTOR_MAX_DEG: Final[float] = -135.0


# =============================================================================
# Rates / Timing Defaults
# =============================================================================
DEFAULT_SCAN_RATE_HZ: Final[float] = 5.5
DEFAULT_PUBLISH_RATE_HZ: Final[float] = 10.0

DEFAULT_HEALTH_PUBLISH_HZ: Final[float] = 1.0
DEFAULT_STATE_PUBLISH_HZ: Final[float] = 2.0
DEFAULT_HEARTBEAT_HZ: Final[float] = 1.0
DEFAULT_WATCHDOG_CHECK_HZ: Final[float] = 5.0

DEFAULT_STALE_TIMEOUT_S: Final[float] = 1.0

SCAN_RATE_MIN_HZ: Final[float] = 3.0
SCAN_RATE_WARN_HZ: Final[float] = 4.5
SCAN_RATE_TARGET_HZ: Final[float] = DEFAULT_SCAN_RATE_HZ

PUBLISH_RATE_MIN_HZ: Final[float] = 1.0
HEALTH_PUBLISH_HZ_MIN: Final[float] = 0.2
STATE_PUBLISH_HZ_MIN: Final[float] = 0.2
WATCHDOG_TIMEOUT_S_MIN: Final[float] = 0.20


# =============================================================================
# Scan Quality Thresholds
# =============================================================================
DEFAULT_QUALITY_WARN_VALID_RATIO: Final[float] = 0.60
DEFAULT_QUALITY_ERROR_VALID_RATIO: Final[float] = 0.30

QUALITY_GOOD_VALID_RATIO: Final[float] = 0.75
QUALITY_WARN_VALID_RATIO: Final[float] = DEFAULT_QUALITY_WARN_VALID_RATIO
QUALITY_ERROR_VALID_RATIO: Final[float] = DEFAULT_QUALITY_ERROR_VALID_RATIO

MAX_ALLOWED_INF_RATIO_DEFAULT: Final[float] = 0.70
MAX_ALLOWED_NAN_RATIO_DEFAULT: Final[float] = 0.05
MIN_VALID_POINTS_DEFAULT: Final[int] = 120


# =============================================================================
# Filter Defaults
# =============================================================================
RANGE_FILTER_ENABLED_DEFAULT: Final[bool] = True
ANGLE_FILTER_ENABLED_DEFAULT: Final[bool] = False
SECTOR_EXTRACTION_ENABLED_DEFAULT: Final[bool] = True

REPLACE_INVALID_WITH_INF_DEFAULT: Final[bool] = True
DROP_OUT_OF_RANGE_DEFAULT: Final[bool] = False


# =============================================================================
# Backend / Mode Defaults
# =============================================================================
BACKEND_REAL: Final[str] = "real"
BACKEND_DRYRUN: Final[str] = "dryrun"

SUPPORTED_BACKENDS: Final[Tuple[str, str]] = (BACKEND_REAL, BACKEND_DRYRUN)
SUPPORTED_LIDAR_MODELS: Final[Tuple[str, ...]] = (LIDAR_MODEL_RPLIDAR_A1,)

DEFAULT_BACKEND: Final[str] = BACKEND_REAL


# =============================================================================
# QoS Defaults
# =============================================================================
QOS_DEPTH_DEFAULT: Final[int] = 10
QOS_SENSOR_DEPTH_DEFAULT: Final[int] = 10
QOS_STATE_DEPTH_DEFAULT: Final[int] = 10

QOS_SCAN_RELIABILITY_DEFAULT: Final[str] = "best_effort"
QOS_STATE_RELIABILITY_DEFAULT: Final[str] = "reliable"


# =============================================================================
# CLI / Script Defaults
# =============================================================================
CLI_TIMEOUT_S_DEFAULT: Final[float] = 5.0
CLI_SPIN_TIME_S_DEFAULT: Final[float] = 10.0
CLI_SCAN_SAMPLE_COUNT_DEFAULT: Final[int] = 5
CLI_PORT_SEARCH_TIMEOUT_S_DEFAULT: Final[float] = 3.0


# =============================================================================
# Status / Diagnostic Labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_WARN: Final[str] = "WARN"
STATUS_ERROR: Final[str] = "ERROR"
STATUS_STALE: Final[str] = "STALE"
STATUS_OFFLINE: Final[str] = "OFFLINE"
STATUS_DEGRADED: Final[str] = "DEGRADED"

WATCHDOG_NAME_SCAN: Final[str] = "scan_watchdog"
WATCHDOG_NAME_DRIVER: Final[str] = "lidar_driver_watchdog"


# =============================================================================
# Structured defaults
# =============================================================================
@dataclass(frozen=True)
class LidarDefaults:
    package_name: str = PACKAGE_NAME
    robot_name: str = ROBOT_NAME
    lidar_model: str = DEFAULT_LIDAR_MODEL

    driver_node_name: str = NODE_NAME_LIDAR_DRIVER
    py_driver_node_name: str = NODE_NAME_LIDAR_PY_DRIVER
    health_node_name: str = NODE_NAME_LIDAR_HEALTH
    filter_node_name: str = NODE_NAME_LIDAR_FILTER
    watchdog_node_name: str = NODE_NAME_LIDAR_WATCHDOG
    state_publisher_node_name: str = NODE_NAME_LIDAR_STATE_PUBLISHER

    frame_id: str = DEFAULT_FRAME_ID
    base_frame_id: str = DEFAULT_BASE_FRAME_ID

    scan_topic: str = DEFAULT_SCAN_TOPIC
    filtered_scan_topic: str = DEFAULT_FILTERED_SCAN_TOPIC
    health_topic: str = DEFAULT_HEALTH_TOPIC
    state_topic: str = DEFAULT_STATE_TOPIC
    scan_quality_topic: str = DEFAULT_SCAN_QUALITY_TOPIC
    heartbeat_topic: str = DEFAULT_HEARTBEAT_TOPIC
    watchdog_state_topic: str = DEFAULT_WATCHDOG_STATE_TOPIC
    sector_topic: str = DEFAULT_SECTOR_TOPIC

    serial_port: str = DEFAULT_SERIAL_PORT
    baudrate: int = DEFAULT_BAUDRATE
    scan_mode: str = DEFAULT_SCAN_MODE
    backend: str = DEFAULT_BACKEND

    min_range_m: float = DEFAULT_MIN_RANGE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    scan_rate_hz: float = DEFAULT_SCAN_RATE_HZ
    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ

    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S
    health_publish_hz: float = DEFAULT_HEALTH_PUBLISH_HZ
    state_publish_hz: float = DEFAULT_STATE_PUBLISH_HZ
    heartbeat_hz: float = DEFAULT_HEARTBEAT_HZ
    watchdog_check_hz: float = DEFAULT_WATCHDOG_CHECK_HZ

    quality_warn_valid_ratio: float = DEFAULT_QUALITY_WARN_VALID_RATIO
    quality_error_valid_ratio: float = DEFAULT_QUALITY_ERROR_VALID_RATIO

    front_sector_min_deg: float = DEFAULT_FRONT_SECTOR_MIN_DEG
    front_sector_max_deg: float = DEFAULT_FRONT_SECTOR_MAX_DEG

    def to_dict(self) -> dict:
        return {
            "identity": {
                "package_name": self.package_name,
                "robot_name": self.robot_name,
                "lidar_model": self.lidar_model,
            },
            "nodes": {
                "driver_node": self.driver_node_name,
                "py_driver_node": self.py_driver_node_name,
                "health_node": self.health_node_name,
                "filter_node": self.filter_node_name,
                "watchdog_node": self.watchdog_node_name,
                "state_publisher_node": self.state_publisher_node_name,
            },
            "frames": {
                "frame_id": self.frame_id,
                "base_frame_id": self.base_frame_id,
            },
            "topics": {
                "scan": self.scan_topic,
                "filtered_scan": self.filtered_scan_topic,
                "health": self.health_topic,
                "state": self.state_topic,
                "scan_quality": self.scan_quality_topic,
                "heartbeat": self.heartbeat_topic,
                "watchdog_state": self.watchdog_state_topic,
                "sector_scan": self.sector_topic,
            },
            "hardware": {
                "serial_port": self.serial_port,
                "baudrate": self.baudrate,
                "scan_mode": self.scan_mode,
                "backend": self.backend,
            },
            "scan": {
                "min_range_m": self.min_range_m,
                "max_range_m": self.max_range_m,
                "scan_rate_hz": self.scan_rate_hz,
                "publish_rate_hz": self.publish_rate_hz,
            },
            "rates_timeouts": {
                "stale_timeout_s": self.stale_timeout_s,
                "health_publish_hz": self.health_publish_hz,
                "state_publish_hz": self.state_publish_hz,
                "heartbeat_hz": self.heartbeat_hz,
                "watchdog_check_hz": self.watchdog_check_hz,
            },
            "quality": {
                "warn_valid_ratio": self.quality_warn_valid_ratio,
                "error_valid_ratio": self.quality_error_valid_ratio,
            },
            "front_sector": {
                "min_deg": self.front_sector_min_deg,
                "max_deg": self.front_sector_max_deg,
            },
        }


DEFAULTS: Final[LidarDefaults] = LidarDefaults()


# =============================================================================
# Helpers
# =============================================================================
def get_lidar_defaults() -> LidarDefaults:
    """Return immutable structured defaults for LiDAR components."""
    return DEFAULTS


def is_supported_backend(backend: str) -> bool:
    """Return True if the backend name is supported by this package."""
    return backend in SUPPORTED_BACKENDS


def is_supported_lidar_model(model: str) -> bool:
    """Return True if the LiDAR model name is supported by this package."""
    return model in SUPPORTED_LIDAR_MODELS


def clamp_range_m(value: float) -> float:
    """Clamp a range value to the configured LiDAR range envelope."""
    v = float(value)

    if v < DEFAULT_MIN_RANGE_M:
        return DEFAULT_MIN_RANGE_M

    if v > DEFAULT_MAX_RANGE_M:
        return DEFAULT_MAX_RANGE_M

    return v


__all__ = [
    # Identity
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "DEFAULT_LIDAR_MODEL",
    "LIDAR_MODEL_RPLIDAR_A1",
    # Nodes
    "NODE_NAME_LIDAR_DRIVER",
    "NODE_NAME_LIDAR_PY_DRIVER",
    "NODE_NAME_LIDAR_HEALTH",
    "NODE_NAME_LIDAR_FILTER",
    "NODE_NAME_LIDAR_WATCHDOG",
    "NODE_NAME_LIDAR_STATE_PUBLISHER",
    # Frames
    "DEFAULT_FRAME_ID",
    "DEFAULT_BASE_FRAME_ID",
    # Topics
    "DEFAULT_SCAN_TOPIC",
    "DEFAULT_FILTERED_SCAN_TOPIC",
    "DEFAULT_HEALTH_TOPIC",
    "DEFAULT_STATE_TOPIC",
    "DEFAULT_SCAN_QUALITY_TOPIC",
    "DEFAULT_HEARTBEAT_TOPIC",
    "DEFAULT_WATCHDOG_STATE_TOPIC",
    "DEFAULT_SECTOR_TOPIC",
    "TOPIC_SCAN",
    "TOPIC_SCAN_FILTERED",
    "TOPIC_SAVO_LIDAR_HEALTH",
    "TOPIC_SAVO_LIDAR_STATE",
    "TOPIC_SAVO_LIDAR_SCAN_QUALITY",
    "TOPIC_SAVO_LIDAR_HEARTBEAT",
    "TOPIC_SAVO_LIDAR_WATCHDOG_STATE",
    "TOPIC_SAVO_LIDAR_SECTOR_SCAN",
    # Serial / hardware
    "DEFAULT_SERIAL_PORT",
    "DEFAULT_BAUDRATE",
    "DEFAULT_SCAN_MODE",
    "SERIAL_TIMEOUT_S_DEFAULT",
    "SERIAL_RECONNECT_DELAY_S_DEFAULT",
    "SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT",
    "MOTOR_START_SETTLE_S_DEFAULT",
    "MOTOR_STOP_TIMEOUT_S_DEFAULT",
    # Scan geometry / range
    "DEFAULT_MIN_RANGE_M",
    "DEFAULT_MAX_RANGE_M",
    "DEFAULT_ANGLE_MIN_RAD",
    "DEFAULT_ANGLE_MAX_RAD",
    "DEFAULT_FRONT_SECTOR_MIN_DEG",
    "DEFAULT_FRONT_SECTOR_MAX_DEG",
    "DEFAULT_LEFT_SECTOR_MIN_DEG",
    "DEFAULT_LEFT_SECTOR_MAX_DEG",
    "DEFAULT_RIGHT_SECTOR_MIN_DEG",
    "DEFAULT_RIGHT_SECTOR_MAX_DEG",
    "DEFAULT_BACK_SECTOR_MIN_DEG",
    "DEFAULT_BACK_SECTOR_MAX_DEG",
    # Rates / timeouts
    "DEFAULT_SCAN_RATE_HZ",
    "DEFAULT_PUBLISH_RATE_HZ",
    "DEFAULT_HEALTH_PUBLISH_HZ",
    "DEFAULT_STATE_PUBLISH_HZ",
    "DEFAULT_HEARTBEAT_HZ",
    "DEFAULT_WATCHDOG_CHECK_HZ",
    "DEFAULT_STALE_TIMEOUT_S",
    "SCAN_RATE_MIN_HZ",
    "SCAN_RATE_WARN_HZ",
    "SCAN_RATE_TARGET_HZ",
    "PUBLISH_RATE_MIN_HZ",
    "HEALTH_PUBLISH_HZ_MIN",
    "STATE_PUBLISH_HZ_MIN",
    "WATCHDOG_TIMEOUT_S_MIN",
    # Quality
    "DEFAULT_QUALITY_WARN_VALID_RATIO",
    "DEFAULT_QUALITY_ERROR_VALID_RATIO",
    "QUALITY_GOOD_VALID_RATIO",
    "QUALITY_WARN_VALID_RATIO",
    "QUALITY_ERROR_VALID_RATIO",
    "MAX_ALLOWED_INF_RATIO_DEFAULT",
    "MAX_ALLOWED_NAN_RATIO_DEFAULT",
    "MIN_VALID_POINTS_DEFAULT",
    # Filters
    "RANGE_FILTER_ENABLED_DEFAULT",
    "ANGLE_FILTER_ENABLED_DEFAULT",
    "SECTOR_EXTRACTION_ENABLED_DEFAULT",
    "REPLACE_INVALID_WITH_INF_DEFAULT",
    "DROP_OUT_OF_RANGE_DEFAULT",
    # Backends
    "BACKEND_REAL",
    "BACKEND_DRYRUN",
    "SUPPORTED_BACKENDS",
    "SUPPORTED_LIDAR_MODELS",
    "DEFAULT_BACKEND",
    # QoS
    "QOS_DEPTH_DEFAULT",
    "QOS_SENSOR_DEPTH_DEFAULT",
    "QOS_STATE_DEPTH_DEFAULT",
    "QOS_SCAN_RELIABILITY_DEFAULT",
    "QOS_STATE_RELIABILITY_DEFAULT",
    # CLI
    "CLI_TIMEOUT_S_DEFAULT",
    "CLI_SPIN_TIME_S_DEFAULT",
    "CLI_SCAN_SAMPLE_COUNT_DEFAULT",
    "CLI_PORT_SEARCH_TIMEOUT_S_DEFAULT",
    # Status labels
    "STATUS_OK",
    "STATUS_WARN",
    "STATUS_ERROR",
    "STATUS_STALE",
    "STATUS_OFFLINE",
    "STATUS_DEGRADED",
    "WATCHDOG_NAME_SCAN",
    "WATCHDOG_NAME_DRIVER",
    # Structured defaults + helpers
    "LidarDefaults",
    "DEFAULTS",
    "get_lidar_defaults",
    "is_supported_backend",
    "is_supported_lidar_model",
    "clamp_range_m",
]