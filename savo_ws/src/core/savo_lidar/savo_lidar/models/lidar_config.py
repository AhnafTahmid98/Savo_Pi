# -*- coding: utf-8 -*-
"""LiDAR driver config used by the Python driver and the C++ driver contract."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    BACKEND_REAL,
    DEFAULT_BAUDRATE,
    DEFAULT_FRAME_ID,
    DEFAULT_HEARTBEAT_HZ,
    DEFAULT_HEARTBEAT_TOPIC,
    DEFAULT_LIDAR_MODEL,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_MODE,
    DEFAULT_SCAN_RATE_HZ,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_SERIAL_PORT,
    DEFAULT_STALE_TIMEOUT_S,
    DEFAULT_STATE_TOPIC,
    MAX_ALLOWED_INF_RATIO_DEFAULT,
    MAX_ALLOWED_NAN_RATIO_DEFAULT,
    MIN_VALID_POINTS_DEFAULT,
    MOTOR_START_SETTLE_S_DEFAULT,
    MOTOR_STOP_TIMEOUT_S_DEFAULT,
    SCAN_RATE_MIN_HZ,
    SCAN_RATE_WARN_HZ,
    SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT,
    SERIAL_RECONNECT_DELAY_S_DEFAULT,
    SERIAL_TIMEOUT_S_DEFAULT,
    SUPPORTED_BACKENDS,
    SUPPORTED_LIDAR_MODELS,
)


@dataclass(frozen=True)
class LidarDriverConfig:
    model: str = DEFAULT_LIDAR_MODEL
    backend: str = BACKEND_DRYRUN

    scan_topic: str = DEFAULT_SCAN_TOPIC
    frame_id: str = DEFAULT_FRAME_ID

    serial_port: str = DEFAULT_SERIAL_PORT
    baudrate: int = DEFAULT_BAUDRATE
    scan_mode: str = DEFAULT_SCAN_MODE

    serial_timeout_s: float = SERIAL_TIMEOUT_S_DEFAULT
    reconnect_on_error: bool = True
    reconnect_delay_s: float = SERIAL_RECONNECT_DELAY_S_DEFAULT
    max_reconnect_delay_s: float = SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT

    motor_start_settle_s: float = MOTOR_START_SETTLE_S_DEFAULT
    motor_stop_timeout_s: float = MOTOR_STOP_TIMEOUT_S_DEFAULT

    min_range_m: float = DEFAULT_MIN_RANGE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M

    expected_scan_rate_hz: float = DEFAULT_SCAN_RATE_HZ
    min_scan_rate_hz: float = SCAN_RATE_MIN_HZ
    scan_rate_warn_hz: float = SCAN_RATE_WARN_HZ
    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S

    inverted: bool = False
    angle_compensate: bool = True
    angle_offset_deg: float = 0.0

    require_valid_scan: bool = True
    min_valid_points: int = MIN_VALID_POINTS_DEFAULT
    max_allowed_inf_ratio: float = MAX_ALLOWED_INF_RATIO_DEFAULT
    max_allowed_nan_ratio: float = MAX_ALLOWED_NAN_RATIO_DEFAULT

    publish_driver_state: bool = True
    driver_state_topic: str = DEFAULT_STATE_TOPIC
    heartbeat_topic: str = DEFAULT_HEARTBEAT_TOPIC
    heartbeat_hz: float = DEFAULT_HEARTBEAT_HZ

    log_scan_summary: bool = False

    def validate(self) -> None:
        if self.model not in SUPPORTED_LIDAR_MODELS:
            raise ValueError(f"Unsupported LiDAR model: {self.model}")

        if self.backend not in SUPPORTED_BACKENDS:
            raise ValueError(f"Unsupported LiDAR backend: {self.backend}")

        if self.backend == BACKEND_REAL and not str(self.serial_port).strip():
            raise ValueError("serial_port is required when backend=real")

        if int(self.baudrate) <= 0:
            raise ValueError(f"baudrate must be > 0, got {self.baudrate}")

        if not str(self.frame_id).strip():
            raise ValueError("frame_id cannot be empty")

        if not str(self.scan_topic).strip():
            raise ValueError("scan_topic cannot be empty")

        if not str(self.scan_mode).strip():
            raise ValueError("scan_mode cannot be empty")

        if self.serial_timeout_s <= 0.0:
            raise ValueError(f"serial_timeout_s must be > 0.0, got {self.serial_timeout_s}")

        if self.reconnect_delay_s <= 0.0:
            raise ValueError(f"reconnect_delay_s must be > 0.0, got {self.reconnect_delay_s}")

        if self.max_reconnect_delay_s < self.reconnect_delay_s:
            raise ValueError(
                "max_reconnect_delay_s must be >= reconnect_delay_s, "
                f"got {self.max_reconnect_delay_s} < {self.reconnect_delay_s}"
            )

        if self.motor_start_settle_s < 0.0:
            raise ValueError(
                f"motor_start_settle_s must be >= 0.0, got {self.motor_start_settle_s}"
            )

        if self.motor_stop_timeout_s <= 0.0:
            raise ValueError(
                f"motor_stop_timeout_s must be > 0.0, got {self.motor_stop_timeout_s}"
            )

        if self.min_range_m <= 0.0:
            raise ValueError(f"min_range_m must be > 0.0, got {self.min_range_m}")

        if self.max_range_m <= self.min_range_m:
            raise ValueError(
                f"max_range_m must be greater than min_range_m "
                f"({self.min_range_m}), got {self.max_range_m}"
            )

        if self.expected_scan_rate_hz <= 0.0:
            raise ValueError(
                f"expected_scan_rate_hz must be > 0.0, got {self.expected_scan_rate_hz}"
            )

        if self.min_scan_rate_hz <= 0.0:
            raise ValueError(f"min_scan_rate_hz must be > 0.0, got {self.min_scan_rate_hz}")

        if self.scan_rate_warn_hz < self.min_scan_rate_hz:
            raise ValueError(
                "scan_rate_warn_hz must be >= min_scan_rate_hz, "
                f"got {self.scan_rate_warn_hz} < {self.min_scan_rate_hz}"
            )

        if self.publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0.0, got {self.publish_rate_hz}")

        if self.stale_timeout_s <= 0.0:
            raise ValueError(f"stale_timeout_s must be > 0.0, got {self.stale_timeout_s}")

        if self.min_valid_points < 1:
            raise ValueError(f"min_valid_points must be >= 1, got {self.min_valid_points}")

        _validate_ratio("max_allowed_inf_ratio", self.max_allowed_inf_ratio)
        _validate_ratio("max_allowed_nan_ratio", self.max_allowed_nan_ratio)

        if self.publish_driver_state and not str(self.driver_state_topic).strip():
            raise ValueError("driver_state_topic cannot be empty when publish_driver_state=true")

        if not str(self.heartbeat_topic).strip():
            raise ValueError("heartbeat_topic cannot be empty")

        if self.heartbeat_hz <= 0.0:
            raise ValueError(f"heartbeat_hz must be > 0.0, got {self.heartbeat_hz}")

    @property
    def is_real_backend(self) -> bool:
        return self.backend == BACKEND_REAL

    @property
    def is_dryrun_backend(self) -> bool:
        return self.backend == BACKEND_DRYRUN

    def to_dict(self) -> Dict[str, Any]:
        return {
            "model": self.model,
            "backend": self.backend,
            "scan_topic": self.scan_topic,
            "frame_id": self.frame_id,
            "serial_port": self.serial_port,
            "baudrate": self.baudrate,
            "scan_mode": self.scan_mode,
            "serial_timeout_s": self.serial_timeout_s,
            "reconnect_on_error": self.reconnect_on_error,
            "reconnect_delay_s": self.reconnect_delay_s,
            "max_reconnect_delay_s": self.max_reconnect_delay_s,
            "motor_start_settle_s": self.motor_start_settle_s,
            "motor_stop_timeout_s": self.motor_stop_timeout_s,
            "min_range_m": self.min_range_m,
            "max_range_m": self.max_range_m,
            "expected_scan_rate_hz": self.expected_scan_rate_hz,
            "min_scan_rate_hz": self.min_scan_rate_hz,
            "scan_rate_warn_hz": self.scan_rate_warn_hz,
            "publish_rate_hz": self.publish_rate_hz,
            "stale_timeout_s": self.stale_timeout_s,
            "inverted": self.inverted,
            "angle_compensate": self.angle_compensate,
            "angle_offset_deg": self.angle_offset_deg,
            "require_valid_scan": self.require_valid_scan,
            "min_valid_points": self.min_valid_points,
            "max_allowed_inf_ratio": self.max_allowed_inf_ratio,
            "max_allowed_nan_ratio": self.max_allowed_nan_ratio,
            "publish_driver_state": self.publish_driver_state,
            "driver_state_topic": self.driver_state_topic,
            "heartbeat_topic": self.heartbeat_topic,
            "heartbeat_hz": self.heartbeat_hz,
            "log_scan_summary": self.log_scan_summary,
        }


def make_driver_config(**overrides: object) -> LidarDriverConfig:
    config = LidarDriverConfig(**overrides)
    config.validate()
    return config


def _validate_ratio(name: str, value: float) -> None:
    if value < 0.0 or value > 1.0:
        raise ValueError(f"{name} must be between 0.0 and 1.0, got {value}")
