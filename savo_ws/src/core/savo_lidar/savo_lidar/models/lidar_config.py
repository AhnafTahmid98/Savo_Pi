"""Configuration models for the Robot Savo LiDAR stack."""

from __future__ import annotations

from dataclasses import dataclass

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    BACKEND_REAL,
    DEFAULT_BAUDRATE,
    DEFAULT_FRAME_ID,
    DEFAULT_LIDAR_MODEL,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_MODE,
    DEFAULT_SCAN_RATE_HZ,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_SERIAL_PORT,
    DEFAULT_STALE_TIMEOUT_S,
    SUPPORTED_BACKENDS,
    SUPPORTED_LIDAR_MODELS,
)


@dataclass(frozen=True)
class LidarDriverConfig:
    model: str = DEFAULT_LIDAR_MODEL
    backend: str = BACKEND_DRYRUN
    serial_port: str = DEFAULT_SERIAL_PORT
    baudrate: int = DEFAULT_BAUDRATE
    frame_id: str = DEFAULT_FRAME_ID
    scan_topic: str = DEFAULT_SCAN_TOPIC
    scan_mode: str = DEFAULT_SCAN_MODE
    min_range_m: float = DEFAULT_MIN_RANGE_M
    max_range_m: float = DEFAULT_MAX_RANGE_M
    expected_scan_rate_hz: float = DEFAULT_SCAN_RATE_HZ
    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ
    stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S
    inverted: bool = False
    angle_compensate: bool = True

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

        if self.publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0.0, got {self.publish_rate_hz}")

        if self.stale_timeout_s <= 0.0:
            raise ValueError(f"stale_timeout_s must be > 0.0, got {self.stale_timeout_s}")

    @property
    def is_real_backend(self) -> bool:
        return self.backend == BACKEND_REAL

    @property
    def is_dryrun_backend(self) -> bool:
        return self.backend == BACKEND_DRYRUN


def make_driver_config(**overrides: object) -> LidarDriverConfig:
    config = LidarDriverConfig(**overrides)
    config.validate()
    return config