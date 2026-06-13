#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""IMU configuration model for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_localization.constants import (
    BNO055_DEFAULT_ADDRESS,
    BNO055_DEFAULT_I2C_BUS,
    BNO055_DEFAULT_MODE,
    BNO055_MODE_IMU,
    BNO055_MODE_NDOF,
    DEFAULT_IMU_ACCEL_STD_MOVING_MPS2,
    DEFAULT_IMU_GYRO_RMS_MOVING_DPS,
    DEFAULT_IMU_MOTION_WINDOW_SAMPLES,
    DEFAULT_IMU_RATE_HZ,
    DEFAULT_IMU_SAMPLE_COUNT,
    DEFAULT_IMU_STATE_TOPIC,
    DEFAULT_IMU_TOPIC,
    FRAME_IMU,
    IMU_MODEL_BNO055,
)


VALID_BNO055_MODES = (BNO055_MODE_IMU, BNO055_MODE_NDOF)


@dataclass(frozen=True)
class ImuConfig:
    model: str = IMU_MODEL_BNO055

    i2c_bus: int = BNO055_DEFAULT_I2C_BUS
    i2c_address: int = BNO055_DEFAULT_ADDRESS
    mode: str = BNO055_DEFAULT_MODE

    frame_id: str = FRAME_IMU
    imu_topic: str = DEFAULT_IMU_TOPIC
    imu_state_topic: str = DEFAULT_IMU_STATE_TOPIC

    publish_rate_hz: float = DEFAULT_IMU_RATE_HZ
    sample_count: int = DEFAULT_IMU_SAMPLE_COUNT

    publish_orientation: bool = True
    publish_magnetic_field: bool = True
    publish_temperature: bool = True
    reset_on_start: bool = True

    gyro_rms_moving_dps: float = DEFAULT_IMU_GYRO_RMS_MOVING_DPS
    accel_std_moving_mps2: float = DEFAULT_IMU_ACCEL_STD_MOVING_MPS2
    motion_window_samples: int = DEFAULT_IMU_MOTION_WINDOW_SAMPLES

    orientation_covariance: tuple[float, float, float] = (0.05, 0.05, 0.10)
    angular_velocity_covariance: tuple[float, float, float] = (0.02, 0.02, 0.02)
    linear_acceleration_covariance: tuple[float, float, float] = (0.10, 0.10, 0.10)

    def validate(self) -> None:
        if self.model != IMU_MODEL_BNO055:
            raise ValueError(f"Unsupported IMU model: {self.model}")

        if self.i2c_bus < 0:
            raise ValueError(f"i2c_bus must be >= 0, got {self.i2c_bus}")

        if not 0x00 <= int(self.i2c_address) <= 0x7F:
            raise ValueError(
                f"i2c_address must be a valid 7-bit I2C address, got {self.i2c_address}"
            )

        if self.mode not in VALID_BNO055_MODES:
            raise ValueError(
                f"mode must be one of {VALID_BNO055_MODES}, got {self.mode!r}"
            )

        if not self.frame_id.strip():
            raise ValueError("frame_id cannot be empty")

        if not self.imu_topic.strip():
            raise ValueError("imu_topic cannot be empty")

        if not self.imu_state_topic.strip():
            raise ValueError("imu_state_topic cannot be empty")

        if self.publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0.0, got {self.publish_rate_hz}")

        if self.sample_count <= 0:
            raise ValueError(f"sample_count must be > 0, got {self.sample_count}")

        if self.gyro_rms_moving_dps < 0.0:
            raise ValueError(
                f"gyro_rms_moving_dps must be >= 0.0, got {self.gyro_rms_moving_dps}"
            )

        if self.accel_std_moving_mps2 < 0.0:
            raise ValueError(
                "accel_std_moving_mps2 must be >= 0.0, "
                f"got {self.accel_std_moving_mps2}"
            )

        if self.motion_window_samples <= 0:
            raise ValueError(
                f"motion_window_samples must be > 0, got {self.motion_window_samples}"
            )

        _validate_covariance_diag(
            self.orientation_covariance,
            name="orientation_covariance",
        )
        _validate_covariance_diag(
            self.angular_velocity_covariance,
            name="angular_velocity_covariance",
        )
        _validate_covariance_diag(
            self.linear_acceleration_covariance,
            name="linear_acceleration_covariance",
        )

    @property
    def imu_only(self) -> bool:
        return self.mode == BNO055_MODE_IMU

    @property
    def ndof_enabled(self) -> bool:
        return self.mode == BNO055_MODE_NDOF

    @property
    def i2c_address_hex(self) -> str:
        return hex(int(self.i2c_address))

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["i2c_address_hex"] = self.i2c_address_hex
        data["imu_only"] = self.imu_only
        data["ndof_enabled"] = self.ndof_enabled
        return data


def make_imu_config(**overrides: Any) -> ImuConfig:
    config = ImuConfig(**overrides)
    config.validate()
    return config


def imu_config_from_ros_params(params: dict[str, Any]) -> ImuConfig:
    config = ImuConfig(
        model=str(params.get("model", IMU_MODEL_BNO055)),
        i2c_bus=int(params.get("i2c_bus", BNO055_DEFAULT_I2C_BUS)),
        i2c_address=_parse_i2c_address(
            params.get("i2c_address", BNO055_DEFAULT_ADDRESS)
        ),
        mode=str(params.get("mode", BNO055_DEFAULT_MODE)).lower(),
        frame_id=str(params.get("frame_id", FRAME_IMU)),
        imu_topic=str(params.get("imu_topic", DEFAULT_IMU_TOPIC)),
        imu_state_topic=str(params.get("imu_state_topic", DEFAULT_IMU_STATE_TOPIC)),
        publish_rate_hz=float(params.get("publish_rate_hz", DEFAULT_IMU_RATE_HZ)),
        sample_count=int(params.get("sample_count", DEFAULT_IMU_SAMPLE_COUNT)),
        publish_orientation=bool(params.get("publish_orientation", True)),
        publish_magnetic_field=bool(params.get("publish_magnetic_field", True)),
        publish_temperature=bool(params.get("publish_temperature", True)),
        reset_on_start=bool(params.get("reset_on_start", True)),
        gyro_rms_moving_dps=float(
            params.get("gyro_rms_moving_dps", DEFAULT_IMU_GYRO_RMS_MOVING_DPS)
        ),
        accel_std_moving_mps2=float(
            params.get("accel_std_moving_mps2", DEFAULT_IMU_ACCEL_STD_MOVING_MPS2)
        ),
        motion_window_samples=int(
            params.get("motion_window_samples", DEFAULT_IMU_MOTION_WINDOW_SAMPLES)
        ),
        orientation_covariance=_covariance_tuple(
            params.get("orientation_covariance", (0.05, 0.05, 0.10))
        ),
        angular_velocity_covariance=_covariance_tuple(
            params.get("angular_velocity_covariance", (0.02, 0.02, 0.02))
        ),
        linear_acceleration_covariance=_covariance_tuple(
            params.get("linear_acceleration_covariance", (0.10, 0.10, 0.10))
        ),
    )
    config.validate()
    return config


def _parse_i2c_address(value: Any) -> int:
    if isinstance(value, str):
        return int(value, 0)

    return int(value)


def _covariance_tuple(value: Any) -> tuple[float, float, float]:
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",") if part.strip()]
        value = parts

    try:
        values = tuple(float(item) for item in value)
    except TypeError as exc:
        raise ValueError(f"covariance value must be iterable, got {value!r}") from exc

    if len(values) != 3:
        raise ValueError(f"covariance must contain exactly 3 values, got {len(values)}")

    return values


def _validate_covariance_diag(
    values: tuple[float, float, float],
    *,
    name: str,
) -> None:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")

    for value in values:
        if float(value) < 0.0:
            raise ValueError(f"{name} values must be >= 0.0, got {value}")