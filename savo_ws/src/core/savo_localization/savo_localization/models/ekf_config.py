#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""EKF configuration models for Robot Savo localization. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from savo_localization.constants import (
    DEFAULT_EKF_HEALTH_TOPIC,
    DEFAULT_EKF_NODE_NAME,
    DEFAULT_EKF_RATE_HZ,
    DEFAULT_EKF_SENSOR_TIMEOUT_S,
    DEFAULT_FILTERED_ODOM_TOPIC,
    DEFAULT_IMU_TOPIC,
    DEFAULT_PUBLISH_TF,
    DEFAULT_TWO_D_MODE,
    DEFAULT_WHEEL_ODOM_TOPIC,
    FRAME_BASE_LINK,
    FRAME_ODOM,
)

STATE_VECTOR_FIELDS = (
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "vroll",
    "vpitch",
    "vyaw",
    "ax",
    "ay",
    "az",
)

STATE_VECTOR_SIZE = len(STATE_VECTOR_FIELDS)

DEFAULT_MAP_FRAME = "map"
DEFAULT_WORLD_FRAME = FRAME_ODOM
DEFAULT_VO_ODOM_TOPIC = "/vo/odom"

# Wheel odometry: use planar velocity from mecanum wheel odom.
DEFAULT_WHEEL_ODOM_EKF_CONFIG = (
    False, False, False,
    False, False, False,
    True, True, False,
    False, False, True,
    False, False, False,
)

# IMU: use yaw and yaw rate by default for 2D localization.
DEFAULT_IMU_EKF_CONFIG = (
    False, False, False,
    False, False, True,
    False, False, False,
    False, False, True,
    False, False, False,
)

# VO is optional. When enabled, use planar pose and yaw.
DEFAULT_VO_ODOM_EKF_CONFIG = (
    True, True, False,
    False, False, True,
    False, False, False,
    False, False, False,
    False, False, False,
)

DEFAULT_PROCESS_NOISE_DIAGONAL = (
    0.05, 0.05, 0.06,
    0.03, 0.03, 0.06,
    0.10, 0.10, 0.10,
    0.03, 0.03, 0.06,
    0.20, 0.20, 0.20,
)

DEFAULT_INITIAL_ESTIMATE_COVARIANCE_DIAGONAL = (
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
)


@dataclass(frozen=True)
class EkfSensorConfig:
    name: str
    topic: str
    enabled: bool = True

    config: tuple[bool, ...] = DEFAULT_WHEEL_ODOM_EKF_CONFIG
    differential: bool = False
    relative: bool = False
    queue_size: int = 10
    pose_rejection_threshold: float | None = None
    twist_rejection_threshold: float | None = None
    nodelay: bool = False

    def validate(self) -> None:
        if not str(self.name).strip():
            raise ValueError("sensor name cannot be empty")

        if not str(self.topic).strip():
            raise ValueError(f"{self.name} topic cannot be empty")

        if len(self.config) != STATE_VECTOR_SIZE:
            raise ValueError(
                f"{self.name} config must contain {STATE_VECTOR_SIZE} values, "
                f"got {len(self.config)}"
            )

        if self.queue_size <= 0:
            raise ValueError(
                f"{self.name} queue_size must be > 0, got {self.queue_size}"
            )

        if (
            self.pose_rejection_threshold is not None
            and self.pose_rejection_threshold < 0.0
        ):
            raise ValueError(
                f"{self.name} pose_rejection_threshold must be >= 0.0"
            )

        if (
            self.twist_rejection_threshold is not None
            and self.twist_rejection_threshold < 0.0
        ):
            raise ValueError(
                f"{self.name} twist_rejection_threshold must be >= 0.0"
            )

    @property
    def active_fields(self) -> tuple[str, ...]:
        return tuple(
            field
            for field, enabled in zip(STATE_VECTOR_FIELDS, self.config)
            if enabled
        )

    def to_robot_localization_dict(self, key: str) -> dict[str, Any]:
        data: dict[str, Any] = {
            key: self.topic,
            f"{key}_config": list(self.config),
            f"{key}_differential": bool(self.differential),
            f"{key}_relative": bool(self.relative),
            f"{key}_queue_size": int(self.queue_size),
            f"{key}_nodelay": bool(self.nodelay),
        }

        if self.pose_rejection_threshold is not None:
            data[f"{key}_pose_rejection_threshold"] = float(
                self.pose_rejection_threshold
            )

        if self.twist_rejection_threshold is not None:
            data[f"{key}_twist_rejection_threshold"] = float(
                self.twist_rejection_threshold
            )

        return data

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "topic": self.topic,
            "enabled": bool(self.enabled),
            "config": list(self.config),
            "active_fields": list(self.active_fields),
            "differential": bool(self.differential),
            "relative": bool(self.relative),
            "queue_size": int(self.queue_size),
            "pose_rejection_threshold": self.pose_rejection_threshold,
            "twist_rejection_threshold": self.twist_rejection_threshold,
            "nodelay": bool(self.nodelay),
        }


@dataclass(frozen=True)
class EkfConfig:
    node_name: str = DEFAULT_EKF_NODE_NAME

    output_topic: str = DEFAULT_FILTERED_ODOM_TOPIC
    health_topic: str = DEFAULT_EKF_HEALTH_TOPIC

    frequency_hz: float = DEFAULT_EKF_RATE_HZ
    sensor_timeout_s: float = DEFAULT_EKF_SENSOR_TIMEOUT_S

    two_d_mode: bool = DEFAULT_TWO_D_MODE
    publish_tf: bool = DEFAULT_PUBLISH_TF
    publish_acceleration: bool = False

    map_frame: str = DEFAULT_MAP_FRAME
    odom_frame: str = FRAME_ODOM
    base_link_frame: str = FRAME_BASE_LINK
    world_frame: str = DEFAULT_WORLD_FRAME

    transform_time_offset_s: float = 0.0
    transform_timeout_s: float = 0.0

    print_diagnostics: bool = True
    debug: bool = False
    debug_out_file: str = "/tmp/savo_localization_ekf_debug.txt"

    wheel_odom: EkfSensorConfig = EkfSensorConfig(
        name="wheel_odom",
        topic=DEFAULT_WHEEL_ODOM_TOPIC,
        enabled=True,
        config=DEFAULT_WHEEL_ODOM_EKF_CONFIG,
        differential=False,
        relative=False,
        queue_size=10,
        twist_rejection_threshold=1.5,
    )

    imu: EkfSensorConfig = EkfSensorConfig(
        name="imu",
        topic=DEFAULT_IMU_TOPIC,
        enabled=True,
        config=DEFAULT_IMU_EKF_CONFIG,
        differential=False,
        relative=False,
        queue_size=20,
        pose_rejection_threshold=1.0,
        twist_rejection_threshold=1.0,
    )

    vo_odom: EkfSensorConfig = EkfSensorConfig(
        name="vo_odom",
        topic=DEFAULT_VO_ODOM_TOPIC,
        enabled=False,
        config=DEFAULT_VO_ODOM_EKF_CONFIG,
        differential=False,
        relative=True,
        queue_size=10,
        pose_rejection_threshold=2.0,
    )

    process_noise_diagonal: tuple[float, ...] = DEFAULT_PROCESS_NOISE_DIAGONAL
    initial_estimate_covariance_diagonal: tuple[float, ...] = (
        DEFAULT_INITIAL_ESTIMATE_COVARIANCE_DIAGONAL
    )

    def validate(self) -> None:
        if not self.node_name.strip():
            raise ValueError("node_name cannot be empty")

        if not self.output_topic.strip():
            raise ValueError("output_topic cannot be empty")

        if not self.health_topic.strip():
            raise ValueError("health_topic cannot be empty")

        if self.frequency_hz <= 0.0:
            raise ValueError(f"frequency_hz must be > 0.0, got {self.frequency_hz}")

        if self.sensor_timeout_s <= 0.0:
            raise ValueError(
                f"sensor_timeout_s must be > 0.0, got {self.sensor_timeout_s}"
            )

        for frame_name, frame_value in (
            ("map_frame", self.map_frame),
            ("odom_frame", self.odom_frame),
            ("base_link_frame", self.base_link_frame),
            ("world_frame", self.world_frame),
        ):
            if not frame_value.strip():
                raise ValueError(f"{frame_name} cannot be empty")

        for sensor in self.sensors:
            sensor.validate()

        if not self.active_sensors:
            raise ValueError("At least one EKF sensor input must be enabled")

        _validate_diagonal(
            self.process_noise_diagonal,
            "process_noise_diagonal",
        )
        _validate_diagonal(
            self.initial_estimate_covariance_diagonal,
            "initial_estimate_covariance_diagonal",
        )

    @property
    def sensors(self) -> tuple[EkfSensorConfig, EkfSensorConfig, EkfSensorConfig]:
        return (self.wheel_odom, self.imu, self.vo_odom)

    @property
    def active_sensors(self) -> tuple[EkfSensorConfig, ...]:
        return tuple(sensor for sensor in self.sensors if sensor.enabled)

    @property
    def uses_vo(self) -> bool:
        return self.vo_odom.enabled

    def process_noise_covariance(self) -> list[float]:
        return diagonal_15_to_covariance(self.process_noise_diagonal)

    def initial_estimate_covariance(self) -> list[float]:
        return diagonal_15_to_covariance(self.initial_estimate_covariance_diagonal)

    def to_robot_localization_params(self) -> dict[str, Any]:
        params: dict[str, Any] = {
            "frequency": float(self.frequency_hz),
            "sensor_timeout": float(self.sensor_timeout_s),
            "two_d_mode": bool(self.two_d_mode),
            "publish_tf": bool(self.publish_tf),
            "publish_acceleration": bool(self.publish_acceleration),
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_link_frame": self.base_link_frame,
            "world_frame": self.world_frame,
            "transform_time_offset": float(self.transform_time_offset_s),
            "transform_timeout": float(self.transform_timeout_s),
            "print_diagnostics": bool(self.print_diagnostics),
            "debug": bool(self.debug),
            "debug_out_file": self.debug_out_file,
            "process_noise_covariance": self.process_noise_covariance(),
            "initial_estimate_covariance": self.initial_estimate_covariance(),
        }

        odom_index = 0
        imu_index = 0

        for sensor in self.active_sensors:
            if sensor.name == "imu":
                key = f"imu{imu_index}"
                imu_index += 1
            else:
                key = f"odom{odom_index}"
                odom_index += 1

            params.update(sensor.to_robot_localization_dict(key))

        return params

    def to_dict(self) -> dict[str, Any]:
        return {
            "node_name": self.node_name,
            "output_topic": self.output_topic,
            "health_topic": self.health_topic,
            "frequency_hz": float(self.frequency_hz),
            "sensor_timeout_s": float(self.sensor_timeout_s),
            "two_d_mode": bool(self.two_d_mode),
            "publish_tf": bool(self.publish_tf),
            "publish_acceleration": bool(self.publish_acceleration),
            "map_frame": self.map_frame,
            "odom_frame": self.odom_frame,
            "base_link_frame": self.base_link_frame,
            "world_frame": self.world_frame,
            "transform_time_offset_s": float(self.transform_time_offset_s),
            "transform_timeout_s": float(self.transform_timeout_s),
            "print_diagnostics": bool(self.print_diagnostics),
            "debug": bool(self.debug),
            "debug_out_file": self.debug_out_file,
            "uses_vo": self.uses_vo,
            "active_sensors": [sensor.name for sensor in self.active_sensors],
            "wheel_odom": self.wheel_odom.to_dict(),
            "imu": self.imu.to_dict(),
            "vo_odom": self.vo_odom.to_dict(),
            "process_noise_diagonal": list(self.process_noise_diagonal),
            "initial_estimate_covariance_diagonal": list(
                self.initial_estimate_covariance_diagonal
            ),
        }


def make_ekf_config(**overrides: Any) -> EkfConfig:
    config = EkfConfig(**overrides)
    config.validate()
    return config


def ekf_config_from_ros_params(params: dict[str, Any]) -> EkfConfig:
    wheel_odom = EkfSensorConfig(
        name="wheel_odom",
        topic=str(params.get("wheel_odom_topic", DEFAULT_WHEEL_ODOM_TOPIC)),
        enabled=_parse_bool(params.get("use_wheel_odom", True)),
        config=_bool_tuple15(
            params.get("wheel_odom_config", DEFAULT_WHEEL_ODOM_EKF_CONFIG)
        ),
        differential=_parse_bool(params.get("wheel_odom_differential", False)),
        relative=_parse_bool(params.get("wheel_odom_relative", False)),
        queue_size=int(params.get("wheel_odom_queue_size", 10)),
        pose_rejection_threshold=_optional_float(
            params.get("wheel_odom_pose_rejection_threshold", None)
        ),
        twist_rejection_threshold=_optional_float(
            params.get("wheel_odom_twist_rejection_threshold", 1.5)
        ),
        nodelay=_parse_bool(params.get("wheel_odom_nodelay", False)),
    )

    imu = EkfSensorConfig(
        name="imu",
        topic=str(params.get("imu_topic", DEFAULT_IMU_TOPIC)),
        enabled=_parse_bool(params.get("use_imu", True)),
        config=_bool_tuple15(params.get("imu_config", DEFAULT_IMU_EKF_CONFIG)),
        differential=_parse_bool(params.get("imu_differential", False)),
        relative=_parse_bool(params.get("imu_relative", False)),
        queue_size=int(params.get("imu_queue_size", 20)),
        pose_rejection_threshold=_optional_float(
            params.get("imu_pose_rejection_threshold", 1.0)
        ),
        twist_rejection_threshold=_optional_float(
            params.get("imu_twist_rejection_threshold", 1.0)
        ),
        nodelay=_parse_bool(params.get("imu_nodelay", False)),
    )

    vo_odom = EkfSensorConfig(
        name="vo_odom",
        topic=str(params.get("vo_odom_topic", DEFAULT_VO_ODOM_TOPIC)),
        enabled=_parse_bool(params.get("use_vo", False)),
        config=_bool_tuple15(params.get("vo_odom_config", DEFAULT_VO_ODOM_EKF_CONFIG)),
        differential=_parse_bool(params.get("vo_odom_differential", False)),
        relative=_parse_bool(params.get("vo_odom_relative", True)),
        queue_size=int(params.get("vo_odom_queue_size", 10)),
        pose_rejection_threshold=_optional_float(
            params.get("vo_odom_pose_rejection_threshold", 2.0)
        ),
        twist_rejection_threshold=_optional_float(
            params.get("vo_odom_twist_rejection_threshold", None)
        ),
        nodelay=_parse_bool(params.get("vo_odom_nodelay", False)),
    )

    config = EkfConfig(
        node_name=str(params.get("node_name", DEFAULT_EKF_NODE_NAME)),
        output_topic=str(params.get("output_topic", DEFAULT_FILTERED_ODOM_TOPIC)),
        health_topic=str(params.get("health_topic", DEFAULT_EKF_HEALTH_TOPIC)),
        frequency_hz=float(params.get("frequency_hz", DEFAULT_EKF_RATE_HZ)),
        sensor_timeout_s=float(
            params.get("sensor_timeout_s", DEFAULT_EKF_SENSOR_TIMEOUT_S)
        ),
        two_d_mode=_parse_bool(params.get("two_d_mode", DEFAULT_TWO_D_MODE)),
        publish_tf=_parse_bool(params.get("publish_tf", DEFAULT_PUBLISH_TF)),
        publish_acceleration=_parse_bool(params.get("publish_acceleration", False)),
        map_frame=str(params.get("map_frame", DEFAULT_MAP_FRAME)),
        odom_frame=str(params.get("odom_frame", FRAME_ODOM)),
        base_link_frame=str(params.get("base_link_frame", FRAME_BASE_LINK)),
        world_frame=str(params.get("world_frame", DEFAULT_WORLD_FRAME)),
        transform_time_offset_s=float(params.get("transform_time_offset_s", 0.0)),
        transform_timeout_s=float(params.get("transform_timeout_s", 0.0)),
        print_diagnostics=_parse_bool(params.get("print_diagnostics", True)),
        debug=_parse_bool(params.get("debug", False)),
        debug_out_file=str(
            params.get("debug_out_file", "/tmp/savo_localization_ekf_debug.txt")
        ),
        wheel_odom=wheel_odom,
        imu=imu,
        vo_odom=vo_odom,
        process_noise_diagonal=_float_tuple15(
            params.get("process_noise_diagonal", DEFAULT_PROCESS_NOISE_DIAGONAL)
        ),
        initial_estimate_covariance_diagonal=_float_tuple15(
            params.get(
                "initial_estimate_covariance_diagonal",
                DEFAULT_INITIAL_ESTIMATE_COVARIANCE_DIAGONAL,
            )
        ),
    )
    config.validate()
    return config


def diagonal_15_to_covariance(diagonal: tuple[float, ...]) -> list[float]:
    _validate_diagonal(diagonal, "diagonal")

    size = STATE_VECTOR_SIZE
    values = [0.0] * (size * size)

    for index, value in enumerate(diagonal):
        values[index * size + index] = float(value)

    return values


def _validate_diagonal(values: tuple[float, ...], name: str) -> None:
    if len(values) != STATE_VECTOR_SIZE:
        raise ValueError(
            f"{name} must contain {STATE_VECTOR_SIZE} values, got {len(values)}"
        )

    for value in values:
        if float(value) < 0.0:
            raise ValueError(f"{name} values must be >= 0.0, got {value}")


def _bool_tuple15(value: Any) -> tuple[bool, ...]:
    if isinstance(value, str):
        value = [part.strip() for part in value.split(",") if part.strip()]

    try:
        result = tuple(_parse_bool(item) for item in value)
    except TypeError as exc:
        raise ValueError(f"EKF config must be iterable, got {value!r}") from exc

    if len(result) != STATE_VECTOR_SIZE:
        raise ValueError(
            f"EKF config must contain {STATE_VECTOR_SIZE} values, got {len(result)}"
        )

    return result


def _float_tuple15(value: Any) -> tuple[float, ...]:
    if isinstance(value, str):
        value = [part.strip() for part in value.split(",") if part.strip()]

    try:
        result = tuple(float(item) for item in value)
    except TypeError as exc:
        raise ValueError(f"EKF diagonal must be iterable, got {value!r}") from exc

    if len(result) != STATE_VECTOR_SIZE:
        raise ValueError(
            f"EKF diagonal must contain {STATE_VECTOR_SIZE} values, got {len(result)}"
        )

    return result


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None

    if isinstance(value, str) and not value.strip():
        return None

    return float(value)


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")

    return bool(value)
