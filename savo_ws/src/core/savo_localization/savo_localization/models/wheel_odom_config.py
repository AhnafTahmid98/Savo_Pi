#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Wheel odometry configuration model for Robot Savo. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_localization.constants import (
    DEFAULT_ODOM_COVARIANCE_SCALE,
    DEFAULT_TRACK_M,
    DEFAULT_WHEEL_ODOM_RATE_HZ,
    DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    DEFAULT_WHEEL_ODOM_TIMEOUT_S,
    DEFAULT_WHEEL_ODOM_TOPIC,
    DEFAULT_WHEELBASE_M,
    DEFAULT_WHEEL_DIAMETER_M,
    DEFAULT_ENCODER_CPR,
    DEFAULT_ENCODER_DECODING,
    DEFAULT_GEAR_RATIO,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    ODOM_MODEL_MECANUM_4ENC,
)
from savo_localization.math.covariance import (
    make_wheel_odom_pose_covariance,
    make_wheel_odom_twist_covariance,
    scale_covariance,
)
from savo_localization.math.encoder_math import (
    counts_per_wheel_rev,
    metres_per_count,
    require_valid_decoding_factor,
)
from savo_localization.utils.frames import normalize_frame_id
from savo_localization.utils.topic_names import normalize_topic


@dataclass(frozen=True)
class WheelOdomGeometryConfig:
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M
    wheelbase_m: float = DEFAULT_WHEELBASE_M
    track_m: float = DEFAULT_TRACK_M

    cpr: int = DEFAULT_ENCODER_CPR
    decoding: int = DEFAULT_ENCODER_DECODING
    gear_ratio: float = DEFAULT_GEAR_RATIO

    def validate(self) -> None:
        if self.wheel_diameter_m <= 0.0:
            raise ValueError(
                f"wheel_diameter_m must be > 0.0, got {self.wheel_diameter_m}"
            )

        if self.wheelbase_m <= 0.0:
            raise ValueError(f"wheelbase_m must be > 0.0, got {self.wheelbase_m}")

        if self.track_m <= 0.0:
            raise ValueError(f"track_m must be > 0.0, got {self.track_m}")

        if self.cpr <= 0:
            raise ValueError(f"cpr must be > 0, got {self.cpr}")

        require_valid_decoding_factor(self.decoding)

        if self.gear_ratio <= 0.0:
            raise ValueError(f"gear_ratio must be > 0.0, got {self.gear_ratio}")

    @property
    def radius_sum_m(self) -> float:
        return float(self.wheelbase_m) + float(self.track_m)

    @property
    def counts_per_wheel_rev(self) -> int:
        return counts_per_wheel_rev(
            cpr=self.cpr,
            decoding=self.decoding,
            gear_ratio=self.gear_ratio,
        )

    @property
    def metres_per_count(self) -> float:
        return metres_per_count(
            wheel_diameter_m=self.wheel_diameter_m,
            counts_per_rev=self.counts_per_wheel_rev,
        )

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["radius_sum_m"] = self.radius_sum_m
        data["counts_per_wheel_rev"] = self.counts_per_wheel_rev
        data["metres_per_count"] = self.metres_per_count
        return data


@dataclass(frozen=True)
class WheelOdomCovarianceConfig:
    scale: float = DEFAULT_ODOM_COVARIANCE_SCALE

    pose_x: float = 0.05
    pose_y: float = 0.10
    pose_z: float = 999.0
    pose_roll: float = 999.0
    pose_pitch: float = 999.0
    pose_yaw: float = 0.10

    twist_vx: float = 0.05
    twist_vy: float = 0.10
    twist_vz: float = 999.0
    twist_wx: float = 999.0
    twist_wy: float = 999.0
    twist_wz: float = 0.10

    def validate(self) -> None:
        if self.scale < 0.0:
            raise ValueError(f"scale must be >= 0.0, got {self.scale}")

        for name, value in self.to_variance_dict().items():
            if float(value) < 0.0:
                raise ValueError(f"{name} must be >= 0.0, got {value}")

    def pose_covariance(self) -> list[float]:
        values = make_wheel_odom_pose_covariance(
            x_variance=self.pose_x,
            y_variance=self.pose_y,
            z_variance=self.pose_z,
            roll_variance=self.pose_roll,
            pitch_variance=self.pose_pitch,
            yaw_variance=self.pose_yaw,
        )
        return scale_covariance(values, self.scale)

    def twist_covariance(self) -> list[float]:
        values = make_wheel_odom_twist_covariance(
            vx_variance=self.twist_vx,
            vy_variance=self.twist_vy,
            vz_variance=self.twist_vz,
            wx_variance=self.twist_wx,
            wy_variance=self.twist_wy,
            wz_variance=self.twist_wz,
        )
        return scale_covariance(values, self.scale)

    def to_variance_dict(self) -> dict[str, float]:
        return {
            "pose_x": self.pose_x,
            "pose_y": self.pose_y,
            "pose_z": self.pose_z,
            "pose_roll": self.pose_roll,
            "pose_pitch": self.pose_pitch,
            "pose_yaw": self.pose_yaw,
            "twist_vx": self.twist_vx,
            "twist_vy": self.twist_vy,
            "twist_vz": self.twist_vz,
            "twist_wx": self.twist_wx,
            "twist_wy": self.twist_wy,
            "twist_wz": self.twist_wz,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "scale": float(self.scale),
            **self.to_variance_dict(),
            "pose_covariance": self.pose_covariance(),
            "twist_covariance": self.twist_covariance(),
        }


@dataclass(frozen=True)
class WheelOdomConfig:
    model: str = ODOM_MODEL_MECANUM_4ENC

    odom_frame_id: str = FRAME_ODOM
    base_frame_id: str = FRAME_BASE_LINK

    wheel_odom_topic: str = DEFAULT_WHEEL_ODOM_TOPIC
    wheel_odom_state_topic: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC

    publish_rate_hz: float = DEFAULT_WHEEL_ODOM_RATE_HZ
    timeout_s: float = DEFAULT_WHEEL_ODOM_TIMEOUT_S
    publish_tf: bool = False

    geometry: WheelOdomGeometryConfig = WheelOdomGeometryConfig()
    covariance: WheelOdomCovarianceConfig = WheelOdomCovarianceConfig()

    reset_pose_on_start: bool = True
    start_x_m: float = 0.0
    start_y_m: float = 0.0
    start_yaw_rad: float = 0.0

    def validate(self) -> None:
        if self.model != ODOM_MODEL_MECANUM_4ENC:
            raise ValueError(f"Unsupported wheel odom model: {self.model}")

        normalize_frame_id(self.odom_frame_id)
        normalize_frame_id(self.base_frame_id)
        normalize_topic(self.wheel_odom_topic)
        normalize_topic(self.wheel_odom_state_topic)

        if self.publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0.0, got {self.publish_rate_hz}")

        if self.timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0.0, got {self.timeout_s}")

        self.geometry.validate()
        self.covariance.validate()

    @property
    def normalized_odom_frame_id(self) -> str:
        return normalize_frame_id(self.odom_frame_id)

    @property
    def normalized_base_frame_id(self) -> str:
        return normalize_frame_id(self.base_frame_id)

    @property
    def normalized_wheel_odom_topic(self) -> str:
        return normalize_topic(self.wheel_odom_topic)

    @property
    def normalized_wheel_odom_state_topic(self) -> str:
        return normalize_topic(self.wheel_odom_state_topic)

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "odom_frame_id": self.normalized_odom_frame_id,
            "base_frame_id": self.normalized_base_frame_id,
            "wheel_odom_topic": self.normalized_wheel_odom_topic,
            "wheel_odom_state_topic": self.normalized_wheel_odom_state_topic,
            "publish_rate_hz": float(self.publish_rate_hz),
            "timeout_s": float(self.timeout_s),
            "publish_tf": bool(self.publish_tf),
            "reset_pose_on_start": bool(self.reset_pose_on_start),
            "start_x_m": float(self.start_x_m),
            "start_y_m": float(self.start_y_m),
            "start_yaw_rad": float(self.start_yaw_rad),
            "geometry": self.geometry.to_dict(),
            "covariance": self.covariance.to_dict(),
        }


def make_wheel_odom_config(**overrides: Any) -> WheelOdomConfig:
    config = WheelOdomConfig(**overrides)
    config.validate()
    return config


def wheel_odom_config_from_ros_params(params: dict[str, Any]) -> WheelOdomConfig:
    geometry = WheelOdomGeometryConfig(
        wheel_diameter_m=float(
            params.get("wheel_diameter_m", DEFAULT_WHEEL_DIAMETER_M)
        ),
        wheelbase_m=float(params.get("wheelbase_m", DEFAULT_WHEELBASE_M)),
        track_m=float(params.get("track_m", DEFAULT_TRACK_M)),
        cpr=int(params.get("cpr", DEFAULT_ENCODER_CPR)),
        decoding=int(params.get("decoding", DEFAULT_ENCODER_DECODING)),
        gear_ratio=float(params.get("gear_ratio", DEFAULT_GEAR_RATIO)),
    )

    covariance = WheelOdomCovarianceConfig(
        scale=float(params.get("odom_covariance_scale", DEFAULT_ODOM_COVARIANCE_SCALE)),
        pose_x=float(params.get("pose_x_covariance", 0.05)),
        pose_y=float(params.get("pose_y_covariance", 0.10)),
        pose_z=float(params.get("pose_z_covariance", 999.0)),
        pose_roll=float(params.get("pose_roll_covariance", 999.0)),
        pose_pitch=float(params.get("pose_pitch_covariance", 999.0)),
        pose_yaw=float(params.get("pose_yaw_covariance", 0.10)),
        twist_vx=float(params.get("twist_vx_covariance", 0.05)),
        twist_vy=float(params.get("twist_vy_covariance", 0.10)),
        twist_vz=float(params.get("twist_vz_covariance", 999.0)),
        twist_wx=float(params.get("twist_wx_covariance", 999.0)),
        twist_wy=float(params.get("twist_wy_covariance", 999.0)),
        twist_wz=float(params.get("twist_wz_covariance", 0.10)),
    )

    config = WheelOdomConfig(
        model=str(params.get("model", ODOM_MODEL_MECANUM_4ENC)),
        odom_frame_id=str(params.get("odom_frame_id", FRAME_ODOM)),
        base_frame_id=str(params.get("base_frame_id", FRAME_BASE_LINK)),
        wheel_odom_topic=str(
            params.get("wheel_odom_topic", DEFAULT_WHEEL_ODOM_TOPIC)
        ),
        wheel_odom_state_topic=str(
            params.get("wheel_odom_state_topic", DEFAULT_WHEEL_ODOM_STATE_TOPIC)
        ),
        publish_rate_hz=float(
            params.get("publish_rate_hz", DEFAULT_WHEEL_ODOM_RATE_HZ)
        ),
        timeout_s=float(params.get("timeout_s", DEFAULT_WHEEL_ODOM_TIMEOUT_S)),
        publish_tf=_parse_bool(params.get("publish_tf", False)),
        geometry=geometry,
        covariance=covariance,
        reset_pose_on_start=_parse_bool(params.get("reset_pose_on_start", True)),
        start_x_m=float(params.get("start_x_m", 0.0)),
        start_y_m=float(params.get("start_y_m", 0.0)),
        start_yaw_rad=float(params.get("start_yaw_rad", 0.0)),
    )
    config.validate()
    return config


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")

    return bool(value)