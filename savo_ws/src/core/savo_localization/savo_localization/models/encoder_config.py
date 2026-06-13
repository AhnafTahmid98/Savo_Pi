#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Encoder configuration models for Robot Savo four-wheel odometry. No ROS imports."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_localization.constants import (
    DEFAULT_ENCODER_CPR,
    DEFAULT_ENCODER_DEBOUNCE_S,
    DEFAULT_ENCODER_DECODING,
    DEFAULT_ENCODER_GPIO_MAP,
    DEFAULT_ENCODER_INVERT_MAP,
    DEFAULT_ENCODER_POLL_S,
    DEFAULT_ENCODER_REPORT_INTERVAL_S,
    DEFAULT_GEAR_RATIO,
    DEFAULT_TRACK_M,
    DEFAULT_USE_HW_DEBOUNCE,
    DEFAULT_USE_INTERNAL_PULLUP,
    DEFAULT_WHEELBASE_M,
    DEFAULT_WHEEL_DIAMETER_M,
    DEFAULT_WHEEL_ODOM_RATE_HZ,
    DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    DEFAULT_WHEEL_ODOM_TIMEOUT_S,
    DEFAULT_WHEEL_ODOM_TOPIC,
    ENCODER_MODEL_QUADRATURE,
    FRAME_BASE_LINK,
    FRAME_ODOM,
    ODOM_MODEL_MECANUM_4ENC,
    WHEEL_FL,
    WHEEL_FR,
    WHEEL_ORDER,
    WHEEL_RL,
    WHEEL_RR,
)
from savo_localization.math.encoder_math import (
    counts_per_wheel_rev,
    metres_per_count,
    require_valid_decoding_factor,
)


@dataclass(frozen=True)
class WheelEncoderConfig:
    name: str
    a_gpio: int
    b_gpio: int
    inverted: bool = True

    def validate(self) -> None:
        if self.name not in WHEEL_ORDER:
            raise ValueError(f"Unsupported wheel name: {self.name!r}")

        _validate_gpio(self.a_gpio, f"{self.name}.a_gpio")
        _validate_gpio(self.b_gpio, f"{self.name}.b_gpio")

        if int(self.a_gpio) == int(self.b_gpio):
            raise ValueError(
                f"{self.name} encoder A/B GPIO pins must be different, "
                f"got {self.a_gpio}"
            )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class EncoderHardwareConfig:
    model: str = ENCODER_MODEL_QUADRATURE

    fl: WheelEncoderConfig = WheelEncoderConfig(
        name=WHEEL_FL,
        a_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_FL][0],
        b_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_FL][1],
        inverted=DEFAULT_ENCODER_INVERT_MAP[WHEEL_FL],
    )
    fr: WheelEncoderConfig = WheelEncoderConfig(
        name=WHEEL_FR,
        a_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_FR][0],
        b_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_FR][1],
        inverted=DEFAULT_ENCODER_INVERT_MAP[WHEEL_FR],
    )
    rl: WheelEncoderConfig = WheelEncoderConfig(
        name=WHEEL_RL,
        a_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_RL][0],
        b_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_RL][1],
        inverted=DEFAULT_ENCODER_INVERT_MAP[WHEEL_RL],
    )
    rr: WheelEncoderConfig = WheelEncoderConfig(
        name=WHEEL_RR,
        a_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_RR][0],
        b_gpio=DEFAULT_ENCODER_GPIO_MAP[WHEEL_RR][1],
        inverted=DEFAULT_ENCODER_INVERT_MAP[WHEEL_RR],
    )

    gpiochip: int | None = None
    poll_s: float = DEFAULT_ENCODER_POLL_S
    debounce_s: float = DEFAULT_ENCODER_DEBOUNCE_S
    report_interval_s: float = DEFAULT_ENCODER_REPORT_INTERVAL_S

    use_internal_pullup: bool = DEFAULT_USE_INTERNAL_PULLUP
    use_hw_debounce: bool = DEFAULT_USE_HW_DEBOUNCE

    def validate(self) -> None:
        if self.model != ENCODER_MODEL_QUADRATURE:
            raise ValueError(f"Unsupported encoder model: {self.model}")

        for wheel in self.wheels:
            wheel.validate()

        _validate_unique_gpio_pins(self.wheels)

        if self.gpiochip is not None and int(self.gpiochip) < 0:
            raise ValueError(f"gpiochip must be >= 0, got {self.gpiochip}")

        if self.poll_s <= 0.0:
            raise ValueError(f"poll_s must be > 0.0, got {self.poll_s}")

        if self.debounce_s < 0.0:
            raise ValueError(f"debounce_s must be >= 0.0, got {self.debounce_s}")

        if self.report_interval_s <= 0.0:
            raise ValueError(
                f"report_interval_s must be > 0.0, got {self.report_interval_s}"
            )

    @property
    def wheels(self) -> tuple[WheelEncoderConfig, WheelEncoderConfig, WheelEncoderConfig, WheelEncoderConfig]:
        return (self.fl, self.fr, self.rl, self.rr)

    @property
    def wheel_map(self) -> dict[str, WheelEncoderConfig]:
        return {
            self.fl.name: self.fl,
            self.fr.name: self.fr,
            self.rl.name: self.rl,
            self.rr.name: self.rr,
        }

    @property
    def all_gpio_pins(self) -> list[int]:
        pins: list[int] = []

        for wheel in self.wheels:
            pins.extend([int(wheel.a_gpio), int(wheel.b_gpio)])

        return pins

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "gpiochip": self.gpiochip,
            "poll_s": self.poll_s,
            "debounce_s": self.debounce_s,
            "report_interval_s": self.report_interval_s,
            "use_internal_pullup": self.use_internal_pullup,
            "use_hw_debounce": self.use_hw_debounce,
            "wheels": {
                wheel.name: wheel.to_dict()
                for wheel in self.wheels
            },
            "all_gpio_pins": self.all_gpio_pins,
        }


@dataclass(frozen=True)
class EncoderGeometryConfig:
    wheel_diameter_m: float = DEFAULT_WHEEL_DIAMETER_M
    cpr: int = DEFAULT_ENCODER_CPR
    decoding: int = DEFAULT_ENCODER_DECODING
    gear_ratio: float = DEFAULT_GEAR_RATIO

    wheelbase_m: float = DEFAULT_WHEELBASE_M
    track_m: float = DEFAULT_TRACK_M

    def validate(self) -> None:
        if self.wheel_diameter_m <= 0.0:
            raise ValueError(
                f"wheel_diameter_m must be > 0.0, got {self.wheel_diameter_m}"
            )

        if self.cpr <= 0:
            raise ValueError(f"cpr must be > 0, got {self.cpr}")

        require_valid_decoding_factor(self.decoding)

        if self.gear_ratio <= 0.0:
            raise ValueError(f"gear_ratio must be > 0.0, got {self.gear_ratio}")

        if self.wheelbase_m <= 0.0:
            raise ValueError(f"wheelbase_m must be > 0.0, got {self.wheelbase_m}")

        if self.track_m <= 0.0:
            raise ValueError(f"track_m must be > 0.0, got {self.track_m}")

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

    @property
    def radius_sum_m(self) -> float:
        return float(self.wheelbase_m) + float(self.track_m)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["counts_per_wheel_rev"] = self.counts_per_wheel_rev
        data["metres_per_count"] = self.metres_per_count
        data["radius_sum_m"] = self.radius_sum_m
        return data


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

    pose_covariance: tuple[float, float, float, float, float, float] = (
        0.05,
        0.10,
        999.0,
        999.0,
        999.0,
        0.10,
    )
    twist_covariance: tuple[float, float, float, float, float, float] = (
        0.05,
        0.10,
        999.0,
        999.0,
        999.0,
        0.10,
    )

    def validate(self) -> None:
        if self.model != ODOM_MODEL_MECANUM_4ENC:
            raise ValueError(f"Unsupported odometry model: {self.model}")

        if not self.odom_frame_id.strip():
            raise ValueError("odom_frame_id cannot be empty")

        if not self.base_frame_id.strip():
            raise ValueError("base_frame_id cannot be empty")

        if not self.wheel_odom_topic.strip():
            raise ValueError("wheel_odom_topic cannot be empty")

        if not self.wheel_odom_state_topic.strip():
            raise ValueError("wheel_odom_state_topic cannot be empty")

        if self.publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0.0, got {self.publish_rate_hz}")

        if self.timeout_s <= 0.0:
            raise ValueError(f"timeout_s must be > 0.0, got {self.timeout_s}")

        _validate_covariance6(self.pose_covariance, "pose_covariance")
        _validate_covariance6(self.twist_covariance, "twist_covariance")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class EncoderConfig:
    hardware: EncoderHardwareConfig = EncoderHardwareConfig()
    geometry: EncoderGeometryConfig = EncoderGeometryConfig()
    wheel_odom: WheelOdomConfig = WheelOdomConfig()

    def validate(self) -> None:
        self.hardware.validate()
        self.geometry.validate()
        self.wheel_odom.validate()

    def to_dict(self) -> dict[str, Any]:
        return {
            "hardware": self.hardware.to_dict(),
            "geometry": self.geometry.to_dict(),
            "wheel_odom": self.wheel_odom.to_dict(),
        }


def make_encoder_config(**overrides: Any) -> EncoderConfig:
    config = EncoderConfig(**overrides)
    config.validate()
    return config


def encoder_config_from_ros_params(params: dict[str, Any]) -> EncoderConfig:
    hardware = EncoderHardwareConfig(
        model=str(params.get("encoder_model", ENCODER_MODEL_QUADRATURE)),
        fl=_wheel_config_from_params(params, WHEEL_FL, "fl"),
        fr=_wheel_config_from_params(params, WHEEL_FR, "fr"),
        rl=_wheel_config_from_params(params, WHEEL_RL, "rl"),
        rr=_wheel_config_from_params(params, WHEEL_RR, "rr"),
        gpiochip=_optional_int(params.get("gpiochip", None)),
        poll_s=float(params.get("poll_s", DEFAULT_ENCODER_POLL_S)),
        debounce_s=float(params.get("debounce_s", DEFAULT_ENCODER_DEBOUNCE_S)),
        report_interval_s=float(
            params.get("report_interval_s", DEFAULT_ENCODER_REPORT_INTERVAL_S)
        ),
        use_internal_pullup=_parse_bool(
            params.get("use_internal_pullup", DEFAULT_USE_INTERNAL_PULLUP)
        ),
        use_hw_debounce=_parse_bool(
            params.get("use_hw_debounce", DEFAULT_USE_HW_DEBOUNCE)
        ),
    )

    geometry = EncoderGeometryConfig(
        wheel_diameter_m=float(
            params.get("wheel_diameter_m", DEFAULT_WHEEL_DIAMETER_M)
        ),
        cpr=int(params.get("cpr", DEFAULT_ENCODER_CPR)),
        decoding=int(params.get("decoding", DEFAULT_ENCODER_DECODING)),
        gear_ratio=float(params.get("gear_ratio", DEFAULT_GEAR_RATIO)),
        wheelbase_m=float(params.get("wheelbase_m", DEFAULT_WHEELBASE_M)),
        track_m=float(params.get("track_m", DEFAULT_TRACK_M)),
    )

    wheel_odom = WheelOdomConfig(
        model=str(params.get("odom_model", ODOM_MODEL_MECANUM_4ENC)),
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
        pose_covariance=_covariance6_tuple(
            params.get(
                "pose_covariance",
                (0.05, 0.10, 999.0, 999.0, 999.0, 0.10),
            )
        ),
        twist_covariance=_covariance6_tuple(
            params.get(
                "twist_covariance",
                (0.05, 0.10, 999.0, 999.0, 999.0, 0.10),
            )
        ),
    )

    config = EncoderConfig(
        hardware=hardware,
        geometry=geometry,
        wheel_odom=wheel_odom,
    )
    config.validate()
    return config


def _wheel_config_from_params(
    params: dict[str, Any],
    wheel_name: str,
    prefix: str,
) -> WheelEncoderConfig:
    default_a, default_b = DEFAULT_ENCODER_GPIO_MAP[wheel_name]

    return WheelEncoderConfig(
        name=wheel_name,
        a_gpio=int(params.get(f"{prefix}_a_gpio", default_a)),
        b_gpio=int(params.get(f"{prefix}_b_gpio", default_b)),
        inverted=_parse_bool(
            params.get(f"invert_{prefix}", DEFAULT_ENCODER_INVERT_MAP[wheel_name])
        ),
    )


def _validate_gpio(gpio: int, name: str) -> None:
    gpio = int(gpio)

    if gpio < 0:
        raise ValueError(f"{name} must be >= 0, got {gpio}")


def _validate_unique_gpio_pins(wheels: tuple[WheelEncoderConfig, ...]) -> None:
    pins: list[int] = []

    for wheel in wheels:
        pins.extend([int(wheel.a_gpio), int(wheel.b_gpio)])

    if len(set(pins)) != len(pins):
        raise ValueError("Encoder GPIO map contains duplicate pins.")


def _optional_int(value: Any) -> int | None:
    if value is None:
        return None

    if isinstance(value, str) and not value.strip():
        return None

    return int(value)


def _parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")

    return bool(value)


def _covariance6_tuple(value: Any) -> tuple[float, float, float, float, float, float]:
    if isinstance(value, str):
        value = [part.strip() for part in value.split(",") if part.strip()]

    try:
        values = tuple(float(item) for item in value)
    except TypeError as exc:
        raise ValueError(f"covariance must be iterable, got {value!r}") from exc

    if len(values) != 6:
        raise ValueError(f"covariance must contain exactly 6 values, got {len(values)}")

    return values


def _validate_covariance6(
    values: tuple[float, float, float, float, float, float],
    name: str,
) -> None:
    if len(values) != 6:
        raise ValueError(f"{name} must contain exactly 6 values")

    for value in values:
        if float(value) < 0.0:
            raise ValueError(f"{name} values must be >= 0.0, got {value}")