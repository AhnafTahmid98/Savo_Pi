#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Parameter models and helpers for savo_perception Python components."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Mapping, Optional, Sequence

from savo_perception.constants import (
    CLEAR_DEBOUNCE_COUNT_DEFAULT,
    FAIL_SAFE_ON_STALE_DEFAULT,
    FRONT_CLEAR_HYSTERESIS_M_DEFAULT,
    FRONT_SLOW_M_DEFAULT,
    FRONT_STOP_M_DEFAULT,
    I2C_BUS_DEFAULT,
    RANGE_HEALTH_HZ_DEFAULT,
    SAFETY_LOOP_HZ_DEFAULT,
    SENSOR_STALE_TIMEOUT_S_DEFAULT,
    SIDE_CLEAR_HYSTERESIS_M_DEFAULT,
    SIDE_SLOW_M_DEFAULT,
    SIDE_STOP_M_DEFAULT,
    SLOWDOWN_EMA_ALPHA_DEFAULT,
    SLOWDOWN_MAX_DEFAULT,
    SLOWDOWN_MIN_DEFAULT,
    STOP_DEBOUNCE_COUNT_DEFAULT,
    TCA9548A_ADDR_DEFAULT,
    TOF_RATE_HZ_DEFAULT,
    ULTRASONIC_ECHO_PIN_DEFAULT,
    ULTRASONIC_MAX_DISTANCE_M_DEFAULT,
    ULTRASONIC_RATE_HZ_DEFAULT,
    ULTRASONIC_TRIG_PIN_DEFAULT,
    VL53_LEFT_CHANNEL_DEFAULT,
    VL53_MEDIAN_WINDOW_DEFAULT,
    VL53_RIGHT_CHANNEL_DEFAULT,
    VL53L1X_ADDR_DEFAULT,
)
from savo_perception.utils.topic_names import (
    CMD_VEL,
    CMD_VEL_SAFE,
    DEPTH_FRONT_M,
    SAFETY_SLOWDOWN_FACTOR,
    SAFETY_STOP,
    SAVO_PERCEPTION_RANGE_HEALTH,
    SAVO_PERCEPTION_SAFETY_STATE,
    TOF_LEFT_M,
    TOF_RIGHT_M,
    ULTRASONIC_FRONT_M,
)


@dataclass(frozen=True)
class Vl53MuxParams:
    bus: int = I2C_BUS_DEFAULT
    tca_addr: int = TCA9548A_ADDR_DEFAULT
    vl53_addr: int = VL53L1X_ADDR_DEFAULT
    left_channel: int = VL53_LEFT_CHANNEL_DEFAULT
    right_channel: int = VL53_RIGHT_CHANNEL_DEFAULT
    rate_hz: float = TOF_RATE_HZ_DEFAULT
    median_window: int = VL53_MEDIAN_WINDOW_DEFAULT
    left_topic: str = TOF_LEFT_M
    right_topic: str = TOF_RIGHT_M

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class UltrasonicParams:
    trig_pin: int = ULTRASONIC_TRIG_PIN_DEFAULT
    echo_pin: int = ULTRASONIC_ECHO_PIN_DEFAULT
    max_distance_m: float = ULTRASONIC_MAX_DISTANCE_M_DEFAULT
    rate_hz: float = ULTRASONIC_RATE_HZ_DEFAULT
    output_topic: str = ULTRASONIC_FRONT_M

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SafetyStopParams:
    depth_front_topic: str = DEPTH_FRONT_M
    tof_left_topic: str = TOF_LEFT_M
    tof_right_topic: str = TOF_RIGHT_M
    ultrasonic_front_topic: str = ULTRASONIC_FRONT_M
    safety_stop_topic: str = SAFETY_STOP
    slowdown_topic: str = SAFETY_SLOWDOWN_FACTOR
    safety_state_topic: str = SAVO_PERCEPTION_SAFETY_STATE

    loop_hz: float = SAFETY_LOOP_HZ_DEFAULT
    stale_timeout_s: float = SENSOR_STALE_TIMEOUT_S_DEFAULT

    front_stop_m: float = FRONT_STOP_M_DEFAULT
    front_slow_m: float = FRONT_SLOW_M_DEFAULT
    side_stop_m: float = SIDE_STOP_M_DEFAULT
    side_slow_m: float = SIDE_SLOW_M_DEFAULT

    front_clear_hysteresis_m: float = FRONT_CLEAR_HYSTERESIS_M_DEFAULT
    side_clear_hysteresis_m: float = SIDE_CLEAR_HYSTERESIS_M_DEFAULT

    stop_debounce_count: int = STOP_DEBOUNCE_COUNT_DEFAULT
    clear_debounce_count: int = CLEAR_DEBOUNCE_COUNT_DEFAULT

    slowdown_min: float = SLOWDOWN_MIN_DEFAULT
    slowdown_max: float = SLOWDOWN_MAX_DEFAULT
    slowdown_ema_alpha: float = SLOWDOWN_EMA_ALPHA_DEFAULT

    fail_safe_on_stale: bool = FAIL_SAFE_ON_STALE_DEFAULT

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class CmdVelSafetyGateParams:
    cmd_vel_topic: str = CMD_VEL
    cmd_vel_safe_topic: str = CMD_VEL_SAFE
    safety_stop_topic: str = SAFETY_STOP
    slowdown_topic: str = SAFETY_SLOWDOWN_FACTOR
    stale_timeout_s: float = SENSOR_STALE_TIMEOUT_S_DEFAULT
    fail_safe_on_stale: bool = FAIL_SAFE_ON_STALE_DEFAULT

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RangeHealthParams:
    depth_front_topic: str = DEPTH_FRONT_M
    tof_left_topic: str = TOF_LEFT_M
    tof_right_topic: str = TOF_RIGHT_M
    ultrasonic_front_topic: str = ULTRASONIC_FRONT_M
    range_health_topic: str = SAVO_PERCEPTION_RANGE_HEALTH
    publish_hz: float = RANGE_HEALTH_HZ_DEFAULT
    stale_timeout_s: float = SENSOR_STALE_TIMEOUT_S_DEFAULT

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _unwrap_ros_param(value: Any) -> Any:
    if hasattr(value, "value"):
        return value.value
    return value


def get_param(mapping: Mapping[str, Any], name: str, default: Any) -> Any:
    return _unwrap_ros_param(mapping.get(name, default))


def to_bool(value: Any) -> bool:
    value = _unwrap_ros_param(value)

    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)):
        return bool(value)

    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False

    raise ValueError(f"Cannot convert to bool: {value!r}")


def to_int(value: Any, *, min_value: Optional[int] = None) -> int:
    out = int(_unwrap_ros_param(value))
    if min_value is not None and out < min_value:
        return min_value
    return out


def to_float(value: Any, *, min_value: Optional[float] = None) -> float:
    out = float(_unwrap_ros_param(value))
    if min_value is not None and out < min_value:
        return min_value
    return out


def to_str(value: Any) -> str:
    return str(_unwrap_ros_param(value))


def declare_params(node: Any, defaults: Mapping[str, Any]) -> Dict[str, Any]:
    declared: Dict[str, Any] = {}

    for name, default in defaults.items():
        param = node.declare_parameter(name, default)
        declared[name] = _unwrap_ros_param(param)

    return declared


def read_node_params(node: Any, names: Sequence[str]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}

    for name in names:
        param = node.get_parameter(name)
        out[name] = _unwrap_ros_param(param)

    return out


def load_vl53_mux_params(values: Mapping[str, Any]) -> Vl53MuxParams:
    return Vl53MuxParams(
        bus=to_int(get_param(values, "bus", I2C_BUS_DEFAULT), min_value=0),
        tca_addr=to_int(get_param(values, "tca_addr", TCA9548A_ADDR_DEFAULT), min_value=0),
        vl53_addr=to_int(get_param(values, "vl53_addr", VL53L1X_ADDR_DEFAULT), min_value=0),
        left_channel=to_int(get_param(values, "left_channel", VL53_LEFT_CHANNEL_DEFAULT), min_value=0),
        right_channel=to_int(get_param(values, "right_channel", VL53_RIGHT_CHANNEL_DEFAULT), min_value=0),
        rate_hz=to_float(get_param(values, "rate_hz", TOF_RATE_HZ_DEFAULT), min_value=0.1),
        median_window=to_int(get_param(values, "median_window", VL53_MEDIAN_WINDOW_DEFAULT), min_value=1),
        left_topic=to_str(get_param(values, "left_topic", TOF_LEFT_M)),
        right_topic=to_str(get_param(values, "right_topic", TOF_RIGHT_M)),
    )


def load_ultrasonic_params(values: Mapping[str, Any]) -> UltrasonicParams:
    return UltrasonicParams(
        trig_pin=to_int(get_param(values, "trig_pin", ULTRASONIC_TRIG_PIN_DEFAULT), min_value=0),
        echo_pin=to_int(get_param(values, "echo_pin", ULTRASONIC_ECHO_PIN_DEFAULT), min_value=0),
        max_distance_m=to_float(
            get_param(values, "max_distance_m", ULTRASONIC_MAX_DISTANCE_M_DEFAULT),
            min_value=0.1,
        ),
        rate_hz=to_float(get_param(values, "rate_hz", ULTRASONIC_RATE_HZ_DEFAULT), min_value=0.1),
        output_topic=to_str(get_param(values, "output_topic", ULTRASONIC_FRONT_M)),
    )


def load_safety_stop_params(values: Mapping[str, Any]) -> SafetyStopParams:
    return SafetyStopParams(
        depth_front_topic=to_str(get_param(values, "depth_front_topic", DEPTH_FRONT_M)),
        tof_left_topic=to_str(get_param(values, "tof_left_topic", TOF_LEFT_M)),
        tof_right_topic=to_str(get_param(values, "tof_right_topic", TOF_RIGHT_M)),
        ultrasonic_front_topic=to_str(get_param(values, "ultrasonic_front_topic", ULTRASONIC_FRONT_M)),
        safety_stop_topic=to_str(get_param(values, "safety_stop_topic", SAFETY_STOP)),
        slowdown_topic=to_str(get_param(values, "slowdown_topic", SAFETY_SLOWDOWN_FACTOR)),
        safety_state_topic=to_str(get_param(values, "safety_state_topic", SAVO_PERCEPTION_SAFETY_STATE)),
        loop_hz=to_float(get_param(values, "loop_hz", SAFETY_LOOP_HZ_DEFAULT), min_value=1.0),
        stale_timeout_s=to_float(
            get_param(values, "stale_timeout_s", SENSOR_STALE_TIMEOUT_S_DEFAULT),
            min_value=0.01,
        ),
        front_stop_m=to_float(get_param(values, "front_stop_m", FRONT_STOP_M_DEFAULT), min_value=0.01),
        front_slow_m=to_float(get_param(values, "front_slow_m", FRONT_SLOW_M_DEFAULT), min_value=0.01),
        side_stop_m=to_float(get_param(values, "side_stop_m", SIDE_STOP_M_DEFAULT), min_value=0.01),
        side_slow_m=to_float(get_param(values, "side_slow_m", SIDE_SLOW_M_DEFAULT), min_value=0.01),
        front_clear_hysteresis_m=to_float(
            get_param(values, "front_clear_hysteresis_m", FRONT_CLEAR_HYSTERESIS_M_DEFAULT),
            min_value=0.0,
        ),
        side_clear_hysteresis_m=to_float(
            get_param(values, "side_clear_hysteresis_m", SIDE_CLEAR_HYSTERESIS_M_DEFAULT),
            min_value=0.0,
        ),
        stop_debounce_count=to_int(
            get_param(values, "stop_debounce_count", STOP_DEBOUNCE_COUNT_DEFAULT),
            min_value=1,
        ),
        clear_debounce_count=to_int(
            get_param(values, "clear_debounce_count", CLEAR_DEBOUNCE_COUNT_DEFAULT),
            min_value=1,
        ),
        slowdown_min=to_float(get_param(values, "slowdown_min", SLOWDOWN_MIN_DEFAULT), min_value=0.0),
        slowdown_max=to_float(get_param(values, "slowdown_max", SLOWDOWN_MAX_DEFAULT), min_value=0.0),
        slowdown_ema_alpha=to_float(
            get_param(values, "slowdown_ema_alpha", SLOWDOWN_EMA_ALPHA_DEFAULT),
            min_value=0.0,
        ),
        fail_safe_on_stale=to_bool(get_param(values, "fail_safe_on_stale", FAIL_SAFE_ON_STALE_DEFAULT)),
    )


def load_cmd_vel_safety_gate_params(values: Mapping[str, Any]) -> CmdVelSafetyGateParams:
    return CmdVelSafetyGateParams(
        cmd_vel_topic=to_str(get_param(values, "cmd_vel_topic", CMD_VEL)),
        cmd_vel_safe_topic=to_str(get_param(values, "cmd_vel_safe_topic", CMD_VEL_SAFE)),
        safety_stop_topic=to_str(get_param(values, "safety_stop_topic", SAFETY_STOP)),
        slowdown_topic=to_str(get_param(values, "slowdown_topic", SAFETY_SLOWDOWN_FACTOR)),
        stale_timeout_s=to_float(
            get_param(values, "stale_timeout_s", SENSOR_STALE_TIMEOUT_S_DEFAULT),
            min_value=0.01,
        ),
        fail_safe_on_stale=to_bool(get_param(values, "fail_safe_on_stale", FAIL_SAFE_ON_STALE_DEFAULT)),
    )


def load_range_health_params(values: Mapping[str, Any]) -> RangeHealthParams:
    return RangeHealthParams(
        depth_front_topic=to_str(get_param(values, "depth_front_topic", DEPTH_FRONT_M)),
        tof_left_topic=to_str(get_param(values, "tof_left_topic", TOF_LEFT_M)),
        tof_right_topic=to_str(get_param(values, "tof_right_topic", TOF_RIGHT_M)),
        ultrasonic_front_topic=to_str(get_param(values, "ultrasonic_front_topic", ULTRASONIC_FRONT_M)),
        range_health_topic=to_str(get_param(values, "range_health_topic", SAVO_PERCEPTION_RANGE_HEALTH)),
        publish_hz=to_float(get_param(values, "publish_hz", RANGE_HEALTH_HZ_DEFAULT), min_value=0.1),
        stale_timeout_s=to_float(
            get_param(values, "stale_timeout_s", SENSOR_STALE_TIMEOUT_S_DEFAULT),
            min_value=0.01,
        ),
    )


__all__ = [
    "Vl53MuxParams",
    "UltrasonicParams",
    "SafetyStopParams",
    "CmdVelSafetyGateParams",
    "RangeHealthParams",
    "get_param",
    "to_bool",
    "to_int",
    "to_float",
    "to_str",
    "declare_params",
    "read_node_params",
    "load_vl53_mux_params",
    "load_ultrasonic_params",
    "load_safety_stop_params",
    "load_cmd_vel_safety_gate_params",
    "load_range_health_params",
]