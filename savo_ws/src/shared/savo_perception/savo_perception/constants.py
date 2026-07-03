# -*- coding: utf-8 -*-

"""Package-wide constants. No ROS imports — safe to import anywhere."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


# =============================================================================
# Package / Identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_perception"
ROBOT_NAME: Final[str] = "Robot Savo"

NODE_NAME_VL53_MUX: Final[str] = "vl53_mux_node"
NODE_NAME_ULTRASONIC: Final[str] = "ultrasonic_node"
NODE_NAME_SAFETY_STOP: Final[str] = "safety_stop_node"
NODE_NAME_RANGE_HEALTH: Final[str] = "range_health_node"
NODE_NAME_CMD_VEL_SAFETY_GATE: Final[str] = "cmd_vel_safety_gate"
NODE_NAME_SENSOR_DASHBOARD: Final[str] = "sensor_dashboard_node"


# =============================================================================
# Topic Names (code defaults)
# =============================================================================
TOPIC_CMD_VEL: Final[str] = "/cmd_vel"
TOPIC_CMD_VEL_SAFE: Final[str] = "/cmd_vel_safe"

TOPIC_DEPTH_FRONT_M: Final[str] = "/depth/min_front_m"

TOPIC_TOF_LEFT_M: Final[str] = "/savo_perception/range/left_m"
TOPIC_TOF_RIGHT_M: Final[str] = "/savo_perception/range/right_m"
TOPIC_ULTRASONIC_FRONT_M: Final[str] = "/savo_perception/range/front_ultrasonic_m"

TOPIC_SAFETY_STOP: Final[str] = "/safety/stop"
TOPIC_SAFETY_SLOWDOWN_FACTOR: Final[str] = "/safety/slowdown_factor"

TOPIC_SAVO_PERCEPTION_RANGE_HEALTH: Final[str] = "/savo_perception/range_health"
TOPIC_SAVO_PERCEPTION_SAFETY_STATE: Final[str] = "/savo_perception/safety_state"
TOPIC_SAVO_PERCEPTION_DASHBOARD: Final[str] = "/savo_perception/dashboard"
TOPIC_SAVO_PERCEPTION_DASHBOARD_TEXT: Final[str] = "/savo_perception/dashboard_text"


# =============================================================================
# Range Sensor Defaults
# =============================================================================
RANGE_SENSOR_ORDER: Final[Tuple[str, str, str]] = ("left_tof", "right_tof", "front_ultrasonic")

I2C_BUS_DEFAULT: Final[int] = 1
TCA9548A_ADDR_DEFAULT: Final[int] = 0x70
VL53L1X_ADDR_DEFAULT: Final[int] = 0x29

VL53_LEFT_CHANNEL_DEFAULT: Final[int] = 3
VL53_RIGHT_CHANNEL_DEFAULT: Final[int] = 2

VL53_MEDIAN_WINDOW_DEFAULT: Final[int] = 5
VL53_VALID_MIN_M_DEFAULT: Final[float] = 0.02
VL53_VALID_MAX_M_DEFAULT: Final[float] = 4.00

ULTRASONIC_TRIG_PIN_DEFAULT: Final[int] = 27
ULTRASONIC_ECHO_PIN_DEFAULT: Final[int] = 22
ULTRASONIC_MAX_DISTANCE_M_DEFAULT: Final[float] = 3.00
ULTRASONIC_VALID_MIN_M_DEFAULT: Final[float] = 0.02
ULTRASONIC_VALID_MAX_M_DEFAULT: Final[float] = 3.00


# =============================================================================
# Runtime Rates / Timeouts
# =============================================================================
TOF_RATE_HZ_DEFAULT: Final[float] = 10.0
ULTRASONIC_RATE_HZ_DEFAULT: Final[float] = 10.0
SAFETY_LOOP_HZ_DEFAULT: Final[float] = 20.0
RANGE_HEALTH_HZ_DEFAULT: Final[float] = 2.0
DASHBOARD_HZ_DEFAULT: Final[float] = 2.0

SENSOR_STALE_TIMEOUT_S_DEFAULT: Final[float] = 0.30

TOF_RATE_HZ_MIN: Final[float] = 1.0
ULTRASONIC_RATE_HZ_MIN: Final[float] = 1.0
SAFETY_LOOP_HZ_MIN: Final[float] = 5.0
RANGE_HEALTH_HZ_MIN: Final[float] = 0.2


# =============================================================================
# Safety Threshold Defaults
# =============================================================================
FRONT_STOP_M_DEFAULT: Final[float] = 0.25
FRONT_SLOW_M_DEFAULT: Final[float] = 0.80

SIDE_STOP_M_DEFAULT: Final[float] = 0.08
SIDE_SLOW_M_DEFAULT: Final[float] = 0.25

FRONT_CLEAR_HYSTERESIS_M_DEFAULT: Final[float] = 0.010
SIDE_CLEAR_HYSTERESIS_M_DEFAULT: Final[float] = 0.010

STOP_DEBOUNCE_COUNT_DEFAULT: Final[int] = 2
CLEAR_DEBOUNCE_COUNT_DEFAULT: Final[int] = 4

FAIL_SAFE_ON_STALE_DEFAULT: Final[bool] = True


# =============================================================================
# Slowdown Defaults
# =============================================================================
SLOWDOWN_DEFAULT: Final[float] = 1.0
SLOWDOWN_MIN_DEFAULT: Final[float] = 0.20
SLOWDOWN_MAX_DEFAULT: Final[float] = 1.0
SLOWDOWN_EMA_ALPHA_DEFAULT: Final[float] = 0.35


# =============================================================================
# QoS Defaults
# =============================================================================
QOS_DEPTH_DEFAULT: Final[int] = 10
QOS_DEPTH_SENSOR_DEFAULT: Final[int] = 10
QOS_CMD_RELIABILITY_DEFAULT: Final[str] = "reliable"
QOS_SENSOR_RELIABILITY_DEFAULT: Final[str] = "best_effort"
QOS_STATE_RELIABILITY_DEFAULT: Final[str] = "reliable"


# =============================================================================
# CLI / Script Timing Defaults
# =============================================================================
CLI_SAMPLE_RATE_HZ_DEFAULT: Final[float] = 10.0
CLI_SAMPLE_DURATION_S_DEFAULT: Final[float] = 10.0
CLI_STATUS_TIMEOUT_S_DEFAULT: Final[float] = 2.0
CLI_INJECT_DURATION_S_DEFAULT: Final[float] = 1.0


# =============================================================================
# Safety / Diagnostics Labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_STALE: Final[str] = "STALE"
STATUS_SLOW: Final[str] = "SLOW"
STATUS_SAFETY_STOP: Final[str] = "SAFETY_STOP"
STATUS_ERROR: Final[str] = "ERROR"

WATCHDOG_NAME_RANGE_SENSORS: Final[str] = "range_sensor_watchdog"


# =============================================================================
# Structured defaults
# =============================================================================
@dataclass(frozen=True)
class PerceptionDefaults:
    node_name_vl53_mux: str = NODE_NAME_VL53_MUX
    node_name_ultrasonic: str = NODE_NAME_ULTRASONIC
    node_name_safety_stop: str = NODE_NAME_SAFETY_STOP
    node_name_range_health: str = NODE_NAME_RANGE_HEALTH
    node_name_cmd_vel_safety_gate: str = NODE_NAME_CMD_VEL_SAFETY_GATE
    robot_name: str = ROBOT_NAME

    cmd_vel_topic: str = TOPIC_CMD_VEL
    cmd_vel_safe_topic: str = TOPIC_CMD_VEL_SAFE
    depth_front_topic: str = TOPIC_DEPTH_FRONT_M
    tof_left_topic: str = TOPIC_TOF_LEFT_M
    tof_right_topic: str = TOPIC_TOF_RIGHT_M
    ultrasonic_front_topic: str = TOPIC_ULTRASONIC_FRONT_M
    safety_stop_topic: str = TOPIC_SAFETY_STOP
    slowdown_topic: str = TOPIC_SAFETY_SLOWDOWN_FACTOR
    range_health_topic: str = TOPIC_SAVO_PERCEPTION_RANGE_HEALTH
    safety_state_topic: str = TOPIC_SAVO_PERCEPTION_SAFETY_STATE

    i2c_bus: int = I2C_BUS_DEFAULT
    tca9548a_addr: int = TCA9548A_ADDR_DEFAULT
    vl53l1x_addr: int = VL53L1X_ADDR_DEFAULT
    vl53_left_channel: int = VL53_LEFT_CHANNEL_DEFAULT
    vl53_right_channel: int = VL53_RIGHT_CHANNEL_DEFAULT
    vl53_median_window: int = VL53_MEDIAN_WINDOW_DEFAULT

    ultrasonic_trig_pin: int = ULTRASONIC_TRIG_PIN_DEFAULT
    ultrasonic_echo_pin: int = ULTRASONIC_ECHO_PIN_DEFAULT
    ultrasonic_max_distance_m: float = ULTRASONIC_MAX_DISTANCE_M_DEFAULT

    tof_rate_hz: float = TOF_RATE_HZ_DEFAULT
    ultrasonic_rate_hz: float = ULTRASONIC_RATE_HZ_DEFAULT
    safety_loop_hz: float = SAFETY_LOOP_HZ_DEFAULT
    range_health_hz: float = RANGE_HEALTH_HZ_DEFAULT
    sensor_stale_timeout_s: float = SENSOR_STALE_TIMEOUT_S_DEFAULT

    front_stop_m: float = FRONT_STOP_M_DEFAULT
    front_slow_m: float = FRONT_SLOW_M_DEFAULT
    side_stop_m: float = SIDE_STOP_M_DEFAULT
    side_slow_m: float = SIDE_SLOW_M_DEFAULT
    front_clear_hysteresis_m: float = FRONT_CLEAR_HYSTERESIS_M_DEFAULT
    side_clear_hysteresis_m: float = SIDE_CLEAR_HYSTERESIS_M_DEFAULT
    stop_debounce_count: int = STOP_DEBOUNCE_COUNT_DEFAULT
    clear_debounce_count: int = CLEAR_DEBOUNCE_COUNT_DEFAULT
    fail_safe_on_stale: bool = FAIL_SAFE_ON_STALE_DEFAULT

    slowdown_default: float = SLOWDOWN_DEFAULT
    slowdown_min: float = SLOWDOWN_MIN_DEFAULT
    slowdown_max: float = SLOWDOWN_MAX_DEFAULT
    slowdown_ema_alpha: float = SLOWDOWN_EMA_ALPHA_DEFAULT

    def to_dict(self) -> dict:
        return {
            "node_names": {
                "vl53_mux": self.node_name_vl53_mux,
                "ultrasonic": self.node_name_ultrasonic,
                "safety_stop": self.node_name_safety_stop,
                "range_health": self.node_name_range_health,
                "cmd_vel_safety_gate": self.node_name_cmd_vel_safety_gate,
            },
            "robot_name": self.robot_name,
            "topics": {
                "cmd_vel_topic": self.cmd_vel_topic,
                "cmd_vel_safe_topic": self.cmd_vel_safe_topic,
                "depth_front_topic": self.depth_front_topic,
                "tof_left_topic": self.tof_left_topic,
                "tof_right_topic": self.tof_right_topic,
                "ultrasonic_front_topic": self.ultrasonic_front_topic,
                "safety_stop_topic": self.safety_stop_topic,
                "slowdown_topic": self.slowdown_topic,
                "range_health_topic": self.range_health_topic,
                "safety_state_topic": self.safety_state_topic,
            },
            "range_sensors": {
                "sensor_order": list(RANGE_SENSOR_ORDER),
                "i2c_bus": self.i2c_bus,
                "tca9548a_addr": f"0x{self.tca9548a_addr:02X}",
                "vl53l1x_addr": f"0x{self.vl53l1x_addr:02X}",
                "vl53_left_channel": self.vl53_left_channel,
                "vl53_right_channel": self.vl53_right_channel,
                "vl53_median_window": self.vl53_median_window,
                "ultrasonic_trig_pin": self.ultrasonic_trig_pin,
                "ultrasonic_echo_pin": self.ultrasonic_echo_pin,
                "ultrasonic_max_distance_m": self.ultrasonic_max_distance_m,
            },
            "rates_timeouts": {
                "tof_rate_hz": self.tof_rate_hz,
                "ultrasonic_rate_hz": self.ultrasonic_rate_hz,
                "safety_loop_hz": self.safety_loop_hz,
                "range_health_hz": self.range_health_hz,
                "sensor_stale_timeout_s": self.sensor_stale_timeout_s,
            },
            "safety": {
                "front_stop_m": self.front_stop_m,
                "front_slow_m": self.front_slow_m,
                "side_stop_m": self.side_stop_m,
                "side_slow_m": self.side_slow_m,
                "front_clear_hysteresis_m": self.front_clear_hysteresis_m,
                "side_clear_hysteresis_m": self.side_clear_hysteresis_m,
                "stop_debounce_count": self.stop_debounce_count,
                "clear_debounce_count": self.clear_debounce_count,
                "fail_safe_on_stale": self.fail_safe_on_stale,
            },
            "slowdown": {
                "default": self.slowdown_default,
                "min": self.slowdown_min,
                "max": self.slowdown_max,
                "ema_alpha": self.slowdown_ema_alpha,
            },
        }


DEFAULTS: Final[PerceptionDefaults] = PerceptionDefaults()


# =============================================================================
# Helpers
# =============================================================================
def get_perception_defaults() -> PerceptionDefaults:
    """Return immutable structured defaults for perception components."""
    return DEFAULTS


def clamp_slowdown(value: float) -> float:
    """Clamp slowdown factor to the valid safety range."""
    v = float(value)
    if v > SLOWDOWN_MAX_DEFAULT:
        return SLOWDOWN_MAX_DEFAULT
    if v < SLOWDOWN_MIN_DEFAULT:
        return SLOWDOWN_MIN_DEFAULT
    return v


__all__ = [
    # Identity
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "NODE_NAME_VL53_MUX",
    "NODE_NAME_ULTRASONIC",
    "NODE_NAME_SAFETY_STOP",
    "NODE_NAME_RANGE_HEALTH",
    "NODE_NAME_CMD_VEL_SAFETY_GATE",
    "NODE_NAME_SENSOR_DASHBOARD",
    # Topics
    "TOPIC_CMD_VEL",
    "TOPIC_CMD_VEL_SAFE",
    "TOPIC_DEPTH_FRONT_M",
    "TOPIC_TOF_LEFT_M",
    "TOPIC_TOF_RIGHT_M",
    "TOPIC_ULTRASONIC_FRONT_M",
    "TOPIC_SAFETY_STOP",
    "TOPIC_SAFETY_SLOWDOWN_FACTOR",
    "TOPIC_SAVO_PERCEPTION_RANGE_HEALTH",
    "TOPIC_SAVO_PERCEPTION_SAFETY_STATE",
    "TOPIC_SAVO_PERCEPTION_DASHBOARD",
    "TOPIC_SAVO_PERCEPTION_DASHBOARD_TEXT",
    # Range sensors
    "RANGE_SENSOR_ORDER",
    "I2C_BUS_DEFAULT",
    "TCA9548A_ADDR_DEFAULT",
    "VL53L1X_ADDR_DEFAULT",
    "VL53_LEFT_CHANNEL_DEFAULT",
    "VL53_RIGHT_CHANNEL_DEFAULT",
    "VL53_MEDIAN_WINDOW_DEFAULT",
    "VL53_VALID_MIN_M_DEFAULT",
    "VL53_VALID_MAX_M_DEFAULT",
    "ULTRASONIC_TRIG_PIN_DEFAULT",
    "ULTRASONIC_ECHO_PIN_DEFAULT",
    "ULTRASONIC_MAX_DISTANCE_M_DEFAULT",
    "ULTRASONIC_VALID_MIN_M_DEFAULT",
    "ULTRASONIC_VALID_MAX_M_DEFAULT",
    # Rates / timeouts
    "TOF_RATE_HZ_DEFAULT",
    "ULTRASONIC_RATE_HZ_DEFAULT",
    "SAFETY_LOOP_HZ_DEFAULT",
    "RANGE_HEALTH_HZ_DEFAULT",
    "DASHBOARD_HZ_DEFAULT",
    "SENSOR_STALE_TIMEOUT_S_DEFAULT",
    "TOF_RATE_HZ_MIN",
    "ULTRASONIC_RATE_HZ_MIN",
    "SAFETY_LOOP_HZ_MIN",
    "RANGE_HEALTH_HZ_MIN",
    # Safety
    "FRONT_STOP_M_DEFAULT",
    "FRONT_SLOW_M_DEFAULT",
    "SIDE_STOP_M_DEFAULT",
    "SIDE_SLOW_M_DEFAULT",
    "FRONT_CLEAR_HYSTERESIS_M_DEFAULT",
    "SIDE_CLEAR_HYSTERESIS_M_DEFAULT",
    "STOP_DEBOUNCE_COUNT_DEFAULT",
    "CLEAR_DEBOUNCE_COUNT_DEFAULT",
    "FAIL_SAFE_ON_STALE_DEFAULT",
    # Slowdown
    "SLOWDOWN_DEFAULT",
    "SLOWDOWN_MIN_DEFAULT",
    "SLOWDOWN_MAX_DEFAULT",
    "SLOWDOWN_EMA_ALPHA_DEFAULT",
    # QoS
    "QOS_DEPTH_DEFAULT",
    "QOS_DEPTH_SENSOR_DEFAULT",
    "QOS_CMD_RELIABILITY_DEFAULT",
    "QOS_SENSOR_RELIABILITY_DEFAULT",
    "QOS_STATE_RELIABILITY_DEFAULT",
    # CLI defaults
    "CLI_SAMPLE_RATE_HZ_DEFAULT",
    "CLI_SAMPLE_DURATION_S_DEFAULT",
    "CLI_STATUS_TIMEOUT_S_DEFAULT",
    "CLI_INJECT_DURATION_S_DEFAULT",
    # Status labels
    "STATUS_OK",
    "STATUS_STALE",
    "STATUS_SLOW",
    "STATUS_SAFETY_STOP",
    "STATUS_ERROR",
    "WATCHDOG_NAME_RANGE_SENSORS",
    # Structured defaults + helpers
    "PerceptionDefaults",
    "DEFAULTS",
    "get_perception_defaults",
    "clamp_slowdown",
]
