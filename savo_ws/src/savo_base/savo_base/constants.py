# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/constants.py
-----------------------------------
Centralized package-wide constants for `savo_base`.

Purpose
-------
Keep stable/default values in one place so nodes, scripts, and utilities use
the same names and defaults (topics, timing, PWM, safety thresholds, etc.).

Notes
-----
- This file is intentionally dependency-free (no ROS imports).
- These are *code defaults* only. Runtime ROS parameters/YAML should override
  them when needed.
- Place this file in the same package directory as `version.py`:
    Savo_Pi/src/savo_base/savo_base/constants.py
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


# =============================================================================
# Package / Identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_base"
NODE_NAME_BASE_DRIVER: Final[str] = "base_driver_node"
ROBOT_NAME: Final[str] = "Robot Savo"


# =============================================================================
# Topic Names (code defaults)
# =============================================================================
TOPIC_CMD_VEL: Final[str] = "/cmd_vel"
TOPIC_CMD_VEL_SAFE: Final[str] = "/cmd_vel_safe"

TOPIC_SAFETY_STOP: Final[str] = "/safety/stop"
TOPIC_SAFETY_SLOWDOWN_FACTOR: Final[str] = "/safety/slowdown_factor"

TOPIC_SAVO_BASE_WATCHDOG_STATE: Final[str] = "/savo_base/watchdog_state"
TOPIC_SAVO_BASE_BASE_STATE: Final[str] = "/savo_base/base_state"


# =============================================================================
# Base Driver Default Rates / Timeouts
# =============================================================================
BASE_LOOP_HZ_DEFAULT: Final[float] = 30.0
BASE_STATUS_PUBLISH_HZ_DEFAULT: Final[float] = 2.0
BASE_WATCHDOG_TIMEOUT_S_DEFAULT: Final[float] = 0.30

# Practical bounds used by params / validation helpers
BASE_LOOP_HZ_MIN: Final[float] = 5.0
BASE_STATUS_PUBLISH_HZ_MIN: Final[float] = 0.2
BASE_WATCHDOG_TIMEOUT_S_MIN: Final[float] = 0.05


# =============================================================================
# Command / Motion Limits (normalized command space defaults)
# =============================================================================
VX_LIMIT_DEFAULT: Final[float] = 1.0
VY_LIMIT_DEFAULT: Final[float] = 1.0
WZ_LIMIT_DEFAULT: Final[float] = 1.0

SLOWDOWN_DEFAULT: Final[float] = 1.0
SLOWDOWN_MIN_DEFAULT: Final[float] = 0.0
SLOWDOWN_MAX_DEFAULT: Final[float] = 1.0


# =============================================================================
# Mecanum Convention Defaults (matches your proven teleop baseline)
# =============================================================================
FORWARD_SIGN_DEFAULT: Final[int] = -1
STRAFE_SIGN_DEFAULT: Final[int] = +1
ROTATE_SIGN_DEFAULT: Final[int] = +1
TURN_GAIN_DEFAULT: Final[float] = 1.0

# Wheel order used consistently in Robot Savo code paths
WHEEL_ORDER: Final[Tuple[str, str, str, str]] = ("FL", "RL", "FR", "RR")


# =============================================================================
# Motor Board / PCA9685 Defaults (Freenove mecanum base)
# =============================================================================
BOARD_BACKEND_DEFAULT: Final[str] = "auto"  # auto | freenove | dryrun
BOARD_NAME_DEFAULT: Final[str] = "robot_savo_freenove_mecanum"

I2C_BUS_DEFAULT: Final[int] = 1
PCA9685_ADDR_DEFAULT: Final[int] = 0x40
PCA9685_PWM_FREQ_HZ_DEFAULT: Final[float] = 50.0

# Safe motor duty ceiling used in your teleop/base defaults
MAX_DUTY_DEFAULT: Final[int] = 3000
MAX_DUTY_ABSOLUTE: Final[int] = 4095

# Direction flip quench to protect H-bridge / avoid hard reversals
QUENCH_MS_DEFAULT: Final[int] = 18


# =============================================================================
# Per-wheel inversion defaults (software-level)
# =============================================================================
INVERT_FL_DEFAULT: Final[bool] = False
INVERT_RL_DEFAULT: Final[bool] = False
INVERT_FR_DEFAULT: Final[bool] = False
INVERT_RR_DEFAULT: Final[bool] = False


# =============================================================================
# QoS Defaults (informational constants for helper modules)
# =============================================================================
QOS_DEPTH_DEFAULT: Final[int] = 10
QOS_DEPTH_SENSOR_DEFAULT: Final[int] = 10
QOS_CMD_RELIABILITY_DEFAULT: Final[str] = "reliable"
QOS_SENSOR_RELIABILITY_DEFAULT: Final[str] = "best_effort"


# =============================================================================
# CLI / Script Timing Defaults (smoke test / poke / teleop helper scripts)
# =============================================================================
CLI_STEP_SLEEP_S_DEFAULT: Final[float] = 0.25
CLI_POKE_DURATION_S_DEFAULT: Final[float] = 0.60
CLI_RAMP_STEP_S_DEFAULT: Final[float] = 0.10
CLI_STATUS_TIMEOUT_S_DEFAULT: Final[float] = 2.0


# =============================================================================
# Safety / Diagnostics Labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_STALE: Final[str] = "STALE"
STATUS_SAFETY_STOP: Final[str] = "SAFETY_STOP"
STATUS_ERROR: Final[str] = "ERROR"

WATCHDOG_NAME_BASE_COMMAND: Final[str] = "base_command_watchdog"


# =============================================================================
# Structured defaults (optional convenience for importers)
# =============================================================================
@dataclass(frozen=True)
class BaseDriverDefaults:
    node_name: str = NODE_NAME_BASE_DRIVER
    robot_name: str = ROBOT_NAME

    cmd_topic: str = TOPIC_CMD_VEL_SAFE
    safety_stop_topic: str = TOPIC_SAFETY_STOP
    slowdown_topic: str = TOPIC_SAFETY_SLOWDOWN_FACTOR
    watchdog_state_topic: str = TOPIC_SAVO_BASE_WATCHDOG_STATE
    base_state_topic: str = TOPIC_SAVO_BASE_BASE_STATE

    loop_hz: float = BASE_LOOP_HZ_DEFAULT
    status_publish_hz: float = BASE_STATUS_PUBLISH_HZ_DEFAULT
    watchdog_timeout_s: float = BASE_WATCHDOG_TIMEOUT_S_DEFAULT

    vx_limit: float = VX_LIMIT_DEFAULT
    vy_limit: float = VY_LIMIT_DEFAULT
    wz_limit: float = WZ_LIMIT_DEFAULT

    slowdown_default: float = SLOWDOWN_DEFAULT
    slowdown_min: float = SLOWDOWN_MIN_DEFAULT
    slowdown_max: float = SLOWDOWN_MAX_DEFAULT

    max_duty: int = MAX_DUTY_DEFAULT
    turn_gain: float = TURN_GAIN_DEFAULT

    forward_sign: int = FORWARD_SIGN_DEFAULT
    strafe_sign: int = STRAFE_SIGN_DEFAULT
    rotate_sign: int = ROTATE_SIGN_DEFAULT

    board_backend: str = BOARD_BACKEND_DEFAULT
    board_name: str = BOARD_NAME_DEFAULT
    i2c_bus: int = I2C_BUS_DEFAULT
    pca9685_addr: int = PCA9685_ADDR_DEFAULT
    pwm_freq_hz: float = PCA9685_PWM_FREQ_HZ_DEFAULT
    quench_ms: int = QUENCH_MS_DEFAULT

    invert_fl: bool = INVERT_FL_DEFAULT
    invert_rl: bool = INVERT_RL_DEFAULT
    invert_fr: bool = INVERT_FR_DEFAULT
    invert_rr: bool = INVERT_RR_DEFAULT

    def to_dict(self) -> dict:
        return {
            "node_name": self.node_name,
            "robot_name": self.robot_name,
            "topics": {
                "cmd_topic": self.cmd_topic,
                "safety_stop_topic": self.safety_stop_topic,
                "slowdown_topic": self.slowdown_topic,
                "watchdog_state_topic": self.watchdog_state_topic,
                "base_state_topic": self.base_state_topic,
            },
            "rates_timeouts": {
                "loop_hz": self.loop_hz,
                "status_publish_hz": self.status_publish_hz,
                "watchdog_timeout_s": self.watchdog_timeout_s,
            },
            "limits": {
                "vx_limit": self.vx_limit,
                "vy_limit": self.vy_limit,
                "wz_limit": self.wz_limit,
                "max_duty": self.max_duty,
                "turn_gain": self.turn_gain,
            },
            "conventions": {
                "forward_sign": self.forward_sign,
                "strafe_sign": self.strafe_sign,
                "rotate_sign": self.rotate_sign,
                "wheel_order": list(WHEEL_ORDER),
            },
            "board": {
                "backend": self.board_backend,
                "name": self.board_name,
                "i2c_bus": self.i2c_bus,
                "pca9685_addr": f"0x{self.pca9685_addr:02X}",
                "pwm_freq_hz": self.pwm_freq_hz,
                "quench_ms": self.quench_ms,
            },
            "invert_flags": {
                "fl": self.invert_fl,
                "rl": self.invert_rl,
                "fr": self.invert_fr,
                "rr": self.invert_rr,
            },
        }


DEFAULTS: Final[BaseDriverDefaults] = BaseDriverDefaults()


# =============================================================================
# Helpers
# =============================================================================
def get_base_driver_defaults() -> BaseDriverDefaults:
    """Return immutable structured defaults for base driver components."""
    return DEFAULTS


def clamp_duty(value: int) -> int:
    """Clamp a PWM duty command to the valid signed PCA9685 command envelope."""
    v = int(value)
    if v > MAX_DUTY_ABSOLUTE:
        return MAX_DUTY_ABSOLUTE
    if v < -MAX_DUTY_ABSOLUTE:
        return -MAX_DUTY_ABSOLUTE
    return v


__all__ = [
    # Identity
    "PACKAGE_NAME",
    "NODE_NAME_BASE_DRIVER",
    "ROBOT_NAME",
    # Topics
    "TOPIC_CMD_VEL",
    "TOPIC_CMD_VEL_SAFE",
    "TOPIC_SAFETY_STOP",
    "TOPIC_SAFETY_SLOWDOWN_FACTOR",
    "TOPIC_SAVO_BASE_WATCHDOG_STATE",
    "TOPIC_SAVO_BASE_BASE_STATE",
    # Rates / timeouts
    "BASE_LOOP_HZ_DEFAULT",
    "BASE_STATUS_PUBLISH_HZ_DEFAULT",
    "BASE_WATCHDOG_TIMEOUT_S_DEFAULT",
    "BASE_LOOP_HZ_MIN",
    "BASE_STATUS_PUBLISH_HZ_MIN",
    "BASE_WATCHDOG_TIMEOUT_S_MIN",
    # Limits
    "VX_LIMIT_DEFAULT",
    "VY_LIMIT_DEFAULT",
    "WZ_LIMIT_DEFAULT",
    "SLOWDOWN_DEFAULT",
    "SLOWDOWN_MIN_DEFAULT",
    "SLOWDOWN_MAX_DEFAULT",
    # Kinematics conventions
    "FORWARD_SIGN_DEFAULT",
    "STRAFE_SIGN_DEFAULT",
    "ROTATE_SIGN_DEFAULT",
    "TURN_GAIN_DEFAULT",
    "WHEEL_ORDER",
    # Board defaults
    "BOARD_BACKEND_DEFAULT",
    "BOARD_NAME_DEFAULT",
    "I2C_BUS_DEFAULT",
    "PCA9685_ADDR_DEFAULT",
    "PCA9685_PWM_FREQ_HZ_DEFAULT",
    "MAX_DUTY_DEFAULT",
    "MAX_DUTY_ABSOLUTE",
    "QUENCH_MS_DEFAULT",
    # Inversion defaults
    "INVERT_FL_DEFAULT",
    "INVERT_RL_DEFAULT",
    "INVERT_FR_DEFAULT",
    "INVERT_RR_DEFAULT",
    # QoS info
    "QOS_DEPTH_DEFAULT",
    "QOS_DEPTH_SENSOR_DEFAULT",
    "QOS_CMD_RELIABILITY_DEFAULT",
    "QOS_SENSOR_RELIABILITY_DEFAULT",
    # CLI defaults
    "CLI_STEP_SLEEP_S_DEFAULT",
    "CLI_POKE_DURATION_S_DEFAULT",
    "CLI_RAMP_STEP_S_DEFAULT",
    "CLI_STATUS_TIMEOUT_S_DEFAULT",
    # Status labels
    "STATUS_OK",
    "STATUS_STALE",
    "STATUS_SAFETY_STOP",
    "STATUS_ERROR",
    "WATCHDOG_NAME_BASE_COMMAND",
    # Structured defaults + helpers
    "BaseDriverDefaults",
    "DEFAULTS",
    "get_base_driver_defaults",
    "clamp_duty",
]