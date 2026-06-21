#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package-wide constants. No ROS imports - safe to import anywhere."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple


# =============================================================================
# Package / Identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_control"
ROBOT_NAME: Final[str] = "Robot Savo"


# =============================================================================
# Node Names
# =============================================================================
NODE_NAME_TWIST_MUX: Final[str] = "twist_mux_node"
NODE_NAME_CMD_VEL_SHAPER: Final[str] = "cmd_vel_shaper_node"
NODE_NAME_CONTROL_MODE_MANAGER: Final[str] = "control_mode_manager_node"
NODE_NAME_HEADING_PID: Final[str] = "heading_pid_node"
NODE_NAME_ROTATE_TO_HEADING: Final[str] = "rotate_to_heading_node"
NODE_NAME_VELOCITY_TEST_PATTERN: Final[str] = "velocity_test_pattern_node"
NODE_NAME_RECOVERY_MANAGER: Final[str] = "recovery_manager_node"
NODE_NAME_BACKUP_ESCAPE: Final[str] = "backup_escape_node"
NODE_NAME_STUCK_DETECTOR: Final[str] = "stuck_detector_node"


# =============================================================================
# Topic Names
# Aligned with include/savo_control/topic_names.hpp
# =============================================================================
TOPIC_CMD_VEL_MANUAL: Final[str] = "/cmd_vel_manual"
TOPIC_CMD_VEL_AUTO: Final[str] = "/cmd_vel_auto"
TOPIC_CMD_VEL_NAV: Final[str] = "/cmd_vel_nav"
TOPIC_CMD_VEL_RECOVERY: Final[str] = "/cmd_vel_recovery"

TOPIC_CMD_VEL_MUX: Final[str] = "/cmd_vel_mux"
TOPIC_CMD_VEL: Final[str] = "/cmd_vel"
TOPIC_CMD_VEL_SAFE: Final[str] = "/cmd_vel_safe"
TOPIC_CMD_VEL_RAW: Final[str] = "/cmd_vel_raw"
TOPIC_CMD_VEL_TEST_PATTERN: Final[str] = "/cmd_vel_test_pattern"

TOPIC_WHEEL_ODOM: Final[str] = "/wheel/odom"
TOPIC_ODOM_FILTERED: Final[str] = "/odometry/filtered"
TOPIC_IMU_DATA: Final[str] = "/imu/data"
TOPIC_IMU_DATA_RAW: Final[str] = "/imu/data_raw"

TOPIC_SAFETY_STOP: Final[str] = "/safety/stop"
TOPIC_SAFETY_SLOWDOWN_FACTOR: Final[str] = "/safety/slowdown_factor"

TOPIC_DEPTH_MIN_FRONT: Final[str] = "/depth/min_front_m"
TOPIC_RANGE_FRONT_ULTRASONIC: Final[str] = "/savo_perception/range/front_ultrasonic_m"
TOPIC_RANGE_LEFT: Final[str] = "/savo_perception/range/left_m"
TOPIC_RANGE_RIGHT: Final[str] = "/savo_perception/range/right_m"

TOPIC_CONTROL_MODE_CMD: Final[str] = "/savo_control/mode_cmd"
TOPIC_CONTROL_MODE_STATE: Final[str] = "/savo_control/mode_state"

TOPIC_HEADING_TARGET: Final[str] = "/savo_control/heading_target"
TOPIC_HEADING_HOLD_ENABLE: Final[str] = "/savo_control/heading_hold_enable"
TOPIC_ROTATE_TARGET: Final[str] = "/savo_control/rotate_target"
TOPIC_ROTATE_STATE: Final[str] = "/savo_control/rotate_state"

TOPIC_AUTO_TEST_ENABLE: Final[str] = "/savo_control/auto_test_enable"
TOPIC_AUTO_TEST_STATE: Final[str] = "/savo_control/auto_test_state"
TOPIC_DISTANCE_TEST_TARGET: Final[str] = "/savo_control/distance_test_target"
TOPIC_DISTANCE_TEST_STATE: Final[str] = "/savo_control/distance_test_state"
TOPIC_STRAIGHT_TEST_ENABLE: Final[str] = "/savo_control/straight_test_enable"
TOPIC_STRAIGHT_TEST_STATE: Final[str] = "/savo_control/straight_test_state"

TOPIC_STUCK_DETECTED: Final[str] = "/savo_control/stuck_detected"
TOPIC_RECOVERY_REQUEST: Final[str] = "/savo_control/recovery_request"
TOPIC_RECOVERY_ACTIVE: Final[str] = "/savo_control/recovery_active"
TOPIC_RECOVERY_TRIGGER: Final[str] = "/savo_control/recovery_trigger"
TOPIC_RECOVERY_STATE: Final[str] = "/savo_control/recovery_state"
TOPIC_RECOVERY_STATUS: Final[str] = "/savo_control/recovery_status"

TOPIC_CONTROL_STATUS: Final[str] = "/savo_control/control_status"
TOPIC_CONTROL_DEBUG: Final[str] = "/savo_control/control_debug"


# =============================================================================
# Frames / Namespaces
# =============================================================================
FRAME_MAP: Final[str] = "map"
FRAME_ODOM: Final[str] = "odom"
FRAME_BASE_LINK: Final[str] = "base_link"
FRAME_BASE_FOOTPRINT: Final[str] = "base_footprint"
FRAME_IMU_LINK: Final[str] = "imu_link"
FRAME_LASER: Final[str] = "laser"
FRAME_CAMERA_LINK: Final[str] = "camera_link"

NAMESPACE_CONTROL: Final[str] = "/savo_control"
NAMESPACE_SAFETY: Final[str] = "/safety"


# =============================================================================
# Control Modes
# =============================================================================
MODE_STOP: Final[str] = "STOP"
MODE_MANUAL: Final[str] = "MANUAL"
MODE_AUTO: Final[str] = "AUTO"
MODE_NAV: Final[str] = "NAV"
MODE_RECOVERY: Final[str] = "RECOVERY"

CONTROL_MODES: Final[Tuple[str, str, str, str, str]] = (
    MODE_STOP,
    MODE_MANUAL,
    MODE_AUTO,
    MODE_NAV,
    MODE_RECOVERY,
)


# =============================================================================
# Default Rates / Timeouts
# =============================================================================
CONTROL_LOOP_HZ_DEFAULT: Final[float] = 30.0
CONTROL_STATUS_PUBLISH_HZ_DEFAULT: Final[float] = 2.0
CMD_INPUT_TIMEOUT_S_DEFAULT: Final[float] = 0.35
SAFETY_STOP_HOLD_S_DEFAULT: Final[float] = 0.10
SOURCE_TIMEOUT_S_DEFAULT: Final[float] = 0.35

CONTROL_LOOP_HZ_MIN: Final[float] = 5.0
CONTROL_STATUS_PUBLISH_HZ_MIN: Final[float] = 0.2
CMD_INPUT_TIMEOUT_S_MIN: Final[float] = 0.05


# =============================================================================
# Motion Limits
# =============================================================================
MAX_VX_DEFAULT: Final[float] = 0.25
MAX_VY_DEFAULT: Final[float] = 0.22
MAX_WZ_DEFAULT: Final[float] = 0.60

DEADBAND_DEFAULT: Final[float] = 0.005
MAX_VX_RISE_RATE_DEFAULT: Final[float] = 0.35
MAX_VX_FALL_RATE_DEFAULT: Final[float] = 0.55
MAX_VY_RISE_RATE_DEFAULT: Final[float] = 0.30
MAX_VY_FALL_RATE_DEFAULT: Final[float] = 0.45
MAX_WZ_RISE_RATE_DEFAULT: Final[float] = 0.90
MAX_WZ_FALL_RATE_DEFAULT: Final[float] = 1.20


# =============================================================================
# Recovery Defaults
# =============================================================================
RECOVERY_BACKUP_SPEED_MPS_DEFAULT: Final[float] = -0.10
RECOVERY_BACKUP_DURATION_S_DEFAULT: Final[float] = 0.80
RECOVERY_TURN_SPEED_RAD_S_DEFAULT: Final[float] = 0.35
RECOVERY_TURN_DURATION_S_DEFAULT: Final[float] = 1.00
RECOVERY_COOLDOWN_S_DEFAULT: Final[float] = 1.50


# =============================================================================
# Status / Diagnostics Labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_WARN: Final[str] = "WARN"
STATUS_ERROR: Final[str] = "ERROR"
STATUS_STALE: Final[str] = "STALE"
STATUS_SAFETY_STOP: Final[str] = "SAFETY_STOP"
STATUS_DISABLED: Final[str] = "DISABLED"
STATUS_UNKNOWN: Final[str] = "UNKNOWN"


# =============================================================================
# Structured defaults
# =============================================================================
@dataclass(frozen=True)
class ControlDefaults:
    node_name_twist_mux: str = NODE_NAME_TWIST_MUX
    node_name_cmd_vel_shaper: str = NODE_NAME_CMD_VEL_SHAPER
    node_name_control_mode_manager: str = NODE_NAME_CONTROL_MODE_MANAGER
    robot_name: str = ROBOT_NAME

    cmd_vel_manual_topic: str = TOPIC_CMD_VEL_MANUAL
    cmd_vel_auto_topic: str = TOPIC_CMD_VEL_AUTO
    cmd_vel_nav_topic: str = TOPIC_CMD_VEL_NAV
    cmd_vel_recovery_topic: str = TOPIC_CMD_VEL_RECOVERY
    cmd_vel_mux_topic: str = TOPIC_CMD_VEL_MUX
    cmd_vel_topic: str = TOPIC_CMD_VEL
    cmd_vel_safe_topic: str = TOPIC_CMD_VEL_SAFE
    safety_stop_topic: str = TOPIC_SAFETY_STOP
    slowdown_topic: str = TOPIC_SAFETY_SLOWDOWN_FACTOR
    mode_cmd_topic: str = TOPIC_CONTROL_MODE_CMD
    mode_state_topic: str = TOPIC_CONTROL_MODE_STATE
    control_status_topic: str = TOPIC_CONTROL_STATUS

    loop_hz: float = CONTROL_LOOP_HZ_DEFAULT
    status_publish_hz: float = CONTROL_STATUS_PUBLISH_HZ_DEFAULT
    cmd_input_timeout_s: float = CMD_INPUT_TIMEOUT_S_DEFAULT
    source_timeout_s: float = SOURCE_TIMEOUT_S_DEFAULT

    max_vx: float = MAX_VX_DEFAULT
    max_vy: float = MAX_VY_DEFAULT
    max_wz: float = MAX_WZ_DEFAULT
    deadband: float = DEADBAND_DEFAULT

    default_mode: str = MODE_STOP
    recovery_backup_speed_mps: float = RECOVERY_BACKUP_SPEED_MPS_DEFAULT
    recovery_backup_duration_s: float = RECOVERY_BACKUP_DURATION_S_DEFAULT
    recovery_turn_speed_rad_s: float = RECOVERY_TURN_SPEED_RAD_S_DEFAULT
    recovery_turn_duration_s: float = RECOVERY_TURN_DURATION_S_DEFAULT
    recovery_cooldown_s: float = RECOVERY_COOLDOWN_S_DEFAULT

    def to_dict(self) -> dict:
        return {
            "robot_name": self.robot_name,
            "nodes": {
                "twist_mux": self.node_name_twist_mux,
                "cmd_vel_shaper": self.node_name_cmd_vel_shaper,
                "control_mode_manager": self.node_name_control_mode_manager,
            },
            "topics": {
                "cmd_vel_manual": self.cmd_vel_manual_topic,
                "cmd_vel_auto": self.cmd_vel_auto_topic,
                "cmd_vel_nav": self.cmd_vel_nav_topic,
                "cmd_vel_recovery": self.cmd_vel_recovery_topic,
                "cmd_vel_mux": self.cmd_vel_mux_topic,
                "cmd_vel": self.cmd_vel_topic,
                "cmd_vel_safe": self.cmd_vel_safe_topic,
                "safety_stop": self.safety_stop_topic,
                "slowdown": self.slowdown_topic,
                "mode_cmd": self.mode_cmd_topic,
                "mode_state": self.mode_state_topic,
                "control_status": self.control_status_topic,
            },
            "rates_timeouts": {
                "loop_hz": self.loop_hz,
                "status_publish_hz": self.status_publish_hz,
                "cmd_input_timeout_s": self.cmd_input_timeout_s,
                "source_timeout_s": self.source_timeout_s,
            },
            "limits": {
                "max_vx": self.max_vx,
                "max_vy": self.max_vy,
                "max_wz": self.max_wz,
                "deadband": self.deadband,
            },
            "modes": {
                "default": self.default_mode,
                "allowed": list(CONTROL_MODES),
            },
            "recovery": {
                "backup_speed_mps": self.recovery_backup_speed_mps,
                "backup_duration_s": self.recovery_backup_duration_s,
                "turn_speed_rad_s": self.recovery_turn_speed_rad_s,
                "turn_duration_s": self.recovery_turn_duration_s,
                "cooldown_s": self.recovery_cooldown_s,
            },
        }


DEFAULTS: Final[ControlDefaults] = ControlDefaults()


# =============================================================================
# Helpers
# =============================================================================
def get_defaults_dict() -> dict:
    """Return package defaults as a dictionary."""
    return DEFAULTS.to_dict()


__all__ = [
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "DEFAULTS",
    "ControlDefaults",
    "get_defaults_dict",
]
