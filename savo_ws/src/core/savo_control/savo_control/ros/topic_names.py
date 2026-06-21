# -*- coding: utf-8 -*-

"""Python topic contract for savo_control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from savo_control import constants as c


CMD_VEL_MANUAL: Final[str] = c.TOPIC_CMD_VEL_MANUAL
CMD_VEL_AUTO: Final[str] = c.TOPIC_CMD_VEL_AUTO
CMD_VEL_NAV: Final[str] = c.TOPIC_CMD_VEL_NAV
CMD_VEL_RECOVERY: Final[str] = c.TOPIC_CMD_VEL_RECOVERY

CMD_VEL_MUX: Final[str] = c.TOPIC_CMD_VEL_MUX
CMD_VEL: Final[str] = c.TOPIC_CMD_VEL
CMD_VEL_SAFE: Final[str] = c.TOPIC_CMD_VEL_SAFE
CMD_VEL_RAW: Final[str] = c.TOPIC_CMD_VEL_RAW
CMD_VEL_TEST_PATTERN: Final[str] = c.TOPIC_CMD_VEL_TEST_PATTERN

WHEEL_ODOM: Final[str] = c.TOPIC_WHEEL_ODOM
ODOM_FILTERED: Final[str] = c.TOPIC_ODOM_FILTERED
IMU_DATA: Final[str] = c.TOPIC_IMU_DATA
IMU_DATA_RAW: Final[str] = c.TOPIC_IMU_DATA_RAW

SAFETY_STOP: Final[str] = c.TOPIC_SAFETY_STOP
SAFETY_SLOWDOWN_FACTOR: Final[str] = c.TOPIC_SAFETY_SLOWDOWN_FACTOR

DEPTH_MIN_FRONT: Final[str] = c.TOPIC_DEPTH_MIN_FRONT
RANGE_FRONT_ULTRASONIC: Final[str] = c.TOPIC_RANGE_FRONT_ULTRASONIC
RANGE_LEFT: Final[str] = c.TOPIC_RANGE_LEFT
RANGE_RIGHT: Final[str] = c.TOPIC_RANGE_RIGHT

CONTROL_MODE_CMD: Final[str] = c.TOPIC_CONTROL_MODE_CMD
CONTROL_MODE_STATE: Final[str] = c.TOPIC_CONTROL_MODE_STATE

HEADING_TARGET: Final[str] = c.TOPIC_HEADING_TARGET
HEADING_HOLD_ENABLE: Final[str] = c.TOPIC_HEADING_HOLD_ENABLE
ROTATE_TARGET: Final[str] = c.TOPIC_ROTATE_TARGET
ROTATE_STATE: Final[str] = c.TOPIC_ROTATE_STATE

AUTO_TEST_ENABLE: Final[str] = c.TOPIC_AUTO_TEST_ENABLE
AUTO_TEST_STATE: Final[str] = c.TOPIC_AUTO_TEST_STATE
DISTANCE_TEST_TARGET: Final[str] = c.TOPIC_DISTANCE_TEST_TARGET
DISTANCE_TEST_STATE: Final[str] = c.TOPIC_DISTANCE_TEST_STATE
STRAIGHT_TEST_ENABLE: Final[str] = c.TOPIC_STRAIGHT_TEST_ENABLE
STRAIGHT_TEST_STATE: Final[str] = c.TOPIC_STRAIGHT_TEST_STATE

STUCK_DETECTED: Final[str] = c.TOPIC_STUCK_DETECTED
RECOVERY_REQUEST: Final[str] = c.TOPIC_RECOVERY_REQUEST
RECOVERY_ACTIVE: Final[str] = c.TOPIC_RECOVERY_ACTIVE
RECOVERY_TRIGGER: Final[str] = c.TOPIC_RECOVERY_TRIGGER
RECOVERY_STATE: Final[str] = c.TOPIC_RECOVERY_STATE
RECOVERY_STATUS: Final[str] = c.TOPIC_RECOVERY_STATUS

CONTROL_STATUS: Final[str] = c.TOPIC_CONTROL_STATUS
CONTROL_DEBUG: Final[str] = c.TOPIC_CONTROL_DEBUG


@dataclass(frozen=True)
class ControlTopics:
    cmd_vel_manual: str = CMD_VEL_MANUAL
    cmd_vel_auto: str = CMD_VEL_AUTO
    cmd_vel_nav: str = CMD_VEL_NAV
    cmd_vel_recovery: str = CMD_VEL_RECOVERY

    cmd_vel_mux: str = CMD_VEL_MUX
    cmd_vel: str = CMD_VEL
    cmd_vel_safe: str = CMD_VEL_SAFE
    cmd_vel_raw: str = CMD_VEL_RAW

    odom_filtered: str = ODOM_FILTERED
    wheel_odom: str = WHEEL_ODOM
    imu_data: str = IMU_DATA

    safety_stop: str = SAFETY_STOP
    slowdown_factor: str = SAFETY_SLOWDOWN_FACTOR

    depth_min_front: str = DEPTH_MIN_FRONT
    range_front_ultrasonic: str = RANGE_FRONT_ULTRASONIC
    range_left: str = RANGE_LEFT
    range_right: str = RANGE_RIGHT

    mode_cmd: str = CONTROL_MODE_CMD
    mode_state: str = CONTROL_MODE_STATE

    recovery_request: str = RECOVERY_REQUEST
    recovery_active: str = RECOVERY_ACTIVE
    recovery_state: str = RECOVERY_STATE
    recovery_status: str = RECOVERY_STATUS

    control_status: str = CONTROL_STATUS

    def to_dict(self) -> dict:
        return {
            "cmd_vel_manual": self.cmd_vel_manual,
            "cmd_vel_auto": self.cmd_vel_auto,
            "cmd_vel_nav": self.cmd_vel_nav,
            "cmd_vel_recovery": self.cmd_vel_recovery,
            "cmd_vel_mux": self.cmd_vel_mux,
            "cmd_vel": self.cmd_vel,
            "cmd_vel_safe": self.cmd_vel_safe,
            "cmd_vel_raw": self.cmd_vel_raw,
            "odom_filtered": self.odom_filtered,
            "wheel_odom": self.wheel_odom,
            "imu_data": self.imu_data,
            "safety_stop": self.safety_stop,
            "slowdown_factor": self.slowdown_factor,
            "depth_min_front": self.depth_min_front,
            "range_front_ultrasonic": self.range_front_ultrasonic,
            "range_left": self.range_left,
            "range_right": self.range_right,
            "mode_cmd": self.mode_cmd,
            "mode_state": self.mode_state,
            "recovery_request": self.recovery_request,
            "recovery_active": self.recovery_active,
            "recovery_state": self.recovery_state,
            "recovery_status": self.recovery_status,
            "control_status": self.control_status,
        }


TOPICS: Final[ControlTopics] = ControlTopics()


def get_control_topics() -> ControlTopics:
    return TOPICS


def get_control_topics_dict() -> dict:
    return TOPICS.to_dict()


def is_control_output_topic(topic: str) -> bool:
    return topic in {
        CMD_VEL_MUX,
        CMD_VEL,
        CMD_VEL_AUTO,
        CMD_VEL_NAV,
        CMD_VEL_MANUAL,
        CMD_VEL_RECOVERY,
    }


def outputs_directly_to_base(topic: str) -> bool:
    return topic == CMD_VEL_SAFE


__all__ = [
    "AUTO_TEST_ENABLE",
    "AUTO_TEST_STATE",
    "CMD_VEL",
    "CMD_VEL_AUTO",
    "CMD_VEL_MANUAL",
    "CMD_VEL_MUX",
    "CMD_VEL_NAV",
    "CMD_VEL_RAW",
    "CMD_VEL_RECOVERY",
    "CMD_VEL_SAFE",
    "CMD_VEL_TEST_PATTERN",
    "CONTROL_DEBUG",
    "CONTROL_MODE_CMD",
    "CONTROL_MODE_STATE",
    "CONTROL_STATUS",
    "DEPTH_MIN_FRONT",
    "DISTANCE_TEST_STATE",
    "DISTANCE_TEST_TARGET",
    "HEADING_HOLD_ENABLE",
    "HEADING_TARGET",
    "IMU_DATA",
    "IMU_DATA_RAW",
    "ODOM_FILTERED",
    "RANGE_FRONT_ULTRASONIC",
    "RANGE_LEFT",
    "RANGE_RIGHT",
    "RECOVERY_ACTIVE",
    "RECOVERY_REQUEST",
    "RECOVERY_STATE",
    "RECOVERY_STATUS",
    "RECOVERY_TRIGGER",
    "ROTATE_STATE",
    "ROTATE_TARGET",
    "SAFETY_SLOWDOWN_FACTOR",
    "SAFETY_STOP",
    "STRAIGHT_TEST_ENABLE",
    "STRAIGHT_TEST_STATE",
    "STUCK_DETECTED",
    "TOPICS",
    "WHEEL_ODOM",
    "ControlTopics",
    "get_control_topics",
    "get_control_topics_dict",
    "is_control_output_topic",
    "outputs_directly_to_base",
]
