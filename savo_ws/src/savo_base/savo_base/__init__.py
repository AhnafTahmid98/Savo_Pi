# -*- coding: utf-8 -*-
"""Package root. Exports version, constants, and top-level helpers. No ROS imports."""

from __future__ import annotations

try:
    from .version import (
        __version__,
        VERSION,
        get_version,
        get_package_version_info,
    )
except Exception:
    __version__ = "0.0.0"
    VERSION = __version__

    def get_version() -> str:
        return __version__

    def get_package_version_info() -> dict:
        return {
            "package": "savo_base",
            "version": __version__,
            "version_tuple": (0, 0, 0),
        }


try:
    from .constants import (
        DEFAULTS,
        BaseDriverDefaults,
        get_base_driver_defaults,
        # Identity
        PACKAGE_NAME,
        ROBOT_NAME,
        NODE_NAME_BASE_DRIVER,
        # Topics
        TOPIC_CMD_VEL,
        TOPIC_CMD_VEL_SAFE,
        TOPIC_SAFETY_STOP,
        TOPIC_SAFETY_SLOWDOWN_FACTOR,
        TOPIC_SAVO_BASE_WATCHDOG_STATE,
        TOPIC_SAVO_BASE_BASE_STATE,
        # Helpers
        clamp_duty,
    )
except Exception:
    DEFAULTS = None
    BaseDriverDefaults = None

    def get_base_driver_defaults():
        return None

    PACKAGE_NAME = "savo_base"
    ROBOT_NAME = "Robot Savo"
    NODE_NAME_BASE_DRIVER = "base_driver_node"

    TOPIC_CMD_VEL = "/cmd_vel"
    TOPIC_CMD_VEL_SAFE = "/cmd_vel_safe"
    TOPIC_SAFETY_STOP = "/safety/stop"
    TOPIC_SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor"
    TOPIC_SAVO_BASE_WATCHDOG_STATE = "/savo_base/watchdog_state"
    TOPIC_SAVO_BASE_BASE_STATE = "/savo_base/base_state"

    def clamp_duty(value: int) -> int:
        v = int(value)
        if v > 4095:
            return 4095
        if v < -4095:
            return -4095
        return v


def get_package_info() -> dict:
    """Lightweight package metadata — safe to call without ROS."""
    return {
        "package": PACKAGE_NAME,
        "robot_name": ROBOT_NAME,
        "version": get_version(),
        "node_defaults": {
            "base_driver": NODE_NAME_BASE_DRIVER,
        },
        "topics": {
            "cmd_vel": TOPIC_CMD_VEL,
            "cmd_vel_safe": TOPIC_CMD_VEL_SAFE,
            "safety_stop": TOPIC_SAFETY_STOP,
            "safety_slowdown_factor": TOPIC_SAFETY_SLOWDOWN_FACTOR,
            "savo_base_watchdog_state": TOPIC_SAVO_BASE_WATCHDOG_STATE,
            "savo_base_base_state": TOPIC_SAVO_BASE_BASE_STATE,
        },
        "has_structured_defaults": DEFAULTS is not None,
    }


__all__ = [
    # version
    "__version__",
    "VERSION",
    "get_version",
    "get_package_version_info",
    # constants/defaults
    "DEFAULTS",
    "BaseDriverDefaults",
    "get_base_driver_defaults",
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "NODE_NAME_BASE_DRIVER",
    "TOPIC_CMD_VEL",
    "TOPIC_CMD_VEL_SAFE",
    "TOPIC_SAFETY_STOP",
    "TOPIC_SAFETY_SLOWDOWN_FACTOR",
    "TOPIC_SAVO_BASE_WATCHDOG_STATE",
    "TOPIC_SAVO_BASE_BASE_STATE",
    "clamp_duty",
    # metadata helper
    "get_package_info",
]