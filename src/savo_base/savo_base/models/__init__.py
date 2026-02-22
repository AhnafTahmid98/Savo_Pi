#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/__init__.py
-----------------------------------------
Public exports for `savo_base.models`.

Purpose
- Provide clean, stable imports for the rest of the Robot Savo stack
- Centralize model exports (state, limits, watchdog, wheel commands, board status)

Usage examples
--------------
from savo_base.models import BaseLimits, BaseState, WatchdogState
from savo_base.models import WheelCommand, WheelNorm, WheelDuty
from savo_base.models import make_robot_savo_watchdog_state
"""

# ============================================================================
# Base limits / configuration models
# ============================================================================
from .base_limits import (
    BaseLimits,
    BaseLimitsValidationError,
    make_robot_savo_base_limits,
)

# ============================================================================
# Base runtime state models
# ============================================================================
from .base_state import (
    BaseState,
    BaseStateValidationError,
    make_robot_savo_base_state,
)

# ============================================================================
# Motor board status models
# ============================================================================
from .motor_board_status import (
    MotorBoardStatus,
    MotorBoardStatusValidationError,
    make_robot_savo_motor_board_status,
)

# ============================================================================
# Watchdog state models
# ============================================================================
from .watchdog_state import (
    WatchdogState,
    WatchdogStateValidationError,
    WatchdogConfigSnapshot,
    WatchdogCounters,
    make_robot_savo_watchdog_state,
)

# ============================================================================
# Wheel command models
# ============================================================================
from .wheel_command import (
    WheelCommand,
    WheelCommandValidationError,
    WheelCommandHeader,
    WheelNorm,
    WheelDuty,
    make_robot_savo_wheel_command,
    make_robot_savo_stop_wheel_command,
)

# ============================================================================
# Public API
# ============================================================================
__all__ = [
    # base_limits
    "BaseLimits",
    "BaseLimitsValidationError",
    "make_robot_savo_base_limits",

    # base_state
    "BaseState",
    "BaseStateValidationError",
    "make_robot_savo_base_state",

    # motor_board_status
    "MotorBoardStatus",
    "MotorBoardStatusValidationError",
    "make_robot_savo_motor_board_status",

    # watchdog_state
    "WatchdogState",
    "WatchdogStateValidationError",
    "WatchdogConfigSnapshot",
    "WatchdogCounters",
    "make_robot_savo_watchdog_state",

    # wheel_command
    "WheelCommand",
    "WheelCommandValidationError",
    "WheelCommandHeader",
    "WheelNorm",
    "WheelDuty",
    "make_robot_savo_wheel_command",
    "make_robot_savo_stop_wheel_command",
]