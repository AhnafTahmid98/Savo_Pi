#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/drivers/__init__.py
------------------------------------------
Professional public exports for the `savo_base.drivers` package (ROS 2 Jazzy).

Purpose
- Provide stable import paths for Robot Savo driver modules
- Re-export commonly used board/driver classes, factory helpers, and exceptions
- Support staged development (real hardware drivers + dry-run driver)

Examples
--------
from savo_base.drivers import create_board
from savo_base.drivers import FreenoveMecanumBoard, DryRunMotorBoard
from savo_base.drivers import PCA9685Driver
from savo_base.drivers import BoardException
"""

from __future__ import annotations

# =============================================================================
# Exceptions (always export)
# =============================================================================
from .board_exceptions import (
    BoardErrorContext,
    BoardException,
    BoardConfigError,
    BoardValidationError,
    UnsupportedBoardOperationError,
    BoardConnectionError,
    I2CBusOpenError,
    I2CCommunicationError,
    I2CDeviceNotFoundError,
    DevicePermissionError,
    DeviceBusyError,
    PCA9685Error,
    PCA9685InitError,
    PCA9685RegisterError,
    PCA9685FrequencyError,
    PCA9685ChannelError,
    PCA9685DutyCycleError,
    MotorBoardError,
    WheelMappingError,
    WheelCommandError,
    MotorSafetyError,
    EmergencyStopActiveError,
    wrap_i2c_error,
    wrap_pca9685_error,
    wrap_motor_board_error,
)

# =============================================================================
# Factory exports (always export)
# =============================================================================
from .board_factory import (
    BOARD_TYPE_PCA9685,
    BOARD_TYPE_FREENOVE_MECANUM,
    SUPPORTED_BOARD_TYPES,
    PCA9685FactoryConfig,
    FreenoveMecanumFactoryConfig,
    create_pca9685_driver,
    create_freenove_mecanum_board,
    create_board,
    create_robot_savo_default_mecanum_board,
    describe_supported_boards,
)

# =============================================================================
# Driver / board classes (best-effort imports for evolving codebase)
# =============================================================================
# We keep imports resilient because your package is being built file-by-file.
# If a module/class is not present yet, symbol remains None (package import still works).

PCA9685Driver = None
PCA9685 = None

FreenoveMecanumBoard = None
RobotSavoBoard = None
RobotSavo = None

DryRunMotorBoard = None
DryRunMecanumBoard = None
MockMotorBoard = None

# --- PCA9685 driver ---
try:
    from .pca9685_driver import PCA9685Driver as _PCA9685Driver  # type: ignore
    PCA9685Driver = _PCA9685Driver
except Exception:
    pass

try:
    from .pca9685_driver import PCA9685 as _PCA9685  # type: ignore
    PCA9685 = _PCA9685
except Exception:
    pass

# Alias fill for PCA9685 names
if PCA9685Driver is None and PCA9685 is not None:
    PCA9685Driver = PCA9685
if PCA9685 is None and PCA9685Driver is not None:
    PCA9685 = PCA9685Driver

# --- Freenove mecanum board ---
try:
    from .freenove_mecanum_board import FreenoveMecanumBoard as _FreenoveMecanumBoard  # type: ignore
    FreenoveMecanumBoard = _FreenoveMecanumBoard
except Exception:
    pass

try:
    from .freenove_mecanum_board import RobotSavoBoard as _RobotSavoBoard  # type: ignore
    RobotSavoBoard = _RobotSavoBoard
except Exception:
    pass

try:
    from .freenove_mecanum_board import RobotSavo as _RobotSavo  # type: ignore
    RobotSavo = _RobotSavo
except Exception:
    pass

# Alias fill for Freenove names
if FreenoveMecanumBoard is None and RobotSavoBoard is not None:
    FreenoveMecanumBoard = RobotSavoBoard
if FreenoveMecanumBoard is None and RobotSavo is not None:
    FreenoveMecanumBoard = RobotSavo

if RobotSavoBoard is None and FreenoveMecanumBoard is not None:
    RobotSavoBoard = FreenoveMecanumBoard
if RobotSavo is None and FreenoveMecanumBoard is not None:
    RobotSavo = FreenoveMecanumBoard

# --- Dry-run motor board ---
try:
    from .dryrun_motor_board import DryRunMotorBoard as _DryRunMotorBoard  # type: ignore
    DryRunMotorBoard = _DryRunMotorBoard
except Exception:
    pass

try:
    from .dryrun_motor_board import DryRunMecanumBoard as _DryRunMecanumBoard  # type: ignore
    DryRunMecanumBoard = _DryRunMecanumBoard
except Exception:
    pass

try:
    from .dryrun_motor_board import MockMotorBoard as _MockMotorBoard  # type: ignore
    MockMotorBoard = _MockMotorBoard
except Exception:
    pass

# Alias fill for dry-run names
if DryRunMecanumBoard is None and DryRunMotorBoard is not None:
    DryRunMecanumBoard = DryRunMotorBoard
if MockMotorBoard is None and DryRunMotorBoard is not None:
    MockMotorBoard = DryRunMotorBoard
if DryRunMotorBoard is None and DryRunMecanumBoard is not None:
    DryRunMotorBoard = DryRunMecanumBoard


# =============================================================================
# Package metadata helpers
# =============================================================================
def available_driver_symbols() -> dict:
    """
    Report which key driver classes are currently importable.
    Useful during staged bringup while files are added incrementally.
    """
    return {
        "PCA9685Driver": PCA9685Driver is not None,
        "PCA9685": PCA9685 is not None,
        "FreenoveMecanumBoard": FreenoveMecanumBoard is not None,
        "RobotSavoBoard": RobotSavoBoard is not None,
        "RobotSavo": RobotSavo is not None,
        "DryRunMotorBoard": DryRunMotorBoard is not None,
        "DryRunMecanumBoard": DryRunMecanumBoard is not None,
        "MockMotorBoard": MockMotorBoard is not None,
    }


def driver_package_summary() -> dict:
    """
    Compact summary of the `savo_base.drivers` package state.
    """
    return {
        "package": "savo_base.drivers",
        "supported_board_types": list(SUPPORTED_BOARD_TYPES),
        "available_symbols": available_driver_symbols(),
    }


# =============================================================================
# Public export surface
# =============================================================================
__all__ = [
    # package helpers
    "available_driver_symbols",
    "driver_package_summary",

    # class exports (resolved best-effort)
    "PCA9685Driver",
    "PCA9685",
    "FreenoveMecanumBoard",
    "RobotSavoBoard",
    "RobotSavo",
    "DryRunMotorBoard",
    "DryRunMecanumBoard",
    "MockMotorBoard",

    # board factory constants/configs/helpers
    "BOARD_TYPE_PCA9685",
    "BOARD_TYPE_FREENOVE_MECANUM",
    "SUPPORTED_BOARD_TYPES",
    "PCA9685FactoryConfig",
    "FreenoveMecanumFactoryConfig",
    "create_pca9685_driver",
    "create_freenove_mecanum_board",
    "create_board",
    "create_robot_savo_default_mecanum_board",
    "describe_supported_boards",

    # exception context + base
    "BoardErrorContext",
    "BoardException",

    # config/validation
    "BoardConfigError",
    "BoardValidationError",
    "UnsupportedBoardOperationError",

    # connection/I2C/device
    "BoardConnectionError",
    "I2CBusOpenError",
    "I2CCommunicationError",
    "I2CDeviceNotFoundError",
    "DevicePermissionError",
    "DeviceBusyError",

    # PCA9685-specific
    "PCA9685Error",
    "PCA9685InitError",
    "PCA9685RegisterError",
    "PCA9685FrequencyError",
    "PCA9685ChannelError",
    "PCA9685DutyCycleError",

    # motor board / safety
    "MotorBoardError",
    "WheelMappingError",
    "WheelCommandError",
    "MotorSafetyError",
    "EmergencyStopActiveError",

    # wrappers
    "wrap_i2c_error",
    "wrap_pca9685_error",
    "wrap_motor_board_error",
]