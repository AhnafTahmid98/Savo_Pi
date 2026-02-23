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
- Provide backward-compatible `make_motor_board(...)` wrapper for legacy callers
  (e.g., base_driver_node.py) that pass old kwargs.

Examples
--------
from savo_base.drivers import create_board
from savo_base.drivers import FreenoveMecanumBoard, DryRunMotorBoard
from savo_base.drivers import PCA9685Driver
from savo_base.drivers import BoardException
from savo_base.drivers import make_motor_board
"""

from __future__ import annotations

from typing import Any, Dict

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
# Backward-compatible factory wrapper for legacy base_driver_node callers
# =============================================================================
def _to_int_addr(value: Any) -> int:
    """Accept 0x40 / '0x40' / '64'."""
    if isinstance(value, str):
        return int(value.strip(), 0)
    return int(value)


def _invert_bool_to_sign(v: Any) -> int:
    """
    Legacy base_driver_node often passes invert_* as booleans.
    Factory expects wheel_inverts tuple of +/-1.
    """
    return -1 if bool(v) else +1


def make_motor_board(**kwargs):
    """
    Backward-compatible wrapper expected by legacy `base_driver_node.py`.

    Converts legacy kwargs into:
        create_board(board_type=..., config={...})

    Supported legacy kwargs (best-effort)
    -------------------------------------
    backend, dryrun, name, board_name,
    i2c_bus, pca9685_addr, pwm_freq_hz, quench_ms, max_duty,
    invert_fl, invert_rl, invert_fr, invert_rr,
    debug, ...
    """
    k: Dict[str, Any] = dict(kwargs)

    # -------------------------------------------------------------------------
    # Detect backend / dryrun intent
    # -------------------------------------------------------------------------
    backend = str(k.pop("backend", "auto")).strip().lower()
    dryrun = bool(k.pop("dryrun", False))

    # Legacy metadata keys (not used by current create_board signature)
    # Keep removed to avoid unexpected keyword errors.
    _legacy_name = k.pop("name", None)
    _legacy_board_name = k.pop("board_name", None)
    _ = (_legacy_name, _legacy_board_name)  # intentionally unused

    # -------------------------------------------------------------------------
    # Common config mapping
    # -------------------------------------------------------------------------
    config: Dict[str, Any] = {}

    if "i2c_bus" in k:
        config["i2c_bus"] = int(k.pop("i2c_bus"))

    # Legacy base_driver uses pca9685_addr; factory expects address
    if "pca9685_addr" in k:
        config["address"] = _to_int_addr(k.pop("pca9685_addr"))
    elif "address" in k:
        config["address"] = _to_int_addr(k.pop("address"))

    if "pwm_freq_hz" in k:
        config["pwm_freq_hz"] = float(k.pop("pwm_freq_hz"))

    if "quench_ms" in k:
        config["quench_ms"] = int(k.pop("quench_ms"))

    if "max_duty" in k:
        config["max_duty"] = int(k.pop("max_duty"))

    if "debug" in k:
        config["debug"] = bool(k.pop("debug"))

    # Legacy invert flags -> factory wheel_inverts=(FL, RL, FR, RR)
    inv_keys = ("invert_fl", "invert_rl", "invert_fr", "invert_rr")
    if any(key in k for key in inv_keys):
        fl = _invert_bool_to_sign(k.pop("invert_fl", False))
        rl = _invert_bool_to_sign(k.pop("invert_rl", False))
        fr = _invert_bool_to_sign(k.pop("invert_fr", False))
        rr = _invert_bool_to_sign(k.pop("invert_rr", False))
        config["wheel_inverts"] = (fl, rl, fr, rr)

    # Any unrecognized kwargs are preserved in extra (future-safe)
    # This avoids breaking older callers while not polluting create_board signature.
    if k:
        config["extra"] = dict(k)

    # -------------------------------------------------------------------------
    # Choose construction path
    # -------------------------------------------------------------------------
    # If explicitly dryrun OR backend says dryrun/sim/mock, construct dryrun board directly
    if dryrun or backend in ("dryrun", "sim", "mock"):
        if DryRunMotorBoard is None:
            raise BoardConfigError(
                "DryRunMotorBoard is not importable, but dryrun backend was requested."
            )
        return DryRunMotorBoard(**config)

    # backend auto / freenove / real -> Freenove mecanum board via factory
    if backend in ("auto", "freenove", "freenove_mecanum", "real", "hardware", "hw"):
        return create_board(BOARD_TYPE_FREENOVE_MECANUM, config=config)

    # optional direct PCA path
    if backend in ("pca", "pca9685", BOARD_TYPE_PCA9685):
        return create_board(BOARD_TYPE_PCA9685, config=config)

    raise BoardConfigError(
        f"Unsupported backend in make_motor_board(): {backend!r}. "
        f"Expected auto/freenove/dryrun/pca9685."
    )


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
        "make_motor_board": True,
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

    # compatibility wrapper
    "make_motor_board",

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