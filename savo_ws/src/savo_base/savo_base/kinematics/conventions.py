#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/kinematics/conventions.py
------------------------------------------------
Professional kinematics conventions and typed constants for Robot Savo (ROS 2 Jazzy).

Purpose
- Centralize motion/kinematics conventions used across `savo_base`
- Keep CLI teleop, ROS nodes, and future autonomy code consistent
- Document Robot Savo's *locked* wheel order and default axis sign conventions

Why this file exists
- `mecanum.py` should focus on math (mixing)
- `scaling.py` should focus on scaling/smoothing
- `conventions.py` should define shared conventions/constants/types so every module
  uses the same vocabulary and ordering

Locked Robot Savo conventions (from real hardware testing)
- Wheel order for tuples/arrays everywhere:
    (FL, RL, FR, RR)
- Default teleop-compatible axis signs:
    forward_sign = -1
    strafe_sign  = +1
    rotate_sign  = +1
- CLI teleop key intent:
    W/S -> vx (+/-)
    A/D -> vy (-/+)
    Q/E -> wz (+/-)   [CCW/CW]
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Tuple


# =============================================================================
# Exceptions
# =============================================================================
class ConventionError(RuntimeError):
    """Base exception for convention helpers."""


class ConventionValueError(ConventionError):
    """Raised when invalid convention values are provided."""


# =============================================================================
# Core enums
# =============================================================================
class WheelName(str, Enum):
    """
    Canonical wheel names for Robot Savo.
    """
    FL = "FL"  # Front Left
    RL = "RL"  # Rear Left
    FR = "FR"  # Front Right
    RR = "RR"  # Rear Right


class AxisName(str, Enum):
    """
    Canonical body command axes for Robot Savo.
    """
    VX = "vx"  # forward/backward
    VY = "vy"  # strafe left/right
    WZ = "wz"  # yaw/rotation


class RotationDirection(str, Enum):
    """
    Human-readable rotation directions used in teleop/docs.
    """
    CCW = "CCW"
    CW = "CW"


# =============================================================================
# Locked tuple ordering / indices
# =============================================================================
# Global wheel tuple order used across kinematics and drivers.
WHEEL_ORDER: Tuple[str, str, str, str] = ("FL", "RL", "FR", "RR")

# Fast index lookup when working with arrays/lists
WHEEL_INDEX: Dict[str, int] = {
    "FL": 0,
    "RL": 1,
    "FR": 2,
    "RR": 3,
}

# Named constants for readability in code using tuples/lists
IDX_FL = 0
IDX_RL = 1
IDX_FR = 2
IDX_RR = 3

NUM_WHEELS = 4


# =============================================================================
# Axis sign conventions (Robot Savo locked defaults)
# =============================================================================
# These defaults match your proven teleop script behavior.
DEFAULT_FORWARD_SIGN = -1
DEFAULT_STRAFE_SIGN = +1
DEFAULT_ROTATE_SIGN = +1
DEFAULT_TURN_GAIN = 1.0

VALID_AXIS_SIGNS = (-1, +1)


@dataclass(frozen=True)
class AxisSignConvention:
    """
    Axis sign convention for body commands (vx, vy, wz).

    Defaults are locked to Robot Savo's proven teleop behavior.
    """
    forward_sign: int = DEFAULT_FORWARD_SIGN
    strafe_sign: int = DEFAULT_STRAFE_SIGN
    rotate_sign: int = DEFAULT_ROTATE_SIGN

    def validate(self) -> "AxisSignConvention":
        """
        Validate signs are each -1 or +1. Returns self if valid.
        """
        _validate_sign("forward_sign", self.forward_sign)
        _validate_sign("strafe_sign", self.strafe_sign)
        _validate_sign("rotate_sign", self.rotate_sign)
        return self


ROBOT_SAVO_AXIS_SIGNS = AxisSignConvention()


# =============================================================================
# Teleop key conventions (documented intent)
# =============================================================================
@dataclass(frozen=True)
class TeleopKeyConvention:
    """
    Human-readable teleop key mapping convention for letters-only CLI teleop.

    This is documentation + shared constants (not keyboard IO implementation).
    """
    forward_inc: str = "w"
    forward_dec: str = "s"
    strafe_dec: str = "a"
    strafe_inc: str = "d"
    rotate_ccw_inc: str = "q"
    rotate_cw_inc: str = "e"
    stop_keys: Tuple[str, str] = ("x", " ")
    scale_down: str = "z"
    scale_up: str = "c"
    reset: str = "r"
    quit_esc: str = "esc"


ROBOT_SAVO_TELEOP_KEYS = TeleopKeyConvention()


# =============================================================================
# Typed containers for shared tuple semantics
# =============================================================================
@dataclass(frozen=True)
class BodyCommand:
    """
    Body-frame command using Robot Savo naming convention.
    """
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class WheelTupleFloat:
    """
    Wheel tuple (FL, RL, FR, RR) using float values.

    Typical uses:
    - normalized wheel commands in [-1, +1]
    - wheel rates (rad/s), etc.
    """
    fl: float
    rl: float
    fr: float
    rr: float

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.fl, self.rl, self.fr, self.rr)

    def as_dict(self) -> Dict[str, float]:
        return {"FL": self.fl, "RL": self.rl, "FR": self.fr, "RR": self.rr}


@dataclass(frozen=True)
class WheelTupleInt:
    """
    Wheel tuple (FL, RL, FR, RR) using integer values.

    Typical uses:
    - signed PWM duties
    - encoder tick deltas
    """
    fl: int
    rl: int
    fr: int
    rr: int

    def as_tuple(self) -> Tuple[int, int, int, int]:
        return (self.fl, self.rl, self.fr, self.rr)

    def as_dict(self) -> Dict[str, int]:
        return {"FL": self.fl, "RL": self.rl, "FR": self.fr, "RR": self.rr}


# =============================================================================
# Validation / helper functions
# =============================================================================
def _validate_sign(name: str, value: int) -> int:
    v = int(value)
    if v not in VALID_AXIS_SIGNS:
        raise ConventionValueError(f"{name} must be -1 or +1, got {value}")
    return v


def validate_axis_signs(forward_sign: int, strafe_sign: int, rotate_sign: int) -> Tuple[int, int, int]:
    """
    Validate and return axis sign triplet.

    Returns:
        (forward_sign, strafe_sign, rotate_sign)
    """
    return (
        _validate_sign("forward_sign", forward_sign),
        _validate_sign("strafe_sign", strafe_sign),
        _validate_sign("rotate_sign", rotate_sign),
    )


def is_valid_wheel_name(name: str) -> bool:
    """
    Check if a string is one of the canonical Robot Savo wheel names.
    """
    return str(name).upper() in WHEEL_INDEX


def normalize_wheel_name(name: str) -> str:
    """
    Normalize a wheel name to canonical uppercase form (FL/RL/FR/RR).
    """
    n = str(name).upper()
    if n not in WHEEL_INDEX:
        raise ConventionValueError(f"Invalid wheel name {name!r}; expected one of {WHEEL_ORDER}")
    return n


def wheel_index(name: str) -> int:
    """
    Return canonical tuple index for a wheel name.

    Example:
        wheel_index("FR") -> 2
    """
    return WHEEL_INDEX[normalize_wheel_name(name)]


def wheel_order_tuple() -> Tuple[str, str, str, str]:
    """
    Return the canonical wheel tuple order.
    """
    return WHEEL_ORDER


def body_command_zero() -> BodyCommand:
    """
    Convenience helper for a zero body command.
    """
    return BodyCommand(0.0, 0.0, 0.0)


def wheel_tuple_float_zero() -> WheelTupleFloat:
    """
    Convenience helper for zero float wheel tuple in locked order (FL, RL, FR, RR).
    """
    return WheelTupleFloat(0.0, 0.0, 0.0, 0.0)


def wheel_tuple_int_zero() -> WheelTupleInt:
    """
    Convenience helper for zero int wheel tuple in locked order (FL, RL, FR, RR).
    """
    return WheelTupleInt(0, 0, 0, 0)


def rotation_from_wz(wz: float) -> RotationDirection | None:
    """
    Interpret a wz sign as a human-readable rotation direction for logs/UI.

    Returns:
        - RotationDirection.CCW if wz > 0
        - RotationDirection.CW  if wz < 0
        - None                  if wz == 0
    """
    if wz > 0:
        return RotationDirection.CCW
    if wz < 0:
        return RotationDirection.CW
    return None


def teleop_key_to_axis_delta(key: str, step: float) -> BodyCommand | None:
    """
    Convert a letters-only teleop key into a body-command delta (vx, vy, wz).

    This captures the *convention* (intent mapping), not keyboard reading.
    It mirrors your proven teleop behavior:
      w -> +vx, s -> -vx
      a -> -vy, d -> +vy
      q -> +wz (CCW), e -> -wz (CW)

    Returns:
        BodyCommand delta, or None if key is not a motion key.
    """
    k = str(key).lower()
    s = float(step)

    if k == ROBOT_SAVO_TELEOP_KEYS.forward_inc:
        return BodyCommand(vx=+s, vy=0.0, wz=0.0)
    if k == ROBOT_SAVO_TELEOP_KEYS.forward_dec:
        return BodyCommand(vx=-s, vy=0.0, wz=0.0)
    if k == ROBOT_SAVO_TELEOP_KEYS.strafe_dec:
        return BodyCommand(vx=0.0, vy=-s, wz=0.0)
    if k == ROBOT_SAVO_TELEOP_KEYS.strafe_inc:
        return BodyCommand(vx=0.0, vy=+s, wz=0.0)
    if k == ROBOT_SAVO_TELEOP_KEYS.rotate_ccw_inc:
        return BodyCommand(vx=0.0, vy=0.0, wz=+s)
    if k == ROBOT_SAVO_TELEOP_KEYS.rotate_cw_inc:
        return BodyCommand(vx=0.0, vy=0.0, wz=-s)

    return None


def is_stop_key(key: str) -> bool:
    """
    Check whether a key is a configured stop key for letters-only teleop.
    """
    k = str(key).lower()
    return k in ROBOT_SAVO_TELEOP_KEYS.stop_keys


def describe_robot_savo_kinematics_conventions() -> str:
    """
    Return a compact human-readable summary string for logs/docs.
    """
    return (
        "Robot Savo kinematics conventions: "
        f"wheel_order={WHEEL_ORDER}, "
        f"axis_signs=(forward={DEFAULT_FORWARD_SIGN}, strafe={DEFAULT_STRAFE_SIGN}, rotate={DEFAULT_ROTATE_SIGN}), "
        "teleop: W/S=vx, A/D=vy, Q/E=wz(CCW/CW)"
    )