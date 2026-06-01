#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/kinematics/__init__.py
---------------------------------------------
Professional public exports for the `savo_base.kinematics` package (ROS 2 Jazzy).

Purpose
- Provide stable import paths for Robot Savo kinematics helpers
- Re-export mecanum mixing, scaling utilities, and convention helpers
- Support staged development (some modules may be added later)

Wheel order convention (Robot Savo, locked)
- (FL, RL, FR, RR)

Examples
--------
from savo_base.kinematics import mix_mecanum
from savo_base.kinematics import signed_normalize_4, signed_clamp_4
from savo_base.kinematics import ControlAxes, default_robot_savo_conventions
"""

from __future__ import annotations

# =============================================================================
# Mecanum kinematics (best-effort import)
# =============================================================================
mix_mecanum = None
mix_mecanum_robot_savo = None
mecanum_inverse_kinematics = None

try:
    from .mecanum import mix_mecanum as _mix_mecanum  # type: ignore
    mix_mecanum = _mix_mecanum
except Exception:
    pass

# Optional aliases (if you defined them in mecanum.py)
try:
    from .mecanum import mix_mecanum_robot_savo as _mix_mecanum_robot_savo  # type: ignore
    mix_mecanum_robot_savo = _mix_mecanum_robot_savo
except Exception:
    pass

try:
    from .mecanum import mecanum_inverse_kinematics as _mecanum_inverse_kinematics  # type: ignore
    mecanum_inverse_kinematics = _mecanum_inverse_kinematics
except Exception:
    pass

# Fallback alias mapping if only one symbol exists
if mix_mecanum_robot_savo is None and mix_mecanum is not None:
    mix_mecanum_robot_savo = mix_mecanum
if mecanum_inverse_kinematics is None and mix_mecanum is not None:
    mecanum_inverse_kinematics = mix_mecanum


# =============================================================================
# Scaling utilities (best-effort import)
# =============================================================================
signed_clamp = None
signed_clamp_4 = None
signed_normalize_4 = None
to_duties = None
scale_normalized_wheels_to_duty = None

try:
    from .scaling import signed_clamp as _signed_clamp  # type: ignore
    signed_clamp = _signed_clamp
except Exception:
    pass

try:
    from .scaling import signed_clamp_4 as _signed_clamp_4  # type: ignore
    signed_clamp_4 = _signed_clamp_4
except Exception:
    pass

try:
    from .scaling import signed_normalize_4 as _signed_normalize_4  # type: ignore
    signed_normalize_4 = _signed_normalize_4
except Exception:
    pass

try:
    from .scaling import to_duties as _to_duties  # type: ignore
    to_duties = _to_duties
except Exception:
    pass

# Optional alias if you use a more descriptive name
try:
    from .scaling import scale_normalized_wheels_to_duty as _scale_normalized_wheels_to_duty  # type: ignore
    scale_normalized_wheels_to_duty = _scale_normalized_wheels_to_duty
except Exception:
    pass

if scale_normalized_wheels_to_duty is None and to_duties is not None:
    scale_normalized_wheels_to_duty = to_duties


# =============================================================================
# Conventions (best-effort import)
# =============================================================================
ControlAxes = None
WheelOrder = None
KinematicSigns = None

DEFAULT_WHEEL_ORDER = None
ROBOT_SAVO_WHEEL_ORDER = None

default_robot_savo_conventions = None
robot_savo_conventions = None
apply_control_signs = None
validate_wheel_order = None
qe_rotate_mapping_note = None

try:
    from .conventions import ControlAxes as _ControlAxes  # type: ignore
    ControlAxes = _ControlAxes
except Exception:
    pass

try:
    from .conventions import WheelOrder as _WheelOrder  # type: ignore
    WheelOrder = _WheelOrder
except Exception:
    pass

try:
    from .conventions import KinematicSigns as _KinematicSigns  # type: ignore
    KinematicSigns = _KinematicSigns
except Exception:
    pass

try:
    from .conventions import DEFAULT_WHEEL_ORDER as _DEFAULT_WHEEL_ORDER  # type: ignore
    DEFAULT_WHEEL_ORDER = _DEFAULT_WHEEL_ORDER
except Exception:
    pass

try:
    from .conventions import ROBOT_SAVO_WHEEL_ORDER as _ROBOT_SAVO_WHEEL_ORDER  # type: ignore
    ROBOT_SAVO_WHEEL_ORDER = _ROBOT_SAVO_WHEEL_ORDER
except Exception:
    pass

try:
    from .conventions import default_robot_savo_conventions as _default_robot_savo_conventions  # type: ignore
    default_robot_savo_conventions = _default_robot_savo_conventions
except Exception:
    pass

try:
    from .conventions import robot_savo_conventions as _robot_savo_conventions  # type: ignore
    robot_savo_conventions = _robot_savo_conventions
except Exception:
    pass

try:
    from .conventions import apply_control_signs as _apply_control_signs  # type: ignore
    apply_control_signs = _apply_control_signs
except Exception:
    pass

try:
    from .conventions import validate_wheel_order as _validate_wheel_order  # type: ignore
    validate_wheel_order = _validate_wheel_order
except Exception:
    pass

try:
    from .conventions import qe_rotate_mapping_note as _qe_rotate_mapping_note  # type: ignore
    qe_rotate_mapping_note = _qe_rotate_mapping_note
except Exception:
    pass

# Fallback aliases
if robot_savo_conventions is None and default_robot_savo_conventions is not None:
    robot_savo_conventions = default_robot_savo_conventions
if ROBOT_SAVO_WHEEL_ORDER is None and DEFAULT_WHEEL_ORDER is not None:
    ROBOT_SAVO_WHEEL_ORDER = DEFAULT_WHEEL_ORDER


# =============================================================================
# Package metadata helpers
# =============================================================================
def available_kinematics_symbols() -> dict:
    """
    Report which key kinematics symbols are currently importable.
    Useful during staged bringup while files are added incrementally.
    """
    return {
        # mecanum
        "mix_mecanum": mix_mecanum is not None,
        "mix_mecanum_robot_savo": mix_mecanum_robot_savo is not None,
        "mecanum_inverse_kinematics": mecanum_inverse_kinematics is not None,

        # scaling
        "signed_clamp": signed_clamp is not None,
        "signed_clamp_4": signed_clamp_4 is not None,
        "signed_normalize_4": signed_normalize_4 is not None,
        "to_duties": to_duties is not None,
        "scale_normalized_wheels_to_duty": scale_normalized_wheels_to_duty is not None,

        # conventions
        "ControlAxes": ControlAxes is not None,
        "WheelOrder": WheelOrder is not None,
        "KinematicSigns": KinematicSigns is not None,
        "DEFAULT_WHEEL_ORDER": DEFAULT_WHEEL_ORDER is not None,
        "ROBOT_SAVO_WHEEL_ORDER": ROBOT_SAVO_WHEEL_ORDER is not None,
        "default_robot_savo_conventions": default_robot_savo_conventions is not None,
        "robot_savo_conventions": robot_savo_conventions is not None,
        "apply_control_signs": apply_control_signs is not None,
        "validate_wheel_order": validate_wheel_order is not None,
        "qe_rotate_mapping_note": qe_rotate_mapping_note is not None,
    }


def kinematics_package_summary() -> dict:
    """
    Compact summary of the `savo_base.kinematics` package state.
    """
    return {
        "package": "savo_base.kinematics",
        "wheel_order_locked": ("FL", "RL", "FR", "RR"),
        "available_symbols": available_kinematics_symbols(),
    }


# =============================================================================
# Public export surface
# =============================================================================
__all__ = [
    # package helpers
    "available_kinematics_symbols",
    "kinematics_package_summary",

    # mecanum
    "mix_mecanum",
    "mix_mecanum_robot_savo",
    "mecanum_inverse_kinematics",

    # scaling
    "signed_clamp",
    "signed_clamp_4",
    "signed_normalize_4",
    "to_duties",
    "scale_normalized_wheels_to_duty",

    # conventions
    "ControlAxes",
    "WheelOrder",
    "KinematicSigns",
    "DEFAULT_WHEEL_ORDER",
    "ROBOT_SAVO_WHEEL_ORDER",
    "default_robot_savo_conventions",
    "robot_savo_conventions",
    "apply_control_signs",
    "validate_wheel_order",
    "qe_rotate_mapping_note",
]