#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/kinematics/mecanum.py
--------------------------------------------
Professional mecanum kinematics helpers for Robot Savo (ROS 2 Jazzy).

Purpose
- Provide a shared, tested kinematics layer for:
    * CLI teleop tools (non-ROS diagnostics)
    * ROS 2 base driver nodes
    * future autonomous control pipelines
- Preserve Robot Savo's proven wheel mixing behavior from real hardware testing

Design notes
- This module is math/utility only (no ROS imports, no hardware imports)
- Output wheel order is always:
    (FL, RL, FR, RR)
- Sign conventions are configurable using axis sign parameters, matching your
  proven teleop workflow:
    forward_sign = -1 (default)
    strafe_sign  = +1 (default)
    rotate_sign  = +1 (default)

Robot Savo proven normalized mix (locked behavior)
- fl = vx - vy - w
- rl = vx + vy - w
- fr = vx + vy + w
- rr = vx - vy + w
where:
- vx: forward/backward command
- vy: left/right strafe command
- w : yaw/rotation command after rotate_sign * turn_gain
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


# =============================================================================
# Exceptions
# =============================================================================
class MecanumKinematicsError(RuntimeError):
    """Base exception for mecanum kinematics helpers."""


class MecanumValueError(MecanumKinematicsError):
    """Raised when invalid kinematics inputs are provided."""


# =============================================================================
# Config / typed containers
# =============================================================================
@dataclass(frozen=True)
class MecanumMixConfig:
    """
    Configuration for Robot Savo mecanum mixing.

    Defaults match your proven teleop script.
    """
    forward_sign: int = -1
    strafe_sign: int = +1
    rotate_sign: int = +1
    turn_gain: float = 1.0


@dataclass(frozen=True)
class WheelNormCommands:
    """
    Normalized wheel commands in range [-1.0 .. +1.0] after scaling.
    Wheel order is fixed: FL, RL, FR, RR.
    """
    fl: float
    rl: float
    fr: float
    rr: float


@dataclass(frozen=True)
class WheelDutyCommands:
    """
    Integer wheel PWM duties (signed), typically mapped to board wrapper input.
    Wheel order is fixed: FL, RL, FR, RR.
    """
    fl: int
    rl: int
    fr: int
    rr: int


# =============================================================================
# Validation helpers
# =============================================================================
def _validate_sign(name: str, value: int) -> int:
    v = int(value)
    if v not in (-1, +1):
        raise MecanumValueError(f"{name} must be -1 or +1, got {value}")
    return v


def _validate_float(name: str, value: float) -> float:
    try:
        v = float(value)
    except Exception as e:
        raise MecanumValueError(f"{name} must be a number, got {value!r}") from e
    return v


def _clamp_unit(v: float) -> float:
    if v > 1.0:
        return 1.0
    if v < -1.0:
        return -1.0
    return v


# =============================================================================
# Core Robot Savo mecanum mixing (proven behavior)
# =============================================================================
def mix_mecanum(
    vx: float,
    vy: float,
    wz: float,
    *,
    forward_sign: int = -1,
    strafe_sign: int = +1,
    rotate_sign: int = +1,
    turn_gain: float = 1.0,
) -> Tuple[float, float, float, float]:
    """
    Mix body commands (vx, vy, wz) into normalized wheel commands (FL, RL, FR, RR).

    Args:
        vx: Forward/backward command (recommended input range: [-1, +1])
        vy: Left/right strafe command (recommended input range: [-1, +1])
        wz: Rotation/yaw command (recommended input range: [-1, +1])

        forward_sign: Axis sign flip for forward/backward (+1 / -1)
        strafe_sign:  Axis sign flip for strafe (+1 / -1)
        rotate_sign:  Axis sign flip for rotation (+1 / -1)
        turn_gain:    Rotation gain multiplier (>= 0 recommended)

    Returns:
        (fl, rl, fr, rr) normalized to [-1.0 .. +1.0]

    Robot Savo locked formula:
        fl =  vx - vy - w
        rl =  vx + vy - w
        fr =  vx + vy + w
        rr =  vx - vy + w
        where w = rotate_sign * turn_gain * wz
        and vx/vy are sign-adjusted by forward_sign/strafe_sign

    Notes
    - Outputs are normalized by dividing all wheels by the largest absolute value
      if any magnitude exceeds 1.0.
    - This preserves vector direction and relative wheel ratios.
    """
    vx_f = _validate_float("vx", vx)
    vy_f = _validate_float("vy", vy)
    wz_f = _validate_float("wz", wz)

    f_sign = _validate_sign("forward_sign", forward_sign)
    s_sign = _validate_sign("strafe_sign", strafe_sign)
    r_sign = _validate_sign("rotate_sign", rotate_sign)
    tg = _validate_float("turn_gain", turn_gain)

    # Apply axis sign conventions (Robot Savo proven teleop compatibility)
    vx_f *= f_sign
    vy_f *= s_sign
    w = r_sign * tg * wz_f

    # Locked Robot Savo mix formula (FL, RL, FR, RR)
    fl = vx_f - vy_f - w
    rl = vx_f + vy_f - w
    fr = vx_f + vy_f + w
    rr = vx_f - vy_f + w

    # Normalize to [-1, +1] preserving ratios
    max_mag = max(1.0, abs(fl), abs(rl), abs(fr), abs(rr))
    fl /= max_mag
    rl /= max_mag
    fr /= max_mag
    rr /= max_mag

    return fl, rl, fr, rr


def mix_mecanum_cfg(
    vx: float,
    vy: float,
    wz: float,
    cfg: MecanumMixConfig,
) -> WheelNormCommands:
    """
    Typed-wrapper version of mix_mecanum() using MecanumMixConfig.
    """
    fl, rl, fr, rr = mix_mecanum(
        vx,
        vy,
        wz,
        forward_sign=cfg.forward_sign,
        strafe_sign=cfg.strafe_sign,
        rotate_sign=cfg.rotate_sign,
        turn_gain=cfg.turn_gain,
    )
    return WheelNormCommands(fl=fl, rl=rl, fr=fr, rr=rr)


# =============================================================================
# Conversion helpers (normalized -> signed duty)
# =============================================================================
def to_duties(
    nfl: float,
    nrl: float,
    nfr: float,
    nrr: float,
    max_duty: int,
) -> Tuple[int, int, int, int]:
    """
    Convert normalized wheel commands [-1..1] to signed integer PWM duties.

    Args:
        nfl, nrl, nfr, nrr:
            Normalized wheel commands (recommended range [-1.0 .. +1.0])
            Values outside range are clamped to [-1.0, +1.0].
        max_duty:
            Positive PWM magnitude ceiling (recommended 0..4095)

    Returns:
        (d_fl, d_rl, d_fr, d_rr) as signed integers

    Notes
    - Values are truncated toward zero using int(...), matching your proven teleop script.
    - Negative duties are preserved for board wrapper sign semantics.
    """
    md = int(max_duty)
    if md < 0:
        raise MecanumValueError(f"max_duty must be >= 0, got {max_duty}")
    if md > 4095:
        md = 4095  # practical PCA9685 clamp boundary

    fl = int(_clamp_unit(_validate_float("nfl", nfl)) * md)
    rl = int(_clamp_unit(_validate_float("nrl", nrl)) * md)
    fr = int(_clamp_unit(_validate_float("nfr", nfr)) * md)
    rr = int(_clamp_unit(_validate_float("nrr", nrr)) * md)

    return fl, rl, fr, rr


def to_duties_obj(norm: WheelNormCommands, max_duty: int) -> WheelDutyCommands:
    """
    Typed-wrapper version of to_duties().
    """
    fl, rl, fr, rr = to_duties(norm.fl, norm.rl, norm.fr, norm.rr, max_duty)
    return WheelDutyCommands(fl=fl, rl=rl, fr=fr, rr=rr)


# =============================================================================
# High-level convenience (body command -> signed duties)
# =============================================================================
def body_to_duties(
    vx: float,
    vy: float,
    wz: float,
    *,
    max_duty: int,
    forward_sign: int = -1,
    strafe_sign: int = +1,
    rotate_sign: int = +1,
    turn_gain: float = 1.0,
) -> Tuple[int, int, int, int]:
    """
    Convenience helper: mix body commands and convert directly to signed duties.

    Returns:
        (d_fl, d_rl, d_fr, d_rr)
    """
    fl, rl, fr, rr = mix_mecanum(
        vx,
        vy,
        wz,
        forward_sign=forward_sign,
        strafe_sign=strafe_sign,
        rotate_sign=rotate_sign,
        turn_gain=turn_gain,
    )
    return to_duties(fl, rl, fr, rr, max_duty=max_duty)


def body_to_duties_cfg(
    vx: float,
    vy: float,
    wz: float,
    *,
    max_duty: int,
    cfg: MecanumMixConfig,
) -> WheelDutyCommands:
    """
    Typed-wrapper version of body_to_duties() using MecanumMixConfig.
    """
    norm = mix_mecanum_cfg(vx, vy, wz, cfg)
    return to_duties_obj(norm, max_duty=max_duty)