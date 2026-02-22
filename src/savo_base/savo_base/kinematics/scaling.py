#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/kinematics/scaling.py
--------------------------------------------
Professional command scaling helpers for Robot Savo (ROS 2 Jazzy).

Purpose
- Provide reusable, hardware-safe scaling utilities for base motion commands
- Keep teleop / ROS node behavior consistent across:
    * CLI teleop tools
    * base driver nodes
    * future autonomy pipelines
- Preserve the feel of your proven real-robot teleop workflow (max duty scaling,
  optional decay-style smoothing, clamping, deadband handling)

Design notes
- Math/utility only (no ROS imports, no hardware imports)
- These helpers do NOT perform mecanum mixing (that belongs in mecanum.py)
- These helpers are intended to be used before or after mixing depending on need:
    * body-command scaling (vx, vy, wz)
    * duty scaling (max_duty adjustments)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


# =============================================================================
# Exceptions
# =============================================================================
class ScalingError(RuntimeError):
    """Base exception for scaling helpers."""


class ScalingValueError(ScalingError):
    """Raised when invalid scaling inputs are provided."""


# =============================================================================
# Typed config / state
# =============================================================================
@dataclass(frozen=True)
class AxisLimits:
    """
    Per-axis command limits for normalized body commands.

    Recommended normalized usage:
    - vx, vy, wz in [-1.0 .. +1.0]
    """
    vx_max: float = 1.0
    vy_max: float = 1.0
    wz_max: float = 1.0


@dataclass(frozen=True)
class DeadbandConfig:
    """
    Deadband parameters to suppress small command noise.
    """
    vx_deadband: float = 0.0
    vy_deadband: float = 0.0
    wz_deadband: float = 0.0


@dataclass(frozen=True)
class SmoothingConfig:
    """
    Exponential decay-style smoothing config.

    decay:
      0..1; higher = more "coast", lower = faster fade to zero
      (matches the style used in your proven teleop script)

    hz:
      nominal loop rate used to compute the dt-based decay exponent.
    """
    decay: float = 0.85
    hz: float = 30.0


@dataclass
class BodyCommandState:
    """
    Mutable body command state for teleop-like incremental control.
    """
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


# =============================================================================
# Validation helpers
# =============================================================================
def _as_float(name: str, value: float) -> float:
    try:
        return float(value)
    except Exception as e:
        raise ScalingValueError(f"{name} must be numeric, got {value!r}") from e


def _clamp(value: float, lo: float, hi: float) -> float:
    if lo > hi:
        raise ScalingValueError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _clamp01(value: float, name: str) -> float:
    v = _as_float(name, value)
    if v < 0.0 or v > 1.0:
        raise ScalingValueError(f"{name} must be in [0,1], got {value}")
    return v


# =============================================================================
# Basic normalized scaling helpers
# =============================================================================
def clamp_unit(value: float) -> float:
    """
    Clamp a normalized command to [-1.0, +1.0].
    """
    v = _as_float("value", value)
    return _clamp(v, -1.0, 1.0)


def clamp_body_unit(vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
    """
    Clamp normalized body commands individually to [-1.0, +1.0].
    """
    return clamp_unit(vx), clamp_unit(vy), clamp_unit(wz)


def apply_axis_limits(
    vx: float,
    vy: float,
    wz: float,
    *,
    limits: AxisLimits,
) -> Tuple[float, float, float]:
    """
    Apply per-axis limits to normalized body commands.

    Example:
    - Keep full forward speed, but reduce strafe and rotation aggressiveness:
      AxisLimits(vx_max=1.0, vy_max=0.6, wz_max=0.5)
    """
    vx_f = _as_float("vx", vx)
    vy_f = _as_float("vy", vy)
    wz_f = _as_float("wz", wz)

    vx_max = abs(_as_float("limits.vx_max", limits.vx_max))
    vy_max = abs(_as_float("limits.vy_max", limits.vy_max))
    wz_max = abs(_as_float("limits.wz_max", limits.wz_max))

    if vx_max > 1.0:
        vx_max = 1.0
    if vy_max > 1.0:
        vy_max = 1.0
    if wz_max > 1.0:
        wz_max = 1.0

    return (
        _clamp(vx_f, -vx_max, +vx_max),
        _clamp(vy_f, -vy_max, +vy_max),
        _clamp(wz_f, -wz_max, +wz_max),
    )


def apply_global_scale(
    vx: float,
    vy: float,
    wz: float,
    *,
    scale: float,
) -> Tuple[float, float, float]:
    """
    Scale all body commands by a single factor.

    Typical uses:
    - "slow mode" in teleop
    - dynamic speed reduction from safety logic (global slowdown factor)
    """
    s = _as_float("scale", scale)
    return (
        _as_float("vx", vx) * s,
        _as_float("vy", vy) * s,
        _as_float("wz", wz) * s,
    )


def apply_directional_scale(
    vx: float,
    vy: float,
    wz: float,
    *,
    vx_scale: float = 1.0,
    vy_scale: float = 1.0,
    wz_scale: float = 1.0,
) -> Tuple[float, float, float]:
    """
    Scale body commands per axis.

    Useful when safety or control layers want to reduce forward speed more than
    strafe/rotation (or vice versa).
    """
    return (
        _as_float("vx", vx) * _as_float("vx_scale", vx_scale),
        _as_float("vy", vy) * _as_float("vy_scale", vy_scale),
        _as_float("wz", wz) * _as_float("wz_scale", wz_scale),
    )


# =============================================================================
# Deadband helpers
# =============================================================================
def apply_deadband(value: float, deadband: float) -> float:
    """
    Zero out small values around 0.

    Behavior:
    - if abs(value) < deadband -> 0.0
    - else unchanged

    Note:
    - This is a simple deadband (no remapping beyond threshold).
    """
    v = _as_float("value", value)
    db = abs(_as_float("deadband", deadband))
    return 0.0 if abs(v) < db else v


def apply_body_deadband(
    vx: float,
    vy: float,
    wz: float,
    *,
    cfg: DeadbandConfig,
) -> Tuple[float, float, float]:
    """
    Apply independent deadbands to body commands.
    """
    return (
        apply_deadband(vx, cfg.vx_deadband),
        apply_deadband(vy, cfg.vy_deadband),
        apply_deadband(wz, cfg.wz_deadband),
    )


# =============================================================================
# Slew / step helpers (teleop-friendly incremental control)
# =============================================================================
def increment_axis(value: float, delta: float, *, limit: float = 1.0) -> float:
    """
    Increment a single axis command and clamp to [-limit, +limit].

    This matches the teleop style where repeated keypresses adjust vx/vy/wz.
    """
    v = _as_float("value", value)
    d = _as_float("delta", delta)
    lim = abs(_as_float("limit", limit))
    if lim == 0.0:
        return 0.0
    if lim > 1.0:
        lim = 1.0
    return _clamp(v + d, -lim, +lim)


def apply_step_command(
    state: BodyCommandState,
    *,
    dvx: float = 0.0,
    dvy: float = 0.0,
    dwz: float = 0.0,
    limits: AxisLimits | None = None,
) -> BodyCommandState:
    """
    Return a new BodyCommandState after applying step increments.

    Example usage in a teleop loop:
    - W key: dvx = +step
    - S key: dvx = -step
    - A key: dvy = -step
    - D key: dvy = +step
    """
    if not isinstance(state, BodyCommandState):
        raise ScalingValueError("state must be BodyCommandState")

    next_vx = _as_float("state.vx", state.vx) + _as_float("dvx", dvx)
    next_vy = _as_float("state.vy", state.vy) + _as_float("dvy", dvy)
    next_wz = _as_float("state.wz", state.wz) + _as_float("dwz", dwz)

    if limits is None:
        next_vx, next_vy, next_wz = clamp_body_unit(next_vx, next_vy, next_wz)
    else:
        next_vx, next_vy, next_wz = apply_axis_limits(
            next_vx, next_vy, next_wz, limits=limits
        )

    return BodyCommandState(vx=next_vx, vy=next_vy, wz=next_wz)


def reset_body_state() -> BodyCommandState:
    """
    Convenience helper returning a zeroed body command state.
    """
    return BodyCommandState(vx=0.0, vy=0.0, wz=0.0)


# =============================================================================
# Decay-style smoothing (matches your teleop behavior style)
# =============================================================================
def decay_factor_from_dt(
    *,
    decay: float,
    dt: float,
    hz: float,
) -> float:
    """
    Compute the dt-aware exponential decay factor used in your proven teleop style:

        d = decay ** (dt * hz)

    where:
    - decay in [0,1]
    - hz is nominal loop rate (e.g. 30)
    - dt is measured loop delta time in seconds

    Returns:
        d in [0,1] (for valid inputs)
    """
    dec = _clamp01(decay, "decay")
    dt_f = _as_float("dt", dt)
    hz_f = _as_float("hz", hz)

    if dt_f < 0.0:
        raise ScalingValueError(f"dt must be >= 0, got {dt}")
    if hz_f <= 0.0:
        raise ScalingValueError(f"hz must be > 0, got {hz}")

    return dec ** (dt_f * hz_f)


def apply_decay_to_axis(value: float, *, decay: float, dt: float, hz: float) -> float:
    """
    Apply dt-aware exponential decay to a single axis command.
    """
    v = _as_float("value", value)
    d = decay_factor_from_dt(decay=decay, dt=dt, hz=hz)
    return v * d


def apply_decay_to_body(
    vx: float,
    vy: float,
    wz: float,
    *,
    decay: float,
    dt: float,
    hz: float,
) -> Tuple[float, float, float]:
    """
    Apply dt-aware exponential decay to body commands (vx, vy, wz).

    This directly matches the smoothing style in your teleop script:
        d = decay ** (dt * hz)
        vx *= d; vy *= d; wz *= d
    """
    d = decay_factor_from_dt(decay=decay, dt=dt, hz=hz)
    return (
        _as_float("vx", vx) * d,
        _as_float("vy", vy) * d,
        _as_float("wz", wz) * d,
    )


def apply_decay_to_state(
    state: BodyCommandState,
    *,
    cfg: SmoothingConfig,
    dt: float,
) -> BodyCommandState:
    """
    Apply dt-aware decay smoothing to a BodyCommandState and return a new state.
    """
    if not isinstance(state, BodyCommandState):
        raise ScalingValueError("state must be BodyCommandState")

    vx, vy, wz = apply_decay_to_body(
        state.vx,
        state.vy,
        state.wz,
        decay=cfg.decay,
        dt=dt,
        hz=cfg.hz,
    )
    return BodyCommandState(vx=vx, vy=vy, wz=wz)


# =============================================================================
# Duty scaling helpers (for max_duty style control)
# =============================================================================
def clamp_max_duty(max_duty: int) -> int:
    """
    Clamp max_duty to PCA9685-safe range [0 .. 4095].
    """
    md = int(max_duty)
    if md < 0:
        return 0
    if md > 4095:
        return 4095
    return md


def scale_max_duty(
    current_max_duty: int,
    *,
    factor: float,
    min_duty: int = 600,
    max_duty_limit: int = 4095,
) -> int:
    """
    Scale a max_duty value by a factor and clamp into a configurable range.

    This mirrors your teleop behavior style:
    - downscale factor ~0.85
    - upscale factor ~1.15
    """
    cur = int(current_max_duty)
    fac = _as_float("factor", factor)

    if fac < 0.0:
        raise ScalingValueError(f"factor must be >= 0, got {factor}")

    lo = int(min_duty)
    hi = int(max_duty_limit)

    if lo < 0:
        lo = 0
    if hi > 4095:
        hi = 4095
    if lo > hi:
        raise ScalingValueError(f"Invalid duty range: min_duty={lo} > max_duty_limit={hi}")

    scaled = int(cur * fac)
    if scaled < lo:
        return lo
    if scaled > hi:
        return hi
    return scaled


def apply_signed_duty_scale(
    d_fl: int,
    d_rl: int,
    d_fr: int,
    d_rr: int,
    *,
    scale: float,
    clamp_to_pca9685: bool = True,
) -> Tuple[int, int, int, int]:
    """
    Scale signed wheel duties by a factor.

    Useful if you already mixed to wheel duties and want a final speed reduction
    without recomputing kinematics.
    """
    s = _as_float("scale", scale)

    vals = (
        int(int(d_fl) * s),
        int(int(d_rl) * s),
        int(int(d_fr) * s),
        int(int(d_rr) * s),
    )

    if not clamp_to_pca9685:
        return vals

    def _clamp_signed(v: int) -> int:
        if v > 4095:
            return 4095
        if v < -4095:
            return -4095
        return v

    return tuple(_clamp_signed(v) for v in vals)  # type: ignore[return-value]


# =============================================================================
# Convenience pipeline helpers
# =============================================================================
def prepare_body_command(
    vx: float,
    vy: float,
    wz: float,
    *,
    deadband: DeadbandConfig | None = None,
    axis_limits: AxisLimits | None = None,
    global_scale: float | None = None,
    directional_scale: Tuple[float, float, float] | None = None,
    final_unit_clamp: bool = True,
) -> Tuple[float, float, float]:
    """
    Convenience helper to process a body command through common scaling steps.

    Processing order:
    1) deadband (optional)
    2) axis limits (optional)
    3) global scale (optional)
    4) directional scale (optional)
    5) final unit clamp (optional, default True)

    This is useful in ROS nodes where you want a clean, readable command path.
    """
    x = _as_float("vx", vx)
    y = _as_float("vy", vy)
    z = _as_float("wz", wz)

    if deadband is not None:
        x, y, z = apply_body_deadband(x, y, z, cfg=deadband)

    if axis_limits is not None:
        x, y, z = apply_axis_limits(x, y, z, limits=axis_limits)

    if global_scale is not None:
        x, y, z = apply_global_scale(x, y, z, scale=global_scale)

    if directional_scale is not None:
        if len(directional_scale) != 3:
            raise ScalingValueError("directional_scale must be a 3-tuple (vx_scale, vy_scale, wz_scale)")
        x, y, z = apply_directional_scale(
            x, y, z,
            vx_scale=directional_scale[0],
            vy_scale=directional_scale[1],
            wz_scale=directional_scale[2],
        )

    if final_unit_clamp:
        x, y, z = clamp_body_unit(x, y, z)

    return x, y, z