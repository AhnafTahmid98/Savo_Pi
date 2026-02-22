#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/base_limits.py
--------------------------------------------
Professional motion/safety limit models for Robot Savo base control.

Purpose
- Centralize numeric limits used by teleop, base controllers, and safety shaping
- Provide validation and clamping helpers for translational/rotational commands
- Keep limits explicit and reusable across ROS2 Jazzy and non-ROS tools

Scope
- This file models *limits* only (not motor drivers, not kinematics)
- It can be used by:
  - CLI teleop (letters / keyboard)
  - cmd_vel shaper nodes
  - safety gates / recovery behaviors
  - dry-run simulations

Notes
- Robot Savo is a mecanum base (holonomic), so commands are modeled as:
  - vx (forward/back)
  - vy (strafe left/right)
  - wz (yaw rate / rotate)
- Values are typically normalized in [-1..1] before mapping to wheel duties,
  but this module supports both normalized limits and physical SI limits.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Tuple, Optional
import math


# =============================================================================
# Exceptions (local lightweight models-layer exceptions)
# =============================================================================
class LimitsValidationError(ValueError):
    """Raised when a limits model contains invalid values."""
    pass


# =============================================================================
# Generic helpers
# =============================================================================
def clamp(value: float, lo: float, hi: float) -> float:
    """
    Clamp scalar value to [lo, hi].
    """
    if lo > hi:
        raise LimitsValidationError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def clamp_abs(value: float, max_abs: float) -> float:
    """
    Clamp scalar value to [-max_abs, +max_abs].
    """
    if max_abs < 0.0:
        raise LimitsValidationError(f"max_abs must be >= 0, got {max_abs}")
    return clamp(value, -max_abs, +max_abs)


def is_finite_number(x: float) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


# =============================================================================
# Core limits dataclasses
# =============================================================================
@dataclass(frozen=True)
class AxisLimit:
    """
    Symmetric absolute limit for one command axis.

    Example
    -------
    - vx normalized limit: max_abs = 1.0
    - wz physical limit:   max_abs = 1.2 (rad/s)
    """
    max_abs: float

    def validate(self, *, field_name: str = "axis") -> "AxisLimit":
        if not is_finite_number(self.max_abs):
            raise LimitsValidationError(f"{field_name}.max_abs must be finite, got {self.max_abs!r}")
        if self.max_abs < 0.0:
            raise LimitsValidationError(f"{field_name}.max_abs must be >= 0, got {self.max_abs}")
        return self

    def clamp(self, value: float) -> float:
        self.validate()
        if not is_finite_number(value):
            raise LimitsValidationError(f"Value for axis clamp must be finite, got {value!r}")
        return clamp_abs(float(value), float(self.max_abs))

    def as_tuple(self) -> Tuple[float, float]:
        self.validate()
        m = float(self.max_abs)
        return (-m, +m)

    def to_dict(self) -> Dict[str, float]:
        self.validate()
        return {"max_abs": float(self.max_abs)}


@dataclass(frozen=True)
class AccelLimit:
    """
    Symmetric acceleration/jerk-like rate limit (per-axis), units depend on usage.

    Examples
    --------
    - normalized teleop ramp rate in units/s
    - physical acceleration in m/s^2
    - angular acceleration in rad/s^2
    """
    max_delta_per_s: float

    def validate(self, *, field_name: str = "accel") -> "AccelLimit":
        if not is_finite_number(self.max_delta_per_s):
            raise LimitsValidationError(
                f"{field_name}.max_delta_per_s must be finite, got {self.max_delta_per_s!r}"
            )
        if self.max_delta_per_s < 0.0:
            raise LimitsValidationError(
                f"{field_name}.max_delta_per_s must be >= 0, got {self.max_delta_per_s}"
            )
        return self

    def clamp_delta(self, delta: float, dt_s: float) -> float:
        self.validate()
        if not is_finite_number(delta):
            raise LimitsValidationError(f"delta must be finite, got {delta!r}")
        if not is_finite_number(dt_s) or dt_s < 0.0:
            raise LimitsValidationError(f"dt_s must be finite and >= 0, got {dt_s!r}")
        max_step = float(self.max_delta_per_s) * float(dt_s)
        return clamp_abs(float(delta), max_step)

    def to_dict(self) -> Dict[str, float]:
        self.validate()
        return {"max_delta_per_s": float(self.max_delta_per_s)}


# =============================================================================
# Normalized command limits (dimensionless, typically [-1..1])
# =============================================================================
@dataclass(frozen=True)
class NormalizedCommandLimits:
    """
    Limits for normalized body-frame commands (dimensionless).

    Typical mapping
    ---------------
    vx in [-1,1] -> forward/back command
    vy in [-1,1] -> strafe command
    wz in [-1,1] -> yaw command

    These are the limits most useful for:
    - keyboard teleop
    - non-ROS direct board control
    - kinematics mixing before duty scaling
    """
    vx: AxisLimit = AxisLimit(1.0)
    vy: AxisLimit = AxisLimit(1.0)
    wz: AxisLimit = AxisLimit(1.0)

    # Optional overall Euclidean cap in XY plane (prevents diagonal overspeed)
    max_xy_norm: float = 1.0

    def validate(self) -> "NormalizedCommandLimits":
        self.vx.validate(field_name="vx")
        self.vy.validate(field_name="vy")
        self.wz.validate(field_name="wz")

        if not is_finite_number(self.max_xy_norm):
            raise LimitsValidationError(f"max_xy_norm must be finite, got {self.max_xy_norm!r}")
        if self.max_xy_norm < 0.0:
            raise LimitsValidationError(f"max_xy_norm must be >= 0, got {self.max_xy_norm}")

        # Usually normalized limits are <= 1.0, but we allow >1.0 intentionally
        # for advanced tuning/testing. Validation keeps it generic and safe.
        return self

    def clamp_command(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """
        Clamp per-axis first, then clamp XY vector magnitude to max_xy_norm.
        """
        self.validate()

        cx = self.vx.clamp(vx)
        cy = self.vy.clamp(vy)
        cz = self.wz.clamp(wz)

        xy_mag = math.hypot(cx, cy)
        if xy_mag > float(self.max_xy_norm) and xy_mag > 0.0:
            scale = float(self.max_xy_norm) / xy_mag
            cx *= scale
            cy *= scale

        return cx, cy, cz

    def to_dict(self) -> Dict[str, object]:
        self.validate()
        return {
            "vx": self.vx.to_dict(),
            "vy": self.vy.to_dict(),
            "wz": self.wz.to_dict(),
            "max_xy_norm": float(self.max_xy_norm),
        }


# =============================================================================
# Physical command limits (SI units)
# =============================================================================
@dataclass(frozen=True)
class PhysicalCommandLimits:
    """
    SI-unit body command limits for cmd_vel-like control.

    Units
    -----
    - vx_mps, vy_mps : m/s
    - wz_radps       : rad/s

    These are useful for:
    - ROS2 cmd_vel shaping / safety
    - realistic motion envelopes
    - converting normalized teleop to physical commands
    """
    vx_mps: AxisLimit = AxisLimit(0.60)
    vy_mps: AxisLimit = AxisLimit(0.60)
    wz_radps: AxisLimit = AxisLimit(1.20)

    max_xy_speed_mps: float = 0.60

    def validate(self) -> "PhysicalCommandLimits":
        self.vx_mps.validate(field_name="vx_mps")
        self.vy_mps.validate(field_name="vy_mps")
        self.wz_radps.validate(field_name="wz_radps")

        if not is_finite_number(self.max_xy_speed_mps):
            raise LimitsValidationError(
                f"max_xy_speed_mps must be finite, got {self.max_xy_speed_mps!r}"
            )
        if self.max_xy_speed_mps < 0.0:
            raise LimitsValidationError(
                f"max_xy_speed_mps must be >= 0, got {self.max_xy_speed_mps}"
            )
        return self

    def clamp_command(self, vx_mps: float, vy_mps: float, wz_radps: float) -> Tuple[float, float, float]:
        self.validate()

        cx = self.vx_mps.clamp(vx_mps)
        cy = self.vy_mps.clamp(vy_mps)
        cz = self.wz_radps.clamp(wz_radps)

        xy_mag = math.hypot(cx, cy)
        if xy_mag > float(self.max_xy_speed_mps) and xy_mag > 0.0:
            scale = float(self.max_xy_speed_mps) / xy_mag
            cx *= scale
            cy *= scale

        return cx, cy, cz

    def to_dict(self) -> Dict[str, object]:
        self.validate()
        return {
            "vx_mps": self.vx_mps.to_dict(),
            "vy_mps": self.vy_mps.to_dict(),
            "wz_radps": self.wz_radps.to_dict(),
            "max_xy_speed_mps": float(self.max_xy_speed_mps),
        }


# =============================================================================
# Rate / shaper limits
# =============================================================================
@dataclass(frozen=True)
class CommandRateLimits:
    """
    Per-axis rate limits for command shaping (slew-rate limiting).

    Units depend on command domain:
    - For normalized commands: units/s
    - For physical commands:   m/s^2 (vx, vy), rad/s^2 (wz)

    This model is intentionally generic.
    """
    vx: AccelLimit = AccelLimit(2.0)
    vy: AccelLimit = AccelLimit(2.0)
    wz: AccelLimit = AccelLimit(3.0)

    def validate(self) -> "CommandRateLimits":
        self.vx.validate(field_name="vx")
        self.vy.validate(field_name="vy")
        self.wz.validate(field_name="wz")
        return self

    def clamp_step(
        self,
        current_vx: float,
        current_vy: float,
        current_wz: float,
        target_vx: float,
        target_vy: float,
        target_wz: float,
        dt_s: float,
    ) -> Tuple[float, float, float]:
        """
        Slew-limit one step from current -> target using per-axis max delta/s.
        """
        self.validate()
        if not is_finite_number(dt_s) or dt_s < 0.0:
            raise LimitsValidationError(f"dt_s must be finite and >= 0, got {dt_s!r}")

        dx = self.vx.clamp_delta(float(target_vx) - float(current_vx), dt_s)
        dy = self.vy.clamp_delta(float(target_vy) - float(current_vy), dt_s)
        dz = self.wz.clamp_delta(float(target_wz) - float(current_wz), dt_s)

        return float(current_vx) + dx, float(current_vy) + dy, float(current_wz) + dz

    def to_dict(self) -> Dict[str, object]:
        self.validate()
        return {
            "vx": self.vx.to_dict(),
            "vy": self.vy.to_dict(),
            "wz": self.wz.to_dict(),
        }


# =============================================================================
# Duty limits (motor command domain)
# =============================================================================
@dataclass(frozen=True)
class DutyLimits:
    """
    Signed duty limits for wheel commands sent to the motor board.

    PCA9685 raw duty is 0..4095 per channel, but wheel commands in Robot Savo
    are represented as signed values in [-max_abs_duty, +max_abs_duty].
    """
    max_abs_duty: int = 3000
    hard_cap_duty: int = 4095
    min_active_duty: int = 0  # optional deadband-compensation helper (not auto-applied)

    def validate(self) -> "DutyLimits":
        try:
            mad = int(self.max_abs_duty)
            hcd = int(self.hard_cap_duty)
            mid = int(self.min_active_duty)
        except Exception as e:
            raise LimitsValidationError("Duty limits must be integer-like values") from e

        if hcd < 0 or hcd > 4095:
            raise LimitsValidationError(f"hard_cap_duty must be in [0,4095], got {hcd}")
        if mad < 0 or mad > hcd:
            raise LimitsValidationError(
                f"max_abs_duty must be in [0, hard_cap_duty={hcd}], got {mad}"
            )
        if mid < 0 or mid > mad:
            raise LimitsValidationError(
                f"min_active_duty must be in [0, max_abs_duty={mad}], got {mid}"
            )
        return self

    def clamp_signed(self, duty: int) -> int:
        self.validate()
        try:
            d = int(duty)
        except Exception as e:
            raise LimitsValidationError(f"duty must be int-like, got {duty!r}") from e

        m = int(self.max_abs_duty)
        if d > m:
            return m
        if d < -m:
            return -m
        return d

    def clamp_signed_4(self, fl: int, rl: int, fr: int, rr: int) -> Tuple[int, int, int, int]:
        self.validate()
        return (
            self.clamp_signed(fl),
            self.clamp_signed(rl),
            self.clamp_signed(fr),
            self.clamp_signed(rr),
        )

    def to_dict(self) -> Dict[str, int]:
        self.validate()
        return {
            "max_abs_duty": int(self.max_abs_duty),
            "hard_cap_duty": int(self.hard_cap_duty),
            "min_active_duty": int(self.min_active_duty),
        }


# =============================================================================
# Aggregate base limits profile
# =============================================================================
@dataclass(frozen=True)
class BaseLimits:
    """
    Aggregated Robot Savo base limits profile.

    This is the main object most modules should import/use.
    It groups:
    - normalized command limits (teleop / pre-kinematics)
    - physical command limits (cmd_vel / controller layer)
    - rate limits (slew shaping)
    - duty limits (motor board layer)
    """
    normalized: NormalizedCommandLimits = NormalizedCommandLimits()
    physical: PhysicalCommandLimits = PhysicalCommandLimits()
    rate_normalized: CommandRateLimits = CommandRateLimits(
        vx=AccelLimit(3.0),
        vy=AccelLimit(3.0),
        wz=AccelLimit(4.0),
    )
    rate_physical: CommandRateLimits = CommandRateLimits(
        vx=AccelLimit(0.8),   # m/s^2
        vy=AccelLimit(0.8),   # m/s^2
        wz=AccelLimit(1.8),   # rad/s^2
    )
    duty: DutyLimits = DutyLimits(max_abs_duty=3000, hard_cap_duty=4095, min_active_duty=0)

    profile_name: str = "robot_savo_default_base_limits"

    def validate(self) -> "BaseLimits":
        self.normalized.validate()
        self.physical.validate()
        self.rate_normalized.validate()
        self.rate_physical.validate()
        self.duty.validate()

        if not isinstance(self.profile_name, str) or not self.profile_name.strip():
            raise LimitsValidationError("profile_name must be a non-empty string")
        return self

    def clamp_normalized(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        self.validate()
        return self.normalized.clamp_command(vx, vy, wz)

    def clamp_physical(self, vx_mps: float, vy_mps: float, wz_radps: float) -> Tuple[float, float, float]:
        self.validate()
        return self.physical.clamp_command(vx_mps, vy_mps, wz_radps)

    def clamp_duties(self, fl: int, rl: int, fr: int, rr: int) -> Tuple[int, int, int, int]:
        self.validate()
        return self.duty.clamp_signed_4(fl, rl, fr, rr)

    def to_dict(self) -> Dict[str, object]:
        self.validate()
        return {
            "profile_name": self.profile_name,
            "normalized": self.normalized.to_dict(),
            "physical": self.physical.to_dict(),
            "rate_normalized": self.rate_normalized.to_dict(),
            "rate_physical": self.rate_physical.to_dict(),
            "duty": self.duty.to_dict(),
        }


# =============================================================================
# Canonical Robot Savo defaults (easy imports)
# =============================================================================
def robot_savo_default_base_limits() -> BaseLimits:
    """
    Canonical default limits profile for Robot Savo base.

    Tuned to match your current practical stage:
    - real robot testing
    - conservative speed envelope
    - PCA9685 duty default around 3000
    """
    return BaseLimits().validate()


def robot_savo_safe_indoor_limits() -> BaseLimits:
    """
    More conservative indoor profile for first tests / crowded environments.
    """
    return BaseLimits(
        normalized=NormalizedCommandLimits(
            vx=AxisLimit(0.70),
            vy=AxisLimit(0.70),
            wz=AxisLimit(0.60),
            max_xy_norm=0.75,
        ),
        physical=PhysicalCommandLimits(
            vx_mps=AxisLimit(0.35),
            vy_mps=AxisLimit(0.35),
            wz_radps=AxisLimit(0.90),
            max_xy_speed_mps=0.35,
        ),
        rate_normalized=CommandRateLimits(
            vx=AccelLimit(2.0),
            vy=AccelLimit(2.0),
            wz=AccelLimit(2.5),
        ),
        rate_physical=CommandRateLimits(
            vx=AccelLimit(0.50),
            vy=AccelLimit(0.50),
            wz=AccelLimit(1.00),
        ),
        duty=DutyLimits(max_abs_duty=2200, hard_cap_duty=4095, min_active_duty=0),
        profile_name="robot_savo_safe_indoor_limits",
    ).validate()


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # exceptions
    "LimitsValidationError",

    # helpers
    "clamp",
    "clamp_abs",
    "is_finite_number",

    # primitive limits
    "AxisLimit",
    "AccelLimit",

    # grouped limits
    "NormalizedCommandLimits",
    "PhysicalCommandLimits",
    "CommandRateLimits",
    "DutyLimits",
    "BaseLimits",

    # profiles
    "robot_savo_default_base_limits",
    "robot_savo_safe_indoor_limits",
]