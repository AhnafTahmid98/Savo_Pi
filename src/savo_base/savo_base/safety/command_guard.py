#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/command_guard.py
----------------------------------------------
Professional command guarding utilities for `savo_base` (ROS2 Jazzy / real robot).

Purpose
-------
Sanitize and gate incoming base motion commands before they reach the motor board.

This module is intentionally ROS-message agnostic at its core so it can be used by:
- base_driver_node.py
- base_watchdog_node.py
- teleop adapters
- future LLM/text-command adapters (after conversion to vx/vy/wz)

What it handles
---------------
- finite-value validation (reject NaN/Inf)
- deadbanding (small noise -> zero)
- velocity limit clamping
- acceleration limiting (slew-rate limiting)
- timeout checks (command freshness)
- estop/watchdog/enable gating (force stop)
- safe fallback behavior

Notes
-----
- This module does NOT talk to hardware.
- This module does NOT require ROS imports.
- It works with your `savo_base.models` dataclasses if available, but also
  supports plain values and returns generic dataclasses defined here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Optional, Tuple


# =============================================================================
# Generic dataclasses (ROS-agnostic)
# =============================================================================
@dataclass
class Velocity3:
    """Robot-centric velocity command in base frame."""
    vx: float = 0.0   # m/s
    vy: float = 0.0   # m/s
    wz: float = 0.0   # rad/s


@dataclass
class CommandLimits:
    """Motion and rate limits for guarding."""
    max_vx_mps: float = 0.35
    max_vy_mps: float = 0.35
    max_wz_radps: float = 1.20

    max_ax_mps2: float = 0.80
    max_ay_mps2: float = 0.80
    max_awz_radps2: float = 2.00

    deadband_vx: float = 0.01
    deadband_vy: float = 0.01
    deadband_wz: float = 0.02


@dataclass
class GuardInputs:
    """
    Runtime gate inputs usually coming from ROS node state.
    """
    base_enabled: bool = True
    estop_active: bool = False
    watchdog_tripped: bool = False

    # command freshness
    command_timeout_s: float = 0.50
    now_monotonic_s: float = 0.0
    command_stamp_monotonic_s: Optional[float] = None


@dataclass
class GuardStatus:
    """
    Detailed result status for logging / state publishing.
    """
    accepted: bool = True
    forced_stop: bool = False
    stale_command: bool = False
    invalid_command: bool = False

    reason: str = "ok"

    # optional diagnostics
    clamped_vx: bool = False
    clamped_vy: bool = False
    clamped_wz: bool = False

    accel_limited_vx: bool = False
    accel_limited_vy: bool = False
    accel_limited_wz: bool = False

    deadbanded_vx: bool = False
    deadbanded_vy: bool = False
    deadbanded_wz: bool = False

    command_age_s: Optional[float] = None


@dataclass
class GuardResult:
    """
    Output of command guard evaluation.
    """
    command: Velocity3 = field(default_factory=Velocity3)
    status: GuardStatus = field(default_factory=GuardStatus)


# =============================================================================
# Utility helpers
# =============================================================================
def monotonic_time_s() -> float:
    """Monotonic time in seconds (safe for timeout checks)."""
    return time.monotonic()


def zero_velocity() -> Velocity3:
    return Velocity3(0.0, 0.0, 0.0)


def is_finite_number(x: float) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(float(x))


def is_valid_velocity(cmd: Velocity3) -> bool:
    return (
        is_finite_number(cmd.vx) and
        is_finite_number(cmd.vy) and
        is_finite_number(cmd.wz)
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _apply_deadband(value: float, threshold: float) -> Tuple[float, bool]:
    if abs(value) < max(0.0, threshold):
        return 0.0, True
    return value, False


def _slew_limit(target: float, prev: float, max_rate_per_s: float, dt_s: float) -> Tuple[float, bool]:
    """
    Limit change rate of a scalar signal.

    Returns (limited_value, was_limited)
    """
    if dt_s <= 0.0 or max_rate_per_s <= 0.0:
        # No valid dt/rate -> no slew limiting applied
        return target, False

    max_delta = max_rate_per_s * dt_s
    delta = target - prev

    if delta > max_delta:
        return prev + max_delta, True
    if delta < -max_delta:
        return prev - max_delta, True
    return target, False


def command_age_s(now_monotonic_s: float, command_stamp_monotonic_s: Optional[float]) -> Optional[float]:
    if command_stamp_monotonic_s is None:
        return None
    return max(0.0, float(now_monotonic_s) - float(command_stamp_monotonic_s))


def is_command_stale(
    *,
    now_monotonic_s: float,
    command_stamp_monotonic_s: Optional[float],
    timeout_s: float,
) -> Tuple[bool, Optional[float]]:
    """
    Returns (stale, age_s).
    If no stamp is available, treat as stale.
    """
    age = command_age_s(now_monotonic_s, command_stamp_monotonic_s)
    if age is None:
        return True, None
    return (age > max(0.0, timeout_s)), age


# =============================================================================
# Core command guard
# =============================================================================
class CommandGuard:
    """
    Stateful command guard with acceleration limiting.

    Typical usage in `base_driver_node`:
    ------------------------------
        guard = CommandGuard(limits, command_timeout_s=0.5)

        result = guard.evaluate(
            raw_cmd=Velocity3(vx, vy, wz),
            base_enabled=enabled,
            estop_active=estop,
            watchdog_tripped=trip,
            command_stamp_monotonic_s=last_cmd_stamp,
        )

        safe_cmd = result.command
        status = result.status
    """

    def __init__(
        self,
        limits: Optional[CommandLimits] = None,
        *,
        command_timeout_s: float = 0.50,
        stop_on_invalid: bool = True,
        stop_on_stale: bool = True,
        require_enable: bool = True,
        startup_zero_reference: bool = True,
    ) -> None:
        self.limits = limits or CommandLimits()
        self.command_timeout_s = max(0.01, float(command_timeout_s))

        self.stop_on_invalid = bool(stop_on_invalid)
        self.stop_on_stale = bool(stop_on_stale)
        self.require_enable = bool(require_enable)

        # Last output (post-guard) command used as slew-rate reference
        self._last_output_cmd = zero_velocity() if startup_zero_reference else zero_velocity()
        self._last_eval_monotonic_s: Optional[float] = None

    # -------------------------------------------------------------------------
    # Public helpers
    # -------------------------------------------------------------------------
    def reset(self) -> None:
        """Reset internal slew-rate state."""
        self._last_output_cmd = zero_velocity()
        self._last_eval_monotonic_s = None

    def force_stop_state(self) -> None:
        """
        Force internal state to zero (useful after estop/watchdog to prevent a jump
        when motion resumes).
        """
        self._last_output_cmd = zero_velocity()

    @property
    def last_output_command(self) -> Velocity3:
        return Velocity3(
            self._last_output_cmd.vx,
            self._last_output_cmd.vy,
            self._last_output_cmd.wz,
        )

    # -------------------------------------------------------------------------
    # Evaluation
    # -------------------------------------------------------------------------
    def evaluate(
        self,
        *,
        raw_cmd: Velocity3,
        base_enabled: bool = True,
        estop_active: bool = False,
        watchdog_tripped: bool = False,
        command_stamp_monotonic_s: Optional[float] = None,
        now_monotonic_s: Optional[float] = None,
    ) -> GuardResult:
        """
        Evaluate and sanitize a command.

        Order of operations
        -------------------
        1) validity check (NaN/Inf)
        2) stale check
        3) hard gates (enable/estop/watchdog)
        4) deadband
        5) velocity clamps
        6) acceleration limiting (slew)
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        status = GuardStatus()

        # Compute dt for slew limiting
        if self._last_eval_monotonic_s is None:
            dt_s = 0.0
        else:
            dt_s = max(0.0, now_s - self._last_eval_monotonic_s)
        self._last_eval_monotonic_s = now_s

        # Freshness check
        stale, age_s = is_command_stale(
            now_monotonic_s=now_s,
            command_stamp_monotonic_s=command_stamp_monotonic_s,
            timeout_s=self.command_timeout_s,
        )
        status.command_age_s = age_s

        # Validate numeric content
        if not is_valid_velocity(raw_cmd):
            status.invalid_command = True
            status.accepted = False
            status.reason = "invalid_command_nan_or_inf"
            if self.stop_on_invalid:
                status.forced_stop = True
                out = zero_velocity()
                self._last_output_cmd = out
                return GuardResult(command=out, status=status)
            # If configured not to stop, fall back to previous safe command
            out = self.last_output_command
            return GuardResult(command=out, status=status)

        # Stale command
        if stale:
            status.stale_command = True
            status.accepted = False
            status.reason = "stale_command"
            if self.stop_on_stale:
                status.forced_stop = True
                out = zero_velocity()
                self._last_output_cmd = out
                return GuardResult(command=out, status=status)

        # Hard gates: estop / watchdog / enable
        if estop_active:
            status.accepted = False
            status.forced_stop = True
            status.reason = "estop_active"
            out = zero_velocity()
            self._last_output_cmd = out
            return GuardResult(command=out, status=status)

        if watchdog_tripped:
            status.accepted = False
            status.forced_stop = True
            status.reason = "watchdog_tripped"
            out = zero_velocity()
            self._last_output_cmd = out
            return GuardResult(command=out, status=status)

        if self.require_enable and not base_enabled:
            status.accepted = False
            status.forced_stop = True
            status.reason = "base_disabled"
            out = zero_velocity()
            self._last_output_cmd = out
            return GuardResult(command=out, status=status)

        # Start with a copy
        cmd = Velocity3(float(raw_cmd.vx), float(raw_cmd.vy), float(raw_cmd.wz))

        # Deadband
        cmd.vx, status.deadbanded_vx = _apply_deadband(cmd.vx, self.limits.deadband_vx)
        cmd.vy, status.deadbanded_vy = _apply_deadband(cmd.vy, self.limits.deadband_vy)
        cmd.wz, status.deadbanded_wz = _apply_deadband(cmd.wz, self.limits.deadband_wz)

        # Velocity clamps
        pre_vx, pre_vy, pre_wz = cmd.vx, cmd.vy, cmd.wz

        cmd.vx = _clamp(cmd.vx, -abs(self.limits.max_vx_mps), abs(self.limits.max_vx_mps))
        cmd.vy = _clamp(cmd.vy, -abs(self.limits.max_vy_mps), abs(self.limits.max_vy_mps))
        cmd.wz = _clamp(cmd.wz, -abs(self.limits.max_wz_radps), abs(self.limits.max_wz_radps))

        status.clamped_vx = (cmd.vx != pre_vx)
        status.clamped_vy = (cmd.vy != pre_vy)
        status.clamped_wz = (cmd.wz != pre_wz)

        # Slew-rate limiting (acceleration limiting)
        cmd.vx, status.accel_limited_vx = _slew_limit(
            cmd.vx, self._last_output_cmd.vx, abs(self.limits.max_ax_mps2), dt_s
        )
        cmd.vy, status.accel_limited_vy = _slew_limit(
            cmd.vy, self._last_output_cmd.vy, abs(self.limits.max_ay_mps2), dt_s
        )
        cmd.wz, status.accel_limited_wz = _slew_limit(
            cmd.wz, self._last_output_cmd.wz, abs(self.limits.max_awz_radps2), dt_s
        )

        status.accepted = True
        status.reason = "ok"

        self._last_output_cmd = Velocity3(cmd.vx, cmd.vy, cmd.wz)
        return GuardResult(command=cmd, status=status)


# =============================================================================
# Functional wrapper (stateless path, optional use)
# =============================================================================
def guard_command_once(
    *,
    raw_cmd: Velocity3,
    limits: Optional[CommandLimits] = None,
    prev_output_cmd: Optional[Velocity3] = None,
    dt_s: float = 0.0,
    inputs: Optional[GuardInputs] = None,
    stop_on_invalid: bool = True,
    stop_on_stale: bool = True,
    require_enable: bool = True,
) -> GuardResult:
    """
    Stateless one-shot command guard (useful for unit tests or custom nodes).

    If you want acceleration limiting across time in production, prefer the
    stateful `CommandGuard` class.
    """
    limits = limits or CommandLimits()
    prev = prev_output_cmd or zero_velocity()
    inputs = inputs or GuardInputs(now_monotonic_s=monotonic_time_s())

    status = GuardStatus()

    stale, age_s = is_command_stale(
        now_monotonic_s=inputs.now_monotonic_s,
        command_stamp_monotonic_s=inputs.command_stamp_monotonic_s,
        timeout_s=inputs.command_timeout_s,
    )
    status.command_age_s = age_s

    # Validate
    if not is_valid_velocity(raw_cmd):
        status.invalid_command = True
        status.accepted = False
        status.reason = "invalid_command_nan_or_inf"
        if stop_on_invalid:
            status.forced_stop = True
            return GuardResult(command=zero_velocity(), status=status)
        return GuardResult(command=Velocity3(prev.vx, prev.vy, prev.wz), status=status)

    # Stale
    if stale:
        status.stale_command = True
        status.accepted = False
        status.reason = "stale_command"
        if stop_on_stale:
            status.forced_stop = True
            return GuardResult(command=zero_velocity(), status=status)

    # Hard gates
    if inputs.estop_active:
        status.accepted = False
        status.forced_stop = True
        status.reason = "estop_active"
        return GuardResult(command=zero_velocity(), status=status)

    if inputs.watchdog_tripped:
        status.accepted = False
        status.forced_stop = True
        status.reason = "watchdog_tripped"
        return GuardResult(command=zero_velocity(), status=status)

    if require_enable and not inputs.base_enabled:
        status.accepted = False
        status.forced_stop = True
        status.reason = "base_disabled"
        return GuardResult(command=zero_velocity(), status=status)

    # Process
    cmd = Velocity3(float(raw_cmd.vx), float(raw_cmd.vy), float(raw_cmd.wz))

    cmd.vx, status.deadbanded_vx = _apply_deadband(cmd.vx, limits.deadband_vx)
    cmd.vy, status.deadbanded_vy = _apply_deadband(cmd.vy, limits.deadband_vy)
    cmd.wz, status.deadbanded_wz = _apply_deadband(cmd.wz, limits.deadband_wz)

    pre_vx, pre_vy, pre_wz = cmd.vx, cmd.vy, cmd.wz

    cmd.vx = _clamp(cmd.vx, -abs(limits.max_vx_mps), abs(limits.max_vx_mps))
    cmd.vy = _clamp(cmd.vy, -abs(limits.max_vy_mps), abs(limits.max_vy_mps))
    cmd.wz = _clamp(cmd.wz, -abs(limits.max_wz_radps), abs(limits.max_wz_radps))

    status.clamped_vx = (cmd.vx != pre_vx)
    status.clamped_vy = (cmd.vy != pre_vy)
    status.clamped_wz = (cmd.wz != pre_wz)

    cmd.vx, status.accel_limited_vx = _slew_limit(cmd.vx, prev.vx, abs(limits.max_ax_mps2), max(0.0, dt_s))
    cmd.vy, status.accel_limited_vy = _slew_limit(cmd.vy, prev.vy, abs(limits.max_ay_mps2), max(0.0, dt_s))
    cmd.wz, status.accel_limited_wz = _slew_limit(cmd.wz, prev.wz, abs(limits.max_awz_radps2), max(0.0, dt_s))

    status.accepted = True
    status.reason = "ok"
    return GuardResult(command=cmd, status=status)


# =============================================================================
# Interop helpers (optional bridge to your model classes)
# =============================================================================
def velocity3_from_any(obj: object) -> Velocity3:
    """
    Best-effort conversion from common objects to Velocity3.

    Supports:
    - Velocity3
    - objects/dataclasses with attributes vx, vy, wz
    - dict-like {'vx':..., 'vy':..., 'wz':...}
    """
    if isinstance(obj, Velocity3):
        return Velocity3(obj.vx, obj.vy, obj.wz)

    # dict-like
    if isinstance(obj, dict):
        return Velocity3(
            float(obj.get("vx", 0.0)),
            float(obj.get("vy", 0.0)),
            float(obj.get("wz", 0.0)),
        )

    # attribute-like (e.g., your WheelCommand/BaseState wrappers)
    vx = float(getattr(obj, "vx", 0.0))
    vy = float(getattr(obj, "vy", 0.0))
    wz = float(getattr(obj, "wz", 0.0))
    return Velocity3(vx, vy, wz)


def commandlimits_from_any(obj: object) -> CommandLimits:
    """
    Best-effort conversion from your `BaseLimits` model or dict to CommandLimits.
    Missing fields fall back to defaults.
    """
    if isinstance(obj, CommandLimits):
        return CommandLimits(**obj.__dict__)

    if isinstance(obj, dict):
        d = dict(obj)
        return CommandLimits(
            max_vx_mps=float(d.get("max_vx_mps", 0.35)),
            max_vy_mps=float(d.get("max_vy_mps", 0.35)),
            max_wz_radps=float(d.get("max_wz_radps", 1.20)),
            max_ax_mps2=float(d.get("max_ax_mps2", 0.80)),
            max_ay_mps2=float(d.get("max_ay_mps2", 0.80)),
            max_awz_radps2=float(d.get("max_awz_radps2", 2.00)),
            deadband_vx=float(d.get("deadband_vx", 0.01)),
            deadband_vy=float(d.get("deadband_vy", 0.01)),
            deadband_wz=float(d.get("deadband_wz", 0.02)),
        )

    return CommandLimits(
        max_vx_mps=float(getattr(obj, "max_vx_mps", 0.35)),
        max_vy_mps=float(getattr(obj, "max_vy_mps", 0.35)),
        max_wz_radps=float(getattr(obj, "max_wz_radps", 1.20)),
        max_ax_mps2=float(getattr(obj, "max_ax_mps2", 0.80)),
        max_ay_mps2=float(getattr(obj, "max_ay_mps2", 0.80)),
        max_awz_radps2=float(getattr(obj, "max_awz_radps2", 2.00)),
        deadband_vx=float(getattr(obj, "deadband_vx", 0.01)),
        deadband_vy=float(getattr(obj, "deadband_vy", 0.01)),
        deadband_wz=float(getattr(obj, "deadband_wz", 0.02)),
    )


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # dataclasses
    "Velocity3",
    "CommandLimits",
    "GuardInputs",
    "GuardStatus",
    "GuardResult",
    # helpers
    "monotonic_time_s",
    "zero_velocity",
    "is_finite_number",
    "is_valid_velocity",
    "command_age_s",
    "is_command_stale",
    # core
    "CommandGuard",
    "guard_command_once",
    # interop
    "velocity3_from_any",
    "commandlimits_from_any",
]