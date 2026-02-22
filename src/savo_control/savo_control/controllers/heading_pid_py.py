#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control.controllers.heading_pid_py
====================================================

Purpose
-------
Python heading/yaw controller helper aligned with the C++ HeadingController
(`heading_controller.hpp/.cpp`) for tuning, diagnostics, and Python test nodes.

Important
---------
- ROS-independent (pure Python)
- Intended for Python tools/test nodes, not to replace the authoritative C++ path
- Built on top of `pid_py.py` (Python PID mirror of C++ pid.hpp)

Behavior aligned with C++ HeadingController
-------------------------------------------
- heading target management (set/clear/capture hold)
- wrapped angular error (shortest angular distance)
- PID-based yaw-rate command generation (wz)
- optional max clamp / min effective command / deadband
- heading tolerance check
- update_to_target() without mutating stored target
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Optional
import math

from .pid_py import Pid, PidConfig, PidResult


# =============================================================================
# Small math helpers (Python mirror of control_math usage)
# =============================================================================

def wrap_angle_rad(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    if not isfinite(angle):
        return 0.0
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    # keep behavior stable around boundary
    if a <= -math.pi:
        return math.pi
    return a


def shortest_angular_distance_rad(from_angle: float, to_angle: float) -> float:
    """
    Shortest signed angular distance from `from_angle` to `to_angle`.
    Positive => CCW, Negative => CW
    """
    return wrap_angle_rad(to_angle - from_angle)


def saturate_abs(value: float, abs_limit: float) -> float:
    """Clamp |value| to abs_limit."""
    if not isfinite(value):
        return 0.0
    if not isfinite(abs_limit) or abs_limit <= 0.0:
        return value
    lim = abs(abs_limit)
    return max(-lim, min(lim, value))


def copy_sign(magnitude: float, sign_source: float) -> float:
    """Return |magnitude| with sign of sign_source."""
    if not isfinite(magnitude):
        magnitude = 0.0
    if not isfinite(sign_source):
        sign_source = 0.0
    return math.copysign(abs(magnitude), sign_source)


def apply_deadband(value: float, deadband: float) -> float:
    """Zero small values within deadband."""
    if not isfinite(value):
        return 0.0
    db = abs(deadband) if isfinite(deadband) else 0.0
    if db <= 0.0:
        return value
    return 0.0 if abs(value) < db else value


# =============================================================================
# Config / Result mirrors (C++ aligned)
# =============================================================================

@dataclass
class HeadingControllerConfig:
    # Underlying yaw PID configuration (maps to angular.z [rad/s])
    pid: PidConfig = field(default_factory=PidConfig)

    # Heading goal tolerance (rad)
    heading_tolerance_rad: float = 0.03  # ~1.7 deg

    # Final output cleanup deadband
    output_deadband_rad_s: float = 0.0

    # Minimum effective yaw-rate (anti-stiction helper), disabled if <= 0
    min_effective_wz_rad_s: float = 0.0

    # Additional external clamp after PID output, disabled if <= 0
    max_wz_rad_s: float = 0.0

    # Hold mode behavior
    reset_pid_on_hold_capture: bool = True

    # Reset PID if target changes significantly
    reset_pid_on_target_change: bool = True
    target_change_reset_threshold_rad: float = 0.02

    # Controller-level dt validity
    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0


@dataclass
class HeadingControllerResult:
    # Final commanded yaw-rate (Twist.angular.z)
    wz_cmd_rad_s: float = 0.0

    # Diagnostics
    target_yaw_rad: float = 0.0
    current_yaw_rad: float = 0.0
    error_yaw_rad: float = 0.0

    # Goal checks
    has_target: bool = False
    within_tolerance: bool = False

    # Validity flags
    valid: bool = False
    dt_valid: bool = False
    yaw_valid: bool = False

    # PID breakdown
    pid: PidResult = field(default_factory=PidResult)


# =============================================================================
# Heading Controller (Python mirror of C++ HeadingController)
# =============================================================================

class HeadingController:
    """
    Reusable heading/yaw controller built on top of `Pid`.

    Suitable for Python test nodes such as:
    - heading hold test utilities
    - rotate-to-heading test routines
    - diagnostics/tuning scripts

    Real robot note
    ---------------
    C++ heading controller remains authoritative for production runtime path.
    """

    def __init__(self, cfg: Optional[HeadingControllerConfig] = None) -> None:
        self._config = cfg if cfg is not None else HeadingControllerConfig()
        self._normalize_config()

        self._pid = Pid(self._config.pid)
        self._pid.set_config(self._config.pid)

        self._has_target = False
        self._target_yaw_rad = 0.0

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------
    def set_config(self, cfg: HeadingControllerConfig) -> None:
        self._config = cfg
        self._normalize_config()
        self._pid.set_config(self._config.pid)

    @property
    def config(self) -> HeadingControllerConfig:
        return self._config

    def pid(self) -> Pid:
        return self._pid

    # -------------------------------------------------------------------------
    # Target management
    # -------------------------------------------------------------------------
    def has_target(self) -> bool:
        return self._has_target

    def target_yaw_rad(self) -> float:
        return self._target_yaw_rad

    def set_target_yaw(self, target_yaw_rad: float) -> None:
        """Set absolute heading target (wrapped to [-pi, pi])."""
        if not isfinite(target_yaw_rad):
            return

        new_target = wrap_angle_rad(target_yaw_rad)

        if self._has_target and self._config.reset_pid_on_target_change:
            delta = abs(shortest_angular_distance_rad(self._target_yaw_rad, new_target))
            if delta >= abs(self._config.target_change_reset_threshold_rad):
                self._pid.reset()

        self._target_yaw_rad = new_target
        self._has_target = True

    def capture_hold_target(self, current_yaw_rad: float) -> None:
        """Capture current yaw as target (heading hold behavior)."""
        if not isfinite(current_yaw_rad):
            return

        self._target_yaw_rad = wrap_angle_rad(current_yaw_rad)
        self._has_target = True

        if self._config.reset_pid_on_hold_capture:
            self._pid.reset()

    def clear_target(self) -> None:
        """Clear target and reset controller state."""
        self._has_target = False
        self._target_yaw_rad = 0.0
        self._pid.reset()

    # -------------------------------------------------------------------------
    # State management
    # -------------------------------------------------------------------------
    def reset(self) -> None:
        self._pid.reset()

    # -------------------------------------------------------------------------
    # Goal checks
    # -------------------------------------------------------------------------
    def is_within_tolerance(self, current_yaw_rad: float) -> bool:
        if (not self._has_target) or (not isfinite(current_yaw_rad)):
            return False

        err = shortest_angular_distance_rad(wrap_angle_rad(current_yaw_rad), self._target_yaw_rad)
        return abs(err) <= abs(self._config.heading_tolerance_rad)

    # -------------------------------------------------------------------------
    # Main update (uses stored target)
    # -------------------------------------------------------------------------
    def update(self, current_yaw_rad: float, dt_sec: float) -> HeadingControllerResult:
        out = HeadingControllerResult()
        out.current_yaw_rad = current_yaw_rad if isfinite(current_yaw_rad) else 0.0
        out.dt_valid = self._valid_dt(dt_sec)
        out.yaw_valid = isfinite(current_yaw_rad)
        out.has_target = self._has_target
        out.valid = False

        if not out.yaw_valid:
            # Safe invalid yaw behavior
            out.wz_cmd_rad_s = 0.0
            return out

        current_wrapped = wrap_angle_rad(current_yaw_rad)
        out.current_yaw_rad = current_wrapped

        if not self._has_target:
            out.wz_cmd_rad_s = 0.0
            return out

        out.target_yaw_rad = self._target_yaw_rad

        # Positive => rotate CCW, Negative => rotate CW
        out.error_yaw_rad = shortest_angular_distance_rad(current_wrapped, self._target_yaw_rad)

        out.within_tolerance = abs(out.error_yaw_rad) <= abs(self._config.heading_tolerance_rad)

        # PID update on wrapped angular error
        out.pid = self._pid.update(out.error_yaw_rad, dt_sec)

        if not out.pid.valid:
            out.wz_cmd_rad_s = 0.0
            return out

        wz_cmd = out.pid.output

        # Optional external clamp (after PID output clamp)
        if self._config.max_wz_rad_s > 0.0 and isfinite(self._config.max_wz_rad_s):
            wz_cmd = saturate_abs(wz_cmd, abs(self._config.max_wz_rad_s))

        # Optional min effective command (anti-stiction)
        if self._config.min_effective_wz_rad_s > 0.0 and isfinite(self._config.min_effective_wz_rad_s):
            min_eff = abs(self._config.min_effective_wz_rad_s)
            if abs(wz_cmd) > 0.0 and abs(wz_cmd) < min_eff:
                wz_cmd = copy_sign(min_eff, wz_cmd)

        # Optional output deadband
        wz_cmd = apply_deadband(wz_cmd, self._config.output_deadband_rad_s)

        # Safe default: stop rotation if already within tolerance
        if out.within_tolerance:
            wz_cmd = 0.0

        out.wz_cmd_rad_s = wz_cmd
        out.valid = out.dt_valid and out.yaw_valid and out.pid.valid
        return out

    # -------------------------------------------------------------------------
    # Convenience update with explicit temporary target (non-persistent)
    # -------------------------------------------------------------------------
    def update_to_target(
        self,
        current_yaw_rad: float,
        target_yaw_rad: float,
        dt_sec: float,
    ) -> HeadingControllerResult:
        """
        Temporary target update path:
        - does NOT mutate stored target state
        - does NOT call set_target_yaw()
        - avoids target-change PID reset side-effects
        """
        out = HeadingControllerResult()
        out.current_yaw_rad = current_yaw_rad if isfinite(current_yaw_rad) else 0.0
        out.dt_valid = self._valid_dt(dt_sec)
        out.yaw_valid = isfinite(current_yaw_rad)
        out.has_target = isfinite(target_yaw_rad)
        out.valid = False

        if (not out.yaw_valid) or (not isfinite(target_yaw_rad)):
            out.wz_cmd_rad_s = 0.0
            out.has_target = False
            return out

        current_wrapped = wrap_angle_rad(current_yaw_rad)
        target_wrapped = wrap_angle_rad(target_yaw_rad)

        out.current_yaw_rad = current_wrapped
        out.target_yaw_rad = target_wrapped
        out.error_yaw_rad = shortest_angular_distance_rad(current_wrapped, target_wrapped)

        out.within_tolerance = abs(out.error_yaw_rad) <= abs(self._config.heading_tolerance_rad)

        # Direct PID update on wrapped error (no stored target mutation)
        out.pid = self._pid.update(out.error_yaw_rad, dt_sec)

        if not out.pid.valid:
            out.wz_cmd_rad_s = 0.0
            return out

        wz_cmd = out.pid.output

        if self._config.max_wz_rad_s > 0.0 and isfinite(self._config.max_wz_rad_s):
            wz_cmd = saturate_abs(wz_cmd, abs(self._config.max_wz_rad_s))

        if self._config.min_effective_wz_rad_s > 0.0 and isfinite(self._config.min_effective_wz_rad_s):
            min_eff = abs(self._config.min_effective_wz_rad_s)
            if abs(wz_cmd) > 0.0 and abs(wz_cmd) < min_eff:
                wz_cmd = copy_sign(min_eff, wz_cmd)

        wz_cmd = apply_deadband(wz_cmd, self._config.output_deadband_rad_s)

        if out.within_tolerance:
            wz_cmd = 0.0

        out.wz_cmd_rad_s = wz_cmd
        out.valid = out.dt_valid and out.yaw_valid and out.pid.valid
        return out

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _valid_dt(self, dt_sec: float) -> bool:
        return (
            isfinite(dt_sec)
            and dt_sec >= self._config.min_dt_sec
            and dt_sec <= self._config.max_dt_sec
        )

    def _normalize_config(self) -> None:
        c = self._config

        if (not isfinite(c.heading_tolerance_rad)) or (c.heading_tolerance_rad < 0.0):
            c.heading_tolerance_rad = 0.03

        if (not isfinite(c.output_deadband_rad_s)) or (c.output_deadband_rad_s < 0.0):
            c.output_deadband_rad_s = 0.0

        if (not isfinite(c.min_effective_wz_rad_s)) or (c.min_effective_wz_rad_s < 0.0):
            c.min_effective_wz_rad_s = 0.0

        if not isfinite(c.max_wz_rad_s):
            c.max_wz_rad_s = 0.0
        elif c.max_wz_rad_s < 0.0:
            c.max_wz_rad_s = abs(c.max_wz_rad_s)

        if (not isfinite(c.target_change_reset_threshold_rad)) or (c.target_change_reset_threshold_rad < 0.0):
            c.target_change_reset_threshold_rad = 0.02

        if (not isfinite(c.min_dt_sec)) or (c.min_dt_sec <= 0.0):
            c.min_dt_sec = 1e-6

        if (not isfinite(c.max_dt_sec)) or (c.max_dt_sec < c.min_dt_sec):
            c.max_dt_sec = max(1.0, c.min_dt_sec)


# =============================================================================
# Backward-friendly alias
# =============================================================================

HeadingPid = HeadingController


# =============================================================================
# Self-test (no ROS dependency)
# =============================================================================

if __name__ == "__main__":
    # Example cautious config for tuning
    pid_cfg = PidConfig(kp=1.5, ki=0.0, kd=0.05)
    pid_cfg.set_output_symmetric(0.6)  # rad/s

    cfg = HeadingControllerConfig(
        pid=pid_cfg,
        heading_tolerance_rad=0.03,
        output_deadband_rad_s=0.01,
        min_effective_wz_rad_s=0.05,
        max_wz_rad_s=0.5,
        reset_pid_on_hold_capture=True,
        reset_pid_on_target_change=True,
        target_change_reset_threshold_rad=0.02,
        min_dt_sec=1e-4,
        max_dt_sec=0.5,
    )

    ctrl = HeadingController(cfg)
    ctrl.set_target_yaw(math.radians(90.0))  # +pi/2 target

    print("=== heading_pid_py.py self-test ===")
    dt = 0.05
    # Simulated current yaw values approaching target
    yaw_list = [
        math.radians(0),
        math.radians(10),
        math.radians(25),
        math.radians(40),
        math.radians(55),
        math.radians(70),
        math.radians(80),
        math.radians(86),
        math.radians(89),
        math.radians(90),
    ]

    for i, yaw in enumerate(yaw_list, start=1):
        r = ctrl.update(yaw, dt)
        print(
            f"step={i:02d} yaw={math.degrees(r.current_yaw_rad):6.1f}deg "
            f"target={math.degrees(r.target_yaw_rad):6.1f}deg "
            f"err={math.degrees(r.error_yaw_rad):+6.2f}deg "
            f"wz={r.wz_cmd_rad_s:+.3f} rad/s "
            f"tol={int(r.within_tolerance)} valid={int(r.valid)}"
        )