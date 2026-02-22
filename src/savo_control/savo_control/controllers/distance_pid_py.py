#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control.controllers.distance_pid_py
=====================================================

Purpose
-------
Python distance/approach controller helper built on top of `pid_py.Pid`,
aligned with the design style used in C++ reusable controllers.

Intended for
------------
- distance approach test nodes
- docking/alignment experiments (linear axis)
- diagnostics/tuning tools
- Python prototyping before/alongside C++ runtime implementations

Important
---------
- ROS-independent (pure Python)
- Uses the generic PID core from `pid_py.py`
- C++ runtime controllers remain authoritative for production robot control
- Higher-level nodes are responsible for safety gating and final command routing

Behavior
--------
- Target distance management (set/clear)
- Error = target_distance - current_distance
- PID-based linear velocity command generation (vx_cmd_m_s)
- Goal tolerance checks (within distance tolerance)
- Optional output deadband / min effective command / max clamp
- Optional direction constraints:
    * bidirectional (default)
    * approach_from_far_only  (no reverse command if too close)
    * back_away_only          (no forward command if too far)
- Optional zero command when within tolerance (safe default)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Optional
import math

from .pid_py import Pid, PidConfig, PidResult


# =============================================================================
# Config / Result
# =============================================================================

@dataclass
class DistanceControllerConfig:
    # Underlying scalar PID config (output typically maps to linear.x [m/s])
    pid: PidConfig = field(default_factory=PidConfig)

    # Goal tolerance for "at target" checks (meters)
    distance_tolerance_m: float = 0.03  # 3 cm default

    # Final output cleanup deadband (m/s)
    output_deadband_m_s: float = 0.0

    # Optional minimum effective speed magnitude (anti-stiction helper)
    # Disabled if <= 0
    min_effective_vx_m_s: float = 0.0

    # Optional additional external clamp after PID output (m/s)
    # Disabled if <= 0 (PID output clamp alone applies)
    max_vx_m_s: float = 0.0

    # Safe default behavior: if within tolerance -> command zero
    zero_output_within_tolerance: bool = True

    # Target-change behavior
    reset_pid_on_target_change: bool = True
    target_change_reset_threshold_m: float = 0.01  # 1 cm

    # Controller-level dt validity (PID also validates dt)
    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0

    # Direction constraint policy
    #
    # "bidirectional":
    #   allow positive and negative output (approach or back away)
    #
    # "approach_from_far_only":
    #   allow only motion that reduces positive error when target is closer than current.
    #   With error = target - current:
    #     if current > target => error < 0 (need to move toward object / reduce distance)
    #   This policy forbids the opposite sign command if generated.
    #
    # "back_away_only":
    #   allow only motion that increases distance (useful for retreat tests)
    #
    # NOTE:
    # Exact physical sign of forward/backward depends on robot convention.
    # This wrapper assumes PID output sign convention is already correct for your node.
    direction_mode: str = "bidirectional"


@dataclass
class DistanceControllerResult:
    # Final commanded linear velocity (e.g., Twist.linear.x)
    vx_cmd_m_s: float = 0.0

    # Diagnostics
    target_distance_m: float = 0.0
    current_distance_m: float = 0.0
    error_distance_m: float = 0.0

    # Goal checks
    has_target: bool = False
    within_tolerance: bool = False

    # Validity
    valid: bool = False
    dt_valid: bool = False
    distance_valid: bool = False

    # PID breakdown
    pid: PidResult = field(default_factory=PidResult)

    # Extra diagnostics
    direction_limited: bool = False


# =============================================================================
# Distance PID wrapper
# =============================================================================

class DistancePid:
    """
    Distance/approach controller built on top of the generic scalar PID.

    Sign convention
    ---------------
    This class computes:
        error = target_distance_m - current_distance_m

    The sign of resulting command (vx_cmd_m_s) depends on your robot/node convention.
    Tune/use this wrapper in the same sign convention as your velocity consumer.
    """

    VALID_DIRECTION_MODES = {
        "bidirectional",
        "approach_from_far_only",
        "back_away_only",
    }

    def __init__(self, cfg: Optional[DistanceControllerConfig] = None) -> None:
        self._config = cfg if cfg is not None else DistanceControllerConfig()
        self._normalize_config()

        self._pid = Pid(self._config.pid)
        self._pid.set_config(self._config.pid)

        self._has_target = False
        self._target_distance_m = 0.0

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------
    def set_config(self, cfg: DistanceControllerConfig) -> None:
        self._config = cfg
        self._normalize_config()
        self._pid.set_config(self._config.pid)

    @property
    def config(self) -> DistanceControllerConfig:
        return self._config

    def pid(self) -> Pid:
        return self._pid

    # -------------------------------------------------------------------------
    # Target management
    # -------------------------------------------------------------------------
    def has_target(self) -> bool:
        return self._has_target

    def target_distance_m(self) -> float:
        return self._target_distance_m

    def set_target_distance(self, target_distance_m: float) -> None:
        """Set target distance (meters). Ignores non-finite values."""
        if not isfinite(target_distance_m):
            return

        new_target = float(target_distance_m)

        if self._has_target and self._config.reset_pid_on_target_change:
            delta = abs(new_target - self._target_distance_m)
            if delta >= abs(self._config.target_change_reset_threshold_m):
                self._pid.reset()

        self._target_distance_m = new_target
        self._has_target = True

    def clear_target(self) -> None:
        """Clear target and reset PID."""
        self._has_target = False
        self._target_distance_m = 0.0
        self._pid.reset()

    # -------------------------------------------------------------------------
    # State management
    # -------------------------------------------------------------------------
    def reset(self) -> None:
        self._pid.reset()

    # -------------------------------------------------------------------------
    # Goal checks
    # -------------------------------------------------------------------------
    def is_within_tolerance(self, current_distance_m: float) -> bool:
        if (not self._has_target) or (not isfinite(current_distance_m)):
            return False
        err = self._target_distance_m - float(current_distance_m)
        return abs(err) <= abs(self._config.distance_tolerance_m)

    # -------------------------------------------------------------------------
    # Main update (uses stored target)
    # -------------------------------------------------------------------------
    def update(self, current_distance_m: float, dt_sec: float) -> DistanceControllerResult:
        out = DistanceControllerResult()
        out.current_distance_m = float(current_distance_m) if isfinite(current_distance_m) else 0.0
        out.dt_valid = self._valid_dt(dt_sec)
        out.distance_valid = isfinite(current_distance_m)
        out.has_target = self._has_target
        out.valid = False

        if not out.distance_valid:
            out.vx_cmd_m_s = 0.0
            return out

        if not self._has_target:
            out.vx_cmd_m_s = 0.0
            return out

        out.target_distance_m = self._target_distance_m
        out.error_distance_m = self._target_distance_m - out.current_distance_m
        out.within_tolerance = abs(out.error_distance_m) <= abs(self._config.distance_tolerance_m)

        out.pid = self._pid.update(out.error_distance_m, dt_sec)

        if not out.pid.valid:
            out.vx_cmd_m_s = 0.0
            return out

        vx_cmd = out.pid.output

        # Optional external clamp (after PID output clamp)
        if self._config.max_vx_m_s > 0.0 and isfinite(self._config.max_vx_m_s):
            vx_cmd = self._saturate_abs(vx_cmd, abs(self._config.max_vx_m_s))

        # Direction policy limiting (before min_effective/deadband final shaping)
        vx_cmd, limited = self._apply_direction_policy(vx_cmd, out.error_distance_m)
        out.direction_limited = limited

        # Optional min effective command (anti-stiction)
        if self._config.min_effective_vx_m_s > 0.0 and isfinite(self._config.min_effective_vx_m_s):
            min_eff = abs(self._config.min_effective_vx_m_s)
            if abs(vx_cmd) > 0.0 and abs(vx_cmd) < min_eff:
                vx_cmd = math.copysign(min_eff, vx_cmd)

        # Optional final deadband
        vx_cmd = self._apply_deadband(vx_cmd, self._config.output_deadband_m_s)

        # Safe default near-goal behavior
        if self._config.zero_output_within_tolerance and out.within_tolerance:
            vx_cmd = 0.0

        out.vx_cmd_m_s = vx_cmd
        out.valid = out.dt_valid and out.distance_valid and out.pid.valid
        return out

    # -------------------------------------------------------------------------
    # Convenience update with explicit temporary target (non-persistent)
    # -------------------------------------------------------------------------
    def update_to_target(
        self,
        current_distance_m: float,
        target_distance_m: float,
        dt_sec: float,
    ) -> DistanceControllerResult:
        """
        Temporary target update path:
        - does NOT mutate stored target state
        - avoids target-change reset side-effects
        """
        out = DistanceControllerResult()
        out.current_distance_m = float(current_distance_m) if isfinite(current_distance_m) else 0.0
        out.dt_valid = self._valid_dt(dt_sec)
        out.distance_valid = isfinite(current_distance_m)
        out.has_target = isfinite(target_distance_m)
        out.valid = False

        if (not out.distance_valid) or (not isfinite(target_distance_m)):
            out.vx_cmd_m_s = 0.0
            out.has_target = False
            return out

        out.target_distance_m = float(target_distance_m)
        out.error_distance_m = out.target_distance_m - out.current_distance_m
        out.within_tolerance = abs(out.error_distance_m) <= abs(self._config.distance_tolerance_m)

        out.pid = self._pid.update(out.error_distance_m, dt_sec)

        if not out.pid.valid:
            out.vx_cmd_m_s = 0.0
            return out

        vx_cmd = out.pid.output

        if self._config.max_vx_m_s > 0.0 and isfinite(self._config.max_vx_m_s):
            vx_cmd = self._saturate_abs(vx_cmd, abs(self._config.max_vx_m_s))

        vx_cmd, limited = self._apply_direction_policy(vx_cmd, out.error_distance_m)
        out.direction_limited = limited

        if self._config.min_effective_vx_m_s > 0.0 and isfinite(self._config.min_effective_vx_m_s):
            min_eff = abs(self._config.min_effective_vx_m_s)
            if abs(vx_cmd) > 0.0 and abs(vx_cmd) < min_eff:
                vx_cmd = math.copysign(min_eff, vx_cmd)

        vx_cmd = self._apply_deadband(vx_cmd, self._config.output_deadband_m_s)

        if self._config.zero_output_within_tolerance and out.within_tolerance:
            vx_cmd = 0.0

        out.vx_cmd_m_s = vx_cmd
        out.valid = out.dt_valid and out.distance_valid and out.pid.valid
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

        if (not isfinite(c.distance_tolerance_m)) or (c.distance_tolerance_m < 0.0):
            c.distance_tolerance_m = 0.03

        if (not isfinite(c.output_deadband_m_s)) or (c.output_deadband_m_s < 0.0):
            c.output_deadband_m_s = 0.0

        if (not isfinite(c.min_effective_vx_m_s)) or (c.min_effective_vx_m_s < 0.0):
            c.min_effective_vx_m_s = 0.0

        if not isfinite(c.max_vx_m_s):
            c.max_vx_m_s = 0.0
        elif c.max_vx_m_s < 0.0:
            c.max_vx_m_s = abs(c.max_vx_m_s)

        if (not isfinite(c.target_change_reset_threshold_m)) or (c.target_change_reset_threshold_m < 0.0):
            c.target_change_reset_threshold_m = 0.01

        if (not isfinite(c.min_dt_sec)) or (c.min_dt_sec <= 0.0):
            c.min_dt_sec = 1e-6

        if (not isfinite(c.max_dt_sec)) or (c.max_dt_sec < c.min_dt_sec):
            c.max_dt_sec = max(1.0, c.min_dt_sec)

        if c.direction_mode not in self.VALID_DIRECTION_MODES:
            c.direction_mode = "bidirectional"

    @staticmethod
    def _saturate_abs(value: float, abs_limit: float) -> float:
        if not isfinite(value):
            return 0.0
        if (not isfinite(abs_limit)) or abs_limit <= 0.0:
            return value
        lim = abs(abs_limit)
        return max(-lim, min(lim, value))

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        if not isfinite(value):
            return 0.0
        db = abs(deadband) if isfinite(deadband) else 0.0
        if db <= 0.0:
            return value
        return 0.0 if abs(value) < db else value

    def _apply_direction_policy(self, vx_cmd: float, error_distance_m: float) -> tuple[float, bool]:
        """
        Apply optional direction constraints.

        We treat the PID output sign as already meaningful for the robot's
        configured sign convention. Policies only block the "opposite" direction.

        Returns:
            (vx_cmd_after_policy, direction_limited_flag)
        """
        mode = self._config.direction_mode

        if mode == "bidirectional":
            return vx_cmd, False

        # Determine the "helpful" sign from error:
        # if error > 0, helpful output should be positive (increase command in positive direction)
        # if error < 0, helpful output should be negative
        # if error == 0, command should be zeroed by tolerance/deadband anyway
        if not isfinite(vx_cmd) or not isfinite(error_distance_m):
            return 0.0, True

        if abs(error_distance_m) <= 0.0:
            return 0.0, abs(vx_cmd) > 0.0

        helpful_sign = 1.0 if error_distance_m > 0.0 else -1.0
        cmd_sign = 0.0 if vx_cmd == 0.0 else (1.0 if vx_cmd > 0.0 else -1.0)

        # "approach_from_far_only":
        # only allow command when current distance is greater than target (error < 0)
        # i.e., only allow negative helpful sign according to this error convention.
        if mode == "approach_from_far_only":
            if error_distance_m >= 0.0:
                # too close or exact -> do not command movement in this mode
                return 0.0, abs(vx_cmd) > 0.0
            # when error < 0, only allow negative sign command
            if cmd_sign > 0.0:
                return 0.0, True
            return vx_cmd, False

        # "back_away_only":
        # only allow command when current distance is less than target (error > 0)
        # i.e., only allow positive helpful sign
        if mode == "back_away_only":
            if error_distance_m <= 0.0:
                return 0.0, abs(vx_cmd) > 0.0
            if cmd_sign < 0.0:
                return 0.0, True
            return vx_cmd, False

        # Fallback safety
        return vx_cmd, False


# =============================================================================
# Optional alias for naming consistency
# =============================================================================

DistanceController = DistancePid


# =============================================================================
# Self-test (no ROS dependency)
# =============================================================================

if __name__ == "__main__":
    # Conservative starting config for real-robot tuning
    pid_cfg = PidConfig(kp=0.8, ki=0.0, kd=0.02)
    pid_cfg.set_output_symmetric(0.12)     # m/s
    pid_cfg.set_integral_clamp_symmetric(0.5)

    cfg = DistanceControllerConfig(
        pid=pid_cfg,
        distance_tolerance_m=0.03,
        output_deadband_m_s=0.005,
        min_effective_vx_m_s=0.02,
        max_vx_m_s=0.10,
        zero_output_within_tolerance=True,
        reset_pid_on_target_change=True,
        target_change_reset_threshold_m=0.01,
        min_dt_sec=1e-4,
        max_dt_sec=0.5,
        direction_mode="bidirectional",
    )

    ctrl = DistancePid(cfg)
    ctrl.set_target_distance(0.80)  # target distance [m]

    print("=== distance_pid_py.py self-test ===")
    dt = 0.05
    # Simulated measured distances approaching target from farther away
    distances = [1.20, 1.10, 1.00, 0.92, 0.87, 0.84, 0.82, 0.805, 0.800, 0.798]

    for i, d in enumerate(distances, start=1):
        r = ctrl.update(d, dt)
        print(
            f"step={i:02d} cur={r.current_distance_m:.3f}m "
            f"target={r.target_distance_m:.3f}m "
            f"err={r.error_distance_m:+.3f}m "
            f"vx={r.vx_cmd_m_s:+.3f}m/s "
            f"tol={int(r.within_tolerance)} "
            f"lim={int(r.direction_limited)} "
            f"valid={int(r.valid)}"
        )