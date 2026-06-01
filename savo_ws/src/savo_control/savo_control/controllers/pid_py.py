#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control.controllers.pid_py
============================================

Purpose
-------
ROS-independent scalar PID controller utility for Python-side tools and wrappers,
aligned with the C++ `pid.hpp` / `pid.cpp` implementation.

Intended for
------------
- heading_pid_py.py
- distance_pid_py.py
- Python test/tuning nodes
- diagnostics dashboards / offline analysis

Important
---------
The C++ PID remains the authoritative production/runtime implementation.
This Python module mirrors behavior for testing and tooling, and should not
replace the real-time C++ control path on the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
import math


# =============================================================================
# PID parameter set (mirrors C++ PidConfig)
# =============================================================================

@dataclass
class PidConfig:
    # Gains
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    # Output clamp
    output_min: float = -math.inf
    output_max: float = math.inf

    # Integral state clamp (absolute value applied internally)
    integral_clamp: float = math.inf

    # Derivative low-pass filter coefficient [0, 1]
    # 0.0 -> raw derivative
    # 1.0 -> very heavy filtering
    d_filter_alpha: float = 0.0

    # dt validity bounds
    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0

    # If True, integral pauses on invalid dt
    freeze_integral_on_invalid_dt: bool = True

    # Reserved for compatibility with C++ config (not used in current implementation)
    derivative_on_measurement: bool = False

    # Helper utilities (same intent as C++)
    def set_output_symmetric(self, abs_limit: float) -> None:
        a = abs(float(abs_limit))
        self.output_min = -a
        self.output_max = a

    def set_integral_clamp_symmetric(self, abs_limit: float) -> None:
        self.integral_clamp = abs(float(abs_limit))


# =============================================================================
# PID debug/result output bundle (mirrors C++ PidResult)
# =============================================================================

@dataclass
class PidResult:
    # Final saturated output
    output: float = 0.0

    # Unsaturated raw sum (P+I+D)
    output_raw: float = 0.0

    # Terms
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0

    # Diagnostics/internal states
    error: float = 0.0
    integral_state: float = 0.0
    derivative_state: float = 0.0  # filtered derivative used for d_term
    dt_sec: float = 0.0

    # Flags
    valid: bool = False
    dt_valid: bool = False
    saturated: bool = False


# =============================================================================
# Scalar PID controller (Python mirror of C++ Pid)
# =============================================================================

class Pid:
    """
    Scalar PID controller aligned with Robot SAVO C++ pid.hpp.

    Caller responsibilities (same as C++):
    - compute the correct scalar error (e.g., wrapped angle error)
    - provide dt_sec from the loop timing
    - apply robot-specific safety / mode gating in higher layers
    """

    def __init__(self, config: PidConfig | None = None) -> None:
        self._config = config if config is not None else PidConfig()
        self._normalize_config()

        # Internal state (mirrors C++)
        self._integral_state = 0.0
        self._prev_error = 0.0
        self._derivative_state = 0.0

        self._has_prev_error = False

        self._last_output = 0.0
        self._has_last_output = False

    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------
    def set_config(self, config: PidConfig) -> None:
        self._config = config
        self._normalize_config()

    def config(self) -> PidConfig:
        return self._config

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self._config.kp = float(kp)
        self._config.ki = float(ki)
        self._config.kd = float(kd)
        self._normalize_config()

    # -------------------------------------------------------------------------
    # State management
    # -------------------------------------------------------------------------
    def reset(self) -> None:
        self._integral_state = 0.0
        self._prev_error = 0.0
        self._derivative_state = 0.0
        self._has_prev_error = False
        self._last_output = 0.0
        self._has_last_output = False

    def reset_to(self, error: float, output: float = 0.0) -> None:
        self._integral_state = 0.0
        self._prev_error = float(error) if isfinite(error) else 0.0
        self._derivative_state = 0.0
        self._has_prev_error = bool(isfinite(error))
        self._last_output = float(output) if isfinite(output) else 0.0
        self._has_last_output = bool(isfinite(output))

    def has_previous_error(self) -> bool:
        return self._has_prev_error

    def previous_error(self) -> float:
        return self._prev_error

    def integral_state(self) -> float:
        return self._integral_state

    def derivative_state(self) -> float:
        return self._derivative_state

    def has_last_output(self) -> bool:
        return self._has_last_output

    def last_output(self) -> float:
        return self._last_output

    # -------------------------------------------------------------------------
    # Compute PID output from error
    # -------------------------------------------------------------------------
    def update(self, error: float, dt_sec: float) -> PidResult:
        self._normalize_config()

        r = PidResult()
        r.error = float(error) if error is not None else float("nan")
        r.dt_sec = float(dt_sec) if dt_sec is not None else float("nan")

        error_ok = isfinite(r.error)
        dt_ok = self._valid_dt(r.dt_sec)
        r.dt_valid = dt_ok

        if not error_ok:
            # Safe invalid-input behavior (aligned with C++)
            r.output = 0.0
            r.output_raw = 0.0
            r.valid = False
            self._last_output = r.output
            self._has_last_output = True
            return r

        # P term
        r.p_term = self._config.kp * r.error

        # Integral term
        if dt_ok:
            self._integral_state += r.error * r.dt_sec

            i_clamp = abs(self._config.integral_clamp)
            if isfinite(i_clamp):
                self._integral_state = self._clamp(self._integral_state, -i_clamp, i_clamp)
        elif self._config.freeze_integral_on_invalid_dt:
            # Hold integral as-is
            pass

        r.integral_state = self._integral_state
        r.i_term = self._config.ki * self._integral_state

        # Derivative term (on error)
        if dt_ok and self._has_prev_error:
            d_raw = (r.error - self._prev_error) / r.dt_sec
        else:
            d_raw = 0.0

        # Optional derivative low-pass filtering
        alpha = self._clamp(self._config.d_filter_alpha, 0.0, 1.0)
        if (not self._has_prev_error) or (not dt_ok):
            # Reset derivative state on startup / invalid dt to avoid spikes
            self._derivative_state = 0.0
        elif alpha <= 0.0:
            self._derivative_state = d_raw
        else:
            # Exponential smoothing: new = (1-alpha)*raw + alpha*prev
            self._derivative_state = (1.0 - alpha) * d_raw + alpha * self._derivative_state

        r.derivative_state = self._derivative_state
        r.d_term = self._config.kd * self._derivative_state

        # Raw sum
        r.output_raw = r.p_term + r.i_term + r.d_term

        # Output clamp
        out_min = min(self._config.output_min, self._config.output_max)
        out_max = max(self._config.output_min, self._config.output_max)
        r.output = self._clamp(r.output_raw, out_min, out_max)
        r.saturated = not self._nearly_equal(r.output, r.output_raw)

        # Update states
        self._prev_error = r.error
        self._has_prev_error = True

        self._last_output = r.output
        self._has_last_output = True

        r.valid = True
        return r

    # Convenience method (same semantics as C++)
    def update_from_setpoint(self, setpoint: float, measurement: float, dt_sec: float) -> PidResult:
        error = float(setpoint) - float(measurement)
        return self.update(error, dt_sec)

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        if min_value > max_value:
            min_value, max_value = max_value, min_value
        return max(min_value, min(max_value, value))

    @staticmethod
    def _nearly_equal(a: float, b: float, eps: float = 1e-12) -> bool:
        return abs(a - b) <= eps

    def _valid_dt(self, dt_sec: float) -> bool:
        return (
            isfinite(dt_sec)
            and dt_sec >= self._config.min_dt_sec
            and dt_sec <= self._config.max_dt_sec
        )

    def _normalize_config(self) -> None:
        c = self._config

        # Gains
        if not isfinite(c.kp):
            c.kp = 0.0
        if not isfinite(c.ki):
            c.ki = 0.0
        if not isfinite(c.kd):
            c.kd = 0.0

        # Output clamps
        if not isfinite(c.output_min):
            c.output_min = -math.inf
        if not isfinite(c.output_max):
            c.output_max = math.inf
        if c.output_min > c.output_max:
            c.output_min, c.output_max = c.output_max, c.output_min

        # Integral clamp
        if not isfinite(c.integral_clamp):
            c.integral_clamp = math.inf
        else:
            c.integral_clamp = abs(c.integral_clamp)

        # D filter alpha
        if not isfinite(c.d_filter_alpha):
            c.d_filter_alpha = 0.0
        c.d_filter_alpha = self._clamp(c.d_filter_alpha, 0.0, 1.0)

        # dt bounds
        if (not isfinite(c.min_dt_sec)) or (c.min_dt_sec <= 0.0):
            c.min_dt_sec = 1e-6
        if (not isfinite(c.max_dt_sec)) or (c.max_dt_sec < c.min_dt_sec):
            c.max_dt_sec = max(1.0, c.min_dt_sec)


# =============================================================================
# Self-test (no ROS dependency)
# =============================================================================

if __name__ == "__main__":
    cfg = PidConfig(
        kp=1.0,
        ki=0.2,
        kd=0.05,
        output_min=-0.5,
        output_max=0.5,
        integral_clamp=1.0,
        d_filter_alpha=0.2,
        min_dt_sec=1e-4,
        max_dt_sec=0.5,
    )

    pid = Pid(cfg)

    print("=== pid_py.py self-test (C++-aligned) ===")
    dt = 0.05
    errors = [1.0, 0.8, 0.6, 0.45, 0.3, 0.2, 0.1, 0.05, 0.0]

    for i, e in enumerate(errors, start=1):
        r = pid.update(e, dt)
        print(
            f"step={i:02d} e={r.error:+.3f} dt={r.dt_sec:.3f} "
            f"P={r.p_term:+.3f} I={r.i_term:+.3f} D={r.d_term:+.3f} "
            f"raw={r.output_raw:+.3f} out={r.output:+.3f} "
            f"sat={int(r.saturated)} valid={int(r.valid)}"
        )