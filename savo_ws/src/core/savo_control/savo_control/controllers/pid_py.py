#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Scalar PID controller for Python fallback, tests, and tuning tools."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf, isfinite

from savo_control.utils import clamp, within_tolerance


@dataclass
class PidConfig:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0

    output_min: float = -inf
    output_max: float = inf
    integral_clamp: float = inf

    d_filter_alpha: float = 0.0

    min_dt_sec: float = 1e-6
    max_dt_sec: float = 1.0

    freeze_integral_on_invalid_dt: bool = True
    derivative_on_measurement: bool = False

    def set_output_symmetric(self, abs_limit: float) -> None:
        limit = abs(float(abs_limit)) if isfinite(float(abs_limit)) else inf
        self.output_min = -limit
        self.output_max = limit

    def set_integral_clamp_symmetric(self, abs_limit: float) -> None:
        self.integral_clamp = abs(float(abs_limit)) if isfinite(float(abs_limit)) else inf


@dataclass
class PidResult:
    output: float = 0.0
    output_raw: float = 0.0

    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0

    error: float = 0.0
    integral_state: float = 0.0
    derivative_state: float = 0.0
    dt_sec: float = 0.0

    valid: bool = False
    dt_valid: bool = False
    saturated: bool = False

    def to_dict(self) -> dict:
        return {
            "output": self.output,
            "output_raw": self.output_raw,
            "p_term": self.p_term,
            "i_term": self.i_term,
            "d_term": self.d_term,
            "error": self.error,
            "integral_state": self.integral_state,
            "derivative_state": self.derivative_state,
            "dt_sec": self.dt_sec,
            "valid": self.valid,
            "dt_valid": self.dt_valid,
            "saturated": self.saturated,
        }


class Pid:
    """Small scalar PID controller aligned with the C++ production PID contract."""

    def __init__(self, config: PidConfig | None = None) -> None:
        self._config = config if config is not None else PidConfig()
        self._normalize_config()

        self._integral_state = 0.0
        self._prev_error = 0.0
        self._derivative_state = 0.0
        self._has_prev_error = False

        self._last_output = 0.0
        self._has_last_output = False

    def set_config(self, config: PidConfig) -> None:
        self._config = config
        self._normalize_config()

    def config(self) -> PidConfig:
        return self._config

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self._config.kp = float(kp) if isfinite(float(kp)) else 0.0
        self._config.ki = float(ki) if isfinite(float(ki)) else 0.0
        self._config.kd = float(kd) if isfinite(float(kd)) else 0.0
        self._normalize_config()

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
        self._has_prev_error = isfinite(error)

        self._last_output = float(output) if isfinite(output) else 0.0
        self._has_last_output = isfinite(output)

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

    def update(self, error: float, dt_sec: float) -> PidResult:
        self._normalize_config()

        result = PidResult()
        result.error = float(error) if error is not None else float("nan")
        result.dt_sec = float(dt_sec) if dt_sec is not None else float("nan")

        error_valid = isfinite(result.error)
        result.dt_valid = self._valid_dt(result.dt_sec)

        if not error_valid:
            self._last_output = 0.0
            self._has_last_output = True
            return result

        result.p_term = self._config.kp * result.error

        if result.dt_valid:
            self._integral_state += result.error * result.dt_sec

            if isfinite(self._config.integral_clamp):
                limit = abs(self._config.integral_clamp)
                self._integral_state = clamp(self._integral_state, -limit, limit)

        result.integral_state = self._integral_state
        result.i_term = self._config.ki * self._integral_state

        if result.dt_valid and self._has_prev_error:
            derivative_raw = (result.error - self._prev_error) / result.dt_sec
        else:
            derivative_raw = 0.0

        alpha = clamp(self._config.d_filter_alpha, 0.0, 1.0)

        if not self._has_prev_error or not result.dt_valid:
            self._derivative_state = 0.0
        elif alpha <= 0.0:
            self._derivative_state = derivative_raw
        else:
            self._derivative_state = (
                (1.0 - alpha) * derivative_raw
                + alpha * self._derivative_state
            )

        result.derivative_state = self._derivative_state
        result.d_term = self._config.kd * self._derivative_state

        result.output_raw = result.p_term + result.i_term + result.d_term

        output_min = min(self._config.output_min, self._config.output_max)
        output_max = max(self._config.output_min, self._config.output_max)

        result.output = clamp(result.output_raw, output_min, output_max)
        result.saturated = not within_tolerance(result.output, result.output_raw, 1e-12)

        self._prev_error = result.error
        self._has_prev_error = True

        self._last_output = result.output
        self._has_last_output = True

        result.valid = True
        return result

    def update_from_setpoint(
        self,
        setpoint: float,
        measurement: float,
        dt_sec: float,
    ) -> PidResult:
        try:
            error = float(setpoint) - float(measurement)
        except (TypeError, ValueError):
            error = float("nan")

        return self.update(error, dt_sec)

    def _valid_dt(self, dt_sec: float) -> bool:
        return (
            isfinite(dt_sec)
            and self._config.min_dt_sec <= dt_sec <= self._config.max_dt_sec
        )

    def _normalize_config(self) -> None:
        cfg = self._config

        cfg.kp = cfg.kp if isfinite(cfg.kp) else 0.0
        cfg.ki = cfg.ki if isfinite(cfg.ki) else 0.0
        cfg.kd = cfg.kd if isfinite(cfg.kd) else 0.0

        if cfg.output_min > cfg.output_max:
            cfg.output_min, cfg.output_max = cfg.output_max, cfg.output_min

        if not isfinite(cfg.integral_clamp):
            cfg.integral_clamp = inf
        else:
            cfg.integral_clamp = abs(cfg.integral_clamp)

        cfg.d_filter_alpha = (
            clamp(cfg.d_filter_alpha, 0.0, 1.0)
            if isfinite(cfg.d_filter_alpha)
            else 0.0
        )

        if not isfinite(cfg.min_dt_sec) or cfg.min_dt_sec <= 0.0:
            cfg.min_dt_sec = 1e-6

        if not isfinite(cfg.max_dt_sec) or cfg.max_dt_sec < cfg.min_dt_sec:
            cfg.max_dt_sec = max(1.0, cfg.min_dt_sec)


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

    print("=== pid_py.py self-test ===")
    for i, error in enumerate([1.0, 0.8, 0.6, 0.45, 0.3, 0.2, 0.1, 0.0], start=1):
        result = pid.update(error, 0.05)
        print(
            f"step={i:02d} e={result.error:+.3f} "
            f"P={result.p_term:+.3f} I={result.i_term:+.3f} "
            f"D={result.d_term:+.3f} raw={result.output_raw:+.3f} "
            f"out={result.output:+.3f} sat={int(result.saturated)} "
            f"valid={int(result.valid)}"
        )
