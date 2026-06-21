#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for the shared Python fallback scalar PID controller."""

from __future__ import annotations

from math import inf, isclose

from savo_control.controllers.pid_py import Pid, PidConfig, PidResult


def test_pid_config_defaults_are_safe():
    cfg = PidConfig()

    assert cfg.kp == 0.0
    assert cfg.ki == 0.0
    assert cfg.kd == 0.0
    assert cfg.output_min == -inf
    assert cfg.output_max == inf
    assert cfg.integral_clamp == inf
    assert cfg.d_filter_alpha == 0.0
    assert cfg.min_dt_sec == 1e-6
    assert cfg.max_dt_sec == 1.0
    assert cfg.freeze_integral_on_invalid_dt is True
    assert cfg.derivative_on_measurement is False


def test_pid_config_set_output_symmetric():
    cfg = PidConfig()

    cfg.set_output_symmetric(0.5)

    assert cfg.output_min == -0.5
    assert cfg.output_max == 0.5


def test_pid_config_set_output_symmetric_accepts_negative_limit():
    cfg = PidConfig()

    cfg.set_output_symmetric(-0.5)

    assert cfg.output_min == -0.5
    assert cfg.output_max == 0.5


def test_pid_config_set_integral_clamp_symmetric():
    cfg = PidConfig()

    cfg.set_integral_clamp_symmetric(1.2)

    assert cfg.integral_clamp == 1.2


def test_pid_config_set_integral_clamp_symmetric_accepts_negative_limit():
    cfg = PidConfig()

    cfg.set_integral_clamp_symmetric(-1.2)

    assert cfg.integral_clamp == 1.2


def test_pid_result_defaults_and_to_dict():
    result = PidResult()

    assert result.output == 0.0
    assert result.output_raw == 0.0
    assert result.p_term == 0.0
    assert result.i_term == 0.0
    assert result.d_term == 0.0
    assert result.error == 0.0
    assert result.integral_state == 0.0
    assert result.derivative_state == 0.0
    assert result.dt_sec == 0.0
    assert result.valid is False
    assert result.dt_valid is False
    assert result.saturated is False

    assert result.to_dict() == {
        "output": 0.0,
        "output_raw": 0.0,
        "p_term": 0.0,
        "i_term": 0.0,
        "d_term": 0.0,
        "error": 0.0,
        "integral_state": 0.0,
        "derivative_state": 0.0,
        "dt_sec": 0.0,
        "valid": False,
        "dt_valid": False,
        "saturated": False,
    }


def test_pid_initial_state():
    pid = Pid()

    assert isinstance(pid.config(), PidConfig)
    assert pid.has_previous_error() is False
    assert pid.previous_error() == 0.0
    assert pid.integral_state() == 0.0
    assert pid.derivative_state() == 0.0
    assert pid.has_last_output() is False
    assert pid.last_output() == 0.0


def test_pid_p_only_output():
    cfg = PidConfig(kp=2.0, ki=0.0, kd=0.0)
    pid = Pid(cfg)

    result = pid.update(0.25, 0.05)

    assert result.valid is True
    assert result.dt_valid is True
    assert result.error == 0.25
    assert result.p_term == 0.5
    assert result.i_term == 0.0
    assert result.d_term == 0.0
    assert result.output_raw == 0.5
    assert result.output == 0.5
    assert result.saturated is False


def test_pid_output_saturation():
    cfg = PidConfig(kp=2.0, output_min=-0.5, output_max=0.5)
    pid = Pid(cfg)

    result = pid.update(1.0, 0.05)

    assert result.valid is True
    assert result.output_raw == 2.0
    assert result.output == 0.5
    assert result.saturated is True
    assert pid.has_last_output() is True
    assert pid.last_output() == 0.5


def test_pid_output_limits_are_sorted_when_reversed():
    cfg = PidConfig(kp=2.0, output_min=0.5, output_max=-0.5)
    pid = Pid(cfg)

    assert pid.config().output_min == -0.5
    assert pid.config().output_max == 0.5

    result = pid.update(1.0, 0.05)

    assert result.output == 0.5
    assert result.saturated is True


def test_pid_integral_term_accumulates_with_valid_dt():
    cfg = PidConfig(kp=0.0, ki=1.0, kd=0.0)
    pid = Pid(cfg)

    first = pid.update(1.0, 0.10)
    second = pid.update(1.0, 0.10)

    assert isclose(first.integral_state, 0.10, abs_tol=1e-9)
    assert isclose(first.i_term, 0.10, abs_tol=1e-9)

    assert isclose(second.integral_state, 0.20, abs_tol=1e-9)
    assert isclose(second.i_term, 0.20, abs_tol=1e-9)


def test_pid_integral_clamp():
    cfg = PidConfig(kp=0.0, ki=1.0, kd=0.0, integral_clamp=0.15)
    pid = Pid(cfg)

    pid.update(1.0, 0.10)
    result = pid.update(1.0, 0.10)

    assert result.integral_state == 0.15
    assert result.i_term == 0.15


def test_pid_integral_clamp_accepts_negative_value():
    cfg = PidConfig(kp=0.0, ki=1.0, kd=0.0, integral_clamp=-0.15)
    pid = Pid(cfg)

    result = pid.update(10.0, 0.10)

    assert pid.config().integral_clamp == 0.15
    assert result.integral_state == 0.15


def test_pid_derivative_term_is_zero_on_first_valid_sample():
    cfg = PidConfig(kp=0.0, ki=0.0, kd=1.0)
    pid = Pid(cfg)

    result = pid.update(1.0, 0.10)

    assert result.valid is True
    assert result.derivative_state == 0.0
    assert result.d_term == 0.0
    assert result.output == 0.0


def test_pid_derivative_term_uses_error_delta():
    cfg = PidConfig(kp=0.0, ki=0.0, kd=1.0)
    pid = Pid(cfg)

    pid.update(1.0, 0.10)
    result = pid.update(0.5, 0.10)

    assert isclose(result.derivative_state, -5.0, abs_tol=1e-9)
    assert isclose(result.d_term, -5.0, abs_tol=1e-9)
    assert isclose(result.output, -5.0, abs_tol=1e-9)


def test_pid_derivative_filter_alpha():
    cfg = PidConfig(kp=0.0, ki=0.0, kd=1.0, d_filter_alpha=0.5)
    pid = Pid(cfg)

    pid.update(1.0, 0.10)
    first = pid.update(0.5, 0.10)
    second = pid.update(0.0, 0.10)

    assert isclose(first.derivative_state, -2.5, abs_tol=1e-9)
    assert isclose(second.derivative_state, -3.75, abs_tol=1e-9)


def test_pid_invalid_error_returns_invalid_zero_output():
    cfg = PidConfig(kp=1.0, ki=1.0, kd=1.0)
    pid = Pid(cfg)

    result = pid.update(float("nan"), 0.10)

    assert result.valid is False
    assert result.output == 0.0
    assert result.output_raw == 0.0
    assert result.dt_valid is True
    assert pid.has_last_output() is True
    assert pid.last_output() == 0.0


def test_pid_invalid_dt_keeps_valid_result_but_dt_invalid():
    cfg = PidConfig(kp=1.0, ki=1.0, kd=1.0)
    pid = Pid(cfg)

    first = pid.update(1.0, 0.10)
    second = pid.update(0.5, -0.10)

    assert first.valid is True
    assert first.dt_valid is True

    assert second.valid is True
    assert second.dt_valid is False
    assert second.p_term == 0.5
    assert second.integral_state == first.integral_state
    assert second.derivative_state == 0.0


def test_pid_dt_too_large_is_invalid_dt():
    cfg = PidConfig(kp=1.0, max_dt_sec=0.5)
    pid = Pid(cfg)

    result = pid.update(1.0, 1.0)

    assert result.valid is True
    assert result.dt_valid is False
    assert result.output == 1.0


def test_pid_reset_clears_state():
    cfg = PidConfig(kp=1.0, ki=1.0, kd=1.0)
    pid = Pid(cfg)

    pid.update(1.0, 0.10)
    pid.update(0.5, 0.10)

    assert pid.has_previous_error() is True
    assert pid.has_last_output() is True
    assert pid.integral_state() != 0.0

    pid.reset()

    assert pid.has_previous_error() is False
    assert pid.previous_error() == 0.0
    assert pid.integral_state() == 0.0
    assert pid.derivative_state() == 0.0
    assert pid.has_last_output() is False
    assert pid.last_output() == 0.0


def test_pid_reset_to_sets_known_state():
    pid = Pid()

    pid.reset_to(error=0.25, output=0.10)

    assert pid.has_previous_error() is True
    assert pid.previous_error() == 0.25
    assert pid.integral_state() == 0.0
    assert pid.derivative_state() == 0.0
    assert pid.has_last_output() is True
    assert pid.last_output() == 0.10


def test_pid_reset_to_ignores_invalid_values_safely():
    pid = Pid()

    pid.reset_to(error=float("nan"), output=float("nan"))

    assert pid.has_previous_error() is False
    assert pid.previous_error() == 0.0
    assert pid.has_last_output() is False
    assert pid.last_output() == 0.0


def test_pid_update_from_setpoint():
    cfg = PidConfig(kp=2.0)
    pid = Pid(cfg)

    result = pid.update_from_setpoint(1.0, 0.75, 0.05)

    assert result.valid is True
    assert result.error == 0.25
    assert result.output == 0.5


def test_pid_update_from_setpoint_invalid_values():
    cfg = PidConfig(kp=2.0)
    pid = Pid(cfg)

    result = pid.update_from_setpoint("bad", 0.75, 0.05)

    assert result.valid is False
    assert result.output == 0.0


def test_pid_set_config_replaces_config_and_normalizes():
    pid = Pid()

    cfg = PidConfig(kp=1.0, output_min=1.0, output_max=-1.0, d_filter_alpha=2.0)
    pid.set_config(cfg)

    assert pid.config().kp == 1.0
    assert pid.config().output_min == -1.0
    assert pid.config().output_max == 1.0
    assert pid.config().d_filter_alpha == 1.0


def test_pid_set_gains():
    pid = Pid()

    pid.set_gains(1.0, 2.0, 3.0)

    assert pid.config().kp == 1.0
    assert pid.config().ki == 2.0
    assert pid.config().kd == 3.0


def test_pid_set_gains_sanitizes_invalid_values():
    pid = Pid()

    pid.set_gains(float("nan"), float("inf"), -float("inf"))

    assert pid.config().kp == 0.0
    assert pid.config().ki == 0.0
    assert pid.config().kd == 0.0


def test_pid_normalizes_invalid_config_values():
    cfg = PidConfig(
        kp=float("nan"),
        ki=float("inf"),
        kd=-float("inf"),
        integral_clamp=-1.0,
        d_filter_alpha=-1.0,
        min_dt_sec=-1.0,
        max_dt_sec=-2.0,
    )

    pid = Pid(cfg)

    assert pid.config().kp == 0.0
    assert pid.config().ki == 0.0
    assert pid.config().kd == 0.0
    assert pid.config().integral_clamp == 1.0
    assert pid.config().d_filter_alpha == 0.0
    assert pid.config().min_dt_sec == 1e-6
    assert pid.config().max_dt_sec == 1.0
