#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for the Python fallback heading PID controller."""

from __future__ import annotations

from math import isclose, pi

from savo_control.controllers.heading_pid_py import (
    HeadingController,
    HeadingControllerConfig,
    HeadingControllerResult,
    HeadingPid,
    copy_sign,
    saturate_abs,
    shortest_angular_distance_rad,
    wrap_angle_rad,
)
from savo_control.controllers.pid_py import PidConfig, PidResult


def make_pid_config(kp: float = 1.0) -> PidConfig:
    cfg = PidConfig(kp=kp, ki=0.0, kd=0.0)
    cfg.set_output_symmetric(1.0)
    return cfg


def make_controller(
    *,
    tolerance_rad: float = 0.03,
    max_wz_rad_s: float = 0.50,
    min_effective_wz_rad_s: float = 0.0,
    output_deadband_rad_s: float = 0.0,
) -> HeadingController:
    return HeadingController(
        HeadingControllerConfig(
            pid=make_pid_config(),
            heading_tolerance_rad=tolerance_rad,
            max_wz_rad_s=max_wz_rad_s,
            min_effective_wz_rad_s=min_effective_wz_rad_s,
            output_deadband_rad_s=output_deadband_rad_s,
        )
    )


def test_heading_pid_alias():
    assert HeadingPid is HeadingController


def test_default_config_is_safe():
    cfg = HeadingControllerConfig()

    assert isinstance(cfg.pid, PidConfig)
    assert cfg.heading_tolerance_rad == 0.03
    assert cfg.output_deadband_rad_s == 0.0
    assert cfg.min_effective_wz_rad_s == 0.0
    assert cfg.max_wz_rad_s == 0.0
    assert cfg.reset_pid_on_hold_capture is True
    assert cfg.reset_pid_on_target_change is True
    assert cfg.target_change_reset_threshold_rad == 0.02
    assert cfg.min_dt_sec == 1e-6
    assert cfg.max_dt_sec == 1.0


def test_wrap_angle_rad():
    assert isclose(wrap_angle_rad(0.0), 0.0, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(pi), pi, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(-pi), pi, abs_tol=1e-9)
    assert isclose(wrap_angle_rad(3.0 * pi), pi, abs_tol=1e-9)
    assert wrap_angle_rad(float("nan")) == 0.0


def test_shortest_angular_distance_rad():
    assert isclose(
        shortest_angular_distance_rad(0.0, pi / 2.0),
        pi / 2.0,
        abs_tol=1e-9,
    )
    assert isclose(
        shortest_angular_distance_rad(pi / 2.0, 0.0),
        -pi / 2.0,
        abs_tol=1e-9,
    )
    assert isclose(
        shortest_angular_distance_rad(pi, -pi),
        0.0,
        abs_tol=1e-9,
    )
    assert shortest_angular_distance_rad(float("nan"), 0.0) == 0.0


def test_saturate_abs():
    assert saturate_abs(0.25, 0.50) == 0.25
    assert saturate_abs(2.0, 0.50) == 0.50
    assert saturate_abs(-2.0, 0.50) == -0.50
    assert saturate_abs(2.0, 0.0) == 2.0
    assert saturate_abs(float("nan"), 0.5) == 0.0


def test_copy_sign():
    assert copy_sign(0.2, 1.0) == 0.2
    assert copy_sign(0.2, -1.0) == -0.2
    assert copy_sign(-0.2, 1.0) == 0.2
    assert copy_sign(float("nan"), 1.0) == 0.0


def test_target_lifecycle():
    ctrl = make_controller()

    assert ctrl.has_target() is False
    assert ctrl.target_yaw_rad() == 0.0

    ctrl.set_target_yaw(pi / 2.0)
    assert ctrl.has_target() is True
    assert isclose(ctrl.target_yaw_rad(), pi / 2.0, abs_tol=1e-9)

    ctrl.clear_target()
    assert ctrl.has_target() is False
    assert ctrl.target_yaw_rad() == 0.0


def test_invalid_target_is_ignored():
    ctrl = make_controller()
    ctrl.set_target_yaw(pi / 2.0)
    ctrl.set_target_yaw(float("nan"))

    assert ctrl.has_target() is True
    assert isclose(ctrl.target_yaw_rad(), pi / 2.0, abs_tol=1e-9)


def test_capture_hold_target():
    ctrl = make_controller()

    ctrl.capture_hold_target(3.0 * pi)

    assert ctrl.has_target() is True
    assert isclose(ctrl.target_yaw_rad(), pi, abs_tol=1e-9)


def test_update_without_target_is_invalid_and_zero():
    ctrl = make_controller()

    result = ctrl.update(0.0, 0.05)

    assert result.valid is False
    assert result.has_target is False
    assert result.wz_cmd_rad_s == 0.0


def test_update_with_invalid_yaw_is_invalid_and_zero():
    ctrl = make_controller()
    ctrl.set_target_yaw(pi / 2.0)

    result = ctrl.update(float("nan"), 0.05)

    assert result.valid is False
    assert result.yaw_valid is False
    assert result.wz_cmd_rad_s == 0.0


def test_positive_heading_error_outputs_positive_wz():
    ctrl = make_controller(max_wz_rad_s=0.50)
    ctrl.set_target_yaw(pi / 2.0)

    result = ctrl.update(0.0, 0.05)

    assert result.valid is True
    assert result.dt_valid is True
    assert result.yaw_valid is True
    assert isclose(result.error_yaw_rad, pi / 2.0, abs_tol=1e-9)
    assert result.wz_cmd_rad_s > 0.0
    assert result.wz_cmd_rad_s == 0.50


def test_negative_heading_error_outputs_negative_wz():
    ctrl = make_controller(max_wz_rad_s=0.50)
    ctrl.set_target_yaw(0.0)

    result = ctrl.update(pi / 2.0, 0.05)

    assert result.valid is True
    assert isclose(result.error_yaw_rad, -pi / 2.0, abs_tol=1e-9)
    assert result.wz_cmd_rad_s < 0.0
    assert result.wz_cmd_rad_s == -0.50


def test_within_tolerance_outputs_zero():
    ctrl = make_controller(tolerance_rad=0.05)
    ctrl.set_target_yaw(0.0)

    result = ctrl.update(0.03, 0.05)

    assert result.valid is True
    assert result.within_tolerance is True
    assert result.wz_cmd_rad_s == 0.0


def test_is_within_tolerance():
    ctrl = make_controller(tolerance_rad=0.05)
    ctrl.set_target_yaw(0.0)

    assert ctrl.is_within_tolerance(0.00) is True
    assert ctrl.is_within_tolerance(0.04) is True
    assert ctrl.is_within_tolerance(0.06) is False
    assert ctrl.is_within_tolerance(float("nan")) is False


def test_min_effective_wz_is_applied_outside_tolerance():
    pid_cfg = PidConfig(kp=0.10, ki=0.0, kd=0.0)
    pid_cfg.set_output_symmetric(1.0)

    ctrl = HeadingController(
        HeadingControllerConfig(
            pid=pid_cfg,
            heading_tolerance_rad=0.001,
            min_effective_wz_rad_s=0.05,
            max_wz_rad_s=0.50,
        )
    )
    ctrl.set_target_yaw(0.10)

    result = ctrl.update(0.0, 0.05)

    assert result.within_tolerance is False
    assert result.wz_cmd_rad_s == 0.05


def test_output_deadband_zeroes_small_command():
    pid_cfg = PidConfig(kp=0.10, ki=0.0, kd=0.0)
    pid_cfg.set_output_symmetric(1.0)

    ctrl = HeadingController(
        HeadingControllerConfig(
            pid=pid_cfg,
            heading_tolerance_rad=0.001,
            output_deadband_rad_s=0.02,
            max_wz_rad_s=0.50,
        )
    )
    ctrl.set_target_yaw(0.10)

    result = ctrl.update(0.0, 0.05)

    assert result.within_tolerance is False
    assert result.wz_cmd_rad_s == 0.0


def test_update_to_target_does_not_store_target():
    ctrl = make_controller()

    result = ctrl.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=pi / 2.0,
        dt_sec=0.05,
    )

    assert result.has_target is True
    assert isclose(result.target_yaw_rad, pi / 2.0, abs_tol=1e-9)
    assert result.wz_cmd_rad_s > 0.0

    assert ctrl.has_target() is False
    assert ctrl.target_yaw_rad() == 0.0


def test_update_to_target_rejects_invalid_target():
    ctrl = make_controller()

    result = ctrl.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=float("nan"),
        dt_sec=0.05,
    )

    assert result.valid is False
    assert result.has_target is False
    assert result.wz_cmd_rad_s == 0.0


def test_reset_pid_on_target_change():
    ctrl = make_controller()
    ctrl.set_target_yaw(0.0)
    ctrl.update(0.50, 0.05)

    assert ctrl.pid().has_previous_error() is True

    ctrl.set_target_yaw(0.50)

    assert ctrl.pid().has_previous_error() is False


def test_small_target_change_does_not_reset_pid():
    ctrl = make_controller()
    ctrl.set_target_yaw(0.0)
    ctrl.update(0.50, 0.05)

    assert ctrl.pid().has_previous_error() is True

    ctrl.set_target_yaw(0.005)

    assert ctrl.pid().has_previous_error() is True


def test_capture_hold_target_resets_pid_by_default():
    ctrl = make_controller()
    ctrl.set_target_yaw(0.0)
    ctrl.update(0.50, 0.05)

    assert ctrl.pid().has_previous_error() is True

    ctrl.capture_hold_target(0.50)

    assert ctrl.pid().has_previous_error() is False


def test_invalid_config_values_are_normalized():
    cfg = HeadingControllerConfig(
        heading_tolerance_rad=-1.0,
        output_deadband_rad_s=-1.0,
        min_effective_wz_rad_s=-1.0,
        max_wz_rad_s=-0.50,
        target_change_reset_threshold_rad=-1.0,
        min_dt_sec=-1.0,
        max_dt_sec=-2.0,
    )

    ctrl = HeadingController(cfg)

    assert ctrl.config.heading_tolerance_rad == 0.03
    assert ctrl.config.output_deadband_rad_s == 0.0
    assert ctrl.config.min_effective_wz_rad_s == 0.0
    assert ctrl.config.max_wz_rad_s == 0.50
    assert ctrl.config.target_change_reset_threshold_rad == 0.02
    assert ctrl.config.min_dt_sec == 1e-6
    assert ctrl.config.max_dt_sec == 1.0


def test_invalid_dt_keeps_result_invalid_but_pid_result_present():
    ctrl = make_controller()
    ctrl.set_target_yaw(pi / 2.0)

    result = ctrl.update(0.0, -0.1)

    assert result.dt_valid is False
    assert result.valid is False
    assert isinstance(result.pid, PidResult)


def test_result_defaults_are_safe():
    result = HeadingControllerResult()

    assert result.wz_cmd_rad_s == 0.0
    assert result.target_yaw_rad == 0.0
    assert result.current_yaw_rad == 0.0
    assert result.error_yaw_rad == 0.0
    assert result.has_target is False
    assert result.within_tolerance is False
    assert result.valid is False
    assert result.dt_valid is False
    assert result.yaw_valid is False
    assert isinstance(result.pid, PidResult)
