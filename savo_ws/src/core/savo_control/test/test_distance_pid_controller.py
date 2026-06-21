#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for the Python fallback distance PID controller."""

from __future__ import annotations

from math import isclose

from savo_control.controllers.distance_pid_py import (
    DistanceController,
    DistanceControllerConfig,
    DistanceControllerResult,
    DistancePid,
)
from savo_control.controllers.pid_py import PidConfig, PidResult
from savo_control.models import DistanceApproachConfig


def make_pid_config(kp: float = 0.5) -> PidConfig:
    cfg = PidConfig(kp=kp, ki=0.0, kd=0.0)
    cfg.set_output_symmetric(0.20)
    return cfg


def make_controller(
    *,
    direction_mode: str = "bidirectional",
    tolerance_m: float = 0.04,
    max_vx_m_s: float = 0.10,
    min_effective_vx_m_s: float = 0.0,
    output_deadband_m_s: float = 0.0,
) -> DistancePid:
    return DistancePid(
        DistanceControllerConfig(
            pid=make_pid_config(),
            distance_tolerance_m=tolerance_m,
            max_vx_m_s=max_vx_m_s,
            min_effective_vx_m_s=min_effective_vx_m_s,
            output_deadband_m_s=output_deadband_m_s,
            direction_mode=direction_mode,
        )
    )


def test_distance_controller_alias():
    assert DistanceController is DistancePid


def test_default_config_is_safe():
    cfg = DistanceControllerConfig()

    assert isinstance(cfg.pid, PidConfig)
    assert cfg.distance_tolerance_m == 0.03
    assert cfg.output_deadband_m_s == 0.0
    assert cfg.min_effective_vx_m_s == 0.0
    assert cfg.max_vx_m_s == 0.0
    assert cfg.zero_output_within_tolerance is True
    assert cfg.reset_pid_on_target_change is True
    assert cfg.direction_mode == "bidirectional"


def test_from_distance_approach_config_forward_only():
    model_cfg = DistanceApproachConfig(
        target_distance_m=0.60,
        tolerance_m=0.04,
        kp=0.45,
        ki=0.01,
        kd=0.02,
        max_forward_vx=0.10,
        allow_reverse=False,
        max_reverse_vx=0.05,
        min_vx_when_active=0.04,
    )

    cfg = DistanceControllerConfig.from_distance_approach_config(model_cfg)

    assert cfg.pid.kp == 0.45
    assert cfg.pid.ki == 0.01
    assert cfg.pid.kd == 0.02
    assert cfg.distance_tolerance_m == 0.04
    assert cfg.min_effective_vx_m_s == 0.04
    assert cfg.max_vx_m_s == 0.10
    assert cfg.zero_output_within_tolerance is True
    assert cfg.direction_mode == "approach_from_far_only"


def test_from_distance_approach_config_bidirectional_when_reverse_allowed():
    model_cfg = DistanceApproachConfig(
        max_forward_vx=0.10,
        allow_reverse=True,
        max_reverse_vx=0.06,
    )

    cfg = DistanceControllerConfig.from_distance_approach_config(model_cfg)

    assert cfg.direction_mode == "bidirectional"
    assert cfg.max_vx_m_s == 0.10


def test_target_lifecycle():
    ctrl = make_controller()

    assert ctrl.has_target() is False
    assert ctrl.target_distance_m() == 0.0

    ctrl.set_target_distance(0.60)
    assert ctrl.has_target() is True
    assert ctrl.target_distance_m() == 0.60

    ctrl.clear_target()
    assert ctrl.has_target() is False
    assert ctrl.target_distance_m() == 0.0


def test_invalid_target_is_ignored():
    ctrl = make_controller()
    ctrl.set_target_distance(0.60)
    ctrl.set_target_distance(float("nan"))

    assert ctrl.has_target() is True
    assert ctrl.target_distance_m() == 0.60


def test_update_without_target_is_invalid_and_zero():
    ctrl = make_controller()

    result = ctrl.update(0.80, 0.05)

    assert result.valid is False
    assert result.has_target is False
    assert result.vx_cmd_m_s == 0.0


def test_update_with_invalid_distance_is_invalid_and_zero():
    ctrl = make_controller()
    ctrl.set_target_distance(0.60)

    result = ctrl.update(float("nan"), 0.05)

    assert result.valid is False
    assert result.distance_valid is False
    assert result.vx_cmd_m_s == 0.0


def test_error_sign_matches_distance_approach_contract():
    ctrl = make_controller()
    ctrl.set_target_distance(0.60)

    far = ctrl.update(0.80, 0.05)
    close = ctrl.update(0.40, 0.05)

    assert isclose(far.error_distance_m, 0.20, abs_tol=1e-9)
    assert far.vx_cmd_m_s > 0.0

    assert isclose(close.error_distance_m, -0.20, abs_tol=1e-9)
    assert close.vx_cmd_m_s < 0.0


def test_far_distance_clamps_positive_forward_speed():
    ctrl = make_controller(max_vx_m_s=0.10)
    ctrl.set_target_distance(0.60)

    result = ctrl.update(1.20, 0.05)

    assert result.valid is True
    assert result.dt_valid is True
    assert result.distance_valid is True
    assert isclose(result.error_distance_m, 0.60, abs_tol=1e-9)
    assert result.vx_cmd_m_s == 0.10


def test_close_distance_clamps_negative_reverse_speed_bidirectional():
    ctrl = make_controller(max_vx_m_s=0.10)
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.10, 0.05)

    assert result.valid is True
    assert result.vx_cmd_m_s == -0.10


def test_within_tolerance_outputs_zero():
    ctrl = make_controller(tolerance_m=0.04)
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.62, 0.05)

    assert result.valid is True
    assert result.within_tolerance is True
    assert result.vx_cmd_m_s == 0.0


def test_is_within_tolerance():
    ctrl = make_controller(tolerance_m=0.04)
    ctrl.set_target_distance(0.60)

    assert ctrl.is_within_tolerance(0.60) is True
    assert ctrl.is_within_tolerance(0.64) is True
    assert ctrl.is_within_tolerance(0.65) is False


def test_approach_from_far_only_allows_forward_when_far():
    ctrl = make_controller(direction_mode="approach_from_far_only")
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.80, 0.05)

    assert result.vx_cmd_m_s > 0.0
    assert result.direction_limited is False


def test_approach_from_far_only_blocks_reverse_when_too_close():
    ctrl = make_controller(direction_mode="approach_from_far_only")
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.40, 0.05)

    assert result.vx_cmd_m_s == 0.0
    assert result.direction_limited is True


def test_back_away_only_blocks_forward_when_far():
    ctrl = make_controller(direction_mode="back_away_only")
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.80, 0.05)

    assert result.vx_cmd_m_s == 0.0
    assert result.direction_limited is True


def test_back_away_only_allows_reverse_when_too_close():
    ctrl = make_controller(direction_mode="back_away_only")
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.40, 0.05)

    assert result.vx_cmd_m_s < 0.0
    assert result.direction_limited is False


def test_invalid_direction_mode_falls_back_to_bidirectional():
    cfg = DistanceControllerConfig(
        pid=make_pid_config(),
        direction_mode="bad",
    )

    ctrl = DistancePid(cfg)

    assert ctrl.config.direction_mode == "bidirectional"


def test_min_effective_vx_is_applied_outside_tolerance():
    ctrl = make_controller(
        tolerance_m=0.01,
        min_effective_vx_m_s=0.04,
        max_vx_m_s=0.10,
    )
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.66, 0.05)

    assert result.within_tolerance is False
    assert result.vx_cmd_m_s == 0.04


def test_output_deadband_zeroes_small_command():
    pid_cfg = PidConfig(kp=0.1, ki=0.0, kd=0.0)
    pid_cfg.set_output_symmetric(0.20)

    ctrl = DistancePid(
        DistanceControllerConfig(
            pid=pid_cfg,
            distance_tolerance_m=0.001,
            output_deadband_m_s=0.01,
            max_vx_m_s=0.20,
            direction_mode="bidirectional",
        )
    )
    ctrl.set_target_distance(0.60)

    result = ctrl.update(0.65, 0.05)

    assert result.within_tolerance is False
    assert result.vx_cmd_m_s == 0.0


def test_update_to_target_does_not_store_target():
    ctrl = make_controller()

    result = ctrl.update_to_target(
        current_distance_m=0.80,
        target_distance_m=0.60,
        dt_sec=0.05,
    )

    assert result.has_target is True
    assert result.target_distance_m == 0.60
    assert result.vx_cmd_m_s > 0.0

    assert ctrl.has_target() is False
    assert ctrl.target_distance_m() == 0.0


def test_update_to_target_rejects_invalid_target():
    ctrl = make_controller()

    result = ctrl.update_to_target(
        current_distance_m=0.80,
        target_distance_m=float("nan"),
        dt_sec=0.05,
    )

    assert result.valid is False
    assert result.has_target is False
    assert result.vx_cmd_m_s == 0.0


def test_reset_pid_on_target_change():
    ctrl = make_controller()
    ctrl.set_target_distance(0.60)
    ctrl.update(0.80, 0.05)

    assert ctrl.pid().has_previous_error() is True

    ctrl.set_target_distance(0.80)

    assert ctrl.pid().has_previous_error() is False


def test_small_target_change_does_not_reset_pid():
    ctrl = make_controller()
    ctrl.set_target_distance(0.60)
    ctrl.update(0.80, 0.05)

    assert ctrl.pid().has_previous_error() is True

    ctrl.set_target_distance(0.605)

    assert ctrl.pid().has_previous_error() is True


def test_result_defaults_are_safe():
    result = DistanceControllerResult()

    assert result.vx_cmd_m_s == 0.0
    assert result.target_distance_m == 0.0
    assert result.current_distance_m == 0.0
    assert result.error_distance_m == 0.0
    assert result.has_target is False
    assert result.within_tolerance is False
    assert result.valid is False
    assert result.dt_valid is False
    assert result.distance_valid is False
    assert isinstance(result.pid, PidResult)
    assert result.direction_limited is False
