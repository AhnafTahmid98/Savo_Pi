#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for distance approach models."""

from __future__ import annotations

from math import isclose

from savo_control.models import (
    DistanceApproachConfig,
    DistanceApproachState,
    TwistCommand,
    command_from_error,
)


def test_default_config_is_safe_and_valid():
    cfg = DistanceApproachConfig()

    assert cfg.target_distance_m == 0.60
    assert cfg.tolerance_m == 0.04
    assert cfg.hard_min_distance_m == 0.35
    assert cfg.allow_reverse is False
    assert cfg.max_forward_vx == 0.10


def test_config_sanitized_clamps_invalid_ranges():
    cfg = DistanceApproachConfig(
        target_distance_m=-0.60,
        tolerance_m=-0.04,
        hard_min_distance_m=-0.35,
        min_valid_distance_m=-0.05,
        max_valid_distance_m=0.01,
        distance_timeout_s=-0.40,
        max_forward_vx=-0.10,
        max_reverse_vx=-0.05,
        min_vx_when_active=-0.04,
        disable_min_vx_below_error_m=-0.08,
    ).sanitized()

    assert cfg.target_distance_m == 0.60
    assert cfg.tolerance_m == 0.04
    assert cfg.hard_min_distance_m == 0.35
    assert cfg.min_valid_distance_m == 0.05
    assert cfg.max_valid_distance_m > cfg.min_valid_distance_m
    assert cfg.distance_timeout_s == 0.40
    assert cfg.max_forward_vx == 0.10
    assert cfg.max_reverse_vx == 0.05
    assert cfg.min_vx_when_active == 0.04
    assert cfg.disable_min_vx_below_error_m == 0.08


def test_valid_distance():
    cfg = DistanceApproachConfig()

    assert cfg.valid_distance(0.05) is True
    assert cfg.valid_distance(0.60) is True
    assert cfg.valid_distance(3.00) is True

    assert cfg.valid_distance(0.01) is False
    assert cfg.valid_distance(3.50) is False
    assert cfg.valid_distance(float("nan")) is False
    assert cfg.valid_distance(float("inf")) is False


def test_too_close():
    cfg = DistanceApproachConfig(hard_min_distance_m=0.35)

    assert cfg.too_close(0.20) is True
    assert cfg.too_close(0.34) is True
    assert cfg.too_close(0.35) is False
    assert cfg.too_close(0.50) is False


def test_goal_reached():
    cfg = DistanceApproachConfig(target_distance_m=0.60, tolerance_m=0.04)

    assert cfg.goal_reached(0.56) is True
    assert cfg.goal_reached(0.60) is True
    assert cfg.goal_reached(0.64) is True
    assert cfg.goal_reached(0.55) is False
    assert cfg.goal_reached(0.65) is False


def test_error_m():
    cfg = DistanceApproachConfig(target_distance_m=0.60)

    assert isclose(cfg.error_m(0.75), 0.15, abs_tol=1e-9)
    assert isclose(cfg.error_m(0.60), 0.00, abs_tol=1e-9)
    assert isclose(cfg.error_m(0.45), -0.15, abs_tol=1e-9)


def test_config_to_dict_uses_sanitized_values():
    data = DistanceApproachConfig(
        target_distance_m=-0.60,
        tolerance_m=-0.04,
        max_forward_vx=-0.10,
    ).to_dict()

    assert data["target_distance_m"] == 0.60
    assert data["tolerance_m"] == 0.04
    assert data["max_forward_vx"] == 0.10
    assert data["allow_reverse"] is False


def test_command_from_positive_error_clamps_forward_speed():
    cfg = DistanceApproachConfig(kp=0.45, max_forward_vx=0.10)

    cmd = command_from_error(0.30, cfg)

    assert isinstance(cmd, TwistCommand)
    assert cmd.vx == 0.10
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == "distance_approach"


def test_command_from_small_positive_error_applies_min_vx():
    cfg = DistanceApproachConfig(
        kp=0.10,
        max_forward_vx=0.10,
        min_vx_when_active=0.04,
        disable_min_vx_below_error_m=0.08,
    )

    cmd = command_from_error(0.20, cfg)

    assert cmd.vx == 0.04


def test_command_from_small_goal_error_does_not_apply_min_vx():
    cfg = DistanceApproachConfig(
        kp=0.10,
        min_vx_when_active=0.04,
        disable_min_vx_below_error_m=0.08,
    )

    cmd = command_from_error(0.04, cfg)

    assert isclose(cmd.vx, 0.004, abs_tol=1e-9)


def test_command_from_negative_error_stops_when_reverse_disabled():
    cfg = DistanceApproachConfig(allow_reverse=False)

    cmd = command_from_error(-0.20, cfg)

    assert cmd.vx == 0.0
    assert cmd.source == "distance_approach"


def test_command_from_negative_error_reverses_when_enabled():
    cfg = DistanceApproachConfig(
        allow_reverse=True,
        kp=0.45,
        max_reverse_vx=0.04,
    )

    cmd = command_from_error(-0.20, cfg)

    assert cmd.vx == -0.04


def test_command_from_invalid_error_returns_zero():
    cfg = DistanceApproachConfig()

    cmd = command_from_error(float("nan"), cfg)

    assert cmd.vx == 0.0
    assert cmd.source == "invalid_error"


def test_idle_state():
    state = DistanceApproachState.idle(target_distance_m=0.70)

    assert state.state == "IDLE"
    assert state.target_distance_m == 0.70
    assert state.running() is False
    assert state.goal_reached() is False


def test_stopped_state():
    state = DistanceApproachState.stopped("SAFETY_STOP", target_distance_m=0.60)

    assert state.state == "SAFETY_STOP"
    assert state.command.vx == 0.0
    assert state.command.source == "safety_stop"
    assert state.running() is False


def test_running_state_to_dict():
    cmd = TwistCommand(vx=0.10, source="distance_approach")
    state = DistanceApproachState(
        state="RUNNING",
        distance_m=0.80,
        target_distance_m=0.60,
        error_m=0.20,
        command=cmd,
        safety_stop=False,
        stale=False,
        valid=True,
    )

    assert state.running() is True
    assert state.goal_reached() is False

    data = state.to_dict()
    assert data["state"] == "RUNNING"
    assert data["distance_m"] == 0.80
    assert data["target_distance_m"] == 0.60
    assert data["error_m"] == 0.20
    assert data["command"]["vx"] == 0.10
    assert data["valid"] is True


def test_goal_reached_state():
    state = DistanceApproachState(state="GOAL_REACHED")

    assert state.running() is False
    assert state.goal_reached() is True


def test_status_text_with_values():
    state = DistanceApproachState(
        state="RUNNING",
        distance_m=0.80,
        target_distance_m=0.60,
        error_m=0.20,
        command=TwistCommand(vx=0.10),
    )

    assert state.status_text() == (
        "state=RUNNING; distance_m=0.800; "
        "target_m=0.600; error_m=0.200; vx=0.100"
    )


def test_status_text_without_values():
    state = DistanceApproachState.idle()

    assert state.status_text() == (
        "state=IDLE; distance_m=nan; "
        "target_m=0.600; error_m=nan; vx=0.000"
    )
