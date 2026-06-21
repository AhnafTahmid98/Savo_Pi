#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for stuck-detection state models."""

from __future__ import annotations

import pytest

from savo_control.models import (
    StuckDetectorState,
    StuckState,
    TwistCommand,
    should_detect_stuck,
)


def test_stuck_state_values_are_canonical():
    assert StuckState.values() == (
        "CLEAR",
        "WATCHING",
        "STUCK",
        "RECOVERY_REQUESTED",
        "SUPPRESSED",
        "STALE",
    )


def test_stuck_state_from_text_accepts_valid_inputs():
    assert StuckState.from_text("CLEAR") == StuckState.CLEAR
    assert StuckState.from_text("watching") == StuckState.WATCHING
    assert StuckState.from_text(" stuck ") == StuckState.STUCK
    assert StuckState.from_text("recovery_requested") == StuckState.RECOVERY_REQUESTED
    assert StuckState.from_text("suppressed") == StuckState.SUPPRESSED
    assert StuckState.from_text("stale") == StuckState.STALE


def test_stuck_state_from_text_rejects_invalid_input():
    with pytest.raises(ValueError):
        StuckState.from_text("bad")


def test_stuck_state_from_text_uses_default():
    assert StuckState.from_text("bad", default=StuckState.CLEAR) == StuckState.CLEAR
    assert StuckState.from_text(None, default=StuckState.STALE) == StuckState.STALE


def test_clear_state():
    state = StuckDetectorState.clear(reason="startup")

    assert state.state == StuckState.CLEAR
    assert state.reason == "startup"
    assert state.active() is False
    assert state.stuck_detected() is False
    assert state.can_request_recovery() is False


def test_stuck_state_factory():
    state = StuckDetectorState.stuck(
        cmd_vel_safe=TwistCommand(vx=0.10, source="safe"),
        stuck_duration_s=2.5,
        reason="no_motion",
    )

    assert state.state == StuckState.STUCK
    assert state.cmd_vel_safe.vx == 0.10
    assert state.cmd_vel_safe.source == "safe"
    assert state.stuck_duration_s == 2.5
    assert state.reason == "no_motion"
    assert state.active() is True
    assert state.stuck_detected() is True
    assert state.can_request_recovery() is True


def test_stuck_state_factory_clamps_negative_duration():
    state = StuckDetectorState.stuck(
        cmd_vel_safe=TwistCommand(vx=0.10),
        stuck_duration_s=-1.0,
    )

    assert state.stuck_duration_s == 0.0


def test_stuck_state_factory_sanitizes_command():
    state = StuckDetectorState.stuck(
        cmd_vel_safe=TwistCommand(vx=float("nan"), vy=0.1, wz=float("inf")),
        stuck_duration_s=1.0,
    )

    assert state.cmd_vel_safe.vx == 0.0
    assert state.cmd_vel_safe.vy == 0.1
    assert state.cmd_vel_safe.wz == 0.0


def test_suppressed_state():
    state = StuckDetectorState.suppressed(reason="safety_stop", safety_stop=True)

    assert state.state == StuckState.SUPPRESSED
    assert state.safety_stop is True
    assert state.reason == "safety_stop"
    assert state.active() is False
    assert state.stuck_detected() is False
    assert state.can_request_recovery() is False


def test_stale_state():
    state = StuckDetectorState.stale(
        cmd_stale=True,
        odom_stale=True,
        reason="timeout",
    )

    assert state.state == StuckState.STALE
    assert state.cmd_stale is True
    assert state.odom_stale is True
    assert state.reason == "timeout"
    assert state.active() is False
    assert state.stuck_detected() is False
    assert state.can_request_recovery() is False


def test_active_states():
    assert StuckDetectorState(state=StuckState.CLEAR).active() is False
    assert StuckDetectorState(state=StuckState.WATCHING).active() is True
    assert StuckDetectorState(state=StuckState.STUCK).active() is True
    assert StuckDetectorState(state=StuckState.RECOVERY_REQUESTED).active() is True
    assert StuckDetectorState(state=StuckState.SUPPRESSED).active() is False
    assert StuckDetectorState(state=StuckState.STALE).active() is False


def test_stuck_detected_states():
    assert StuckDetectorState(state=StuckState.STUCK).stuck_detected() is True
    assert (
        StuckDetectorState(state=StuckState.RECOVERY_REQUESTED).stuck_detected()
        is True
    )
    assert StuckDetectorState(state=StuckState.WATCHING).stuck_detected() is False
    assert StuckDetectorState(state=StuckState.CLEAR).stuck_detected() is False


def test_can_request_recovery_requires_clean_stuck_state():
    assert StuckDetectorState(state=StuckState.STUCK).can_request_recovery() is True

    assert (
        StuckDetectorState(state=StuckState.STUCK, safety_stop=True)
        .can_request_recovery()
        is False
    )
    assert (
        StuckDetectorState(state=StuckState.STUCK, cmd_stale=True)
        .can_request_recovery()
        is False
    )
    assert (
        StuckDetectorState(state=StuckState.STUCK, odom_stale=True)
        .can_request_recovery()
        is False
    )
    assert StuckDetectorState(state=StuckState.CLEAR).can_request_recovery() is False


def test_observed_linear_speed():
    state = StuckDetectorState(
        observed_vx=3.0,
        observed_vy=4.0,
    )

    assert state.observed_linear_speed() == 5.0


def test_to_dict():
    state = StuckDetectorState(
        state=StuckState.STUCK,
        cmd_vel_safe=TwistCommand(vx=0.10, source="safe", stamp_sec=1.5),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.0,
        safety_stop=False,
        cmd_stale=False,
        odom_stale=False,
        stuck_duration_s=2.5,
        clear_duration_s=0.0,
        recovery_request=True,
        reason="no_motion",
    )

    assert state.to_dict() == {
        "state": "STUCK",
        "cmd_vel_safe": {
            "vx": 0.10,
            "vy": 0.0,
            "wz": 0.0,
            "source": "safe",
            "stamp_sec": 1.5,
        },
        "observed_vx": 0.0,
        "observed_vy": 0.0,
        "observed_wz": 0.0,
        "safety_stop": False,
        "cmd_stale": False,
        "odom_stale": False,
        "stuck_duration_s": 2.5,
        "clear_duration_s": 0.0,
        "recovery_request": True,
        "reason": "no_motion",
    }


def test_status_text_without_reason():
    state = StuckDetectorState(
        state=StuckState.CLEAR,
        stuck_duration_s=0.0,
        recovery_request=False,
    )

    assert state.status_text() == (
        "state=CLEAR; safety_stop=false; cmd_stale=false; "
        "odom_stale=false; stuck_s=0.00; recovery_request=false"
    )


def test_status_text_with_reason():
    state = StuckDetectorState(
        state=StuckState.STUCK,
        safety_stop=False,
        cmd_stale=False,
        odom_stale=False,
        stuck_duration_s=2.5,
        recovery_request=True,
        reason="no_motion",
    )

    assert state.status_text() == (
        "state=STUCK; safety_stop=false; cmd_stale=false; "
        "odom_stale=false; stuck_s=2.50; recovery_request=true; "
        "reason=no_motion"
    )


def test_should_detect_stuck_false_when_no_command():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand.zero(),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is False


def test_should_detect_stuck_for_forward_command_without_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vx=0.10),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is True


def test_should_not_detect_stuck_for_forward_command_with_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vx=0.10),
        observed_vx=0.03,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is False


def test_should_detect_stuck_for_strafe_command_without_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vy=0.10),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is True


def test_should_not_detect_stuck_for_strafe_command_with_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vy=0.10),
        observed_vx=0.0,
        observed_vy=0.03,
        observed_wz=0.0,
    ) is False


def test_should_detect_stuck_for_turn_command_without_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(wz=0.20),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is True


def test_should_not_detect_stuck_for_turn_command_with_motion():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(wz=0.20),
        observed_vx=0.0,
        observed_vy=0.0,
        observed_wz=0.08,
    ) is False


def test_should_detect_stuck_with_mixed_command_if_requested_axis_does_not_move():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vx=0.10, wz=0.20),
        observed_vx=0.03,
        observed_vy=0.0,
        observed_wz=0.0,
    ) is True


def test_should_detect_stuck_uses_abs_thresholds():
    assert should_detect_stuck(
        cmd_vel_safe=TwistCommand(vx=-0.10),
        observed_vx=-0.01,
        observed_vy=0.0,
        observed_wz=0.0,
        min_cmd_vx=-0.05,
        min_observed_linear_m_s=-0.025,
    ) is True
