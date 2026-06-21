#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for Python control-mode models."""

from __future__ import annotations

import pytest

from savo_control.models import (
    ControlMode,
    ControlModeState,
    MODE_PRIORITY,
    VALID_CONTROL_MODES,
    highest_priority_mode,
    is_valid_mode,
    normalize_mode,
)


def test_control_mode_values_are_canonical():
    assert VALID_CONTROL_MODES == (
        "STOP",
        "MANUAL",
        "AUTO",
        "NAV",
        "RECOVERY",
    )


def test_control_mode_from_text_accepts_common_inputs():
    assert ControlMode.from_text("STOP") == ControlMode.STOP
    assert ControlMode.from_text("manual") == ControlMode.MANUAL
    assert ControlMode.from_text(" Auto ") == ControlMode.AUTO
    assert ControlMode.from_text("nav") == ControlMode.NAV
    assert ControlMode.from_text("recovery") == ControlMode.RECOVERY


def test_control_mode_from_text_maps_safe_idle_states_to_stop():
    assert ControlMode.from_text("IDLE") == ControlMode.STOP
    assert ControlMode.from_text("disabled") == ControlMode.STOP


def test_control_mode_from_text_rejects_invalid_mode():
    with pytest.raises(ValueError):
        ControlMode.from_text("DRIVE_FAST")


def test_control_mode_from_text_uses_default_when_requested():
    assert ControlMode.from_text("bad", default=ControlMode.STOP) == ControlMode.STOP
    assert ControlMode.from_text(None, default=ControlMode.MANUAL) == ControlMode.MANUAL


def test_is_valid_mode():
    assert is_valid_mode("STOP") is True
    assert is_valid_mode("manual") is True
    assert is_valid_mode("AUTO") is True
    assert is_valid_mode("NAV") is True
    assert is_valid_mode("RECOVERY") is True
    assert is_valid_mode("bad") is False


def test_normalize_mode_defaults_to_stop():
    assert normalize_mode("manual") == "MANUAL"
    assert normalize_mode("bad") == "STOP"
    assert normalize_mode(None) == "STOP"


def test_mode_priority_keeps_stop_highest():
    assert MODE_PRIORITY[ControlMode.STOP] > MODE_PRIORITY[ControlMode.RECOVERY]
    assert MODE_PRIORITY[ControlMode.RECOVERY] > MODE_PRIORITY[ControlMode.MANUAL]
    assert MODE_PRIORITY[ControlMode.MANUAL] > MODE_PRIORITY[ControlMode.NAV]
    assert MODE_PRIORITY[ControlMode.NAV] > MODE_PRIORITY[ControlMode.AUTO]


def test_highest_priority_mode():
    assert highest_priority_mode([]) == ControlMode.STOP
    assert highest_priority_mode(["AUTO", "NAV"]) == ControlMode.NAV
    assert highest_priority_mode(["AUTO", "MANUAL", "NAV"]) == ControlMode.MANUAL
    assert highest_priority_mode(["AUTO", "RECOVERY", "MANUAL"]) == ControlMode.RECOVERY
    assert highest_priority_mode(["AUTO", "STOP", "MANUAL"]) == ControlMode.STOP


def test_control_mode_state_stopped_factory():
    state = ControlModeState.stopped()

    assert state.mode == ControlMode.STOP
    assert state.reason == "startup"
    assert state.source == ""
    assert state.stamp_sec == 0.0


def test_control_mode_state_to_dict():
    state = ControlModeState(
        mode=ControlMode.MANUAL,
        reason="operator",
        source="keyboard",
        stamp_sec=12.5,
    )

    assert state.to_dict() == {
        "mode": "MANUAL",
        "reason": "operator",
        "source": "keyboard",
        "stamp_sec": 12.5,
    }


def test_control_mode_state_status_text():
    assert ControlModeState(ControlMode.AUTO).status_text() == "mode=AUTO"

    state = ControlModeState(ControlMode.STOP, reason="safety_stop")
    assert state.status_text() == "mode=STOP; reason=safety_stop"
