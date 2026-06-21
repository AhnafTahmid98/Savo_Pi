#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for legacy control-mode interface compatibility helpers."""

from __future__ import annotations

import pytest

from savo_control.interfaces import (
    ControlMode,
    ControlModeReason,
    is_motion_mode,
    is_stop_like,
    mode_display_label,
    normalize_mode_text,
    parse_control_mode,
    parse_control_reason,
    plain_mux_mode_string,
    safe_mode_fallback,
)
from savo_control.models import ControlMode as ModelControlMode


def test_interface_control_mode_is_canonical_model_control_mode():
    assert ControlMode is ModelControlMode


def test_normalize_mode_text():
    assert normalize_mode_text(" manual ") == "MANUAL"
    assert normalize_mode_text("nav-2") == "NAV_2"
    assert normalize_mode_text("safety stop") == "SAFETY_STOP"
    assert normalize_mode_text(None) == ""


def test_parse_control_mode_accepts_canonical_modes():
    assert parse_control_mode("STOP") == ControlMode.STOP
    assert parse_control_mode("MANUAL") == ControlMode.MANUAL
    assert parse_control_mode("AUTO") == ControlMode.AUTO
    assert parse_control_mode("NAV") == ControlMode.NAV
    assert parse_control_mode("RECOVERY") == ControlMode.RECOVERY


def test_parse_control_mode_accepts_legacy_aliases():
    assert parse_control_mode("IDLE") == ControlMode.STOP
    assert parse_control_mode("DISABLED") == ControlMode.STOP

    assert parse_control_mode("TELEOP") == ControlMode.MANUAL
    assert parse_control_mode("MAN") == ControlMode.MANUAL

    assert parse_control_mode("AUTONOMOUS") == ControlMode.AUTO
    assert parse_control_mode("AUTON") == ControlMode.AUTO

    assert parse_control_mode("NAV2") == ControlMode.NAV


def test_parse_control_mode_accepts_existing_enum():
    assert parse_control_mode(ControlMode.MANUAL) == ControlMode.MANUAL


def test_parse_control_mode_uses_default_for_invalid_value():
    assert parse_control_mode("bad", default=ControlMode.STOP) == ControlMode.STOP
    assert parse_control_mode(None, default=ControlMode.AUTO) == ControlMode.AUTO


def test_parse_control_mode_raises_without_default_for_invalid_value():
    with pytest.raises(ValueError):
        parse_control_mode("bad")


def test_is_stop_like():
    assert is_stop_like("STOP") is True
    assert is_stop_like("IDLE") is True
    assert is_stop_like("DISABLED") is True

    assert is_stop_like("MANUAL") is False
    assert is_stop_like("AUTO") is False
    assert is_stop_like("NAV") is False
    assert is_stop_like("RECOVERY") is False


def test_is_motion_mode():
    assert is_motion_mode("MANUAL") is True
    assert is_motion_mode("TELEOP") is True
    assert is_motion_mode("AUTO") is True
    assert is_motion_mode("NAV") is True
    assert is_motion_mode("RECOVERY") is True

    assert is_motion_mode("STOP") is False
    assert is_motion_mode("IDLE") is False
    assert is_motion_mode("bad") is False


def test_safe_mode_fallback():
    assert safe_mode_fallback() == ControlMode.STOP


def test_plain_mux_mode_string_allows_only_mux_sources():
    assert plain_mux_mode_string("MANUAL") == "MANUAL"
    assert plain_mux_mode_string("TELEOP") == "MANUAL"

    assert plain_mux_mode_string("AUTO") == "AUTO"
    assert plain_mux_mode_string("AUTONOMOUS") == "AUTO"

    assert plain_mux_mode_string("NAV") == "NAV"
    assert plain_mux_mode_string("NAV2") == "NAV"

    assert plain_mux_mode_string("STOP") == "STOP"
    assert plain_mux_mode_string("RECOVERY") == "STOP"
    assert plain_mux_mode_string("bad") == "STOP"


def test_control_mode_reason_values_are_canonical():
    assert [reason.value for reason in ControlModeReason] == [
        "NONE",
        "STARTUP",
        "REQUESTED",
        "MANUAL_OVERRIDE",
        "RECOVERY_ACTIVE",
        "RECOVERY_LATCHED",
        "SAFETY_STOP_ACTIVE",
        "EXTERNAL_STOP",
        "INVALID_TIME",
        "TIMEOUT",
        "NO_SOURCE_ALLOWED",
        "UNKNOWN",
    ]


def test_parse_control_reason_accepts_valid_values():
    assert parse_control_reason("NONE") == ControlModeReason.NONE
    assert parse_control_reason("startup") == ControlModeReason.STARTUP
    assert parse_control_reason("requested") == ControlModeReason.REQUESTED
    assert parse_control_reason("manual_override") == ControlModeReason.MANUAL_OVERRIDE
    assert parse_control_reason("recovery_active") == ControlModeReason.RECOVERY_ACTIVE
    assert parse_control_reason("safety_stop_active") == ControlModeReason.SAFETY_STOP_ACTIVE
    assert parse_control_reason("timeout") == ControlModeReason.TIMEOUT


def test_parse_control_reason_uses_default_for_invalid_value():
    assert parse_control_reason("bad") == ControlModeReason.UNKNOWN
    assert (
        parse_control_reason("bad", default=ControlModeReason.NONE)
        == ControlModeReason.NONE
    )


def test_mode_display_label():
    assert mode_display_label("STOP") == "STOP (safe hold)"
    assert mode_display_label("IDLE") == "STOP (safe hold)"
    assert mode_display_label("MANUAL") == "MANUAL (teleop/app)"
    assert mode_display_label("TELEOP") == "MANUAL (teleop/app)"
    assert mode_display_label("AUTO") == "AUTO (local auto)"
    assert mode_display_label("NAV") == "NAV (Nav2 goal)"
    assert mode_display_label("RECOVERY") == "RECOVERY (override path)"
    assert mode_display_label("bad") == "STOP (safe hold)"
