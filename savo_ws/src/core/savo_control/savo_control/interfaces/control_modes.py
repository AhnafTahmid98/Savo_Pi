# -*- coding: utf-8 -*-

"""Compatibility helpers for control mode parsing and display."""

from __future__ import annotations

from enum import Enum

from savo_control.models import ControlMode, normalize_mode


class ControlModeReason(str, Enum):
    NONE = "NONE"
    STARTUP = "STARTUP"
    REQUESTED = "REQUESTED"
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
    RECOVERY_ACTIVE = "RECOVERY_ACTIVE"
    RECOVERY_LATCHED = "RECOVERY_LATCHED"
    SAFETY_STOP_ACTIVE = "SAFETY_STOP_ACTIVE"
    EXTERNAL_STOP = "EXTERNAL_STOP"
    INVALID_TIME = "INVALID_TIME"
    TIMEOUT = "TIMEOUT"
    NO_SOURCE_ALLOWED = "NO_SOURCE_ALLOWED"
    UNKNOWN = "UNKNOWN"


_MODE_ALIASES = {
    "STOP": ControlMode.STOP,
    "IDLE": ControlMode.STOP,
    "DISABLED": ControlMode.STOP,
    "MANUAL": ControlMode.MANUAL,
    "TELEOP": ControlMode.MANUAL,
    "MAN": ControlMode.MANUAL,
    "AUTO": ControlMode.AUTO,
    "AUTONOMOUS": ControlMode.AUTO,
    "AUTON": ControlMode.AUTO,
    "NAV": ControlMode.NAV,
    "NAV2": ControlMode.NAV,
    "RECOVERY": ControlMode.RECOVERY,
}


def normalize_mode_text(value: object) -> str:
    if value is None:
        return ""

    return str(value).strip().upper().replace("-", "_").replace(" ", "_")


def parse_control_mode(
    value: object,
    default: ControlMode | None = None,
) -> ControlMode:
    if isinstance(value, ControlMode):
        return value

    key = normalize_mode_text(value)
    mode = _MODE_ALIASES.get(key)

    if mode is not None:
        return mode

    if default is not None:
        return default

    return ControlMode.from_text(value)


def is_stop_like(value: object) -> bool:
    return parse_control_mode(value, default=ControlMode.STOP) == ControlMode.STOP


def is_motion_mode(value: object) -> bool:
    mode = parse_control_mode(value, default=ControlMode.STOP)

    return mode in {
        ControlMode.MANUAL,
        ControlMode.AUTO,
        ControlMode.NAV,
        ControlMode.RECOVERY,
    }


def safe_mode_fallback() -> ControlMode:
    return ControlMode.STOP


def plain_mux_mode_string(mode: object) -> str:
    parsed = parse_control_mode(mode, default=ControlMode.STOP)

    if parsed == ControlMode.MANUAL:
        return ControlMode.MANUAL.value
    if parsed == ControlMode.AUTO:
        return ControlMode.AUTO.value
    if parsed == ControlMode.NAV:
        return ControlMode.NAV.value

    return ControlMode.STOP.value


def parse_control_reason(
    value: object,
    default: ControlModeReason = ControlModeReason.UNKNOWN,
) -> ControlModeReason:
    if isinstance(value, ControlModeReason):
        return value

    key = normalize_mode_text(value)

    for reason in ControlModeReason:
        if key == reason.value:
            return reason

    return default


def mode_display_label(mode: object) -> str:
    parsed = parse_control_mode(mode, default=ControlMode.STOP)

    labels = {
        ControlMode.STOP: "STOP (safe hold)",
        ControlMode.MANUAL: "MANUAL (teleop/app)",
        ControlMode.AUTO: "AUTO (local auto)",
        ControlMode.NAV: "NAV (Nav2 goal)",
        ControlMode.RECOVERY: "RECOVERY (override path)",
    }

    return labels[parsed]


__all__ = [
    "ControlMode",
    "ControlModeReason",
    "is_motion_mode",
    "is_stop_like",
    "mode_display_label",
    "normalize_mode",
    "normalize_mode_text",
    "parse_control_mode",
    "parse_control_reason",
    "plain_mux_mode_string",
    "safe_mode_fallback",
]
