#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control.interfaces.control_modes
==================================================

Purpose
-------
Lean Python-side control mode contract aligned with the authoritative
C++ control mode manager (`control_mode_manager_node.cpp` + control_mode_manager.hpp).

This module is intentionally minimal:
- enum names aligned with C++ runtime mode strings
- parsing aliases aligned with C++ on_mode_cmd_() behavior
- helper functions for Python dashboards/tests/tools
- no transition policy or arbitration logic (owned by C++)

Why this matters (real robot)
-----------------------------
On a real robot, duplicated mode-policy logic across C++ and Python can drift and
cause unsafe behavior. The C++ mode manager is the single source of truth.
Python uses this file only for parsing, validation, and display convenience.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional


# =============================================================================
# Authoritative runtime modes (aligned with C++)
# =============================================================================

class ControlMode(str, Enum):
    """
    Control modes aligned with the C++ ControlMode enum used by control_mode_manager.

    Canonical runtime strings published/consumed in your stack:
      - STOP
      - MANUAL
      - AUTO
      - NAV
      - RECOVERY
    """
    STOP = "STOP"
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    NAV = "NAV"
    RECOVERY = "RECOVERY"


# =============================================================================
# Optional helper enums for Python-side diagnostics/status parsing
# (These mirror strings seen in C++ debug/status output where useful.)
# =============================================================================

class ControlModeReason(str, Enum):
    """
    Reasons emitted by C++ status JSON/debug text (non-authoritative helper enum).
    """
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


# =============================================================================
# Parsing aliases (aligned with your C++ on_mode_cmd_ and parse_mode_or_default_)
# =============================================================================

_MODE_ALIASES = {
    # canonical
    "STOP": ControlMode.STOP,
    "MANUAL": ControlMode.MANUAL,
    "AUTO": ControlMode.AUTO,
    "NAV": ControlMode.NAV,
    "RECOVERY": ControlMode.RECOVERY,

    # compatibility aliases used in your C++ node
    "IDLE": ControlMode.STOP,          # C++ treats IDLE as STOP
    "TELEOP": ControlMode.MANUAL,
    "MAN": ControlMode.MANUAL,
    "AUTONOMOUS": ControlMode.AUTO,
    "AUTON": ControlMode.AUTO,
    "NAV2": ControlMode.NAV,
}


# =============================================================================
# Core helpers
# =============================================================================

def normalize_mode_text(value: object) -> str:
    """
    Normalize incoming text the same way the C++ node does conceptually:
    - strip whitespace
    - uppercase
    - convert '-' and spaces to '_'

    This helps Python-side tools accept user/test input robustly.
    """
    if value is None:
        return ""
    s = str(value).strip().upper()
    s = s.replace("-", "_").replace(" ", "_")
    return s


def parse_control_mode(
    value: object,
    default: Optional[ControlMode] = None,
) -> ControlMode:
    """
    Parse a mode string/object into ControlMode using aliases aligned with C++.

    Examples
    --------
    parse_control_mode("manual") -> ControlMode.MANUAL
    parse_control_mode("nav2")   -> ControlMode.NAV
    parse_control_mode("idle")   -> ControlMode.STOP

    Parameters
    ----------
    value:
        Input mode text or enum.
    default:
        If provided, returned when parsing fails. If None, ValueError is raised.

    Returns
    -------
    ControlMode
    """
    if isinstance(value, ControlMode):
        return value

    key = normalize_mode_text(value)
    mode = _MODE_ALIASES.get(key)

    if mode is not None:
        return mode

    if default is not None:
        return default

    valid = ", ".join(m.value for m in ControlMode)
    raise ValueError(f"Invalid control mode '{value}'. Valid modes: {valid}")


def is_stop_like(value: object) -> bool:
    """
    True if the given value maps to STOP semantics (including IDLE alias).
    """
    try:
        return parse_control_mode(value) == ControlMode.STOP
    except ValueError:
        return False


def is_motion_mode(value: object) -> bool:
    """
    True if mode can be associated with motion-producing command paths.

    NOTE:
    Actual motion is still governed by the C++ control stack + safety gate.
    This is only a helper for Python dashboards/tests.
    """
    try:
        mode = parse_control_mode(value)
    except ValueError:
        return False

    return mode in {
        ControlMode.MANUAL,
        ControlMode.AUTO,
        ControlMode.NAV,
        ControlMode.RECOVERY,
    }


def safe_mode_fallback() -> ControlMode:
    """
    Safe fallback for Python tools when mode text is missing/invalid.

    Aligned with your C++ node behavior where unknown commands are rejected,
    and STOP is the safest neutral mode for display/testing fallbacks.
    """
    return ControlMode.STOP


# =============================================================================
# Mux-facing compatibility helpers (aligned with C++ publish_plain_mode_state_)
# =============================================================================

def plain_mux_mode_string(mode: object) -> str:
    """
    Return the plain mode string style expected by twist_mux_node.

    Mirrors the C++ logic in plain_mux_mode_string_():
      - MANUAL -> "MANUAL"
      - AUTO   -> "AUTO"
      - NAV    -> "NAV"
      - RECOVERY -> "STOP"  (safe neutral base mode; recovery path handled separately)
      - STOP/default -> "STOP"
    """
    m = parse_control_mode(mode, default=ControlMode.STOP)

    if m == ControlMode.MANUAL:
        return "MANUAL"
    if m == ControlMode.AUTO:
        return "AUTO"
    if m == ControlMode.NAV:
        return "NAV"
    if m == ControlMode.RECOVERY:
        return "STOP"   # matches your C++ comment/behavior
    return "STOP"


# =============================================================================
# Optional helpers for Python dashboards/status parsing
# =============================================================================

def parse_control_reason(value: object, default: ControlModeReason = ControlModeReason.UNKNOWN) -> ControlModeReason:
    """
    Parse C++ reason strings from /savo_control/control_status JSON or debug outputs.
    """
    if isinstance(value, ControlModeReason):
        return value

    key = normalize_mode_text(value)
    # reason names are already underscore style, so direct enum check is enough
    for reason in ControlModeReason:
        if key == reason.value:
            return reason
    return default


def mode_display_label(mode: object) -> str:
    """
    Human-friendly stable label for dashboards/UI text.
    """
    m = parse_control_mode(mode, default=ControlMode.STOP)
    labels = {
        ControlMode.STOP: "STOP (safe hold)",
        ControlMode.MANUAL: "MANUAL (teleop/app)",
        ControlMode.AUTO: "AUTO (local auto)",
        ControlMode.NAV: "NAV (Nav2 goal)",
        ControlMode.RECOVERY: "RECOVERY (override path)",
    }
    return labels[m]


# =============================================================================
# Self-test (no ROS dependency)
# =============================================================================

if __name__ == "__main__":
    tests = ["stop", "manual", "teleop", "man", "auto", "auton", "nav2", "idle", "recovery", "bad"]
    print("=== control_modes.py self-test (C++-aligned) ===")
    for t in tests:
        try:
            m = parse_control_mode(t)
            print(
                f"{t!r:>10} -> {m.value:8s} | "
                f"motion={is_motion_mode(m)} | "
                f"mux='{plain_mux_mode_string(m)}'"
            )
        except ValueError as e:
            print(f"{t!r:>10} -> ERROR: {e}")

    print("safe_mode_fallback =", safe_mode_fallback().value)