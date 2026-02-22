#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/models/watchdog_state.py
-----------------------------------------------
Professional watchdog state models for Robot Savo (ROS 2 Jazzy, real robot).

Purpose
- Track command freshness / watchdog timing for savo_base motion execution
- Support safe timeout behavior (zero outputs when commands become stale)
- Provide clear runtime status for logging, dashboards, and diagnostics

Scope
- Pure Python data models (no ROS imports, no hardware I/O)
- Intended for use by:
  - savo_base execution node / cmd loop
  - motor board adapters
  - teleop / LLM text-command bridge (indirectly via command timestamps)
  - diagnostics dashboards and telemetry
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Any, Optional
import math
import time


# =============================================================================
# Exceptions
# =============================================================================
class WatchdogStateValidationError(ValueError):
    """Raised when a watchdog model contains invalid values."""
    pass


# =============================================================================
# Helpers
# =============================================================================
def _is_finite(x: float) -> bool:
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def _now_unix() -> float:
    return float(time.time())


def _now_mono() -> float:
    return float(time.monotonic())


def _clamp(x: float, lo: float, hi: float) -> float:
    if lo > hi:
        raise WatchdogStateValidationError(f"Invalid clamp range: lo={lo} > hi={hi}")
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


# =============================================================================
# Watchdog configuration snapshot
# =============================================================================
@dataclass
class WatchdogConfigSnapshot:
    """
    Runtime configuration snapshot for the command watchdog.

    Notes
    -----
    - timeout_s: command freshness timeout. If no command arrives within this time,
      motion outputs should be zeroed (watchdog trip).
    - loop_hz: main execution loop target (used for diagnostics only here).
    - auto_rearm: if True, watchdog may clear tripped state automatically when a
      fresh command is received again.
    """
    enabled: bool = True
    timeout_s: float = 0.30
    loop_hz: float = 30.0
    auto_rearm: bool = True
    zero_on_trip: bool = True
    warn_ratio: float = 0.70     # warn when age/timeout >= warn_ratio
    stale_ratio: float = 1.00    # stale threshold, normally 1.0
    source_name: str = "savo_base"

    def validate(self) -> "WatchdogConfigSnapshot":
        if not isinstance(self.enabled, bool):
            raise WatchdogStateValidationError("enabled must be bool")
        if not isinstance(self.auto_rearm, bool):
            raise WatchdogStateValidationError("auto_rearm must be bool")
        if not isinstance(self.zero_on_trip, bool):
            raise WatchdogStateValidationError("zero_on_trip must be bool")

        if not _is_finite(self.timeout_s) or float(self.timeout_s) <= 0.0:
            raise WatchdogStateValidationError("timeout_s must be finite and > 0")
        if not _is_finite(self.loop_hz) or float(self.loop_hz) <= 0.0:
            raise WatchdogStateValidationError("loop_hz must be finite and > 0")

        if not _is_finite(self.warn_ratio):
            raise WatchdogStateValidationError("warn_ratio must be finite")
        if not _is_finite(self.stale_ratio):
            raise WatchdogStateValidationError("stale_ratio must be finite")

        self.warn_ratio = float(_clamp(float(self.warn_ratio), 0.0, 10.0))
        self.stale_ratio = float(_clamp(float(self.stale_ratio), 0.1, 10.0))

        if self.warn_ratio > self.stale_ratio:
            raise WatchdogStateValidationError(
                f"warn_ratio ({self.warn_ratio}) must be <= stale_ratio ({self.stale_ratio})"
            )

        if not isinstance(self.source_name, str) or not self.source_name.strip():
            raise WatchdogStateValidationError("source_name must be non-empty str")

        return self

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "enabled": self.enabled,
            "timeout_s": float(self.timeout_s),
            "loop_hz": float(self.loop_hz),
            "auto_rearm": self.auto_rearm,
            "zero_on_trip": self.zero_on_trip,
            "warn_ratio": float(self.warn_ratio),
            "stale_ratio": float(self.stale_ratio),
            "source_name": self.source_name,
        }


# =============================================================================
# Watchdog counters / statistics
# =============================================================================
@dataclass
class WatchdogCounters:
    """Counters for watchdog events and command freshness tracking."""
    tick_count: int = 0
    command_rx_count: int = 0
    trip_count: int = 0
    auto_rearm_count: int = 0
    manual_reset_count: int = 0
    zero_action_count: int = 0
    warning_count: int = 0

    def validate(self) -> "WatchdogCounters":
        for name in (
            "tick_count",
            "command_rx_count",
            "trip_count",
            "auto_rearm_count",
            "manual_reset_count",
            "zero_action_count",
            "warning_count",
        ):
            v = getattr(self, name)
            if not isinstance(v, int) or v < 0:
                raise WatchdogStateValidationError(f"{name} must be int >= 0")
        return self

    def to_dict(self) -> Dict[str, int]:
        self.validate()
        return {
            "tick_count": int(self.tick_count),
            "command_rx_count": int(self.command_rx_count),
            "trip_count": int(self.trip_count),
            "auto_rearm_count": int(self.auto_rearm_count),
            "manual_reset_count": int(self.manual_reset_count),
            "zero_action_count": int(self.zero_action_count),
            "warning_count": int(self.warning_count),
        }


# =============================================================================
# Main watchdog state
# =============================================================================
@dataclass
class WatchdogState:
    """
    Aggregated command-watchdog runtime state for Robot Savo base execution.

    Typical flow (savo_base executor)
    ---------------------------------
    - on command receive: call record_command_rx(...)
    - each loop tick: call tick(...)
    - if tripped and zero_on_trip: execution layer zeros outputs and records action
    """
    name: str = "base_command_watchdog"
    robot_name: str = "Robot Savo"

    config: WatchdogConfigSnapshot = field(default_factory=WatchdogConfigSnapshot)
    counters: WatchdogCounters = field(default_factory=WatchdogCounters)

    # Lifecycle / state flags
    initialized: bool = False
    active: bool = False            # watchdog logic running
    tripped: bool = False           # stale timeout exceeded
    stale: bool = False             # command age >= stale threshold
    warning: bool = False           # command age >= warning threshold
    zero_action_pending: bool = False  # execution loop should zero outputs now

    # Timestamps (monotonic = source of truth for freshness)
    created_unix_s: float = field(default_factory=_now_unix)
    last_update_unix_s: float = field(default_factory=_now_unix)
    last_tick_monotonic_s: float = 0.0
    last_command_rx_monotonic_s: float = 0.0
    last_trip_monotonic_s: float = 0.0
    last_rearm_monotonic_s: float = 0.0
    last_manual_reset_monotonic_s: float = 0.0
    last_zero_action_monotonic_s: float = 0.0

    # Metadata
    last_command_source: str = ""      # e.g. "teleop", "llm_bridge", "nav"
    last_command_note: str = ""
    last_warning_msg: str = ""
    last_error_msg: str = ""
    note: str = ""

    def validate(self) -> "WatchdogState":
        if not isinstance(self.name, str) or not self.name.strip():
            raise WatchdogStateValidationError("name must be non-empty str")
        if not isinstance(self.robot_name, str) or not self.robot_name.strip():
            raise WatchdogStateValidationError("robot_name must be non-empty str")

        self.config.validate()
        self.counters.validate()

        for f in ("initialized", "active", "tripped", "stale", "warning", "zero_action_pending"):
            if not isinstance(getattr(self, f), bool):
                raise WatchdogStateValidationError(f"{f} must be bool")

        for f in (
            "created_unix_s",
            "last_update_unix_s",
            "last_tick_monotonic_s",
            "last_command_rx_monotonic_s",
            "last_trip_monotonic_s",
            "last_rearm_monotonic_s",
            "last_manual_reset_monotonic_s",
            "last_zero_action_monotonic_s",
        ):
            v = getattr(self, f)
            if not _is_finite(v) or float(v) < 0.0:
                raise WatchdogStateValidationError(f"{f} must be finite and >= 0, got {v!r}")

        for f in ("last_command_source", "last_command_note", "last_warning_msg", "last_error_msg", "note"):
            if not isinstance(getattr(self, f), str):
                raise WatchdogStateValidationError(f"{f} must be str")

        # Normalize flags if disabled
        if not self.config.enabled:
            self.tripped = False
            self.stale = False
            self.warning = False
            self.zero_action_pending = False

        # If not active, no pending zero action should remain
        if not self.active:
            self.zero_action_pending = False

        return self

    # -------------------------------------------------------------------------
    # Basic timing helpers
    # -------------------------------------------------------------------------
    def touch(self) -> "WatchdogState":
        self.last_update_unix_s = _now_unix()
        return self

    def command_age_s(self) -> float:
        self.validate()
        if self.last_command_rx_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - float(self.last_command_rx_monotonic_s))

    def time_since_trip_s(self) -> float:
        self.validate()
        if self.last_trip_monotonic_s <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - float(self.last_trip_monotonic_s))

    def timeout_ratio(self) -> float:
        self.validate()
        age = self.command_age_s()
        if not math.isfinite(age):
            return math.inf
        return float(age / float(self.config.timeout_s))

    # -------------------------------------------------------------------------
    # Lifecycle control
    # -------------------------------------------------------------------------
    def mark_initialized(self, *, active: bool = True) -> "WatchdogState":
        self.initialized = True
        self.active = bool(active)
        self.touch()
        return self.validate()

    def set_active(self, active: bool) -> "WatchdogState":
        self.active = bool(active)
        if not self.active:
            self.warning = False
            self.stale = False
            self.tripped = False
            self.zero_action_pending = False
        self.touch()
        return self.validate()

    # -------------------------------------------------------------------------
    # Command reception
    # -------------------------------------------------------------------------
    def record_command_rx(self, *, source: str = "", note: str = "") -> "WatchdogState":
        """
        Record arrival of a fresh motion command (cmd_vel, wheel command, etc.).

        If watchdog was tripped and auto_rearm is enabled, this re-arms it.
        """
        self.validate()

        now = _now_mono()
        self.counters.command_rx_count += 1
        self.last_command_rx_monotonic_s = now
        self.last_command_source = str(source)
        self.last_command_note = str(note)

        # Fresh command clears warning/stale immediately
        self.warning = False
        self.stale = False
        self.last_warning_msg = ""

        # Auto re-arm behavior after trip
        if self.tripped and self.config.enabled and self.config.auto_rearm and self.active:
            self.tripped = False
            self.zero_action_pending = False
            self.last_rearm_monotonic_s = now
            self.counters.auto_rearm_count += 1

        self.touch()
        return self.validate()

    # -------------------------------------------------------------------------
    # Tick / evaluation
    # -------------------------------------------------------------------------
    def tick(self, now_monotonic_s: Optional[float] = None) -> "WatchdogState":
        """
        Evaluate watchdog state based on command freshness.

        Call once per control loop iteration.
        """
        self.validate()

        now = _now_mono() if now_monotonic_s is None else float(now_monotonic_s)
        if not _is_finite(now) or now < 0.0:
            raise WatchdogStateValidationError(f"now_monotonic_s must be finite and >=0, got {now!r}")

        self.counters.tick_count += 1
        self.last_tick_monotonic_s = now

        if not self.initialized or not self.active or not self.config.enabled:
            self.warning = False
            self.stale = False
            self.tripped = False
            self.zero_action_pending = False
            self.touch()
            return self.validate()

        # No command has ever been received -> treat as stale (and trip if enabled)
        if self.last_command_rx_monotonic_s <= 0.0:
            self.warning = True
            self.stale = True
            self.last_warning_msg = "No command received yet"
            if not self.tripped:
                self._trip(now=now, reason="No command received (startup stale)")
            self.touch()
            return self.validate()

        age = max(0.0, now - float(self.last_command_rx_monotonic_s))
        ratio = age / float(self.config.timeout_s)

        # Warning region
        if ratio >= float(self.config.warn_ratio):
            if not self.warning:
                self.counters.warning_count += 1
            self.warning = True
            self.last_warning_msg = (
                f"Command aging: age={age:.3f}s "
                f"({ratio:.2f}x timeout {self.config.timeout_s:.3f}s)"
            )
        else:
            self.warning = False
            self.last_warning_msg = ""

        # Stale / trip region
        self.stale = (ratio >= float(self.config.stale_ratio))
        if self.stale and not self.tripped:
            self._trip(now=now, reason=f"Command timeout: age={age:.3f}s >={self.config.timeout_s:.3f}s")

        self.touch()
        return self.validate()

    def _trip(self, *, now: float, reason: str) -> None:
        """Internal trip handler."""
        self.tripped = True
        self.last_trip_monotonic_s = float(now)
        self.counters.trip_count += 1
        self.last_error_msg = str(reason)

        if self.config.zero_on_trip:
            self.zero_action_pending = True

    # -------------------------------------------------------------------------
    # Execution-layer acknowledgements
    # -------------------------------------------------------------------------
    def mark_zero_action_done(self, *, note: str = "Outputs zeroed by watchdog") -> "WatchdogState":
        """
        Call this after the execution layer actually zeroes motor outputs due to watchdog.
        """
        self.validate()
        self.zero_action_pending = False
        self.counters.zero_action_count += 1
        self.last_zero_action_monotonic_s = _now_mono()
        self.note = str(note)
        self.touch()
        return self.validate()

    # -------------------------------------------------------------------------
    # Manual control / reset
    # -------------------------------------------------------------------------
    def manual_reset(self, *, clear_last_error: bool = False, note: str = "") -> "WatchdogState":
        """
        Manual watchdog reset (e.g., operator action). Does not fake a command receive.
        """
        self.validate()
        now = _now_mono()

        self.tripped = False
        self.stale = False
        self.warning = False
        self.zero_action_pending = False
        self.last_warning_msg = ""
        if clear_last_error:
            self.last_error_msg = ""

        self.counters.manual_reset_count += 1
        self.last_manual_reset_monotonic_s = now
        if note:
            self.note = str(note)

        self.touch()
        return self.validate()

    # -------------------------------------------------------------------------
    # Status helpers
    # -------------------------------------------------------------------------
    def status_level(self) -> str:
        self.validate()

        if not self.config.enabled:
            return "DISABLED"
        if not self.initialized:
            return "UNINITIALIZED"
        if not self.active:
            return "INACTIVE"
        if self.tripped:
            return "TRIPPED"
        if self.stale:
            return "STALE"
        if self.warning:
            return "WARNING"
        return "OK"

    def short_summary(self) -> str:
        self.validate()
        age = self.command_age_s()
        age_str = "inf" if not math.isfinite(age) else f"{age:.3f}s"
        return (
            f"[{self.status_level()}] wd={self.name} enabled={self.config.enabled} active={self.active} "
            f"timeout={self.config.timeout_s:.3f}s age={age_str} "
            f"tripped={self.tripped} zero_pending={self.zero_action_pending} "
            f"cmd_rx={self.counters.command_rx_count} trips={self.counters.trip_count}"
        )

    def to_dict(self) -> Dict[str, Any]:
        self.validate()

        age = self.command_age_s()
        ratio = self.timeout_ratio()

        return {
            "name": self.name,
            "robot_name": self.robot_name,
            "status_level": self.status_level(),

            "config": self.config.to_dict(),
            "counters": self.counters.to_dict(),

            "flags": {
                "initialized": self.initialized,
                "active": self.active,
                "tripped": self.tripped,
                "stale": self.stale,
                "warning": self.warning,
                "zero_action_pending": self.zero_action_pending,
            },

            "timing": {
                "created_unix_s": float(self.created_unix_s),
                "last_update_unix_s": float(self.last_update_unix_s),
                "last_tick_monotonic_s": float(self.last_tick_monotonic_s),
                "last_command_rx_monotonic_s": float(self.last_command_rx_monotonic_s),
                "last_trip_monotonic_s": float(self.last_trip_monotonic_s),
                "last_rearm_monotonic_s": float(self.last_rearm_monotonic_s),
                "last_manual_reset_monotonic_s": float(self.last_manual_reset_monotonic_s),
                "last_zero_action_monotonic_s": float(self.last_zero_action_monotonic_s),
                "command_age_s": age,
                "timeout_ratio": ratio,
                "time_since_trip_s": self.time_since_trip_s(),
            },

            "diagnostics": {
                "last_command_source": self.last_command_source,
                "last_command_note": self.last_command_note,
                "last_warning_msg": self.last_warning_msg,
                "last_error_msg": self.last_error_msg,
                "note": self.note,
                "summary": self.short_summary(),
            },
        }


# =============================================================================
# Canonical constructors for Robot Savo
# =============================================================================
def make_robot_savo_watchdog_state(
    *,
    timeout_s: float = 0.30,
    loop_hz: float = 30.0,
    enabled: bool = True,
    auto_rearm: bool = True,
    zero_on_trip: bool = True,
    warn_ratio: float = 0.70,
    stale_ratio: float = 1.00,
    source_name: str = "savo_base",
) -> WatchdogState:
    """
    Create a validated watchdog state for Robot Savo base command freshness monitoring.

    Recommended baseline (real robot)
    ---------------------------------
    - loop_hz ≈ 30
    - timeout_s ≈ 0.30 (consistent with your safety/perception stale strategy)
    """
    wd = WatchdogState(
        name="base_command_watchdog",
        robot_name="Robot Savo",
        config=WatchdogConfigSnapshot(
            enabled=bool(enabled),
            timeout_s=float(timeout_s),
            loop_hz=float(loop_hz),
            auto_rearm=bool(auto_rearm),
            zero_on_trip=bool(zero_on_trip),
            warn_ratio=float(warn_ratio),
            stale_ratio=float(stale_ratio),
            source_name=str(source_name),
        ),
    )
    return wd.validate()


__all__ = [
    "WatchdogStateValidationError",
    "WatchdogConfigSnapshot",
    "WatchdogCounters",
    "WatchdogState",
    "make_robot_savo_watchdog_state",
]