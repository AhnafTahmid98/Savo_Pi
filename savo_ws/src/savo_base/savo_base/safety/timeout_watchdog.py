#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/timeout_watchdog.py
-------------------------------------------------
Professional timeout watchdog helper for `savo_base` (ROS2 Jazzy / real robot).

Purpose
-------
Detect stale command/control updates and expose a deterministic watchdog trip state
for safe motor stopping.

Typical use in Robot Savo
-------------------------
- `base_driver_node.py` or `base_watchdog_node.py` feeds timestamps whenever a
  fresh motion command is received (e.g., from /cmd_vel_safe).
- If no fresh command arrives within timeout, watchdog trips.
- Base output path forces stop until valid commands resume (or manual reset,
  depending on policy).

Design goals
------------
- Pure Python (no ROS dependency)
- Monotonic time-based (safe against wall-clock jumps)
- Explicit policy and state transitions
- Rich status for logs / state topics / dashboards
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import time


# =============================================================================
# Dataclasses
# =============================================================================
@dataclass
class TimeoutWatchdogConfig:
    """
    Watchdog behavior configuration.
    """
    timeout_s: float = 0.50

    # If True, watchdog auto-clears when a fresh kick arrives after a trip.
    # If False, watchdog remains tripped until manual clear/reset.
    auto_clear_on_fresh_kick: bool = True

    # If True, first check before any kick can be considered tripped.
    # Useful when a node expects commands immediately after startup.
    trip_if_never_kicked: bool = False


@dataclass
class TimeoutWatchdogState:
    """
    Runtime state of the watchdog.
    """
    tripped: bool = False
    ever_kicked: bool = False

    last_reason: str = "init"

    # Monotonic timestamps
    last_kick_s: Optional[float] = None
    last_check_s: Optional[float] = None
    last_trip_s: Optional[float] = None
    last_clear_s: Optional[float] = None

    # Diagnostics
    current_age_s: Optional[float] = None
    timeout_s: float = 0.50


@dataclass
class TimeoutWatchdogResult:
    """
    Result returned by kick()/check()/clear().
    """
    tripped: bool
    changed: bool
    event: str          # e.g., "kicked", "tripped", "cleared", "no_change"
    reason: str
    state: TimeoutWatchdogState = field(default_factory=TimeoutWatchdogState)


# =============================================================================
# Helpers
# =============================================================================
def monotonic_time_s() -> float:
    """Return monotonic time in seconds."""
    return time.monotonic()


def _copy_state(s: TimeoutWatchdogState) -> TimeoutWatchdogState:
    return TimeoutWatchdogState(
        tripped=s.tripped,
        ever_kicked=s.ever_kicked,
        last_reason=s.last_reason,
        last_kick_s=s.last_kick_s,
        last_check_s=s.last_check_s,
        last_trip_s=s.last_trip_s,
        last_clear_s=s.last_clear_s,
        current_age_s=s.current_age_s,
        timeout_s=s.timeout_s,
    )


# =============================================================================
# Core watchdog
# =============================================================================
class TimeoutWatchdog:
    """
    Monotonic timeout watchdog for command freshness.

    Core semantics
    --------------
    - `kick()` marks a fresh event (e.g., new cmd_vel_safe received).
    - `check()` evaluates timeout and may trip watchdog.
    - `is_tripped(now)` combines state + timeout evaluation (non-mutating helper
      available via `peek_status`, mutating via `check`).
    - `clear()` manually clears a tripped watchdog (if policy requires).

    Recommended pattern in real robot base loop
    -------------------------------------------
    - On every fresh command callback: `watchdog.kick()`
    - In control loop tick: `wd = watchdog.check()`
    - If `wd.tripped`: force stop command output
    """

    def __init__(
        self,
        config: Optional[TimeoutWatchdogConfig] = None,
        *,
        timeout_s: Optional[float] = None,
        auto_clear_on_fresh_kick: Optional[bool] = None,
        trip_if_never_kicked: Optional[bool] = None,
        start_tripped: bool = False,
    ) -> None:
        cfg = config or TimeoutWatchdogConfig()
        if timeout_s is not None:
            cfg.timeout_s = float(timeout_s)
        if auto_clear_on_fresh_kick is not None:
            cfg.auto_clear_on_fresh_kick = bool(auto_clear_on_fresh_kick)
        if trip_if_never_kicked is not None:
            cfg.trip_if_never_kicked = bool(trip_if_never_kicked)

        self.config = TimeoutWatchdogConfig(
            timeout_s=max(0.01, float(cfg.timeout_s)),
            auto_clear_on_fresh_kick=bool(cfg.auto_clear_on_fresh_kick),
            trip_if_never_kicked=bool(cfg.trip_if_never_kicked),
        )

        self._state = TimeoutWatchdogState(
            tripped=bool(start_tripped),
            ever_kicked=False,
            last_reason="startup_tripped" if start_tripped else "startup_clear",
            timeout_s=self.config.timeout_s,
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    @property
    def state(self) -> TimeoutWatchdogState:
        return _copy_state(self._state)

    @property
    def tripped(self) -> bool:
        return self._state.tripped

    @property
    def timeout_s(self) -> float:
        return self.config.timeout_s

    def set_timeout(self, timeout_s: float) -> TimeoutWatchdogResult:
        """
        Update timeout at runtime (e.g., from params reload).
        """
        self.config.timeout_s = max(0.01, float(timeout_s))
        self._state.timeout_s = self.config.timeout_s
        self._state.last_reason = "timeout_updated"
        return TimeoutWatchdogResult(
            tripped=self._state.tripped,
            changed=False,
            event="timeout_updated",
            reason=self._state.last_reason,
            state=self.state,
        )

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _now(self, now_monotonic_s: Optional[float]) -> float:
        return monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)

    def _compute_age(self, now_s: float) -> Optional[float]:
        if self._state.last_kick_s is None:
            return None
        return max(0.0, now_s - self._state.last_kick_s)

    def _trip(self, *, now_s: float, reason: str) -> TimeoutWatchdogResult:
        prev = self._state.tripped
        self._state.tripped = True
        self._state.last_reason = reason
        self._state.last_check_s = now_s
        self._state.current_age_s = self._compute_age(now_s)
        if not prev:
            self._state.last_trip_s = now_s

        return TimeoutWatchdogResult(
            tripped=True,
            changed=(not prev),
            event="tripped" if not prev else "no_change",
            reason=reason,
            state=self.state,
        )

    def _clear_internal(self, *, now_s: float, reason: str, event: str) -> TimeoutWatchdogResult:
        prev = self._state.tripped
        self._state.tripped = False
        self._state.last_reason = reason
        self._state.last_check_s = now_s
        self._state.current_age_s = self._compute_age(now_s)
        if prev:
            self._state.last_clear_s = now_s

        return TimeoutWatchdogResult(
            tripped=False,
            changed=prev,
            event=event if prev else "no_change",
            reason=reason,
            state=self.state,
        )

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    def kick(
        self,
        *,
        source: str = "command",
        now_monotonic_s: Optional[float] = None,
        event_stamp_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogResult:
        """
        Register a fresh event (kick the watchdog).

        Parameters
        ----------
        source : str
            Diagnostic source label, e.g., "cmd_vel_safe".
        now_monotonic_s : float | None
            Processing time (defaults to current monotonic time).
        event_stamp_monotonic_s : float | None
            Optional upstream event time if you want to preserve callback stamp
            timing. If omitted, `now_monotonic_s` is used.

        Behavior
        --------
        - updates last_kick time
        - can auto-clear a tripped watchdog depending on policy
        """
        now_s = self._now(now_monotonic_s)
        kick_s = now_s if event_stamp_monotonic_s is None else float(event_stamp_monotonic_s)

        # Guard against accidental future timestamps (shouldn't happen, but safe)
        if kick_s > now_s:
            kick_s = now_s

        self._state.last_kick_s = kick_s
        self._state.ever_kicked = True
        self._state.last_check_s = now_s
        self._state.current_age_s = max(0.0, now_s - kick_s)

        if self._state.tripped and self.config.auto_clear_on_fresh_kick:
            self._state.last_reason = f"auto_cleared_by_kick_{source}"
            self._state.last_clear_s = now_s
            self._state.tripped = False
            return TimeoutWatchdogResult(
                tripped=False,
                changed=True,
                event="cleared",
                reason=self._state.last_reason,
                state=self.state,
            )

        self._state.last_reason = f"kicked_by_{source}"
        return TimeoutWatchdogResult(
            tripped=self._state.tripped,
            changed=False,
            event="kicked",
            reason=self._state.last_reason,
            state=self.state,
        )

    def check(
        self,
        *,
        source: str = "periodic_check",
        now_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogResult:
        """
        Evaluate timeout and update tripped state.

        This is the main periodic call used in the control loop.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_check_s = now_s
        self._state.timeout_s = self.config.timeout_s

        # No kick received yet
        if not self._state.ever_kicked:
            self._state.current_age_s = None
            if self.config.trip_if_never_kicked:
                return self._trip(now_s=now_s, reason=f"never_kicked_{source}")
            self._state.last_reason = f"waiting_first_kick_{source}"
            return TimeoutWatchdogResult(
                tripped=self._state.tripped,
                changed=False,
                event="no_change",
                reason=self._state.last_reason,
                state=self.state,
            )

        age_s = self._compute_age(now_s)
        self._state.current_age_s = age_s

        if age_s is None:
            # Defensive fallback (shouldn't happen if ever_kicked=True)
            return self._trip(now_s=now_s, reason=f"internal_no_age_after_kick_{source}")

        if age_s > self.config.timeout_s:
            return self._trip(
                now_s=now_s,
                reason=f"timeout_{source}_age_{age_s:.3f}s_gt_{self.config.timeout_s:.3f}s",
            )

        # Still fresh: remain clear (unless manually latched by policy elsewhere)
        if self._state.tripped:
            # If tripped and no auto-clear policy, remain tripped until manual clear
            self._state.last_reason = f"still_tripped_waiting_manual_clear_{source}"
            return TimeoutWatchdogResult(
                tripped=True,
                changed=False,
                event="no_change",
                reason=self._state.last_reason,
                state=self.state,
            )

        self._state.last_reason = f"fresh_{source}"
        return TimeoutWatchdogResult(
            tripped=False,
            changed=False,
            event="no_change",
            reason=self._state.last_reason,
            state=self.state,
        )

    def clear(
        self,
        *,
        operator_confirmed: bool = False,
        source: str = "operator",
        now_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogResult:
        """
        Manually clear a tripped watchdog.

        Use this when `auto_clear_on_fresh_kick=False` or for controlled recovery.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_check_s = now_s

        if not self._state.tripped:
            self._state.last_reason = f"clear_ignored_not_tripped_{source}"
            self._state.current_age_s = self._compute_age(now_s)
            return TimeoutWatchdogResult(
                tripped=False,
                changed=False,
                event="clear_ignored",
                reason=self._state.last_reason,
                state=self.state,
            )

        if not operator_confirmed:
            self._state.last_reason = f"clear_rejected_operator_not_confirmed_{source}"
            self._state.current_age_s = self._compute_age(now_s)
            return TimeoutWatchdogResult(
                tripped=True,
                changed=False,
                event="clear_rejected",
                reason=self._state.last_reason,
                state=self.state,
            )

        return self._clear_internal(
            now_s=now_s,
            reason=f"cleared_by_{source}",
            event="cleared",
        )

    def force_trip(
        self,
        *,
        source: str = "software_fault",
        now_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogResult:
        """
        Force watchdog trip (software fault path / supervisory path).
        """
        now_s = self._now(now_monotonic_s)
        return self._trip(now_s=now_s, reason=f"forced_trip_{source}")

    def reset(
        self,
        *,
        clear_trip: bool = True,
        clear_kick_history: bool = False,
        source: str = "reset",
        now_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogResult:
        """
        Administrative reset for startup/recovery/testing.

        Parameters
        ----------
        clear_trip : bool
            Clear tripped state.
        clear_kick_history : bool
            Forget last kick timestamp and `ever_kicked`.
        """
        now_s = self._now(now_monotonic_s)
        changed = False

        if clear_trip and self._state.tripped:
            self._state.tripped = False
            self._state.last_clear_s = now_s
            changed = True

        if clear_kick_history:
            self._state.ever_kicked = False
            self._state.last_kick_s = None
            self._state.current_age_s = None

        self._state.last_check_s = now_s
        self._state.timeout_s = self.config.timeout_s
        self._state.last_reason = f"reset_{source}"

        return TimeoutWatchdogResult(
            tripped=self._state.tripped,
            changed=changed,
            event="reset",
            reason=self._state.last_reason,
            state=self.state,
        )

    # -------------------------------------------------------------------------
    # Non-mutating view helpers
    # -------------------------------------------------------------------------
    def peek_status(
        self,
        *,
        now_monotonic_s: Optional[float] = None,
    ) -> TimeoutWatchdogState:
        """
        Return a computed snapshot without mutating internal trip state.
        Useful for dashboards/diagnostics.
        """
        now_s = self._now(now_monotonic_s)
        snap = self.state
        snap.timeout_s = self.config.timeout_s

        if snap.last_kick_s is not None:
            snap.current_age_s = max(0.0, now_s - snap.last_kick_s)
        else:
            snap.current_age_s = None

        return snap

    def as_dict(self) -> dict:
        """
        JSON/state-topic friendly snapshot of current internal state.
        """
        s = self._state
        return {
            "tripped": s.tripped,
            "ever_kicked": s.ever_kicked,
            "last_reason": s.last_reason,
            "last_kick_s": s.last_kick_s,
            "last_check_s": s.last_check_s,
            "last_trip_s": s.last_trip_s,
            "last_clear_s": s.last_clear_s,
            "current_age_s": s.current_age_s,
            "timeout_s": self.config.timeout_s,
            "auto_clear_on_fresh_kick": self.config.auto_clear_on_fresh_kick,
            "trip_if_never_kicked": self.config.trip_if_never_kicked,
        }


# =============================================================================
# Functional helper (one-shot utility)
# =============================================================================
def evaluate_timeout_watchdog_once(
    *,
    prev_state: Optional[TimeoutWatchdogState],
    timeout_s: float,
    now_monotonic_s: float,
    kick: bool = False,
    auto_clear_on_fresh_kick: bool = True,
    trip_if_never_kicked: bool = False,
    clear_request: bool = False,
    operator_confirmed: bool = False,
) -> TimeoutWatchdogResult:
    """
    One-shot evaluation helper (useful for unit tests / stateless adapters).

    For production nodes, prefer the stateful `TimeoutWatchdog` class.
    """
    wd = TimeoutWatchdog(
        timeout_s=timeout_s,
        auto_clear_on_fresh_kick=auto_clear_on_fresh_kick,
        trip_if_never_kicked=trip_if_never_kicked,
        start_tripped=bool(prev_state.tripped) if prev_state else False,
    )

    # Restore selected state (internal utility restoration)
    if prev_state is not None:
        wd._state = _copy_state(prev_state)  # intentional for utility helper
        wd._state.timeout_s = wd.config.timeout_s

    result: TimeoutWatchdogResult
    if kick:
        result = wd.kick(now_monotonic_s=now_monotonic_s, source="oneshot")
    else:
        result = wd.check(now_monotonic_s=now_monotonic_s, source="oneshot")

    if clear_request:
        result = wd.clear(
            operator_confirmed=operator_confirmed,
            source="oneshot_clear",
            now_monotonic_s=now_monotonic_s,
        )

    return result


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "TimeoutWatchdogConfig",
    "TimeoutWatchdogState",
    "TimeoutWatchdogResult",
    "TimeoutWatchdog",
    "monotonic_time_s",
    "evaluate_timeout_watchdog_once",
]