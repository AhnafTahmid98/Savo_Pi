#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/stale_command_policy.py
-----------------------------------------------------
Professional stale-command safety policy for `savo_base` (ROS2 Jazzy / real robot).

Purpose
-------
Decide what the base should do when motion commands become stale (timeout watchdog trip),
while also supporting optional recovery behavior (hold zero, require fresh command burst,
manual clear, etc.).

Why this exists
---------------
`timeout_watchdog.py` detects timing/freshness.
This file defines the *control policy* that turns watchdog state into safe base behavior.

Example usage in Robot Savo
---------------------------
- `base_driver_node.py` receives Twist/WheelCommand
- `TimeoutWatchdog` trips when command stream stops
- `StaleCommandPolicy` decides:
    - force zero immediately
    - whether output remains blocked
    - what is required to recover (fresh kick, N fresh commands, operator clear)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import time


# =============================================================================
# Helpers
# =============================================================================
def monotonic_time_s() -> float:
    return time.monotonic()


# =============================================================================
# Enums
# =============================================================================
class RecoveryMode(str, Enum):
    """
    Recovery mode after stale trip.
    """
    AUTO_ON_FIRST_FRESH = "auto_on_first_fresh"
    AUTO_AFTER_N_FRESH = "auto_after_n_fresh"
    MANUAL_CLEAR_ONLY = "manual_clear_only"


class OutputAction(str, Enum):
    """
    Action returned to base control code.
    """
    PASS = "pass"               # command may pass through (subject to other guards)
    FORCE_ZERO = "force_zero"   # override to zero command
    HOLD_BLOCKED = "hold_blocked"  # keep blocking output until recovery condition met


# =============================================================================
# Dataclasses
# =============================================================================
@dataclass
class StaleCommandPolicyConfig:
    """
    Configuration for stale command handling.
    """
    # Safety response when watchdog is stale/tripped
    force_zero_on_stale: bool = True

    # Recovery behavior after stale event
    recovery_mode: RecoveryMode = RecoveryMode.AUTO_ON_FIRST_FRESH

    # Used when recovery_mode == AUTO_AFTER_N_FRESH
    fresh_events_required: int = 2

    # Optional minimum time to remain blocked after stale trip (debounce/cooldown)
    min_hold_after_trip_s: float = 0.0

    # If True, a manual clear may still require a fresh event before PASS resumes
    require_fresh_after_manual_clear: bool = True


@dataclass
class StaleCommandPolicyState:
    """
    Runtime state of the stale command policy.
    """
    blocked: bool = False                   # if True, motion output is blocked by policy
    stale_active: bool = False              # mirrors watchdog stale/tripped input
    manual_clear_required: bool = False     # true in MANUAL_CLEAR_ONLY mode after stale trip

    fresh_events_since_trip: int = 0
    pending_fresh_after_manual_clear: bool = False

    last_reason: str = "init"

    # timestamps (monotonic)
    last_update_s: Optional[float] = None
    last_trip_s: Optional[float] = None
    last_clear_s: Optional[float] = None
    last_fresh_event_s: Optional[float] = None


@dataclass
class StaleCommandPolicyResult:
    """
    Result of policy evaluation/update.
    """
    action: OutputAction
    blocked: bool
    changed: bool
    event: str               # "stale_trip", "recovered", "manual_clear", "no_change", ...
    reason: str
    state: StaleCommandPolicyState = field(default_factory=StaleCommandPolicyState)


# =============================================================================
# Core policy
# =============================================================================
class StaleCommandPolicy:
    """
    Professional stale-command output policy.

    Integration concept
    -------------------
    A base node typically calls:
      1) `on_watchdog_status(is_stale=watchdog.tripped)` in the loop
      2) `on_fresh_command_event()` in each new command callback
      3) `evaluate_output_action()` before writing motor output

    Or simpler:
      - call `step(...)` with both signals each cycle/callback
    """

    def __init__(
        self,
        config: Optional[StaleCommandPolicyConfig] = None,
        *,
        force_zero_on_stale: Optional[bool] = None,
        recovery_mode: Optional[RecoveryMode | str] = None,
        fresh_events_required: Optional[int] = None,
        min_hold_after_trip_s: Optional[float] = None,
        require_fresh_after_manual_clear: Optional[bool] = None,
        start_blocked: bool = False,
    ) -> None:
        cfg = config or StaleCommandPolicyConfig()

        if force_zero_on_stale is not None:
            cfg.force_zero_on_stale = bool(force_zero_on_stale)
        if recovery_mode is not None:
            cfg.recovery_mode = (
                recovery_mode if isinstance(recovery_mode, RecoveryMode)
                else RecoveryMode(str(recovery_mode))
            )
        if fresh_events_required is not None:
            cfg.fresh_events_required = int(fresh_events_required)
        if min_hold_after_trip_s is not None:
            cfg.min_hold_after_trip_s = float(min_hold_after_trip_s)
        if require_fresh_after_manual_clear is not None:
            cfg.require_fresh_after_manual_clear = bool(require_fresh_after_manual_clear)

        cfg.fresh_events_required = max(1, int(cfg.fresh_events_required))
        cfg.min_hold_after_trip_s = max(0.0, float(cfg.min_hold_after_trip_s))

        self.config = StaleCommandPolicyConfig(
            force_zero_on_stale=bool(cfg.force_zero_on_stale),
            recovery_mode=cfg.recovery_mode,
            fresh_events_required=cfg.fresh_events_required,
            min_hold_after_trip_s=cfg.min_hold_after_trip_s,
            require_fresh_after_manual_clear=bool(cfg.require_fresh_after_manual_clear),
        )

        self._state = StaleCommandPolicyState(
            blocked=bool(start_blocked),
            stale_active=False,
            manual_clear_required=bool(start_blocked and self.config.recovery_mode == RecoveryMode.MANUAL_CLEAR_ONLY),
            pending_fresh_after_manual_clear=False,
            last_reason="startup_blocked" if start_blocked else "startup_clear",
        )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    @property
    def state(self) -> StaleCommandPolicyState:
        s = self._state
        return StaleCommandPolicyState(
            blocked=s.blocked,
            stale_active=s.stale_active,
            manual_clear_required=s.manual_clear_required,
            fresh_events_since_trip=s.fresh_events_since_trip,
            pending_fresh_after_manual_clear=s.pending_fresh_after_manual_clear,
            last_reason=s.last_reason,
            last_update_s=s.last_update_s,
            last_trip_s=s.last_trip_s,
            last_clear_s=s.last_clear_s,
            last_fresh_event_s=s.last_fresh_event_s,
        )

    @property
    def blocked(self) -> bool:
        return self._state.blocked

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------
    def _now(self, now_monotonic_s: Optional[float]) -> float:
        return monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)

    def _hold_elapsed(self, now_s: float) -> bool:
        if self.config.min_hold_after_trip_s <= 0.0:
            return True
        if self._state.last_trip_s is None:
            return True
        return (now_s - self._state.last_trip_s) >= self.config.min_hold_after_trip_s

    def _current_blocking_action(self) -> OutputAction:
        if self._state.blocked:
            return OutputAction.FORCE_ZERO if self.config.force_zero_on_stale else OutputAction.HOLD_BLOCKED
        return OutputAction.PASS

    def _result(self, *, action: OutputAction, changed: bool, event: str, reason: str) -> StaleCommandPolicyResult:
        self._state.last_reason = reason
        return StaleCommandPolicyResult(
            action=action,
            blocked=self._state.blocked,
            changed=changed,
            event=event,
            reason=reason,
            state=self.state,
        )

    def _enter_stale_block(self, *, now_s: float, source: str) -> StaleCommandPolicyResult:
        prev_blocked = self._state.blocked
        self._state.stale_active = True
        self._state.blocked = True
        self._state.fresh_events_since_trip = 0
        self._state.last_trip_s = now_s

        if self.config.recovery_mode == RecoveryMode.MANUAL_CLEAR_ONLY:
            self._state.manual_clear_required = True
        else:
            self._state.manual_clear_required = False

        # If manual clear later is used, optionally require a fresh command after clear
        self._state.pending_fresh_after_manual_clear = False

        action = self._current_blocking_action()
        return self._result(
            action=action,
            changed=(not prev_blocked),
            event="stale_trip" if not prev_blocked else "stale_still_active",
            reason=f"stale_trip_by_{source}",
        )

    def _try_auto_recover(self, *, now_s: float, source: str) -> Optional[StaleCommandPolicyResult]:
        """
        Attempt auto recovery if policy and conditions permit.
        Returns None if no state change/action decision should be emitted here.
        """
        if self.config.recovery_mode == RecoveryMode.MANUAL_CLEAR_ONLY:
            return None

        if self._state.manual_clear_required:
            return None

        if not self._hold_elapsed(now_s):
            return None

        if self.config.recovery_mode == RecoveryMode.AUTO_ON_FIRST_FRESH:
            if self._state.fresh_events_since_trip >= 1:
                prev = self._state.blocked
                self._state.blocked = False
                return self._result(
                    action=OutputAction.PASS,
                    changed=prev,
                    event="recovered",
                    reason=f"auto_recovered_first_fresh_{source}",
                )

        elif self.config.recovery_mode == RecoveryMode.AUTO_AFTER_N_FRESH:
            if self._state.fresh_events_since_trip >= self.config.fresh_events_required:
                prev = self._state.blocked
                self._state.blocked = False
                return self._result(
                    action=OutputAction.PASS,
                    changed=prev,
                    event="recovered",
                    reason=f"auto_recovered_after_{self._state.fresh_events_since_trip}_fresh_{source}",
                )

        return None

    # -------------------------------------------------------------------------
    # Public API — state update signals
    # -------------------------------------------------------------------------
    def on_watchdog_status(
        self,
        *,
        is_stale: bool,
        source: str = "watchdog",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Feed current watchdog stale/tripped state.

        Call periodically (e.g., control loop).
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s

        if is_stale:
            return self._enter_stale_block(now_s=now_s, source=source)

        # watchdog reports fresh/non-stale
        self._state.stale_active = False

        # If blocked, maybe auto recover (depends on policy and fresh command events count)
        if self._state.blocked:
            recovered = self._try_auto_recover(now_s=now_s, source=source)
            if recovered is not None:
                return recovered

            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="no_change",
                reason=f"blocked_waiting_recovery_{source}",
            )

        return self._result(
            action=OutputAction.PASS,
            changed=False,
            event="no_change",
            reason=f"clear_nonstale_{source}",
        )

    def on_fresh_command_event(
        self,
        *,
        source: str = "command",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Notify policy that a fresh command event was received.

        This is separate from watchdog freshness to support policies like:
        'require N fresh commands after stale trip'.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s
        self._state.last_fresh_event_s = now_s

        if self._state.blocked:
            self._state.fresh_events_since_trip += 1

            # If manual clear happened and we require fresh after clear, this fresh event can satisfy it.
            if self._state.pending_fresh_after_manual_clear:
                self._state.pending_fresh_after_manual_clear = False
                prev = self._state.blocked
                self._state.blocked = False
                self._state.last_clear_s = now_s
                return self._result(
                    action=OutputAction.PASS,
                    changed=prev,
                    event="recovered",
                    reason=f"recovered_after_manual_clear_and_fresh_{source}",
                )

            # Try auto recovery only if watchdog is currently not stale
            if not self._state.stale_active:
                recovered = self._try_auto_recover(now_s=now_s, source=source)
                if recovered is not None:
                    return recovered

            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="fresh_seen_blocked",
                reason=f"fresh_event_count_{self._state.fresh_events_since_trip}_{source}",
            )

        # Not blocked → command may pass
        return self._result(
            action=OutputAction.PASS,
            changed=False,
            event="fresh_seen",
            reason=f"fresh_event_{source}",
        )

    def manual_clear(
        self,
        *,
        operator_confirmed: bool = False,
        source: str = "operator",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Manual clear path (used especially with MANUAL_CLEAR_ONLY mode).
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s

        if not self._state.blocked:
            return self._result(
                action=OutputAction.PASS,
                changed=False,
                event="clear_ignored",
                reason=f"manual_clear_ignored_not_blocked_{source}",
            )

        if not operator_confirmed:
            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="clear_rejected",
                reason=f"manual_clear_rejected_not_confirmed_{source}",
            )

        if self._state.stale_active:
            # Cannot clear while watchdog still says stale. This avoids clearing into a dead command stream.
            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="clear_rejected",
                reason=f"manual_clear_rejected_stale_active_{source}",
            )

        # In manual mode, clear manual requirement and unblock (or wait for fresh if configured)
        self._state.manual_clear_required = False
        self._state.last_clear_s = now_s

        if self.config.require_fresh_after_manual_clear:
            self._state.pending_fresh_after_manual_clear = True
            # Keep blocked until next fresh command event arrives
            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="manual_clear",
                reason=f"manual_clear_accepted_waiting_fresh_{source}",
            )

        prev = self._state.blocked
        self._state.blocked = False
        self._state.pending_fresh_after_manual_clear = False
        return self._result(
            action=OutputAction.PASS,
            changed=prev,
            event="manual_clear",
            reason=f"manual_clear_accepted_{source}",
        )

    # -------------------------------------------------------------------------
    # Combined convenience step API
    # -------------------------------------------------------------------------
    def step(
        self,
        *,
        watchdog_is_stale: bool,
        fresh_command_event: bool = False,
        source: str = "step",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Convenience combined update.

        Order:
        1) watchdog status
        2) optional fresh command event

        Returns the final result after both updates.
        """
        now_s = self._now(now_monotonic_s)
        r = self.on_watchdog_status(is_stale=watchdog_is_stale, source=f"{source}_watchdog", now_monotonic_s=now_s)
        if fresh_command_event:
            r = self.on_fresh_command_event(source=f"{source}_fresh", now_monotonic_s=now_s)
        return r

    # -------------------------------------------------------------------------
    # Output decision helpers
    # -------------------------------------------------------------------------
    def evaluate_output_action(
        self,
        *,
        now_monotonic_s: Optional[float] = None,
        source: str = "evaluate",
    ) -> StaleCommandPolicyResult:
        """
        Return current output action without changing core policy state,
        except for timestamp/reason bookkeeping.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s

        if self._state.blocked:
            return self._result(
                action=self._current_blocking_action(),
                changed=False,
                event="no_change",
                reason=f"blocked_{source}",
            )

        return self._result(
            action=OutputAction.PASS,
            changed=False,
            event="no_change",
            reason=f"pass_{source}",
        )

    def force_block(
        self,
        *,
        source: str = "software_fault",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Force policy block (independent of watchdog), e.g., supervisor fault path.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s
        prev = self._state.blocked
        self._state.blocked = True
        self._state.last_trip_s = now_s
        self._state.fresh_events_since_trip = 0
        self._state.manual_clear_required = (self.config.recovery_mode == RecoveryMode.MANUAL_CLEAR_ONLY)
        return self._result(
            action=self._current_blocking_action(),
            changed=(not prev),
            event="forced_block",
            reason=f"forced_block_{source}",
        )

    def reset(
        self,
        *,
        clear_block: bool = True,
        clear_counters: bool = True,
        source: str = "reset",
        now_monotonic_s: Optional[float] = None,
    ) -> StaleCommandPolicyResult:
        """
        Administrative reset for bringup/testing.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s
        changed = False

        if clear_block and self._state.blocked:
            self._state.blocked = False
            changed = True

        self._state.stale_active = False
        self._state.manual_clear_required = False
        self._state.pending_fresh_after_manual_clear = False

        if clear_counters:
            self._state.fresh_events_since_trip = 0

        self._state.last_clear_s = now_s
        return self._result(
            action=OutputAction.PASS if not self._state.blocked else self._current_blocking_action(),
            changed=changed,
            event="reset",
            reason=f"reset_{source}",
        )

    def as_dict(self) -> dict:
        """
        JSON/state-topic friendly snapshot.
        """
        s = self._state
        return {
            "blocked": s.blocked,
            "stale_active": s.stale_active,
            "manual_clear_required": s.manual_clear_required,
            "fresh_events_since_trip": s.fresh_events_since_trip,
            "pending_fresh_after_manual_clear": s.pending_fresh_after_manual_clear,
            "last_reason": s.last_reason,
            "last_update_s": s.last_update_s,
            "last_trip_s": s.last_trip_s,
            "last_clear_s": s.last_clear_s,
            "last_fresh_event_s": s.last_fresh_event_s,
            "config": {
                "force_zero_on_stale": self.config.force_zero_on_stale,
                "recovery_mode": self.config.recovery_mode.value,
                "fresh_events_required": self.config.fresh_events_required,
                "min_hold_after_trip_s": self.config.min_hold_after_trip_s,
                "require_fresh_after_manual_clear": self.config.require_fresh_after_manual_clear,
            },
        }


# =============================================================================
# Functional one-shot helper
# =============================================================================
def evaluate_stale_command_policy_once(
    *,
    prev_state: Optional[StaleCommandPolicyState],
    watchdog_is_stale: bool,
    fresh_command_event: bool = False,
    config: Optional[StaleCommandPolicyConfig] = None,
    now_monotonic_s: Optional[float] = None,
) -> StaleCommandPolicyResult:
    """
    Stateless-style one-shot evaluation helper (tests/adapters).

    For production nodes, prefer the stateful `StaleCommandPolicy`.
    """
    pol = StaleCommandPolicy(config=config)

    if prev_state is not None:
        # Intentional internal restore for utility use
        pol._state = StaleCommandPolicyState(
            blocked=bool(prev_state.blocked),
            stale_active=bool(prev_state.stale_active),
            manual_clear_required=bool(prev_state.manual_clear_required),
            fresh_events_since_trip=int(prev_state.fresh_events_since_trip),
            pending_fresh_after_manual_clear=bool(prev_state.pending_fresh_after_manual_clear),
            last_reason=str(prev_state.last_reason),
            last_update_s=prev_state.last_update_s,
            last_trip_s=prev_state.last_trip_s,
            last_clear_s=prev_state.last_clear_s,
            last_fresh_event_s=prev_state.last_fresh_event_s,
        )

    return pol.step(
        watchdog_is_stale=watchdog_is_stale,
        fresh_command_event=fresh_command_event,
        source="oneshot",
        now_monotonic_s=now_monotonic_s,
    )


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "RecoveryMode",
    "OutputAction",
    "StaleCommandPolicyConfig",
    "StaleCommandPolicyState",
    "StaleCommandPolicyResult",
    "StaleCommandPolicy",
    "monotonic_time_s",
    "evaluate_stale_command_policy_once",
]