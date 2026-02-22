#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/estop_latch.py
--------------------------------------------
Professional latching Emergency Stop (E-Stop) helper for `savo_base`.

Purpose
-------
Provide a deterministic, reusable latching E-Stop state machine for real robot
testing and production bringup.

Why a latch?
------------
A transient "False" message or operator mistake should NOT automatically clear
an emergency stop. The latch ensures the robot remains stopped until an explicit,
intentional reset/clear action is performed.

Design principles
-----------------
- Pure Python, no ROS dependency (easy to unit test)
- Explicit state transitions
- Operator-intent clear/reset required
- Rich status for logging/state topics
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import time


# =============================================================================
# Data models
# =============================================================================
@dataclass
class EStopLatchState:
    """
    Runtime state of the E-Stop latch.
    """
    latched: bool = False
    input_active: bool = False          # latest raw estop input signal
    reset_required: bool = False        # true once a latch event has happened
    last_reason: str = "init"

    # timestamps (monotonic)
    last_input_change_s: Optional[float] = None
    last_latch_s: Optional[float] = None
    last_clear_s: Optional[float] = None
    last_update_s: Optional[float] = None


@dataclass
class EStopLatchResult:
    """
    Result of an update() call.
    """
    latched: bool
    changed: bool
    event: str                      # e.g. "latched", "cleared", "no_change"
    reason: str                     # human-readable reason
    state: EStopLatchState = field(default_factory=EStopLatchState)


# =============================================================================
# Helper
# =============================================================================
def monotonic_time_s() -> float:
    return time.monotonic()


# =============================================================================
# Core latch implementation
# =============================================================================
class EStopLatch:
    """
    Latching E-Stop state machine.

    Behavior summary
    ----------------
    - If estop input becomes active -> latch immediately.
    - Once latched, it stays latched until explicit `clear_latch()` (or reset path).
    - Clearing can be configured to require the raw estop input to be inactive.

    This class does NOT decide motor behavior directly; nodes should use:
      result.latched == True  -> force stop
    """

    def __init__(
        self,
        *,
        start_latched: bool = False,
        require_input_inactive_to_clear: bool = True,
        record_timestamps: bool = True,
    ) -> None:
        self._state = EStopLatchState(
            latched=bool(start_latched),
            input_active=False,
            reset_required=bool(start_latched),
            last_reason="startup_latched" if start_latched else "startup_clear",
        )
        self._require_input_inactive_to_clear = bool(require_input_inactive_to_clear)
        self._record_timestamps = bool(record_timestamps)

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------
    @property
    def state(self) -> EStopLatchState:
        """
        Return a defensive copy of internal state.
        """
        s = self._state
        return EStopLatchState(
            latched=s.latched,
            input_active=s.input_active,
            reset_required=s.reset_required,
            last_reason=s.last_reason,
            last_input_change_s=s.last_input_change_s,
            last_latch_s=s.last_latch_s,
            last_clear_s=s.last_clear_s,
            last_update_s=s.last_update_s,
        )

    @property
    def latched(self) -> bool:
        return self._state.latched

    @property
    def input_active(self) -> bool:
        return self._state.input_active

    @property
    def reset_required(self) -> bool:
        return self._state.reset_required

    # -------------------------------------------------------------------------
    # Internal timestamp helper
    # -------------------------------------------------------------------------
    def _now(self, now_monotonic_s: Optional[float]) -> float:
        if now_monotonic_s is None:
            return monotonic_time_s()
        return float(now_monotonic_s)

    def _stamp(self, attr_name: str, now_s: float) -> None:
        if self._record_timestamps:
            setattr(self._state, attr_name, now_s)

    # -------------------------------------------------------------------------
    # Core input update path
    # -------------------------------------------------------------------------
    def update(
        self,
        *,
        estop_input_active: bool,
        source: str = "input",
        now_monotonic_s: Optional[float] = None,
    ) -> EStopLatchResult:
        """
        Update latch with the latest raw E-Stop input state.

        Parameters
        ----------
        estop_input_active : bool
            Raw E-Stop signal (True means E-Stop is pressed/active).
        source : str
            Optional source label for diagnostics/logging.
        now_monotonic_s : float | None
            Optional injected monotonic timestamp (useful for tests).

        Returns
        -------
        EStopLatchResult
            Current latch output and transition metadata.
        """
        now_s = self._now(now_monotonic_s)
        prev_input = self._state.input_active
        prev_latched = self._state.latched

        self._state.last_update_s = now_s
        self._state.input_active = bool(estop_input_active)

        if self._state.input_active != prev_input:
            self._stamp("last_input_change_s", now_s)

        # If active input -> force latch
        if self._state.input_active:
            if not self._state.latched:
                self._state.latched = True
                self._state.reset_required = True
                self._state.last_reason = f"latched_by_{source}"
                self._stamp("last_latch_s", now_s)
                return EStopLatchResult(
                    latched=True,
                    changed=True,
                    event="latched",
                    reason=self._state.last_reason,
                    state=self.state,
                )

            # Already latched; remain latched
            self._state.last_reason = f"held_latched_by_{source}"
            return EStopLatchResult(
                latched=True,
                changed=False,
                event="no_change",
                reason=self._state.last_reason,
                state=self.state,
            )

        # Input inactive:
        # DO NOT auto-clear. Latch remains until explicit clear.
        if prev_latched:
            self._state.last_reason = f"input_inactive_waiting_clear_{source}"
            return EStopLatchResult(
                latched=True,
                changed=False,
                event="no_change",
                reason=self._state.last_reason,
                state=self.state,
            )

        # Not latched + inactive input = normal clear state
        self._state.last_reason = f"clear_input_inactive_{source}"
        return EStopLatchResult(
            latched=False,
            changed=False,
            event="no_change",
            reason=self._state.last_reason,
            state=self.state,
        )

    # -------------------------------------------------------------------------
    # Explicit clear/reset path
    # -------------------------------------------------------------------------
    def clear_latch(
        self,
        *,
        operator_confirmed: bool = False,
        source: str = "operator",
        now_monotonic_s: Optional[float] = None,
    ) -> EStopLatchResult:
        """
        Attempt to clear the E-Stop latch.

        Safety behavior
        ---------------
        - Requires `operator_confirmed=True`
        - Optionally requires raw estop input to be inactive

        Returns an EStopLatchResult describing success/failure.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s

        # Already clear
        if not self._state.latched:
            self._state.last_reason = f"clear_request_ignored_not_latched_{source}"
            return EStopLatchResult(
                latched=False,
                changed=False,
                event="clear_ignored",
                reason=self._state.last_reason,
                state=self.state,
            )

        # Require explicit operator confirmation
        if not operator_confirmed:
            self._state.last_reason = f"clear_rejected_operator_not_confirmed_{source}"
            return EStopLatchResult(
                latched=True,
                changed=False,
                event="clear_rejected",
                reason=self._state.last_reason,
                state=self.state,
            )

        # Optional rule: cannot clear while estop input still active
        if self._require_input_inactive_to_clear and self._state.input_active:
            self._state.last_reason = f"clear_rejected_input_still_active_{source}"
            return EStopLatchResult(
                latched=True,
                changed=False,
                event="clear_rejected",
                reason=self._state.last_reason,
                state=self.state,
            )

        # Clear latch
        self._state.latched = False
        self._state.reset_required = False
        self._state.last_reason = f"cleared_by_{source}"
        self._stamp("last_clear_s", now_s)

        return EStopLatchResult(
            latched=False,
            changed=True,
            event="cleared",
            reason=self._state.last_reason,
            state=self.state,
        )

    # -------------------------------------------------------------------------
    # Administrative helpers
    # -------------------------------------------------------------------------
    def force_latch(
        self,
        *,
        source: str = "software",
        now_monotonic_s: Optional[float] = None,
    ) -> EStopLatchResult:
        """
        Force the latch active regardless of input (software estop / fault path).
        """
        now_s = self._now(now_monotonic_s)
        prev_latched = self._state.latched
        self._state.last_update_s = now_s

        self._state.latched = True
        self._state.reset_required = True
        self._state.last_reason = f"latched_by_{source}"
        if not prev_latched:
            self._stamp("last_latch_s", now_s)

        return EStopLatchResult(
            latched=True,
            changed=(not prev_latched),
            event="latched" if not prev_latched else "no_change",
            reason=self._state.last_reason,
            state=self.state,
        )

    def hard_reset(
        self,
        *,
        clear_input_state: bool = False,
        source: str = "hard_reset",
        now_monotonic_s: Optional[float] = None,
    ) -> EStopLatchResult:
        """
        Administrative reset (for diagnostics/testing/startup recovery).
        Use carefully. In production, prefer clear_latch().

        Parameters
        ----------
        clear_input_state : bool
            If True, also resets remembered raw input state to False.
        """
        now_s = self._now(now_monotonic_s)
        self._state.last_update_s = now_s

        self._state.latched = False
        self._state.reset_required = False
        if clear_input_state:
            self._state.input_active = False

        self._state.last_reason = f"hard_reset_by_{source}"
        self._stamp("last_clear_s", now_s)

        return EStopLatchResult(
            latched=False,
            changed=True,
            event="hard_reset",
            reason=self._state.last_reason,
            state=self.state,
        )

    def as_dict(self) -> dict:
        """
        JSON/state-topic friendly snapshot.
        """
        s = self._state
        return {
            "latched": s.latched,
            "input_active": s.input_active,
            "reset_required": s.reset_required,
            "last_reason": s.last_reason,
            "last_input_change_s": s.last_input_change_s,
            "last_latch_s": s.last_latch_s,
            "last_clear_s": s.last_clear_s,
            "last_update_s": s.last_update_s,
        }


# =============================================================================
# Convenience functional helper
# =============================================================================
def evaluate_estop_latch_once(
    *,
    prev_state: Optional[EStopLatchState],
    estop_input_active: bool,
    clear_request: bool = False,
    operator_confirmed: bool = False,
    require_input_inactive_to_clear: bool = True,
    now_monotonic_s: Optional[float] = None,
) -> EStopLatchResult:
    """
    One-shot helper (stateless wrapper style) for tests and quick integrations.

    For production nodes, prefer the stateful EStopLatch class.
    """
    latch = EStopLatch(
        start_latched=bool(prev_state.latched) if prev_state else False,
        require_input_inactive_to_clear=require_input_inactive_to_clear,
    )

    # Restore selected state fields if provided
    if prev_state is not None:
        latch._state = EStopLatchState(  # intentional internal restore for utility helper
            latched=bool(prev_state.latched),
            input_active=bool(prev_state.input_active),
            reset_required=bool(prev_state.reset_required),
            last_reason=str(prev_state.last_reason),
            last_input_change_s=prev_state.last_input_change_s,
            last_latch_s=prev_state.last_latch_s,
            last_clear_s=prev_state.last_clear_s,
            last_update_s=prev_state.last_update_s,
        )

    result = latch.update(
        estop_input_active=estop_input_active,
        source="oneshot_input",
        now_monotonic_s=now_monotonic_s,
    )

    if clear_request:
        result = latch.clear_latch(
            operator_confirmed=operator_confirmed,
            source="oneshot_clear",
            now_monotonic_s=now_monotonic_s,
        )

    return result


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "EStopLatchState",
    "EStopLatchResult",
    "EStopLatch",
    "monotonic_time_s",
    "evaluate_estop_latch_once",
]