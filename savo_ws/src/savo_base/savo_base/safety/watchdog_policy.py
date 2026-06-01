#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/watchdog_policy.py
------------------------------------------------
Professional combined watchdog/safety policy for `savo_base` (ROS2 Jazzy / real robot).

Purpose
-------
Combine:
- timeout watchdog (command freshness timing)
- stale-command recovery policy (force-zero / unblock logic)
- optional e-stop latch state (external input / software latch)

into one deterministic decision point for base motion output.

Typical integration in Robot Savo base_driver_node
--------------------------------------------------
1) On every fresh /cmd_vel_safe (or WheelCommand) callback:
      policy.on_fresh_command_event(source="cmd_vel_safe")

2) In control loop (e.g., 30–50 Hz):
      policy.update_watchdog_and_policy()
      decision = policy.evaluate_motion_permission()

3) If decision.force_zero is True:
      send zero command to motor board
   else:
      pass shaped/scaled command to motor board

Notes
-----
- This file is intentionally pure Python (no ROS dependency).
- It can be used directly in ROS nodes or in non-ROS diagnostics.
- It supports optional e-stop input by boolean signal (recommended).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Any
import time

from .timeout_watchdog import (
    TimeoutWatchdog,
    TimeoutWatchdogConfig,
    TimeoutWatchdogResult,
)
from .stale_command_policy import (
    StaleCommandPolicy,
    StaleCommandPolicyConfig,
    StaleCommandPolicyResult,
    OutputAction,
)


# =============================================================================
# Helpers
# =============================================================================
def monotonic_time_s() -> float:
    return time.monotonic()


# =============================================================================
# Enums
# =============================================================================
class MotionPermission(str, Enum):
    """
    Final motion permission state returned to the base output path.
    """
    PASS = "pass"
    FORCE_ZERO = "force_zero"
    BLOCKED_ESTOP = "blocked_estop"
    BLOCKED_TIMEOUT = "blocked_timeout"
    BLOCKED_POLICY = "blocked_policy"
    BLOCKED_DISABLED = "blocked_disabled"


# =============================================================================
# Dataclasses
# =============================================================================
@dataclass
class WatchdogPolicyConfig:
    """
    Combined safety policy configuration.
    """
    enabled: bool = True

    # Timeout watchdog config
    timeout_watchdog: TimeoutWatchdogConfig = field(default_factory=TimeoutWatchdogConfig)

    # Stale command recovery/output policy config
    stale_policy: StaleCommandPolicyConfig = field(default_factory=StaleCommandPolicyConfig)

    # If True, e-stop active always wins and forces zero.
    estop_has_priority: bool = True

    # If True, a disabled policy still reports PASS (useful in diagnostics mode).
    # If False, disabled policy blocks motion (safer default for production).
    pass_when_disabled: bool = False


@dataclass
class WatchdogPolicyState:
    """
    Combined runtime state snapshot.
    """
    enabled: bool = True

    # External safety inputs
    estop_active: bool = False
    estop_latched: bool = False

    # Combined decision view
    permission: MotionPermission = MotionPermission.FORCE_ZERO
    force_zero: bool = True
    blocked: bool = True

    # Diagnostics / reasoning
    last_reason: str = "init"
    last_update_s: Optional[float] = None
    last_fresh_command_s: Optional[float] = None
    last_manual_clear_s: Optional[float] = None

    # Child component snapshots
    watchdog_tripped: bool = False
    stale_policy_blocked: bool = False
    stale_policy_action: str = OutputAction.FORCE_ZERO.value


@dataclass
class WatchdogPolicyDecision:
    """
    Final decision for motor output path.
    """
    permission: MotionPermission
    force_zero: bool
    blocked: bool
    changed: bool
    reason: str
    state: WatchdogPolicyState = field(default_factory=WatchdogPolicyState)

    @property
    def can_pass(self) -> bool:
        return self.permission == MotionPermission.PASS and not self.force_zero and not self.blocked


@dataclass
class WatchdogPolicyStepResult:
    """
    Composite step result including child updates and final decision.
    """
    watchdog_result: Optional[TimeoutWatchdogResult]
    stale_policy_result: Optional[StaleCommandPolicyResult]
    decision: WatchdogPolicyDecision


# =============================================================================
# Core combined policy
# =============================================================================
class WatchdogPolicy:
    """
    Combined safety policy for Robot Savo base motion output.

    Components
    ----------
    - TimeoutWatchdog: detects stale command stream
    - StaleCommandPolicy: decides recovery behavior and force-zero/blocking
    - External e-stop booleans: immediate safety override

    This class does not require a specific `estop_latch.py` implementation.
    Feed boolean e-stop state from your node (`/safety/stop`, GPIO e-stop, etc.).
    """

    def __init__(
        self,
        config: Optional[WatchdogPolicyConfig] = None,
        *,
        enabled: Optional[bool] = None,
    ) -> None:
        cfg = config or WatchdogPolicyConfig()
        if enabled is not None:
            cfg.enabled = bool(enabled)

        self.config = WatchdogPolicyConfig(
            enabled=bool(cfg.enabled),
            timeout_watchdog=cfg.timeout_watchdog,
            stale_policy=cfg.stale_policy,
            estop_has_priority=bool(cfg.estop_has_priority),
            pass_when_disabled=bool(cfg.pass_when_disabled),
        )

        self.timeout_watchdog = TimeoutWatchdog(config=self.config.timeout_watchdog)
        self.stale_policy = StaleCommandPolicy(config=self.config.stale_policy)

        self._state = WatchdogPolicyState(
            enabled=self.config.enabled,
            permission=MotionPermission.FORCE_ZERO if not self.config.pass_when_disabled else MotionPermission.PASS,
            force_zero=(not self.config.pass_when_disabled),
            blocked=(not self.config.pass_when_disabled),
            last_reason="startup",
            watchdog_tripped=self.timeout_watchdog.tripped,
            stale_policy_blocked=self.stale_policy.blocked,
            stale_policy_action=OutputAction.FORCE_ZERO.value,
        )

        self._last_decision_signature: tuple[Any, ...] = ()

    # -------------------------------------------------------------------------
    # Properties / snapshots
    # -------------------------------------------------------------------------
    @property
    def state(self) -> WatchdogPolicyState:
        s = self._state
        return WatchdogPolicyState(
            enabled=s.enabled,
            estop_active=s.estop_active,
            estop_latched=s.estop_latched,
            permission=s.permission,
            force_zero=s.force_zero,
            blocked=s.blocked,
            last_reason=s.last_reason,
            last_update_s=s.last_update_s,
            last_fresh_command_s=s.last_fresh_command_s,
            last_manual_clear_s=s.last_manual_clear_s,
            watchdog_tripped=s.watchdog_tripped,
            stale_policy_blocked=s.stale_policy_blocked,
            stale_policy_action=s.stale_policy_action,
        )

    def as_dict(self) -> dict:
        """
        JSON/state-topic friendly snapshot.
        """
        s = self._state
        return {
            "enabled": s.enabled,
            "estop_active": s.estop_active,
            "estop_latched": s.estop_latched,
            "permission": s.permission.value,
            "force_zero": s.force_zero,
            "blocked": s.blocked,
            "last_reason": s.last_reason,
            "last_update_s": s.last_update_s,
            "last_fresh_command_s": s.last_fresh_command_s,
            "last_manual_clear_s": s.last_manual_clear_s,
            "watchdog_tripped": s.watchdog_tripped,
            "stale_policy_blocked": s.stale_policy_blocked,
            "stale_policy_action": s.stale_policy_action,
            "timeout_watchdog": self.timeout_watchdog.as_dict(),
            "stale_command_policy": self.stale_policy.as_dict(),
            "config": {
                "estop_has_priority": self.config.estop_has_priority,
                "pass_when_disabled": self.config.pass_when_disabled,
            },
        }

    # -------------------------------------------------------------------------
    # Low-level state feeds
    # -------------------------------------------------------------------------
    def set_enabled(self, enabled: bool, *, source: str = "api", now_monotonic_s: Optional[float] = None) -> None:
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        self.config.enabled = bool(enabled)
        self._state.enabled = bool(enabled)
        self._state.last_update_s = now_s
        self._state.last_reason = f"enabled_set_{enabled}_{source}"

    def set_estop_state(
        self,
        *,
        estop_active: Optional[bool] = None,
        estop_latched: Optional[bool] = None,
        source: str = "estop_input",
        now_monotonic_s: Optional[float] = None,
    ) -> None:
        """
        Feed e-stop state (recommended from base node).

        Parameters
        ----------
        estop_active
            True if e-stop condition currently active (hardware button, /safety/stop, etc.)
        estop_latched
            True if your estop latch remains latched even if physical input released
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        if estop_active is not None:
            self._state.estop_active = bool(estop_active)
        if estop_latched is not None:
            self._state.estop_latched = bool(estop_latched)
        self._state.last_update_s = now_s
        self._state.last_reason = f"estop_state_updated_{source}"

    def ingest_estop_object(
        self,
        estop_obj: Any,
        *,
        source: str = "estop_object",
        now_monotonic_s: Optional[float] = None,
    ) -> None:
        """
        Optional duck-typed adapter for an `EstopLatch`-like object.

        Supported attributes/properties (any subset):
        - `.active`
        - `.latched`
        - `.is_active`
        - `.is_latched`
        - `.state.active`
        - `.state.latched`
        """
        active = None
        latched = None

        for name in ("active", "is_active"):
            if hasattr(estop_obj, name):
                try:
                    active = bool(getattr(estop_obj, name))
                    break
                except Exception:
                    pass

        for name in ("latched", "is_latched"):
            if hasattr(estop_obj, name):
                try:
                    latched = bool(getattr(estop_obj, name))
                    break
                except Exception:
                    pass

        if hasattr(estop_obj, "state"):
            st = getattr(estop_obj, "state")
            if active is None and hasattr(st, "active"):
                try:
                    active = bool(getattr(st, "active"))
                except Exception:
                    pass
            if latched is None and hasattr(st, "latched"):
                try:
                    latched = bool(getattr(st, "latched"))
                except Exception:
                    pass

        self.set_estop_state(
            estop_active=active,
            estop_latched=latched,
            source=source,
            now_monotonic_s=now_monotonic_s,
        )

    # -------------------------------------------------------------------------
    # Command freshness integration
    # -------------------------------------------------------------------------
    def on_fresh_command_event(
        self,
        *,
        source: str = "command",
        now_monotonic_s: Optional[float] = None,
        event_stamp_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyStepResult:
        """
        Feed a fresh motion command event into both timeout watchdog and stale policy.
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        self._state.last_update_s = now_s
        self._state.last_fresh_command_s = now_s

        wd_r = self.timeout_watchdog.kick(
            source=source,
            now_monotonic_s=now_s,
            event_stamp_monotonic_s=event_stamp_monotonic_s,
        )

        sp_r = self.stale_policy.on_fresh_command_event(
            source=source,
            now_monotonic_s=now_s,
        )

        self._refresh_child_snapshots()
        decision = self.evaluate_motion_permission(
            source=f"{source}_fresh_event",
            now_monotonic_s=now_s,
        )
        return WatchdogPolicyStepResult(
            watchdog_result=wd_r,
            stale_policy_result=sp_r,
            decision=decision,
        )

    def update_watchdog_and_policy(
        self,
        *,
        source: str = "control_loop",
        now_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyStepResult:
        """
        Periodic update: check timeout watchdog, feed stale status into stale policy,
        then compute final motion permission decision.

        Call this once per base loop tick.
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        self._state.last_update_s = now_s

        wd_r = self.timeout_watchdog.check(source=source, now_monotonic_s=now_s)

        sp_r = self.stale_policy.on_watchdog_status(
            is_stale=wd_r.tripped,
            source=source,
            now_monotonic_s=now_s,
        )

        self._refresh_child_snapshots()
        decision = self.evaluate_motion_permission(source=source, now_monotonic_s=now_s)

        return WatchdogPolicyStepResult(
            watchdog_result=wd_r,
            stale_policy_result=sp_r,
            decision=decision,
        )

    # -------------------------------------------------------------------------
    # Manual recovery / supervisory control
    # -------------------------------------------------------------------------
    def manual_clear_stale_policy(
        self,
        *,
        operator_confirmed: bool = False,
        source: str = "operator",
        now_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyStepResult:
        """
        Manual clear path for stale command policy.
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        self._state.last_update_s = now_s
        self._state.last_manual_clear_s = now_s

        sp_r = self.stale_policy.manual_clear(
            operator_confirmed=operator_confirmed,
            source=source,
            now_monotonic_s=now_s,
        )
        self._refresh_child_snapshots()

        decision = self.evaluate_motion_permission(source=f"{source}_manual_clear", now_monotonic_s=now_s)
        return WatchdogPolicyStepResult(
            watchdog_result=None,
            stale_policy_result=sp_r,
            decision=decision,
        )

    def force_timeout_trip(
        self,
        *,
        source: str = "software_fault",
        now_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyStepResult:
        """
        Force timeout watchdog trip (supervisory fault injection / test path).
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        wd_r = self.timeout_watchdog.force_trip(source=source, now_monotonic_s=now_s)
        sp_r = self.stale_policy.on_watchdog_status(is_stale=wd_r.tripped, source=source, now_monotonic_s=now_s)
        self._refresh_child_snapshots()
        decision = self.evaluate_motion_permission(source=f"{source}_forced_trip", now_monotonic_s=now_s)
        return WatchdogPolicyStepResult(
            watchdog_result=wd_r,
            stale_policy_result=sp_r,
            decision=decision,
        )

    def reset(
        self,
        *,
        clear_estop_inputs: bool = False,
        source: str = "reset",
        now_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyStepResult:
        """
        Administrative reset (bringup/testing).

        Notes
        -----
        - By default this does NOT clear external e-stop inputs.
        - If e-stop remains active/latched, final decision will still block.
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)

        wd_r = self.timeout_watchdog.reset(
            clear_trip=True,
            clear_kick_history=False,
            source=source,
            now_monotonic_s=now_s,
        )
        sp_r = self.stale_policy.reset(
            clear_block=True,
            clear_counters=True,
            source=source,
            now_monotonic_s=now_s,
        )

        if clear_estop_inputs:
            self._state.estop_active = False
            self._state.estop_latched = False

        self._state.last_update_s = now_s
        self._state.last_reason = f"reset_{source}"

        self._refresh_child_snapshots()
        decision = self.evaluate_motion_permission(source=f"{source}_reset", now_monotonic_s=now_s)
        return WatchdogPolicyStepResult(
            watchdog_result=wd_r,
            stale_policy_result=sp_r,
            decision=decision,
        )

    # -------------------------------------------------------------------------
    # Final decision logic
    # -------------------------------------------------------------------------
    def evaluate_motion_permission(
        self,
        *,
        source: str = "evaluate",
        now_monotonic_s: Optional[float] = None,
    ) -> WatchdogPolicyDecision:
        """
        Compute the final motion permission for the base output path.

        Priority (safe default)
        -----------------------
        1) disabled policy handling
        2) e-stop (active/latch)
        3) timeout watchdog / stale policy block
        4) pass
        """
        now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)
        self._state.last_update_s = now_s
        self._refresh_child_snapshots()

        # Disabled policy behavior
        if not self.config.enabled:
            if self.config.pass_when_disabled:
                return self._commit_decision(
                    permission=MotionPermission.PASS,
                    force_zero=False,
                    blocked=False,
                    reason=f"policy_disabled_pass_{source}",
                )
            return self._commit_decision(
                permission=MotionPermission.BLOCKED_DISABLED,
                force_zero=True,
                blocked=True,
                reason=f"policy_disabled_block_{source}",
            )

        # E-stop priority (recommended for real robot)
        estop_asserted = self._state.estop_active or self._state.estop_latched
        if self.config.estop_has_priority and estop_asserted:
            return self._commit_decision(
                permission=MotionPermission.BLOCKED_ESTOP,
                force_zero=True,
                blocked=True,
                reason=f"estop_asserted_{source}",
            )

        # Timeout watchdog / stale policy path
        if self.timeout_watchdog.tripped:
            # If stale policy says hold_blocked but not force_zero, we still block;
            # for motor output path on Robot Savo, force_zero is usually safer.
            sp_eval = self.stale_policy.evaluate_output_action(source=source, now_monotonic_s=now_s)
            if sp_eval.action == OutputAction.PASS:
                # Defensive: timeout tripped should not pass motion.
                return self._commit_decision(
                    permission=MotionPermission.BLOCKED_TIMEOUT,
                    force_zero=True,
                    blocked=True,
                    reason=f"timeout_tripped_defensive_zero_{source}",
                )
            return self._commit_decision(
                permission=MotionPermission.BLOCKED_TIMEOUT,
                force_zero=(sp_eval.action == OutputAction.FORCE_ZERO),
                blocked=True,
                reason=f"timeout_tripped_{source}",
            )

        # Watchdog clear, but stale policy may still be blocked during recovery
        sp_eval = self.stale_policy.evaluate_output_action(source=source, now_monotonic_s=now_s)
        if sp_eval.action != OutputAction.PASS or self.stale_policy.blocked:
            return self._commit_decision(
                permission=MotionPermission.BLOCKED_POLICY,
                force_zero=(sp_eval.action == OutputAction.FORCE_ZERO),
                blocked=True,
                reason=f"stale_policy_blocked_{source}",
            )

        # If e-stop is not priority but still asserted, enforce block here too
        if (not self.config.estop_has_priority) and estop_asserted:
            return self._commit_decision(
                permission=MotionPermission.BLOCKED_ESTOP,
                force_zero=True,
                blocked=True,
                reason=f"estop_asserted_nonpriority_{source}",
            )

        return self._commit_decision(
            permission=MotionPermission.PASS,
            force_zero=False,
            blocked=False,
            reason=f"motion_allowed_{source}",
        )

    # -------------------------------------------------------------------------
    # Internal decision bookkeeping
    # -------------------------------------------------------------------------
    def _refresh_child_snapshots(self) -> None:
        self._state.watchdog_tripped = bool(self.timeout_watchdog.tripped)
        self._state.stale_policy_blocked = bool(self.stale_policy.blocked)
        try:
            sp = self.stale_policy.evaluate_output_action(source="snapshot")
            self._state.stale_policy_action = sp.action.value
        except Exception:
            self._state.stale_policy_action = OutputAction.FORCE_ZERO.value if self.stale_policy.blocked else OutputAction.PASS.value

    def _commit_decision(
        self,
        *,
        permission: MotionPermission,
        force_zero: bool,
        blocked: bool,
        reason: str,
    ) -> WatchdogPolicyDecision:
        signature = (permission.value, bool(force_zero), bool(blocked), str(reason))
        changed = signature != self._last_decision_signature
        self._last_decision_signature = signature

        self._state.permission = permission
        self._state.force_zero = bool(force_zero)
        self._state.blocked = bool(blocked)
        self._state.last_reason = reason

        return WatchdogPolicyDecision(
            permission=permission,
            force_zero=bool(force_zero),
            blocked=bool(blocked),
            changed=changed,
            reason=reason,
            state=self.state,
        )


# =============================================================================
# Functional one-shot helper (tests/adapters)
# =============================================================================
def evaluate_watchdog_policy_once(
    *,
    timeout_watchdog: TimeoutWatchdog,
    stale_policy: StaleCommandPolicy,
    enabled: bool,
    estop_active: bool,
    estop_latched: bool,
    estop_has_priority: bool = True,
    pass_when_disabled: bool = False,
    source: str = "oneshot",
    now_monotonic_s: Optional[float] = None,
) -> WatchdogPolicyDecision:
    """
    Compute a one-shot final decision from externally-managed components.

    Useful in tests or when nodes manage child policies directly.
    """
    now_s = monotonic_time_s() if now_monotonic_s is None else float(now_monotonic_s)

    if not enabled:
        if pass_when_disabled:
            return WatchdogPolicyDecision(
                permission=MotionPermission.PASS,
                force_zero=False,
                blocked=False,
                changed=False,
                reason=f"policy_disabled_pass_{source}",
            )
        return WatchdogPolicyDecision(
            permission=MotionPermission.BLOCKED_DISABLED,
            force_zero=True,
            blocked=True,
            changed=False,
            reason=f"policy_disabled_block_{source}",
        )

    estop_asserted = bool(estop_active or estop_latched)
    if estop_has_priority and estop_asserted:
        return WatchdogPolicyDecision(
            permission=MotionPermission.BLOCKED_ESTOP,
            force_zero=True,
            blocked=True,
            changed=False,
            reason=f"estop_asserted_{source}",
        )

    if timeout_watchdog.tripped:
        sp_eval = stale_policy.evaluate_output_action(source=source, now_monotonic_s=now_s)
        return WatchdogPolicyDecision(
            permission=MotionPermission.BLOCKED_TIMEOUT,
            force_zero=(sp_eval.action == OutputAction.FORCE_ZERO),
            blocked=True,
            changed=False,
            reason=f"timeout_tripped_{source}",
        )

    sp_eval = stale_policy.evaluate_output_action(source=source, now_monotonic_s=now_s)
    if sp_eval.action != OutputAction.PASS or stale_policy.blocked:
        return WatchdogPolicyDecision(
            permission=MotionPermission.BLOCKED_POLICY,
            force_zero=(sp_eval.action == OutputAction.FORCE_ZERO),
            blocked=True,
            changed=False,
            reason=f"stale_policy_blocked_{source}",
        )

    if (not estop_has_priority) and estop_asserted:
        return WatchdogPolicyDecision(
            permission=MotionPermission.BLOCKED_ESTOP,
            force_zero=True,
            blocked=True,
            changed=False,
            reason=f"estop_asserted_nonpriority_{source}",
        )

    return WatchdogPolicyDecision(
        permission=MotionPermission.PASS,
        force_zero=False,
        blocked=False,
        changed=False,
        reason=f"motion_allowed_{source}",
    )


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    "MotionPermission",
    "WatchdogPolicyConfig",
    "WatchdogPolicyState",
    "WatchdogPolicyDecision",
    "WatchdogPolicyStepResult",
    "WatchdogPolicy",
    "monotonic_time_s",
    "evaluate_watchdog_policy_once",
]