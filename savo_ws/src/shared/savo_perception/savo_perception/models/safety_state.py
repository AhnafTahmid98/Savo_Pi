#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Safety state models for perception stop/slowdown decisions."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

from savo_perception.constants import (
    SLOWDOWN_DEFAULT,
    SLOWDOWN_MAX_DEFAULT,
    SLOWDOWN_MIN_DEFAULT,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_SAFETY_STOP,
    STATUS_SLOW,
)


def clamp_slowdown_factor(value: float) -> float:
    v = float(value)
    if v < SLOWDOWN_MIN_DEFAULT:
        return SLOWDOWN_MIN_DEFAULT
    if v > SLOWDOWN_MAX_DEFAULT:
        return SLOWDOWN_MAX_DEFAULT
    return v


@dataclass(frozen=True)
class SafetyDecision:
    stop_required: bool
    slowdown_factor: float = SLOWDOWN_DEFAULT
    status: str = STATUS_OK
    reason: str = "clear"

    front_distance_m: Optional[float] = None
    side_distance_m: Optional[float] = None

    stale_sensors: List[str] = field(default_factory=list)
    invalid_sensors: List[str] = field(default_factory=list)

    source: str = "safety_fusion"
    stamp_mono_s: float = field(default_factory=time.monotonic)

    @classmethod
    def clear(cls, *, reason: str = "clear") -> "SafetyDecision":
        return cls(
            stop_required=False,
            slowdown_factor=SLOWDOWN_DEFAULT,
            status=STATUS_OK,
            reason=reason,
        )

    @classmethod
    def slow(
        cls,
        *,
        slowdown_factor: float,
        reason: str,
        front_distance_m: Optional[float] = None,
        side_distance_m: Optional[float] = None,
    ) -> "SafetyDecision":
        return cls(
            stop_required=False,
            slowdown_factor=clamp_slowdown_factor(slowdown_factor),
            status=STATUS_SLOW,
            reason=reason,
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
        )

    @classmethod
    def stop(
        cls,
        *,
        reason: str,
        front_distance_m: Optional[float] = None,
        side_distance_m: Optional[float] = None,
        stale_sensors: Optional[List[str]] = None,
        invalid_sensors: Optional[List[str]] = None,
    ) -> "SafetyDecision":
        return cls(
            stop_required=True,
            slowdown_factor=0.0,
            status=STATUS_SAFETY_STOP,
            reason=reason,
            front_distance_m=front_distance_m,
            side_distance_m=side_distance_m,
            stale_sensors=list(stale_sensors or []),
            invalid_sensors=list(invalid_sensors or []),
        )

    @classmethod
    def error(
        cls,
        *,
        reason: str,
        stale_sensors: Optional[List[str]] = None,
        invalid_sensors: Optional[List[str]] = None,
    ) -> "SafetyDecision":
        return cls(
            stop_required=True,
            slowdown_factor=0.0,
            status=STATUS_ERROR,
            reason=reason,
            stale_sensors=list(stale_sensors or []),
            invalid_sensors=list(invalid_sensors or []),
        )

    def age_s(self, now_mono_s: Optional[float] = None) -> float:
        now_s = time.monotonic() if now_mono_s is None else float(now_mono_s)
        return max(0.0, now_s - float(self.stamp_mono_s))

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["slowdown_factor"] = clamp_slowdown_factor(self.slowdown_factor) if not self.stop_required else 0.0
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


@dataclass(frozen=True)
class SafetyState:
    active_decision: SafetyDecision
    last_clear_decision: Optional[SafetyDecision] = None
    stop_count: int = 0
    clear_count: int = 0
    update_count: int = 0

    @classmethod
    def initial(cls) -> "SafetyState":
        return cls(active_decision=SafetyDecision.clear(reason="initial"))

    def with_decision(
        self,
        decision: SafetyDecision,
        *,
        stop_count: Optional[int] = None,
        clear_count: Optional[int] = None,
    ) -> "SafetyState":
        last_clear = self.last_clear_decision
        if not decision.stop_required:
            last_clear = decision

        return SafetyState(
            active_decision=decision,
            last_clear_decision=last_clear,
            stop_count=self.stop_count if stop_count is None else int(stop_count),
            clear_count=self.clear_count if clear_count is None else int(clear_count),
            update_count=self.update_count + 1,
        )

    @property
    def stop_required(self) -> bool:
        return self.active_decision.stop_required

    @property
    def slowdown_factor(self) -> float:
        return self.active_decision.slowdown_factor

    @property
    def status(self) -> str:
        return self.active_decision.status

    @property
    def reason(self) -> str:
        return self.active_decision.reason

    def to_dict(self) -> Dict[str, Any]:
        return {
            "active_decision": self.active_decision.to_dict(),
            "last_clear_decision": (
                self.last_clear_decision.to_dict()
                if self.last_clear_decision is not None
                else None
            ),
            "stop_count": self.stop_count,
            "clear_count": self.clear_count,
            "update_count": self.update_count,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))


def choose_worst_decision(decisions: List[SafetyDecision]) -> SafetyDecision:
    if not decisions:
        return SafetyDecision.clear(reason="no_decisions")

    stop_decisions = [d for d in decisions if d.stop_required]
    if stop_decisions:
        return stop_decisions[0]

    return min(decisions, key=lambda d: float(d.slowdown_factor))


__all__ = [
    "SafetyDecision",
    "SafetyState",
    "clamp_slowdown_factor",
    "choose_worst_decision",
]