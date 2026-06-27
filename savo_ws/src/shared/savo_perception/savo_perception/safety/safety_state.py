#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Safety-state exports for the perception safety package."""

from __future__ import annotations

from typing import Any, Dict

from savo_perception.models.safety_state import (
    SafetyDecision,
    SafetyState,
    choose_worst_decision,
    clamp_slowdown_factor,
)


def decision_publish_payload(decision: SafetyDecision) -> Dict[str, Any]:
    return {
        "stop_required": decision.stop_required,
        "slowdown_factor": decision.slowdown_factor,
        "status": decision.status,
        "reason": decision.reason,
        "front_distance_m": decision.front_distance_m,
        "side_distance_m": decision.side_distance_m,
        "stale_sensors": list(decision.stale_sensors),
        "invalid_sensors": list(decision.invalid_sensors),
        "source": decision.source,
        "stamp_mono_s": decision.stamp_mono_s,
    }


def state_publish_payload(state: SafetyState) -> Dict[str, Any]:
    return {
        "stop_required": state.stop_required,
        "slowdown_factor": state.slowdown_factor,
        "status": state.status,
        "reason": state.reason,
        "stop_count": state.stop_count,
        "clear_count": state.clear_count,
        "update_count": state.update_count,
        "active_decision": decision_publish_payload(state.active_decision),
    }


__all__ = [
    "SafetyDecision",
    "SafetyState",
    "clamp_slowdown_factor",
    "choose_worst_decision",
    "decision_publish_payload",
    "state_publish_payload",
]