#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Safety logic helpers for savo_perception Python fallback and diagnostics."""

from __future__ import annotations

from savo_perception.safety.range_fusion import (
    RangeFusionConfig,
    RangeFusionResult,
    fuse_range_snapshot,
    min_optional,
    required_stale_sensors,
    slowdown_from_distance,
)
from savo_perception.safety.safety_policy import (
    SafetyPolicy,
    SafetyPolicyConfig,
    SafetyPolicyUpdate,
    fusion_config_for_state,
)
from savo_perception.safety.safety_state import (
    SafetyDecision,
    SafetyState,
    choose_worst_decision,
    clamp_slowdown_factor,
    decision_publish_payload,
    state_publish_payload,
)


__all__ = [
    # range fusion
    "RangeFusionConfig",
    "RangeFusionResult",
    "fuse_range_snapshot",
    "slowdown_from_distance",
    "min_optional",
    "required_stale_sensors",
    # safety policy
    "SafetyPolicy",
    "SafetyPolicyConfig",
    "SafetyPolicyUpdate",
    "fusion_config_for_state",
    # safety state
    "SafetyDecision",
    "SafetyState",
    "clamp_slowdown_factor",
    "choose_worst_decision",
    "decision_publish_payload",
    "state_publish_payload",
]