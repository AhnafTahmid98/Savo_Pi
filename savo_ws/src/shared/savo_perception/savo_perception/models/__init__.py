#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared Python data models for savo_perception."""

from __future__ import annotations

from savo_perception.models.range_sample import (
    RangePair,
    RangeSample,
    RangeSnapshot,
    is_valid_distance,
    make_invalid_snapshot,
    sanitize_distance,
)
from savo_perception.models.safety_state import (
    SafetyDecision,
    SafetyState,
    choose_worst_decision,
    clamp_slowdown_factor,
)
from savo_perception.models.sensor_health import (
    RangeHealthSummary,
    SensorHealth,
    summarize_samples,
)


__all__ = [
    # range_sample
    "RangeSample",
    "RangePair",
    "RangeSnapshot",
    "is_valid_distance",
    "sanitize_distance",
    "make_invalid_snapshot",
    # safety_state
    "SafetyDecision",
    "SafetyState",
    "clamp_slowdown_factor",
    "choose_worst_decision",
    # sensor_health
    "SensorHealth",
    "RangeHealthSummary",
    "summarize_samples",
]