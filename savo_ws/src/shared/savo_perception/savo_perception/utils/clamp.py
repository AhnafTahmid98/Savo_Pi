#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small numeric clamp helpers used by perception safety logic."""

from __future__ import annotations

import math
from typing import Optional

from savo_perception.constants import (
    SLOWDOWN_MAX_DEFAULT,
    SLOWDOWN_MIN_DEFAULT,
)


def clamp(value: float, min_value: float, max_value: float) -> float:
    v = float(value)
    lo = float(min_value)
    hi = float(max_value)

    if lo > hi:
        lo, hi = hi, lo

    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def clamp_abs(value: float, limit: float) -> float:
    lim = abs(float(limit))
    if lim <= 0.0:
        return 0.0
    return clamp(float(value), -lim, lim)


def clamp01(value: float) -> float:
    return clamp(float(value), 0.0, 1.0)


def clamp_slowdown(value: float) -> float:
    return clamp(float(value), SLOWDOWN_MIN_DEFAULT, SLOWDOWN_MAX_DEFAULT)


def safe_float(
    value: object,
    *,
    default: Optional[float] = None,
    min_value: Optional[float] = None,
    max_value: Optional[float] = None,
) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return default

    if not math.isfinite(out):
        return default

    if min_value is not None and out < float(min_value):
        out = float(min_value)

    if max_value is not None and out > float(max_value):
        out = float(max_value)

    return out


def safe_positive_float(value: object, *, default: float) -> float:
    out = safe_float(value, default=default, min_value=0.0)
    return float(default) if out is None else out


def safe_rate_hz(value: object, *, default: float, min_hz: float = 0.1) -> float:
    out = safe_float(value, default=default, min_value=min_hz)
    return float(default) if out is None else out


def valid_unit_interval(value: object) -> bool:
    out = safe_float(value)
    return out is not None and 0.0 <= out <= 1.0


def valid_distance_m(
    value: object,
    *,
    min_m: float = 0.0,
    max_m: float = 10.0,
) -> bool:
    out = safe_float(value)
    return out is not None and float(min_m) <= out <= float(max_m)


__all__ = [
    "clamp",
    "clamp_abs",
    "clamp01",
    "clamp_slowdown",
    "safe_float",
    "safe_positive_float",
    "safe_rate_hz",
    "valid_unit_interval",
    "valid_distance_m",
]