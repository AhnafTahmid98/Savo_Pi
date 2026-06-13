# -*- coding: utf-8 -*-
"""Small numeric clamp helpers."""

from __future__ import annotations

import math


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    minimum = float(minimum)
    maximum = float(maximum)

    if minimum > maximum:
        raise ValueError(f"minimum cannot be greater than maximum: {minimum} > {maximum}")

    value = float(value)
    return max(minimum, min(maximum, value))


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    minimum = int(minimum)
    maximum = int(maximum)

    if minimum > maximum:
        raise ValueError(f"minimum cannot be greater than maximum: {minimum} > {maximum}")

    value = int(value)
    return max(minimum, min(maximum, value))


def finite_or_default(value: float, default: float) -> float:
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(value_f):
        return float(default)

    return value_f


def positive_or_default(value: float, default: float) -> float:
    value_f = finite_or_default(value, default)

    if value_f <= 0.0:
        return float(default)

    return value_f


def ratio_or_default(value: float, default: float = 0.0) -> float:
    value_f = finite_or_default(value, default)
    return clamp_float(value_f, 0.0, 1.0)


__all__ = [
    "clamp_float",
    "clamp_int",
    "finite_or_default",
    "positive_or_default",
    "ratio_or_default",
]
