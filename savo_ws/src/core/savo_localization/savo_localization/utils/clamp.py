#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small numeric clamp helpers. No ROS imports."""

from __future__ import annotations

import math
from typing import TypeVar

Number = TypeVar("Number", int, float)


def clamp(value: Number, minimum: Number, maximum: Number) -> Number:
    if maximum < minimum:
        raise ValueError(f"maximum must be >= minimum, got {maximum} < {minimum}")

    return max(minimum, min(maximum, value))


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    return float(clamp(float(value), float(minimum), float(maximum)))


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    return int(clamp(int(value), int(minimum), int(maximum)))


def clamp_abs(value: float, max_abs: float) -> float:
    max_abs = abs(float(max_abs))
    return clamp_float(float(value), -max_abs, max_abs)


def clamp_unit(value: float) -> float:
    return clamp_float(float(value), 0.0, 1.0)


def clamp_signed_unit(value: float) -> float:
    return clamp_float(float(value), -1.0, 1.0)


def is_finite_number(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_or_default(value: object, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(number):
        return float(default)

    return number


def non_negative_or_default(value: object, default: float = 0.0) -> float:
    number = finite_or_default(value, default)

    if number < 0.0:
        return float(default)

    return number


def positive_or_default(value: object, default: float) -> float:
    number = finite_or_default(value, default)

    if number <= 0.0:
        return float(default)

    return number