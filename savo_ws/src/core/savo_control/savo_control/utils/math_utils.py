# -*- coding: utf-8 -*-

"""Small math helpers for control-side Python tools."""

from __future__ import annotations

import math


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        low, high = high, low

    if value < low:
        return low
    if value > high:
        return high
    return value


def clamp_abs(value: float, max_abs: float) -> float:
    limit = abs(max_abs)
    return clamp(value, -limit, limit)


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < abs(deadband):
        return 0.0
    return value


def sign(value: float, *, zero: int = 0) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return zero


def wrap_angle_rad(angle_rad: float) -> float:
    wrapped = math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    if math.isclose(wrapped, -math.pi, abs_tol=1e-12):
        return math.pi

    return wrapped


def shortest_angle_error_rad(target_rad: float, current_rad: float) -> float:
    return wrap_angle_rad(target_rad - current_rad)


def near_zero(value: float, eps: float = 1e-9) -> bool:
    return abs(value) <= abs(eps)


def within_tolerance(value: float, target: float, tolerance: float) -> bool:
    return abs(value - target) <= abs(tolerance) + 1e-9


def hypot2(x: float, y: float) -> float:
    return math.hypot(finite_or_zero(x), finite_or_zero(y))


def safe_divide(numerator: float, denominator: float, *, default: float = 0.0) -> float:
    if not math.isfinite(numerator) or not math.isfinite(denominator):
        return default

    if denominator == 0.0:
        return default

    return numerator / denominator


def lerp(start: float, end: float, alpha: float) -> float:
    a = clamp(alpha, 0.0, 1.0)
    return start + (end - start) * a


__all__ = [
    "apply_deadband",
    "clamp",
    "clamp_abs",
    "finite_or_zero",
    "hypot2",
    "lerp",
    "near_zero",
    "safe_divide",
    "shortest_angle_error_rad",
    "sign",
    "within_tolerance",
    "wrap_angle_rad",
]
