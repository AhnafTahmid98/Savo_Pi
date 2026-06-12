"""Small numeric guards used by LiDAR filters and diagnostics."""

from __future__ import annotations

import math


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    if minimum > maximum:
        raise ValueError(f"Invalid clamp range: minimum={minimum}, maximum={maximum}")

    value = float(value)

    if value < minimum:
        return float(minimum)

    if value > maximum:
        return float(maximum)

    return value


def clamp_int(value: int, minimum: int, maximum: int) -> int:
    if minimum > maximum:
        raise ValueError(f"Invalid clamp range: minimum={minimum}, maximum={maximum}")

    value = int(value)

    if value < minimum:
        return int(minimum)

    if value > maximum:
        return int(maximum)

    return value


def finite_or_default(value: float, default: float) -> float:
    value = float(value)

    if math.isfinite(value):
        return value

    return float(default)


def positive_or_default(value: float, default: float) -> float:
    value = float(value)

    if value > 0.0 and math.isfinite(value):
        return value

    return float(default)


def ratio_or_default(value: float, default: float = 0.0) -> float:
    value = finite_or_default(value, default)
    return clamp_float(value, 0.0, 1.0)