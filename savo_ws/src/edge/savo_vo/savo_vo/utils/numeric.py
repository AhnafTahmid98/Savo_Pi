"""Small numeric helpers used by savo_vo."""

from math import isfinite


def clamp(value: float, minimum: float, maximum: float) -> float:
    if minimum > maximum:
        raise ValueError("minimum must be less than or equal to maximum")

    return max(minimum, min(maximum, value))


def clamp01(value: float) -> float:
    return clamp(value, 0.0, 1.0)


def safe_divide(numerator: float, denominator: float, default: float = 0.0) -> float:
    if denominator == 0.0:
        return default

    return numerator / denominator


def is_finite(value: float) -> bool:
    return isfinite(value)


def all_finite(values: list[float] | tuple[float, ...]) -> bool:
    return all(is_finite(value) for value in values)