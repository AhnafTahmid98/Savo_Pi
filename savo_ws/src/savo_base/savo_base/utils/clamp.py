#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/clamp.py
-------------------------------------
Small, reusable clamping helpers for the `savo_base` package.

Why this file exists
--------------------
Real robot control code touches many values that must stay bounded:
- normalized velocity commands (e.g. -1.0 .. 1.0)
- duty cycle limits (e.g. 0 .. 4095)
- slowdown factors (e.g. 0.0 .. 1.0)
- watchdog ratios / safety parameters

Keeping clamp logic centralized avoids duplicated edge-case handling.

Design goals
------------
- Tiny and dependency-free
- Predictable behavior (no hidden side effects)
- ROS2 Jazzy friendly (safe in both nodes and pure Python modules)
- Explicit helpers for float / int / tuple use-cases
"""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple, TypeVar

Number = TypeVar("Number", int, float)


def clamp(value: Number, lo: Number, hi: Number) -> Number:
    """
    Clamp `value` into the closed interval [lo, hi].

    Parameters
    ----------
    value : int | float
        Input value to clamp.
    lo : int | float
        Lower bound.
    hi : int | float
        Upper bound.

    Returns
    -------
    int | float
        Clamped value (same numeric type family as the inputs).

    Notes
    -----
    - If `lo > hi`, the bounds are swapped to keep behavior robust.
    - This helper is intentionally permissive for runtime safety code.
    """
    if lo > hi:
        lo, hi = hi, lo
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def clamp_float(value: float, lo: float, hi: float) -> float:
    """
    Clamp a float into [lo, hi] and return float.
    """
    return float(clamp(float(value), float(lo), float(hi)))


def clamp_int(value: int, lo: int, hi: int) -> int:
    """
    Clamp an integer into [lo, hi] and return int.

    The input is converted with `int(...)` before clamping.
    """
    v = int(value)
    a = int(lo)
    b = int(hi)
    if a > b:
        a, b = b, a
    if v < a:
        return a
    if v > b:
        return b
    return v


def clamp01(value: float) -> float:
    """
    Clamp a float into [0.0, 1.0].

    Common use-cases:
    - slowdown factor
    - normalized scale
    - interpolation factor
    """
    return clamp_float(float(value), 0.0, 1.0)


def clamp_symmetric(value: float, limit: float) -> float:
    """
    Clamp a float symmetrically into [-abs(limit), +abs(limit)].

    Examples
    --------
    >>> clamp_symmetric(1.5, 1.0)
    1.0
    >>> clamp_symmetric(-2.0, 0.5)
    -0.5
    """
    lim = abs(float(limit))
    return clamp_float(float(value), -lim, +lim)


def clamp_tuple4(
    v1: float,
    v2: float,
    v3: float,
    v4: float,
    *,
    lo: float,
    hi: float,
) -> Tuple[float, float, float, float]:
    """
    Clamp four float values into [lo, hi].

    Useful for wheel normalized values (FL, RL, FR, RR) or duty values
    before final int conversion.
    """
    return (
        clamp_float(v1, lo, hi),
        clamp_float(v2, lo, hi),
        clamp_float(v3, lo, hi),
        clamp_float(v4, lo, hi),
    )


def clamp_iterable(values: Iterable[float], *, lo: float, hi: float) -> Tuple[float, ...]:
    """
    Clamp an iterable of numeric values into [lo, hi] and return a tuple.

    Useful for generic pipelines and debug tooling where wheel count may vary.
    """
    return tuple(clamp_float(float(v), lo, hi) for v in values)


def clamp_signed_duty(value: int, max_abs_duty: int = 4095) -> int:
    """
    Clamp signed duty command into [-max_abs_duty, +max_abs_duty].

    Parameters
    ----------
    value : int
        Signed duty command (can be negative for reverse direction).
    max_abs_duty : int
        Maximum absolute duty. Typically <= 4095 for PCA9685.

    Returns
    -------
    int
        Safe signed duty command.
    """
    lim = clamp_int(int(max_abs_duty), 0, 4095)
    return clamp_int(int(value), -lim, +lim)


def clamp_unsigned_duty(value: int, max_duty: int = 4095) -> int:
    """
    Clamp unsigned duty command into [0, max_duty].

    Useful for low-level PCA9685 channel writes.
    """
    lim = clamp_int(int(max_duty), 0, 4095)
    return clamp_int(int(value), 0, lim)