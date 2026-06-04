#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / utils / filters.py
==============================================

Small ROS-independent filtering and limiting helpers for Robot Savo control
nodes.

Purpose
-------
Shared helpers for:

    - command clamping
    - deadband handling
    - exponential moving average filtering
    - rate limiting
    - finite-value safety checks
    - Twist-like velocity shaping logic

Architecture
------------
This file is pure Python utility code.

It does NOT:
    - create ROS nodes
    - publish topics
    - subscribe to topics
    - command hardware
    - touch /cmd_vel_safe
    - access GPIO, PCA9685, motors, encoders, or sensors

All functions/classes are deterministic and safe to unit test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


def is_finite_number(value: float) -> bool:
    """Return True if value can be converted to a finite float."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def safe_float(value: float, default: float = 0.0) -> float:
    """Return finite float value, otherwise default."""
    try:
        value_f = float(value)
    except (TypeError, ValueError):
        return float(default)

    if not math.isfinite(value_f):
        return float(default)

    return value_f


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value to [low, high]."""
    value_f = safe_float(value)

    low_f = safe_float(low)
    high_f = safe_float(high)

    if low_f > high_f:
        low_f, high_f = high_f, low_f

    return max(low_f, min(high_f, value_f))


def clamp_abs(value: float, max_abs: float) -> float:
    """Clamp value to [-max_abs, +max_abs]."""
    limit = abs(safe_float(max_abs))
    return clamp(value, -limit, limit)


def apply_deadband(value: float, deadband: float) -> float:
    """
    Return zero if abs(value) is below deadband.

    Useful for joystick noise or tiny unstable commands.
    """
    value_f = safe_float(value)
    db = abs(safe_float(deadband))

    if abs(value_f) < db:
        return 0.0

    return value_f


def apply_scaled_deadband(value: float, deadband: float, max_abs: float = 1.0) -> float:
    """
    Apply deadband and rescale remaining value.

    Example:
        deadband=0.1, max_abs=1.0
        input 0.1 -> 0.0
        input 1.0 -> 1.0

    This is useful for joystick-style input.
    """
    value_f = safe_float(value)
    db = abs(safe_float(deadband))
    limit = abs(safe_float(max_abs, 1.0))

    if limit <= 0.0:
        return 0.0

    value_f = clamp(value_f, -limit, limit)

    if abs(value_f) <= db:
        return 0.0

    sign = 1.0 if value_f > 0.0 else -1.0
    scaled = (abs(value_f) - db) / max(1.0e-9, limit - db)

    return sign * clamp(scaled * limit, 0.0, limit)


def lerp(current: float, target: float, alpha: float) -> float:
    """
    Linear interpolation.

    alpha:
        0.0 = keep current
        1.0 = jump to target
    """
    current_f = safe_float(current)
    target_f = safe_float(target)
    alpha_f = clamp(alpha, 0.0, 1.0)

    return current_f + alpha_f * (target_f - current_f)


def sign(value: float, deadband: float = 0.0) -> int:
    """Return -1, 0, or +1."""
    value_f = safe_float(value)
    db = abs(safe_float(deadband))

    if abs(value_f) <= db:
        return 0

    return 1 if value_f > 0.0 else -1


def limit_symmetric_pair(
    x: float,
    y: float,
    max_abs_x: float,
    max_abs_y: float,
) -> tuple[float, float]:
    """Clamp a pair independently to symmetric limits."""
    return clamp_abs(x, max_abs_x), clamp_abs(y, max_abs_y)


def normalize_combined_2d(
    x: float,
    y: float,
    max_combined: float = 1.0,
) -> tuple[float, float]:
    """
    Scale x/y together if sqrt(x^2 + y^2) exceeds max_combined.

    Useful for translational command limiting.
    """
    x_f = safe_float(x)
    y_f = safe_float(y)
    limit = abs(safe_float(max_combined, 1.0))

    if limit <= 0.0:
        return 0.0, 0.0

    mag = math.sqrt(x_f * x_f + y_f * y_f)

    if mag <= limit or mag <= 1.0e-9:
        return x_f, y_f

    scale = limit / mag
    return x_f * scale, y_f * scale


def normalize_mecanum_command(
    vx: float,
    vy: float,
    wz: float,
    max_combined: float = 1.0,
    wz_weight: float = 0.70,
) -> tuple[float, float, float]:
    """
    Scale vx/vy/wz together using a mecanum-friendly combined demand metric.

    This is not wheel kinematics. It is only a safe command envelope helper.

    combined = |vx| + |vy| + wz_weight * |wz|

    If combined > max_combined, all components are scaled down together.
    """
    vx_f = safe_float(vx)
    vy_f = safe_float(vy)
    wz_f = safe_float(wz)

    limit = abs(safe_float(max_combined, 1.0))
    weight = max(0.0, safe_float(wz_weight, 0.70))

    if limit <= 0.0:
        return 0.0, 0.0, 0.0

    combined = abs(vx_f) + abs(vy_f) + weight * abs(wz_f)

    if combined <= limit or combined <= 1.0e-9:
        return vx_f, vy_f, wz_f

    scale = limit / combined

    return vx_f * scale, vy_f * scale, wz_f * scale


@dataclass
class ExponentialMovingAverage:
    """
    Exponential moving average filter.

    alpha:
        1.0 = no smoothing
        0.0 = frozen output
    """

    alpha: float = 0.5
    value: Optional[float] = None

    def reset(self, value: Optional[float] = None) -> None:
        self.value = None if value is None else safe_float(value)

    def update(self, new_value: float) -> float:
        new_f = safe_float(new_value)
        alpha_f = clamp(self.alpha, 0.0, 1.0)

        if self.value is None:
            self.value = new_f
        else:
            self.value = lerp(self.value, new_f, alpha_f)

        return self.value


@dataclass
class RateLimiter:
    """
    Symmetric/asymmetric rate limiter.

    Use this to limit how fast a value can change per second.

    max_rise_per_s:
        maximum increase per second

    max_fall_per_s:
        maximum decrease per second

    If max_fall_per_s is None, it uses max_rise_per_s.
    """

    max_rise_per_s: float
    max_fall_per_s: Optional[float] = None
    value: float = 0.0
    initialized: bool = False

    def reset(self, value: float = 0.0) -> None:
        self.value = safe_float(value)
        self.initialized = True

    def update(self, target: float, dt_s: float) -> float:
        target_f = safe_float(target)
        dt = max(0.0, safe_float(dt_s))

        if not self.initialized:
            self.value = target_f
            self.initialized = True
            return self.value

        rise = abs(safe_float(self.max_rise_per_s))
        fall = abs(safe_float(self.max_fall_per_s, rise))

        delta = target_f - self.value

        if delta >= 0.0:
            max_delta = rise * dt
        else:
            max_delta = fall * dt

        if abs(delta) <= max_delta:
            self.value = target_f
        else:
            self.value += math.copysign(max_delta, delta)

        return self.value


@dataclass
class SlewRateLimiter3:
    """
    3-axis rate limiter for vx, vy, wz.

    Useful for command shaping before publishing Twist.
    """

    vx_limiter: RateLimiter
    vy_limiter: RateLimiter
    wz_limiter: RateLimiter

    def reset(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        self.vx_limiter.reset(vx)
        self.vy_limiter.reset(vy)
        self.wz_limiter.reset(wz)

    def update(
        self,
        target_vx: float,
        target_vy: float,
        target_wz: float,
        dt_s: float,
    ) -> tuple[float, float, float]:
        vx = self.vx_limiter.update(target_vx, dt_s)
        vy = self.vy_limiter.update(target_vy, dt_s)
        wz = self.wz_limiter.update(target_wz, dt_s)
        return vx, vy, wz


@dataclass
class FirstOrderCommandFilter3:
    """
    3-axis first-order smoothing filter for vx, vy, wz.
    """

    alpha_vx: float = 0.45
    alpha_vy: float = 0.45
    alpha_wz: float = 0.40

    vx: Optional[float] = None
    vy: Optional[float] = None
    wz: Optional[float] = None

    def reset(
        self,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        wz: Optional[float] = None,
    ) -> None:
        self.vx = None if vx is None else safe_float(vx)
        self.vy = None if vy is None else safe_float(vy)
        self.wz = None if wz is None else safe_float(wz)

    def update(
        self,
        target_vx: float,
        target_vy: float,
        target_wz: float,
    ) -> tuple[float, float, float]:
        target_vx = safe_float(target_vx)
        target_vy = safe_float(target_vy)
        target_wz = safe_float(target_wz)

        if self.vx is None:
            self.vx = target_vx
        else:
            self.vx = lerp(self.vx, target_vx, self.alpha_vx)

        if self.vy is None:
            self.vy = target_vy
        else:
            self.vy = lerp(self.vy, target_vy, self.alpha_vy)

        if self.wz is None:
            self.wz = target_wz
        else:
            self.wz = lerp(self.wz, target_wz, self.alpha_wz)

        return self.vx, self.vy, self.wz


@dataclass
class DebounceBool:
    """
    Boolean debounce helper.

    Example:
        require true_count_required=3 before output true
        require false_count_required=3 before output false
    """

    true_count_required: int = 2
    false_count_required: int = 2
    state: bool = False
    true_count: int = 0
    false_count: int = 0

    def reset(self, state: bool = False) -> None:
        self.state = bool(state)
        self.true_count = 0
        self.false_count = 0

    def update(self, value: bool) -> bool:
        if bool(value):
            self.true_count += 1
            self.false_count = 0
        else:
            self.false_count += 1
            self.true_count = 0

        if self.true_count >= max(1, int(self.true_count_required)):
            self.state = True

        if self.false_count >= max(1, int(self.false_count_required)):
            self.state = False

        return self.state


@dataclass
class TimeoutTracker:
    """
    Simple freshness/timeout tracker using monotonic seconds.

    This is ROS-independent. Pass time.monotonic() or node.get_clock time converted
    to seconds.
    """

    timeout_s: float
    last_seen_s: Optional[float] = None

    def mark_seen(self, now_s: float) -> None:
        self.last_seen_s = safe_float(now_s)

    def clear(self) -> None:
        self.last_seen_s = None

    def age_s(self, now_s: float) -> Optional[float]:
        if self.last_seen_s is None:
            return None
        return max(0.0, safe_float(now_s) - self.last_seen_s)

    def is_fresh(self, now_s: float) -> bool:
        age = self.age_s(now_s)
        if age is None:
            return False
        return age <= max(0.0, safe_float(self.timeout_s))

    def is_stale(self, now_s: float) -> bool:
        return not self.is_fresh(now_s)


def shape_command_tuple(
    vx: float,
    vy: float,
    wz: float,
    *,
    max_vx: float,
    max_vy: float,
    max_wz: float,
    deadband_vx: float = 0.0,
    deadband_vy: float = 0.0,
    deadband_wz: float = 0.0,
    normalize_mecanum: bool = True,
    max_combined: float = 1.0,
    wz_combined_weight: float = 0.70,
) -> tuple[float, float, float]:
    """
    Apply common command safety shaping to vx/vy/wz.

    This is useful for unit tests and Python helper nodes.
    """
    vx_f = apply_deadband(safe_float(vx), deadband_vx)
    vy_f = apply_deadband(safe_float(vy), deadband_vy)
    wz_f = apply_deadband(safe_float(wz), deadband_wz)

    vx_f = clamp_abs(vx_f, max_vx)
    vy_f = clamp_abs(vy_f, max_vy)
    wz_f = clamp_abs(wz_f, max_wz)

    if normalize_mecanum:
        vx_f, vy_f, wz_f = normalize_mecanum_command(
            vx_f,
            vy_f,
            wz_f,
            max_combined=max_combined,
            wz_weight=wz_combined_weight,
        )

    return vx_f, vy_f, wz_f