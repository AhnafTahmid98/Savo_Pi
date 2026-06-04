#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for command limiting helpers: clamp, deadband, normalization, EMA, rate limiter."""

from __future__ import annotations

import math

import pytest

from savo_control.utils.filters import (
    DebounceBool,
    ExponentialMovingAverage,
    FirstOrderCommandFilter3,
    RateLimiter,
    SlewRateLimiter3,
    TimeoutTracker,
    apply_deadband,
    apply_scaled_deadband,
    clamp,
    clamp_abs,
    is_finite_number,
    lerp,
    limit_symmetric_pair,
    normalize_combined_2d,
    normalize_mecanum_command,
    safe_float,
    shape_command_tuple,
    sign,
)


def test_is_finite_number() -> None:
    assert is_finite_number(1.0)
    assert is_finite_number("1.5")
    assert not is_finite_number(float("nan"))
    assert not is_finite_number(float("inf"))
    assert not is_finite_number("not-a-number")


def test_safe_float_valid_and_invalid() -> None:
    assert safe_float(1.25) == pytest.approx(1.25)
    assert safe_float("2.5") == pytest.approx(2.5)

    assert safe_float(float("nan")) == pytest.approx(0.0)
    assert safe_float(float("inf")) == pytest.approx(0.0)
    assert safe_float("bad", default=7.0) == pytest.approx(7.0)


def test_clamp_inside_range() -> None:
    assert clamp(0.5, 0.0, 1.0) == pytest.approx(0.5)


def test_clamp_low_and_high() -> None:
    assert clamp(-1.0, 0.0, 1.0) == pytest.approx(0.0)
    assert clamp(2.0, 0.0, 1.0) == pytest.approx(1.0)


def test_clamp_swaps_reversed_bounds() -> None:
    assert clamp(0.5, 1.0, 0.0) == pytest.approx(0.5)
    assert clamp(-1.0, 1.0, 0.0) == pytest.approx(0.0)
    assert clamp(2.0, 1.0, 0.0) == pytest.approx(1.0)


def test_clamp_abs() -> None:
    assert clamp_abs(0.5, 1.0) == pytest.approx(0.5)
    assert clamp_abs(2.0, 1.0) == pytest.approx(1.0)
    assert clamp_abs(-2.0, 1.0) == pytest.approx(-1.0)


def test_clamp_abs_negative_limit_is_allowed() -> None:
    assert clamp_abs(2.0, -1.0) == pytest.approx(1.0)


def test_apply_deadband() -> None:
    assert apply_deadband(0.00, 0.10) == pytest.approx(0.0)
    assert apply_deadband(0.05, 0.10) == pytest.approx(0.0)
    assert apply_deadband(-0.05, 0.10) == pytest.approx(0.0)

    assert apply_deadband(0.15, 0.10) == pytest.approx(0.15)
    assert apply_deadband(-0.15, 0.10) == pytest.approx(-0.15)


def test_apply_deadband_boundary() -> None:
    # Current implementation zeros only if abs(value) < deadband.
    assert apply_deadband(0.10, 0.10) == pytest.approx(0.10)
    assert apply_deadband(-0.10, 0.10) == pytest.approx(-0.10)


def test_apply_scaled_deadband_inside_deadband() -> None:
    assert apply_scaled_deadband(0.05, deadband=0.10, max_abs=1.0) == pytest.approx(0.0)
    assert apply_scaled_deadband(-0.05, deadband=0.10, max_abs=1.0) == pytest.approx(0.0)


def test_apply_scaled_deadband_rescales_positive() -> None:
    output = apply_scaled_deadband(0.55, deadband=0.10, max_abs=1.0)

    # (0.55 - 0.10) / (1.00 - 0.10) = 0.5
    assert output == pytest.approx(0.5)


def test_apply_scaled_deadband_rescales_negative() -> None:
    output = apply_scaled_deadband(-0.55, deadband=0.10, max_abs=1.0)

    assert output == pytest.approx(-0.5)


def test_apply_scaled_deadband_clamps_to_limit() -> None:
    assert apply_scaled_deadband(2.0, deadband=0.10, max_abs=1.0) == pytest.approx(1.0)
    assert apply_scaled_deadband(-2.0, deadband=0.10, max_abs=1.0) == pytest.approx(-1.0)


def test_lerp() -> None:
    assert lerp(0.0, 10.0, 0.0) == pytest.approx(0.0)
    assert lerp(0.0, 10.0, 1.0) == pytest.approx(10.0)
    assert lerp(0.0, 10.0, 0.5) == pytest.approx(5.0)


def test_lerp_clamps_alpha() -> None:
    assert lerp(0.0, 10.0, -1.0) == pytest.approx(0.0)
    assert lerp(0.0, 10.0, 2.0) == pytest.approx(10.0)


def test_sign() -> None:
    assert sign(1.0) == 1
    assert sign(-1.0) == -1
    assert sign(0.0) == 0


def test_sign_with_deadband() -> None:
    assert sign(0.05, deadband=0.10) == 0
    assert sign(-0.05, deadband=0.10) == 0
    assert sign(0.20, deadband=0.10) == 1
    assert sign(-0.20, deadband=0.10) == -1


def test_limit_symmetric_pair() -> None:
    x, y = limit_symmetric_pair(2.0, -3.0, max_abs_x=1.0, max_abs_y=2.0)

    assert x == pytest.approx(1.0)
    assert y == pytest.approx(-2.0)


def test_normalize_combined_2d_inside_limit() -> None:
    x, y = normalize_combined_2d(0.3, 0.4, max_combined=1.0)

    assert x == pytest.approx(0.3)
    assert y == pytest.approx(0.4)


def test_normalize_combined_2d_scales_vector() -> None:
    x, y = normalize_combined_2d(3.0, 4.0, max_combined=1.0)

    assert math.sqrt(x * x + y * y) == pytest.approx(1.0)
    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)


def test_normalize_combined_2d_zero_limit() -> None:
    x, y = normalize_combined_2d(1.0, 1.0, max_combined=0.0)

    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)


def test_normalize_mecanum_command_inside_limit() -> None:
    vx, vy, wz = normalize_mecanum_command(
        0.2,
        0.2,
        0.2,
        max_combined=1.0,
        wz_weight=0.7,
    )

    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(0.2)
    assert wz == pytest.approx(0.2)


def test_normalize_mecanum_command_scales_when_over_limit() -> None:
    vx, vy, wz = normalize_mecanum_command(
        1.0,
        1.0,
        1.0,
        max_combined=1.0,
        wz_weight=1.0,
    )

    # combined demand = 3.0, scale = 1/3
    assert vx == pytest.approx(1.0 / 3.0)
    assert vy == pytest.approx(1.0 / 3.0)
    assert wz == pytest.approx(1.0 / 3.0)


def test_normalize_mecanum_command_with_rotation_weight() -> None:
    vx, vy, wz = normalize_mecanum_command(
        0.5,
        0.5,
        1.0,
        max_combined=1.0,
        wz_weight=0.5,
    )

    # combined demand = 0.5 + 0.5 + 0.5*1.0 = 1.5
    # scale = 1 / 1.5
    expected = 1.0 / 1.5

    assert vx == pytest.approx(0.5 * expected)
    assert vy == pytest.approx(0.5 * expected)
    assert wz == pytest.approx(1.0 * expected)


def test_exponential_moving_average_first_update_sets_value() -> None:
    ema = ExponentialMovingAverage(alpha=0.5)

    assert ema.update(10.0) == pytest.approx(10.0)


def test_exponential_moving_average_smooths() -> None:
    ema = ExponentialMovingAverage(alpha=0.5)

    ema.update(0.0)
    assert ema.update(10.0) == pytest.approx(5.0)
    assert ema.update(10.0) == pytest.approx(7.5)


def test_exponential_moving_average_alpha_one_jumps_to_target() -> None:
    ema = ExponentialMovingAverage(alpha=1.0)

    ema.update(0.0)
    assert ema.update(10.0) == pytest.approx(10.0)


def test_exponential_moving_average_alpha_zero_freezes_after_first_value() -> None:
    ema = ExponentialMovingAverage(alpha=0.0)

    ema.update(0.0)
    assert ema.update(10.0) == pytest.approx(0.0)


def test_exponential_moving_average_reset() -> None:
    ema = ExponentialMovingAverage(alpha=0.5)

    ema.update(10.0)
    ema.reset()
    assert ema.value is None
    assert ema.update(4.0) == pytest.approx(4.0)


def test_rate_limiter_first_update_initializes_to_target() -> None:
    limiter = RateLimiter(max_rise_per_s=1.0)

    assert limiter.update(5.0, dt_s=0.1) == pytest.approx(5.0)


def test_rate_limiter_rise_limit() -> None:
    limiter = RateLimiter(max_rise_per_s=1.0)
    limiter.reset(0.0)

    assert limiter.update(10.0, dt_s=0.5) == pytest.approx(0.5)
    assert limiter.update(10.0, dt_s=0.5) == pytest.approx(1.0)


def test_rate_limiter_fall_limit() -> None:
    limiter = RateLimiter(max_rise_per_s=1.0, max_fall_per_s=2.0)
    limiter.reset(10.0)

    assert limiter.update(0.0, dt_s=0.5) == pytest.approx(9.0)
    assert limiter.update(0.0, dt_s=0.5) == pytest.approx(8.0)


def test_rate_limiter_reaches_target_if_delta_small() -> None:
    limiter = RateLimiter(max_rise_per_s=10.0)
    limiter.reset(0.0)

    assert limiter.update(1.0, dt_s=1.0) == pytest.approx(1.0)


def test_rate_limiter_handles_zero_dt() -> None:
    limiter = RateLimiter(max_rise_per_s=1.0)
    limiter.reset(0.0)

    assert limiter.update(10.0, dt_s=0.0) == pytest.approx(0.0)


def test_slew_rate_limiter_3_axes() -> None:
    limiter = SlewRateLimiter3(
        vx_limiter=RateLimiter(max_rise_per_s=1.0),
        vy_limiter=RateLimiter(max_rise_per_s=2.0),
        wz_limiter=RateLimiter(max_rise_per_s=3.0),
    )
    limiter.reset(0.0, 0.0, 0.0)

    vx, vy, wz = limiter.update(10.0, 10.0, 10.0, dt_s=0.5)

    assert vx == pytest.approx(0.5)
    assert vy == pytest.approx(1.0)
    assert wz == pytest.approx(1.5)


def test_first_order_command_filter_first_update_sets_values() -> None:
    filt = FirstOrderCommandFilter3(alpha_vx=0.5, alpha_vy=0.5, alpha_wz=0.5)

    vx, vy, wz = filt.update(1.0, 2.0, 3.0)

    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(2.0)
    assert wz == pytest.approx(3.0)


def test_first_order_command_filter_smooths_values() -> None:
    filt = FirstOrderCommandFilter3(alpha_vx=0.5, alpha_vy=0.25, alpha_wz=1.0)

    filt.update(0.0, 0.0, 0.0)
    vx, vy, wz = filt.update(1.0, 1.0, 1.0)

    assert vx == pytest.approx(0.5)
    assert vy == pytest.approx(0.25)
    assert wz == pytest.approx(1.0)


def test_first_order_command_filter_reset() -> None:
    filt = FirstOrderCommandFilter3(alpha_vx=0.5, alpha_vy=0.5, alpha_wz=0.5)

    filt.update(1.0, 1.0, 1.0)
    filt.reset()
    vx, vy, wz = filt.update(2.0, 3.0, 4.0)

    assert vx == pytest.approx(2.0)
    assert vy == pytest.approx(3.0)
    assert wz == pytest.approx(4.0)


def test_debounce_bool_true_transition() -> None:
    debounce = DebounceBool(true_count_required=3, false_count_required=2)

    assert debounce.update(True) is False
    assert debounce.update(True) is False
    assert debounce.update(True) is True


def test_debounce_bool_false_transition() -> None:
    debounce = DebounceBool(true_count_required=2, false_count_required=3, state=True)

    assert debounce.update(False) is True
    assert debounce.update(False) is True
    assert debounce.update(False) is False


def test_debounce_bool_reset() -> None:
    debounce = DebounceBool(true_count_required=2, false_count_required=2)

    debounce.update(True)
    debounce.update(True)
    assert debounce.state is True

    debounce.reset(False)
    assert debounce.state is False
    assert debounce.true_count == 0
    assert debounce.false_count == 0


def test_timeout_tracker_no_seen_is_stale() -> None:
    tracker = TimeoutTracker(timeout_s=1.0)

    assert tracker.is_fresh(now_s=10.0) is False
    assert tracker.is_stale(now_s=10.0) is True
    assert tracker.age_s(now_s=10.0) is None


def test_timeout_tracker_fresh_and_stale() -> None:
    tracker = TimeoutTracker(timeout_s=1.0)

    tracker.mark_seen(10.0)

    assert tracker.is_fresh(now_s=10.5) is True
    assert tracker.is_stale(now_s=10.5) is False
    assert tracker.age_s(now_s=10.5) == pytest.approx(0.5)

    assert tracker.is_fresh(now_s=11.1) is False
    assert tracker.is_stale(now_s=11.1) is True


def test_timeout_tracker_clear() -> None:
    tracker = TimeoutTracker(timeout_s=1.0)

    tracker.mark_seen(10.0)
    tracker.clear()

    assert tracker.last_seen_s is None
    assert tracker.is_fresh(now_s=10.1) is False


def test_shape_command_tuple_clamps_values() -> None:
    vx, vy, wz = shape_command_tuple(
        1.0,
        -1.0,
        2.0,
        max_vx=0.2,
        max_vy=0.3,
        max_wz=0.4,
        normalize_mecanum=False,
    )

    assert vx == pytest.approx(0.2)
    assert vy == pytest.approx(-0.3)
    assert wz == pytest.approx(0.4)


def test_shape_command_tuple_applies_deadband() -> None:
    vx, vy, wz = shape_command_tuple(
        0.01,
        -0.01,
        0.01,
        max_vx=1.0,
        max_vy=1.0,
        max_wz=1.0,
        deadband_vx=0.05,
        deadband_vy=0.05,
        deadband_wz=0.05,
        normalize_mecanum=False,
    )

    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)


def test_shape_command_tuple_normalizes_mecanum() -> None:
    vx, vy, wz = shape_command_tuple(
        1.0,
        1.0,
        1.0,
        max_vx=1.0,
        max_vy=1.0,
        max_wz=1.0,
        normalize_mecanum=True,
        max_combined=1.0,
        wz_combined_weight=1.0,
    )

    assert vx == pytest.approx(1.0 / 3.0)
    assert vy == pytest.approx(1.0 / 3.0)
    assert wz == pytest.approx(1.0 / 3.0)


def test_shape_command_tuple_rejects_invalid_values() -> None:
    vx, vy, wz = shape_command_tuple(
        float("nan"),
        float("inf"),
        -float("inf"),
        max_vx=1.0,
        max_vy=1.0,
        max_wz=1.0,
        normalize_mecanum=False,
    )

    assert vx == pytest.approx(0.0)
    assert vy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)