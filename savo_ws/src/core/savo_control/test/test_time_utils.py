#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for shared time utilities."""

from __future__ import annotations

from savo_control.utils import (
    TimeCheck,
    age_s,
    check_age,
    clamp_dt,
    is_fresh,
    is_stale,
    safe_rate_period_s,
)


def test_age_s_with_explicit_now():
    assert age_s(10.0, now=12.5) == 2.5
    assert age_s(5.0, now=5.0) == 0.0


def test_is_stale():
    assert is_stale(10.0, 2.0, now=12.1) is True
    assert is_stale(10.0, 2.0, now=12.0) is False
    assert is_stale(10.0, 2.0, now=11.5) is False


def test_is_fresh():
    assert is_fresh(10.0, 2.0, now=12.1) is False
    assert is_fresh(10.0, 2.0, now=12.0) is True
    assert is_fresh(10.0, 2.0, now=11.5) is True


def test_check_age_returns_time_check():
    check = check_age(10.0, 2.0, now=12.5)

    assert isinstance(check, TimeCheck)
    assert check.age_s == 2.5
    assert check.timeout_s == 2.0
    assert check.stale is True
    assert check.fresh is False


def test_time_check_to_dict():
    check = TimeCheck(age_s=0.25, timeout_s=0.50)

    assert check.to_dict() == {
        "age_s": 0.25,
        "timeout_s": 0.50,
        "fresh": True,
        "stale": False,
    }


def test_time_check_boundary_is_fresh():
    check = TimeCheck(age_s=0.50, timeout_s=0.50)

    assert check.stale is False
    assert check.fresh is True


def test_clamp_dt():
    assert clamp_dt(0.0001, min_dt_s=0.001, max_dt_s=0.20) == 0.001
    assert clamp_dt(0.50, min_dt_s=0.001, max_dt_s=0.20) == 0.20
    assert clamp_dt(0.05, min_dt_s=0.001, max_dt_s=0.20) == 0.05


def test_safe_rate_period_s():
    assert safe_rate_period_s(20.0) == 0.05
    assert safe_rate_period_s(10.0) == 0.10
    assert safe_rate_period_s(0.0) == 0.10
    assert safe_rate_period_s(-5.0) == 0.10


def test_safe_rate_period_s_uses_fallback():
    assert safe_rate_period_s(0.0, fallback_hz=5.0) == 0.20
    assert safe_rate_period_s(-1.0, fallback_hz=2.0) == 0.50


def test_safe_rate_period_s_handles_bad_fallback():
    assert safe_rate_period_s(0.0, fallback_hz=0.0) == 0.10
    assert safe_rate_period_s(-1.0, fallback_hz=-5.0) == 0.10
