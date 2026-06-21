#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for validation utilities."""

from __future__ import annotations

from math import inf, nan

import pytest

from savo_control.utils import (
    require_bool,
    require_choice,
    require_keys,
    require_mapping,
    require_non_empty_string,
    require_non_negative_float,
    require_positive_float,
    sanitize_float,
    validate_limits,
    validate_rate,
    validate_timeout,
    validate_topic_name,
)


def test_require_non_empty_string():
    assert require_non_empty_string(" AUTO ", name="mode") == "AUTO"
    assert require_non_empty_string(123, name="value") == "123"

    with pytest.raises(ValueError):
        require_non_empty_string("", name="mode")

    with pytest.raises(ValueError):
        require_non_empty_string("   ", name="mode")


def test_require_positive_float():
    assert require_positive_float("1.5", name="rate") == 1.5
    assert require_positive_float(0.1, name="rate") == 0.1

    with pytest.raises(ValueError):
        require_positive_float(0.0, name="rate")

    with pytest.raises(ValueError):
        require_positive_float(-1.0, name="rate")

    with pytest.raises(ValueError):
        require_positive_float("bad", name="rate")


def test_require_non_negative_float():
    assert require_non_negative_float("0.0", name="timeout") == 0.0
    assert require_non_negative_float(1.5, name="timeout") == 1.5

    with pytest.raises(ValueError):
        require_non_negative_float(-0.1, name="timeout")

    with pytest.raises(ValueError):
        require_non_negative_float("bad", name="timeout")


def test_require_bool():
    assert require_bool(True, name="enabled") is True
    assert require_bool(False, name="enabled") is False

    with pytest.raises(ValueError):
        require_bool("true", name="enabled")

    with pytest.raises(ValueError):
        require_bool(1, name="enabled")


def test_require_choice():
    assert require_choice("AUTO", ["STOP", "AUTO"], name="mode") == "AUTO"

    with pytest.raises(ValueError):
        require_choice("MANUAL", ["STOP", "AUTO"], name="mode")

    with pytest.raises(ValueError):
        require_choice("", ["STOP", "AUTO"], name="mode")


def test_require_mapping():
    data = {"rate": 30.0}

    assert require_mapping(data, name="params") is data

    with pytest.raises(ValueError):
        require_mapping([], name="params")

    with pytest.raises(ValueError):
        require_mapping("bad", name="params")


def test_require_keys():
    require_keys({"a": 1, "b": 2}, ["a", "b"], name="params")

    with pytest.raises(ValueError):
        require_keys({"a": 1}, ["a", "b"], name="params")


def test_validate_topic_name():
    assert validate_topic_name("/cmd_vel", name="cmd") == "/cmd_vel"
    assert validate_topic_name("/savo_control/mode_cmd", name="mode") == (
        "/savo_control/mode_cmd"
    )

    with pytest.raises(ValueError):
        validate_topic_name("cmd_vel", name="cmd")

    with pytest.raises(ValueError):
        validate_topic_name("//cmd_vel", name="cmd")

    with pytest.raises(ValueError):
        validate_topic_name("/cmd vel", name="cmd")

    with pytest.raises(ValueError):
        validate_topic_name("", name="cmd")


def test_sanitize_float():
    assert sanitize_float("0.25") == 0.25
    assert sanitize_float(0.0) == 0.0
    assert sanitize_float(None, default=1.0) == 1.0
    assert sanitize_float("bad", default=2.0) == 2.0
    assert sanitize_float(nan, default=3.0) == 3.0
    assert sanitize_float(inf, default=4.0) == 4.0
    assert sanitize_float(-inf, default=5.0) == 5.0


def test_validate_limits():
    assert validate_limits(max_vx=0.2, max_vy=0.2, max_wz=0.5) == (
        0.2,
        0.2,
        0.5,
    )

    with pytest.raises(ValueError):
        validate_limits(max_vx=0.0, max_vy=0.2, max_wz=0.5)

    with pytest.raises(ValueError):
        validate_limits(max_vx=0.2, max_vy=-0.2, max_wz=0.5)

    with pytest.raises(ValueError):
        validate_limits(max_vx=0.2, max_vy=0.2, max_wz="bad")


def test_validate_rate():
    assert validate_rate(30.0) == 30.0
    assert validate_rate("20.0") == 20.0

    with pytest.raises(ValueError):
        validate_rate(0.0)

    with pytest.raises(ValueError):
        validate_rate(-1.0)


def test_validate_timeout():
    assert validate_timeout(0.0) == 0.0
    assert validate_timeout(0.5) == 0.5
    assert validate_timeout("1.0") == 1.0

    with pytest.raises(ValueError):
        validate_timeout(-0.1)

    with pytest.raises(ValueError):
        validate_timeout("bad")
