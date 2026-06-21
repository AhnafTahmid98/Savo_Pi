#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for the pure Python TwistCommand model."""

from __future__ import annotations

from math import inf

from savo_control.models import TwistCommand


def test_zero_command():
    cmd = TwistCommand.zero()

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == "zero"
    assert cmd.stamp_sec == 0.0
    assert cmd.moving() is False


def test_zero_command_custom_metadata():
    cmd = TwistCommand.zero(source="timeout", stamp_sec=12.5)

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == "timeout"
    assert cmd.stamp_sec == 12.5


def test_finite():
    assert TwistCommand(vx=0.1, vy=0.2, wz=0.3).finite() is True
    assert TwistCommand(vx=float("nan"), vy=0.2, wz=0.3).finite() is False
    assert TwistCommand(vx=0.1, vy=inf, wz=0.3).finite() is False
    assert TwistCommand(vx=0.1, vy=0.2, wz=-inf).finite() is False


def test_sanitized_replaces_non_finite_values():
    cmd = TwistCommand(
        vx=float("nan"),
        vy=inf,
        wz=-inf,
        source="bad",
        stamp_sec=4.0,
    ).sanitized()

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == "bad"
    assert cmd.stamp_sec == 4.0


def test_clamp_positive_and_negative_values():
    cmd = TwistCommand(vx=0.5, vy=-0.4, wz=1.2, source="test")
    limited = cmd.clamp(max_vx=0.2, max_vy=0.15, max_wz=0.6)

    assert limited.vx == 0.2
    assert limited.vy == -0.15
    assert limited.wz == 0.6
    assert limited.source == "test"


def test_clamp_uses_absolute_limits():
    cmd = TwistCommand(vx=-0.5, vy=0.4, wz=-1.2)
    limited = cmd.clamp(max_vx=-0.2, max_vy=-0.15, max_wz=-0.6)

    assert limited.vx == -0.2
    assert limited.vy == 0.15
    assert limited.wz == -0.6


def test_with_deadband():
    cmd = TwistCommand(vx=0.004, vy=-0.006, wz=0.009)
    filtered = cmd.with_deadband(
        vx_deadband=0.005,
        vy_deadband=0.005,
        wz_deadband=0.010,
    )

    assert filtered.vx == 0.0
    assert filtered.vy == -0.006
    assert filtered.wz == 0.0


def test_with_deadband_uses_absolute_thresholds():
    cmd = TwistCommand(vx=-0.004, vy=0.006, wz=-0.009)
    filtered = cmd.with_deadband(
        vx_deadband=-0.005,
        vy_deadband=-0.005,
        wz_deadband=-0.010,
    )

    assert filtered.vx == 0.0
    assert filtered.vy == 0.006
    assert filtered.wz == 0.0


def test_linear_speed():
    assert TwistCommand(vx=3.0, vy=4.0).linear_speed() == 5.0
    assert TwistCommand.zero().linear_speed() == 0.0


def test_moving():
    assert TwistCommand(vx=0.01).moving(linear_eps=0.001) is True
    assert TwistCommand(vy=0.01).moving(linear_eps=0.001) is True
    assert TwistCommand(wz=0.01).moving(angular_eps=0.001) is True

    assert TwistCommand(vx=0.0001).moving(linear_eps=0.001) is False
    assert TwistCommand(wz=0.0001).moving(angular_eps=0.001) is False


def test_to_dict_sanitizes_values():
    data = TwistCommand(
        vx=float("nan"),
        vy=0.2,
        wz=0.3,
        source="dict",
        stamp_sec=7.0,
    ).to_dict()

    assert data == {
        "vx": 0.0,
        "vy": 0.2,
        "wz": 0.3,
        "source": "dict",
        "stamp_sec": 7.0,
    }


def test_status_text():
    text = TwistCommand(
        vx=0.1,
        vy=-0.2,
        wz=0.3,
        source="manual",
    ).status_text()

    assert text == "vx=0.100; vy=-0.200; wz=0.300; source=manual"


def test_status_text_uses_unknown_source_when_empty():
    text = TwistCommand(vx=0.1).status_text()

    assert text == "vx=0.100; vy=0.000; wz=0.000; source=unknown"
