#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for pure control-status node helpers."""

from __future__ import annotations

from savo_control.models import TwistCommand
from savo_control.nodes.control_status_helpers import (
    NONZERO_EPS,
    BoolSample,
    CommandSample,
    OdomSample,
    ScalarSample,
    TextSample,
)


def test_nonzero_eps_is_small_positive_value():
    assert NONZERO_EPS > 0.0
    assert NONZERO_EPS < 1.0e-2


def test_command_sample_default_is_stale_and_not_moving():
    sample = CommandSample()

    assert sample.stamp_s is None
    assert sample.fresh(now_s=1.0, timeout_s=0.5) is False
    assert sample.moving() is False


def test_command_sample_fresh():
    sample = CommandSample(
        command=TwistCommand(vx=0.1, source="manual"),
        stamp_s=1.0,
    )

    assert sample.fresh(now_s=1.2, timeout_s=0.5) is True
    assert sample.fresh(now_s=1.6, timeout_s=0.5) is False


def test_command_sample_moving_forward():
    sample = CommandSample(
        command=TwistCommand(vx=NONZERO_EPS * 2.0),
        stamp_s=1.0,
    )

    assert sample.moving() is True


def test_command_sample_moving_strafe():
    sample = CommandSample(
        command=TwistCommand(vy=NONZERO_EPS * 2.0),
        stamp_s=1.0,
    )

    assert sample.moving() is True


def test_command_sample_moving_rotate():
    sample = CommandSample(
        command=TwistCommand(wz=NONZERO_EPS * 2.0),
        stamp_s=1.0,
    )

    assert sample.moving() is True


def test_command_sample_not_moving_below_threshold():
    sample = CommandSample(
        command=TwistCommand(
            vx=NONZERO_EPS * 0.5,
            vy=NONZERO_EPS * 0.5,
            wz=NONZERO_EPS * 0.5,
        ),
        stamp_s=1.0,
    )

    assert sample.moving() is False


def test_command_sample_moving_sanitizes_invalid_values():
    moving = CommandSample(
        command=TwistCommand(vx=float("nan"), vy=0.1, wz=float("inf")),
        stamp_s=1.0,
    )
    stopped = CommandSample(
        command=TwistCommand(vx=float("nan"), vy=0.0, wz=float("inf")),
        stamp_s=1.0,
    )

    assert moving.moving() is True
    assert stopped.moving() is False


def test_bool_sample_default_is_stale():
    sample = BoolSample()

    assert sample.value is None
    assert sample.stamp_s is None
    assert sample.fresh(now_s=1.0, timeout_s=0.5) is False


def test_bool_sample_freshness():
    sample = BoolSample(value=True, stamp_s=1.0)

    assert sample.value is True
    assert sample.fresh(now_s=1.2, timeout_s=0.5) is True
    assert sample.fresh(now_s=1.6, timeout_s=0.5) is False


def test_scalar_sample_default_is_stale():
    sample = ScalarSample()

    assert sample.value is None
    assert sample.stamp_s is None
    assert sample.fresh(now_s=1.0, timeout_s=0.5) is False


def test_scalar_sample_freshness():
    sample = ScalarSample(value=0.75, stamp_s=1.0)

    assert sample.value == 0.75
    assert sample.fresh(now_s=1.2, timeout_s=0.5) is True
    assert sample.fresh(now_s=1.6, timeout_s=0.5) is False


def test_text_sample_default_is_stale():
    sample = TextSample()

    assert sample.value == ""
    assert sample.stamp_s is None
    assert sample.fresh(now_s=1.0, timeout_s=0.5) is False


def test_text_sample_freshness():
    sample = TextSample(value="AUTO", stamp_s=1.0)

    assert sample.value == "AUTO"
    assert sample.fresh(now_s=1.2, timeout_s=0.5) is True
    assert sample.fresh(now_s=1.6, timeout_s=0.5) is False


def test_odom_sample_default_is_stale():
    sample = OdomSample()

    assert sample.linear_speed == 0.0
    assert sample.angular_speed == 0.0
    assert sample.stamp_s is None
    assert sample.fresh(now_s=1.0, timeout_s=0.5) is False


def test_odom_sample_freshness():
    sample = OdomSample(
        linear_speed=0.1,
        angular_speed=0.2,
        stamp_s=1.0,
    )

    assert sample.linear_speed == 0.1
    assert sample.angular_speed == 0.2
    assert sample.fresh(now_s=1.2, timeout_s=0.5) is True
    assert sample.fresh(now_s=1.6, timeout_s=0.5) is False


def test_boundary_freshness_is_inclusive():
    cmd = CommandSample(command=TwistCommand(vx=0.1), stamp_s=1.0)
    bool_sample = BoolSample(value=True, stamp_s=1.0)
    scalar = ScalarSample(value=0.5, stamp_s=1.0)
    text = TextSample(value="AUTO", stamp_s=1.0)
    odom = OdomSample(linear_speed=0.1, angular_speed=0.2, stamp_s=1.0)

    assert cmd.fresh(now_s=1.5, timeout_s=0.5) is True
    assert bool_sample.fresh(now_s=1.5, timeout_s=0.5) is True
    assert scalar.fresh(now_s=1.5, timeout_s=0.5) is True
    assert text.fresh(now_s=1.5, timeout_s=0.5) is True
    assert odom.fresh(now_s=1.5, timeout_s=0.5) is True


def test_negative_age_is_treated_as_fresh():
    sample = CommandSample(command=TwistCommand(vx=0.1), stamp_s=2.0)

    assert sample.fresh(now_s=1.0, timeout_s=0.5) is True
