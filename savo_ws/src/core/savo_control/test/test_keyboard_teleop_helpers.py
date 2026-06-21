#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for pure keyboard teleop helpers."""

from __future__ import annotations

from math import isclose

from savo_control.models import TwistCommand
from savo_control.nodes.keyboard_teleop_helpers import (
    TELEOP_SOURCE,
    TeleopKeyResult,
    TeleopLimits,
    TeleopSpeeds,
    apply_teleop_key,
    make_teleop_command,
    normalize_key,
    stop_command,
)


def assert_close(value: float, expected: float) -> None:
    assert isclose(value, expected, abs_tol=1e-9)


def test_teleop_source_is_canonical():
    assert TELEOP_SOURCE == "keyboard_teleop"


def test_normalize_key():
    assert normalize_key("W") == "w"
    assert normalize_key(" q ") == " q "
    assert normalize_key(None) == ""
    assert normalize_key(123) == "123"


def test_default_speeds():
    speeds = TeleopSpeeds()

    assert speeds.linear == 0.12
    assert speeds.angular == 0.35


def test_speeds_sanitized():
    speeds = TeleopSpeeds(
        linear=-0.12,
        angular=float("nan"),
    ).sanitized()

    assert speeds.linear == 0.0
    assert speeds.angular == 0.0


def test_default_limits():
    limits = TeleopLimits()

    assert limits.default_linear == 0.12
    assert limits.default_angular == 0.35
    assert limits.max_linear == 0.25
    assert limits.max_angular == 0.60
    assert limits.linear_step == 0.02
    assert limits.angular_step == 0.05


def test_limits_sanitized_clamps_defaults():
    limits = TeleopLimits(
        default_linear=1.0,
        default_angular=2.0,
        max_linear=0.25,
        max_angular=0.60,
        linear_step=-0.02,
        angular_step=-0.05,
    ).sanitized()

    assert limits.default_linear == 0.25
    assert limits.default_angular == 0.60
    assert limits.max_linear == 0.25
    assert limits.max_angular == 0.60
    assert limits.linear_step == 0.02
    assert limits.angular_step == 0.05


def test_limits_sanitized_handles_invalid_max_values():
    limits = TeleopLimits(
        default_linear=0.12,
        default_angular=0.35,
        max_linear=float("nan"),
        max_angular=-0.60,
    ).sanitized()

    assert limits.max_linear == 1.0e-6
    assert limits.max_angular == 0.60
    assert limits.default_linear == 1.0e-6
    assert limits.default_angular == 0.35


def test_limits_default_speeds():
    speeds = TeleopLimits(
        default_linear=0.10,
        default_angular=0.30,
        max_linear=0.25,
        max_angular=0.60,
    ).default_speeds()

    assert speeds == TeleopSpeeds(linear=0.10, angular=0.30)


def test_key_result_defaults_are_safe():
    result = TeleopKeyResult()

    assert result.handled is False
    assert result.command is None
    assert result.speeds == TeleopSpeeds()
    assert result.speed_changed is False
    assert result.stop_requested is False
    assert result.help_requested is False
    assert result.label == ""


def test_make_teleop_command_clamps_values():
    limits = TeleopLimits(max_linear=0.20, max_angular=0.50).sanitized()

    cmd = make_teleop_command(
        vx=1.0,
        vy=-1.0,
        wz=2.0,
        limits=limits,
        stamp_sec=2.0,
    )

    assert isinstance(cmd, TwistCommand)
    assert cmd.source == TELEOP_SOURCE
    assert cmd.stamp_sec == 2.0
    assert cmd.vx == 0.20
    assert cmd.vy == -0.20
    assert cmd.wz == 0.50


def test_make_teleop_command_sanitizes_invalid_values():
    limits = TeleopLimits(max_linear=0.20, max_angular=0.50).sanitized()

    cmd = make_teleop_command(
        vx=float("nan"),
        vy=float("inf"),
        wz=-float("inf"),
        limits=limits,
    )

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0


def test_stop_command():
    cmd = stop_command(stamp_sec=3.0)

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == TELEOP_SOURCE
    assert cmd.stamp_sec == 3.0


def test_forward_key():
    result = apply_teleop_key(
        "w",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
        stamp_sec=1.0,
    )

    assert result.handled is True
    assert result.label == "forward"
    assert result.command is not None
    assert result.command.vx == 0.12
    assert result.command.vy == 0.0
    assert result.command.wz == 0.0
    assert result.command.stamp_sec == 1.0


def test_backward_key():
    result = apply_teleop_key(
        "s",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "backward"
    assert result.command.vx == -0.12
    assert result.command.vy == 0.0
    assert result.command.wz == 0.0


def test_strafe_left_key():
    result = apply_teleop_key(
        "a",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "strafe_left"
    assert result.command.vx == 0.0
    assert result.command.vy == 0.12
    assert result.command.wz == 0.0


def test_strafe_right_key():
    result = apply_teleop_key(
        "d",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "strafe_right"
    assert result.command.vx == 0.0
    assert result.command.vy == -0.12
    assert result.command.wz == 0.0


def test_rotate_left_key():
    result = apply_teleop_key(
        "q",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "rotate_left"
    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.wz == 0.35


def test_rotate_right_key():
    result = apply_teleop_key(
        "e",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "rotate_right"
    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.wz == -0.35


def test_uppercase_movement_key_is_accepted():
    result = apply_teleop_key(
        "W",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.label == "forward"
    assert result.command.vx == 0.12


def test_stop_keys():
    for key in ["x", " "]:
        result = apply_teleop_key(
            key,
            speeds=TeleopSpeeds(linear=0.12, angular=0.35),
            limits=TeleopLimits(),
            stamp_sec=5.0,
        )

        assert result.handled is True
        assert result.stop_requested is True
        assert result.label == "stop"
        assert result.command.vx == 0.0
        assert result.command.vy == 0.0
        assert result.command.wz == 0.0
        assert result.command.stamp_sec == 5.0


def test_linear_speed_up():
    result = apply_teleop_key(
        "t",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command is None
    assert result.speed_changed is True
    assert result.label == "linear_up"
    assert_close(result.speeds.linear, 0.14)
    assert result.speeds.angular == 0.35


def test_linear_speed_down():
    result = apply_teleop_key(
        "g",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command is None
    assert result.speed_changed is True
    assert result.label == "linear_down"
    assert_close(result.speeds.linear, 0.10)
    assert result.speeds.angular == 0.35


def test_angular_speed_up():
    result = apply_teleop_key(
        "y",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command is None
    assert result.speed_changed is True
    assert result.label == "angular_up"
    assert result.speeds.linear == 0.12
    assert_close(result.speeds.angular, 0.40)


def test_angular_speed_down():
    result = apply_teleop_key(
        "h",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command is None
    assert result.speed_changed is True
    assert result.label == "angular_down"
    assert result.speeds.linear == 0.12
    assert_close(result.speeds.angular, 0.30)


def test_speed_changes_are_clamped_to_limits():
    limits = TeleopLimits(
        default_linear=0.12,
        default_angular=0.35,
        max_linear=0.13,
        max_angular=0.37,
        linear_step=0.05,
        angular_step=0.05,
    ).sanitized()

    linear = apply_teleop_key(
        "t",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=limits,
    )
    angular = apply_teleop_key(
        "y",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=limits,
    )

    assert linear.speeds.linear == 0.13
    assert angular.speeds.angular == 0.37


def test_speed_down_is_clamped_to_zero():
    limits = TeleopLimits(linear_step=0.05, angular_step=0.10).sanitized()

    linear = apply_teleop_key(
        "g",
        speeds=TeleopSpeeds(linear=0.02, angular=0.05),
        limits=limits,
    )
    angular = apply_teleop_key(
        "h",
        speeds=TeleopSpeeds(linear=0.02, angular=0.05),
        limits=limits,
    )

    assert linear.speeds.linear == 0.0
    assert angular.speeds.angular == 0.0


def test_reset_speeds_key():
    limits = TeleopLimits(
        default_linear=0.12,
        default_angular=0.35,
        max_linear=0.25,
        max_angular=0.60,
    ).sanitized()

    result = apply_teleop_key(
        "r",
        speeds=TeleopSpeeds(linear=0.20, angular=0.50),
        limits=limits,
    )

    assert result.handled is True
    assert result.command is None
    assert result.speed_changed is True
    assert result.label == "reset_speeds"
    assert result.speeds == TeleopSpeeds(linear=0.12, angular=0.35)


def test_help_key():
    result = apply_teleop_key(
        "p",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command is None
    assert result.help_requested is True
    assert result.label == "help"


def test_ignored_key():
    result = apply_teleop_key(
        "z",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
    )

    assert result.handled is False
    assert result.command is None
    assert result.speed_changed is False
    assert result.stop_requested is False
    assert result.help_requested is False
    assert result.label == "ignored"


def test_speeds_are_sanitized_before_use():
    result = apply_teleop_key(
        "w",
        speeds=TeleopSpeeds(linear=float("nan"), angular=float("inf")),
        limits=TeleopLimits(),
    )

    assert result.handled is True
    assert result.command.vx == 0.0
    assert result.command.wz == 0.0


def test_command_uses_custom_source():
    result = apply_teleop_key(
        "w",
        speeds=TeleopSpeeds(linear=0.12, angular=0.35),
        limits=TeleopLimits(),
        stamp_sec=7.0,
        source="custom",
    )

    assert result.command.source == "custom"
    assert result.command.stamp_sec == 7.0
