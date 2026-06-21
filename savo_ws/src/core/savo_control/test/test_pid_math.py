#!/usr/bin/env python3
# -*- coding: utf-8 -*-

" Unit tests for the Python PID/controller helpers used by Robot Savo."

from __future__ import annotations

import math

import pytest

from savo_control.controllers.distance_pid_py import DistanceControllerConfig, DistancePid
from savo_control.controllers.heading_pid_py import HeadingController, HeadingControllerConfig
from savo_control.controllers.pid_py import Pid, PidConfig


def test_generic_pid_p_only_positive_error() -> None:
    """Positive error with P-only control should produce positive output."""
    pid = Pid(PidConfig(kp=2.0, ki=0.0, kd=0.0))

    result = pid.update(error=0.5, dt_sec=0.1)

    assert result.valid is True
    assert result.output == pytest.approx(1.0)


def test_generic_pid_p_only_negative_error() -> None:
    """Negative error with P-only control should produce negative output."""
    pid = Pid(PidConfig(kp=2.0, ki=0.0, kd=0.0))

    result = pid.update(error=-0.5, dt_sec=0.1)

    assert result.valid is True
    assert result.output == pytest.approx(-1.0)


def test_generic_pid_output_limits() -> None:
    """PID output must respect configured min/max limits."""
    pid = Pid(
        PidConfig(
            kp=10.0,
            ki=0.0,
            kd=0.0,
            output_min=-1.0,
            output_max=1.0,
        )
    )

    high = pid.update(error=10.0, dt_sec=0.1)
    low = pid.update(error=-10.0, dt_sec=0.1)

    assert high.output == pytest.approx(1.0)
    assert high.saturated is True

    assert low.output == pytest.approx(-1.0)
    assert low.saturated is True


def test_generic_pid_integral_accumulates_and_clamps() -> None:
    """Integral term should accumulate, but it must not exceed integral_clamp."""
    pid = Pid(
        PidConfig(
            kp=0.0,
            ki=1.0,
            kd=0.0,
            integral_clamp=0.2,
        )
    )

    outputs = [pid.update(error=1.0, dt_sec=0.1).output for _ in range(20)]

    assert outputs[1] > outputs[0]
    assert max(outputs) == pytest.approx(0.2)


def test_generic_pid_reset_clears_integral_effect() -> None:
    """After reset, old integral history should no longer affect the output."""
    pid = Pid(PidConfig(kp=0.0, ki=1.0, kd=0.0))

    for _ in range(5):
        pid.update(error=1.0, dt_sec=0.1)

    before_reset = pid.update(error=1.0, dt_sec=0.1).output

    pid.reset()
    after_reset = pid.update(error=1.0, dt_sec=0.1).output

    assert after_reset < before_reset
    assert after_reset == pytest.approx(0.1)


def test_generic_pid_derivative_response_after_first_sample() -> None:
    """
    Derivative term needs a previous sample.

    First update should not create a false derivative spike.
    Second update with changed error should create derivative response.
    """
    pid = Pid(PidConfig(kp=0.0, ki=0.0, kd=1.0))

    first = pid.update(error=0.0, dt_sec=0.1)
    second = pid.update(error=1.0, dt_sec=0.1)

    assert first.output == pytest.approx(0.0)
    assert second.output > 0.0


def test_generic_pid_zero_dt_is_safe_and_finite() -> None:
    """A zero dt should not crash or produce NaN/Inf output."""
    pid = Pid(PidConfig(kp=1.0, ki=1.0, kd=1.0))

    result = pid.update(error=1.0, dt_sec=0.0)

    assert math.isfinite(result.output)
    assert result.dt_valid is False
    assert result.valid is True


def test_heading_controller_zero_error_outputs_zero() -> None:
    """If current heading equals target heading, angular command should be zero."""
    controller = HeadingController(
        HeadingControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=0.0,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.wz_cmd_rad_s == pytest.approx(0.0)


def test_heading_controller_positive_error_outputs_positive() -> None:
    """Positive heading error should generate positive angular velocity."""
    controller = HeadingController(
        HeadingControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=math.pi / 2.0,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.wz_cmd_rad_s > 0.0


def test_heading_controller_negative_error_outputs_negative() -> None:
    """Negative heading error should generate negative angular velocity."""
    controller = HeadingController(
        HeadingControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=-math.pi / 2.0,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.wz_cmd_rad_s < 0.0


def test_heading_controller_wraparound_positive_direction() -> None:
    """
    Heading controller must use shortest angular distance.

    From +170 deg to -170 deg is a small positive turn of about +20 deg,
    not a large negative turn.
    """
    controller = HeadingController(
        HeadingControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_yaw_rad=math.radians(170.0),
        target_yaw_rad=math.radians(-170.0),
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.wz_cmd_rad_s > 0.0


def test_heading_controller_output_limit() -> None:
    """Angular command must respect max_wz_rad_s."""
    controller = HeadingController(
        HeadingControllerConfig(
            pid=PidConfig(kp=10.0, ki=0.0, kd=0.0),
            max_wz_rad_s=0.5,
        )
    )

    result = controller.update_to_target(
        current_yaw_rad=0.0,
        target_yaw_rad=math.pi,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert abs(result.wz_cmd_rad_s) <= 0.5 + 1.0e-9


def test_distance_pid_zero_error_outputs_zero() -> None:
    """If current distance equals target distance, linear command should be zero."""
    controller = DistancePid(
        DistanceControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_distance_m=0.60,
        target_distance_m=0.60,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.vx_cmd_m_s == pytest.approx(0.0)


def test_distance_pid_too_far_uses_current_convention() -> None:
    """
    Check the current production sign convention.

    With the current implementation:
    error = current_distance - target_distance

    If current distance is larger than target distance, error is positive.
    The controller therefore produces positive vx for approach motion.
    """
    controller = DistancePid(
        DistanceControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_distance_m=1.00,
        target_distance_m=0.60,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.error_distance_m == pytest.approx(0.40)
    assert result.vx_cmd_m_s > 0.0


def test_distance_pid_too_close_uses_current_convention() -> None:
    """
    Check the current production sign convention for being too close.

    If current distance is smaller than target distance, error is negative.
    The controller therefore produces negative vx before policy limiting.
    """
    controller = DistancePid(
        DistanceControllerConfig(
            pid=PidConfig(kp=1.0, ki=0.0, kd=0.0)
        )
    )

    result = controller.update_to_target(
        current_distance_m=0.40,
        target_distance_m=0.60,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert result.error_distance_m == pytest.approx(-0.20)
    assert result.vx_cmd_m_s < 0.0


def test_distance_pid_output_limit() -> None:
    """Distance controller output must respect max_vx_m_s."""
    controller = DistancePid(
        DistanceControllerConfig(
            pid=PidConfig(kp=10.0, ki=0.0, kd=0.0),
            max_vx_m_s=0.2,
        )
    )

    result = controller.update_to_target(
        current_distance_m=2.00,
        target_distance_m=0.60,
        dt_sec=0.1,
    )

    assert result.valid is True
    assert abs(result.vx_cmd_m_s) <= 0.2 + 1.0e-9