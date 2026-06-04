#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for PID, HeadingPID, and DistancePID: gains, limits, anti-windup, reset, sign conventions."""

from __future__ import annotations

import math

import pytest

from savo_control.controllers.distance_pid_py import DistancePID
from savo_control.controllers.heading_pid_py import HeadingPID
from savo_control.controllers.pid_py import PID


def _maybe_call(obj, method_names: list[str], *args, **kwargs):
    """
    Call the first method that exists on obj.

    This makes the tests tolerant to slightly different method names while still
    checking the actual controller behavior.

    Example supported names:
      update()
      compute()
      step()
    """
    for name in method_names:
        if hasattr(obj, name):
            method = getattr(obj, name)
            return method(*args, **kwargs)

    raise AttributeError(
        f"{type(obj).__name__} has none of these methods: {method_names}"
    )


def _make_pid(**kwargs):
    """
    Create PID with common parameter-name fallbacks.

    This supports both simple and explicit constructor styles.
    """
    try:
        return PID(**kwargs)
    except TypeError:
        # Fallback for constructors that use output_min/output_max instead of
        # min_output/max_output.
        mapped = dict(kwargs)
        if "min_output" in mapped:
            mapped["output_min"] = mapped.pop("min_output")
        if "max_output" in mapped:
            mapped["output_max"] = mapped.pop("max_output")
        try:
            return PID(**mapped)
        except TypeError:
            # Very simple fallback.
            return PID(
                kwargs.get("kp", 1.0),
                kwargs.get("ki", 0.0),
                kwargs.get("kd", 0.0),
            )


def _compute_pid(pid, error: float, dt: float = 0.1) -> float:
    """
    Compute PID output using common method-name and signature fallbacks.
    """
    # Most expected style: update(error, dt)
    for method_name in ("update", "compute", "step"):
        if not hasattr(pid, method_name):
            continue

        method = getattr(pid, method_name)

        try:
            return float(method(error, dt))
        except TypeError:
            pass

        try:
            return float(method(error=error, dt=dt))
        except TypeError:
            pass

        try:
            return float(method(error))
        except TypeError:
            pass

    raise AttributeError(f"Could not compute PID output for {type(pid).__name__}")


def _reset_pid(pid) -> None:
    for name in ("reset", "clear"):
        if hasattr(pid, name):
            getattr(pid, name)()
            return


def test_generic_pid_p_only_positive_error() -> None:
    pid = _make_pid(kp=2.0, ki=0.0, kd=0.0)

    output = _compute_pid(pid, error=0.5, dt=0.1)

    assert output == pytest.approx(1.0)


def test_generic_pid_p_only_negative_error() -> None:
    pid = _make_pid(kp=2.0, ki=0.0, kd=0.0)

    output = _compute_pid(pid, error=-0.5, dt=0.1)

    assert output == pytest.approx(-1.0)


def test_generic_pid_zero_error_outputs_zero_for_p_only() -> None:
    pid = _make_pid(kp=2.0, ki=0.0, kd=0.0)

    output = _compute_pid(pid, error=0.0, dt=0.1)

    assert output == pytest.approx(0.0)


def test_generic_pid_output_limit_positive() -> None:
    pid = _make_pid(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        min_output=-1.0,
        max_output=1.0,
    )

    output = _compute_pid(pid, error=10.0, dt=0.1)

    assert output <= 1.0
    assert output == pytest.approx(1.0)


def test_generic_pid_output_limit_negative() -> None:
    pid = _make_pid(
        kp=10.0,
        ki=0.0,
        kd=0.0,
        min_output=-1.0,
        max_output=1.0,
    )

    output = _compute_pid(pid, error=-10.0, dt=0.1)

    assert output >= -1.0
    assert output == pytest.approx(-1.0)


def test_generic_pid_integral_accumulates() -> None:
    pid = _make_pid(kp=0.0, ki=1.0, kd=0.0)

    output_1 = _compute_pid(pid, error=1.0, dt=0.1)
    output_2 = _compute_pid(pid, error=1.0, dt=0.1)
    output_3 = _compute_pid(pid, error=1.0, dt=0.1)

    assert output_2 > output_1
    assert output_3 > output_2


def test_generic_pid_integral_limit_if_supported() -> None:
    pid = _make_pid(
        kp=0.0,
        ki=1.0,
        kd=0.0,
        integral_limit=0.2,
        min_output=-10.0,
        max_output=10.0,
    )

    outputs = [_compute_pid(pid, error=1.0, dt=0.1) for _ in range(20)]

    # If integral_limit is implemented, it should not grow forever.
    # Expected max around 0.2. If constructor ignored integral_limit, this test
    # may fail and we should align PID implementation.
    assert max(outputs) <= pytest.approx(0.25)


def test_generic_pid_reset_clears_integral_effect() -> None:
    pid = _make_pid(kp=0.0, ki=1.0, kd=0.0)

    for _ in range(5):
        _compute_pid(pid, error=1.0, dt=0.1)

    before_reset = _compute_pid(pid, error=1.0, dt=0.1)

    _reset_pid(pid)

    after_reset = _compute_pid(pid, error=1.0, dt=0.1)

    assert after_reset < before_reset


def test_generic_pid_derivative_response() -> None:
    pid = _make_pid(kp=0.0, ki=0.0, kd=1.0)

    first = _compute_pid(pid, error=0.0, dt=0.1)
    second = _compute_pid(pid, error=1.0, dt=0.1)

    assert first == pytest.approx(0.0)
    assert second > 0.0


def test_generic_pid_rejects_or_handles_zero_dt() -> None:
    pid = _make_pid(kp=1.0, ki=1.0, kd=1.0)

    output = _compute_pid(pid, error=1.0, dt=0.0)

    assert math.isfinite(output)


def test_heading_pid_zero_error_outputs_zero() -> None:
    pid = HeadingPID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=0.0,
        target_heading=0.0,
        dt=0.1,
    )

    assert float(output) == pytest.approx(0.0)


def test_heading_pid_positive_shortest_error() -> None:
    pid = HeadingPID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=0.0,
        target_heading=math.pi / 2.0,
        dt=0.1,
    )

    assert float(output) > 0.0


def test_heading_pid_negative_shortest_error() -> None:
    pid = HeadingPID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=0.0,
        target_heading=-math.pi / 2.0,
        dt=0.1,
    )

    assert float(output) < 0.0


def test_heading_pid_wraparound_positive_direction() -> None:
    # From +170 deg to -170 deg should rotate +20 deg.
    pid = HeadingPID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=math.radians(170.0),
        target_heading=math.radians(-170.0),
        dt=0.1,
    )

    assert float(output) > 0.0
    assert abs(float(output)) == pytest.approx(math.radians(20.0), rel=0.2)


def test_heading_pid_wraparound_negative_direction() -> None:
    # From -170 deg to +170 deg should rotate -20 deg.
    pid = HeadingPID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=math.radians(-170.0),
        target_heading=math.radians(170.0),
        dt=0.1,
    )

    assert float(output) < 0.0
    assert abs(float(output)) == pytest.approx(math.radians(20.0), rel=0.2)


def test_heading_pid_output_limit_if_supported() -> None:
    try:
        pid = HeadingPID(kp=10.0, ki=0.0, kd=0.0, max_output=0.5)
    except TypeError:
        try:
            pid = HeadingPID(kp=10.0, ki=0.0, kd=0.0, max_wz=0.5)
        except TypeError:
            pytest.skip("HeadingPID does not expose output limit constructor")

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_heading=0.0,
        target_heading=math.pi,
        dt=0.1,
    )

    assert abs(float(output)) <= 0.5 + 1.0e-9


def test_distance_pid_zero_error_outputs_zero() -> None:
    pid = DistancePID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_distance=0.60,
        target_distance=0.60,
        dt=0.1,
    )

    assert float(output) == pytest.approx(0.0)


def test_distance_pid_too_far_outputs_positive_forward_command() -> None:
    # Error convention should be measured - target.
    # If current distance is larger than target, robot should move forward.
    pid = DistancePID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_distance=1.00,
        target_distance=0.60,
        dt=0.1,
    )

    assert float(output) > 0.0


def test_distance_pid_too_close_outputs_negative_or_zero() -> None:
    # Depending on config, reverse may be disabled. So too-close output can be
    # negative or zero, but it should not be positive.
    pid = DistancePID(kp=1.0, ki=0.0, kd=0.0)

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_distance=0.40,
        target_distance=0.60,
        dt=0.1,
    )

    assert float(output) <= 0.0


def test_distance_pid_output_limit_if_supported() -> None:
    try:
        pid = DistancePID(kp=10.0, ki=0.0, kd=0.0, max_output=0.2)
    except TypeError:
        try:
            pid = DistancePID(kp=10.0, ki=0.0, kd=0.0, max_vx=0.2)
        except TypeError:
            pytest.skip("DistancePID does not expose output limit constructor")

    output = _maybe_call(
        pid,
        ["update", "compute", "step"],
        current_distance=2.00,
        target_distance=0.60,
        dt=0.1,
    )

    assert abs(float(output)) <= 0.2 + 1.0e-9


def test_pid_outputs_are_finite_for_normal_inputs() -> None:
    generic = _make_pid(kp=1.0, ki=0.1, kd=0.01)
    heading = HeadingPID(kp=1.0, ki=0.0, kd=0.0)
    distance = DistancePID(kp=1.0, ki=0.0, kd=0.0)

    out_generic = _compute_pid(generic, error=0.5, dt=0.1)

    out_heading = _maybe_call(
        heading,
        ["update", "compute", "step"],
        current_heading=0.0,
        target_heading=0.5,
        dt=0.1,
    )

    out_distance = _maybe_call(
        distance,
        ["update", "compute", "step"],
        current_distance=1.0,
        target_distance=0.6,
        dt=0.1,
    )

    assert math.isfinite(float(out_generic))
    assert math.isfinite(float(out_heading))
    assert math.isfinite(float(out_distance))