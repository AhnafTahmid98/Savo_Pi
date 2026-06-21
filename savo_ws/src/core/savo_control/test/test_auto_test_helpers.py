#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for pure auto-test manager helpers."""

from __future__ import annotations

from math import isclose

from savo_control.models import TwistCommand
from savo_control.nodes.auto_test_helpers import (
    AUTO_TEST_SOURCE,
    ActiveAutoTest,
    AutoTestLimits,
    AutoTestProfile,
    AutoTestState,
    AutoTestStepResult,
    TestStage as AutoTestStage,
    command_from_values,
    constant_profile,
    default_auto_tests,
    normalize_test_name,
    profile_from_mapping,
    pulse_profile,
    start_active_test,
    status_text,
    step_active_test,
    stop_command,
)


def assert_close(value: float, expected: float) -> None:
    assert isclose(value, expected, abs_tol=1e-9)


def test_auto_test_source_is_canonical():
    assert AUTO_TEST_SOURCE == "auto_test"


def test_auto_test_state_values_are_canonical():
    assert [state.value for state in AutoTestState] == [
        "IDLE",
        "RUNNING",
        "STOPPING",
        "FINISHED",
        "UNKNOWN_TEST",
        "SAFETY_STOP",
        "TIMEOUT",
        "DISABLED",
    ]


def test_auto_test_limits_defaults():
    limits = AutoTestLimits()

    assert limits.max_vx == 0.18
    assert limits.max_vy == 0.18
    assert limits.max_wz == 0.45
    assert limits.low_speed_scale == 0.50


def test_auto_test_limits_sanitized():
    limits = AutoTestLimits(
        max_vx=-0.20,
        max_vy=float("nan"),
        max_wz=-0.60,
        low_speed_scale=2.0,
    ).sanitized()

    assert limits.max_vx == 0.20
    assert limits.max_vy == 0.0
    assert limits.max_wz == 0.60
    assert limits.low_speed_scale == 1.0


def test_normalize_test_name():
    assert normalize_test_name(" forward_slow ") == "forward_slow"
    assert normalize_test_name(None) == ""


def test_stop_command():
    cmd = stop_command(stamp_sec=2.0)

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == AUTO_TEST_SOURCE
    assert cmd.stamp_sec == 2.0


def test_command_from_values_clamps_and_sanitizes():
    limits = AutoTestLimits(max_vx=0.10, max_vy=0.11, max_wz=0.30).sanitized()

    cmd = command_from_values(
        vx=1.0,
        vy=-1.0,
        wz=1.0,
        limits=limits,
        stamp_sec=3.0,
    )

    assert cmd.vx == 0.10
    assert cmd.vy == -0.11
    assert cmd.wz == 0.30
    assert cmd.source == AUTO_TEST_SOURCE
    assert cmd.stamp_sec == 3.0


def test_command_from_values_handles_invalid_values():
    limits = AutoTestLimits(max_vx=0.10, max_vy=0.11, max_wz=0.30).sanitized()

    cmd = command_from_values(
        vx=float("nan"),
        vy=float("inf"),
        wz=-float("inf"),
        limits=limits,
    )

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0


def test_test_stage_sanitized():
    limits = AutoTestLimits(max_vx=0.10, max_vy=0.10, max_wz=0.25).sanitized()
    stage = AutoTestStage(
        name="forward",
        duration_s=-1.0,
        command=TwistCommand(vx=1.0, vy=-1.0, wz=1.0, source="raw"),
    )

    safe = stage.sanitized(limits)

    assert safe.name == "forward"
    assert safe.duration_s == 0.0
    assert safe.command.vx == 0.10
    assert safe.command.vy == -0.10
    assert safe.command.wz == 0.25


def test_constant_profile():
    limits = AutoTestLimits().sanitized()

    profile = constant_profile(
        name="forward_slow",
        duration_s=2.0,
        vx=0.10,
        vy=0.0,
        wz=0.0,
        limits=limits,
    )

    assert profile.name == "forward_slow"
    assert profile.test_type == "constant_twist"
    assert len(profile.stages) == 1
    assert profile.stages[0].name == "forward_slow"
    assert profile.stages[0].duration_s == 2.0
    assert profile.stages[0].command.vx == 0.10
    assert profile.total_duration_s() == 2.0
    assert profile.empty() is False


def test_pulse_profile():
    limits = AutoTestLimits().sanitized()

    profile = pulse_profile(
        name="forward_pulse",
        pulses=3,
        active_s=0.6,
        rest_s=0.8,
        vx=0.10,
        vy=0.0,
        wz=0.0,
        limits=limits,
    )

    assert profile.name == "forward_pulse"
    assert profile.test_type == "pulse_twist"
    assert len(profile.stages) == 1
    assert profile.pulses == 3
    assert profile.active_s == 0.6
    assert profile.rest_s == 0.8
    assert profile.stages[0].command.vx == 0.10
    assert_close(profile.total_duration_s(), 4.2)
    assert profile.empty() is False


def test_profile_sanitized():
    limits = AutoTestLimits(max_vx=0.10, max_vy=0.10, max_wz=0.25).sanitized()

    profile = AutoTestProfile(
        name="raw",
        test_type="constant_twist",
        stages=(
            AutoTestStage(
                name="raw_stage",
                duration_s=-1.0,
                command=TwistCommand(vx=1.0, vy=1.0, wz=1.0),
            ),
        ),
        pulses=-2,
        active_s=-1.0,
        rest_s=-1.0,
    ).sanitized(limits)

    assert profile.name == "raw"
    assert profile.pulses == 0
    assert profile.active_s == 0.0
    assert profile.rest_s == 0.0
    assert profile.stages[0].duration_s == 0.0
    assert profile.stages[0].command.vx == 0.10
    assert profile.stages[0].command.vy == 0.10
    assert profile.stages[0].command.wz == 0.25


def test_empty_profile():
    profile = AutoTestProfile(
        name="empty",
        test_type="constant_twist",
        stages=(),
    )

    assert profile.empty() is True
    assert profile.total_duration_s() == 0.0


def test_default_auto_tests_contains_expected_profiles():
    tests = default_auto_tests(AutoTestLimits())

    for name in [
        "forward_slow",
        "backward_slow",
        "strafe_left_slow",
        "strafe_right_slow",
        "rotate_ccw_slow",
        "rotate_cw_slow",
        "forward_pulse",
        "rotate_pulse",
    ]:
        assert name in tests

    assert tests["forward_slow"].stages[0].command.vx > 0.0
    assert tests["backward_slow"].stages[0].command.vx < 0.0
    assert tests["strafe_left_slow"].stages[0].command.vy > 0.0
    assert tests["strafe_right_slow"].stages[0].command.vy < 0.0
    assert tests["rotate_ccw_slow"].stages[0].command.wz > 0.0
    assert tests["rotate_cw_slow"].stages[0].command.wz < 0.0


def test_profile_from_mapping_constant():
    profile = profile_from_mapping(
        "custom_forward",
        {
            "type": "constant_twist",
            "duration_s": 1.5,
            "vx": 0.12,
            "vy": 0.0,
            "wz": 0.0,
        },
        limits=AutoTestLimits(),
    )

    assert profile.name == "custom_forward"
    assert profile.test_type == "constant_twist"
    assert profile.stages[0].duration_s == 1.5
    assert profile.stages[0].command.vx == 0.12


def test_profile_from_mapping_pulse():
    profile = profile_from_mapping(
        "custom_pulse",
        {
            "type": "pulse_twist",
            "pulses": 2,
            "active_s": 0.4,
            "rest_s": 0.6,
            "vx": 0.10,
            "vy": 0.0,
            "wz": 0.0,
        },
        limits=AutoTestLimits(),
    )

    assert profile.name == "custom_pulse"
    assert profile.test_type == "pulse_twist"
    assert profile.pulses == 2
    assert profile.active_s == 0.4
    assert profile.rest_s == 0.6
    assert profile.stages[0].command.vx == 0.10


def test_start_active_test():
    profile = default_auto_tests()["forward_slow"]

    active = start_active_test(profile, now_s=10.0)

    assert isinstance(active, ActiveAutoTest)
    assert active.profile == profile
    assert active.start_s == 10.0
    assert active.stage_index == 0
    assert active.pulses_done == 0
    assert active.pulse_active is True
    assert active.last_pulse_switch_s == 10.0
    assert active.elapsed_s(now_s=10.5) == 0.5


def test_active_elapsed_does_not_go_negative():
    profile = default_auto_tests()["forward_slow"]
    active = start_active_test(profile, now_s=10.0)

    assert active.elapsed_s(now_s=9.0) == 0.0


def test_step_constant_test_running():
    profile = default_auto_tests()["forward_slow"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.5, max_duration_s=10.0)

    assert isinstance(result, AutoTestStepResult)
    assert result.state == AutoTestState.RUNNING
    assert result.command.vx > 0.0
    assert result.active_test == active
    assert result.finished is False
    assert result.reason == "forward_slow"


def test_step_constant_test_finished():
    profile = default_auto_tests()["forward_slow"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=3.0, max_duration_s=10.0)

    assert result.state == AutoTestState.FINISHED
    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.wz == 0.0
    assert result.active_test is None
    assert result.finished is True
    assert result.reason == "finished"


def test_step_empty_profile_finished():
    profile = AutoTestProfile(
        name="empty",
        test_type="constant_twist",
        stages=(),
    )
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.1, max_duration_s=10.0)

    assert result.state == AutoTestState.FINISHED
    assert result.finished is True
    assert result.reason == "empty_profile"
    assert result.command.vx == 0.0


def test_step_timeout():
    profile = default_auto_tests()["forward_slow"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=20.0, max_duration_s=5.0)

    assert result.state == AutoTestState.TIMEOUT
    assert result.finished is True
    assert result.active_test is None
    assert result.reason == "timeout"
    assert result.command.vx == 0.0


def test_step_multistage_constant_test_advances_stage():
    limits = AutoTestLimits().sanitized()
    profile = AutoTestProfile(
        name="multi",
        test_type="constant_twist",
        stages=(
            AutoTestStage(
                name="first",
                duration_s=1.0,
                command=command_from_values(vx=0.10, limits=limits),
            ),
            AutoTestStage(
                name="second",
                duration_s=1.0,
                command=command_from_values(vx=-0.10, limits=limits),
            ),
        ),
    )
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=2.0, max_duration_s=10.0)

    assert result.state == AutoTestState.RUNNING
    assert result.finished is False
    assert result.active_test is not None
    assert result.active_test.stage_index == 1
    assert result.command.vx < 0.0
    assert result.reason == "second"


def test_step_pulse_active():
    profile = default_auto_tests()["forward_pulse"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.2, max_duration_s=10.0)

    assert result.state == AutoTestState.RUNNING
    assert result.finished is False
    assert result.active_test == active
    assert result.command.vx > 0.0
    assert result.reason == "pulse_active"


def test_step_pulse_switches_to_rest():
    profile = default_auto_tests()["forward_pulse"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.7, max_duration_s=10.0)

    assert result.state == AutoTestState.RUNNING
    assert result.finished is False
    assert result.active_test is not None
    assert result.active_test.pulses_done == 1
    assert result.active_test.pulse_active is False
    assert result.command.vx == 0.0
    assert result.reason == "pulse_rest"


def test_step_pulse_switches_back_to_active():
    profile = default_auto_tests()["forward_pulse"]
    active = start_active_test(profile, now_s=1.0)

    rest = step_active_test(active, now_s=1.7, max_duration_s=10.0)
    result = step_active_test(rest.active_test, now_s=2.6, max_duration_s=10.0)

    assert result.state == AutoTestState.RUNNING
    assert result.finished is False
    assert result.active_test is not None
    assert result.active_test.pulse_active is True
    assert result.command.vx > 0.0
    assert result.reason == "pulse_active"


def test_step_pulse_finishes_after_last_active_period():
    profile = pulse_profile(
        name="one_pulse",
        pulses=1,
        active_s=0.5,
        rest_s=0.5,
        vx=0.10,
        vy=0.0,
        wz=0.0,
        limits=AutoTestLimits(),
    )
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.6, max_duration_s=10.0)

    assert result.state == AutoTestState.FINISHED
    assert result.finished is True
    assert result.active_test is None
    assert result.command.vx == 0.0
    assert result.reason == "finished"


def test_step_pulse_no_pulses_finishes():
    profile = pulse_profile(
        name="no_pulse",
        pulses=0,
        active_s=0.5,
        rest_s=0.5,
        vx=0.10,
        vy=0.0,
        wz=0.0,
        limits=AutoTestLimits(),
    )
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.1, max_duration_s=10.0)

    assert result.state == AutoTestState.FINISHED
    assert result.finished is True
    assert result.active_test is None
    assert result.reason == "no_pulses"


def test_status_text():
    text = status_text(
        state=AutoTestState.RUNNING,
        active_name="forward_slow",
        reason="tracking",
        enabled=True,
        safety_stop=False,
        command=TwistCommand(vx=0.10, source=AUTO_TEST_SOURCE),
    )

    assert "state=RUNNING" in text
    assert "enabled=true" in text
    assert "active_test=forward_slow" in text
    assert "reason=tracking" in text
    assert "safety_stop=false" in text
    assert "vx=0.100" in text
    assert "vy=0.000" in text
    assert "wz=0.000" in text


def test_status_text_defaults():
    text = status_text(
        state=AutoTestState.IDLE,
    )

    assert "state=IDLE" in text
    assert "enabled=true" in text
    assert "active_test=none" in text
    assert "safety_stop=false" in text
    assert "vx=0.000" in text
