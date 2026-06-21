#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for pure recovery-test manager helpers."""

from __future__ import annotations

from math import isclose

from savo_control.models import TwistCommand
from savo_control.nodes.recovery_test_helpers import (
    RECOVERY_TEST_SOURCE,
    ActiveRecoveryTest,
    RecoveryTestLimits,
    RecoveryTestProfile,
    RecoveryTestStage,
    RecoveryTestState,
    RecoveryTestStepResult,
    backup_profile,
    backup_then_rotate_profile,
    command_from_values,
    default_recovery_tests,
    normalize_test_name,
    profile_from_mapping,
    request_only_profile,
    rotate_profile,
    start_active_test,
    status_text,
    step_active_test,
    stop_command,
)


def assert_close(value: float, expected: float) -> None:
    assert isclose(value, expected, abs_tol=1e-9)


def test_recovery_test_source_is_canonical():
    assert RECOVERY_TEST_SOURCE == "recovery_test"


def test_recovery_test_state_values_are_canonical():
    assert [state.value for state in RecoveryTestState] == [
        "IDLE",
        "RUNNING",
        "STOPPING",
        "FINISHED",
        "UNKNOWN_TEST",
        "SAFETY_STOP",
        "TIMEOUT",
        "DISABLED",
    ]


def test_recovery_test_limits_defaults():
    limits = RecoveryTestLimits()

    assert limits.max_backup_vx == 0.10
    assert limits.max_turn_wz == 0.35
    assert limits.max_duration_s == 8.0


def test_recovery_test_limits_sanitized():
    limits = RecoveryTestLimits(
        max_backup_vx=-0.20,
        max_turn_wz=-0.60,
        max_duration_s=-1.0,
    ).sanitized()

    assert limits.max_backup_vx == 0.20
    assert limits.max_turn_wz == 0.60
    assert limits.max_duration_s == 0.1


def test_normalize_test_name():
    assert normalize_test_name(" backup_short ") == "backup_short"
    assert normalize_test_name(None) == ""


def test_stop_command():
    cmd = stop_command(stamp_sec=2.0)

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == RECOVERY_TEST_SOURCE
    assert cmd.stamp_sec == 2.0


def test_command_from_values_clamps_and_sanitizes():
    limits = RecoveryTestLimits(
        max_backup_vx=0.10,
        max_turn_wz=0.30,
    ).sanitized()

    cmd = command_from_values(
        vx=-1.0,
        vy=1.0,
        wz=1.0,
        limits=limits,
        stamp_sec=3.0,
    )

    assert cmd.vx == -0.10
    assert cmd.vy == 0.10
    assert cmd.wz == 0.30
    assert cmd.source == RECOVERY_TEST_SOURCE
    assert cmd.stamp_sec == 3.0


def test_command_from_values_handles_invalid_values():
    limits = RecoveryTestLimits(
        max_backup_vx=0.10,
        max_turn_wz=0.30,
    ).sanitized()

    cmd = command_from_values(
        vx=float("nan"),
        vy=float("inf"),
        wz=-float("inf"),
        limits=limits,
    )

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0


def test_recovery_test_stage_sanitized():
    limits = RecoveryTestLimits(
        max_backup_vx=0.10,
        max_turn_wz=0.25,
    ).sanitized()

    stage = RecoveryTestStage(
        name="backup",
        duration_s=-1.0,
        command=TwistCommand(vx=-1.0, vy=1.0, wz=1.0, source="raw"),
        request_recovery=True,
    )

    safe = stage.sanitized(limits)

    assert safe.name == "backup"
    assert safe.duration_s == 0.0
    assert safe.command.vx == -0.10
    assert safe.command.vy == 0.10
    assert safe.command.wz == 0.25
    assert safe.request_recovery is True


def test_request_only_profile():
    profile = request_only_profile("request_only")

    assert profile.name == "request_only"
    assert len(profile.stages) == 1
    assert profile.stages[0].name == "request"
    assert profile.stages[0].duration_s == 0.5
    assert profile.stages[0].command.vx == 0.0
    assert profile.stages[0].request_recovery is True
    assert profile.total_duration_s() == 0.5
    assert profile.empty() is False


def test_backup_profile():
    profile = backup_profile(
        name="backup_short",
        duration_s=1.0,
        vx=-0.06,
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "backup_short"
    assert len(profile.stages) == 1
    assert profile.stages[0].name == "backup"
    assert profile.stages[0].duration_s == 1.0
    assert profile.stages[0].command.vx == -0.06
    assert profile.stages[0].request_recovery is True


def test_rotate_profile():
    profile = rotate_profile(
        name="rotate_left",
        duration_s=1.2,
        wz=0.25,
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "rotate_left"
    assert len(profile.stages) == 1
    assert profile.stages[0].name == "rotate"
    assert profile.stages[0].duration_s == 1.2
    assert profile.stages[0].command.wz == 0.25
    assert profile.stages[0].request_recovery is True


def test_backup_then_rotate_profile():
    profile = backup_then_rotate_profile(
        name="backup_then_left",
        backup_s=1.0,
        rotate_s=1.0,
        vx=-0.06,
        wz=0.25,
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "backup_then_left"
    assert len(profile.stages) == 2

    assert profile.stages[0].name == "backup"
    assert profile.stages[0].command.vx == -0.06

    assert profile.stages[1].name == "rotate"
    assert profile.stages[1].command.wz == 0.25

    assert profile.total_duration_s() == 2.0


def test_profile_sanitized():
    limits = RecoveryTestLimits(
        max_backup_vx=0.10,
        max_turn_wz=0.25,
    ).sanitized()

    profile = RecoveryTestProfile(
        name="raw",
        stages=(
            RecoveryTestStage(
                name="raw_stage",
                duration_s=-1.0,
                command=TwistCommand(vx=-1.0, vy=1.0, wz=1.0),
                request_recovery=True,
            ),
        ),
    ).sanitized(limits)

    assert profile.name == "raw"
    assert profile.stages[0].duration_s == 0.0
    assert profile.stages[0].command.vx == -0.10
    assert profile.stages[0].command.vy == 0.10
    assert profile.stages[0].command.wz == 0.25


def test_empty_profile():
    profile = RecoveryTestProfile(
        name="empty",
        stages=(),
    )

    assert profile.empty() is True
    assert profile.total_duration_s() == 0.0


def test_default_recovery_tests_contains_expected_profiles():
    tests = default_recovery_tests(RecoveryTestLimits())

    for name in [
        "request_only",
        "backup_short",
        "backup_long",
        "rotate_left",
        "rotate_right",
        "backup_then_left",
        "backup_then_right",
    ]:
        assert name in tests

    assert tests["request_only"].stages[0].request_recovery is True
    assert tests["backup_short"].stages[0].command.vx < 0.0
    assert tests["backup_long"].stages[0].command.vx < 0.0
    assert tests["rotate_left"].stages[0].command.wz > 0.0
    assert tests["rotate_right"].stages[0].command.wz < 0.0
    assert tests["backup_then_left"].stages[1].command.wz > 0.0
    assert tests["backup_then_right"].stages[1].command.wz < 0.0


def test_profile_from_mapping_request_only():
    profile = profile_from_mapping(
        "custom_request",
        {"type": "request_only"},
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "custom_request"
    assert profile.stages[0].request_recovery is True
    assert profile.stages[0].command.vx == 0.0


def test_profile_from_mapping_backup():
    profile = profile_from_mapping(
        "custom_backup",
        {
            "type": "backup",
            "duration_s": 1.5,
            "vx": -0.08,
        },
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "custom_backup"
    assert profile.stages[0].name == "backup"
    assert profile.stages[0].duration_s == 1.5
    assert profile.stages[0].command.vx == -0.08


def test_profile_from_mapping_rotate():
    profile = profile_from_mapping(
        "custom_rotate",
        {
            "type": "rotate",
            "duration_s": 1.3,
            "wz": -0.22,
        },
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "custom_rotate"
    assert profile.stages[0].name == "rotate"
    assert profile.stages[0].duration_s == 1.3
    assert profile.stages[0].command.wz == -0.22


def test_profile_from_mapping_backup_then_rotate():
    profile = profile_from_mapping(
        "custom_combo",
        {
            "type": "backup_then_rotate",
            "backup_s": 0.8,
            "rotate_s": 0.9,
            "vx": -0.07,
            "wz": 0.23,
        },
        limits=RecoveryTestLimits(),
    )

    assert profile.name == "custom_combo"
    assert len(profile.stages) == 2
    assert profile.stages[0].duration_s == 0.8
    assert profile.stages[0].command.vx == -0.07
    assert profile.stages[1].duration_s == 0.9
    assert profile.stages[1].command.wz == 0.23


def test_start_active_test():
    profile = default_recovery_tests()["backup_short"]

    active = start_active_test(profile, now_s=10.0)

    assert isinstance(active, ActiveRecoveryTest)
    assert active.profile == profile
    assert active.start_s == 10.0
    assert active.stage_start_s == 10.0
    assert active.stage_index == 0
    assert active.elapsed_s(now_s=10.5) == 0.5
    assert active.stage_elapsed_s(now_s=10.5) == 0.5


def test_active_elapsed_does_not_go_negative():
    profile = default_recovery_tests()["backup_short"]
    active = start_active_test(profile, now_s=10.0)

    assert active.elapsed_s(now_s=9.0) == 0.0
    assert active.stage_elapsed_s(now_s=9.0) == 0.0


def test_step_backup_test_running():
    profile = default_recovery_tests()["backup_short"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.5, max_duration_s=8.0)

    assert isinstance(result, RecoveryTestStepResult)
    assert result.state == RecoveryTestState.RUNNING
    assert result.command.vx < 0.0
    assert result.request_recovery is True
    assert result.active_test == active
    assert result.finished is False
    assert result.reason == "backup"


def test_step_backup_test_finished():
    profile = default_recovery_tests()["backup_short"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=2.0, max_duration_s=8.0)

    assert result.state == RecoveryTestState.FINISHED
    assert result.command.vx == 0.0
    assert result.command.vy == 0.0
    assert result.command.wz == 0.0
    assert result.request_recovery is False
    assert result.active_test is None
    assert result.finished is True
    assert result.reason == "finished"


def test_step_empty_profile_finished():
    profile = RecoveryTestProfile(
        name="empty",
        stages=(),
    )
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=1.1, max_duration_s=8.0)

    assert result.state == RecoveryTestState.FINISHED
    assert result.finished is True
    assert result.reason == "empty_profile"
    assert result.command.vx == 0.0
    assert result.request_recovery is False


def test_step_timeout():
    profile = default_recovery_tests()["backup_short"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=20.0, max_duration_s=5.0)

    assert result.state == RecoveryTestState.TIMEOUT
    assert result.finished is True
    assert result.active_test is None
    assert result.reason == "timeout"
    assert result.command.vx == 0.0
    assert result.request_recovery is False


def test_step_multistage_test_advances_stage():
    profile = default_recovery_tests()["backup_then_left"]
    active = start_active_test(profile, now_s=1.0)

    result = step_active_test(active, now_s=2.0, max_duration_s=8.0)

    assert result.state == RecoveryTestState.RUNNING
    assert result.finished is False
    assert result.active_test is not None
    assert result.active_test.stage_index == 1
    assert result.command.wz > 0.0
    assert result.request_recovery is True
    assert result.reason == "rotate"


def test_step_multistage_test_finishes_after_second_stage():
    profile = default_recovery_tests()["backup_then_left"]
    active = start_active_test(profile, now_s=1.0)

    second = step_active_test(active, now_s=2.0, max_duration_s=8.0)
    done = step_active_test(second.active_test, now_s=3.0, max_duration_s=8.0)

    assert done.state == RecoveryTestState.FINISHED
    assert done.finished is True
    assert done.active_test is None
    assert done.command.vx == 0.0
    assert done.request_recovery is False


def test_status_text():
    text = status_text(
        state=RecoveryTestState.RUNNING,
        active_name="backup_short",
        reason="backup",
        enabled=True,
        safety_stop=False,
        request_recovery=True,
        command=TwistCommand(vx=-0.06, source=RECOVERY_TEST_SOURCE),
    )

    assert "state=RUNNING" in text
    assert "enabled=true" in text
    assert "active_test=backup_short" in text
    assert "reason=backup" in text
    assert "safety_stop=false" in text
    assert "request_recovery=true" in text
    assert "vx=-0.060" in text
    assert "vy=0.000" in text
    assert "wz=0.000" in text


def test_status_text_defaults():
    text = status_text(
        state=RecoveryTestState.IDLE,
    )

    assert "state=IDLE" in text
    assert "enabled=true" in text
    assert "active_test=none" in text
    assert "safety_stop=false" in text
    assert "request_recovery=false" in text
    assert "vx=0.000" in text
