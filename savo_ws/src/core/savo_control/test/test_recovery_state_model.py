#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for recovery-state models."""

from __future__ import annotations

import pytest

from savo_control.models import (
    RECOVERY_ACTIVE_PHASES,
    RecoveryPhase,
    RecoveryState,
    TwistCommand,
    recovery_command,
)


def test_recovery_phase_values_are_canonical():
    assert RecoveryPhase.values() == (
        "IDLE",
        "REQUESTED",
        "BACKING_UP",
        "SETTLING",
        "TURNING",
        "DONE",
        "ABORTED",
    )


def test_recovery_phase_from_text_accepts_valid_inputs():
    assert RecoveryPhase.from_text("IDLE") == RecoveryPhase.IDLE
    assert RecoveryPhase.from_text("requested") == RecoveryPhase.REQUESTED
    assert RecoveryPhase.from_text(" backing_up ") == RecoveryPhase.BACKING_UP
    assert RecoveryPhase.from_text("settling") == RecoveryPhase.SETTLING
    assert RecoveryPhase.from_text("turning") == RecoveryPhase.TURNING
    assert RecoveryPhase.from_text("done") == RecoveryPhase.DONE
    assert RecoveryPhase.from_text("aborted") == RecoveryPhase.ABORTED


def test_recovery_phase_from_text_rejects_invalid_input():
    with pytest.raises(ValueError):
        RecoveryPhase.from_text("bad")


def test_recovery_phase_from_text_uses_default():
    assert (
        RecoveryPhase.from_text("bad", default=RecoveryPhase.IDLE)
        == RecoveryPhase.IDLE
    )
    assert (
        RecoveryPhase.from_text(None, default=RecoveryPhase.ABORTED)
        == RecoveryPhase.ABORTED
    )


def test_active_phases():
    assert RecoveryPhase.REQUESTED in RECOVERY_ACTIVE_PHASES
    assert RecoveryPhase.BACKING_UP in RECOVERY_ACTIVE_PHASES
    assert RecoveryPhase.SETTLING in RECOVERY_ACTIVE_PHASES
    assert RecoveryPhase.TURNING in RECOVERY_ACTIVE_PHASES

    assert RecoveryPhase.IDLE not in RECOVERY_ACTIVE_PHASES
    assert RecoveryPhase.DONE not in RECOVERY_ACTIVE_PHASES
    assert RecoveryPhase.ABORTED not in RECOVERY_ACTIVE_PHASES


def test_idle_state():
    state = RecoveryState.idle()

    assert state.phase == RecoveryPhase.IDLE
    assert state.active is False
    assert state.request is False
    assert state.attempt == 0
    assert state.max_attempts == 3
    assert state.command.vx == 0.0
    assert state.running() is False
    assert state.finished() is False
    assert state.can_retry() is True


def test_requested_state():
    state = RecoveryState.requested(reason="stuck")

    assert state.phase == RecoveryPhase.REQUESTED
    assert state.active is True
    assert state.request is True
    assert state.reason == "stuck"
    assert state.running() is True
    assert state.finished() is False


def test_done_state():
    state = RecoveryState.done(reason="completed")

    assert state.phase == RecoveryPhase.DONE
    assert state.active is False
    assert state.request is False
    assert state.command.vx == 0.0
    assert state.command.source == "recovery_done"
    assert state.reason == "completed"
    assert state.running() is False
    assert state.finished() is True


def test_aborted_state():
    state = RecoveryState.aborted(reason="safety_stop")

    assert state.phase == RecoveryPhase.ABORTED
    assert state.active is False
    assert state.request is False
    assert state.command.vx == 0.0
    assert state.command.source == "recovery_aborted"
    assert state.reason == "safety_stop"
    assert state.running() is False
    assert state.finished() is True


def test_running_requires_active_phase_and_active_true():
    backing = RecoveryState(
        phase=RecoveryPhase.BACKING_UP,
        active=True,
    )
    inactive_backing = RecoveryState(
        phase=RecoveryPhase.BACKING_UP,
        active=False,
    )
    done = RecoveryState(
        phase=RecoveryPhase.DONE,
        active=True,
    )

    assert backing.running() is True
    assert inactive_backing.running() is False
    assert done.running() is False


def test_can_retry():
    assert RecoveryState(attempt=0, max_attempts=3).can_retry() is True
    assert RecoveryState(attempt=2, max_attempts=3).can_retry() is True
    assert RecoveryState(attempt=3, max_attempts=3).can_retry() is False
    assert RecoveryState(attempt=4, max_attempts=3).can_retry() is False


def test_to_dict():
    cmd = TwistCommand(vx=-0.06, source="recovery_backup")
    state = RecoveryState(
        phase=RecoveryPhase.BACKING_UP,
        active=True,
        request=True,
        attempt=1,
        max_attempts=3,
        command=cmd,
        reason="stuck",
        elapsed_s=0.5,
    )

    assert state.to_dict() == {
        "phase": "BACKING_UP",
        "active": True,
        "request": True,
        "attempt": 1,
        "max_attempts": 3,
        "command": {
            "vx": -0.06,
            "vy": 0.0,
            "wz": 0.0,
            "source": "recovery_backup",
            "stamp_sec": 0.0,
        },
        "reason": "stuck",
        "elapsed_s": 0.5,
    }


def test_status_text_without_reason():
    state = RecoveryState(
        phase=RecoveryPhase.IDLE,
        active=False,
        request=False,
        attempt=0,
        max_attempts=3,
        elapsed_s=0.0,
    )

    assert state.status_text() == (
        "phase=IDLE; active=false; request=false; "
        "attempt=0/3; elapsed_s=0.00"
    )


def test_status_text_with_reason():
    state = RecoveryState(
        phase=RecoveryPhase.BACKING_UP,
        active=True,
        request=True,
        attempt=1,
        max_attempts=3,
        reason="stuck",
        elapsed_s=0.75,
    )

    assert state.status_text() == (
        "phase=BACKING_UP; active=true; request=true; "
        "attempt=1/3; elapsed_s=0.75; reason=stuck"
    )


def test_recovery_command_backing_up():
    cmd = recovery_command(
        phase=RecoveryPhase.BACKING_UP,
        backup_speed_m_s=0.06,
    )

    assert cmd.vx == -0.06
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0
    assert cmd.source == "recovery_backup"


def test_recovery_command_backing_up_uses_abs_speed():
    cmd = recovery_command(
        phase=RecoveryPhase.BACKING_UP,
        backup_speed_m_s=-0.06,
    )

    assert cmd.vx == -0.06


def test_recovery_command_turning_positive_sign():
    cmd = recovery_command(
        phase=RecoveryPhase.TURNING,
        turn_speed_rad_s=0.35,
        turn_sign=1,
    )

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.35
    assert cmd.source == "recovery_turn"


def test_recovery_command_turning_negative_sign():
    cmd = recovery_command(
        phase=RecoveryPhase.TURNING,
        turn_speed_rad_s=0.35,
        turn_sign=-1,
    )

    assert cmd.wz == -0.35
    assert cmd.source == "recovery_turn"


def test_recovery_command_turning_zero_sign_defaults_positive():
    cmd = recovery_command(
        phase=RecoveryPhase.TURNING,
        turn_speed_rad_s=0.35,
        turn_sign=0,
    )

    assert cmd.wz == 0.35


def test_recovery_command_idle_done_and_aborted_are_zero():
    for phase in [
        RecoveryPhase.IDLE,
        RecoveryPhase.REQUESTED,
        RecoveryPhase.SETTLING,
        RecoveryPhase.DONE,
        RecoveryPhase.ABORTED,
    ]:
        cmd = recovery_command(phase=phase)
        assert cmd.vx == 0.0
        assert cmd.vy == 0.0
        assert cmd.wz == 0.0
        assert cmd.source == "recovery_zero"
