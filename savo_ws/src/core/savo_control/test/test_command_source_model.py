#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for command-source models."""

from __future__ import annotations

import pytest

from savo_control.models import (
    COMMAND_SOURCE_PRIORITY,
    MODE_TO_SOURCE,
    SOURCE_TO_MODE,
    CommandSource,
    CommandSourceState,
    ControlMode,
    TwistCommand,
    highest_priority_source,
    mode_for_source,
    source_for_mode,
)


def test_command_source_values_are_canonical():
    assert CommandSource.values() == (
        "manual",
        "auto",
        "nav",
        "recovery",
        "none",
    )


def test_command_source_from_text_accepts_valid_inputs():
    assert CommandSource.from_text("manual") == CommandSource.MANUAL
    assert CommandSource.from_text("AUTO") == CommandSource.AUTO
    assert CommandSource.from_text(" nav ") == CommandSource.NAV
    assert CommandSource.from_text("recovery") == CommandSource.RECOVERY
    assert CommandSource.from_text("none") == CommandSource.NONE


def test_command_source_from_text_rejects_invalid_input():
    with pytest.raises(ValueError):
        CommandSource.from_text("base")


def test_command_source_from_text_uses_default():
    assert (
        CommandSource.from_text("bad", default=CommandSource.NONE)
        == CommandSource.NONE
    )
    assert (
        CommandSource.from_text(None, default=CommandSource.MANUAL)
        == CommandSource.MANUAL
    )


def test_command_source_priority_is_safe():
    assert COMMAND_SOURCE_PRIORITY[CommandSource.RECOVERY] > COMMAND_SOURCE_PRIORITY[CommandSource.MANUAL]
    assert COMMAND_SOURCE_PRIORITY[CommandSource.MANUAL] > COMMAND_SOURCE_PRIORITY[CommandSource.NAV]
    assert COMMAND_SOURCE_PRIORITY[CommandSource.NAV] > COMMAND_SOURCE_PRIORITY[CommandSource.AUTO]
    assert COMMAND_SOURCE_PRIORITY[CommandSource.AUTO] > COMMAND_SOURCE_PRIORITY[CommandSource.NONE]


def test_mode_to_source_mapping():
    assert MODE_TO_SOURCE[ControlMode.STOP] == CommandSource.NONE
    assert MODE_TO_SOURCE[ControlMode.MANUAL] == CommandSource.MANUAL
    assert MODE_TO_SOURCE[ControlMode.AUTO] == CommandSource.AUTO
    assert MODE_TO_SOURCE[ControlMode.NAV] == CommandSource.NAV
    assert MODE_TO_SOURCE[ControlMode.RECOVERY] == CommandSource.RECOVERY


def test_source_to_mode_mapping():
    assert SOURCE_TO_MODE[CommandSource.NONE] == ControlMode.STOP
    assert SOURCE_TO_MODE[CommandSource.MANUAL] == ControlMode.MANUAL
    assert SOURCE_TO_MODE[CommandSource.AUTO] == ControlMode.AUTO
    assert SOURCE_TO_MODE[CommandSource.NAV] == ControlMode.NAV
    assert SOURCE_TO_MODE[CommandSource.RECOVERY] == ControlMode.RECOVERY


def test_source_for_mode():
    assert source_for_mode(ControlMode.STOP) == CommandSource.NONE
    assert source_for_mode(ControlMode.MANUAL) == CommandSource.MANUAL
    assert source_for_mode(ControlMode.AUTO) == CommandSource.AUTO
    assert source_for_mode(ControlMode.NAV) == CommandSource.NAV
    assert source_for_mode(ControlMode.RECOVERY) == CommandSource.RECOVERY


def test_mode_for_source():
    assert mode_for_source(CommandSource.NONE) == ControlMode.STOP
    assert mode_for_source(CommandSource.MANUAL) == ControlMode.MANUAL
    assert mode_for_source(CommandSource.AUTO) == ControlMode.AUTO
    assert mode_for_source(CommandSource.NAV) == ControlMode.NAV
    assert mode_for_source(CommandSource.RECOVERY) == ControlMode.RECOVERY


def test_inactive_source_state():
    state = CommandSourceState.inactive(CommandSource.RECOVERY)

    assert state.source == CommandSource.RECOVERY
    assert state.active is False
    assert state.available is False
    assert state.stale is False
    assert state.priority == COMMAND_SOURCE_PRIORITY[CommandSource.RECOVERY]
    assert state.command.vx == 0.0
    assert state.command.source == "inactive"
    assert state.usable() is False


def test_inactive_source_state_custom_reason():
    state = CommandSourceState.inactive(CommandSource.MANUAL, reason="timeout")

    assert state.command.source == "timeout"
    assert state.usable() is False


def test_source_state_from_command():
    command = TwistCommand(vx=0.1, vy=0.0, wz=0.2, source="manual")
    state = CommandSourceState.from_command(CommandSource.MANUAL, command)

    assert state.source == CommandSource.MANUAL
    assert state.command == command
    assert state.active is True
    assert state.available is True
    assert state.stale is False
    assert state.priority == COMMAND_SOURCE_PRIORITY[CommandSource.MANUAL]
    assert state.usable() is True


def test_source_state_from_command_sanitizes_command():
    command = TwistCommand(vx=float("nan"), vy=0.1, wz=float("inf"))
    state = CommandSourceState.from_command(CommandSource.AUTO, command)

    assert state.command.vx == 0.0
    assert state.command.vy == 0.1
    assert state.command.wz == 0.0


def test_source_state_not_usable_when_inactive_unavailable_or_stale():
    command = TwistCommand(vx=0.1)

    inactive = CommandSourceState.from_command(
        CommandSource.MANUAL,
        command,
        active=False,
    )
    unavailable = CommandSourceState.from_command(
        CommandSource.MANUAL,
        command,
        available=False,
    )
    stale = CommandSourceState.from_command(
        CommandSource.MANUAL,
        command,
        stale=True,
    )

    assert inactive.usable() is False
    assert unavailable.usable() is False
    assert stale.usable() is False


def test_source_state_to_dict():
    state = CommandSourceState.from_command(
        CommandSource.NAV,
        TwistCommand(vx=0.2, source="nav", stamp_sec=3.0),
    )

    assert state.to_dict() == {
        "source": "nav",
        "active": True,
        "available": True,
        "stale": False,
        "priority": COMMAND_SOURCE_PRIORITY[CommandSource.NAV],
        "command": {
            "vx": 0.2,
            "vy": 0.0,
            "wz": 0.0,
            "source": "nav",
            "stamp_sec": 3.0,
        },
    }


def test_source_state_status_text():
    state = CommandSourceState.from_command(
        CommandSource.AUTO,
        TwistCommand(vx=0.1),
    )

    text = state.status_text()

    assert "source=auto" in text
    assert "active=True" in text
    assert "available=True" in text
    assert "stale=False" in text


def test_highest_priority_source_returns_none_when_no_usable_sources():
    auto = CommandSourceState.from_command(
        CommandSource.AUTO,
        TwistCommand(vx=0.1),
        active=False,
    )
    nav = CommandSourceState.from_command(
        CommandSource.NAV,
        TwistCommand(vx=0.1),
        stale=True,
    )

    winner = highest_priority_source([auto, nav])

    assert winner.source == CommandSource.NONE
    assert winner.usable() is False


def test_highest_priority_source_selects_highest_usable_source():
    auto = CommandSourceState.from_command(
        CommandSource.AUTO,
        TwistCommand(vx=0.1, source="auto"),
    )
    nav = CommandSourceState.from_command(
        CommandSource.NAV,
        TwistCommand(vx=0.2, source="nav"),
    )
    manual = CommandSourceState.from_command(
        CommandSource.MANUAL,
        TwistCommand(vx=0.3, source="manual"),
    )

    winner = highest_priority_source([auto, nav, manual])

    assert winner.source == CommandSource.MANUAL
    assert winner.command.source == "manual"


def test_highest_priority_source_prefers_recovery_over_manual():
    manual = CommandSourceState.from_command(
        CommandSource.MANUAL,
        TwistCommand(vx=0.3, source="manual"),
    )
    recovery = CommandSourceState.from_command(
        CommandSource.RECOVERY,
        TwistCommand(vx=-0.1, source="recovery"),
    )

    winner = highest_priority_source([manual, recovery])

    assert winner.source == CommandSource.RECOVERY
    assert winner.command.source == "recovery"
