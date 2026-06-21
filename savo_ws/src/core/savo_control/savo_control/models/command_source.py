# -*- coding: utf-8 -*-

"""Command source model for mux/fallback logic."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Final

from .control_mode import ControlMode
from .twist_command import TwistCommand


class CommandSource(str, Enum):
    MANUAL = "manual"
    AUTO = "auto"
    NAV = "nav"
    RECOVERY = "recovery"
    NONE = "none"

    @classmethod
    def from_text(
        cls,
        value: object,
        *,
        default: "CommandSource" | None = None,
    ) -> "CommandSource":
        if isinstance(value, cls):
            return value

        if isinstance(value, str):
            normalized = value.strip().lower()

            for source in cls:
                if source.value == normalized:
                    return source

        if default is not None:
            return default

        raise ValueError(f"Invalid command source: {value!r}")

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(source.value for source in cls)


COMMAND_SOURCE_PRIORITY: Final[dict[CommandSource, int]] = {
    CommandSource.RECOVERY: 100,
    CommandSource.MANUAL: 90,
    CommandSource.NAV: 80,
    CommandSource.AUTO: 70,
    CommandSource.NONE: 0,
}

MODE_TO_SOURCE: Final[dict[ControlMode, CommandSource]] = {
    ControlMode.MANUAL: CommandSource.MANUAL,
    ControlMode.AUTO: CommandSource.AUTO,
    ControlMode.NAV: CommandSource.NAV,
    ControlMode.RECOVERY: CommandSource.RECOVERY,
    ControlMode.STOP: CommandSource.NONE,
}

SOURCE_TO_MODE: Final[dict[CommandSource, ControlMode]] = {
    CommandSource.MANUAL: ControlMode.MANUAL,
    CommandSource.AUTO: ControlMode.AUTO,
    CommandSource.NAV: ControlMode.NAV,
    CommandSource.RECOVERY: ControlMode.RECOVERY,
    CommandSource.NONE: ControlMode.STOP,
}


@dataclass(frozen=True)
class CommandSourceState:
    source: CommandSource
    command: TwistCommand
    active: bool = False
    available: bool = True
    stale: bool = False
    priority: int = 0

    @classmethod
    def inactive(
        cls,
        source: CommandSource,
        *,
        reason: str = "inactive",
    ) -> "CommandSourceState":
        return cls(
            source=source,
            command=TwistCommand.zero(source=reason),
            active=False,
            available=False,
            stale=False,
            priority=COMMAND_SOURCE_PRIORITY[source],
        )

    @classmethod
    def from_command(
        cls,
        source: CommandSource,
        command: TwistCommand,
        *,
        active: bool = True,
        available: bool = True,
        stale: bool = False,
    ) -> "CommandSourceState":
        return cls(
            source=source,
            command=command.sanitized(),
            active=active,
            available=available,
            stale=stale,
            priority=COMMAND_SOURCE_PRIORITY[source],
        )

    def usable(self) -> bool:
        return self.active and self.available and not self.stale

    def to_dict(self) -> dict:
        return {
            "source": self.source.value,
            "active": self.active,
            "available": self.available,
            "stale": self.stale,
            "priority": self.priority,
            "command": self.command.to_dict(),
        }

    def status_text(self) -> str:
        return (
            f"source={self.source.value}; active={self.active}; "
            f"available={self.available}; stale={self.stale}; priority={self.priority}"
        )


def source_for_mode(mode: ControlMode) -> CommandSource:
    return MODE_TO_SOURCE.get(mode, CommandSource.NONE)


def mode_for_source(source: CommandSource) -> ControlMode:
    return SOURCE_TO_MODE.get(source, ControlMode.STOP)


def highest_priority_source(sources: list[CommandSourceState]) -> CommandSourceState:
    usable = [source for source in sources if source.usable()]

    if not usable:
        return CommandSourceState.inactive(CommandSource.NONE)

    return max(usable, key=lambda item: item.priority)


__all__ = [
    "COMMAND_SOURCE_PRIORITY",
    "MODE_TO_SOURCE",
    "SOURCE_TO_MODE",
    "CommandSource",
    "CommandSourceState",
    "highest_priority_source",
    "mode_for_source",
    "source_for_mode",
]
