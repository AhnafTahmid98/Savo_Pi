# -*- coding: utf-8 -*-

"""Control mode model shared by Python fallback nodes and diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Final, Iterable


class ControlMode(str, Enum):
    STOP = "STOP"
    MANUAL = "MANUAL"
    AUTO = "AUTO"
    NAV = "NAV"
    RECOVERY = "RECOVERY"

    @classmethod
    def from_text(cls, value: object, *, default: "ControlMode" | None = None) -> "ControlMode":
        if isinstance(value, cls):
            return value

        if isinstance(value, str):
            normalized = value.strip().upper()

            if normalized in {"IDLE", "DISABLED"}:
                return cls.STOP

            for mode in cls:
                if mode.value == normalized:
                    return mode

        if default is not None:
            return default

        raise ValueError(f"Invalid control mode: {value!r}")

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(mode.value for mode in cls)


VALID_CONTROL_MODES: Final[tuple[str, ...]] = ControlMode.values()

MODE_PRIORITY: Final[dict[ControlMode, int]] = {
    ControlMode.STOP: 100,
    ControlMode.RECOVERY: 90,
    ControlMode.MANUAL: 80,
    ControlMode.NAV: 70,
    ControlMode.AUTO: 60,
}


@dataclass(frozen=True)
class ControlModeState:
    mode: ControlMode
    reason: str = ""
    source: str = ""
    stamp_sec: float = 0.0

    @classmethod
    def stopped(cls, reason: str = "startup") -> "ControlModeState":
        return cls(mode=ControlMode.STOP, reason=reason)

    def to_dict(self) -> dict:
        return {
            "mode": self.mode.value,
            "reason": self.reason,
            "source": self.source,
            "stamp_sec": self.stamp_sec,
        }

    def status_text(self) -> str:
        if self.reason:
            return f"mode={self.mode.value}; reason={self.reason}"
        return f"mode={self.mode.value}"


def is_valid_mode(value: object) -> bool:
    try:
        ControlMode.from_text(value)
    except ValueError:
        return False
    return True


def normalize_mode(value: object, *, default: ControlMode = ControlMode.STOP) -> str:
    return ControlMode.from_text(value, default=default).value


def highest_priority_mode(modes: Iterable[object]) -> ControlMode:
    parsed = [ControlMode.from_text(mode, default=ControlMode.STOP) for mode in modes]

    if not parsed:
        return ControlMode.STOP

    return max(parsed, key=lambda mode: MODE_PRIORITY[mode])


__all__ = [
    "ControlMode",
    "ControlModeState",
    "VALID_CONTROL_MODES",
    "MODE_PRIORITY",
    "is_valid_mode",
    "normalize_mode",
    "highest_priority_mode",
]
