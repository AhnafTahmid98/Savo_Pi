# -*- coding: utf-8 -*-

"""Pure Python recovery-state models."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .twist_command import TwistCommand


class RecoveryPhase(str, Enum):
    IDLE = "IDLE"
    REQUESTED = "REQUESTED"
    BACKING_UP = "BACKING_UP"
    SETTLING = "SETTLING"
    TURNING = "TURNING"
    DONE = "DONE"
    ABORTED = "ABORTED"

    @classmethod
    def from_text(
        cls,
        value: object,
        *,
        default: "RecoveryPhase" | None = None,
    ) -> "RecoveryPhase":
        if isinstance(value, cls):
            return value

        if isinstance(value, str):
            normalized = value.strip().upper()
            for phase in cls:
                if phase.value == normalized:
                    return phase

        if default is not None:
            return default

        raise ValueError(f"Invalid recovery phase: {value!r}")

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(phase.value for phase in cls)


RECOVERY_ACTIVE_PHASES = (
    RecoveryPhase.REQUESTED,
    RecoveryPhase.BACKING_UP,
    RecoveryPhase.SETTLING,
    RecoveryPhase.TURNING,
)


@dataclass(frozen=True)
class RecoveryState:
    phase: RecoveryPhase = RecoveryPhase.IDLE
    active: bool = False
    request: bool = False
    attempt: int = 0
    max_attempts: int = 3
    command: TwistCommand = TwistCommand.zero(source="recovery")
    reason: str = ""
    elapsed_s: float = 0.0

    @classmethod
    def idle(cls) -> "RecoveryState":
        return cls(phase=RecoveryPhase.IDLE)

    @classmethod
    def requested(cls, *, reason: str = "stuck") -> "RecoveryState":
        return cls(
            phase=RecoveryPhase.REQUESTED,
            active=True,
            request=True,
            reason=reason,
        )

    @classmethod
    def aborted(cls, *, reason: str = "aborted") -> "RecoveryState":
        return cls(
            phase=RecoveryPhase.ABORTED,
            active=False,
            request=False,
            command=TwistCommand.zero(source="recovery_aborted"),
            reason=reason,
        )

    @classmethod
    def done(cls, *, reason: str = "done") -> "RecoveryState":
        return cls(
            phase=RecoveryPhase.DONE,
            active=False,
            request=False,
            command=TwistCommand.zero(source="recovery_done"),
            reason=reason,
        )

    def running(self) -> bool:
        return self.phase in RECOVERY_ACTIVE_PHASES and self.active

    def finished(self) -> bool:
        return self.phase in {RecoveryPhase.DONE, RecoveryPhase.ABORTED}

    def can_retry(self) -> bool:
        return self.attempt < self.max_attempts

    def to_dict(self) -> dict:
        return {
            "phase": self.phase.value,
            "active": self.active,
            "request": self.request,
            "attempt": self.attempt,
            "max_attempts": self.max_attempts,
            "command": self.command.to_dict(),
            "reason": self.reason,
            "elapsed_s": self.elapsed_s,
        }

    def status_text(self) -> str:
        parts = [
            f"phase={self.phase.value}",
            f"active={str(self.active).lower()}",
            f"request={str(self.request).lower()}",
            f"attempt={self.attempt}/{self.max_attempts}",
            f"elapsed_s={self.elapsed_s:.2f}",
        ]

        if self.reason:
            parts.append(f"reason={self.reason}")

        return "; ".join(parts)


def recovery_command(
    *,
    phase: RecoveryPhase,
    backup_speed_m_s: float = 0.06,
    turn_speed_rad_s: float = 0.35,
    turn_sign: int = 1,
) -> TwistCommand:
    if phase == RecoveryPhase.BACKING_UP:
        return TwistCommand(vx=-abs(backup_speed_m_s), source="recovery_backup")

    if phase == RecoveryPhase.TURNING:
        sign = 1 if turn_sign >= 0 else -1
        return TwistCommand(wz=abs(turn_speed_rad_s) * sign, source="recovery_turn")

    return TwistCommand.zero(source="recovery_zero")


__all__ = [
    "RECOVERY_ACTIVE_PHASES",
    "RecoveryPhase",
    "RecoveryState",
    "recovery_command",
]
