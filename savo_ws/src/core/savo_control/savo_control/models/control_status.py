# -*- coding: utf-8 -*-

"""Pure Python status model for the control layer."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .control_mode import ControlMode, normalize_mode
from .twist_command import TwistCommand


STATUS_OK = "OK"
STATUS_WARN = "WARN"
STATUS_ERROR = "ERROR"
STATUS_STALE = "STALE"
STATUS_STOPPED = "STOPPED"
STATUS_UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ControlStatus:
    status: str = STATUS_UNKNOWN
    mode: ControlMode = ControlMode.STOP
    selected_source: str = "none"

    cmd_vel: TwistCommand = field(default_factory=lambda: TwistCommand.zero(source="cmd_vel"))
    cmd_vel_mux: TwistCommand = field(
        default_factory=lambda: TwistCommand.zero(source="cmd_vel_mux")
    )
    cmd_vel_safe: TwistCommand = field(
        default_factory=lambda: TwistCommand.zero(source="cmd_vel_safe")
    )

    safety_stop: bool = False
    slowdown_factor: float = 1.0
    recovery_active: bool = False
    stuck_detected: bool = False

    odom_fresh: bool = True
    command_fresh: bool = True
    note: str = ""

    @classmethod
    def stopped(cls, *, reason: str = "startup") -> "ControlStatus":
        return cls(
            status=STATUS_STOPPED,
            mode=ControlMode.STOP,
            selected_source="none",
            note=reason,
        )

    @classmethod
    def ok(
        cls,
        *,
        mode: object = ControlMode.STOP,
        selected_source: str = "none",
    ) -> "ControlStatus":
        return cls(
            status=STATUS_OK,
            mode=ControlMode.from_text(mode, default=ControlMode.STOP),
            selected_source=selected_source,
        )

    def normalized_status(self) -> str:
        status = str(self.status).strip().upper()
        if status in {
            STATUS_OK,
            STATUS_WARN,
            STATUS_ERROR,
            STATUS_STALE,
            STATUS_STOPPED,
        }:
            return status
        return STATUS_UNKNOWN

    def healthy(self) -> bool:
        return (
            self.normalized_status() == STATUS_OK
            and not self.safety_stop
            and self.command_fresh
            and self.odom_fresh
        )

    def blocked(self) -> bool:
        return self.safety_stop or self.stuck_detected or self.normalized_status() == STATUS_ERROR

    def to_dict(self) -> dict[str, Any]:
        return {
            "status": self.normalized_status(),
            "mode": self.mode.value,
            "selected_source": self.selected_source,
            "cmd_vel": self.cmd_vel.to_dict(),
            "cmd_vel_mux": self.cmd_vel_mux.to_dict(),
            "cmd_vel_safe": self.cmd_vel_safe.to_dict(),
            "safety_stop": self.safety_stop,
            "slowdown_factor": self.slowdown_factor,
            "recovery_active": self.recovery_active,
            "stuck_detected": self.stuck_detected,
            "odom_fresh": self.odom_fresh,
            "command_fresh": self.command_fresh,
            "note": self.note,
        }

    def status_text(self) -> str:
        parts = [
            f"status={self.normalized_status()}",
            f"mode={self.mode.value}",
            f"source={self.selected_source}",
            f"safety_stop={str(self.safety_stop).lower()}",
            f"slowdown={self.slowdown_factor:.2f}",
            f"recovery={str(self.recovery_active).lower()}",
            f"stuck={str(self.stuck_detected).lower()}",
        ]

        if self.note:
            parts.append(f"note={self.note}")

        return "; ".join(parts)


def control_status_from_values(
    *,
    status: str = STATUS_UNKNOWN,
    mode: object = ControlMode.STOP,
    selected_source: str = "none",
    safety_stop: bool = False,
    slowdown_factor: float = 1.0,
    recovery_active: bool = False,
    stuck_detected: bool = False,
    odom_fresh: bool = True,
    command_fresh: bool = True,
    note: str = "",
) -> ControlStatus:
    return ControlStatus(
        status=status,
        mode=ControlMode.from_text(normalize_mode(mode), default=ControlMode.STOP),
        selected_source=selected_source,
        safety_stop=safety_stop,
        slowdown_factor=float(slowdown_factor),
        recovery_active=bool(recovery_active),
        stuck_detected=bool(stuck_detected),
        odom_fresh=bool(odom_fresh),
        command_fresh=bool(command_fresh),
        note=note,
    )


__all__ = [
    "ControlStatus",
    "STATUS_ERROR",
    "STATUS_OK",
    "STATUS_STALE",
    "STATUS_STOPPED",
    "STATUS_UNKNOWN",
    "STATUS_WARN",
    "control_status_from_values",
]
