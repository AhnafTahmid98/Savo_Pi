# -*- coding: utf-8 -*-

"""Pure Python stuck-detection state model."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .twist_command import TwistCommand


class StuckState(str, Enum):
    CLEAR = "CLEAR"
    WATCHING = "WATCHING"
    STUCK = "STUCK"
    RECOVERY_REQUESTED = "RECOVERY_REQUESTED"
    SUPPRESSED = "SUPPRESSED"
    STALE = "STALE"

    @classmethod
    def from_text(
        cls,
        value: object,
        *,
        default: "StuckState" | None = None,
    ) -> "StuckState":
        if isinstance(value, cls):
            return value

        if isinstance(value, str):
            normalized = value.strip().upper()
            for state in cls:
                if state.value == normalized:
                    return state

        if default is not None:
            return default

        raise ValueError(f"Invalid stuck state: {value!r}")

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(state.value for state in cls)


@dataclass(frozen=True)
class StuckDetectorState:
    state: StuckState = StuckState.CLEAR
    cmd_vel_safe: TwistCommand = TwistCommand.zero(source="cmd_vel_safe")
    observed_vx: float = 0.0
    observed_vy: float = 0.0
    observed_wz: float = 0.0
    safety_stop: bool = False
    cmd_stale: bool = False
    odom_stale: bool = False
    stuck_duration_s: float = 0.0
    clear_duration_s: float = 0.0
    recovery_request: bool = False
    reason: str = ""

    @classmethod
    def clear(cls, *, reason: str = "clear") -> "StuckDetectorState":
        return cls(state=StuckState.CLEAR, reason=reason)

    @classmethod
    def stuck(
        cls,
        *,
        cmd_vel_safe: TwistCommand,
        stuck_duration_s: float,
        reason: str = "no_motion",
    ) -> "StuckDetectorState":
        return cls(
            state=StuckState.STUCK,
            cmd_vel_safe=cmd_vel_safe.sanitized(),
            stuck_duration_s=max(0.0, stuck_duration_s),
            reason=reason,
        )

    @classmethod
    def suppressed(
        cls,
        *,
        reason: str = "safety_stop",
        safety_stop: bool = True,
    ) -> "StuckDetectorState":
        return cls(
            state=StuckState.SUPPRESSED,
            safety_stop=safety_stop,
            reason=reason,
        )

    @classmethod
    def stale(
        cls,
        *,
        cmd_stale: bool = False,
        odom_stale: bool = False,
        reason: str = "stale_data",
    ) -> "StuckDetectorState":
        return cls(
            state=StuckState.STALE,
            cmd_stale=cmd_stale,
            odom_stale=odom_stale,
            reason=reason,
        )

    def active(self) -> bool:
        return self.state in {
            StuckState.WATCHING,
            StuckState.STUCK,
            StuckState.RECOVERY_REQUESTED,
        }

    def stuck_detected(self) -> bool:
        return self.state in {
            StuckState.STUCK,
            StuckState.RECOVERY_REQUESTED,
        }

    def can_request_recovery(self) -> bool:
        return (
            self.state == StuckState.STUCK
            and not self.safety_stop
            and not self.cmd_stale
            and not self.odom_stale
        )

    def observed_linear_speed(self) -> float:
        return TwistCommand(
            vx=self.observed_vx,
            vy=self.observed_vy,
            wz=0.0,
        ).linear_speed()

    def to_dict(self) -> dict:
        return {
            "state": self.state.value,
            "cmd_vel_safe": self.cmd_vel_safe.to_dict(),
            "observed_vx": self.observed_vx,
            "observed_vy": self.observed_vy,
            "observed_wz": self.observed_wz,
            "safety_stop": self.safety_stop,
            "cmd_stale": self.cmd_stale,
            "odom_stale": self.odom_stale,
            "stuck_duration_s": self.stuck_duration_s,
            "clear_duration_s": self.clear_duration_s,
            "recovery_request": self.recovery_request,
            "reason": self.reason,
        }

    def status_text(self) -> str:
        parts = [
            f"state={self.state.value}",
            f"safety_stop={str(self.safety_stop).lower()}",
            f"cmd_stale={str(self.cmd_stale).lower()}",
            f"odom_stale={str(self.odom_stale).lower()}",
            f"stuck_s={self.stuck_duration_s:.2f}",
            f"recovery_request={str(self.recovery_request).lower()}",
        ]

        if self.reason:
            parts.append(f"reason={self.reason}")

        return "; ".join(parts)


def should_detect_stuck(
    *,
    cmd_vel_safe: TwistCommand,
    observed_vx: float,
    observed_vy: float,
    observed_wz: float,
    min_cmd_vx: float = 0.05,
    min_cmd_vy: float = 0.05,
    min_cmd_wz: float = 0.12,
    min_observed_linear_m_s: float = 0.025,
    min_observed_strafe_m_s: float = 0.020,
    min_observed_angular_rad_s: float = 0.06,
) -> bool:
    cmd = cmd_vel_safe.sanitized()

    commanded_linear = abs(cmd.vx) >= abs(min_cmd_vx)
    commanded_strafe = abs(cmd.vy) >= abs(min_cmd_vy)
    commanded_turn = abs(cmd.wz) >= abs(min_cmd_wz)

    if not commanded_linear and not commanded_strafe and not commanded_turn:
        return False

    moving_linear = abs(observed_vx) >= abs(min_observed_linear_m_s)
    moving_strafe = abs(observed_vy) >= abs(min_observed_strafe_m_s)
    moving_turn = abs(observed_wz) >= abs(min_observed_angular_rad_s)

    if commanded_linear and not moving_linear:
        return True
    if commanded_strafe and not moving_strafe:
        return True
    if commanded_turn and not moving_turn:
        return True

    return False


__all__ = [
    "StuckDetectorState",
    "StuckState",
    "should_detect_stuck",
]
