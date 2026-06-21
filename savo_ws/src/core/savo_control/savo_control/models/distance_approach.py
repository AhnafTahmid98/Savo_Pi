# -*- coding: utf-8 -*-

"""Pure Python models for distance approach control."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .twist_command import TwistCommand


@dataclass(frozen=True)
class DistanceApproachConfig:
    target_distance_m: float = 0.60
    tolerance_m: float = 0.04
    hard_min_distance_m: float = 0.35

    min_valid_distance_m: float = 0.05
    max_valid_distance_m: float = 3.00
    distance_timeout_s: float = 0.40

    kp: float = 0.45
    ki: float = 0.0
    kd: float = 0.03

    max_forward_vx: float = 0.10
    allow_reverse: bool = False
    max_reverse_vx: float = 0.05
    min_vx_when_active: float = 0.04
    disable_min_vx_below_error_m: float = 0.08

    def sanitized(self) -> "DistanceApproachConfig":
        min_valid = max(0.01, abs(self.min_valid_distance_m))
        max_valid = max(min_valid + 0.01, abs(self.max_valid_distance_m))

        return DistanceApproachConfig(
            target_distance_m=max(0.01, abs(self.target_distance_m)),
            tolerance_m=max(0.001, abs(self.tolerance_m)),
            hard_min_distance_m=max(0.01, abs(self.hard_min_distance_m)),
            min_valid_distance_m=min_valid,
            max_valid_distance_m=max_valid,
            distance_timeout_s=max(0.05, abs(self.distance_timeout_s)),
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            max_forward_vx=abs(self.max_forward_vx),
            allow_reverse=self.allow_reverse,
            max_reverse_vx=abs(self.max_reverse_vx),
            min_vx_when_active=abs(self.min_vx_when_active),
            disable_min_vx_below_error_m=abs(self.disable_min_vx_below_error_m),
        )

    def valid_distance(self, distance_m: float) -> bool:
        cfg = self.sanitized()
        return (
            math.isfinite(distance_m)
            and cfg.min_valid_distance_m <= distance_m <= cfg.max_valid_distance_m
        )

    def too_close(self, distance_m: float) -> bool:
        cfg = self.sanitized()
        return distance_m < cfg.hard_min_distance_m

    def goal_reached(self, distance_m: float) -> bool:
        cfg = self.sanitized()
        return abs(distance_m - cfg.target_distance_m) <= cfg.tolerance_m + 1e-9

    def error_m(self, distance_m: float) -> float:
        cfg = self.sanitized()
        return distance_m - cfg.target_distance_m

    def to_dict(self) -> dict:
        cfg = self.sanitized()
        return {
            "target_distance_m": cfg.target_distance_m,
            "tolerance_m": cfg.tolerance_m,
            "hard_min_distance_m": cfg.hard_min_distance_m,
            "min_valid_distance_m": cfg.min_valid_distance_m,
            "max_valid_distance_m": cfg.max_valid_distance_m,
            "distance_timeout_s": cfg.distance_timeout_s,
            "kp": cfg.kp,
            "ki": cfg.ki,
            "kd": cfg.kd,
            "max_forward_vx": cfg.max_forward_vx,
            "allow_reverse": cfg.allow_reverse,
            "max_reverse_vx": cfg.max_reverse_vx,
            "min_vx_when_active": cfg.min_vx_when_active,
            "disable_min_vx_below_error_m": cfg.disable_min_vx_below_error_m,
        }


@dataclass(frozen=True)
class DistanceApproachState:
    state: str = "IDLE"
    distance_m: float | None = None
    target_distance_m: float = 0.60
    error_m: float | None = None
    command: TwistCommand = TwistCommand.zero(source="distance_approach")
    safety_stop: bool = False
    stale: bool = False
    valid: bool = False

    @classmethod
    def idle(cls, *, target_distance_m: float = 0.60) -> "DistanceApproachState":
        return cls(state="IDLE", target_distance_m=target_distance_m)

    @classmethod
    def stopped(cls, reason: str, *, target_distance_m: float = 0.60) -> "DistanceApproachState":
        return cls(
            state=reason,
            target_distance_m=target_distance_m,
            command=TwistCommand.zero(source=reason.lower()),
        )

    def running(self) -> bool:
        return self.state == "RUNNING"

    def goal_reached(self) -> bool:
        return self.state == "GOAL_REACHED"

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "distance_m": self.distance_m,
            "target_distance_m": self.target_distance_m,
            "error_m": self.error_m,
            "command": self.command.to_dict(),
            "safety_stop": self.safety_stop,
            "stale": self.stale,
            "valid": self.valid,
        }

    def status_text(self) -> str:
        distance = "nan" if self.distance_m is None else f"{self.distance_m:.3f}"
        error = "nan" if self.error_m is None else f"{self.error_m:.3f}"

        return (
            f"state={self.state}; distance_m={distance}; "
            f"target_m={self.target_distance_m:.3f}; error_m={error}; "
            f"vx={self.command.sanitized().vx:.3f}"
        )


def command_from_error(error_m: float, cfg: DistanceApproachConfig) -> TwistCommand:
    safe_cfg = cfg.sanitized()

    if not math.isfinite(error_m):
        return TwistCommand.zero(source="invalid_error")

    raw_vx = safe_cfg.kp * error_m

    if raw_vx >= 0.0:
        vx = min(raw_vx, safe_cfg.max_forward_vx)
    elif safe_cfg.allow_reverse:
        vx = max(raw_vx, -safe_cfg.max_reverse_vx)
    else:
        vx = 0.0

    if (
        vx > 0.0
        and abs(error_m) > safe_cfg.disable_min_vx_below_error_m
        and vx < safe_cfg.min_vx_when_active
    ):
        vx = safe_cfg.min_vx_when_active

    return TwistCommand(vx=vx, source="distance_approach").sanitized()


__all__ = [
    "DistanceApproachConfig",
    "DistanceApproachState",
    "command_from_error",
]
