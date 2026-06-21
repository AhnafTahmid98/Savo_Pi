# -*- coding: utf-8 -*-

"""Pure helpers for control_status_node.py."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from savo_control.models import TwistCommand


NONZERO_EPS = 1.0e-4


@dataclass(frozen=True)
class CommandSample:
    command: TwistCommand = TwistCommand.zero()
    stamp_s: Optional[float] = None

    def fresh(self, *, now_s: float, timeout_s: float) -> bool:
        return self.stamp_s is not None and (now_s - self.stamp_s) <= timeout_s

    def moving(self) -> bool:
        cmd = self.command.sanitized()

        return (
            abs(cmd.vx) > NONZERO_EPS
            or abs(cmd.vy) > NONZERO_EPS
            or abs(cmd.wz) > NONZERO_EPS
        )


@dataclass(frozen=True)
class ScalarSample:
    value: Optional[float] = None
    stamp_s: Optional[float] = None

    def fresh(self, *, now_s: float, timeout_s: float) -> bool:
        return self.stamp_s is not None and (now_s - self.stamp_s) <= timeout_s


@dataclass(frozen=True)
class BoolSample:
    value: Optional[bool] = None
    stamp_s: Optional[float] = None

    def fresh(self, *, now_s: float, timeout_s: float) -> bool:
        return self.stamp_s is not None and (now_s - self.stamp_s) <= timeout_s


@dataclass(frozen=True)
class TextSample:
    value: str = ""
    stamp_s: Optional[float] = None

    def fresh(self, *, now_s: float, timeout_s: float) -> bool:
        return self.stamp_s is not None and (now_s - self.stamp_s) <= timeout_s


@dataclass(frozen=True)
class OdomSample:
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    stamp_s: Optional[float] = None

    def fresh(self, *, now_s: float, timeout_s: float) -> bool:
        return self.stamp_s is not None and (now_s - self.stamp_s) <= timeout_s


__all__ = [
    "NONZERO_EPS",
    "BoolSample",
    "CommandSample",
    "OdomSample",
    "ScalarSample",
    "TextSample",
]
