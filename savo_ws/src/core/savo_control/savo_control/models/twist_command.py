# -*- coding: utf-8 -*-

"""Pure Python Twist command model."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TwistCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    source: str = ""
    stamp_sec: float = 0.0

    @classmethod
    def zero(cls, *, source: str = "zero", stamp_sec: float = 0.0) -> "TwistCommand":
        return cls(vx=0.0, vy=0.0, wz=0.0, source=source, stamp_sec=stamp_sec)

    def finite(self) -> bool:
        return (
            math.isfinite(self.vx)
            and math.isfinite(self.vy)
            and math.isfinite(self.wz)
        )

    def sanitized(self) -> "TwistCommand":
        return TwistCommand(
            vx=self.vx if math.isfinite(self.vx) else 0.0,
            vy=self.vy if math.isfinite(self.vy) else 0.0,
            wz=self.wz if math.isfinite(self.wz) else 0.0,
            source=self.source,
            stamp_sec=self.stamp_sec,
        )

    def clamp(
        self,
        *,
        max_vx: float,
        max_vy: float,
        max_wz: float,
    ) -> "TwistCommand":
        cmd = self.sanitized()

        return TwistCommand(
            vx=_clamp(cmd.vx, -abs(max_vx), abs(max_vx)),
            vy=_clamp(cmd.vy, -abs(max_vy), abs(max_vy)),
            wz=_clamp(cmd.wz, -abs(max_wz), abs(max_wz)),
            source=cmd.source,
            stamp_sec=cmd.stamp_sec,
        )

    def with_deadband(
        self,
        *,
        vx_deadband: float = 0.0,
        vy_deadband: float = 0.0,
        wz_deadband: float = 0.0,
    ) -> "TwistCommand":
        cmd = self.sanitized()

        return TwistCommand(
            vx=_deadband(cmd.vx, abs(vx_deadband)),
            vy=_deadband(cmd.vy, abs(vy_deadband)),
            wz=_deadband(cmd.wz, abs(wz_deadband)),
            source=cmd.source,
            stamp_sec=cmd.stamp_sec,
        )

    def linear_speed(self) -> float:
        cmd = self.sanitized()
        return math.hypot(cmd.vx, cmd.vy)

    def moving(self, *, linear_eps: float = 1e-6, angular_eps: float = 1e-6) -> bool:
        cmd = self.sanitized()
        return cmd.linear_speed() > linear_eps or abs(cmd.wz) > angular_eps

    def to_dict(self) -> dict:
        cmd = self.sanitized()
        return {
            "vx": cmd.vx,
            "vy": cmd.vy,
            "wz": cmd.wz,
            "source": cmd.source,
            "stamp_sec": cmd.stamp_sec,
        }

    def status_text(self) -> str:
        cmd = self.sanitized()
        return (
            f"vx={cmd.vx:.3f}; vy={cmd.vy:.3f}; "
            f"wz={cmd.wz:.3f}; source={cmd.source or 'unknown'}"
        )


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


def _deadband(value: float, threshold: float) -> float:
    if abs(value) < threshold:
        return 0.0
    return value


__all__ = [
    "TwistCommand",
]
