# -*- coding: utf-8 -*-

"""Pure keyboard teleop helpers for keyboard_teleop_node.py."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from savo_control.models import TwistCommand
from savo_control.utils import clamp, finite_or_zero


TELEOP_SOURCE = "keyboard_teleop"


@dataclass(frozen=True)
class TeleopSpeeds:
    linear: float = 0.12
    angular: float = 0.35

    def sanitized(self) -> "TeleopSpeeds":
        return TeleopSpeeds(
            linear=max(0.0, finite_or_zero(self.linear)),
            angular=max(0.0, finite_or_zero(self.angular)),
        )


@dataclass(frozen=True)
class TeleopLimits:
    default_linear: float = 0.12
    default_angular: float = 0.35
    max_linear: float = 0.25
    max_angular: float = 0.60
    linear_step: float = 0.02
    angular_step: float = 0.05

    def sanitized(self) -> "TeleopLimits":
        max_linear = max(1.0e-6, abs(finite_or_zero(self.max_linear)))
        max_angular = max(1.0e-6, abs(finite_or_zero(self.max_angular)))

        default_linear = clamp(
            abs(finite_or_zero(self.default_linear)),
            0.0,
            max_linear,
        )
        default_angular = clamp(
            abs(finite_or_zero(self.default_angular)),
            0.0,
            max_angular,
        )

        return TeleopLimits(
            default_linear=default_linear,
            default_angular=default_angular,
            max_linear=max_linear,
            max_angular=max_angular,
            linear_step=max(0.0, abs(finite_or_zero(self.linear_step))),
            angular_step=max(0.0, abs(finite_or_zero(self.angular_step))),
        )

    def default_speeds(self) -> TeleopSpeeds:
        safe = self.sanitized()
        return TeleopSpeeds(
            linear=safe.default_linear,
            angular=safe.default_angular,
        )


@dataclass(frozen=True)
class TeleopKeyResult:
    handled: bool = False
    command: Optional[TwistCommand] = None
    speeds: TeleopSpeeds = TeleopSpeeds()
    speed_changed: bool = False
    stop_requested: bool = False
    help_requested: bool = False
    label: str = ""


def normalize_key(key: object) -> str:
    if key is None:
        return ""

    return str(key).lower()


def make_teleop_command(
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    limits: TeleopLimits,
    stamp_sec: float = 0.0,
    source: str = TELEOP_SOURCE,
) -> TwistCommand:
    safe_limits = limits.sanitized()

    return TwistCommand(
        vx=finite_or_zero(vx),
        vy=finite_or_zero(vy),
        wz=finite_or_zero(wz),
        source=source,
        stamp_sec=stamp_sec,
    ).clamp(
        max_vx=safe_limits.max_linear,
        max_vy=safe_limits.max_linear,
        max_wz=safe_limits.max_angular,
    )


def stop_command(
    *,
    stamp_sec: float = 0.0,
    source: str = TELEOP_SOURCE,
) -> TwistCommand:
    return TwistCommand.zero(source=source, stamp_sec=stamp_sec)


def apply_teleop_key(
    key: object,
    *,
    speeds: TeleopSpeeds,
    limits: TeleopLimits,
    stamp_sec: float = 0.0,
    source: str = TELEOP_SOURCE,
) -> TeleopKeyResult:
    key_text = normalize_key(key)
    safe_limits = limits.sanitized()
    safe_speeds = speeds.sanitized()

    linear = clamp(safe_speeds.linear, 0.0, safe_limits.max_linear)
    angular = clamp(safe_speeds.angular, 0.0, safe_limits.max_angular)
    current_speeds = TeleopSpeeds(linear=linear, angular=angular)

    movement = {
        "w": (linear, 0.0, 0.0, "forward"),
        "s": (-linear, 0.0, 0.0, "backward"),
        "a": (0.0, linear, 0.0, "strafe_left"),
        "d": (0.0, -linear, 0.0, "strafe_right"),
        "q": (0.0, 0.0, angular, "rotate_left"),
        "e": (0.0, 0.0, -angular, "rotate_right"),
    }

    if key_text in movement:
        vx, vy, wz, label = movement[key_text]
        return TeleopKeyResult(
            handled=True,
            command=make_teleop_command(
                vx=vx,
                vy=vy,
                wz=wz,
                limits=safe_limits,
                stamp_sec=stamp_sec,
                source=source,
            ),
            speeds=current_speeds,
            label=label,
        )

    if key_text in {"x", " "}:
        return TeleopKeyResult(
            handled=True,
            command=stop_command(stamp_sec=stamp_sec, source=source),
            speeds=current_speeds,
            stop_requested=True,
            label="stop",
        )

    if key_text == "t":
        return _speed_result(
            current_speeds,
            linear=clamp(linear + safe_limits.linear_step, 0.0, safe_limits.max_linear),
            angular=angular,
            label="linear_up",
        )

    if key_text == "g":
        return _speed_result(
            current_speeds,
            linear=clamp(linear - safe_limits.linear_step, 0.0, safe_limits.max_linear),
            angular=angular,
            label="linear_down",
        )

    if key_text == "y":
        return _speed_result(
            current_speeds,
            linear=linear,
            angular=clamp(
                angular + safe_limits.angular_step,
                0.0,
                safe_limits.max_angular,
            ),
            label="angular_up",
        )

    if key_text == "h":
        return _speed_result(
            current_speeds,
            linear=linear,
            angular=clamp(
                angular - safe_limits.angular_step,
                0.0,
                safe_limits.max_angular,
            ),
            label="angular_down",
        )

    if key_text == "r":
        return TeleopKeyResult(
            handled=True,
            speeds=safe_limits.default_speeds(),
            speed_changed=True,
            label="reset_speeds",
        )

    if key_text == "p":
        return TeleopKeyResult(
            handled=True,
            speeds=current_speeds,
            help_requested=True,
            label="help",
        )

    return TeleopKeyResult(
        handled=False,
        speeds=current_speeds,
        label="ignored",
    )


def _speed_result(
    old_speeds: TeleopSpeeds,
    *,
    linear: float,
    angular: float,
    label: str,
) -> TeleopKeyResult:
    new_speeds = TeleopSpeeds(linear=linear, angular=angular).sanitized()

    return TeleopKeyResult(
        handled=True,
        speeds=new_speeds,
        speed_changed=(
            new_speeds.linear != old_speeds.linear
            or new_speeds.angular != old_speeds.angular
        ),
        label=label,
    )


__all__ = [
    "TELEOP_SOURCE",
    "TeleopKeyResult",
    "TeleopLimits",
    "TeleopSpeeds",
    "apply_teleop_key",
    "make_teleop_command",
    "normalize_key",
    "stop_command",
]
