# -*- coding: utf-8 -*-

"""Pan-tilt command models for fallback tools and tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Optional

from savo_head.constants import (
    MANUAL_STEP_DEG_DEFAULT,
    PAN_CENTER_DEG_DEFAULT,
    TILT_CENTER_DEG_DEFAULT,
)
from savo_head.models.head_state import (
    MODE_CENTERING,
    PanTiltLimits,
    PanTiltState,
)


COMMAND_ABSOLUTE: Final[str] = "absolute"
COMMAND_DELTA: Final[str] = "delta"
COMMAND_CENTER: Final[str] = "center"
COMMAND_HOLD: Final[str] = "hold"
COMMAND_STOP: Final[str] = "stop"

SOURCE_MANUAL: Final[str] = "manual"
SOURCE_SCAN: Final[str] = "scan"
SOURCE_SYSTEM: Final[str] = "system"
SOURCE_FALLBACK: Final[str] = "fallback"

VALID_COMMAND_TYPES: Final[tuple[str, ...]] = (
    COMMAND_ABSOLUTE,
    COMMAND_DELTA,
    COMMAND_CENTER,
    COMMAND_HOLD,
    COMMAND_STOP,
)

VALID_COMMAND_SOURCES: Final[tuple[str, ...]] = (
    SOURCE_MANUAL,
    SOURCE_SCAN,
    SOURCE_SYSTEM,
    SOURCE_FALLBACK,
)


@dataclass(frozen=True)
class PanTiltCommand:
    command_type: str = COMMAND_ABSOLUTE

    pan_deg: Optional[int] = None
    tilt_deg: Optional[int] = None

    pan_delta_deg: int = 0
    tilt_delta_deg: int = 0

    source: str = SOURCE_MANUAL
    stamp_s: float = 0.0
    priority: int = 0
    reason: str = ""

    def normalized(self, limits: PanTiltLimits | None = None) -> "PanTiltCommand":
        lim = limits or PanTiltLimits()

        command_type = (
            self.command_type if self.command_type in VALID_COMMAND_TYPES else COMMAND_HOLD
        )
        source = self.source if self.source in VALID_COMMAND_SOURCES else SOURCE_SYSTEM

        pan = None if self.pan_deg is None else lim.clamp_pan(self.pan_deg)
        tilt = None if self.tilt_deg is None else lim.clamp_tilt(self.tilt_deg)

        return PanTiltCommand(
            command_type=command_type,
            pan_deg=pan,
            tilt_deg=tilt,
            pan_delta_deg=int(self.pan_delta_deg),
            tilt_delta_deg=int(self.tilt_delta_deg),
            source=source,
            stamp_s=float(self.stamp_s),
            priority=int(self.priority),
            reason=str(self.reason),
        )

    def validation_errors(self) -> list[str]:
        errors: list[str] = []

        if self.command_type not in VALID_COMMAND_TYPES:
            errors.append(f"invalid command_type: {self.command_type!r}")

        if self.source not in VALID_COMMAND_SOURCES:
            errors.append(f"invalid source: {self.source!r}")

        if self.command_type == COMMAND_ABSOLUTE:
            if self.pan_deg is None and self.tilt_deg is None:
                errors.append("absolute command requires pan_deg or tilt_deg")

        if self.command_type == COMMAND_DELTA:
            if self.pan_delta_deg == 0 and self.tilt_delta_deg == 0:
                errors.append("delta command requires non-zero pan_delta_deg or tilt_delta_deg")

        return errors

    def is_valid(self) -> bool:
        return not self.validation_errors()

    def target_from_state(
        self,
        current: PanTiltState,
        limits: PanTiltLimits | None = None,
    ) -> PanTiltState:
        lim = limits or PanTiltLimits()
        cmd = self.normalized(lim)

        mode = current.mode

        if cmd.command_type == COMMAND_CENTER:
            pan = PAN_CENTER_DEG_DEFAULT
            tilt = TILT_CENTER_DEG_DEFAULT
            mode = MODE_CENTERING

        elif cmd.command_type in (COMMAND_HOLD, COMMAND_STOP):
            pan = current.pan_deg
            tilt = current.tilt_deg

        elif cmd.command_type == COMMAND_DELTA:
            pan = current.pan_deg + cmd.pan_delta_deg
            tilt = current.tilt_deg + cmd.tilt_delta_deg

        else:
            pan = current.pan_deg if cmd.pan_deg is None else cmd.pan_deg
            tilt = current.tilt_deg if cmd.tilt_deg is None else cmd.tilt_deg

        return PanTiltState(
            pan_deg=lim.clamp_pan(pan),
            tilt_deg=lim.clamp_tilt(tilt),
            mode=mode,
            status=current.status,
            stamp_s=cmd.stamp_s,
            source=cmd.source,
        )

    def to_dict(self) -> dict:
        return {
            "command_type": self.command_type,
            "pan_deg": self.pan_deg,
            "tilt_deg": self.tilt_deg,
            "pan_delta_deg": int(self.pan_delta_deg),
            "tilt_delta_deg": int(self.tilt_delta_deg),
            "source": self.source,
            "stamp_s": float(self.stamp_s),
            "priority": int(self.priority),
            "reason": self.reason,
        }


def absolute_command(
    pan_deg: int | None = None,
    tilt_deg: int | None = None,
    *,
    source: str = SOURCE_MANUAL,
    stamp_s: float = 0.0,
    reason: str = "",
) -> PanTiltCommand:
    return PanTiltCommand(
        command_type=COMMAND_ABSOLUTE,
        pan_deg=pan_deg,
        tilt_deg=tilt_deg,
        source=source,
        stamp_s=stamp_s,
        reason=reason,
    )


def delta_command(
    pan_delta_deg: int = 0,
    tilt_delta_deg: int = 0,
    *,
    source: str = SOURCE_MANUAL,
    stamp_s: float = 0.0,
    reason: str = "",
) -> PanTiltCommand:
    return PanTiltCommand(
        command_type=COMMAND_DELTA,
        pan_delta_deg=pan_delta_deg,
        tilt_delta_deg=tilt_delta_deg,
        source=source,
        stamp_s=stamp_s,
        reason=reason,
    )


def center_command(
    *,
    source: str = SOURCE_SYSTEM,
    stamp_s: float = 0.0,
    reason: str = "center",
) -> PanTiltCommand:
    return PanTiltCommand(
        command_type=COMMAND_CENTER,
        source=source,
        stamp_s=stamp_s,
        priority=10,
        reason=reason,
    )


def hold_command(
    *,
    source: str = SOURCE_SYSTEM,
    stamp_s: float = 0.0,
    reason: str = "hold",
) -> PanTiltCommand:
    return PanTiltCommand(
        command_type=COMMAND_HOLD,
        source=source,
        stamp_s=stamp_s,
        priority=5,
        reason=reason,
    )


def stop_command(
    *,
    source: str = SOURCE_SYSTEM,
    stamp_s: float = 0.0,
    reason: str = "stop",
) -> PanTiltCommand:
    return PanTiltCommand(
        command_type=COMMAND_STOP,
        source=source,
        stamp_s=stamp_s,
        priority=20,
        reason=reason,
    )


def manual_step_command(key: str, step_deg: int = MANUAL_STEP_DEG_DEFAULT) -> PanTiltCommand:
    key_norm = str(key).strip().lower()
    step = int(step_deg)

    if key_norm == "a":
        return delta_command(pan_delta_deg=-step, reason="manual_left")
    if key_norm == "d":
        return delta_command(pan_delta_deg=step, reason="manual_right")
    if key_norm == "w":
        return delta_command(tilt_delta_deg=step, reason="manual_up")
    if key_norm == "s":
        return delta_command(tilt_delta_deg=-step, reason="manual_down")
    if key_norm == "c":
        return center_command(source=SOURCE_MANUAL, reason="manual_center")
    if key_norm == " ":
        return hold_command(source=SOURCE_MANUAL, reason="manual_hold")

    return hold_command(source=SOURCE_MANUAL, reason=f"ignored_key:{key_norm}")


__all__ = [
    "COMMAND_ABSOLUTE",
    "COMMAND_DELTA",
    "COMMAND_CENTER",
    "COMMAND_HOLD",
    "COMMAND_STOP",
    "SOURCE_MANUAL",
    "SOURCE_SCAN",
    "SOURCE_SYSTEM",
    "SOURCE_FALLBACK",
    "VALID_COMMAND_TYPES",
    "VALID_COMMAND_SOURCES",
    "PanTiltCommand",
    "absolute_command",
    "delta_command",
    "center_command",
    "hold_command",
    "stop_command",
    "manual_step_command",
]
