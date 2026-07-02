# -*- coding: utf-8 -*-

"""Head state models used by Python fallback tools and tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Optional

from savo_head.constants import (
    PAN_CENTER_DEG_DEFAULT,
    PAN_MAX_DEG_DEFAULT,
    PAN_MIN_DEG_DEFAULT,
    STATUS_DRYRUN,
    STATUS_OK,
    TILT_CENTER_DEG_DEFAULT,
    TILT_MAX_DEG_DEFAULT,
    TILT_MIN_DEG_DEFAULT,
    clamp_angle_deg,
)


MODE_MANUAL: Final[str] = "manual"
MODE_AUTO: Final[str] = "auto"
MODE_IDLE: Final[str] = "idle"
MODE_CENTERING: Final[str] = "centering"

VALID_HEAD_MODES: Final[tuple[str, ...]] = (
    MODE_IDLE,
    MODE_MANUAL,
    MODE_AUTO,
    MODE_CENTERING,
)


@dataclass(frozen=True)
class PanTiltLimits:
    pan_min_deg: int = PAN_MIN_DEG_DEFAULT
    pan_center_deg: int = PAN_CENTER_DEG_DEFAULT
    pan_max_deg: int = PAN_MAX_DEG_DEFAULT

    tilt_min_deg: int = TILT_MIN_DEG_DEFAULT
    tilt_center_deg: int = TILT_CENTER_DEG_DEFAULT
    tilt_max_deg: int = TILT_MAX_DEG_DEFAULT

    def clamp_pan(self, value: int) -> int:
        return max(self.pan_min_deg, min(self.pan_max_deg, int(value)))

    def clamp_tilt(self, value: int) -> int:
        return max(self.tilt_min_deg, min(self.tilt_max_deg, int(value)))

    def contains_pan(self, value: int) -> bool:
        return self.pan_min_deg <= int(value) <= self.pan_max_deg

    def contains_tilt(self, value: int) -> bool:
        return self.tilt_min_deg <= int(value) <= self.tilt_max_deg


@dataclass(frozen=True)
class PanTiltState:
    pan_deg: int = PAN_CENTER_DEG_DEFAULT
    tilt_deg: int = TILT_CENTER_DEG_DEFAULT
    mode: str = MODE_IDLE
    status: str = STATUS_OK
    stamp_s: float = 0.0
    source: str = "unknown"

    def normalized(self, limits: PanTiltLimits | None = None) -> "PanTiltState":
        lim = limits or PanTiltLimits()
        return PanTiltState(
            pan_deg=lim.clamp_pan(clamp_angle_deg(self.pan_deg)),
            tilt_deg=lim.clamp_tilt(clamp_angle_deg(self.tilt_deg)),
            mode=self.mode if self.mode in VALID_HEAD_MODES else MODE_IDLE,
            status=self.status,
            stamp_s=float(self.stamp_s),
            source=str(self.source),
        )

    def is_stale(self, now_s: float, timeout_s: float) -> bool:
        if self.stamp_s <= 0.0:
            return True
        return (float(now_s) - float(self.stamp_s)) > float(timeout_s)

    def as_tuple(self) -> tuple[int, int]:
        return (int(self.pan_deg), int(self.tilt_deg))

    def to_dict(self) -> dict:
        return {
            "pan_deg": int(self.pan_deg),
            "tilt_deg": int(self.tilt_deg),
            "mode": self.mode,
            "status": self.status,
            "stamp_s": float(self.stamp_s),
            "source": self.source,
        }


@dataclass(frozen=True)
class HeadRuntimeState:
    pan_tilt: PanTiltState = PanTiltState()
    hardware_status: str = STATUS_OK
    scan_status: str = MODE_IDLE
    tf_status: str = STATUS_OK
    camera_status: str = STATUS_DRYRUN
    apriltag_status: str = STATUS_DRYRUN
    last_error: Optional[str] = None

    def ok(self) -> bool:
        return (
            self.hardware_status == STATUS_OK
            and self.tf_status == STATUS_OK
            and self.last_error is None
        )

    def to_dict(self) -> dict:
        return {
            "pan_tilt": self.pan_tilt.to_dict(),
            "hardware_status": self.hardware_status,
            "scan_status": self.scan_status,
            "tf_status": self.tf_status,
            "camera_status": self.camera_status,
            "apriltag_status": self.apriltag_status,
            "ok": self.ok(),
            "last_error": self.last_error,
        }


def centered_state(stamp_s: float = 0.0, source: str = "center") -> PanTiltState:
    return PanTiltState(
        pan_deg=PAN_CENTER_DEG_DEFAULT,
        tilt_deg=TILT_CENTER_DEG_DEFAULT,
        mode=MODE_CENTERING,
        status=STATUS_OK,
        stamp_s=float(stamp_s),
        source=source,
    )


__all__ = [
    "MODE_MANUAL",
    "MODE_AUTO",
    "MODE_IDLE",
    "MODE_CENTERING",
    "VALID_HEAD_MODES",
    "PanTiltLimits",
    "PanTiltState",
    "HeadRuntimeState",
    "centered_state",
]
