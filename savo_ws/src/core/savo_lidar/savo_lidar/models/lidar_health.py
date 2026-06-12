"""Health model for Robot Savo's LiDAR hardware and scan stream."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import (
    STATUS_ERROR,
    STATUS_OFFLINE,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)


@dataclass
class LidarHealth:
    status: str = STATUS_OFFLINE
    hardware_ok: bool = False
    scan_ok: bool = False
    stale: bool = True

    scan_rate_hz: float = 0.0
    valid_ratio: float = 0.0
    last_scan_age_s: float | None = None

    fault_latched: bool = False
    fault_reason: str = ""
    message: str = ""

    def update_from_scan(
        self,
        *,
        scan_rate_hz: float,
        valid_ratio: float,
        stale: bool,
        hardware_ok: bool = True,
        last_scan_age_s: float | None = 0.0,
    ) -> None:
        self.hardware_ok = bool(hardware_ok)
        self.scan_ok = not bool(stale) and valid_ratio > 0.0
        self.stale = bool(stale)
        self.scan_rate_hz = float(scan_rate_hz)
        self.valid_ratio = float(valid_ratio)
        self.last_scan_age_s = last_scan_age_s
        self._refresh_status()

    def latch_fault(self, reason: str) -> None:
        self.fault_latched = True
        self.fault_reason = str(reason)
        self.message = str(reason)
        self.status = STATUS_ERROR

    def clear_fault(self) -> None:
        self.fault_latched = False
        self.fault_reason = ""
        self._refresh_status()

    def mark_offline(self, message: str = "LiDAR offline") -> None:
        self.hardware_ok = False
        self.scan_ok = False
        self.stale = True
        self.message = str(message)
        self.status = STATUS_OFFLINE

    def _refresh_status(self) -> None:
        if self.fault_latched:
            self.status = STATUS_ERROR
            return

        if not self.hardware_ok:
            self.status = STATUS_OFFLINE
            return

        if self.stale:
            self.status = STATUS_STALE
            return

        if not self.scan_ok:
            self.status = STATUS_WARN
            return

        self.status = STATUS_OK

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)