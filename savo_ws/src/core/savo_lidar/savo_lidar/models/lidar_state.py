"""Runtime state model for the Robot Savo LiDAR stack."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    DEFAULT_FRAME_ID,
    DEFAULT_LIDAR_MODEL,
    DEFAULT_SCAN_TOPIC,
    STATUS_OFFLINE,
)


@dataclass
class LidarState:
    node: str
    status: str = STATUS_OFFLINE
    model: str = DEFAULT_LIDAR_MODEL
    backend: str = BACKEND_DRYRUN
    frame_id: str = DEFAULT_FRAME_ID
    scan_topic: str = DEFAULT_SCAN_TOPIC

    hardware_ok: bool = False
    scan_ok: bool = False
    driver_running: bool = False

    scan_count: int = 0
    last_scan_age_s: float | None = None
    scan_rate_hz: float = 0.0
    valid_ratio: float = 0.0

    message: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def mark_scan(
        self,
        *,
        scan_rate_hz: float,
        valid_ratio: float,
        last_scan_age_s: float = 0.0,
    ) -> None:
        self.scan_count += 1
        self.scan_ok = True
        self.last_scan_age_s = float(last_scan_age_s)
        self.scan_rate_hz = float(scan_rate_hz)
        self.valid_ratio = float(valid_ratio)

    def mark_stale(self, message: str = "scan stream stale") -> None:
        self.scan_ok = False
        self.message = str(message)

    def mark_hardware(self, ok: bool, message: str = "") -> None:
        self.hardware_ok = bool(ok)

        if message:
            self.message = str(message)

    def mark_driver_running(self, running: bool) -> None:
        self.driver_running = bool(running)

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)

        if not self.extra:
            data.pop("extra", None)

        return data