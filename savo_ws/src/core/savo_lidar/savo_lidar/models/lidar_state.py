# -*- coding: utf-8 -*-
"""Runtime state reported by the LiDAR driver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    DEFAULT_FRAME_ID,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_STATE_TOPIC,
    STATUS_ERROR,
    STATUS_OFFLINE,
    STATUS_OK,
    STATUS_STALE,
    STATUS_WARN,
)


@dataclass(frozen=True)
class LidarState:
    node: str = "lidar_driver_node"
    status: str = STATUS_OFFLINE
    backend: str = BACKEND_DRYRUN

    scan_topic: str = DEFAULT_SCAN_TOPIC
    state_topic: str = DEFAULT_STATE_TOPIC
    frame_id: str = DEFAULT_FRAME_ID

    connected: bool = False
    motor_running: bool = False
    publishing_scan: bool = False

    scan_count: int = 0
    last_scan_age_s: Optional[float] = None
    scan_rate_hz: float = 0.0

    valid_points: int = 0
    total_points: int = 0
    valid_ratio: float = 0.0

    min_range_m: Optional[float] = None
    max_range_m: Optional[float] = None
    front_min_m: Optional[float] = None

    error: str = ""
    detail: str = ""

    def is_ok(self) -> bool:
        return self.status == STATUS_OK

    def is_warn(self) -> bool:
        return self.status == STATUS_WARN

    def is_error(self) -> bool:
        return self.status == STATUS_ERROR

    def is_stale(self) -> bool:
        return self.status == STATUS_STALE

    def has_scan(self) -> bool:
        return self.scan_count > 0 and self.publishing_scan

    def to_dict(self) -> Dict[str, Any]:
        return {
            "node": self.node,
            "status": self.status,
            "backend": self.backend,
            "scan_topic": self.scan_topic,
            "state_topic": self.state_topic,
            "frame_id": self.frame_id,
            "connected": self.connected,
            "motor_running": self.motor_running,
            "publishing_scan": self.publishing_scan,
            "scan_count": self.scan_count,
            "last_scan_age_s": self.last_scan_age_s,
            "scan_rate_hz": self.scan_rate_hz,
            "valid_points": self.valid_points,
            "total_points": self.total_points,
            "valid_ratio": self.valid_ratio,
            "min_range_m": self.min_range_m,
            "max_range_m": self.max_range_m,
            "front_min_m": self.front_min_m,
            "error": self.error,
            "detail": self.detail,
        }


def make_lidar_state(**overrides: object) -> LidarState:
    return LidarState(**overrides)


__all__ = [
    "LidarState",
    "make_lidar_state",
]
