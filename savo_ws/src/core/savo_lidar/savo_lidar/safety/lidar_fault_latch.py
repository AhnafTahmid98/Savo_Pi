"""Fault latch for LiDAR errors that should stay visible until cleared."""

from __future__ import annotations

from dataclasses import dataclass

from savo_lidar.constants import STATUS_ERROR, STATUS_OK


@dataclass
class LidarFaultLatch:
    latched: bool = False
    reason: str = ""
    fault_count: int = 0

    def latch(self, reason: str) -> None:
        reason = str(reason).strip() or "LiDAR fault"

        self.latched = True
        self.reason = reason
        self.fault_count += 1

    def clear(self) -> None:
        self.latched = False
        self.reason = ""

    def status(self) -> str:
        if self.latched:
            return STATUS_ERROR

        return STATUS_OK

    def to_dict(self) -> dict[str, object]:
        return {
            "latched": self.latched,
            "reason": self.reason,
            "fault_count": self.fault_count,
            "status": self.status(),
        }