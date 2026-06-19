#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pointcloud status model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import (
    DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    FRAME_CAMERA_DEPTH,
    TOPIC_REALSENSE_POINTS,
)


# =============================================================================
# Pointcloud status
# =============================================================================
@dataclass(frozen=True)
class PointcloudStatus:
    enabled: bool = False
    ok: bool = False
    stale: bool = True
    topic: str = TOPIC_REALSENSE_POINTS
    frame_id: str = FRAME_CAMERA_DEPTH

    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None
    point_count: Optional[int] = None

    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S
    message: str = "Pointcloud disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "stale": self.stale,
            "topic": self.topic,
            "frame_id": self.frame_id,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "point_count": self.point_count,
            "stale_timeout_s": self.stale_timeout_s,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_pointcloud_disabled_status(
    topic: str = TOPIC_REALSENSE_POINTS,
) -> PointcloudStatus:
    return PointcloudStatus(
        enabled=False,
        ok=False,
        stale=True,
        topic=topic,
        message="Pointcloud disabled.",
    )


def make_pointcloud_status(
    enabled: bool,
    msg_count: int = 0,
    rate_hz: float = 0.0,
    age_s: Optional[float] = None,
    point_count: Optional[int] = None,
    topic: str = TOPIC_REALSENSE_POINTS,
    frame_id: str = FRAME_CAMERA_DEPTH,
    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    extra: Optional[Dict[str, Any]] = None,
) -> PointcloudStatus:
    if not enabled:
        return make_pointcloud_disabled_status(topic=topic)

    count = max(0, int(msg_count))
    rate = max(0.0, float(rate_hz))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout
    ok = count > 0 and not stale

    if ok:
        message = "Pointcloud OK."
    elif count <= 0:
        message = "Waiting for pointcloud."
    else:
        message = "Pointcloud stale."

    return PointcloudStatus(
        enabled=True,
        ok=ok,
        stale=stale,
        topic=str(topic),
        frame_id=str(frame_id),
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        point_count=point_count,
        stale_timeout_s=timeout,
        message=message,
        extra=dict(extra or {}),
    )


def pointcloud_status_from_dict(data: Dict[str, Any]) -> PointcloudStatus:
    return PointcloudStatus(
        enabled=bool(data.get("enabled", False)),
        ok=bool(data.get("ok", False)),
        stale=bool(data.get("stale", True)),
        topic=str(data.get("topic", TOPIC_REALSENSE_POINTS)),
        frame_id=str(data.get("frame_id", FRAME_CAMERA_DEPTH)),
        msg_count=max(0, int(data.get("msg_count", 0))),
        rate_hz=max(0.0, float(data.get("rate_hz", 0.0))),
        age_s=data.get("age_s"),
        point_count=data.get("point_count"),
        stale_timeout_s=max(
            0.0,
            float(data.get("stale_timeout_s", DEFAULT_POINTCLOUD_STALE_TIMEOUT_S)),
        ),
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    print(make_pointcloud_disabled_status().to_json(indent=2))
    print(
        make_pointcloud_status(
            enabled=True,
            msg_count=20,
            rate_hz=8.0,
            age_s=0.08,
            point_count=12000,
        ).to_json(indent=2)
    )


if __name__ == "__main__":
    main()


__all__ = [
    "PointcloudStatus",
    "make_pointcloud_disabled_status",
    "make_pointcloud_status",
    "pointcloud_status_from_dict",
    "main",
]