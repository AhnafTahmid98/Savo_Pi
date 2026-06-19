#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""RealSense pointcloud readiness check. No ROS runtime required."""

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
from savo_mapping.models.pointcloud_status import (
    PointcloudStatus,
    make_pointcloud_status,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_disabled,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Pointcloud readiness result
# =============================================================================
@dataclass(frozen=True)
class PointcloudReadyResult:
    ok: bool
    enabled: bool = False
    topic: str = TOPIC_REALSENSE_POINTS
    frame_id: str = FRAME_CAMERA_DEPTH
    stale: bool = True

    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None

    point_count: int = 0
    is_dense: bool = False
    width: int = 0
    height: int = 0

    status: PointcloudStatus = field(default_factory=PointcloudStatus)

    message: str = "Pointcloud disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "enabled": self.enabled,
            "topic": self.topic,
            "frame_id": self.frame_id,
            "stale": self.stale,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "point_count": self.point_count,
            "is_dense": self.is_dense,
            "width": self.width,
            "height": self.height,
            "status": self.status.to_dict(),
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_pointcloud_ready(
    enabled: bool,
    msg_count: int = 0,
    rate_hz: float = 0.0,
    age_s: Optional[float] = None,
    point_count: int = 0,
    frame_id: str = FRAME_CAMERA_DEPTH,
    topic: str = TOPIC_REALSENSE_POINTS,
    min_rate_hz: float = 3.0,
    min_points: int = 100,
    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    is_dense: bool = False,
    width: int = 0,
    height: int = 0,
    extra: Optional[Dict[str, Any]] = None,
) -> PointcloudReadyResult:
    if not enabled:
        status = make_pointcloud_status(enabled=False, topic=topic)

        return PointcloudReadyResult(
            ok=False,
            enabled=False,
            topic=str(topic),
            frame_id=str(frame_id),
            stale=True,
            status=status,
            message="Pointcloud disabled.",
            extra=dict(extra or {}),
        )

    count = max(0, int(msg_count))
    rate = max(0.0, float(rate_hz))
    points = max(0, int(point_count))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if count <= 0:
        failures.append("no_messages")

    if stale:
        failures.append("stale")

    if rate < min_rate_hz:
        failures.append("rate_low")

    if points < min_points:
        failures.append("point_count_low")

    if not str(frame_id).strip():
        failures.append("missing_frame_id")

    ok = not failures

    if ok:
        message = "Pointcloud ready."
    else:
        message = f"Pointcloud not ready: {', '.join(failures)}."

    status = make_pointcloud_status(
        enabled=True,
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        point_count=points,
        topic=topic,
        frame_id=frame_id,
        stale_timeout_s=timeout,
        extra={
            "min_rate_hz": min_rate_hz,
            "min_points": min_points,
            "is_dense": is_dense,
            "width": width,
            "height": height,
            **dict(extra or {}),
        },
    )

    return PointcloudReadyResult(
        ok=ok,
        enabled=True,
        topic=str(topic),
        frame_id=str(frame_id),
        stale=stale,
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        point_count=points,
        is_dense=bool(is_dense),
        width=max(0, int(width)),
        height=max(0, int(height)),
        status=status,
        message=message,
        extra={
            "failures": failures,
            "min_rate_hz": min_rate_hz,
            "min_points": min_points,
            "stale_timeout_s": timeout,
            **dict(extra or {}),
        },
    )


def evaluate_pointcloud_summary(
    summary: Dict[str, Any],
    enabled: bool,
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_REALSENSE_POINTS,
    min_rate_hz: float = 3.0,
    min_points: int = 100,
    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
) -> PointcloudReadyResult:
    return evaluate_pointcloud_ready(
        enabled=enabled,
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        point_count=int(summary.get("point_count", 0)),
        frame_id=str(summary.get("frame_id", FRAME_CAMERA_DEPTH)),
        topic=topic,
        min_rate_hz=min_rate_hz,
        min_points=min_points,
        stale_timeout_s=stale_timeout_s,
        is_dense=bool(summary.get("is_dense", False)),
        width=int(summary.get("width", 0)),
        height=int(summary.get("height", 0)),
        extra={
            "row_step": summary.get("row_step", 0),
            "point_step": summary.get("point_step", 0),
        },
    )


def evaluate_pointcloud_msg(
    msg: Any,
    enabled: bool,
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_REALSENSE_POINTS,
    min_rate_hz: float = 3.0,
    min_points: int = 100,
    stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
) -> PointcloudReadyResult:
    from savo_mapping.ros.adapters import pointcloud_summary

    return evaluate_pointcloud_summary(
        summary=pointcloud_summary(msg),
        enabled=enabled,
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        topic=topic,
        min_rate_hz=min_rate_hz,
        min_points=min_points,
        stale_timeout_s=stale_timeout_s,
    )


def pointcloud_result_to_diagnostic(
    result: PointcloudReadyResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if not result.enabled:
        return make_disabled(
            "pointcloud",
            result.message,
            required=required,
            values=values,
        )

    if result.ok:
        return make_ok(
            "pointcloud",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "pointcloud",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "pointcloud",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    disabled = evaluate_pointcloud_ready(enabled=False)

    ready = evaluate_pointcloud_ready(
        enabled=True,
        msg_count=20,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
        frame_id=FRAME_CAMERA_DEPTH,
        width=160,
        height=75,
    )

    print(disabled.to_json(indent=2))
    print(ready.to_json(indent=2))
    print(pointcloud_result_to_diagnostic(ready).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "PointcloudReadyResult",
    "evaluate_pointcloud_ready",
    "evaluate_pointcloud_summary",
    "evaluate_pointcloud_msg",
    "pointcloud_result_to_diagnostic",
    "main",
]