#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map topic readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import (
    DEFAULT_MAP_STALE_TIMEOUT_S,
    FRAME_MAP,
    TOPIC_MAP,
)
from savo_mapping.models.map_quality import (
    MapQuality,
    calculate_map_quality,
    make_empty_map_quality,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Map topic result
# =============================================================================
@dataclass(frozen=True)
class MapTopicResult:
    ok: bool
    topic: str = TOPIC_MAP
    frame_id: str = FRAME_MAP
    stale: bool = True

    msg_count: int = 0
    rate_hz: float = 0.0
    age_s: Optional[float] = None

    width_cells: int = 0
    height_cells: int = 0
    resolution_m: float = 0.0
    cell_count: int = 0

    quality: MapQuality = field(default_factory=make_empty_map_quality)

    message: str = "Waiting for map topic."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "topic": self.topic,
            "frame_id": self.frame_id,
            "stale": self.stale,
            "msg_count": self.msg_count,
            "rate_hz": self.rate_hz,
            "age_s": self.age_s,
            "width_cells": self.width_cells,
            "height_cells": self.height_cells,
            "resolution_m": self.resolution_m,
            "cell_count": self.cell_count,
            "quality": self.quality.to_dict(),
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_map_topic_ready(
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    free_cells: int,
    occupied_cells: int,
    unknown_cells: int,
    frame_id: str = FRAME_MAP,
    topic: str = TOPIC_MAP,
    stale_timeout_s: float = DEFAULT_MAP_STALE_TIMEOUT_S,
    required: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> MapTopicResult:
    count = max(0, int(msg_count))
    rate = max(0.0, float(rate_hz))
    width = max(0, int(width_cells))
    height = max(0, int(height_cells))
    resolution = max(0.0, float(resolution_m))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout

    quality = calculate_map_quality(
        width_cells=width,
        height_cells=height,
        resolution_m=resolution,
        free_cells=free_cells,
        occupied_cells=occupied_cells,
        unknown_cells=unknown_cells,
        extra=dict(extra or {}),
    )

    failures: list[str] = []

    if count <= 0:
        failures.append("no_messages")

    if stale:
        failures.append("stale")

    if width <= 0 or height <= 0:
        failures.append("invalid_dimensions")

    if resolution <= 0.0:
        failures.append("invalid_resolution")

    if not quality.ok:
        failures.append("quality_weak")

    ok = not failures

    if ok:
        message = "Map topic ready."
    elif not required and count <= 0:
        message = "Map topic not publishing yet."
    else:
        message = f"Map topic not ready: {', '.join(failures)}."

    return MapTopicResult(
        ok=ok,
        topic=str(topic),
        frame_id=str(frame_id),
        stale=stale,
        msg_count=count,
        rate_hz=rate,
        age_s=age_s,
        width_cells=width,
        height_cells=height,
        resolution_m=resolution,
        cell_count=width * height,
        quality=quality,
        message=message,
        extra={
            "failures": failures,
            "required": required,
            "stale_timeout_s": timeout,
            **dict(extra or {}),
        },
    )


def evaluate_map_summary(
    summary: Dict[str, Any],
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_MAP,
    stale_timeout_s: float = DEFAULT_MAP_STALE_TIMEOUT_S,
    required: bool = False,
) -> MapTopicResult:
    return evaluate_map_topic_ready(
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        width_cells=int(summary.get("width_cells", 0)),
        height_cells=int(summary.get("height_cells", 0)),
        resolution_m=float(summary.get("resolution_m", 0.0)),
        free_cells=int(summary.get("free_cells", 0)),
        occupied_cells=int(summary.get("occupied_cells", 0)),
        unknown_cells=int(summary.get("unknown_cells", 0)),
        frame_id=str(summary.get("frame_id", FRAME_MAP)),
        topic=topic,
        stale_timeout_s=stale_timeout_s,
        required=required,
        extra={
            "cell_count": summary.get("cell_count", 0),
        },
    )


def evaluate_map_msg(
    msg: Any,
    msg_count: int,
    rate_hz: float,
    age_s: Optional[float],
    topic: str = TOPIC_MAP,
    stale_timeout_s: float = DEFAULT_MAP_STALE_TIMEOUT_S,
    required: bool = False,
) -> MapTopicResult:
    from savo_mapping.ros.adapters import occupancy_grid_summary

    return evaluate_map_summary(
        summary=occupancy_grid_summary(msg),
        msg_count=msg_count,
        rate_hz=rate_hz,
        age_s=age_s,
        topic=topic,
        stale_timeout_s=stale_timeout_s,
        required=required,
    )


def map_topic_result_to_diagnostic(
    result: MapTopicResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "map",
            result.message,
            required=required,
            values=values,
        )

    if result.stale and result.msg_count > 0:
        return make_stale(
            "map",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "map",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    result = evaluate_map_topic_ready(
        msg_count=3,
        rate_hz=1.0,
        age_s=0.2,
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        free_cells=5000,
        occupied_cells=500,
        unknown_cells=2500,
    )

    print(result.to_json(indent=2))
    print(map_topic_result_to_diagnostic(result).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "MapTopicResult",
    "evaluate_map_topic_ready",
    "evaluate_map_summary",
    "evaluate_map_msg",
    "map_topic_result_to_diagnostic",
    "main",
]