#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Voxel layer readiness check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_disabled,
    make_error,
    make_ok,
    make_stale,
    make_warn,
)


# =============================================================================
# Voxel layer result
# =============================================================================
@dataclass(frozen=True)
class VoxelLayerResult:
    ok: bool
    enabled: bool = False
    stale: bool = True

    layer_name: str = "voxel_layer"
    pointcloud_topic: str = "/savo_edge/realsense/points"

    costmap_active: bool = False
    pointcloud_ok: bool = False
    tf_ok: bool = False
    clearing_enabled: bool = True
    marking_enabled: bool = True

    obstacle_max_range_m: float = 2.5
    obstacle_min_range_m: float = 0.15
    max_obstacle_height_m: float = 1.5
    min_obstacle_height_m: float = 0.05

    msg_count: int = 0
    age_s: Optional[float] = None
    stale_timeout_s: float = 2.0

    message: str = "Voxel layer disabled."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "enabled": self.enabled,
            "stale": self.stale,
            "layer_name": self.layer_name,
            "pointcloud_topic": self.pointcloud_topic,
            "costmap_active": self.costmap_active,
            "pointcloud_ok": self.pointcloud_ok,
            "tf_ok": self.tf_ok,
            "clearing_enabled": self.clearing_enabled,
            "marking_enabled": self.marking_enabled,
            "obstacle_max_range_m": self.obstacle_max_range_m,
            "obstacle_min_range_m": self.obstacle_min_range_m,
            "max_obstacle_height_m": self.max_obstacle_height_m,
            "min_obstacle_height_m": self.min_obstacle_height_m,
            "msg_count": self.msg_count,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_voxel_layer_ready(
    enabled: bool,
    costmap_active: bool = False,
    pointcloud_ok: bool = False,
    tf_ok: bool = False,
    clearing_enabled: bool = True,
    marking_enabled: bool = True,
    msg_count: int = 0,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 2.0,
    layer_name: str = "voxel_layer",
    pointcloud_topic: str = "/savo_edge/realsense/points",
    obstacle_max_range_m: float = 2.5,
    obstacle_min_range_m: float = 0.15,
    max_obstacle_height_m: float = 1.5,
    min_obstacle_height_m: float = 0.05,
    extra: Optional[Dict[str, Any]] = None,
) -> VoxelLayerResult:
    if not enabled:
        return VoxelLayerResult(
            ok=False,
            enabled=False,
            stale=True,
            layer_name=str(layer_name),
            pointcloud_topic=str(pointcloud_topic),
            message="Voxel layer disabled.",
            extra=dict(extra or {}),
        )

    count = max(0, int(msg_count))
    timeout = max(0.0, float(stale_timeout_s))
    stale = True if age_s is None else float(age_s) > timeout

    failures: list[str] = []

    if stale:
        failures.append("stale")

    if not costmap_active:
        failures.append("costmap_not_active")

    if not pointcloud_ok:
        failures.append("pointcloud_not_ready")

    if not tf_ok:
        failures.append("tf_not_ready")

    if not clearing_enabled and not marking_enabled:
        failures.append("marking_and_clearing_disabled")

    if obstacle_max_range_m <= obstacle_min_range_m:
        failures.append("invalid_obstacle_range")

    if max_obstacle_height_m <= min_obstacle_height_m:
        failures.append("invalid_obstacle_height")

    ok = not failures

    if ok:
        message = "Voxel layer ready."
    else:
        message = f"Voxel layer not ready: {', '.join(failures)}."

    return VoxelLayerResult(
        ok=ok,
        enabled=True,
        stale=stale,
        layer_name=str(layer_name),
        pointcloud_topic=str(pointcloud_topic),
        costmap_active=bool(costmap_active),
        pointcloud_ok=bool(pointcloud_ok),
        tf_ok=bool(tf_ok),
        clearing_enabled=bool(clearing_enabled),
        marking_enabled=bool(marking_enabled),
        obstacle_max_range_m=max(0.0, float(obstacle_max_range_m)),
        obstacle_min_range_m=max(0.0, float(obstacle_min_range_m)),
        max_obstacle_height_m=max(0.0, float(max_obstacle_height_m)),
        min_obstacle_height_m=max(0.0, float(min_obstacle_height_m)),
        msg_count=count,
        age_s=age_s,
        stale_timeout_s=timeout,
        message=message,
        extra={
            "failures": failures,
            **dict(extra or {}),
        },
    )


def evaluate_voxel_layer_from_inputs(
    enabled: bool,
    pointcloud_result_ok: bool,
    tf_result_ok: bool,
    costmap_active: bool,
    msg_count: int = 0,
    age_s: Optional[float] = None,
) -> VoxelLayerResult:
    return evaluate_voxel_layer_ready(
        enabled=enabled,
        costmap_active=costmap_active,
        pointcloud_ok=pointcloud_result_ok,
        tf_ok=tf_result_ok,
        msg_count=msg_count,
        age_s=age_s,
    )


def voxel_layer_result_to_diagnostic(
    result: VoxelLayerResult,
    required: bool = False,
) -> DiagnosticItem:
    values = result.to_dict()

    if not result.enabled:
        return make_disabled(
            "voxel_layer",
            result.message,
            required=required,
            values=values,
        )

    if result.ok:
        return make_ok(
            "voxel_layer",
            result.message,
            required=required,
            values=values,
        )

    if result.stale:
        return make_stale(
            "voxel_layer",
            result.message,
            required=required,
            values=values,
        )

    if required:
        return make_error(
            "voxel_layer",
            result.message,
            required=required,
            values=values,
        )

    return make_warn(
        "voxel_layer",
        result.message,
        required=required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    disabled = evaluate_voxel_layer_ready(enabled=False)

    ready = evaluate_voxel_layer_ready(
        enabled=True,
        costmap_active=True,
        pointcloud_ok=True,
        tf_ok=True,
        clearing_enabled=True,
        marking_enabled=True,
        msg_count=5,
        age_s=0.1,
    )

    print(disabled.to_json(indent=2))
    print(ready.to_json(indent=2))
    print(voxel_layer_result_to_diagnostic(ready).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "VoxelLayerResult",
    "evaluate_voxel_layer_ready",
    "evaluate_voxel_layer_from_inputs",
    "voxel_layer_result_to_diagnostic",
    "main",
]