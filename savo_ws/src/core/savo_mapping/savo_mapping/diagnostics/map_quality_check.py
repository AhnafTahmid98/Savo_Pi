#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map quality diagnostic check. No ROS runtime required."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.models.map_quality import (
    MapQuality,
    MapQualityThresholds,
    calculate_map_quality,
    make_empty_map_quality,
    map_quality_from_occupancy_values,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    make_error,
    make_ok,
    make_warn,
)


# =============================================================================
# Map quality check result
# =============================================================================
@dataclass(frozen=True)
class MapQualityCheckResult:
    ok: bool
    quality: MapQuality = field(default_factory=make_empty_map_quality)
    thresholds: MapQualityThresholds = field(default_factory=MapQualityThresholds)

    required: bool = False
    message: str = "Map quality not checked."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "required": self.required,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "quality": self.quality.to_dict(),
            "thresholds": self.thresholds.to_dict(),
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Check logic
# =============================================================================
def evaluate_map_quality(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    free_cells: int,
    occupied_cells: int,
    unknown_cells: int,
    thresholds: Optional[MapQualityThresholds] = None,
    required: bool = False,
    extra: Optional[Dict[str, Any]] = None,
) -> MapQualityCheckResult:
    limits = thresholds or MapQualityThresholds()

    quality = calculate_map_quality(
        width_cells=width_cells,
        height_cells=height_cells,
        resolution_m=resolution_m,
        free_cells=free_cells,
        occupied_cells=occupied_cells,
        unknown_cells=unknown_cells,
        thresholds=limits,
        extra=dict(extra or {}),
    )

    if quality.ok:
        message = "Map quality ready."
    else:
        message = quality.message

    return MapQualityCheckResult(
        ok=quality.ok,
        quality=quality,
        thresholds=limits,
        required=bool(required),
        message=message,
        extra=dict(extra or {}),
    )


def evaluate_occupancy_values_quality(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    values: list[int],
    thresholds: Optional[MapQualityThresholds] = None,
    required: bool = False,
) -> MapQualityCheckResult:
    limits = thresholds or MapQualityThresholds()

    quality = map_quality_from_occupancy_values(
        width_cells=width_cells,
        height_cells=height_cells,
        resolution_m=resolution_m,
        values=values,
        thresholds=limits,
    )

    if quality.ok:
        message = "Map quality ready."
    else:
        message = quality.message

    return MapQualityCheckResult(
        ok=quality.ok,
        quality=quality,
        thresholds=limits,
        required=bool(required),
        message=message,
    )


def evaluate_map_quality_from_result(
    quality: MapQuality,
    thresholds: Optional[MapQualityThresholds] = None,
    required: bool = False,
) -> MapQualityCheckResult:
    limits = thresholds or MapQualityThresholds()

    if quality.ok:
        message = "Map quality ready."
    else:
        message = quality.message

    return MapQualityCheckResult(
        ok=quality.ok,
        quality=quality,
        thresholds=limits,
        required=bool(required),
        message=message,
    )


def map_quality_result_to_diagnostic(
    result: MapQualityCheckResult,
    required: Optional[bool] = None,
) -> DiagnosticItem:
    is_required = result.required if required is None else bool(required)
    values = result.to_dict()

    if result.ok:
        return make_ok(
            "map_quality",
            result.message,
            required=is_required,
            values=values,
        )

    if is_required:
        return make_error(
            "map_quality",
            result.message,
            required=is_required,
            values=values,
        )

    return make_warn(
        "map_quality",
        result.message,
        required=is_required,
        values=values,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    good = evaluate_map_quality(
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        free_cells=5000,
        occupied_cells=500,
        unknown_cells=2500,
    )

    weak = evaluate_map_quality(
        width_cells=10,
        height_cells=10,
        resolution_m=0.05,
        free_cells=20,
        occupied_cells=2,
        unknown_cells=78,
    )

    print(good.to_json(indent=2))
    print(weak.to_json(indent=2))
    print(map_quality_result_to_diagnostic(good).to_dict())


if __name__ == "__main__":
    main()


__all__ = [
    "MapQualityCheckResult",
    "evaluate_map_quality",
    "evaluate_occupancy_values_quality",
    "evaluate_map_quality_from_result",
    "map_quality_result_to_diagnostic",
    "main",
]