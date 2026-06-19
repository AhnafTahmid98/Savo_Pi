#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map quality model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import (
    DEFAULT_MAX_UNKNOWN_RATIO,
    DEFAULT_MIN_KNOWN_RATIO,
    DEFAULT_MIN_MAP_HEIGHT_M,
    DEFAULT_MIN_MAP_WIDTH_M,
    DEFAULT_MIN_OCCUPIED_CELLS,
)


# =============================================================================
# Map quality thresholds
# =============================================================================
@dataclass(frozen=True)
class MapQualityThresholds:
    min_known_ratio: float = DEFAULT_MIN_KNOWN_RATIO
    max_unknown_ratio: float = DEFAULT_MAX_UNKNOWN_RATIO
    min_occupied_cells: int = DEFAULT_MIN_OCCUPIED_CELLS
    min_map_width_m: float = DEFAULT_MIN_MAP_WIDTH_M
    min_map_height_m: float = DEFAULT_MIN_MAP_HEIGHT_M

    def to_dict(self) -> dict:
        return {
            "min_known_ratio": self.min_known_ratio,
            "max_unknown_ratio": self.max_unknown_ratio,
            "min_occupied_cells": self.min_occupied_cells,
            "min_map_width_m": self.min_map_width_m,
            "min_map_height_m": self.min_map_height_m,
        }


# =============================================================================
# Map quality result
# =============================================================================
@dataclass(frozen=True)
class MapQuality:
    width_cells: int = 0
    height_cells: int = 0
    resolution_m: float = 0.0

    free_cells: int = 0
    occupied_cells: int = 0
    unknown_cells: int = 0

    known_ratio: float = 0.0
    unknown_ratio: float = 1.0

    width_m: float = 0.0
    height_m: float = 0.0

    ok: bool = False
    message: str = "No map data."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "width_cells": self.width_cells,
            "height_cells": self.height_cells,
            "resolution_m": self.resolution_m,
            "width_m": self.width_m,
            "height_m": self.height_m,
            "free_cells": self.free_cells,
            "occupied_cells": self.occupied_cells,
            "unknown_cells": self.unknown_cells,
            "known_ratio": self.known_ratio,
            "unknown_ratio": self.unknown_ratio,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_empty_map_quality(message: str = "No map data.") -> MapQuality:
    return MapQuality(
        ok=False,
        message=message,
    )


def calculate_map_quality(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    free_cells: int,
    occupied_cells: int,
    unknown_cells: int,
    thresholds: Optional[MapQualityThresholds] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> MapQuality:
    limits = thresholds or MapQualityThresholds()

    width = max(0, int(width_cells))
    height = max(0, int(height_cells))
    resolution = max(0.0, float(resolution_m))

    free = max(0, int(free_cells))
    occupied = max(0, int(occupied_cells))
    unknown = max(0, int(unknown_cells))

    total = max(1, width * height)
    known = free + occupied

    known_ratio = min(1.0, max(0.0, known / total))
    unknown_ratio = min(1.0, max(0.0, unknown / total))

    width_m = width * resolution
    height_m = height * resolution

    failures: list[str] = []

    if known_ratio < limits.min_known_ratio:
        failures.append("known_ratio_low")

    if unknown_ratio > limits.max_unknown_ratio:
        failures.append("unknown_ratio_high")

    if occupied < limits.min_occupied_cells:
        failures.append("occupied_cells_low")

    if width_m < limits.min_map_width_m:
        failures.append("map_width_low")

    if height_m < limits.min_map_height_m:
        failures.append("map_height_low")

    ok = not failures

    message = "Map quality OK." if ok else f"Map quality weak: {', '.join(failures)}."

    return MapQuality(
        width_cells=width,
        height_cells=height,
        resolution_m=resolution,
        free_cells=free,
        occupied_cells=occupied,
        unknown_cells=unknown,
        known_ratio=known_ratio,
        unknown_ratio=unknown_ratio,
        width_m=width_m,
        height_m=height_m,
        ok=ok,
        message=message,
        extra={
            "failures": failures,
            "thresholds": limits.to_dict(),
            **dict(extra or {}),
        },
    )


def map_quality_from_occupancy_values(
    width_cells: int,
    height_cells: int,
    resolution_m: float,
    values: list[int],
    thresholds: Optional[MapQualityThresholds] = None,
) -> MapQuality:
    free = 0
    occupied = 0
    unknown = 0

    for value in values:
        cell = int(value)

        if cell < 0:
            unknown += 1
        elif cell >= 50:
            occupied += 1
        else:
            free += 1

    return calculate_map_quality(
        width_cells=width_cells,
        height_cells=height_cells,
        resolution_m=resolution_m,
        free_cells=free,
        occupied_cells=occupied,
        unknown_cells=unknown,
        thresholds=thresholds,
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    values = [-1] * 20 + [0] * 120 + [100] * 60
    quality = map_quality_from_occupancy_values(
        width_cells=20,
        height_cells=10,
        resolution_m=0.05,
        values=values,
    )

    print(quality.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "MapQualityThresholds",
    "MapQuality",
    "make_empty_map_quality",
    "calculate_map_quality",
    "map_quality_from_occupancy_values",
    "main",
]