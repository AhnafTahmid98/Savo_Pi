#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frontier detection helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

from savo_mapping.models.exploration_status import ExplorationGoal


UNKNOWN_VALUE = -1
DEFAULT_OCCUPIED_THRESHOLD = 50


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution_m: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    def __post_init__(self) -> None:
        if self.width <= 0:
            raise ValueError("Grid width must be positive.")

        if self.height <= 0:
            raise ValueError("Grid height must be positive.")

        if self.resolution_m <= 0.0:
            raise ValueError("Grid resolution must be positive.")

    @property
    def cell_count(self) -> int:
        return self.width * self.height

    @property
    def width_m(self) -> float:
        return self.width * self.resolution_m

    @property
    def height_m(self) -> float:
        return self.height * self.resolution_m


@dataclass(frozen=True)
class FrontierCell:
    x: int
    y: int
    index: int
    world_x: float
    world_y: float

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "index": self.index,
            "world_x": self.world_x,
            "world_y": self.world_y,
        }


@dataclass(frozen=True)
class FrontierCluster:
    id: int
    cells: tuple[FrontierCell, ...]
    centroid_x: float
    centroid_y: float
    min_x: int
    min_y: int
    max_x: int
    max_y: int
    score: float
    frame_id: str = "map"

    @property
    def size_cells(self) -> int:
        return len(self.cells)

    @property
    def width_cells(self) -> int:
        return self.max_x - self.min_x + 1

    @property
    def height_cells(self) -> int:
        return self.max_y - self.min_y + 1

    def to_goal(self, reason: str = "frontier") -> ExplorationGoal:
        return ExplorationGoal(
            x=self.centroid_x,
            y=self.centroid_y,
            yaw=0.0,
            frame_id=self.frame_id,
            score=self.score,
            reason=reason,
        )

    def to_dict(self, *, include_cells: bool = False) -> dict:
        data = {
            "id": self.id,
            "size_cells": self.size_cells,
            "centroid_x": self.centroid_x,
            "centroid_y": self.centroid_y,
            "min_x": self.min_x,
            "min_y": self.min_y,
            "max_x": self.max_x,
            "max_y": self.max_y,
            "width_cells": self.width_cells,
            "height_cells": self.height_cells,
            "score": self.score,
            "frame_id": self.frame_id,
        }

        if include_cells:
            data["cells"] = [cell.to_dict() for cell in self.cells]

        return data


@dataclass(frozen=True)
class FrontierDetectionResult:
    ok: bool
    grid: GridSpec
    frontier_cells: tuple[FrontierCell, ...] = ()
    clusters: tuple[FrontierCluster, ...] = ()
    min_frontier_size_cells: int = 3
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD
    message: str = "Frontier detection not run."
    timestamp_s: float = field(default_factory=time.time)

    @property
    def frontier_count(self) -> int:
        return len(self.clusters)

    @property
    def cell_count(self) -> int:
        return len(self.frontier_cells)

    @property
    def best_cluster(self) -> Optional[FrontierCluster]:
        if not self.clusters:
            return None

        return max(self.clusters, key=lambda cluster: cluster.score)

    def goals(self) -> tuple[ExplorationGoal, ...]:
        return tuple(
            cluster.to_goal(reason="frontier")
            for cluster in self.clusters
        )

    def to_dict(self, *, include_cells: bool = False) -> dict:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "frontier_count": self.frontier_count,
            "cell_count": self.cell_count,
            "min_frontier_size_cells": self.min_frontier_size_cells,
            "occupied_threshold": self.occupied_threshold,
            "grid": {
                "width": self.grid.width,
                "height": self.grid.height,
                "resolution_m": self.grid.resolution_m,
                "origin_x": self.grid.origin_x,
                "origin_y": self.grid.origin_y,
                "origin_yaw": self.grid.origin_yaw,
                "frame_id": self.grid.frame_id,
            },
            "clusters": [
                cluster.to_dict(include_cells=include_cells)
                for cluster in self.clusters
            ],
        }

    def to_json(self, *, indent: Optional[int] = None, include_cells: bool = False) -> str:
        return json.dumps(
            self.to_dict(include_cells=include_cells),
            indent=indent,
            sort_keys=True,
        )


def validate_grid_values(grid: GridSpec, values: Sequence[int]) -> None:
    if len(values) != grid.cell_count:
        raise ValueError(
            f"Grid values length mismatch: expected {grid.cell_count}, got {len(values)}."
        )


def grid_index(grid: GridSpec, x: int, y: int) -> int:
    if not in_bounds(grid, x, y):
        raise IndexError(f"Grid cell out of bounds: ({x}, {y}).")

    return y * grid.width + x


def in_bounds(grid: GridSpec, x: int, y: int) -> bool:
    return 0 <= x < grid.width and 0 <= y < grid.height


def cell_value(grid: GridSpec, values: Sequence[int], x: int, y: int) -> int:
    return int(values[grid_index(grid, x, y)])


def is_unknown(value: int) -> bool:
    return int(value) == UNKNOWN_VALUE


def is_free(value: int, occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD) -> bool:
    value = int(value)
    return 0 <= value < int(occupied_threshold)


def is_occupied(value: int, occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD) -> bool:
    return int(value) >= int(occupied_threshold)


def grid_to_world(grid: GridSpec, x: int, y: int) -> tuple[float, float]:
    local_x = (float(x) + 0.5) * grid.resolution_m
    local_y = (float(y) + 0.5) * grid.resolution_m

    if abs(grid.origin_yaw) < 1.0e-12:
        return grid.origin_x + local_x, grid.origin_y + local_y

    cos_yaw = math.cos(grid.origin_yaw)
    sin_yaw = math.sin(grid.origin_yaw)

    world_x = grid.origin_x + local_x * cos_yaw - local_y * sin_yaw
    world_y = grid.origin_y + local_x * sin_yaw + local_y * cos_yaw

    return world_x, world_y


def iter_neighbors(
    grid: GridSpec,
    x: int,
    y: int,
    *,
    diagonal: bool = False,
) -> Iterable[tuple[int, int]]:
    offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    if diagonal:
        offsets.extend(((-1, -1), (-1, 1), (1, -1), (1, 1)))

    for dx, dy in offsets:
        nx = x + dx
        ny = y + dy

        if in_bounds(grid, nx, ny):
            yield nx, ny


def is_frontier_cell(
    grid: GridSpec,
    values: Sequence[int],
    x: int,
    y: int,
    *,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
) -> bool:
    value = cell_value(grid, values, x, y)

    if not is_free(value, occupied_threshold):
        return False

    return any(
        is_unknown(cell_value(grid, values, nx, ny))
        for nx, ny in iter_neighbors(grid, x, y)
    )


def find_frontier_cells(
    grid: GridSpec,
    values: Sequence[int],
    *,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
) -> tuple[FrontierCell, ...]:
    validate_grid_values(grid, values)

    cells: list[FrontierCell] = []

    for y in range(grid.height):
        for x in range(grid.width):
            if not is_frontier_cell(
                grid,
                values,
                x,
                y,
                occupied_threshold=occupied_threshold,
            ):
                continue

            world_x, world_y = grid_to_world(grid, x, y)

            cells.append(
                FrontierCell(
                    x=x,
                    y=y,
                    index=grid_index(grid, x, y),
                    world_x=world_x,
                    world_y=world_y,
                )
            )

    return tuple(cells)


def cluster_frontier_cells(
    grid: GridSpec,
    frontier_cells: Sequence[FrontierCell],
    *,
    min_frontier_size_cells: int = 3,
    diagonal: bool = True,
) -> tuple[FrontierCluster, ...]:
    by_xy = {
        (cell.x, cell.y): cell
        for cell in frontier_cells
    }

    visited: set[tuple[int, int]] = set()
    clusters: list[FrontierCluster] = []

    for seed in frontier_cells:
        seed_key = (seed.x, seed.y)

        if seed_key in visited:
            continue

        queue: deque[tuple[int, int]] = deque([seed_key])
        visited.add(seed_key)

        group: list[FrontierCell] = []

        while queue:
            current = queue.popleft()
            cell = by_xy[current]
            group.append(cell)

            for nx, ny in iter_neighbors(grid, cell.x, cell.y, diagonal=diagonal):
                key = (nx, ny)

                if key in visited or key not in by_xy:
                    continue

                visited.add(key)
                queue.append(key)

        if len(group) < int(min_frontier_size_cells):
            continue

        clusters.append(
            _make_cluster(
                cluster_id=len(clusters),
                cells=group,
                frame_id=grid.frame_id,
            )
        )

    return tuple(sorted(clusters, key=lambda cluster: cluster.score, reverse=True))


def detect_frontiers(
    width: int,
    height: int,
    resolution_m: float,
    values: Sequence[int],
    *,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
    frame_id: str = "map",
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
    min_frontier_size_cells: int = 3,
    diagonal_clustering: bool = True,
) -> FrontierDetectionResult:
    grid = GridSpec(
        width=int(width),
        height=int(height),
        resolution_m=float(resolution_m),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        origin_yaw=float(origin_yaw),
        frame_id=str(frame_id),
    )

    cells = find_frontier_cells(
        grid,
        values,
        occupied_threshold=occupied_threshold,
    )
    clusters = cluster_frontier_cells(
        grid,
        cells,
        min_frontier_size_cells=max(1, int(min_frontier_size_cells)),
        diagonal=bool(diagonal_clustering),
    )

    ok = bool(clusters)

    if ok:
        message = f"Detected {len(clusters)} frontier cluster(s)."
    else:
        message = "No valid frontier clusters detected."

    return FrontierDetectionResult(
        ok=ok,
        grid=grid,
        frontier_cells=cells,
        clusters=clusters,
        min_frontier_size_cells=max(1, int(min_frontier_size_cells)),
        occupied_threshold=int(occupied_threshold),
        message=message,
    )


def detect_frontiers_from_summary(
    summary: dict,
    values: Sequence[int],
    *,
    min_frontier_size_cells: int = 3,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
) -> FrontierDetectionResult:
    origin = summary.get("origin", {})

    return detect_frontiers(
        width=int(summary["width_cells"]),
        height=int(summary["height_cells"]),
        resolution_m=float(summary["resolution_m"]),
        values=values,
        origin_x=float(origin.get("x", 0.0)),
        origin_y=float(origin.get("y", 0.0)),
        origin_yaw=float(origin.get("yaw", 0.0)),
        frame_id=str(summary.get("frame_id", "map")),
        occupied_threshold=occupied_threshold,
        min_frontier_size_cells=min_frontier_size_cells,
    )


def _make_cluster(
    cluster_id: int,
    cells: Sequence[FrontierCell],
    frame_id: str,
) -> FrontierCluster:
    xs = [cell.x for cell in cells]
    ys = [cell.y for cell in cells]
    world_xs = [cell.world_x for cell in cells]
    world_ys = [cell.world_y for cell in cells]

    size = len(cells)
    score = float(size)

    return FrontierCluster(
        id=int(cluster_id),
        cells=tuple(cells),
        centroid_x=sum(world_xs) / size,
        centroid_y=sum(world_ys) / size,
        min_x=min(xs),
        min_y=min(ys),
        max_x=max(xs),
        max_y=max(ys),
        score=score,
        frame_id=frame_id,
    )


def main() -> None:
    width = 8
    height = 6
    values = [-1] * (width * height)

    for y in range(1, 5):
        for x in range(1, 5):
            values[y * width + x] = 0

    result = detect_frontiers(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        min_frontier_size_cells=2,
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "DEFAULT_OCCUPIED_THRESHOLD",
    "UNKNOWN_VALUE",
    "GridSpec",
    "FrontierCell",
    "FrontierCluster",
    "FrontierDetectionResult",
    "cell_value",
    "cluster_frontier_cells",
    "detect_frontiers",
    "detect_frontiers_from_summary",
    "find_frontier_cells",
    "grid_index",
    "grid_to_world",
    "in_bounds",
    "is_free",
    "is_frontier_cell",
    "is_occupied",
    "is_unknown",
    "iter_neighbors",
    "main",
    "validate_grid_values",
]
