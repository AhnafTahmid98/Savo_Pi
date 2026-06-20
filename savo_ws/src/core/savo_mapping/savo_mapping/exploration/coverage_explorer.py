#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Coverage exploration helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from savo_mapping.exploration.frontier_detector import (
    DEFAULT_OCCUPIED_THRESHOLD,
    GridSpec,
    grid_index,
    grid_to_world,
    is_free,
    validate_grid_values,
)
from savo_mapping.exploration.goal_selector import GoalBlacklistEntry, RobotPose2D
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class CoveragePolicy:
    min_goal_distance_m: float = 0.40
    max_goal_distance_m: float = 10.0
    sample_step_cells: int = 4
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD
    max_candidates: int = 100
    prefer_farther: bool = True

    def __post_init__(self) -> None:
        if self.min_goal_distance_m < 0.0:
            raise ValueError("min_goal_distance_m must be non-negative.")

        if self.max_goal_distance_m <= 0.0:
            raise ValueError("max_goal_distance_m must be positive.")

        if self.min_goal_distance_m > self.max_goal_distance_m:
            raise ValueError("min_goal_distance_m cannot be greater than max_goal_distance_m.")

        if self.sample_step_cells <= 0:
            raise ValueError("sample_step_cells must be positive.")

        if self.max_candidates <= 0:
            raise ValueError("max_candidates must be positive.")

    def to_dict(self) -> dict:
        return {
            "min_goal_distance_m": self.min_goal_distance_m,
            "max_goal_distance_m": self.max_goal_distance_m,
            "sample_step_cells": self.sample_step_cells,
            "occupied_threshold": self.occupied_threshold,
            "max_candidates": self.max_candidates,
            "prefer_farther": self.prefer_farther,
        }


@dataclass(frozen=True)
class CoverageCandidate:
    x_cell: int
    y_cell: int
    index: int
    world_x: float
    world_y: float
    distance_m: float
    score: float
    rejected: bool = False
    rejection_reason: str = ""

    def to_goal(self) -> ExplorationGoal:
        return ExplorationGoal(
            x=self.world_x,
            y=self.world_y,
            yaw=0.0,
            frame_id="map",
            score=self.score,
            reason="coverage_selected",
        )

    def to_dict(self) -> dict:
        return {
            "x_cell": self.x_cell,
            "y_cell": self.y_cell,
            "index": self.index,
            "world_x": self.world_x,
            "world_y": self.world_y,
            "distance_m": self.distance_m,
            "score": self.score,
            "rejected": self.rejected,
            "rejection_reason": self.rejection_reason,
        }


@dataclass(frozen=True)
class CoverageResult:
    ok: bool
    selected_goal: Optional[ExplorationGoal] = None
    candidates: tuple[CoverageCandidate, ...] = ()
    rejected_candidates: tuple[CoverageCandidate, ...] = ()
    policy: CoveragePolicy = field(default_factory=CoveragePolicy)
    message: str = "Coverage exploration not run."
    timestamp_s: float = field(default_factory=time.time)

    @property
    def candidate_count(self) -> int:
        return len(self.candidates)

    @property
    def rejected_count(self) -> int:
        return len(self.rejected_candidates)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "selected_goal": (
                self.selected_goal.to_dict()
                if self.selected_goal is not None
                else None
            ),
            "candidate_count": self.candidate_count,
            "rejected_count": self.rejected_count,
            "candidates": [candidate.to_dict() for candidate in self.candidates],
            "rejected_candidates": [
                candidate.to_dict()
                for candidate in self.rejected_candidates
            ],
            "policy": self.policy.to_dict(),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def select_coverage_goal(
    width: int,
    height: int,
    resolution_m: float,
    values: Sequence[int],
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[CoveragePolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
    frame_id: str = "map",
) -> CoverageResult:
    limits = policy or CoveragePolicy()
    pose = robot_pose or RobotPose2D(frame_id=frame_id)

    grid = GridSpec(
        width=int(width),
        height=int(height),
        resolution_m=float(resolution_m),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        origin_yaw=float(origin_yaw),
        frame_id=str(frame_id),
    )

    validate_grid_values(grid, values)

    candidates: list[CoverageCandidate] = []
    rejected: list[CoverageCandidate] = []

    for y in range(0, grid.height, limits.sample_step_cells):
        for x in range(0, grid.width, limits.sample_step_cells):
            index = grid_index(grid, x, y)
            value = int(values[index])

            if not is_free(value, limits.occupied_threshold):
                continue

            world_x, world_y = grid_to_world(grid, x, y)
            candidate = _make_candidate(
                grid=grid,
                x=x,
                y=y,
                world_x=world_x,
                world_y=world_y,
                robot_pose=pose,
                policy=limits,
                blacklist=blacklist,
            )

            if candidate.rejected:
                rejected.append(candidate)
            else:
                candidates.append(candidate)

    candidates.sort(key=lambda item: item.score, reverse=True)
    candidates = candidates[: limits.max_candidates]

    if not candidates:
        return CoverageResult(
            ok=False,
            candidates=tuple(candidates),
            rejected_candidates=tuple(rejected),
            policy=limits,
            message="No valid coverage goal candidate.",
        )

    selected = candidates[0]

    return CoverageResult(
        ok=True,
        selected_goal=selected.to_goal(),
        candidates=tuple(candidates),
        rejected_candidates=tuple(rejected),
        policy=limits,
        message="Coverage goal selected.",
    )


def _make_candidate(
    *,
    grid: GridSpec,
    x: int,
    y: int,
    world_x: float,
    world_y: float,
    robot_pose: RobotPose2D,
    policy: CoveragePolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> CoverageCandidate:
    distance_m = robot_pose.distance_to(world_x, world_y)
    rejected, reason = _rejection_reason(
        world_x,
        world_y,
        distance_m=distance_m,
        policy=policy,
        blacklist=blacklist,
    )
    score = distance_m if policy.prefer_farther else -distance_m

    return CoverageCandidate(
        x_cell=x,
        y_cell=y,
        index=grid_index(grid, x, y),
        world_x=world_x,
        world_y=world_y,
        distance_m=distance_m,
        score=score,
        rejected=rejected,
        rejection_reason=reason,
    )


def _rejection_reason(
    x: float,
    y: float,
    *,
    distance_m: float,
    policy: CoveragePolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> tuple[bool, str]:
    if distance_m < policy.min_goal_distance_m:
        return True, "goal_too_close"

    if distance_m > policy.max_goal_distance_m:
        return True, "goal_too_far"

    for entry in blacklist:
        if entry.contains(x, y):
            return True, f"blacklisted:{entry.reason}"

    return False, ""


def main() -> None:
    width = 20
    height = 16
    values = [-1] * (width * height)

    for y in range(2, 14):
        for x in range(2, 18):
            values[y * width + x] = 0

    result = select_coverage_goal(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=CoveragePolicy(min_goal_distance_m=0.0),
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "CoverageCandidate",
    "CoveragePolicy",
    "CoverageResult",
    "main",
    "select_coverage_goal",
]