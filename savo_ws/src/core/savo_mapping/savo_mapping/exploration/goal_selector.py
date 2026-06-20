#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Frontier goal selection helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from savo_mapping.exploration.frontier_detector import (
    FrontierCluster,
    FrontierDetectionResult,
)
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class RobotPose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    frame_id: str = "map"

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(float(x) - self.x, float(y) - self.y)

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class GoalBlacklistEntry:
    x: float
    y: float
    radius_m: float = 0.60
    reason: str = "failed_goal"

    def contains(self, x: float, y: float) -> bool:
        return math.hypot(float(x) - self.x, float(y) - self.y) <= self.radius_m

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "radius_m": self.radius_m,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class GoalSelectionPolicy:
    min_goal_distance_m: float = 0.30
    max_goal_distance_m: float = 12.0
    min_cluster_size_cells: int = 3
    distance_weight: float = 1.0
    size_weight: float = 0.15
    prefer_nearest: bool = True
    blacklist_radius_m: float = 0.60

    def __post_init__(self) -> None:
        if self.min_goal_distance_m < 0.0:
            raise ValueError("min_goal_distance_m must be non-negative.")

        if self.max_goal_distance_m <= 0.0:
            raise ValueError("max_goal_distance_m must be positive.")

        if self.min_goal_distance_m > self.max_goal_distance_m:
            raise ValueError("min_goal_distance_m cannot be greater than max_goal_distance_m.")

        if self.min_cluster_size_cells <= 0:
            raise ValueError("min_cluster_size_cells must be positive.")

        if self.blacklist_radius_m <= 0.0:
            raise ValueError("blacklist_radius_m must be positive.")

    def to_dict(self) -> dict:
        return {
            "min_goal_distance_m": self.min_goal_distance_m,
            "max_goal_distance_m": self.max_goal_distance_m,
            "min_cluster_size_cells": self.min_cluster_size_cells,
            "distance_weight": self.distance_weight,
            "size_weight": self.size_weight,
            "prefer_nearest": self.prefer_nearest,
            "blacklist_radius_m": self.blacklist_radius_m,
        }


@dataclass(frozen=True)
class GoalCandidate:
    cluster_id: int
    goal: ExplorationGoal
    distance_m: float
    cluster_size_cells: int
    raw_cluster_score: float
    selection_score: float
    rejected: bool = False
    rejection_reason: str = ""

    def to_dict(self) -> dict:
        return {
            "cluster_id": self.cluster_id,
            "goal": self.goal.to_dict(),
            "distance_m": self.distance_m,
            "cluster_size_cells": self.cluster_size_cells,
            "raw_cluster_score": self.raw_cluster_score,
            "selection_score": self.selection_score,
            "rejected": self.rejected,
            "rejection_reason": self.rejection_reason,
        }


@dataclass(frozen=True)
class GoalSelectionResult:
    ok: bool
    selected_goal: Optional[ExplorationGoal] = None
    selected_candidate: Optional[GoalCandidate] = None
    candidates: tuple[GoalCandidate, ...] = ()
    rejected_candidates: tuple[GoalCandidate, ...] = ()
    policy: GoalSelectionPolicy = field(default_factory=GoalSelectionPolicy)
    robot_pose: RobotPose2D = field(default_factory=RobotPose2D)
    message: str = "Goal selection not run."
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
            "selected_candidate": (
                self.selected_candidate.to_dict()
                if self.selected_candidate is not None
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
            "robot_pose": self.robot_pose.to_dict(),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def select_goal_from_detection(
    detection: FrontierDetectionResult,
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[GoalSelectionPolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> GoalSelectionResult:
    return select_goal_from_clusters(
        detection.clusters,
        robot_pose=robot_pose,
        policy=policy,
        blacklist=blacklist,
    )


def select_goal_from_clusters(
    clusters: Sequence[FrontierCluster],
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[GoalSelectionPolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> GoalSelectionResult:
    pose = robot_pose or RobotPose2D()
    limits = policy or GoalSelectionPolicy()

    candidates: list[GoalCandidate] = []
    rejected: list[GoalCandidate] = []

    for cluster in clusters:
        candidate = make_goal_candidate(
            cluster,
            robot_pose=pose,
            policy=limits,
            blacklist=blacklist,
        )

        if candidate.rejected:
            rejected.append(candidate)
        else:
            candidates.append(candidate)

    candidates.sort(key=lambda item: item.selection_score, reverse=True)

    if not candidates:
        return GoalSelectionResult(
            ok=False,
            candidates=tuple(candidates),
            rejected_candidates=tuple(rejected),
            policy=limits,
            robot_pose=pose,
            message="No valid frontier goal candidate.",
        )

    selected = candidates[0]

    return GoalSelectionResult(
        ok=True,
        selected_goal=selected.goal,
        selected_candidate=selected,
        candidates=tuple(candidates),
        rejected_candidates=tuple(rejected),
        policy=limits,
        robot_pose=pose,
        message="Frontier goal selected.",
    )


def make_goal_candidate(
    cluster: FrontierCluster,
    *,
    robot_pose: RobotPose2D,
    policy: GoalSelectionPolicy,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> GoalCandidate:
    distance_m = robot_pose.distance_to(cluster.centroid_x, cluster.centroid_y)
    rejected, reason = _rejection_reason(
        cluster,
        distance_m=distance_m,
        policy=policy,
        blacklist=blacklist,
    )

    score = _selection_score(
        cluster,
        distance_m=distance_m,
        policy=policy,
    )

    goal = cluster.to_goal(
        reason="frontier_selected" if not rejected else reason
    )

    return GoalCandidate(
        cluster_id=cluster.id,
        goal=goal,
        distance_m=distance_m,
        cluster_size_cells=cluster.size_cells,
        raw_cluster_score=cluster.score,
        selection_score=score,
        rejected=rejected,
        rejection_reason=reason,
    )


def make_blacklist_entry_from_goal(
    goal: ExplorationGoal,
    *,
    radius_m: float = 0.60,
    reason: str = "failed_goal",
) -> GoalBlacklistEntry:
    return GoalBlacklistEntry(
        x=goal.x,
        y=goal.y,
        radius_m=radius_m,
        reason=reason,
    )


def _selection_score(
    cluster: FrontierCluster,
    *,
    distance_m: float,
    policy: GoalSelectionPolicy,
) -> float:
    size_score = float(cluster.size_cells) * policy.size_weight
    distance_score = float(distance_m) * policy.distance_weight

    if policy.prefer_nearest:
        return size_score - distance_score

    return size_score + distance_score


def _rejection_reason(
    cluster: FrontierCluster,
    *,
    distance_m: float,
    policy: GoalSelectionPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> tuple[bool, str]:
    if cluster.size_cells < policy.min_cluster_size_cells:
        return True, "cluster_too_small"

    if distance_m < policy.min_goal_distance_m:
        return True, "goal_too_close"

    if distance_m > policy.max_goal_distance_m:
        return True, "goal_too_far"

    for entry in blacklist:
        if entry.contains(cluster.centroid_x, cluster.centroid_y):
            return True, f"blacklisted:{entry.reason}"

    return False, ""


def main() -> None:
    from savo_mapping.exploration.frontier_detector import detect_frontiers

    width = 8
    height = 6
    values = [-1] * (width * height)

    for y in range(1, 5):
        for x in range(1, 5):
            values[y * width + x] = 0

    detection = detect_frontiers(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        min_frontier_size_cells=2,
    )

    result = select_goal_from_detection(
        detection,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=GoalSelectionPolicy(min_cluster_size_cells=2),
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "GoalBlacklistEntry",
    "GoalCandidate",
    "GoalSelectionPolicy",
    "GoalSelectionResult",
    "RobotPose2D",
    "make_blacklist_entry_from_goal",
    "make_goal_candidate",
    "main",
    "select_goal_from_clusters",
    "select_goal_from_detection",
]