#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Area exploration helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

from savo_mapping.exploration.goal_selector import GoalBlacklistEntry, RobotPose2D
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class AreaRegion:
    key: str
    label: str
    center_x: float
    center_y: float
    yaw: float = 0.0
    frame_id: str = "map"
    area_type: str = "general"
    priority: int = 0
    visited: bool = False
    confirmed: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Area key cannot be empty.")

        if not self.label.strip():
            raise ValueError("Area label cannot be empty.")

    def distance_to(self, pose: RobotPose2D) -> float:
        return math.hypot(self.center_x - pose.x, self.center_y - pose.y)

    def to_goal(self) -> ExplorationGoal:
        return ExplorationGoal(
            x=self.center_x,
            y=self.center_y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            score=float(self.priority),
            reason=f"area_selected:{self.key}",
        )

    def with_visited(self, visited: bool = True) -> "AreaRegion":
        return AreaRegion(
            key=self.key,
            label=self.label,
            center_x=self.center_x,
            center_y=self.center_y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            area_type=self.area_type,
            priority=self.priority,
            visited=visited,
            confirmed=self.confirmed,
            metadata=dict(self.metadata),
        )

    def with_confirmed(self, confirmed: bool = True) -> "AreaRegion":
        return AreaRegion(
            key=self.key,
            label=self.label,
            center_x=self.center_x,
            center_y=self.center_y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            area_type=self.area_type,
            priority=self.priority,
            visited=self.visited,
            confirmed=confirmed,
            metadata=dict(self.metadata),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "area_type": self.area_type,
            "priority": self.priority,
            "visited": self.visited,
            "confirmed": self.confirmed,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class AreaPolicy:
    min_goal_distance_m: float = 0.25
    max_goal_distance_m: float = 20.0
    skip_visited: bool = True
    require_confirmed: bool = False
    prefer_nearest: bool = True
    priority_weight: float = 10.0
    distance_weight: float = 1.0

    def __post_init__(self) -> None:
        if self.min_goal_distance_m < 0.0:
            raise ValueError("min_goal_distance_m must be non-negative.")

        if self.max_goal_distance_m <= 0.0:
            raise ValueError("max_goal_distance_m must be positive.")

        if self.min_goal_distance_m > self.max_goal_distance_m:
            raise ValueError(
                "min_goal_distance_m cannot be greater than max_goal_distance_m."
            )

        if self.priority_weight < 0.0:
            raise ValueError("priority_weight must be non-negative.")

        if self.distance_weight < 0.0:
            raise ValueError("distance_weight must be non-negative.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_goal_distance_m": self.min_goal_distance_m,
            "max_goal_distance_m": self.max_goal_distance_m,
            "skip_visited": self.skip_visited,
            "require_confirmed": self.require_confirmed,
            "prefer_nearest": self.prefer_nearest,
            "priority_weight": self.priority_weight,
            "distance_weight": self.distance_weight,
        }


@dataclass(frozen=True)
class AreaCandidate:
    area: AreaRegion
    distance_m: float
    score: float
    rejected: bool = False
    rejection_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "area": self.area.to_dict(),
            "distance_m": self.distance_m,
            "score": self.score,
            "rejected": self.rejected,
            "rejection_reason": self.rejection_reason,
        }


@dataclass(frozen=True)
class AreaResult:
    ok: bool
    selected_goal: Optional[ExplorationGoal] = None
    selected_area: Optional[AreaRegion] = None
    candidates: tuple[AreaCandidate, ...] = ()
    rejected_candidates: tuple[AreaCandidate, ...] = ()
    policy: AreaPolicy = field(default_factory=AreaPolicy)
    message: str = "Area exploration not run."
    timestamp_s: float = field(default_factory=time.time)

    @property
    def candidate_count(self) -> int:
        return len(self.candidates)

    @property
    def rejected_count(self) -> int:
        return len(self.rejected_candidates)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "selected_goal": (
                self.selected_goal.to_dict()
                if self.selected_goal is not None
                else None
            ),
            "selected_area": (
                self.selected_area.to_dict()
                if self.selected_area is not None
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


def select_area_goal(
    areas: Sequence[AreaRegion],
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[AreaPolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> AreaResult:
    pose = robot_pose or RobotPose2D()
    limits = policy or AreaPolicy()

    candidates: list[AreaCandidate] = []
    rejected: list[AreaCandidate] = []

    for area in areas:
        candidate = _make_candidate(
            area,
            robot_pose=pose,
            policy=limits,
            blacklist=blacklist,
        )

        if candidate.rejected:
            rejected.append(candidate)
        else:
            candidates.append(candidate)

    candidates.sort(key=lambda item: item.score, reverse=True)

    if not candidates:
        return AreaResult(
            ok=False,
            candidates=tuple(candidates),
            rejected_candidates=tuple(rejected),
            policy=limits,
            message="No valid area goal candidate.",
        )

    selected = candidates[0]

    return AreaResult(
        ok=True,
        selected_goal=selected.area.to_goal(),
        selected_area=selected.area,
        candidates=tuple(candidates),
        rejected_candidates=tuple(rejected),
        policy=limits,
        message="Area goal selected.",
    )


def mark_area_visited(
    areas: Sequence[AreaRegion],
    selected: AreaRegion,
) -> tuple[AreaRegion, ...]:
    updated: list[AreaRegion] = []

    for area in areas:
        if area.key == selected.key:
            updated.append(area.with_visited(True))
        else:
            updated.append(area)

    return tuple(updated)


def area_from_dict(data: dict[str, Any]) -> AreaRegion:
    return AreaRegion(
        key=str(data["key"]),
        label=str(data.get("label", data["key"])),
        center_x=float(data["center_x"]),
        center_y=float(data["center_y"]),
        yaw=float(data.get("yaw", 0.0)),
        frame_id=str(data.get("frame_id", "map")),
        area_type=str(data.get("area_type", "general")),
        priority=int(data.get("priority", 0)),
        visited=bool(data.get("visited", False)),
        confirmed=bool(data.get("confirmed", False)),
        metadata=dict(data.get("metadata", {})),
    )


def areas_from_dicts(items: Sequence[dict[str, Any]]) -> tuple[AreaRegion, ...]:
    return tuple(area_from_dict(item) for item in items)


def _make_candidate(
    area: AreaRegion,
    *,
    robot_pose: RobotPose2D,
    policy: AreaPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> AreaCandidate:
    distance_m = area.distance_to(robot_pose)
    rejected, reason = _rejection_reason(
        area,
        distance_m=distance_m,
        policy=policy,
        blacklist=blacklist,
    )

    priority_score = float(area.priority) * policy.priority_weight
    distance_score = float(distance_m) * policy.distance_weight

    if policy.prefer_nearest:
        score = priority_score - distance_score
    else:
        score = priority_score + distance_score

    return AreaCandidate(
        area=area,
        distance_m=distance_m,
        score=score,
        rejected=rejected,
        rejection_reason=reason,
    )


def _rejection_reason(
    area: AreaRegion,
    *,
    distance_m: float,
    policy: AreaPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> tuple[bool, str]:
    if policy.skip_visited and area.visited:
        return True, "already_visited"

    if policy.require_confirmed and not area.confirmed:
        return True, "not_confirmed"

    if distance_m < policy.min_goal_distance_m:
        return True, "goal_too_close"

    if distance_m > policy.max_goal_distance_m:
        return True, "goal_too_far"

    for entry in blacklist:
        if entry.contains(area.center_x, area.center_y):
            return True, f"blacklisted:{entry.reason}"

    return False, ""


def main() -> None:
    areas = (
        AreaRegion(
            key="info_desk",
            label="Info Desk",
            center_x=2.0,
            center_y=1.5,
            area_type="service",
            priority=3,
            confirmed=True,
        ),
        AreaRegion(
            key="main_corridor",
            label="Main Corridor",
            center_x=5.0,
            center_y=2.0,
            area_type="corridor",
            priority=2,
            confirmed=True,
        ),
        AreaRegion(
            key="elevator_area",
            label="Elevator Area",
            center_x=7.0,
            center_y=3.0,
            area_type="access",
            priority=4,
            confirmed=False,
        ),
    )

    result = select_area_goal(
        areas,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=AreaPolicy(
            min_goal_distance_m=0.0,
            require_confirmed=False,
        ),
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "AreaCandidate",
    "AreaPolicy",
    "AreaRegion",
    "AreaResult",
    "area_from_dict",
    "areas_from_dicts",
    "main",
    "mark_area_visited",
    "select_area_goal",
]