#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Waypoint exploration helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from savo_mapping.exploration.goal_selector import GoalBlacklistEntry, RobotPose2D
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    name: str = ""
    priority: int = 0
    visited: bool = False

    def distance_to(self, pose: RobotPose2D) -> float:
        return math.hypot(self.x - pose.x, self.y - pose.y)

    def to_goal(self) -> ExplorationGoal:
        return ExplorationGoal(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            score=float(self.priority),
            reason=self.name or "waypoint_selected",
        )

    def with_visited(self, visited: bool = True) -> "Waypoint":
        return Waypoint(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            name=self.name,
            priority=self.priority,
            visited=visited,
        )

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "name": self.name,
            "priority": self.priority,
            "visited": self.visited,
        }


@dataclass(frozen=True)
class WaypointPolicy:
    min_goal_distance_m: float = 0.20
    max_goal_distance_m: float = 20.0
    prefer_nearest: bool = True
    skip_visited: bool = True

    def __post_init__(self) -> None:
        if self.min_goal_distance_m < 0.0:
            raise ValueError("min_goal_distance_m must be non-negative.")

        if self.max_goal_distance_m <= 0.0:
            raise ValueError("max_goal_distance_m must be positive.")

        if self.min_goal_distance_m > self.max_goal_distance_m:
            raise ValueError("min_goal_distance_m cannot be greater than max_goal_distance_m.")

    def to_dict(self) -> dict:
        return {
            "min_goal_distance_m": self.min_goal_distance_m,
            "max_goal_distance_m": self.max_goal_distance_m,
            "prefer_nearest": self.prefer_nearest,
            "skip_visited": self.skip_visited,
        }


@dataclass(frozen=True)
class WaypointCandidate:
    waypoint: Waypoint
    distance_m: float
    score: float
    rejected: bool = False
    rejection_reason: str = ""

    def to_dict(self) -> dict:
        return {
            "waypoint": self.waypoint.to_dict(),
            "distance_m": self.distance_m,
            "score": self.score,
            "rejected": self.rejected,
            "rejection_reason": self.rejection_reason,
        }


@dataclass(frozen=True)
class WaypointResult:
    ok: bool
    selected_goal: Optional[ExplorationGoal] = None
    selected_waypoint: Optional[Waypoint] = None
    candidates: tuple[WaypointCandidate, ...] = ()
    rejected_candidates: tuple[WaypointCandidate, ...] = ()
    policy: WaypointPolicy = field(default_factory=WaypointPolicy)
    message: str = "Waypoint exploration not run."
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
            "selected_waypoint": (
                self.selected_waypoint.to_dict()
                if self.selected_waypoint is not None
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


def select_waypoint_goal(
    waypoints: Sequence[Waypoint],
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[WaypointPolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> WaypointResult:
    pose = robot_pose or RobotPose2D()
    limits = policy or WaypointPolicy()

    candidates: list[WaypointCandidate] = []
    rejected: list[WaypointCandidate] = []

    for waypoint in waypoints:
        candidate = _make_candidate(
            waypoint,
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
        return WaypointResult(
            ok=False,
            candidates=tuple(candidates),
            rejected_candidates=tuple(rejected),
            policy=limits,
            message="No valid waypoint goal candidate.",
        )

    selected = candidates[0]
    goal = selected.waypoint.to_goal()

    return WaypointResult(
        ok=True,
        selected_goal=goal,
        selected_waypoint=selected.waypoint,
        candidates=tuple(candidates),
        rejected_candidates=tuple(rejected),
        policy=limits,
        message="Waypoint goal selected.",
    )


def mark_waypoint_visited(
    waypoints: Sequence[Waypoint],
    selected: Waypoint,
) -> tuple[Waypoint, ...]:
    updated: list[Waypoint] = []

    for waypoint in waypoints:
        if _same_waypoint(waypoint, selected):
            updated.append(waypoint.with_visited(True))
        else:
            updated.append(waypoint)

    return tuple(updated)


def waypoints_from_dicts(items: Sequence[dict]) -> tuple[Waypoint, ...]:
    return tuple(waypoint_from_dict(item) for item in items)


def waypoint_from_dict(data: dict) -> Waypoint:
    return Waypoint(
        x=float(data["x"]),
        y=float(data["y"]),
        yaw=float(data.get("yaw", 0.0)),
        frame_id=str(data.get("frame_id", "map")),
        name=str(data.get("name", "")),
        priority=int(data.get("priority", 0)),
        visited=bool(data.get("visited", False)),
    )


def _make_candidate(
    waypoint: Waypoint,
    *,
    robot_pose: RobotPose2D,
    policy: WaypointPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> WaypointCandidate:
    distance_m = waypoint.distance_to(robot_pose)
    rejected, reason = _rejection_reason(
        waypoint,
        distance_m=distance_m,
        policy=policy,
        blacklist=blacklist,
    )

    distance_score = -distance_m if policy.prefer_nearest else distance_m
    priority_score = float(waypoint.priority) * 10.0
    score = priority_score + distance_score

    return WaypointCandidate(
        waypoint=waypoint,
        distance_m=distance_m,
        score=score,
        rejected=rejected,
        rejection_reason=reason,
    )


def _rejection_reason(
    waypoint: Waypoint,
    *,
    distance_m: float,
    policy: WaypointPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> tuple[bool, str]:
    if policy.skip_visited and waypoint.visited:
        return True, "already_visited"

    if distance_m < policy.min_goal_distance_m:
        return True, "goal_too_close"

    if distance_m > policy.max_goal_distance_m:
        return True, "goal_too_far"

    for entry in blacklist:
        if entry.contains(waypoint.x, waypoint.y):
            return True, f"blacklisted:{entry.reason}"

    return False, ""


def _same_waypoint(first: Waypoint, second: Waypoint) -> bool:
    return (
        first.name == second.name
        and math.isclose(first.x, second.x)
        and math.isclose(first.y, second.y)
        and first.frame_id == second.frame_id
    )


def main() -> None:
    waypoints = (
        Waypoint(x=1.0, y=1.0, name="corridor_start", priority=1),
        Waypoint(x=4.0, y=2.0, name="info_desk_area", priority=2),
        Waypoint(x=7.0, y=3.0, name="a201_area", priority=3),
    )

    result = select_waypoint_goal(
        waypoints,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=WaypointPolicy(min_goal_distance_m=0.0),
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "Waypoint",
    "WaypointCandidate",
    "WaypointPolicy",
    "WaypointResult",
    "main",
    "mark_waypoint_visited",
    "select_waypoint_goal",
    "waypoint_from_dict",
    "waypoints_from_dicts",
]