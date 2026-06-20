#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Semantic exploration helpers for Robot Savo mapping."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

from savo_mapping.exploration.goal_selector import GoalBlacklistEntry, RobotPose2D
from savo_mapping.models.exploration_status import ExplorationGoal


@dataclass(frozen=True)
class SemanticTarget:
    key: str
    label: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    target_type: str = "place"
    source: str = "unknown"
    tag_id: Optional[int] = None
    confidence: float = 1.0
    priority: int = 0
    confirmed: bool = False
    visited: bool = False
    needs_review: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.key.strip():
            raise ValueError("Semantic target key cannot be empty.")

        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Semantic target confidence must be between 0 and 1.")

    def distance_to(self, pose: RobotPose2D) -> float:
        return math.hypot(self.x - pose.x, self.y - pose.y)

    def display_label(self) -> str:
        return self.label.strip() or self.key

    def to_goal(self) -> ExplorationGoal:
        return ExplorationGoal(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            score=float(self.priority) + self.confidence,
            reason=f"semantic_selected:{self.key}",
        )

    def with_visited(self, visited: bool = True) -> "SemanticTarget":
        return SemanticTarget(
            key=self.key,
            label=self.label,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            target_type=self.target_type,
            source=self.source,
            tag_id=self.tag_id,
            confidence=self.confidence,
            priority=self.priority,
            confirmed=self.confirmed,
            visited=visited,
            needs_review=self.needs_review,
            metadata=dict(self.metadata),
        )

    def with_confirmed(self, confirmed: bool = True) -> "SemanticTarget":
        return SemanticTarget(
            key=self.key,
            label=self.label,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            frame_id=self.frame_id,
            target_type=self.target_type,
            source=self.source,
            tag_id=self.tag_id,
            confidence=self.confidence,
            priority=self.priority,
            confirmed=confirmed,
            visited=self.visited,
            needs_review=not confirmed,
            metadata=dict(self.metadata),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "label": self.label,
            "display_label": self.display_label(),
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "target_type": self.target_type,
            "source": self.source,
            "tag_id": self.tag_id,
            "confidence": self.confidence,
            "priority": self.priority,
            "confirmed": self.confirmed,
            "visited": self.visited,
            "needs_review": self.needs_review,
            "metadata": dict(self.metadata),
        }


@dataclass(frozen=True)
class SemanticPolicy:
    min_goal_distance_m: float = 0.25
    max_goal_distance_m: float = 20.0
    min_confidence: float = 0.0
    skip_visited: bool = True
    prefer_unconfirmed: bool = True
    require_label: bool = False
    prefer_nearest: bool = True
    priority_weight: float = 10.0
    confidence_weight: float = 5.0
    unconfirmed_bonus: float = 20.0
    review_bonus: float = 10.0
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

        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be between 0 and 1.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_goal_distance_m": self.min_goal_distance_m,
            "max_goal_distance_m": self.max_goal_distance_m,
            "min_confidence": self.min_confidence,
            "skip_visited": self.skip_visited,
            "prefer_unconfirmed": self.prefer_unconfirmed,
            "require_label": self.require_label,
            "prefer_nearest": self.prefer_nearest,
            "priority_weight": self.priority_weight,
            "confidence_weight": self.confidence_weight,
            "unconfirmed_bonus": self.unconfirmed_bonus,
            "review_bonus": self.review_bonus,
            "distance_weight": self.distance_weight,
        }


@dataclass(frozen=True)
class SemanticCandidate:
    target: SemanticTarget
    distance_m: float
    score: float
    rejected: bool = False
    rejection_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "target": self.target.to_dict(),
            "distance_m": self.distance_m,
            "score": self.score,
            "rejected": self.rejected,
            "rejection_reason": self.rejection_reason,
        }


@dataclass(frozen=True)
class SemanticResult:
    ok: bool
    selected_goal: Optional[ExplorationGoal] = None
    selected_target: Optional[SemanticTarget] = None
    candidates: tuple[SemanticCandidate, ...] = ()
    rejected_candidates: tuple[SemanticCandidate, ...] = ()
    policy: SemanticPolicy = field(default_factory=SemanticPolicy)
    message: str = "Semantic exploration not run."
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
            "selected_target": (
                self.selected_target.to_dict()
                if self.selected_target is not None
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


def select_semantic_goal(
    targets: Sequence[SemanticTarget],
    *,
    robot_pose: Optional[RobotPose2D] = None,
    policy: Optional[SemanticPolicy] = None,
    blacklist: Sequence[GoalBlacklistEntry] = (),
) -> SemanticResult:
    pose = robot_pose or RobotPose2D()
    limits = policy or SemanticPolicy()

    candidates: list[SemanticCandidate] = []
    rejected: list[SemanticCandidate] = []

    for target in targets:
        candidate = _make_candidate(
            target,
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
        return SemanticResult(
            ok=False,
            candidates=tuple(candidates),
            rejected_candidates=tuple(rejected),
            policy=limits,
            message="No valid semantic goal candidate.",
        )

    selected = candidates[0]

    return SemanticResult(
        ok=True,
        selected_goal=selected.target.to_goal(),
        selected_target=selected.target,
        candidates=tuple(candidates),
        rejected_candidates=tuple(rejected),
        policy=limits,
        message="Semantic goal selected.",
    )


def make_target_from_apriltag(
    *,
    tag_id: int,
    key: str,
    label: str,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = "map",
    confidence: float = 1.0,
    confirmed: bool = False,
    priority: int = 0,
) -> SemanticTarget:
    return SemanticTarget(
        key=key,
        label=label,
        x=x,
        y=y,
        yaw=yaw,
        frame_id=frame_id,
        target_type="apriltag_location",
        source="apriltag",
        tag_id=int(tag_id),
        confidence=float(confidence),
        priority=int(priority),
        confirmed=bool(confirmed),
        visited=False,
        needs_review=not bool(confirmed),
        metadata={"source": "apriltag", "tag_id": int(tag_id)},
    )


def mark_semantic_target_visited(
    targets: Sequence[SemanticTarget],
    selected: SemanticTarget,
) -> tuple[SemanticTarget, ...]:
    return tuple(
        target.with_visited(True) if target.key == selected.key else target
        for target in targets
    )


def mark_semantic_target_confirmed(
    targets: Sequence[SemanticTarget],
    selected: SemanticTarget,
) -> tuple[SemanticTarget, ...]:
    return tuple(
        target.with_confirmed(True) if target.key == selected.key else target
        for target in targets
    )


def semantic_target_from_dict(data: dict[str, Any]) -> SemanticTarget:
    return SemanticTarget(
        key=str(data["key"]),
        label=str(data.get("label", data["key"])),
        x=float(data["x"]),
        y=float(data["y"]),
        yaw=float(data.get("yaw", 0.0)),
        frame_id=str(data.get("frame_id", "map")),
        target_type=str(data.get("target_type", "place")),
        source=str(data.get("source", "unknown")),
        tag_id=(
            None
            if data.get("tag_id") is None
            else int(data["tag_id"])
        ),
        confidence=float(data.get("confidence", 1.0)),
        priority=int(data.get("priority", 0)),
        confirmed=bool(data.get("confirmed", False)),
        visited=bool(data.get("visited", False)),
        needs_review=bool(data.get("needs_review", True)),
        metadata=dict(data.get("metadata", {})),
    )


def semantic_targets_from_dicts(
    items: Sequence[dict[str, Any]],
) -> tuple[SemanticTarget, ...]:
    return tuple(semantic_target_from_dict(item) for item in items)


def _make_candidate(
    target: SemanticTarget,
    *,
    robot_pose: RobotPose2D,
    policy: SemanticPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> SemanticCandidate:
    distance_m = target.distance_to(robot_pose)
    rejected, reason = _rejection_reason(
        target,
        distance_m=distance_m,
        policy=policy,
        blacklist=blacklist,
    )

    score = _score_target(
        target,
        distance_m=distance_m,
        policy=policy,
    )

    return SemanticCandidate(
        target=target,
        distance_m=distance_m,
        score=score,
        rejected=rejected,
        rejection_reason=reason,
    )


def _score_target(
    target: SemanticTarget,
    *,
    distance_m: float,
    policy: SemanticPolicy,
) -> float:
    score = float(target.priority) * policy.priority_weight
    score += float(target.confidence) * policy.confidence_weight

    if policy.prefer_unconfirmed and not target.confirmed:
        score += policy.unconfirmed_bonus

    if target.needs_review:
        score += policy.review_bonus

    if policy.prefer_nearest:
        score -= distance_m * policy.distance_weight
    else:
        score += distance_m * policy.distance_weight

    return score


def _rejection_reason(
    target: SemanticTarget,
    *,
    distance_m: float,
    policy: SemanticPolicy,
    blacklist: Sequence[GoalBlacklistEntry],
) -> tuple[bool, str]:
    if policy.skip_visited and target.visited:
        return True, "already_visited"

    if policy.require_label and not target.label.strip():
        return True, "missing_label"

    if target.confidence < policy.min_confidence:
        return True, "confidence_too_low"

    if distance_m < policy.min_goal_distance_m:
        return True, "goal_too_close"

    if distance_m > policy.max_goal_distance_m:
        return True, "goal_too_far"

    for entry in blacklist:
        if entry.contains(target.x, target.y):
            return True, f"blacklisted:{entry.reason}"

    return False, ""


def main() -> None:
    targets = (
        make_target_from_apriltag(
            tag_id=21,
            key="a201",
            label="A201",
            x=4.2,
            y=8.7,
            confidence=0.95,
            confirmed=True,
            priority=3,
        ),
        make_target_from_apriltag(
            tag_id=7,
            key="info_desk",
            label="Info Desk",
            x=2.0,
            y=1.5,
            confidence=0.85,
            confirmed=False,
            priority=5,
        ),
    )

    result = select_semantic_goal(
        targets,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=SemanticPolicy(min_goal_distance_m=0.0),
    )

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "SemanticCandidate",
    "SemanticPolicy",
    "SemanticResult",
    "SemanticTarget",
    "main",
    "make_target_from_apriltag",
    "mark_semantic_target_confirmed",
    "mark_semantic_target_visited",
    "select_semantic_goal",
    "semantic_target_from_dict",
    "semantic_targets_from_dicts",
]