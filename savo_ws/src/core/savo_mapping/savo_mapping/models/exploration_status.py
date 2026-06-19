#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Autonomous exploration status model. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional


# =============================================================================
# Exploration states
# =============================================================================
class ExplorationState(str, Enum):
    IDLE = "idle"
    WAITING_FOR_MAP = "waiting_for_map"
    SEARCHING_FRONTIER = "searching_frontier"
    GOAL_SELECTED = "goal_selected"
    NAVIGATING = "navigating"
    GOAL_REACHED = "goal_reached"
    GOAL_FAILED = "goal_failed"
    COMPLETE = "complete"
    PAUSED = "paused"
    ERROR = "error"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(state.value for state in cls)

    @classmethod
    def from_string(cls, value: str) -> "ExplorationState":
        normalized = normalize_exploration_state(value)

        for state in cls:
            if state.value == normalized:
                return state

        raise ValueError(f"Unsupported exploration state: {value!r}")


# =============================================================================
# Helpers
# =============================================================================
def normalize_exploration_state(value: str) -> str:
    state = str(value).strip().lower().replace("-", "_").replace(" ", "_")

    if not state:
        raise ValueError("Exploration state cannot be empty.")

    return state


def is_valid_exploration_state(value: str) -> bool:
    try:
        ExplorationState.from_string(value)
        return True
    except ValueError:
        return False


def require_valid_exploration_state(value: str) -> str:
    return ExplorationState.from_string(value).value


# =============================================================================
# Exploration goal
# =============================================================================
@dataclass(frozen=True)
class ExplorationGoal:
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"
    score: float = 0.0
    reason: str = ""

    def to_dict(self) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "frame_id": self.frame_id,
            "score": self.score,
            "reason": self.reason,
        }


# =============================================================================
# Exploration status
# =============================================================================
@dataclass(frozen=True)
class ExplorationStatus:
    state: str = ExplorationState.IDLE.value
    active: bool = False
    paused: bool = False

    frontier_count: int = 0
    visited_goal_count: int = 0
    failed_goal_count: int = 0
    blacklist_count: int = 0

    current_goal: Optional[ExplorationGoal] = None

    message: str = "Exploration idle."
    timestamp_s: float = field(default_factory=time.time)
    extra: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        require_valid_exploration_state(self.state)

    @property
    def complete(self) -> bool:
        return self.state == ExplorationState.COMPLETE.value

    @property
    def error(self) -> bool:
        return self.state == ExplorationState.ERROR.value

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "active": self.active,
            "paused": self.paused,
            "complete": self.complete,
            "error": self.error,
            "frontier_count": self.frontier_count,
            "visited_goal_count": self.visited_goal_count,
            "failed_goal_count": self.failed_goal_count,
            "blacklist_count": self.blacklist_count,
            "current_goal": (
                self.current_goal.to_dict()
                if self.current_goal is not None
                else None
            ),
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_exploration_status(
    state: str = ExplorationState.IDLE.value,
    active: bool = False,
    paused: bool = False,
    frontier_count: int = 0,
    visited_goal_count: int = 0,
    failed_goal_count: int = 0,
    blacklist_count: int = 0,
    current_goal: Optional[ExplorationGoal] = None,
    message: str = "",
    extra: Optional[Dict[str, Any]] = None,
) -> ExplorationStatus:
    valid_state = require_valid_exploration_state(state)

    if not message:
        message = f"Exploration state: {valid_state}."

    return ExplorationStatus(
        state=valid_state,
        active=active,
        paused=paused,
        frontier_count=max(0, int(frontier_count)),
        visited_goal_count=max(0, int(visited_goal_count)),
        failed_goal_count=max(0, int(failed_goal_count)),
        blacklist_count=max(0, int(blacklist_count)),
        current_goal=current_goal,
        message=message,
        extra=dict(extra or {}),
    )


def make_idle_exploration_status() -> ExplorationStatus:
    return make_exploration_status(
        state=ExplorationState.IDLE.value,
        active=False,
        paused=False,
        message="Exploration idle.",
    )


def make_goal_selected_status(
    goal: ExplorationGoal,
    frontier_count: int,
) -> ExplorationStatus:
    return make_exploration_status(
        state=ExplorationState.GOAL_SELECTED.value,
        active=True,
        frontier_count=frontier_count,
        current_goal=goal,
        message="Exploration goal selected.",
    )


def make_navigating_status(
    goal: ExplorationGoal,
    frontier_count: int,
    visited_goal_count: int = 0,
    failed_goal_count: int = 0,
    blacklist_count: int = 0,
) -> ExplorationStatus:
    return make_exploration_status(
        state=ExplorationState.NAVIGATING.value,
        active=True,
        frontier_count=frontier_count,
        visited_goal_count=visited_goal_count,
        failed_goal_count=failed_goal_count,
        blacklist_count=blacklist_count,
        current_goal=goal,
        message="Navigating to exploration goal.",
    )


def make_complete_exploration_status(
    visited_goal_count: int,
    failed_goal_count: int = 0,
) -> ExplorationStatus:
    return make_exploration_status(
        state=ExplorationState.COMPLETE.value,
        active=False,
        visited_goal_count=visited_goal_count,
        failed_goal_count=failed_goal_count,
        message="Autonomous exploration complete.",
    )


def exploration_status_from_dict(data: Dict[str, Any]) -> ExplorationStatus:
    goal_data = data.get("current_goal")
    goal = None

    if isinstance(goal_data, dict):
        goal = ExplorationGoal(
            x=float(goal_data.get("x", 0.0)),
            y=float(goal_data.get("y", 0.0)),
            yaw=float(goal_data.get("yaw", 0.0)),
            frame_id=str(goal_data.get("frame_id", "map")),
            score=float(goal_data.get("score", 0.0)),
            reason=str(goal_data.get("reason", "")),
        )

    return ExplorationStatus(
        state=require_valid_exploration_state(
            str(data.get("state", ExplorationState.IDLE.value))
        ),
        active=bool(data.get("active", False)),
        paused=bool(data.get("paused", False)),
        frontier_count=max(0, int(data.get("frontier_count", 0))),
        visited_goal_count=max(0, int(data.get("visited_goal_count", 0))),
        failed_goal_count=max(0, int(data.get("failed_goal_count", 0))),
        blacklist_count=max(0, int(data.get("blacklist_count", 0))),
        current_goal=goal,
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    goal = ExplorationGoal(
        x=2.5,
        y=1.2,
        yaw=0.0,
        score=0.82,
        reason="closest_frontier",
    )

    print(make_idle_exploration_status().to_json(indent=2))
    print(make_goal_selected_status(goal, frontier_count=8).to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "ExplorationState",
    "ExplorationGoal",
    "ExplorationStatus",
    "normalize_exploration_state",
    "is_valid_exploration_state",
    "require_valid_exploration_state",
    "make_exploration_status",
    "make_idle_exploration_status",
    "make_goal_selected_status",
    "make_navigating_status",
    "make_complete_exploration_status",
    "exploration_status_from_dict",
    "main",
]