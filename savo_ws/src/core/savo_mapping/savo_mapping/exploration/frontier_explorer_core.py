#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Core frontier exploration cycle for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence

from savo_mapping.exploration.goal_blacklist import GoalBlacklist
from savo_mapping.exploration.exploration_policy import (
    ExplorationPolicy,
    make_autonomous_exploration_policy,
)
from savo_mapping.exploration.frontier_detector import (
    FrontierDetectionResult,
    detect_frontiers,
)
from savo_mapping.exploration.goal_selector import (
    GoalSelectionResult,
    RobotPose2D,
    select_goal_from_detection,
)
from savo_mapping.models.exploration_status import ExplorationGoal


DECISION_DISABLED = "disabled"
DECISION_WAITING = "waiting"
DECISION_PAUSED = "paused"
DECISION_COMPLETE = "complete"
DECISION_GOAL_SELECTED = "goal_selected"
DECISION_KEEP_GOAL = "keep_goal"
DECISION_STOPPED = "stopped"


@dataclass(frozen=True)
class ExplorationMapInput:
    width: int
    height: int
    resolution_m: float
    values: Sequence[int]
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0
    frame_id: str = "map"

    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "resolution_m": self.resolution_m,
            "cell_count": len(self.values),
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "origin_yaw": self.origin_yaw,
            "frame_id": self.frame_id,
        }


@dataclass(frozen=True)
class ExplorationSafetyState:
    safety_stop: bool = False
    scan_stale: bool = False
    odom_stale: bool = False
    tf_ok: bool = True
    nav2_ready: bool = True
    battery_low: bool = False

    @property
    def clear(self) -> bool:
        return not (
            self.safety_stop
            or self.scan_stale
            or self.odom_stale
            or not self.tf_ok
            or not self.nav2_ready
            or self.battery_low
        )

    def blocking_reasons(self) -> tuple[str, ...]:
        reasons: list[str] = []

        if self.safety_stop:
            reasons.append("safety_stop")
        if self.scan_stale:
            reasons.append("scan_stale")
        if self.odom_stale:
            reasons.append("odom_stale")
        if not self.tf_ok:
            reasons.append("tf_failure")
        if not self.nav2_ready:
            reasons.append("nav2_not_ready")
        if self.battery_low:
            reasons.append("battery_low")

        return tuple(reasons)

    def to_dict(self) -> dict:
        return {
            "safety_stop": self.safety_stop,
            "scan_stale": self.scan_stale,
            "odom_stale": self.odom_stale,
            "tf_ok": self.tf_ok,
            "nav2_ready": self.nav2_ready,
            "battery_low": self.battery_low,
            "clear": self.clear,
            "blocking_reasons": self.blocking_reasons(),
        }


@dataclass(frozen=True)
class ExplorationCycleInput:
    map_input: ExplorationMapInput
    robot_pose: RobotPose2D = field(default_factory=RobotPose2D)
    safety: ExplorationSafetyState = field(default_factory=ExplorationSafetyState)

    def to_dict(self) -> dict:
        return {
            "map_input": self.map_input.to_dict(),
            "robot_pose": self.robot_pose.to_dict(),
            "safety": self.safety.to_dict(),
        }


@dataclass(frozen=True)
class ExplorationRuntimeState:
    active_goal: Optional[ExplorationGoal] = None
    previous_goal: Optional[ExplorationGoal] = None
    last_decision: str = DECISION_WAITING
    paused: bool = False
    pause_reason: str = ""
    no_candidate_cycles: int = 0
    failed_goal_count: int = 0
    reached_goal_count: int = 0
    last_update_s: float = field(default_factory=time.time)

    def with_decision(
        self,
        decision: str,
        *,
        active_goal: Optional[ExplorationGoal] = None,
        previous_goal: Optional[ExplorationGoal] = None,
        paused: Optional[bool] = None,
        pause_reason: Optional[str] = None,
        no_candidate_cycles: Optional[int] = None,
        failed_goal_count: Optional[int] = None,
        reached_goal_count: Optional[int] = None,
    ) -> "ExplorationRuntimeState":
        return ExplorationRuntimeState(
            active_goal=active_goal,
            previous_goal=(
                self.previous_goal
                if previous_goal is None
                else previous_goal
            ),
            last_decision=decision,
            paused=self.paused if paused is None else bool(paused),
            pause_reason=(
                self.pause_reason
                if pause_reason is None
                else str(pause_reason)
            ),
            no_candidate_cycles=(
                self.no_candidate_cycles
                if no_candidate_cycles is None
                else int(no_candidate_cycles)
            ),
            failed_goal_count=(
                self.failed_goal_count
                if failed_goal_count is None
                else int(failed_goal_count)
            ),
            reached_goal_count=(
                self.reached_goal_count
                if reached_goal_count is None
                else int(reached_goal_count)
            ),
        )

    def to_dict(self) -> dict:
        return {
            "active_goal": (
                self.active_goal.to_dict()
                if self.active_goal is not None
                else None
            ),
            "previous_goal": (
                self.previous_goal.to_dict()
                if self.previous_goal is not None
                else None
            ),
            "last_decision": self.last_decision,
            "paused": self.paused,
            "pause_reason": self.pause_reason,
            "no_candidate_cycles": self.no_candidate_cycles,
            "failed_goal_count": self.failed_goal_count,
            "reached_goal_count": self.reached_goal_count,
            "last_update_s": self.last_update_s,
        }


@dataclass(frozen=True)
class ExplorationCycleResult:
    ok: bool
    decision: str
    message: str
    state: ExplorationRuntimeState
    detection: Optional[FrontierDetectionResult] = None
    selection: Optional[GoalSelectionResult] = None
    selected_goal: Optional[ExplorationGoal] = None
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "decision": self.decision,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "state": self.state.to_dict(),
            "detection": (
                self.detection.to_dict()
                if self.detection is not None
                else None
            ),
            "selection": (
                self.selection.to_dict()
                if self.selection is not None
                else None
            ),
            "selected_goal": (
                self.selected_goal.to_dict()
                if self.selected_goal is not None
                else None
            ),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


class FrontierExplorerCore:
    def __init__(
        self,
        *,
        policy: Optional[ExplorationPolicy] = None,
        blacklist: Optional[GoalBlacklist] = None,
    ) -> None:
        self.policy = policy or make_autonomous_exploration_policy()
        blacklist_policy = self.policy.blacklist
        self.blacklist = blacklist or GoalBlacklist(
            default_radius_m=blacklist_policy.radius_m,
            default_ttl_s=blacklist_policy.ttl_s,
            max_records=blacklist_policy.max_records,
        )
        self.state = ExplorationRuntimeState()

    def run_cycle(self, cycle_input: ExplorationCycleInput) -> ExplorationCycleResult:
        if not self.policy.enabled:
            self.state = self.state.with_decision(DECISION_DISABLED)
            return ExplorationCycleResult(
                ok=False,
                decision=DECISION_DISABLED,
                message="Frontier exploration disabled.",
                state=self.state,
            )

        if self.policy.require_safety_clear and not cycle_input.safety.clear:
            reasons = ", ".join(cycle_input.safety.blocking_reasons())
            self.state = self.state.with_decision(
                DECISION_PAUSED,
                active_goal=self.state.active_goal,
                paused=True,
                pause_reason=reasons,
            )
            return ExplorationCycleResult(
                ok=False,
                decision=DECISION_PAUSED,
                message=f"Frontier exploration paused: {reasons}.",
                state=self.state,
            )

        map_input = cycle_input.map_input
        detection = detect_frontiers(
            width=map_input.width,
            height=map_input.height,
            resolution_m=map_input.resolution_m,
            values=map_input.values,
            origin_x=map_input.origin_x,
            origin_y=map_input.origin_y,
            origin_yaw=map_input.origin_yaw,
            frame_id=map_input.frame_id,
            occupied_threshold=self.policy.frontier.occupied_threshold,
            min_frontier_size_cells=self.policy.frontier.min_frontier_size_cells,
            diagonal_clustering=self.policy.frontier.diagonal_clustering,
        )

        if not detection.ok:
            no_candidate_cycles = self.state.no_candidate_cycles + 1
            decision = (
                DECISION_COMPLETE
                if no_candidate_cycles >= self.policy.stop.max_no_candidate_cycles
                else DECISION_WAITING
            )
            self.state = self.state.with_decision(
                decision,
                no_candidate_cycles=no_candidate_cycles,
            )
            return ExplorationCycleResult(
                ok=False,
                decision=decision,
                message=detection.message,
                state=self.state,
                detection=detection,
            )

        selection = select_goal_from_detection(
            detection,
            robot_pose=cycle_input.robot_pose,
            policy=self.policy.goal_selection,
            blacklist=(
                self.blacklist.to_goal_selector_entries()
                if self.policy.blacklist.enabled
                else ()
            ),
        )

        if not selection.ok or selection.selected_goal is None:
            self.state = self.state.with_decision(DECISION_WAITING)
            return ExplorationCycleResult(
                ok=False,
                decision=DECISION_WAITING,
                message=selection.message,
                state=self.state,
                detection=detection,
                selection=selection,
            )

        self.state = self.state.with_decision(
            DECISION_GOAL_SELECTED,
            active_goal=selection.selected_goal,
            no_candidate_cycles=0,
        )
        return ExplorationCycleResult(
            ok=True,
            decision=DECISION_GOAL_SELECTED,
            message="Frontier exploration goal selected.",
            state=self.state,
            detection=detection,
            selection=selection,
            selected_goal=selection.selected_goal,
        )

    def mark_goal_failed(self, *, reason: str = "nav2_failed") -> Optional[ExplorationGoal]:
        goal = self.state.active_goal

        if goal is None:
            return None

        if self.policy.blacklist.enabled:
            self.blacklist.add_goal(
                goal,
                radius_m=self.policy.blacklist.radius_m,
                reason=reason,
                ttl_s=self.policy.blacklist.ttl_s,
            )

        self.state = self.state.with_decision(
            DECISION_WAITING,
            active_goal=None,
            failed_goal_count=self.state.failed_goal_count + 1,
        )
        return goal

    def mark_goal_reached(self) -> Optional[ExplorationGoal]:
        goal = self.state.active_goal

        if goal is None:
            return None

        self.state = self.state.with_decision(
            DECISION_WAITING,
            active_goal=None,
            reached_goal_count=self.state.reached_goal_count + 1,
        )
        return goal

    def reset(self, *, clear_blacklist: bool = False) -> None:
        self.state = ExplorationRuntimeState()

        if clear_blacklist:
            self.blacklist.clear()


def make_cycle_input_from_grid(
    *,
    width: int,
    height: int,
    resolution_m: float,
    values: Sequence[int],
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
    frame_id: str = "map",
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
    safety: Optional[ExplorationSafetyState] = None,
) -> ExplorationCycleInput:
    return ExplorationCycleInput(
        map_input=ExplorationMapInput(
            width=width,
            height=height,
            resolution_m=resolution_m,
            values=values,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
            frame_id=frame_id,
        ),
        robot_pose=RobotPose2D(
            x=robot_x,
            y=robot_y,
            yaw=robot_yaw,
            frame_id=frame_id,
        ),
        safety=safety or ExplorationSafetyState(),
    )


__all__ = [
    "DECISION_COMPLETE",
    "DECISION_DISABLED",
    "DECISION_GOAL_SELECTED",
    "DECISION_KEEP_GOAL",
    "DECISION_PAUSED",
    "DECISION_STOPPED",
    "DECISION_WAITING",
    "ExplorationCycleInput",
    "ExplorationCycleResult",
    "ExplorationMapInput",
    "ExplorationRuntimeState",
    "ExplorationSafetyState",
    "FrontierExplorerCore",
    "make_cycle_input_from_grid",
]
