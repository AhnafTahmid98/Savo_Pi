#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared exploration strategy interface for Robot Savo mapping."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Protocol, Sequence

from savo_mapping.exploration.area_explorer import (
    AreaPolicy,
    AreaRegion,
    select_area_goal,
)
from savo_mapping.exploration.coverage_explorer import (
    CoveragePolicy,
    select_coverage_goal,
)
from savo_mapping.exploration.exploration_policy import (
    ExplorationPolicy,
    make_autonomous_exploration_policy,
)
from savo_mapping.exploration.frontier_explorer_core import (
    DECISION_DISABLED,
    DECISION_GOAL_SELECTED,
    DECISION_WAITING,
    ExplorationCycleInput,
    ExplorationCycleResult,
    ExplorationMapInput,
    ExplorationRuntimeState,
    ExplorationSafetyState,
    FrontierExplorerCore,
)
from savo_mapping.exploration.goal_selector import RobotPose2D
from savo_mapping.exploration.semantic_explorer import (
    SemanticPolicy,
    SemanticTarget,
    select_semantic_goal,
)
from savo_mapping.exploration.waypoint_explorer import (
    Waypoint,
    WaypointPolicy,
    select_waypoint_goal,
)
from savo_mapping.models.exploration_status import ExplorationGoal


STRATEGY_FRONTIER = "frontier"
STRATEGY_COVERAGE = "coverage"
STRATEGY_WAYPOINT = "waypoint"
STRATEGY_AREA = "area"
STRATEGY_SEMANTIC = "semantic"

SUPPORTED_STRATEGIES = (
    STRATEGY_FRONTIER,
    STRATEGY_COVERAGE,
    STRATEGY_WAYPOINT,
    STRATEGY_AREA,
    STRATEGY_SEMANTIC,
)


@dataclass(frozen=True)
class ExplorationStrategyContext:
    map_input: ExplorationMapInput
    robot_pose: RobotPose2D = field(default_factory=RobotPose2D)
    safety: ExplorationSafetyState = field(default_factory=ExplorationSafetyState)
    map_name: Optional[str] = None
    session_id: Optional[str] = None
    waypoints: tuple[Waypoint, ...] = ()
    areas: tuple[AreaRegion, ...] = ()
    semantic_targets: tuple[SemanticTarget, ...] = ()
    extra: dict[str, Any] = field(default_factory=dict)

    def to_cycle_input(self) -> ExplorationCycleInput:
        return ExplorationCycleInput(
            map_input=self.map_input,
            robot_pose=self.robot_pose,
            safety=self.safety,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "map_input": self.map_input.to_dict(),
            "robot_pose": self.robot_pose.to_dict(),
            "safety": self.safety.to_dict(),
            "map_name": self.map_name,
            "session_id": self.session_id,
            "waypoints": [waypoint.to_dict() for waypoint in self.waypoints],
            "areas": [area.to_dict() for area in self.areas],
            "semantic_targets": [
                target.to_dict()
                for target in self.semantic_targets
            ],
            "extra": dict(self.extra),
        }


@dataclass(frozen=True)
class ExplorationStrategyStatus:
    name: str
    enabled: bool
    active: bool = False
    message: str = ""
    selected_goal: Optional[ExplorationGoal] = None
    state: Optional[dict[str, Any]] = None
    timestamp_s: float = field(default_factory=time.time)
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "enabled": self.enabled,
            "active": self.active,
            "message": self.message,
            "selected_goal": (
                self.selected_goal.to_dict()
                if self.selected_goal is not None
                else None
            ),
            "state": self.state,
            "timestamp_s": self.timestamp_s,
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


class ExplorationStrategy(Protocol):
    @property
    def name(self) -> str:
        ...

    @property
    def enabled(self) -> bool:
        ...

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        ...

    def mark_goal_failed(
        self,
        *,
        reason: str = "nav2_failed",
    ) -> Optional[ExplorationGoal]:
        ...

    def mark_goal_reached(self) -> Optional[ExplorationGoal]:
        ...

    def reset(self, *, clear_blacklist: bool = False) -> None:
        ...

    def status(self) -> ExplorationStrategyStatus:
        ...


class FrontierExplorationStrategy:
    def __init__(
        self,
        *,
        policy: Optional[ExplorationPolicy] = None,
        core: Optional[FrontierExplorerCore] = None,
    ) -> None:
        self._policy = policy or make_autonomous_exploration_policy()
        self._core = core or FrontierExplorerCore(policy=self._policy)

    @property
    def name(self) -> str:
        return STRATEGY_FRONTIER

    @property
    def enabled(self) -> bool:
        return bool(self._policy.enabled)

    @property
    def core(self) -> FrontierExplorerCore:
        return self._core

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        return self._core.run_cycle(context.to_cycle_input())

    def mark_goal_failed(
        self,
        *,
        reason: str = "nav2_failed",
    ) -> Optional[ExplorationGoal]:
        return self._core.mark_goal_failed(reason=reason)

    def mark_goal_reached(self) -> Optional[ExplorationGoal]:
        return self._core.mark_goal_reached()

    def reset(self, *, clear_blacklist: bool = False) -> None:
        self._core.reset(clear_blacklist=clear_blacklist)

    def status(self) -> ExplorationStrategyStatus:
        state = self._core.state

        return ExplorationStrategyStatus(
            name=self.name,
            enabled=self.enabled,
            active=state.active_goal is not None,
            message="frontier strategy ready.",
            selected_goal=state.active_goal,
            state=state.to_dict(),
            extra={
                "blacklist_count": self._core.blacklist.count,
                "policy": self._policy.to_dict(),
            },
        )


class CoverageExplorationStrategy:
    def __init__(
        self,
        *,
        enabled: bool = False,
        policy: Optional[CoveragePolicy] = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._policy = policy or CoveragePolicy()
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    @property
    def name(self) -> str:
        return STRATEGY_COVERAGE

    @property
    def enabled(self) -> bool:
        return self._enabled

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        if not self.enabled:
            return self._disabled_result()

        result = select_coverage_goal(
            width=context.map_input.width,
            height=context.map_input.height,
            resolution_m=context.map_input.resolution_m,
            values=context.map_input.values,
            robot_pose=context.robot_pose,
            policy=self._policy,
            origin_x=context.map_input.origin_x,
            origin_y=context.map_input.origin_y,
            origin_yaw=context.map_input.origin_yaw,
            frame_id=context.map_input.frame_id,
        )

        return self._result_from_goal_result(
            ok=result.ok,
            message=result.message,
            selected_goal=result.selected_goal,
            extra={"coverage": result.to_dict()},
        )

    def mark_goal_failed(
        self,
        *,
        reason: str = "nav2_failed",
    ) -> Optional[ExplorationGoal]:
        goal = self._state.active_goal
        self._state = ExplorationRuntimeState(
            previous_goal=goal,
            failed_goal_count=self._state.failed_goal_count + int(goal is not None),
            last_decision="goal_failed",
        )
        return goal

    def mark_goal_reached(self) -> Optional[ExplorationGoal]:
        goal = self._state.active_goal
        self._state = ExplorationRuntimeState(
            previous_goal=goal,
            reached_goal_count=self._state.reached_goal_count + int(goal is not None),
            last_decision="goal_reached",
        )
        return goal

    def reset(self, *, clear_blacklist: bool = False) -> None:
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    def status(self) -> ExplorationStrategyStatus:
        return _status_from_state(
            name=self.name,
            enabled=self.enabled,
            state=self._state,
            message="coverage strategy ready.",
            extra={"policy": self._policy.to_dict()},
        )

    def _disabled_result(self) -> ExplorationCycleResult:
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)
        return ExplorationCycleResult(
            ok=False,
            decision=DECISION_DISABLED,
            message="coverage strategy is disabled.",
            state=self._state,
        )

    def _result_from_goal_result(
        self,
        *,
        ok: bool,
        message: str,
        selected_goal: Optional[ExplorationGoal],
        extra: dict[str, Any],
    ) -> ExplorationCycleResult:
        decision = DECISION_GOAL_SELECTED if ok else DECISION_WAITING

        self._state = ExplorationRuntimeState(
            active_goal=selected_goal,
            previous_goal=self._state.active_goal,
            last_decision=decision,
        )

        return ExplorationCycleResult(
            ok=ok,
            decision=decision,
            message=message,
            state=self._state,
            selected_goal=selected_goal,
        )


class WaypointExplorationStrategy:
    def __init__(
        self,
        *,
        enabled: bool = False,
        policy: Optional[WaypointPolicy] = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._policy = policy or WaypointPolicy()
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    @property
    def name(self) -> str:
        return STRATEGY_WAYPOINT

    @property
    def enabled(self) -> bool:
        return self._enabled

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        if not self.enabled:
            return self._disabled_result()

        result = select_waypoint_goal(
            context.waypoints,
            robot_pose=context.robot_pose,
            policy=self._policy,
        )

        return self._result_from_goal_result(
            ok=result.ok,
            message=result.message,
            selected_goal=result.selected_goal,
        )

    def mark_goal_failed(
        self,
        *,
        reason: str = "nav2_failed",
    ) -> Optional[ExplorationGoal]:
        return _mark_failed(self)

    def mark_goal_reached(self) -> Optional[ExplorationGoal]:
        return _mark_reached(self)

    def reset(self, *, clear_blacklist: bool = False) -> None:
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    def status(self) -> ExplorationStrategyStatus:
        return _status_from_state(
            name=self.name,
            enabled=self.enabled,
            state=self._state,
            message="waypoint strategy ready.",
            extra={"policy": self._policy.to_dict()},
        )

    def _disabled_result(self) -> ExplorationCycleResult:
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)
        return ExplorationCycleResult(
            ok=False,
            decision=DECISION_DISABLED,
            message="waypoint strategy is disabled.",
            state=self._state,
        )

    def _result_from_goal_result(
        self,
        *,
        ok: bool,
        message: str,
        selected_goal: Optional[ExplorationGoal],
    ) -> ExplorationCycleResult:
        decision = DECISION_GOAL_SELECTED if ok else DECISION_WAITING

        self._state = ExplorationRuntimeState(
            active_goal=selected_goal,
            previous_goal=self._state.active_goal,
            last_decision=decision,
        )

        return ExplorationCycleResult(
            ok=ok,
            decision=decision,
            message=message,
            state=self._state,
            selected_goal=selected_goal,
        )


class AreaExplorationStrategy(WaypointExplorationStrategy):
    def __init__(
        self,
        *,
        enabled: bool = False,
        policy: Optional[AreaPolicy] = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._policy = policy or AreaPolicy()
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    @property
    def name(self) -> str:
        return STRATEGY_AREA

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        if not self.enabled:
            return self._disabled_result()

        result = select_area_goal(
            context.areas,
            robot_pose=context.robot_pose,
            policy=self._policy,
        )

        return self._result_from_goal_result(
            ok=result.ok,
            message=result.message,
            selected_goal=result.selected_goal,
        )

    def status(self) -> ExplorationStrategyStatus:
        return _status_from_state(
            name=self.name,
            enabled=self.enabled,
            state=self._state,
            message="area strategy ready.",
            extra={"policy": self._policy.to_dict()},
        )


class SemanticExplorationStrategy(WaypointExplorationStrategy):
    def __init__(
        self,
        *,
        enabled: bool = False,
        policy: Optional[SemanticPolicy] = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._policy = policy or SemanticPolicy()
        self._state = ExplorationRuntimeState(last_decision=DECISION_DISABLED)

    @property
    def name(self) -> str:
        return STRATEGY_SEMANTIC

    def run_cycle(
        self,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        if not self.enabled:
            return self._disabled_result()

        result = select_semantic_goal(
            context.semantic_targets,
            robot_pose=context.robot_pose,
            policy=self._policy,
        )

        return self._result_from_goal_result(
            ok=result.ok,
            message=result.message,
            selected_goal=result.selected_goal,
        )

    def status(self) -> ExplorationStrategyStatus:
        return _status_from_state(
            name=self.name,
            enabled=self.enabled,
            state=self._state,
            message="semantic strategy ready.",
            extra={"policy": self._policy.to_dict()},
        )


class ExplorationStrategyRegistry:
    def __init__(self) -> None:
        self._strategies: dict[str, ExplorationStrategy] = {}

    def register(self, strategy: ExplorationStrategy) -> None:
        name = validate_strategy_name(strategy.name)
        self._strategies[name] = strategy

    def get(self, name: str) -> ExplorationStrategy:
        clean_name = validate_strategy_name(name)

        if clean_name not in self._strategies:
            raise KeyError(f"Exploration strategy not registered: {clean_name}")

        return self._strategies[clean_name]

    def names(self) -> tuple[str, ...]:
        return tuple(self._strategies.keys())

    def statuses(self) -> dict[str, dict[str, Any]]:
        return {
            name: strategy.status().to_dict()
            for name, strategy in self._strategies.items()
        }

    def run_cycle(
        self,
        name: str,
        context: ExplorationStrategyContext,
    ) -> ExplorationCycleResult:
        return self.get(name).run_cycle(context)

    def reset_all(self, *, clear_blacklist: bool = False) -> None:
        for strategy in self._strategies.values():
            strategy.reset(clear_blacklist=clear_blacklist)


def make_default_strategy_registry(
    *,
    frontier_policy: Optional[ExplorationPolicy] = None,
    enable_coverage: bool = False,
    enable_waypoint: bool = False,
    enable_area: bool = False,
    enable_semantic: bool = False,
) -> ExplorationStrategyRegistry:
    registry = ExplorationStrategyRegistry()

    registry.register(
        FrontierExplorationStrategy(
            policy=frontier_policy or make_autonomous_exploration_policy(),
        )
    )
    registry.register(CoverageExplorationStrategy(enabled=enable_coverage))
    registry.register(WaypointExplorationStrategy(enabled=enable_waypoint))
    registry.register(AreaExplorationStrategy(enabled=enable_area))
    registry.register(SemanticExplorationStrategy(enabled=enable_semantic))

    return registry


def validate_strategy_name(name: str) -> str:
    clean_name = str(name).strip().lower().replace("-", "_").replace(" ", "_")

    if clean_name not in SUPPORTED_STRATEGIES:
        raise ValueError(f"Unsupported exploration strategy: {name}")

    return clean_name


def make_strategy_context_from_grid(
    *,
    width: int,
    height: int,
    resolution_m: float,
    values: Sequence[int],
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
    frame_id: str = "map",
    map_name: Optional[str] = None,
    session_id: Optional[str] = None,
    waypoints: Sequence[Waypoint] = (),
    areas: Sequence[AreaRegion] = (),
    semantic_targets: Sequence[SemanticTarget] = (),
) -> ExplorationStrategyContext:
    return ExplorationStrategyContext(
        map_input=ExplorationMapInput(
            width=width,
            height=height,
            resolution_m=resolution_m,
            values=values,
            frame_id=frame_id,
        ),
        robot_pose=RobotPose2D(
            x=robot_x,
            y=robot_y,
            yaw=robot_yaw,
            frame_id=frame_id,
        ),
        map_name=map_name,
        session_id=session_id,
        waypoints=tuple(waypoints),
        areas=tuple(areas),
        semantic_targets=tuple(semantic_targets),
    )


def _status_from_state(
    *,
    name: str,
    enabled: bool,
    state: ExplorationRuntimeState,
    message: str,
    extra: Optional[dict[str, Any]] = None,
) -> ExplorationStrategyStatus:
    return ExplorationStrategyStatus(
        name=name,
        enabled=enabled,
        active=state.active_goal is not None,
        message=message,
        selected_goal=state.active_goal,
        state=state.to_dict(),
        extra=extra or {},
    )


def _mark_failed(strategy: Any) -> Optional[ExplorationGoal]:
    goal = strategy._state.active_goal
    strategy._state = ExplorationRuntimeState(
        previous_goal=goal,
        failed_goal_count=strategy._state.failed_goal_count + int(goal is not None),
        last_decision="goal_failed",
    )
    return goal


def _mark_reached(strategy: Any) -> Optional[ExplorationGoal]:
    goal = strategy._state.active_goal
    strategy._state = ExplorationRuntimeState(
        previous_goal=goal,
        reached_goal_count=strategy._state.reached_goal_count + int(goal is not None),
        last_decision="goal_reached",
    )
    return goal


def main() -> None:
    width = 20
    height = 16
    values = [-1] * (width * height)

    for y in range(3, 12):
        for x in range(3, 12):
            values[y * width + x] = 0

    registry = make_default_strategy_registry(enable_coverage=True)
    context = make_strategy_context_from_grid(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        map_name="savonia_campus_heart",
        session_id="demo_session",
    )

    result = registry.run_cycle(STRATEGY_FRONTIER, context)

    print(result.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "STRATEGY_AREA",
    "STRATEGY_COVERAGE",
    "STRATEGY_FRONTIER",
    "STRATEGY_SEMANTIC",
    "STRATEGY_WAYPOINT",
    "SUPPORTED_STRATEGIES",
    "AreaExplorationStrategy",
    "CoverageExplorationStrategy",
    "ExplorationStrategy",
    "ExplorationStrategyContext",
    "ExplorationStrategyRegistry",
    "ExplorationStrategyStatus",
    "FrontierExplorationStrategy",
    "SemanticExplorationStrategy",
    "WaypointExplorationStrategy",
    "make_default_strategy_registry",
    "make_strategy_context_from_grid",
    "validate_strategy_name",
]