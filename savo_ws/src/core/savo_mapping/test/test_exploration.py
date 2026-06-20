#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping exploration helpers."""

from __future__ import annotations

import json

import pytest

from savo_mapping.exploration import (
    STRATEGY_AREA,
    STRATEGY_COVERAGE,
    STRATEGY_FRONTIER,
    STRATEGY_SEMANTIC,
    STRATEGY_WAYPOINT,
    AreaPolicy,
    AreaRegion,
    CoveragePolicy,
    GoalBlacklist,
    GoalSelectionPolicy,
    RobotPose2D,
    SemanticPolicy,
    Waypoint,
    WaypointPolicy,
    detect_frontiers,
    make_autonomous_exploration_policy,
    make_default_strategy_registry,
    make_initial_exploration_state,
    make_strategy_context_from_grid,
    make_target_from_apriltag,
    mark_area_visited,
    mark_semantic_target_confirmed,
    mark_waypoint_visited,
    select_area_goal,
    select_coverage_goal,
    select_goal_from_detection,
    select_semantic_goal,
    select_waypoint_goal,
    validate_strategy_name,
)
from savo_mapping.exploration.frontier_explorer_core import (
    DECISION_GOAL_SELECTED,
    DECISION_PAUSED,
    ExplorationCycleInput,
    ExplorationMapInput,
    ExplorationSafetyState,
    FrontierExplorerCore,
    make_cycle_input_from_grid,
)
from savo_mapping.models.exploration_status import ExplorationGoal


def _frontier_grid() -> tuple[int, int, list[int]]:
    width = 20
    height = 16
    values = [-1] * (width * height)

    for y in range(3, 12):
        for x in range(3, 12):
            values[y * width + x] = 0

    return width, height, values


def test_frontier_detection_finds_clusters() -> None:
    width, height, values = _frontier_grid()

    result = detect_frontiers(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        min_frontier_size_cells=2,
    )

    assert result.ok is True
    assert result.frontier_count > 0
    assert result.cell_count > 0
    assert result.best_cluster is not None


def test_goal_selector_selects_frontier_goal() -> None:
    width, height, values = _frontier_grid()

    detection = detect_frontiers(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        min_frontier_size_cells=2,
    )

    selected = select_goal_from_detection(
        detection,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=GoalSelectionPolicy(
            min_goal_distance_m=0.0,
            min_cluster_size_cells=2,
        ),
    )

    assert selected.ok is True
    assert selected.selected_goal is not None
    assert selected.candidate_count > 0


def test_goal_blacklist_blocks_near_goal() -> None:
    blacklist = GoalBlacklist(default_ttl_s=None)
    blacklist.add(2.0, 3.0, reason="nav2_failed")

    near = blacklist.check(2.1, 3.1)
    far = blacklist.check(5.0, 5.0)

    assert near.blocked is True
    assert near.record is not None
    assert near.record.hit_count == 1
    assert far.blocked is False
    assert blacklist.count == 1


def test_frontier_core_selects_goal() -> None:
    width, height, values = _frontier_grid()
    core = FrontierExplorerCore()

    result = core.run_cycle(
        make_cycle_input_from_grid(
            width=width,
            height=height,
            resolution_m=0.05,
            values=values,
        )
    )

    assert result.ok is True
    assert result.decision == DECISION_GOAL_SELECTED
    assert result.selected_goal is not None


def test_frontier_core_pauses_on_safety_stop() -> None:
    width, height, values = _frontier_grid()
    core = FrontierExplorerCore()

    result = core.run_cycle(
        ExplorationCycleInput(
            map_input=ExplorationMapInput(
                width=width,
                height=height,
                resolution_m=0.05,
                values=values,
            ),
            safety=ExplorationSafetyState(safety_stop=True),
        )
    )

    assert result.ok is False
    assert result.decision == DECISION_PAUSED
    assert result.state.paused is True
    assert "safety_stop" in result.state.pause_reason


def test_coverage_goal_selection() -> None:
    width, height, values = _frontier_grid()

    result = select_coverage_goal(
        width=width,
        height=height,
        resolution_m=0.05,
        values=values,
        robot_pose=RobotPose2D(x=0.0, y=0.0),
        policy=CoveragePolicy(min_goal_distance_m=0.0),
    )

    assert result.ok is True
    assert result.selected_goal is not None
    assert result.candidate_count > 0


def test_waypoint_goal_selection_and_mark_visited() -> None:
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

    assert result.ok is True
    assert result.selected_waypoint is not None
    assert result.selected_waypoint.name == "a201_area"

    updated = mark_waypoint_visited(waypoints, result.selected_waypoint)

    assert any(item.name == "a201_area" and item.visited for item in updated)


def test_area_goal_selection_and_mark_visited() -> None:
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

    assert result.ok is True
    assert result.selected_area is not None
    assert result.selected_area.key == "elevator_area"

    updated = mark_area_visited(areas, result.selected_area)

    assert any(item.key == "elevator_area" and item.visited for item in updated)


def test_semantic_goal_selection_and_confirm() -> None:
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

    assert result.ok is True
    assert result.selected_target is not None
    assert result.selected_target.key == "info_desk"

    confirmed = mark_semantic_target_confirmed(targets, result.selected_target)

    assert any(item.key == "info_desk" and item.confirmed for item in confirmed)


def test_exploration_state_tracks_goal_flow() -> None:
    goal = ExplorationGoal(
        x=2.0,
        y=3.0,
        yaw=0.0,
        frame_id="map",
        score=0.9,
        reason="frontier_selected",
    )

    state = make_initial_exploration_state()
    state = state.start()
    state = state.select_goal(goal)
    state = state.mark_goal_reached()

    assert state.state == "running"
    assert state.active_goal is None
    assert state.last_goal is not None
    assert state.counters.selected_goals == 1
    assert state.counters.reached_goals == 1
    assert [event.event for event in state.events] == [
        "started",
        "goal_selected",
        "goal_reached",
        "goal_attempt_finished",
    ]


def test_exploration_policy_defaults() -> None:
    policy = make_autonomous_exploration_policy()

    assert policy.enabled is True
    assert policy.mode == "autonomous_mapping"
    assert policy.require_nav2 is True
    assert policy.frontier.min_frontier_size_cells > 0
    assert policy.blacklist.enabled is True


def test_strategy_name_validation() -> None:
    assert validate_strategy_name("frontier") == STRATEGY_FRONTIER
    assert validate_strategy_name("semantic") == STRATEGY_SEMANTIC
    assert validate_strategy_name("waypoint") == STRATEGY_WAYPOINT

    with pytest.raises(ValueError):
        validate_strategy_name("room")


def test_strategy_registry_contains_all_strategies() -> None:
    registry = make_default_strategy_registry(
        enable_coverage=True,
        enable_waypoint=True,
        enable_area=True,
        enable_semantic=True,
    )

    assert registry.names() == (
        STRATEGY_FRONTIER,
        STRATEGY_COVERAGE,
        STRATEGY_WAYPOINT,
        STRATEGY_AREA,
        STRATEGY_SEMANTIC,
    )

    statuses = registry.statuses()

    assert set(statuses) == set(registry.names())


def test_waypoint_strategy_runs_real_helper() -> None:
    registry = make_default_strategy_registry(enable_waypoint=True)

    context = make_strategy_context_from_grid(
        width=6,
        height=6,
        resolution_m=0.05,
        values=[-1] * 36,
        waypoints=(
            Waypoint(x=1.0, y=1.0, name="point_a", priority=1),
            Waypoint(x=2.0, y=2.0, name="point_b", priority=2),
        ),
    )

    result = registry.run_cycle(STRATEGY_WAYPOINT, context)

    assert result.ok is True
    assert result.decision == DECISION_GOAL_SELECTED
    assert result.selected_goal is not None


def test_area_strategy_runs_real_helper() -> None:
    registry = make_default_strategy_registry(enable_area=True)

    context = make_strategy_context_from_grid(
        width=6,
        height=6,
        resolution_m=0.05,
        values=[-1] * 36,
        areas=(
            AreaRegion(
                key="info_desk",
                label="Info Desk",
                center_x=2.0,
                center_y=1.5,
                priority=3,
                confirmed=True,
            ),
        ),
    )

    result = registry.run_cycle(STRATEGY_AREA, context)

    assert result.ok is True
    assert result.decision == DECISION_GOAL_SELECTED
    assert result.selected_goal is not None


def test_semantic_strategy_runs_real_helper() -> None:
    registry = make_default_strategy_registry(enable_semantic=True)

    context = make_strategy_context_from_grid(
        width=6,
        height=6,
        resolution_m=0.05,
        values=[-1] * 36,
        semantic_targets=(
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
        ),
    )

    result = registry.run_cycle(STRATEGY_SEMANTIC, context)

    assert result.ok is True
    assert result.decision == DECISION_GOAL_SELECTED
    assert result.selected_goal is not None


def test_exploration_result_json_is_valid() -> None:
    width, height, values = _frontier_grid()
    core = FrontierExplorerCore()

    result = core.run_cycle(
        make_cycle_input_from_grid(
            width=width,
            height=height,
            resolution_m=0.05,
            values=values,
        )
    )

    data = json.loads(result.to_json())

    assert data["ok"] is True
    assert data["decision"] == DECISION_GOAL_SELECTED
    assert data["selected_goal"] is not None
