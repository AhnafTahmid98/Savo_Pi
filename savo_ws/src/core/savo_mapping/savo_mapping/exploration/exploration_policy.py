#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Exploration policy helpers for Robot Savo mapping."""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from typing import Any, Mapping, Optional

from savo_mapping.exploration.frontier_detector import DEFAULT_OCCUPIED_THRESHOLD
from savo_mapping.exploration.goal_selector import GoalSelectionPolicy


@dataclass(frozen=True)
class FrontierDetectionPolicy:
    min_frontier_size_cells: int = 3
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD
    diagonal_clustering: bool = True

    def __post_init__(self) -> None:
        if self.min_frontier_size_cells <= 0:
            raise ValueError("min_frontier_size_cells must be positive.")

        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be between 0 and 100.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "min_frontier_size_cells": self.min_frontier_size_cells,
            "occupied_threshold": self.occupied_threshold,
            "diagonal_clustering": self.diagonal_clustering,
        }


@dataclass(frozen=True)
class ExplorationStopPolicy:
    max_failed_goals: int = 20
    max_no_candidate_cycles: int = 10
    max_same_goal_retries: int = 2
    pause_on_safety_stop: bool = True
    pause_on_stale_scan: bool = True
    pause_on_stale_odom: bool = True
    pause_on_tf_failure: bool = True
    pause_on_low_battery: bool = True

    def __post_init__(self) -> None:
        if self.max_failed_goals < 0:
            raise ValueError("max_failed_goals must be non-negative.")

        if self.max_no_candidate_cycles < 0:
            raise ValueError("max_no_candidate_cycles must be non-negative.")

        if self.max_same_goal_retries < 0:
            raise ValueError("max_same_goal_retries must be non-negative.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "max_failed_goals": self.max_failed_goals,
            "max_no_candidate_cycles": self.max_no_candidate_cycles,
            "max_same_goal_retries": self.max_same_goal_retries,
            "pause_on_safety_stop": self.pause_on_safety_stop,
            "pause_on_stale_scan": self.pause_on_stale_scan,
            "pause_on_stale_odom": self.pause_on_stale_odom,
            "pause_on_tf_failure": self.pause_on_tf_failure,
            "pause_on_low_battery": self.pause_on_low_battery,
        }


@dataclass(frozen=True)
class ExplorationBlacklistPolicy:
    enabled: bool = True
    radius_m: float = 0.60
    ttl_s: Optional[float] = 300.0
    max_records: int = 100

    def __post_init__(self) -> None:
        if self.radius_m <= 0.0:
            raise ValueError("blacklist radius_m must be positive.")

        if self.ttl_s is not None and self.ttl_s <= 0.0:
            raise ValueError("blacklist ttl_s must be positive or None.")

        if self.max_records <= 0:
            raise ValueError("blacklist max_records must be positive.")

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "radius_m": self.radius_m,
            "ttl_s": self.ttl_s,
            "max_records": self.max_records,
        }


@dataclass(frozen=True)
class ExplorationPolicy:
    enabled: bool = False
    mode: str = "manual_mapping"
    publish_rate_hz: float = 2.0
    goal_frame: str = "map"
    require_nav2: bool = True
    require_map: bool = True
    require_safety_clear: bool = True
    frontier: FrontierDetectionPolicy = FrontierDetectionPolicy()
    goal_selection: GoalSelectionPolicy = GoalSelectionPolicy()
    blacklist: ExplorationBlacklistPolicy = ExplorationBlacklistPolicy()
    stop: ExplorationStopPolicy = ExplorationStopPolicy()

    def __post_init__(self) -> None:
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive.")

        if not str(self.goal_frame).strip():
            raise ValueError("goal_frame cannot be empty.")

    def with_updates(self, **updates: Any) -> "ExplorationPolicy":
        return replace(self, **updates)

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "publish_rate_hz": self.publish_rate_hz,
            "goal_frame": self.goal_frame,
            "require_nav2": self.require_nav2,
            "require_map": self.require_map,
            "require_safety_clear": self.require_safety_clear,
            "frontier": self.frontier.to_dict(),
            "goal_selection": self.goal_selection.to_dict(),
            "blacklist": self.blacklist.to_dict(),
            "stop": self.stop.to_dict(),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


def make_manual_exploration_policy() -> ExplorationPolicy:
    return ExplorationPolicy(
        enabled=False,
        mode="manual_mapping",
        require_nav2=False,
        require_map=False,
        require_safety_clear=True,
        frontier=FrontierDetectionPolicy(
            min_frontier_size_cells=3,
            occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
            diagonal_clustering=True,
        ),
        goal_selection=GoalSelectionPolicy(
            min_goal_distance_m=0.30,
            max_goal_distance_m=8.0,
            min_cluster_size_cells=3,
            prefer_nearest=True,
        ),
    )


def make_autonomous_exploration_policy() -> ExplorationPolicy:
    return ExplorationPolicy(
        enabled=True,
        mode="autonomous_mapping",
        require_nav2=True,
        require_map=True,
        require_safety_clear=True,
        frontier=FrontierDetectionPolicy(
            min_frontier_size_cells=5,
            occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
            diagonal_clustering=True,
        ),
        goal_selection=GoalSelectionPolicy(
            min_goal_distance_m=0.35,
            max_goal_distance_m=10.0,
            min_cluster_size_cells=5,
            distance_weight=1.0,
            size_weight=0.15,
            prefer_nearest=True,
            blacklist_radius_m=0.60,
        ),
        blacklist=ExplorationBlacklistPolicy(
            enabled=True,
            radius_m=0.60,
            ttl_s=300.0,
            max_records=100,
        ),
        stop=ExplorationStopPolicy(
            max_failed_goals=20,
            max_no_candidate_cycles=10,
            max_same_goal_retries=2,
        ),
    )


def exploration_policy_from_dict(data: Mapping[str, Any]) -> ExplorationPolicy:
    frontier_data = _mapping(data.get("frontier", {}))
    goal_data = _mapping(data.get("goal_selection", {}))
    blacklist_data = _mapping(data.get("blacklist", {}))
    stop_data = _mapping(data.get("stop", {}))

    return ExplorationPolicy(
        enabled=bool(data.get("enabled", False)),
        mode=str(data.get("mode", "manual_mapping")),
        publish_rate_hz=float(data.get("publish_rate_hz", 2.0)),
        goal_frame=str(data.get("goal_frame", "map")),
        require_nav2=bool(data.get("require_nav2", True)),
        require_map=bool(data.get("require_map", True)),
        require_safety_clear=bool(data.get("require_safety_clear", True)),
        frontier=FrontierDetectionPolicy(
            min_frontier_size_cells=int(
                frontier_data.get("min_frontier_size_cells", 3)
            ),
            occupied_threshold=int(
                frontier_data.get("occupied_threshold", DEFAULT_OCCUPIED_THRESHOLD)
            ),
            diagonal_clustering=bool(
                frontier_data.get("diagonal_clustering", True)
            ),
        ),
        goal_selection=GoalSelectionPolicy(
            min_goal_distance_m=float(goal_data.get("min_goal_distance_m", 0.30)),
            max_goal_distance_m=float(goal_data.get("max_goal_distance_m", 12.0)),
            min_cluster_size_cells=int(goal_data.get("min_cluster_size_cells", 3)),
            distance_weight=float(goal_data.get("distance_weight", 1.0)),
            size_weight=float(goal_data.get("size_weight", 0.15)),
            prefer_nearest=bool(goal_data.get("prefer_nearest", True)),
            blacklist_radius_m=float(goal_data.get("blacklist_radius_m", 0.60)),
        ),
        blacklist=ExplorationBlacklistPolicy(
            enabled=bool(blacklist_data.get("enabled", True)),
            radius_m=float(blacklist_data.get("radius_m", 0.60)),
            ttl_s=_optional_float(blacklist_data.get("ttl_s", 300.0)),
            max_records=int(blacklist_data.get("max_records", 100)),
        ),
        stop=ExplorationStopPolicy(
            max_failed_goals=int(stop_data.get("max_failed_goals", 20)),
            max_no_candidate_cycles=int(
                stop_data.get("max_no_candidate_cycles", 10)
            ),
            max_same_goal_retries=int(stop_data.get("max_same_goal_retries", 2)),
            pause_on_safety_stop=bool(stop_data.get("pause_on_safety_stop", True)),
            pause_on_stale_scan=bool(stop_data.get("pause_on_stale_scan", True)),
            pause_on_stale_odom=bool(stop_data.get("pause_on_stale_odom", True)),
            pause_on_tf_failure=bool(stop_data.get("pause_on_tf_failure", True)),
            pause_on_low_battery=bool(stop_data.get("pause_on_low_battery", True)),
        ),
    )


def merge_exploration_policy(
    base: ExplorationPolicy,
    overrides: Mapping[str, Any],
) -> ExplorationPolicy:
    data = base.to_dict()
    merged = _deep_merge(data, dict(overrides))
    return exploration_policy_from_dict(merged)


def _mapping(value: Any) -> Mapping[str, Any]:
    if value is None:
        return {}

    if not isinstance(value, Mapping):
        raise ValueError("Policy section must be a mapping.")

    return value


def _optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None

    return float(value)


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)

    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, Mapping):
            result[key] = _deep_merge(result[key], dict(value))
        else:
            result[key] = value

    return result


def main() -> None:
    policy = make_autonomous_exploration_policy()
    print(policy.to_json(indent=2))


if __name__ == "__main__":
    main()


__all__ = [
    "ExplorationBlacklistPolicy",
    "ExplorationPolicy",
    "ExplorationStopPolicy",
    "FrontierDetectionPolicy",
    "exploration_policy_from_dict",
    "make_autonomous_exploration_policy",
    "make_manual_exploration_policy",
    "merge_exploration_policy",
]