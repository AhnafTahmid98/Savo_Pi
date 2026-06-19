#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Autonomous mapping helper CLI for Robot Savo."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.models.exploration_status import (  # noqa: E402
    ExplorationGoal,
    make_goal_selected_status,
    make_idle_exploration_status,
    make_navigating_status,
)
from savo_mapping.models.map_metadata import make_saved_map_paths, sanitize_map_name  # noqa: E402
from savo_mapping.models.mapping_mode import MappingMode  # noqa: E402
from savo_mapping.models.mapping_status import make_autonomous_mapping_status  # noqa: E402
from savo_mapping.models.readiness_state import build_autonomous_mapping_ready_state  # noqa: E402
from savo_mapping.ros.params import (  # noqa: E402
    frontier_explorer_params_from_dict,
    mapping_supervisor_params_from_dict,
    pointcloud_monitor_params_from_dict,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare an autonomous frontier-mapping session for Robot Savo.",
    )

    parser.add_argument(
        "--map-name",
        default="Savonia Campus Heart",
        help="Map name to use when saving later.",
    )
    parser.add_argument(
        "--profile",
        default="autonomous_mapping_real_robot_v1.yaml",
        help="Autonomous mapping profile name.",
    )
    parser.add_argument(
        "--maps-dir",
        default="maps/saved",
        help="Saved map directory.",
    )
    parser.add_argument(
        "--driver-impl",
        default="cpp",
        choices=("cpp", "py"),
        help="Preferred runtime implementation.",
    )
    parser.add_argument(
        "--with-rviz",
        action="store_true",
        help="Show the PC-side RViz command in the output.",
    )
    parser.add_argument(
        "--with-pointcloud",
        action="store_true",
        help="Enable future RealSense pointcloud / voxel assist in the plan.",
    )
    parser.add_argument(
        "--dry-status",
        action="store_true",
        help="Print simulated autonomous mapping status.",
    )
    parser.add_argument(
        "--dry-goal",
        action="store_true",
        help="Print simulated frontier goal status.",
    )

    return parser


# =============================================================================
# Command builders
# =============================================================================
def _autonomous_mapping_launch_command(
    profile: str,
    driver_impl: str,
    with_rviz: bool,
    with_pointcloud: bool,
) -> str:
    rviz_value = "true" if with_rviz else "false"
    pointcloud_value = "true" if with_pointcloud else "false"

    return (
        "ros2 launch savo_mapping autonomous_mapping.launch.py "
        f"profile:={profile} "
        f"driver_impl:={driver_impl} "
        f"use_rviz:={rviz_value} "
        f"use_pointcloud:={pointcloud_value}"
    )


def _rviz_command() -> str:
    return "ros2 launch savo_mapping mapping_rviz.launch.py rviz_config:=autonomous_mapping"


def _save_map_command(map_name: str) -> str:
    clean_name = sanitize_map_name(map_name)

    return (
        "ros2 launch savo_mapping mapping_save.launch.py "
        f"map_name:={clean_name}"
    )


def _metadata_command(map_name: str) -> str:
    return (
        "ros2 run savo_mapping save_map_cli.py "
        f'--map-name "{map_name}" '
        "--write-metadata"
    )


def _pause_note() -> str:
    return (
        "Autonomous mapping should pause on safety stop, localization failure, "
        "stale scan, stale odom, or repeated failed goals."
    )


# =============================================================================
# Output
# =============================================================================
def _print_session_plan(args: argparse.Namespace) -> None:
    clean_name = sanitize_map_name(args.map_name)
    paths = make_saved_map_paths(clean_name, maps_dir=args.maps_dir)

    supervisor_params = mapping_supervisor_params_from_dict(
        {
            "require_scan": True,
            "require_odom": True,
            "require_tf": True,
            "require_map": True,
            "require_pointcloud": bool(args.with_pointcloud),
        }
    )

    frontier_params = frontier_explorer_params_from_dict(
        {
            "enabled": True,
            "min_frontier_size_cells": 5,
            "goal_clearance_m": 0.35,
            "goal_reached_radius_m": 0.35,
            "failed_goal_blacklist_radius_m": 0.60,
            "max_failed_goals": 20,
        }
    )

    pointcloud_params = pointcloud_monitor_params_from_dict(
        {
            "enabled": bool(args.with_pointcloud),
        }
    )

    print("Robot Savo autonomous mapping session")
    print(f"- mode: {MappingMode.AUTONOMOUS_MAPPING.value}")
    print(f"- map_name: {clean_name}")
    print(f"- profile: {args.profile}")
    print(f"- runtime: {args.driver_impl}")
    print(f"- scan_topic: {supervisor_params.scan_topic}")
    print(f"- odom_topic: {supervisor_params.odom_topic}")
    print(f"- map_topic: {supervisor_params.map_topic}")
    print(f"- pointcloud_enabled: {pointcloud_params.enabled}")
    print(f"- pointcloud_topic: {pointcloud_params.pointcloud_topic}")
    print(f"- frontier_enabled: {frontier_params.enabled}")
    print(f"- goal_clearance_m: {frontier_params.goal_clearance_m}")
    print(f"- max_failed_goals: {frontier_params.max_failed_goals}")
    print(f"- yaml_file: {paths['yaml_file']}")
    print(f"- image_file: {paths['image_file']}")
    print(f"- metadata_file: {paths['metadata_file']}")
    print()

    print("Start autonomous mapping:")
    print(
        _autonomous_mapping_launch_command(
            args.profile,
            args.driver_impl,
            args.with_rviz,
            args.with_pointcloud,
        )
    )
    print()

    if args.with_rviz:
        print("Open RViz:")
        print(_rviz_command())
        print()

    print("Safety rule:")
    print(_pause_note())
    print()

    print("Save map when mapping is finished:")
    print(_save_map_command(args.map_name))
    print()

    print("Write Robot Savo metadata:")
    print(_metadata_command(args.map_name))


def _print_dry_status(args: argparse.Namespace) -> None:
    readiness = build_autonomous_mapping_ready_state(
        use_pointcloud=bool(args.with_pointcloud),
    )

    status = make_autonomous_mapping_status(
        readiness=readiness,
        map_name=sanitize_map_name(args.map_name),
        session_id=None,
    )

    print()
    print("Simulated autonomous mapping status:")
    print(status.to_json(indent=2))


def _print_dry_goal() -> None:
    goal = ExplorationGoal(
        x=2.50,
        y=1.20,
        yaw=0.0,
        frame_id="map",
        score=0.82,
        reason="closest_frontier",
    )

    idle = make_idle_exploration_status()
    selected = make_goal_selected_status(goal, frontier_count=8)
    navigating = make_navigating_status(goal, frontier_count=8)

    print()
    print("Simulated exploration status:")
    print(idle.to_json(indent=2))
    print(selected.to_json(indent=2))
    print(navigating.to_json(indent=2))


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    _print_session_plan(args)

    if args.dry_status:
        _print_dry_status(args)

    if args.dry_goal:
        _print_dry_goal()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
