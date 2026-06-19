#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Manual mapping helper CLI for Robot Savo."""

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


from savo_mapping.models.map_metadata import make_saved_map_paths, sanitize_map_name  # noqa: E402
from savo_mapping.models.mapping_mode import MappingMode  # noqa: E402
from savo_mapping.models.mapping_status import make_manual_mapping_status  # noqa: E402
from savo_mapping.models.readiness_state import build_manual_mapping_ready_state  # noqa: E402
from savo_mapping.ros.params import mapping_supervisor_params_from_dict  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare a manual LiDAR mapping session for Robot Savo.",
    )

    parser.add_argument(
        "--map-name",
        default="Savonia Campus Heart",
        help="Map name to use when saving later.",
    )
    parser.add_argument(
        "--profile",
        default="manual_mapping_real_robot_v1.yaml",
        help="Mapping profile name.",
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
        "--with-teleop",
        action="store_true",
        help="Show teleop command in the output.",
    )
    parser.add_argument(
        "--dry-status",
        action="store_true",
        help="Print simulated manual mapping status.",
    )

    return parser


# =============================================================================
# Command builders
# =============================================================================
def _manual_mapping_launch_command(profile: str, driver_impl: str, with_rviz: bool) -> str:
    rviz_value = "true" if with_rviz else "false"

    return (
        "ros2 launch savo_mapping manual_mapping.launch.py "
        f"profile:={profile} "
        f"driver_impl:={driver_impl} "
        f"use_rviz:={rviz_value}"
    )


def _rviz_command() -> str:
    return "ros2 launch savo_mapping mapping_rviz.launch.py rviz_config:=manual_mapping"


def _teleop_command() -> str:
    return (
        "ros2 run savo_base teleop_letters_cli.py "
        "--cmd-topic /cmd_vel_safe "
        "--safety-stop-topic /safety/stop "
        "--slowdown-topic /safety/slowdown_factor "
        "--vx 0.12 --vy 0.12 --wz 0.20"
    )


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


# =============================================================================
# Output
# =============================================================================
def _print_session_plan(args: argparse.Namespace) -> None:
    clean_name = sanitize_map_name(args.map_name)
    paths = make_saved_map_paths(clean_name, maps_dir=args.maps_dir)

    params = mapping_supervisor_params_from_dict(
        {
            "require_scan": True,
            "require_odom": True,
            "require_tf": True,
            "require_map": False,
            "require_pointcloud": False,
        }
    )

    print("Robot Savo manual mapping session")
    print(f"- mode: {MappingMode.MANUAL_MAPPING.value}")
    print(f"- map_name: {clean_name}")
    print(f"- profile: {args.profile}")
    print(f"- runtime: {args.driver_impl}")
    print(f"- scan_topic: {params.scan_topic}")
    print(f"- odom_topic: {params.odom_topic}")
    print(f"- map_topic: {params.map_topic}")
    print(f"- yaml_file: {paths['yaml_file']}")
    print(f"- image_file: {paths['image_file']}")
    print(f"- metadata_file: {paths['metadata_file']}")
    print()

    print("Start manual mapping:")
    print(_manual_mapping_launch_command(args.profile, args.driver_impl, args.with_rviz))
    print()

    if args.with_rviz:
        print("Open RViz:")
        print(_rviz_command())
        print()

    if args.with_teleop:
        print("Manual drive:")
        print(_teleop_command())
        print()

    print("Save map when mapping is finished:")
    print(_save_map_command(args.map_name))
    print()

    print("Write Robot Savo metadata:")
    print(_metadata_command(args.map_name))


def _print_dry_status(args: argparse.Namespace) -> None:
    readiness = build_manual_mapping_ready_state()

    status = make_manual_mapping_status(
        readiness=readiness,
        map_name=sanitize_map_name(args.map_name),
        session_id=None,
    )

    print()
    print("Simulated manual mapping status:")
    print(status.to_json(indent=2))


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    _print_session_plan(args)

    if args.dry_status:
        _print_dry_status(args)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
