#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Operator notes CLI for Robot Savo mapping."""

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


from savo_mapping.models.map_metadata import sanitize_map_name  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Print operator notes for Robot Savo mapping sessions.",
    )

    parser.add_argument(
        "--mode",
        default="manual",
        choices=("manual", "autonomous", "rviz", "save", "network", "all"),
        help="Which operator notes to print.",
    )
    parser.add_argument(
        "--map-name",
        default="Savonia Campus Heart",
        help="Map name used in examples.",
    )
    parser.add_argument(
        "--operator-machine",
        default="macbook",
        choices=("macbook", "ubuntu_pc"),
        help="Operator machine used for visualization/control notes.",
    )
    parser.add_argument(
        "--robot-host",
        default="savo-core.local",
        help="Robot core hostname.",
    )

    return parser


# =============================================================================
# Note blocks
# =============================================================================
def _robot_runtime_notes(robot_host: str) -> str:
    return f"""Run on savo-core / robot Pi:

ssh savo@{robot_host}

cd ~/Savo_Pi/savo_ws
source install/setup.bash

# Robot runtime must stay headless.
# Do not run RViz on the Pi.
"""


def _manual_mapping_notes(map_name: str, robot_host: str) -> str:
    clean_name = sanitize_map_name(map_name)

    return f"""Manual mapping workflow:

1. Start robot-side mapping on savo-core:

ssh savo@{robot_host}
cd ~/Savo_Pi/savo_ws
source install/setup.bash

ros2 launch savo_mapping manual_mapping.launch.py \\
  profile:=manual_mapping_real_robot_v1.yaml \\
  use_rviz:=false

2. Drive slowly and smoothly while mapping.

Recommended movement:
- slow forward/backward
- slow rotation
- avoid fast spinning
- revisit corridors from both directions
- avoid pushing close to glass/walls
- stop if localization or scan becomes unstable

3. Save map after mapping:

ros2 launch savo_mapping mapping_save.launch.py \\
  map_name:={clean_name}

4. Write Robot Savo metadata:

ros2 run savo_mapping save_map_cli.py \\
  --map-name "{map_name}" \\
  --write-metadata
"""


def _autonomous_mapping_notes(map_name: str, robot_host: str) -> str:
    clean_name = sanitize_map_name(map_name)

    return f"""Autonomous mapping workflow:

1. Start robot-side autonomous mapping on savo-core:

ssh savo@{robot_host}
cd ~/Savo_Pi/savo_ws
source install/setup.bash

ros2 launch savo_mapping autonomous_mapping.launch.py \\
  profile:=autonomous_mapping_real_robot_v1.yaml \\
  use_rviz:=false \\
  use_pointcloud:=false

2. Watch safety and localization.

Autonomous mapping must pause if:
- /safety/stop is true
- /scan is stale
- /odometry/filtered is stale
- TF is broken
- Nav2 repeatedly fails goals
- robot is physically blocked

3. Save map:

ros2 launch savo_mapping mapping_save.launch.py \\
  map_name:={clean_name}

4. Write Robot Savo metadata:

ros2 run savo_mapping save_map_cli.py \\
  --map-name "{map_name}" \\
  --write-metadata
"""


def _rviz_notes(operator_machine: str) -> str:
    if operator_machine == "macbook":
        machine_label = "MacBook Air"
    else:
        machine_label = "Ubuntu PC"

    return f"""RViz visualization notes:

Run RViz on {machine_label}, not on the Raspberry Pi.

The Pi should run mapping headless:

ros2 launch savo_mapping manual_mapping.launch.py use_rviz:=false

On the visualization machine, after ROS network setup:

cd ~/Savo_Pi/savo_ws
source install/setup.bash

ros2 launch savo_mapping mapping_rviz.launch.py \\
  rviz_config:=manual_mapping

RViz should show:
- /scan
- /map
- TF tree
- robot footprint
- odometry pose
- frontier markers later
- pointcloud / voxel layer later
"""


def _save_notes(map_name: str) -> str:
    clean_name = sanitize_map_name(map_name)

    return f"""Map saving notes:

Map name:
- human name: {map_name}
- saved name: {clean_name}

Expected saved files:
- maps/saved/{clean_name}.yaml
- maps/saved/{clean_name}.pgm
- maps/saved/{clean_name}.metadata.json

Before loading a saved map, check it:

ros2 run savo_mapping load_map_check_cli.py \\
  --map-name "{map_name}"

Or from source tree:

python3 scripts/load_map_check_cli.py \\
  --map-name "{map_name}"
"""


def _network_notes(robot_host: str) -> str:
    return f"""ROS network notes:

Robot runtime:
- host: {robot_host}
- role: headless mapping runtime

Operator machine:
- MacBook Air for SSH/operator and later RViz demos
- Ubuntu PC for heavy validation and RViz debugging

Before testing, check:
- same ROS_DOMAIN_ID
- same network
- robot topics visible from operator machine
- firewall not blocking DDS discovery

Useful checks:

ssh savo@{robot_host}

ros2 topic list
ros2 topic hz /scan
ros2 topic echo /savo_mapping/status --once
ros2 run tf2_ros tf2_echo odom base_link
"""


# =============================================================================
# Printing
# =============================================================================
def _print_block(title: str, text: str) -> None:
    print("=" * 80)
    print(title)
    print("=" * 80)
    print(text.strip())
    print()


def print_notes(args: argparse.Namespace) -> None:
    mode = str(args.mode)
    map_name = str(args.map_name)
    robot_host = str(args.robot_host)
    operator_machine = str(args.operator_machine)

    if mode in ("manual", "all"):
        _print_block(
            "Robot Savo manual mapping notes",
            _robot_runtime_notes(robot_host)
            + "\n"
            + _manual_mapping_notes(map_name, robot_host),
        )

    if mode in ("autonomous", "all"):
        _print_block(
            "Robot Savo autonomous mapping notes",
            _robot_runtime_notes(robot_host)
            + "\n"
            + _autonomous_mapping_notes(map_name, robot_host),
        )

    if mode in ("rviz", "all"):
        _print_block(
            "Robot Savo RViz notes",
            _rviz_notes(operator_machine),
        )

    if mode in ("save", "all"):
        _print_block(
            "Robot Savo map save/load notes",
            _save_notes(map_name),
        )

    if mode in ("network", "all"):
        _print_block(
            "Robot Savo ROS network notes",
            _network_notes(robot_host),
        )


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    print_notes(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
