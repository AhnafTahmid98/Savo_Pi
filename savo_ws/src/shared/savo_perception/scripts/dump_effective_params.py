#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dump effective default parameters for savo_perception nodes."""

from __future__ import annotations

import argparse
import json
from typing import Any, Dict

from savo_perception.ros.params import (
    load_cmd_vel_safety_gate_params,
    load_range_health_params,
    load_safety_stop_params,
    load_ultrasonic_params,
    load_vl53_mux_params,
)
from savo_perception.utils.param_loader import (
    load_ros_params_file,
    overlay_params,
)


def default_param_groups() -> Dict[str, Dict[str, Any]]:
    return {
        "vl53_mux_node": load_vl53_mux_params({}).to_dict(),
        "ultrasonic_node": load_ultrasonic_params({}).to_dict(),
        "safety_stop_node": load_safety_stop_params({}).to_dict(),
        "cmd_vel_safety_gate": load_cmd_vel_safety_gate_params({}).to_dict(),
        "range_health_node": load_range_health_params({}).to_dict(),
    }


def load_file_overlay(path: str, node_name: str) -> Dict[str, Any]:
    if not path:
        return {}

    result = load_ros_params_file(path, node_name=node_name)
    return result.params


def print_yaml_like(data: Dict[str, Any]) -> None:
    for group, params in data.items():
        print(f"{group}:")
        for key, value in params.items():
            print(f"  {key}: {value}")
        print()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dump effective savo_perception parameters"
    )

    parser.add_argument(
        "--node",
        choices=[
            "all",
            "vl53_mux_node",
            "ultrasonic_node",
            "safety_stop_node",
            "cmd_vel_safety_gate",
            "range_health_node",
        ],
        default="all",
    )
    parser.add_argument(
        "--params-file",
        default="",
        help="Optional ROS-style YAML/JSON parameter file to overlay.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print as JSON instead of YAML-like text.",
    )

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    groups = default_param_groups()

    if args.node == "all":
        selected = groups
    else:
        selected = {args.node: groups[args.node]}

    effective: Dict[str, Dict[str, Any]] = {}

    for node_name, defaults in selected.items():
        overlay = load_file_overlay(args.params_file, node_name) if args.params_file else {}
        effective[node_name] = overlay_params(defaults, overlay)

    if args.json:
        print(json.dumps(effective, indent=2, sort_keys=True))
    else:
        print_yaml_like(effective)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())