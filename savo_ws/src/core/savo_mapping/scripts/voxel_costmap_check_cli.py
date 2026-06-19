#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Voxel costmap readiness CLI for Robot Savo mapping."""

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


from savo_mapping.diagnostics.pointcloud_ready_check import evaluate_pointcloud_ready  # noqa: E402
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.diagnostics.tf_ready_check import evaluate_default_tf_ready  # noqa: E402
from savo_mapping.diagnostics.voxel_layer_check import (  # noqa: E402
    evaluate_voxel_layer_ready,
    voxel_layer_result_to_diagnostic,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check mapping-mode voxel costmap readiness.",
    )

    parser.add_argument(
        "--enabled",
        action="store_true",
        help="Evaluate voxel layer as enabled.",
    )
    parser.add_argument(
        "--costmap-active",
        action="store_true",
        help="Simulate/mark costmap as active.",
    )
    parser.add_argument(
        "--pointcloud-ok",
        action="store_true",
        help="Simulate/mark pointcloud as ready.",
    )
    parser.add_argument(
        "--tf-ok",
        action="store_true",
        help="Simulate/mark TF as ready.",
    )
    parser.add_argument(
        "--good",
        action="store_true",
        help="Use known-good simulated voxel inputs.",
    )
    parser.add_argument(
        "--bad",
        action="store_true",
        help="Use known-bad simulated voxel inputs.",
    )

    parser.add_argument("--msg-count", type=int, default=5)
    parser.add_argument("--age", type=float, default=0.1)
    parser.add_argument("--stale-timeout", type=float, default=2.0)

    parser.add_argument("--obstacle-min-range", type=float, default=0.15)
    parser.add_argument("--obstacle-max-range", type=float, default=2.5)
    parser.add_argument("--min-obstacle-height", type=float, default=0.05)
    parser.add_argument("--max-obstacle-height", type=float, default=1.5)

    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# Input builders
# =============================================================================
def _resolve_inputs(args: argparse.Namespace) -> dict:
    if args.good:
        return {
            "enabled": True,
            "costmap_active": True,
            "pointcloud_ok": True,
            "tf_ok": True,
            "msg_count": max(1, args.msg_count),
            "age_s": min(args.age, args.stale_timeout * 0.5),
        }

    if args.bad:
        return {
            "enabled": True,
            "costmap_active": False,
            "pointcloud_ok": False,
            "tf_ok": False,
            "msg_count": 0,
            "age_s": None,
        }

    return {
        "enabled": bool(args.enabled),
        "costmap_active": bool(args.costmap_active),
        "pointcloud_ok": bool(args.pointcloud_ok),
        "tf_ok": bool(args.tf_ok),
        "msg_count": max(0, int(args.msg_count)),
        "age_s": args.age,
    }


def _make_supporting_checks(inputs: dict) -> dict:
    pointcloud = evaluate_pointcloud_ready(
        enabled=bool(inputs["pointcloud_ok"]),
        msg_count=20 if inputs["pointcloud_ok"] else 0,
        rate_hz=8.0 if inputs["pointcloud_ok"] else 0.0,
        age_s=0.05 if inputs["pointcloud_ok"] else None,
        point_count=12000 if inputs["pointcloud_ok"] else 0,
    )

    tf = evaluate_default_tf_ready(
        edge_ages_s={
            "map->odom": 0.05 if inputs["tf_ok"] else None,
            "odom->base_link": 0.05 if inputs["tf_ok"] else None,
            "base_link->laser": 0.0 if inputs["tf_ok"] else None,
        },
        available_edges={
            "map->odom": bool(inputs["tf_ok"]),
            "odom->base_link": bool(inputs["tf_ok"]),
            "base_link->laser": bool(inputs["tf_ok"]),
        },
        frame_count=4 if inputs["tf_ok"] else 0,
    )

    return {
        "pointcloud": pointcloud,
        "tf": tf,
    }


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    inputs = _resolve_inputs(args)
    supporting = _make_supporting_checks(inputs)

    result = evaluate_voxel_layer_ready(
        enabled=inputs["enabled"],
        costmap_active=inputs["costmap_active"],
        pointcloud_ok=supporting["pointcloud"].ok,
        tf_ok=supporting["tf"].ok,
        clearing_enabled=True,
        marking_enabled=True,
        msg_count=inputs["msg_count"],
        age_s=inputs["age_s"],
        stale_timeout_s=args.stale_timeout,
        obstacle_min_range_m=args.obstacle_min_range,
        obstacle_max_range_m=args.obstacle_max_range,
        min_obstacle_height_m=args.min_obstacle_height,
        max_obstacle_height_m=args.max_obstacle_height,
        extra={
            "pointcloud_message": supporting["pointcloud"].message,
            "tf_message": supporting["tf"].message,
        },
    )

    diagnostic = voxel_layer_result_to_diagnostic(result, required=False)

    if args.json:
        print(result.to_json(indent=2))
    else:
        print(
            format_key_value_block(
                "Robot Savo voxel costmap check",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "enabled": result.enabled,
                    "costmap_active": result.costmap_active,
                    "pointcloud_ok": result.pointcloud_ok,
                    "tf_ok": result.tf_ok,
                    "msg_count": result.msg_count,
                    "age_s": result.age_s,
                    "stale": result.stale,
                    "pointcloud_topic": result.pointcloud_topic,
                    "obstacle_min_range_m": result.obstacle_min_range_m,
                    "obstacle_max_range_m": result.obstacle_max_range_m,
                    "min_obstacle_height_m": result.min_obstacle_height_m,
                    "max_obstacle_height_m": result.max_obstacle_height_m,
                },
            )
        )

    return 0 if result.ok or not result.enabled else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))