#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Mapping readiness CLI for Robot Savo."""

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

from savo_mapping.diagnostics import (
    evaluate_apriltag_ready,
    evaluate_default_tf_ready,
    evaluate_location_bridge_ready,
    evaluate_map_topic_ready,
    evaluate_nav2_mapping_ready,
    evaluate_odom_ready,
    evaluate_pointcloud_ready,
    evaluate_scan_ready,
    evaluate_slam_toolbox_ready,
    evaluate_voxel_layer_ready,
)
from savo_mapping.diagnostics import (
    apriltag_result_to_diagnostic,
    location_bridge_result_to_diagnostic,
    map_topic_result_to_diagnostic,
    nav2_mapping_result_to_diagnostic,
    odom_result_to_diagnostic,
    pointcloud_result_to_diagnostic,
    scan_result_to_diagnostic,
    slam_toolbox_result_to_diagnostic,
    tf_result_to_diagnostic,
    voxel_layer_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import format_report
from savo_mapping.models.mapping_mode import MappingMode, require_valid_mapping_mode
from savo_mapping.models.mapping_status import make_mapping_status
from savo_mapping.models.readiness_state import build_readiness_state, make_check
from savo_mapping.utils.diagnostics import DiagnosticItem, build_diagnostic_report


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check Robot Savo mapping readiness from diagnostic inputs.",
    )

    parser.add_argument(
        "--mode",
        default=MappingMode.MANUAL_MAPPING.value,
        choices=MappingMode.values(),
        help="Mapping mode to evaluate.",
    )
    parser.add_argument(
        "--style",
        default="compact",
        choices=("compact", "table", "json"),
        help="Output format.",
    )

    parser.add_argument("--good", action="store_true", help="Use known-good simulated inputs.")
    parser.add_argument("--bad", action="store_true", help="Use known-bad simulated inputs.")

    parser.add_argument("--enable-map", action="store_true", help="Evaluate /map as required.")
    parser.add_argument("--enable-slam", action="store_true", help="Evaluate slam_toolbox.")
    parser.add_argument("--enable-nav2", action="store_true", help="Evaluate Nav2 mapping stack.")
    parser.add_argument("--enable-pointcloud", action="store_true", help="Evaluate RealSense pointcloud.")
    parser.add_argument("--enable-voxel", action="store_true", help="Evaluate voxel layer.")
    parser.add_argument("--enable-apriltag", action="store_true", help="Evaluate AprilTag detection.")
    parser.add_argument("--enable-location-bridge", action="store_true", help="Evaluate savo_location bridge.")

    parser.add_argument("--scan-rate", type=float, default=3.8)
    parser.add_argument("--odom-rate", type=float, default=30.0)
    parser.add_argument("--map-rate", type=float, default=1.0)
    parser.add_argument("--pointcloud-rate", type=float, default=8.0)

    return parser


# =============================================================================
# Diagnostic builders
# =============================================================================
def _good_inputs(args: argparse.Namespace) -> list[DiagnosticItem]:
    scan = evaluate_scan_ready(
        msg_count=20,
        rate_hz=args.scan_rate,
        age_s=0.05,
        range_count=720,
        finite_count=600,
        min_observed_m=0.42,
        max_observed_m=5.8,
    )

    odom = evaluate_odom_ready(
        msg_count=50,
        rate_hz=args.odom_rate,
        age_s=0.02,
        frame_id="odom",
        child_frame_id="base_link",
        x=1.2,
        y=0.8,
        yaw_known=True,
    )

    tf = evaluate_default_tf_ready(
        edge_ages_s={
            "map->odom": 0.05,
            "odom->base_link": 0.05,
            "base_link->laser": 0.0,
        },
        available_edges={
            "map->odom": True,
            "odom->base_link": True,
            "base_link->laser": True,
        },
        frame_count=4,
    )

    items = [
        scan_result_to_diagnostic(scan, required=True),
        odom_result_to_diagnostic(odom, required=True),
        tf_result_to_diagnostic(tf, required=True),
    ]

    if args.enable_map:
        map_result = evaluate_map_topic_ready(
            msg_count=3,
            rate_hz=args.map_rate,
            age_s=0.2,
            width_cells=100,
            height_cells=80,
            resolution_m=0.05,
            free_cells=5000,
            occupied_cells=500,
            unknown_cells=2500,
            required=True,
        )
        items.append(map_topic_result_to_diagnostic(map_result, required=True))

    if args.enable_slam:
        slam = evaluate_slam_toolbox_ready(
            active=True,
            msg_count=10,
            map_msg_count=3,
            rate_hz=1.0,
            age_s=0.2,
            has_map=True,
            has_scan=True,
            has_odom=True,
            has_tf=True,
            require_map=args.enable_map,
        )
        items.append(slam_toolbox_result_to_diagnostic(slam, required=True))

    if args.enable_nav2:
        nav2 = evaluate_nav2_mapping_ready(
            enabled=True,
            lifecycle_active=True,
            planner_active=True,
            controller_active=True,
            bt_navigator_active=True,
            costmaps_active=True,
            action_available=True,
            msg_count=5,
            age_s=0.1,
        )
        items.append(nav2_mapping_result_to_diagnostic(nav2, required=True))

    if args.enable_pointcloud:
        pointcloud = evaluate_pointcloud_ready(
            enabled=True,
            msg_count=20,
            rate_hz=args.pointcloud_rate,
            age_s=0.05,
            point_count=12000,
            width=160,
            height=75,
        )
        items.append(pointcloud_result_to_diagnostic(pointcloud, required=False))

    if args.enable_voxel:
        voxel = evaluate_voxel_layer_ready(
            enabled=True,
            costmap_active=True,
            pointcloud_ok=True,
            tf_ok=True,
            msg_count=5,
            age_s=0.1,
        )
        items.append(voxel_layer_result_to_diagnostic(voxel, required=False))

    if args.enable_apriltag:
        apriltag = evaluate_apriltag_ready(
            enabled=True,
            msg_count=3,
            detection_count=1,
            unique_tag_count=1,
            last_tag_id=21,
            last_label="A201",
            age_s=0.03,
            require_known_label=True,
        )
        items.append(apriltag_result_to_diagnostic(apriltag, required=False))

    if args.enable_location_bridge:
        bridge = evaluate_location_bridge_ready(
            enabled=True,
            package_available=True,
            topic_available=True,
            msg_count=3,
            age_s=0.1,
        )
        items.append(location_bridge_result_to_diagnostic(bridge, required=False))

    return items


def _bad_inputs(args: argparse.Namespace) -> list[DiagnosticItem]:
    scan = evaluate_scan_ready(
        msg_count=0,
        rate_hz=0.0,
        age_s=None,
        range_count=0,
        finite_count=0,
    )

    odom = evaluate_odom_ready(
        msg_count=0,
        rate_hz=0.0,
        age_s=None,
        frame_id="",
        child_frame_id="",
    )

    tf = evaluate_default_tf_ready(
        edge_ages_s={
            "map->odom": None,
            "odom->base_link": None,
            "base_link->laser": None,
        },
        available_edges={
            "map->odom": False,
            "odom->base_link": False,
            "base_link->laser": False,
        },
        frame_count=0,
    )

    items = [
        scan_result_to_diagnostic(scan, required=True),
        odom_result_to_diagnostic(odom, required=True),
        tf_result_to_diagnostic(tf, required=True),
    ]

    if args.enable_map:
        map_result = evaluate_map_topic_ready(
            msg_count=0,
            rate_hz=0.0,
            age_s=None,
            width_cells=0,
            height_cells=0,
            resolution_m=0.0,
            free_cells=0,
            occupied_cells=0,
            unknown_cells=0,
            required=True,
        )
        items.append(map_topic_result_to_diagnostic(map_result, required=True))

    if args.enable_slam:
        slam = evaluate_slam_toolbox_ready(
            active=False,
            msg_count=0,
            map_msg_count=0,
            age_s=None,
            has_map=False,
            has_scan=False,
            has_odom=False,
            has_tf=False,
            require_map=args.enable_map,
        )
        items.append(slam_toolbox_result_to_diagnostic(slam, required=True))

    if args.enable_nav2:
        nav2 = evaluate_nav2_mapping_ready(enabled=True, age_s=None)
        items.append(nav2_mapping_result_to_diagnostic(nav2, required=True))

    if args.enable_pointcloud:
        pointcloud = evaluate_pointcloud_ready(
            enabled=True,
            msg_count=0,
            rate_hz=0.0,
            age_s=None,
            point_count=0,
        )
        items.append(pointcloud_result_to_diagnostic(pointcloud, required=False))

    if args.enable_voxel:
        voxel = evaluate_voxel_layer_ready(
            enabled=True,
            costmap_active=False,
            pointcloud_ok=False,
            tf_ok=False,
            msg_count=0,
            age_s=None,
        )
        items.append(voxel_layer_result_to_diagnostic(voxel, required=False))

    if args.enable_apriltag:
        apriltag = evaluate_apriltag_ready(
            enabled=True,
            msg_count=0,
            detection_count=0,
            unique_tag_count=0,
            age_s=None,
            require_known_label=True,
        )
        items.append(apriltag_result_to_diagnostic(apriltag, required=False))

    if args.enable_location_bridge:
        bridge = evaluate_location_bridge_ready(
            enabled=True,
            package_available=False,
            topic_available=False,
            service_available=False,
            msg_count=0,
            age_s=None,
        )
        items.append(location_bridge_result_to_diagnostic(bridge, required=False))

    return items


def _readiness_from_items(items: Sequence[DiagnosticItem]):
    checks = []

    for item in items:
        checks.append(
            make_check(
                name=item.name,
                ok=item.ok,
                required=item.required,
                enabled=item.enabled,
                message=item.message,
            )
        )

    return build_readiness_state(checks)


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    mode = require_valid_mapping_mode(args.mode)

    if args.bad:
        items = _bad_inputs(args)
    else:
        items = _good_inputs(args)

    report = build_diagnostic_report(
        name=f"savo_mapping_readiness:{mode}",
        items=items,
    )

    readiness = _readiness_from_items(items)

    status = make_mapping_status(
        mode=mode,
        readiness=readiness,
        active=mode != MappingMode.IDLE.value,
        message="Mapping readiness OK." if readiness.ready else "Mapping readiness failed.",
    )

    print(format_report(report, args.style))

    if args.style != "json":
        print()
        print(f"ready={str(status.ready).lower()} degraded={str(status.degraded).lower()}")
        print(f"mode={status.mode}")
        print(f"message={status.message}")

    return 0 if status.ready else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
