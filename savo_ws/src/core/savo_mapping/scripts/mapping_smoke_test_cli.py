#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Source-tree smoke test CLI for Robot Savo mapping."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Callable, Sequence


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
    evaluate_map_quality,
    evaluate_nav2_mapping_ready,
    evaluate_odom_ready,
    evaluate_pointcloud_ready,
    evaluate_scan_ready,
    evaluate_voxel_layer_ready,
)
from savo_mapping.diagnostics.report_formatter import format_report
from savo_mapping.models.apriltag_status import AprilTagObservation
from savo_mapping.models.map_metadata import make_map_metadata
from savo_mapping.models.mapping_mode import MappingMode
from savo_mapping.models.mapping_status import make_idle_status
from savo_mapping.models.pointcloud_status import make_pointcloud_status
from savo_mapping.models.semantic_landmark import make_semantic_landmark
from savo_mapping.ros.topic_contract import (
    get_required_topic_names,
    validate_contract_topics,
)
from savo_mapping.utils.diagnostics import (
    DiagnosticItem,
    build_diagnostic_report,
    make_error,
    make_ok,
)
from savo_mapping.utils.qos import qos_name_for_topic
from savo_mapping.utils.topic_names import get_topic, join_topic, normalize_topic


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a local source-tree smoke test for savo_mapping.",
    )

    parser.add_argument(
        "--style",
        default="compact",
        choices=("compact", "table", "json"),
        help="Output format.",
    )
    parser.add_argument(
        "--include-future",
        action="store_true",
        help="Also test future optional modules: pointcloud, voxel, AprilTag, location bridge.",
    )

    return parser


# =============================================================================
# Test helpers
# =============================================================================
def _run_check(name: str, fn: Callable[[], None]) -> DiagnosticItem:
    try:
        fn()
        return make_ok(name, f"{name} passed.", required=True)
    except Exception as exc:
        return make_error(
            name,
            f"{name} failed: {exc}",
            required=True,
            values={
                "exception_type": type(exc).__name__,
            },
        )


# =============================================================================
# Smoke checks
# =============================================================================
def _check_package_imports() -> None:
    import savo_mapping

    info = savo_mapping.get_package_info()

    if info["package"] != "savo_mapping":
        raise RuntimeError("Unexpected package name.")

    if not info["version"]:
        raise RuntimeError("Missing package version.")


def _check_topic_helpers() -> None:
    if normalize_topic("scan") != "/scan":
        raise RuntimeError("normalize_topic failed.")

    if join_topic("savo_mapping", "ready") != "/savo_mapping/ready":
        raise RuntimeError("join_topic failed.")

    if get_topic("map_quality") != "/savo_mapping/map_quality":
        raise RuntimeError("get_topic failed.")

    if qos_name_for_topic("/scan") != "sensor_data":
        raise RuntimeError("QoS topic mapping failed.")


def _check_topic_contract() -> None:
    validate_contract_topics()

    required = get_required_topic_names()

    for key in ("scan", "odom", "tf", "tf_static"):
        if key not in required:
            raise RuntimeError(f"Missing required topic key: {key}")


def _check_models() -> None:
    status = make_idle_status()

    if status.mode != MappingMode.IDLE.value:
        raise RuntimeError("Idle status mode mismatch.")

    metadata = make_map_metadata(
        name="Savonia Campus Heart",
        width_cells=400,
        height_cells=250,
        resolution_m=0.05,
    )

    if not metadata.valid:
        raise RuntimeError("Map metadata should be valid.")

    pointcloud = make_pointcloud_status(
        enabled=True,
        msg_count=2,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
    )

    if not pointcloud.ok:
        raise RuntimeError("Pointcloud status should be OK.")

    landmark = make_semantic_landmark(
        label="Info Desk",
        x=2.0,
        y=3.5,
    )

    if landmark.key != "info_desk":
        raise RuntimeError("Semantic landmark key mismatch.")


def _check_core_diagnostics() -> None:
    scan = evaluate_scan_ready(
        msg_count=20,
        rate_hz=3.8,
        age_s=0.05,
        range_count=720,
        finite_count=600,
    )

    odom = evaluate_odom_ready(
        msg_count=50,
        rate_hz=30.0,
        age_s=0.02,
        frame_id="odom",
        child_frame_id="base_link",
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

    quality = evaluate_map_quality(
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        free_cells=5000,
        occupied_cells=500,
        unknown_cells=2500,
    )

    if not scan.ok:
        raise RuntimeError(scan.message)

    if not odom.ok:
        raise RuntimeError(odom.message)

    if not tf.ok:
        raise RuntimeError(tf.message)

    if not quality.ok:
        raise RuntimeError(quality.message)


def _check_future_diagnostics() -> None:
    pointcloud = evaluate_pointcloud_ready(
        enabled=True,
        msg_count=20,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
        width=160,
        height=75,
    )

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

    voxel = evaluate_voxel_layer_ready(
        enabled=True,
        costmap_active=True,
        pointcloud_ok=True,
        tf_ok=True,
        msg_count=5,
        age_s=0.1,
    )

    apriltag = evaluate_apriltag_ready(
        enabled=True,
        msg_count=3,
        detection_count=1,
        unique_tag_count=1,
        last_tag_id=21,
        last_label="A201",
        age_s=0.03,
        observations=[
            AprilTagObservation(
                tag_id=21,
                label="A201",
                confidence=0.92,
            )
        ],
        require_known_label=True,
    )

    bridge = evaluate_location_bridge_ready(
        enabled=True,
        package_available=True,
        topic_available=True,
        msg_count=3,
        age_s=0.1,
    )

    checks = {
        "pointcloud": pointcloud,
        "nav2_mapping": nav2,
        "voxel_layer": voxel,
        "apriltag": apriltag,
        "location_bridge": bridge,
    }

    failed = [
        name
        for name, result in checks.items()
        if not result.ok
    ]

    if failed:
        raise RuntimeError(f"Future diagnostic checks failed: {failed}")


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    items = [
        _run_check("package_imports", _check_package_imports),
        _run_check("topic_helpers", _check_topic_helpers),
        _run_check("topic_contract", _check_topic_contract),
        _run_check("models", _check_models),
        _run_check("core_diagnostics", _check_core_diagnostics),
    ]

    if args.include_future:
        items.append(
            _run_check("future_diagnostics", _check_future_diagnostics)
        )

    report = build_diagnostic_report(
        name="savo_mapping_smoke_test",
        items=items,
    )

    print(format_report(report, args.style))

    if args.style != "json":
        print()
        print(f"smoke_test_ok={str(report.ok).lower()}")
        print(f"degraded={str(report.degraded).lower()}")

    return 0 if report.ok else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))