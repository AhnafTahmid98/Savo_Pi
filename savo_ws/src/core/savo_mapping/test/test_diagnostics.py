#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping diagnostics."""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

from savo_mapping.diagnostics import (
    evaluate_apriltag_ready,
    evaluate_default_tf_ready,
    evaluate_location_bridge_ready,
    evaluate_map_files_from_paths,
    evaluate_map_quality,
    evaluate_map_topic_ready,
    evaluate_nav2_mapping_from_nodes,
    evaluate_nav2_mapping_ready,
    evaluate_odom_ready,
    evaluate_pointcloud_ready,
    evaluate_scan_ready,
    evaluate_slam_toolbox_from_readiness,
    evaluate_slam_toolbox_ready,
    evaluate_voxel_layer_from_inputs,
    evaluate_voxel_layer_ready,
)
from savo_mapping.diagnostics import (
    apriltag_result_to_diagnostic,
    location_bridge_result_to_diagnostic,
    map_file_result_to_diagnostic,
    map_quality_result_to_diagnostic,
    map_topic_result_to_diagnostic,
    nav2_mapping_result_to_diagnostic,
    odom_result_to_diagnostic,
    pointcloud_result_to_diagnostic,
    scan_result_to_diagnostic,
    slam_toolbox_result_to_diagnostic,
    tf_result_to_diagnostic,
    voxel_layer_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import (
    format_key_value_block,
    format_mapping_status_dict,
    format_report,
    format_report_compact,
    format_report_json,
    format_report_table,
    parse_json_report,
)
from savo_mapping.models.apriltag_status import AprilTagObservation
from savo_mapping.utils.diagnostics import (
    build_diagnostic_report,
    make_disabled,
    make_ok,
    make_stale,
)


# =============================================================================
# Scan readiness
# =============================================================================
def test_scan_ready_good_result() -> None:
    result = evaluate_scan_ready(
        msg_count=20,
        rate_hz=10.0,
        age_s=0.05,
        range_count=720,
        finite_count=600,
        min_observed_m=0.42,
        max_observed_m=5.8,
    )

    diagnostic = scan_result_to_diagnostic(result)

    assert result.ok is True
    assert result.stale is False
    assert result.message == "LiDAR scan ready."
    assert diagnostic.level == "ok"


def test_scan_ready_bad_result_is_stale() -> None:
    result = evaluate_scan_ready(
        msg_count=0,
        rate_hz=0.0,
        age_s=None,
        range_count=0,
        finite_count=0,
    )

    diagnostic = scan_result_to_diagnostic(result)

    assert result.ok is False
    assert result.stale is True
    assert "no_messages" in result.extra["failures"]
    assert diagnostic.level == "stale"


# =============================================================================
# Odometry readiness
# =============================================================================
def test_odom_ready_good_result() -> None:
    result = evaluate_odom_ready(
        msg_count=50,
        rate_hz=30.0,
        age_s=0.02,
        frame_id="odom",
        child_frame_id="base_link",
        x=1.2,
        y=0.8,
        yaw_known=True,
        vx=0.0,
        vy=0.0,
        wz=0.0,
    )

    diagnostic = odom_result_to_diagnostic(result)

    assert result.ok is True
    assert result.stale is False
    assert result.message == "Filtered odometry ready."
    assert diagnostic.level == "ok"


def test_odom_ready_bad_result_is_stale() -> None:
    result = evaluate_odom_ready(
        msg_count=0,
        rate_hz=0.0,
        age_s=None,
        frame_id="",
        child_frame_id="",
    )

    diagnostic = odom_result_to_diagnostic(result)

    assert result.ok is False
    assert result.stale is True
    assert "missing_frame_id" in result.extra["failures"]
    assert "missing_child_frame_id" in result.extra["failures"]
    assert diagnostic.level == "stale"


# =============================================================================
# TF readiness
# =============================================================================
def test_tf_ready_good_result() -> None:
    result = evaluate_default_tf_ready(
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

    diagnostic = tf_result_to_diagnostic(result)

    assert result.ok is True
    assert result.stale is False
    assert result.message == "TF tree ready."
    assert diagnostic.level == "ok"


def test_tf_ready_bad_result() -> None:
    result = evaluate_default_tf_ready(
        edge_ages_s={
            "map->odom": None,
            "odom->base_link": 0.05,
            "base_link->laser": 0.0,
        },
        available_edges={
            "map->odom": False,
            "odom->base_link": True,
            "base_link->laser": True,
        },
        frame_count=3,
    )

    diagnostic = tf_result_to_diagnostic(result)

    assert result.ok is False
    assert result.stale is True
    assert "map->odom" in result.message
    assert diagnostic.level == "stale"


# =============================================================================
# Map topic
# =============================================================================
def test_map_topic_good_result() -> None:
    result = evaluate_map_topic_ready(
        msg_count=3,
        rate_hz=1.0,
        age_s=0.2,
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        free_cells=5000,
        occupied_cells=500,
        unknown_cells=2500,
        required=True,
    )

    diagnostic = map_topic_result_to_diagnostic(result, required=True)

    assert result.ok is True
    assert result.quality.ok is True
    assert result.message == "Map topic ready."
    assert diagnostic.level == "ok"


def test_map_topic_waiting_result_is_warn_when_optional() -> None:
    result = evaluate_map_topic_ready(
        msg_count=0,
        rate_hz=0.0,
        age_s=None,
        width_cells=0,
        height_cells=0,
        resolution_m=0.0,
        free_cells=0,
        occupied_cells=0,
        unknown_cells=0,
        required=False,
    )

    diagnostic = map_topic_result_to_diagnostic(result, required=False)

    assert result.ok is False
    assert result.message == "Map topic not publishing yet."
    assert diagnostic.level == "warn"


# =============================================================================
# slam_toolbox
# =============================================================================
def test_slam_toolbox_good_result() -> None:
    result = evaluate_slam_toolbox_ready(
        active=True,
        msg_count=10,
        map_msg_count=3,
        rate_hz=1.0,
        age_s=0.2,
        has_map=True,
        has_scan=True,
        has_odom=True,
        has_tf=True,
        require_map=True,
    )

    diagnostic = slam_toolbox_result_to_diagnostic(result, required=True)

    assert result.ok is True
    assert result.active is True
    assert result.message == "slam_toolbox ready."
    assert diagnostic.level == "ok"


def test_slam_toolbox_bad_result_is_error_when_not_active() -> None:
    result = evaluate_slam_toolbox_ready(
        active=False,
        msg_count=0,
        map_msg_count=0,
        age_s=None,
        has_map=False,
        has_scan=False,
        has_odom=False,
        has_tf=False,
    )

    diagnostic = slam_toolbox_result_to_diagnostic(result, required=True)

    assert result.ok is False
    assert result.active is False
    assert "not_active" in result.extra["failures"]
    assert diagnostic.level == "error"


def test_slam_toolbox_from_readiness() -> None:
    result = evaluate_slam_toolbox_from_readiness(
        scan_ok=True,
        odom_ok=True,
        tf_ok=True,
        map_ok=True,
        active=True,
        msg_count=5,
        map_msg_count=2,
        age_s=0.1,
        require_map=True,
    )

    assert result.ok is True


# =============================================================================
# Pointcloud
# =============================================================================
def test_pointcloud_disabled_result() -> None:
    result = evaluate_pointcloud_ready(enabled=False)
    diagnostic = pointcloud_result_to_diagnostic(result)

    assert result.enabled is False
    assert result.ok is False
    assert diagnostic.level == "disabled"


def test_pointcloud_good_result() -> None:
    result = evaluate_pointcloud_ready(
        enabled=True,
        msg_count=20,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
        width=160,
        height=75,
    )

    diagnostic = pointcloud_result_to_diagnostic(result)

    assert result.ok is True
    assert result.enabled is True
    assert result.point_count == 12000
    assert result.status.ok is True
    assert diagnostic.level == "ok"


def test_pointcloud_stale_result() -> None:
    result = evaluate_pointcloud_ready(
        enabled=True,
        msg_count=10,
        rate_hz=8.0,
        age_s=2.0,
        point_count=12000,
        stale_timeout_s=1.0,
    )

    diagnostic = pointcloud_result_to_diagnostic(result)

    assert result.ok is False
    assert result.stale is True
    assert diagnostic.level == "stale"


# =============================================================================
# Nav2 mapping
# =============================================================================
def test_nav2_mapping_disabled_result() -> None:
    result = evaluate_nav2_mapping_ready(enabled=False)
    diagnostic = nav2_mapping_result_to_diagnostic(result)

    assert result.enabled is False
    assert result.ok is False
    assert diagnostic.level == "disabled"


def test_nav2_mapping_good_result() -> None:
    result = evaluate_nav2_mapping_ready(
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

    diagnostic = nav2_mapping_result_to_diagnostic(result, required=True)

    assert result.ok is True
    assert result.message == "Nav2 mapping stack ready."
    assert diagnostic.level == "ok"


def test_nav2_mapping_from_nodes() -> None:
    result = evaluate_nav2_mapping_from_nodes(
        enabled=True,
        active_nodes={
            "lifecycle_manager_navigation",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "global_costmap",
            "local_costmap",
        },
        action_available=True,
        msg_count=5,
        age_s=0.1,
    )

    assert result.ok is True


# =============================================================================
# Voxel layer
# =============================================================================
def test_voxel_layer_disabled_result() -> None:
    result = evaluate_voxel_layer_ready(enabled=False)
    diagnostic = voxel_layer_result_to_diagnostic(result)

    assert result.enabled is False
    assert result.ok is False
    assert diagnostic.level == "disabled"


def test_voxel_layer_good_result() -> None:
    result = evaluate_voxel_layer_ready(
        enabled=True,
        costmap_active=True,
        pointcloud_ok=True,
        tf_ok=True,
        clearing_enabled=True,
        marking_enabled=True,
        msg_count=5,
        age_s=0.1,
    )

    diagnostic = voxel_layer_result_to_diagnostic(result)

    assert result.ok is True
    assert result.message == "Voxel layer ready."
    assert diagnostic.level == "ok"


def test_voxel_layer_from_inputs() -> None:
    result = evaluate_voxel_layer_from_inputs(
        enabled=True,
        pointcloud_result_ok=True,
        tf_result_ok=True,
        costmap_active=True,
        msg_count=5,
        age_s=0.1,
    )

    assert result.ok is True


# =============================================================================
# AprilTag
# =============================================================================
def test_apriltag_disabled_result() -> None:
    result = evaluate_apriltag_ready(enabled=False)
    diagnostic = apriltag_result_to_diagnostic(result)

    assert result.enabled is False
    assert result.ok is False
    assert diagnostic.level == "disabled"


def test_apriltag_good_result() -> None:
    observation = AprilTagObservation(
        tag_id=21,
        label="A201",
        confidence=0.92,
    )

    result = evaluate_apriltag_ready(
        enabled=True,
        msg_count=3,
        detection_count=1,
        unique_tag_count=1,
        last_tag_id=21,
        last_label="A201",
        age_s=0.03,
        observations=[observation],
        require_known_label=True,
    )

    diagnostic = apriltag_result_to_diagnostic(result)

    assert result.ok is True
    assert result.last_tag_id == 21
    assert result.last_label == "A201"
    assert diagnostic.level == "ok"


def test_apriltag_missing_label_fails_when_required() -> None:
    result = evaluate_apriltag_ready(
        enabled=True,
        msg_count=3,
        detection_count=1,
        unique_tag_count=1,
        last_tag_id=21,
        last_label=None,
        age_s=0.03,
        require_known_label=True,
    )

    diagnostic = apriltag_result_to_diagnostic(result)

    assert result.ok is False
    assert "missing_known_label" in result.extra["failures"]
    assert diagnostic.level == "warn"


# =============================================================================
# Location bridge
# =============================================================================
def test_location_bridge_disabled_result() -> None:
    result = evaluate_location_bridge_ready(enabled=False)
    diagnostic = location_bridge_result_to_diagnostic(result)

    assert result.enabled is False
    assert result.ok is False
    assert diagnostic.level == "disabled"


def test_location_bridge_good_result() -> None:
    result = evaluate_location_bridge_ready(
        enabled=True,
        package_available=True,
        topic_available=True,
        service_available=False,
        saved_count=2,
        confirmed_count=1,
        failed_count=0,
        msg_count=5,
        age_s=0.1,
    )

    diagnostic = location_bridge_result_to_diagnostic(result)

    assert result.ok is True
    assert result.status.ok is True
    assert result.saved_count == 2
    assert result.confirmed_count == 1
    assert diagnostic.level == "ok"


def test_location_bridge_bad_result() -> None:
    result = evaluate_location_bridge_ready(
        enabled=True,
        package_available=False,
        topic_available=False,
        service_available=False,
        msg_count=0,
        age_s=None,
    )

    diagnostic = location_bridge_result_to_diagnostic(result)

    assert result.ok is False
    assert result.stale is True
    assert "package_not_available" in result.extra["failures"]
    assert diagnostic.level == "stale"


# =============================================================================
# Map file checks
# =============================================================================
def test_map_file_check_with_valid_temp_map() -> None:
    with TemporaryDirectory() as tmp:
        base = Path(tmp)
        yaml_file = base / "test_map.yaml"
        image_file = base / "test_map.pgm"

        yaml_file.write_text(
            "image: test_map.pgm\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "mode: trinary\n",
            encoding="utf-8",
        )
        image_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")

        result = evaluate_map_files_from_paths(yaml_file)
        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert result.ok is True
        assert result.yaml_exists is True
        assert result.image_exists is True
        assert result.resolution_m == 0.05
        assert diagnostic.level == "ok"


def test_map_file_check_missing_map_fails() -> None:
    with TemporaryDirectory() as tmp:
        yaml_file = Path(tmp) / "missing_map.yaml"

        result = evaluate_map_files_from_paths(yaml_file)
        diagnostic = map_file_result_to_diagnostic(result, required=True)

        assert result.ok is False
        assert result.yaml_exists is False
        assert "yaml_missing" in result.extra["failures"]
        assert diagnostic.level == "error"


# =============================================================================
# Map quality diagnostic
# =============================================================================
def test_map_quality_good_result() -> None:
    result = evaluate_map_quality(
        width_cells=100,
        height_cells=80,
        resolution_m=0.05,
        free_cells=5000,
        occupied_cells=500,
        unknown_cells=2500,
    )

    diagnostic = map_quality_result_to_diagnostic(result)

    assert result.ok is True
    assert result.quality.ok is True
    assert diagnostic.level == "ok"


def test_map_quality_bad_required_result() -> None:
    result = evaluate_map_quality(
        width_cells=10,
        height_cells=10,
        resolution_m=0.05,
        free_cells=20,
        occupied_cells=2,
        unknown_cells=78,
        required=True,
    )

    diagnostic = map_quality_result_to_diagnostic(result)

    assert result.ok is False
    assert result.required is True
    assert diagnostic.level == "error"


# =============================================================================
# Diagnostic report formatting
# =============================================================================
def test_diagnostic_report_ok_and_degraded() -> None:
    report = build_diagnostic_report(
        "mapping_test",
        (
            make_ok("scan", required=True),
            make_ok("odom", required=True),
            make_stale("pointcloud", "stale", required=False),
            make_disabled("apriltag"),
        ),
    )

    assert report.ok is True
    assert report.degraded is True
    assert report.failed_required == ()
    assert report.warnings == ()


def test_report_formatters() -> None:
    report = build_diagnostic_report(
        "mapping_test",
        (
            make_ok("scan", "scan ok", required=True),
            make_ok("odom", "odom ok", required=True),
        ),
    )

    compact = format_report_compact(report)
    table = format_report_table(report)
    json_text = format_report_json(report)
    parsed = parse_json_report(json_text)

    assert "mapping_test: ok=true degraded=false" in compact
    assert "name | level | required | enabled | message" in table
    assert parsed["ok"] is True
    assert format_report(report, "compact") == compact
    assert format_report(report, "table") == table


def test_key_value_formatters() -> None:
    block = format_key_value_block(
        "Robot Savo",
        {
            "ready": True,
            "rate_hz": 3.81234,
            "missing": None,
        },
    )

    status_block = format_mapping_status_dict(
        {
            "ready": True,
            "mode": "manual_mapping",
            "extra": {
                "map_name": "test_map",
            },
        }
    )

    assert "Robot Savo" in block
    assert "ready: true" in block
    assert "rate_hz: 3.812" in block
    assert "missing: none" in block
    assert "Robot Savo mapping status:" in status_block
    assert "manual_mapping" in status_block