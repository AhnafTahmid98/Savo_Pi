#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Diagnostic checks for Robot Savo mapping."""

from __future__ import annotations

from .apriltag_ready_check import (
    AprilTagReadyResult,
    apriltag_result_to_diagnostic,
    evaluate_apriltag_observations,
    evaluate_apriltag_ready,
)
from .location_bridge_check import (
    LocationBridgeReadyResult,
    evaluate_location_bridge_from_status,
    evaluate_location_bridge_ready,
    location_bridge_result_to_diagnostic,
)
from .map_file_check import (
    MapFileResult,
    evaluate_map_files,
    evaluate_map_files_from_paths,
    map_file_result_to_diagnostic,
)
from .map_quality_check import (
    MapQualityCheckResult,
    evaluate_map_quality,
    evaluate_map_quality_from_result,
    evaluate_occupancy_values_quality,
    map_quality_result_to_diagnostic,
)
from .map_topic_check import (
    MapTopicResult,
    evaluate_map_msg,
    evaluate_map_summary,
    evaluate_map_topic_ready,
    map_topic_result_to_diagnostic,
)
from .nav2_mapping_check import (
    Nav2MappingResult,
    evaluate_nav2_mapping_from_nodes,
    evaluate_nav2_mapping_ready,
    nav2_mapping_result_to_diagnostic,
)
from .odom_ready_check import (
    OdomReadyResult,
    evaluate_odom_msg,
    evaluate_odom_ready,
    odom_result_to_diagnostic,
)
from .pointcloud_ready_check import (
    PointcloudReadyResult,
    evaluate_pointcloud_msg,
    evaluate_pointcloud_ready,
    evaluate_pointcloud_summary,
    pointcloud_result_to_diagnostic,
)
from .report_formatter import (
    bool_icon,
    format_bool,
    format_key_value_block,
    format_mapping_status_dict,
    format_report,
    format_report_compact,
    format_report_json,
    format_report_summary,
    format_report_table,
    format_value,
    parse_json_report,
)
from .scan_ready_check import (
    ScanReadyResult,
    evaluate_scan_msg,
    evaluate_scan_ready,
    evaluate_scan_summary,
    scan_result_to_diagnostic,
)
from .slam_toolbox_check import (
    SlamToolboxResult,
    evaluate_slam_toolbox_from_readiness,
    evaluate_slam_toolbox_ready,
    slam_toolbox_result_to_diagnostic,
)
from .tf_ready_check import (
    TfEdgeResult,
    TfReadyResult,
    evaluate_default_tf_ready,
    evaluate_tf_ready,
    make_tf_edge_result,
    tf_result_to_diagnostic,
)
from .voxel_layer_check import (
    VoxelLayerResult,
    evaluate_voxel_layer_from_inputs,
    evaluate_voxel_layer_ready,
    voxel_layer_result_to_diagnostic,
)


__all__ = [
    # Scan
    "ScanReadyResult",
    "evaluate_scan_ready",
    "evaluate_scan_summary",
    "evaluate_scan_msg",
    "scan_result_to_diagnostic",
    # Odom
    "OdomReadyResult",
    "evaluate_odom_ready",
    "evaluate_odom_msg",
    "odom_result_to_diagnostic",
    # TF
    "TfEdgeResult",
    "TfReadyResult",
    "make_tf_edge_result",
    "evaluate_tf_ready",
    "evaluate_default_tf_ready",
    "tf_result_to_diagnostic",
    # Map topic
    "MapTopicResult",
    "evaluate_map_topic_ready",
    "evaluate_map_summary",
    "evaluate_map_msg",
    "map_topic_result_to_diagnostic",
    # slam_toolbox
    "SlamToolboxResult",
    "evaluate_slam_toolbox_ready",
    "evaluate_slam_toolbox_from_readiness",
    "slam_toolbox_result_to_diagnostic",
    # Pointcloud
    "PointcloudReadyResult",
    "evaluate_pointcloud_ready",
    "evaluate_pointcloud_summary",
    "evaluate_pointcloud_msg",
    "pointcloud_result_to_diagnostic",
    # Nav2 mapping
    "Nav2MappingResult",
    "evaluate_nav2_mapping_ready",
    "evaluate_nav2_mapping_from_nodes",
    "nav2_mapping_result_to_diagnostic",
    # Voxel layer
    "VoxelLayerResult",
    "evaluate_voxel_layer_ready",
    "evaluate_voxel_layer_from_inputs",
    "voxel_layer_result_to_diagnostic",
    # AprilTag
    "AprilTagReadyResult",
    "evaluate_apriltag_ready",
    "evaluate_apriltag_observations",
    "apriltag_result_to_diagnostic",
    # Location bridge
    "LocationBridgeReadyResult",
    "evaluate_location_bridge_ready",
    "evaluate_location_bridge_from_status",
    "location_bridge_result_to_diagnostic",
    # Map files
    "MapFileResult",
    "evaluate_map_files",
    "evaluate_map_files_from_paths",
    "map_file_result_to_diagnostic",
    # Map quality
    "MapQualityCheckResult",
    "evaluate_map_quality",
    "evaluate_occupancy_values_quality",
    "evaluate_map_quality_from_result",
    "map_quality_result_to_diagnostic",
    # Formatting
    "bool_icon",
    "format_bool",
    "format_value",
    "format_report_summary",
    "format_report_table",
    "format_report_compact",
    "format_report_json",
    "format_mapping_status_dict",
    "format_key_value_block",
    "format_report",
    "parse_json_report",
]