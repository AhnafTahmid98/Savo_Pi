#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Data models for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

from .apriltag_status import (
    AprilTagObservation,
    AprilTagStatus,
    apriltag_status_from_dict,
    make_apriltag_disabled_status,
    make_apriltag_status,
)
from .exploration_status import (
    ExplorationGoal,
    ExplorationState,
    ExplorationStatus,
    exploration_status_from_dict,
    make_complete_exploration_status,
    make_exploration_status,
    make_goal_selected_status,
    make_idle_exploration_status,
    make_navigating_status,
)
from .location_bridge_status import (
    LocationBridgeResult,
    LocationBridgeState,
    LocationBridgeStatus,
    location_bridge_status_from_dict,
    make_location_bridge_disabled_status,
    make_location_bridge_status,
    make_location_confirmed_status,
    make_location_saved_status,
)
from .semantic_landmark import (
    LandmarkPose,
    LandmarkState,
    SemanticLandmark,
    make_landmark_key,
    make_semantic_landmark,
    require_valid_landmark_state,
    sanitize_landmark_label,
    semantic_landmark_from_dict,
)
from .map_metadata import (
    MapMetadata,
    MapOrigin,
    make_map_metadata,
    make_saved_map_paths,
    map_metadata_from_dict,
    sanitize_map_name,
)
from .map_quality import (
    MapQuality,
    MapQualityThresholds,
    calculate_map_quality,
    make_empty_map_quality,
    map_quality_from_occupancy_values,
)
from .mapping_mode import (
    MappingMode,
    MappingModeProfile,
    get_mode_profile,
    is_valid_mapping_mode,
    list_mapping_modes,
    mode_allows_map_save,
    mode_allows_teleop,
    mode_uses_frontier_explorer,
    mode_uses_nav2,
    require_valid_mapping_mode,
)
from .mapping_status import (
    MappingStatus,
    make_autonomous_mapping_status,
    make_idle_status,
    make_manual_mapping_status,
    make_mapping_status,
    mapping_status_from_dict,
)
from .pointcloud_status import (
    PointcloudStatus,
    make_pointcloud_disabled_status,
    make_pointcloud_status,
    pointcloud_status_from_dict,
)
from .readiness_state import (
    MappingReadinessState,
    ReadinessCheck,
    build_autonomous_mapping_ready_state,
    build_default_not_ready_state,
    build_idle_readiness_state,
    build_manual_mapping_ready_state,
    build_readiness_state,
    make_check,
)


__all__ = [
    # Mapping mode
    "MappingMode",
    "MappingModeProfile",
    "get_mode_profile",
    "is_valid_mapping_mode",
    "list_mapping_modes",
    "mode_allows_map_save",
    "mode_allows_teleop",
    "mode_uses_frontier_explorer",
    "mode_uses_nav2",
    "require_valid_mapping_mode",
    # Readiness
    "ReadinessCheck",
    "MappingReadinessState",
    "make_check",
    "build_readiness_state",
    "build_idle_readiness_state",
    "build_default_not_ready_state",
    "build_manual_mapping_ready_state",
    "build_autonomous_mapping_ready_state",
    # Mapping status
    "MappingStatus",
    "make_mapping_status",
    "make_idle_status",
    "make_manual_mapping_status",
    "make_autonomous_mapping_status",
    "mapping_status_from_dict",
    # Map quality
    "MapQuality",
    "MapQualityThresholds",
    "make_empty_map_quality",
    "calculate_map_quality",
    "map_quality_from_occupancy_values",
    # Map metadata
    "MapOrigin",
    "MapMetadata",
    "sanitize_map_name",
    "make_map_metadata",
    "map_metadata_from_dict",
    "make_saved_map_paths",
    # Pointcloud
    "PointcloudStatus",
    "make_pointcloud_disabled_status",
    "make_pointcloud_status",
    "pointcloud_status_from_dict",
    # Exploration
    "ExplorationState",
    "ExplorationGoal",
    "ExplorationStatus",
    "make_exploration_status",
    "make_idle_exploration_status",
    "make_goal_selected_status",
    "make_navigating_status",
    "make_complete_exploration_status",
    "exploration_status_from_dict",
    # AprilTag
    "AprilTagObservation",
    "AprilTagStatus",
    "make_apriltag_disabled_status",
    "make_apriltag_status",
    "apriltag_status_from_dict",
    # Location bridge
    "LocationBridgeState",
    "LocationBridgeResult",
    "LocationBridgeStatus",
    "make_location_bridge_disabled_status",
    "make_location_bridge_status",
    "make_location_saved_status",
    "make_location_confirmed_status",
    "location_bridge_status_from_dict",
    # Semantic landmarks
    "LandmarkState",
    "LandmarkPose",
    "SemanticLandmark",
    "require_valid_landmark_state",
    "sanitize_landmark_label",
    "make_landmark_key",
    "make_semantic_landmark",
    "semantic_landmark_from_dict",
]
