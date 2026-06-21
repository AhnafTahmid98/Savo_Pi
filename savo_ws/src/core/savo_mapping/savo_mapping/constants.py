#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shared constants for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final


# =============================================================================
# Package identity
# =============================================================================
PACKAGE_NAME: Final[str] = "savo_mapping"
ROBOT_NAME: Final[str] = "Robot Savo"


# =============================================================================
# Node names
# =============================================================================
NODE_NAME_MAPPING_SUPERVISOR: Final[str] = "mapping_supervisor_node"
NODE_NAME_MAPPING_MODE_MANAGER: Final[str] = "mapping_mode_manager_node"
NODE_NAME_FRONTIER_EXPLORER: Final[str] = "frontier_explorer_node"
NODE_NAME_POINTCLOUD_MONITOR: Final[str] = "pointcloud_monitor_node"

NODE_NAME_SEMANTIC_LANDMARK_RECORDER: Final[str] = "semantic_landmark_recorder_node"
NODE_NAME_LOCATION_BRIDGE: Final[str] = "location_bridge_node"

NODE_NAME_MAPPING_DASHBOARD: Final[str] = "mapping_dashboard_node"
NODE_NAME_MAP_EVENT_LOGGER: Final[str] = "map_event_logger_node"


# =============================================================================
# Core topics
# =============================================================================
TOPIC_SCAN: Final[str] = "/scan"
TOPIC_ODOM: Final[str] = "/odometry/filtered"
TOPIC_MAP: Final[str] = "/map"
TOPIC_MAP_METADATA: Final[str] = "/map_metadata"

TOPIC_TF: Final[str] = "/tf"
TOPIC_TF_STATIC: Final[str] = "/tf_static"


# =============================================================================
# Edge / RealSense topics
# =============================================================================
TOPIC_REALSENSE_POINTS: Final[str] = "/savo_edge/realsense/points"
TOPIC_REALSENSE_STATUS: Final[str] = "/realsense/status"
TOPIC_DEPTH_FRONT: Final[str] = "/depth/min_front_m"


# =============================================================================
# Mapping outputs
# =============================================================================
TOPIC_MAPPING_READY: Final[str] = "/savo_mapping/ready"
TOPIC_MAPPING_STATUS: Final[str] = "/savo_mapping/status"
TOPIC_MAPPING_MODE: Final[str] = "/savo_mapping/mode"
TOPIC_MAPPING_WORKFLOW_PHASE: Final[str] = "/savo_mapping/workflow_phase"
TOPIC_SESSION_STATE: Final[str] = "/savo_mapping/session_state"
TOPIC_MAP_QUALITY: Final[str] = "/savo_mapping/map_quality"
TOPIC_EXPLORATION_STATUS: Final[str] = "/savo_mapping/exploration_status"
TOPIC_EXPLORATION_GOAL: Final[str] = "/savo_mapping/exploration/goal"
TOPIC_EXPLORATION_GOAL_POSE: Final[str] = "/savo_mapping/exploration/goal_pose"
TOPIC_SEMANTIC_STATUS: Final[str] = "/savo_mapping/semantic_status"
TOPIC_MAPPING_DASHBOARD: Final[str] = "/savo_mapping/dashboard"
TOPIC_MAPPING_DASHBOARD_TEXT: Final[str] = "/savo_mapping/dashboard_text"
TOPIC_FRONTIER_EXPLORER_STATUS: Final[str] = "/savo_mapping/frontier_explorer/status"
TOPIC_FRONTIER_EXPLORER_GOAL: Final[str] = "/savo_mapping/frontier_explorer/goal"
TOPIC_FRONTIER_EXPLORER_GOAL_POSE: Final[str] = (
    "/savo_mapping/frontier_explorer/goal_pose"
)
TOPIC_NEXT_GOAL: Final[str] = "/savo_mapping/next_goal"
TOPIC_POINTCLOUD_STATUS: Final[str] = "/savo_mapping/pointcloud_status"


# =============================================================================
# Future semantic mapping topics
# =============================================================================
TOPIC_APRILTAG_DETECTIONS: Final[str] = "/apriltag/detections"
TOPIC_APRILTAG_STATUS: Final[str] = "/savo_mapping/apriltag_status"
TOPIC_SEMANTIC_LANDMARKS: Final[str] = "/savo_mapping/semantic_landmarks"
TOPIC_LOCATION_BRIDGE_STATUS: Final[str] = "/savo_mapping/location_bridge/status"
TOPIC_KNOWN_LOCATIONS: Final[str] = "/savo_mapping/known_locations"
TOPIC_LOCATION_CONFIRMATION_COMMAND: Final[str] = (
    "/savo_mapping/location_confirmation_command"
)
TOPIC_LOCATION_CONFIRMATION_STATUS: Final[str] = (
    "/savo_mapping/location_confirmation/status"
)
TOPIC_LOCATION_CONFIRMATION_RESULT: Final[str] = (
    "/savo_mapping/location_confirmation/result"
)
TOPIC_APRILTAG_OBSERVATION: Final[str] = "/savo_mapping/apriltag_observation"
TOPIC_APRILTAG_MAPPER_STATUS: Final[str] = "/savo_mapping/apriltag_mapper/status"
TOPIC_APRILTAG_MAPPER_RESULT: Final[str] = "/savo_mapping/apriltag_mapper/result"
TOPIC_LOCATION_CANDIDATE: Final[str] = "/savo_mapping/location_candidate"
TOPIC_SEMANTIC_LANDMARK_RECORD: Final[str] = "/savo_mapping/semantic_landmark_record"
TOPIC_SEMANTIC_LANDMARK_RECORDER_STATUS: Final[str] = (
    "/savo_mapping/semantic_landmark_recorder/status"
)
TOPIC_SEMANTIC_LANDMARK_RECORDER_RESULT: Final[str] = (
    "/savo_mapping/semantic_landmark_recorder/result"
)
TOPIC_MAP_EVENT_LOGGER_STATUS: Final[str] = "/savo_mapping/map_event_logger/status"
TOPIC_MAP_EVENT_LOGGER_EVENT: Final[str] = "/savo_mapping/map_event_logger/event"


# =============================================================================
# Frame names
# =============================================================================
FRAME_MAP: Final[str] = "map"
FRAME_ODOM: Final[str] = "odom"
FRAME_BASE_LINK: Final[str] = "base_link"
FRAME_LASER: Final[str] = "laser"
FRAME_CAMERA_DEPTH: Final[str] = "camera_depth_optical_frame"


# =============================================================================
# Mapping modes
# =============================================================================
MODE_IDLE: Final[str] = "idle"
MODE_MANUAL_MAPPING: Final[str] = "manual_mapping"
MODE_AUTONOMOUS_MAPPING: Final[str] = "autonomous_mapping"
MODE_LOCALIZATION_CHECK: Final[str] = "localization_check"
MODE_MAP_SAVING: Final[str] = "map_saving"

VALID_MAPPING_MODES: Final[tuple[str, ...]] = (
    MODE_IDLE,
    MODE_MANUAL_MAPPING,
    MODE_AUTONOMOUS_MAPPING,
    MODE_LOCALIZATION_CHECK,
    MODE_MAP_SAVING,
)


# =============================================================================
# Default thresholds
# =============================================================================
DEFAULT_EXPECTED_SCAN_RATE_HZ: Final[float] = 3.5
DEFAULT_MIN_SCAN_RATE_HZ: Final[float] = 2.5

DEFAULT_SCAN_STALE_TIMEOUT_S: Final[float] = 0.75
DEFAULT_ODOM_STALE_TIMEOUT_S: Final[float] = 0.50
DEFAULT_TF_TIMEOUT_S: Final[float] = 0.50
DEFAULT_MAP_STALE_TIMEOUT_S: Final[float] = 2.0
DEFAULT_POINTCLOUD_STALE_TIMEOUT_S: Final[float] = 1.0

DEFAULT_PUBLISH_RATE_HZ: Final[float] = 5.0


# =============================================================================
# Map quality defaults
# =============================================================================
DEFAULT_MIN_KNOWN_RATIO: Final[float] = 0.15
DEFAULT_MAX_UNKNOWN_RATIO: Final[float] = 0.85
DEFAULT_MIN_OCCUPIED_CELLS: Final[int] = 50
DEFAULT_MIN_MAP_WIDTH_M: Final[float] = 2.0
DEFAULT_MIN_MAP_HEIGHT_M: Final[float] = 2.0


# =============================================================================
# Structured defaults
# =============================================================================
@dataclass(frozen=True)
class MappingDefaults:
    package_name: str = PACKAGE_NAME
    robot_name: str = ROBOT_NAME

    scan_topic: str = TOPIC_SCAN
    odom_topic: str = TOPIC_ODOM
    map_topic: str = TOPIC_MAP
    map_metadata_topic: str = TOPIC_MAP_METADATA
    realsense_points_topic: str = TOPIC_REALSENSE_POINTS

    map_frame: str = FRAME_MAP
    odom_frame: str = FRAME_ODOM
    base_frame: str = FRAME_BASE_LINK
    laser_frame: str = FRAME_LASER
    camera_depth_frame: str = FRAME_CAMERA_DEPTH

    expected_scan_rate_hz: float = DEFAULT_EXPECTED_SCAN_RATE_HZ
    min_scan_rate_hz: float = DEFAULT_MIN_SCAN_RATE_HZ

    scan_stale_timeout_s: float = DEFAULT_SCAN_STALE_TIMEOUT_S
    odom_stale_timeout_s: float = DEFAULT_ODOM_STALE_TIMEOUT_S
    tf_timeout_s: float = DEFAULT_TF_TIMEOUT_S
    map_stale_timeout_s: float = DEFAULT_MAP_STALE_TIMEOUT_S
    pointcloud_stale_timeout_s: float = DEFAULT_POINTCLOUD_STALE_TIMEOUT_S

    publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ

    def to_dict(self) -> dict:
        return {
            "package_name": self.package_name,
            "robot_name": self.robot_name,
            "topics": {
                "scan": self.scan_topic,
                "odom": self.odom_topic,
                "map": self.map_topic,
                "map_metadata": self.map_metadata_topic,
                "realsense_points": self.realsense_points_topic,
            },
            "frames": {
                "map": self.map_frame,
                "odom": self.odom_frame,
                "base": self.base_frame,
                "laser": self.laser_frame,
                "camera_depth": self.camera_depth_frame,
            },
            "rates": {
                "expected_scan_rate_hz": self.expected_scan_rate_hz,
                "min_scan_rate_hz": self.min_scan_rate_hz,
                "publish_rate_hz": self.publish_rate_hz,
            },
            "timeouts": {
                "scan_stale_timeout_s": self.scan_stale_timeout_s,
                "odom_stale_timeout_s": self.odom_stale_timeout_s,
                "tf_timeout_s": self.tf_timeout_s,
                "map_stale_timeout_s": self.map_stale_timeout_s,
                "pointcloud_stale_timeout_s": self.pointcloud_stale_timeout_s,
            },
        }


DEFAULTS: Final[MappingDefaults] = MappingDefaults()


def get_mapping_defaults() -> MappingDefaults:
    """Return immutable default mapping settings."""
    return DEFAULTS


__all__ = [
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "NODE_NAME_MAPPING_SUPERVISOR",
    "NODE_NAME_MAPPING_MODE_MANAGER",
    "NODE_NAME_FRONTIER_EXPLORER",
    "NODE_NAME_POINTCLOUD_MONITOR",
    "NODE_NAME_SEMANTIC_LANDMARK_RECORDER",
    "NODE_NAME_LOCATION_BRIDGE",
    "NODE_NAME_MAPPING_DASHBOARD",
    "NODE_NAME_MAP_EVENT_LOGGER",
    "TOPIC_SCAN",
    "TOPIC_ODOM",
    "TOPIC_MAP",
    "TOPIC_MAP_METADATA",
    "TOPIC_TF",
    "TOPIC_TF_STATIC",
    "TOPIC_REALSENSE_POINTS",
    "TOPIC_REALSENSE_STATUS",
    "TOPIC_DEPTH_FRONT",
    "TOPIC_MAPPING_READY",
    "TOPIC_MAPPING_STATUS",
    "TOPIC_MAPPING_MODE",
    "TOPIC_MAPPING_WORKFLOW_PHASE",
    "TOPIC_SESSION_STATE",
    "TOPIC_MAP_QUALITY",
    "TOPIC_EXPLORATION_STATUS",
    "TOPIC_EXPLORATION_GOAL",
    "TOPIC_EXPLORATION_GOAL_POSE",
    "TOPIC_SEMANTIC_STATUS",
    "TOPIC_MAPPING_DASHBOARD",
    "TOPIC_MAPPING_DASHBOARD_TEXT",
    "TOPIC_FRONTIER_EXPLORER_STATUS",
    "TOPIC_FRONTIER_EXPLORER_GOAL",
    "TOPIC_FRONTIER_EXPLORER_GOAL_POSE",
    "TOPIC_NEXT_GOAL",
    "TOPIC_POINTCLOUD_STATUS",
    "TOPIC_APRILTAG_DETECTIONS",
    "TOPIC_APRILTAG_STATUS",
    "TOPIC_SEMANTIC_LANDMARKS",
    "TOPIC_LOCATION_BRIDGE_STATUS",
    "TOPIC_KNOWN_LOCATIONS",
    "TOPIC_LOCATION_CONFIRMATION_COMMAND",
    "TOPIC_LOCATION_CONFIRMATION_STATUS",
    "TOPIC_LOCATION_CONFIRMATION_RESULT",
    "TOPIC_APRILTAG_OBSERVATION",
    "TOPIC_APRILTAG_MAPPER_STATUS",
    "TOPIC_APRILTAG_MAPPER_RESULT",
    "TOPIC_LOCATION_CANDIDATE",
    "TOPIC_SEMANTIC_LANDMARK_RECORD",
    "TOPIC_SEMANTIC_LANDMARK_RECORDER_STATUS",
    "TOPIC_SEMANTIC_LANDMARK_RECORDER_RESULT",
    "TOPIC_MAP_EVENT_LOGGER_STATUS",
    "TOPIC_MAP_EVENT_LOGGER_EVENT",
    "FRAME_MAP",
    "FRAME_ODOM",
    "FRAME_BASE_LINK",
    "FRAME_LASER",
    "FRAME_CAMERA_DEPTH",
    "MODE_IDLE",
    "MODE_MANUAL_MAPPING",
    "MODE_AUTONOMOUS_MAPPING",
    "MODE_LOCALIZATION_CHECK",
    "MODE_MAP_SAVING",
    "VALID_MAPPING_MODES",
    "DEFAULT_EXPECTED_SCAN_RATE_HZ",
    "DEFAULT_MIN_SCAN_RATE_HZ",
    "DEFAULT_SCAN_STALE_TIMEOUT_S",
    "DEFAULT_ODOM_STALE_TIMEOUT_S",
    "DEFAULT_TF_TIMEOUT_S",
    "DEFAULT_MAP_STALE_TIMEOUT_S",
    "DEFAULT_POINTCLOUD_STALE_TIMEOUT_S",
    "DEFAULT_PUBLISH_RATE_HZ",
    "DEFAULT_MIN_KNOWN_RATIO",
    "DEFAULT_MAX_UNKNOWN_RATIO",
    "DEFAULT_MIN_OCCUPIED_CELLS",
    "DEFAULT_MIN_MAP_WIDTH_M",
    "DEFAULT_MIN_MAP_HEIGHT_M",
    "MappingDefaults",
    "DEFAULTS",
    "get_mapping_defaults",
]
