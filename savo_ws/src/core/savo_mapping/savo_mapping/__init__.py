# -*- coding: utf-8 -*-
"""Package root. Exports version, constants, and top-level helpers. No ROS imports."""

from __future__ import annotations

try:
    from .version import (
        __version__,
        VERSION,
        get_version,
        get_package_version_info,
    )
except Exception:
    __version__ = "0.0.0"
    VERSION = __version__

    def get_version() -> str:
        return __version__

    def get_package_version_info() -> dict:
        return {
            "package": "savo_mapping",
            "version": __version__,
            "version_tuple": (0, 0, 0),
        }


try:
    from .constants import (
        DEFAULTS,
        MappingDefaults,
        get_mapping_defaults,
        # Identity
        PACKAGE_NAME,
        ROBOT_NAME,
        NODE_NAME_MAPPING_SUPERVISOR,
        NODE_NAME_MAPPING_MODE_MANAGER,
        NODE_NAME_FRONTIER_EXPLORER,
        NODE_NAME_POINTCLOUD_MONITOR,
        # Topics
        TOPIC_SCAN,
        TOPIC_ODOM,
        TOPIC_MAP,
        TOPIC_MAP_METADATA,
        TOPIC_REALSENSE_POINTS,
        TOPIC_MAPPING_READY,
        TOPIC_MAPPING_STATUS,
        TOPIC_MAPPING_MODE,
        TOPIC_MAPPING_WORKFLOW_PHASE,
        TOPIC_MAP_QUALITY,
        TOPIC_EXPLORATION_STATUS,
        TOPIC_SEMANTIC_STATUS,
        TOPIC_MAPPING_DASHBOARD,
        TOPIC_MAPPING_DASHBOARD_TEXT,
        TOPIC_FRONTIER_EXPLORER_STATUS,
        TOPIC_FRONTIER_EXPLORER_GOAL,
        TOPIC_FRONTIER_EXPLORER_GOAL_POSE,
        TOPIC_LOCATION_BRIDGE_STATUS,
        TOPIC_KNOWN_LOCATIONS,
        TOPIC_LOCATION_CONFIRMATION_COMMAND,
        TOPIC_LOCATION_CONFIRMATION_STATUS,
        TOPIC_LOCATION_CONFIRMATION_RESULT,
        TOPIC_APRILTAG_OBSERVATION,
        TOPIC_APRILTAG_MAPPER_STATUS,
        TOPIC_APRILTAG_MAPPER_RESULT,
        TOPIC_LOCATION_CANDIDATE,
        TOPIC_POINTCLOUD_STATUS,
    )
except Exception:
    DEFAULTS = None
    MappingDefaults = None

    def get_mapping_defaults():
        return None

    PACKAGE_NAME = "savo_mapping"
    ROBOT_NAME = "Robot Savo"

    NODE_NAME_MAPPING_SUPERVISOR = "mapping_supervisor_node"
    NODE_NAME_MAPPING_MODE_MANAGER = "mapping_mode_manager_node"
    NODE_NAME_FRONTIER_EXPLORER = "frontier_explorer_node"
    NODE_NAME_POINTCLOUD_MONITOR = "pointcloud_monitor_node"

    TOPIC_SCAN = "/scan"
    TOPIC_ODOM = "/odometry/filtered"
    TOPIC_MAP = "/map"
    TOPIC_MAP_METADATA = "/map_metadata"
    TOPIC_REALSENSE_POINTS = "/savo_edge/realsense/points"

    TOPIC_MAPPING_READY = "/savo_mapping/ready"
    TOPIC_MAPPING_STATUS = "/savo_mapping/status"
    TOPIC_MAPPING_MODE = "/savo_mapping/mode"
    TOPIC_MAPPING_WORKFLOW_PHASE = "/savo_mapping/workflow_phase"
    TOPIC_MAP_QUALITY = "/savo_mapping/map_quality"
    TOPIC_EXPLORATION_STATUS = "/savo_mapping/exploration_status"
    TOPIC_SEMANTIC_STATUS = "/savo_mapping/semantic_status"
    TOPIC_MAPPING_DASHBOARD = "/savo_mapping/dashboard"
    TOPIC_MAPPING_DASHBOARD_TEXT = "/savo_mapping/dashboard_text"
    TOPIC_FRONTIER_EXPLORER_STATUS = "/savo_mapping/frontier_explorer/status"
    TOPIC_FRONTIER_EXPLORER_GOAL = "/savo_mapping/frontier_explorer/goal"
    TOPIC_FRONTIER_EXPLORER_GOAL_POSE = "/savo_mapping/frontier_explorer/goal_pose"
    TOPIC_LOCATION_BRIDGE_STATUS = "/savo_mapping/location_bridge/status"
    TOPIC_KNOWN_LOCATIONS = "/savo_mapping/known_locations"
    TOPIC_LOCATION_CONFIRMATION_COMMAND = "/savo_mapping/location_confirmation_command"
    TOPIC_LOCATION_CONFIRMATION_STATUS = "/savo_mapping/location_confirmation/status"
    TOPIC_LOCATION_CONFIRMATION_RESULT = "/savo_mapping/location_confirmation/result"
    TOPIC_APRILTAG_OBSERVATION = "/savo_mapping/apriltag_observation"
    TOPIC_APRILTAG_MAPPER_STATUS = "/savo_mapping/apriltag_mapper/status"
    TOPIC_APRILTAG_MAPPER_RESULT = "/savo_mapping/apriltag_mapper/result"
    TOPIC_LOCATION_CANDIDATE = "/savo_mapping/location_candidate"
    TOPIC_POINTCLOUD_STATUS = "/savo_mapping/pointcloud_status"


def get_package_info() -> dict:
    """Lightweight package metadata — safe to call without ROS."""
    return {
        "package": PACKAGE_NAME,
        "robot_name": ROBOT_NAME,
        "version": get_version(),
        "node_defaults": {
            "mapping_supervisor": NODE_NAME_MAPPING_SUPERVISOR,
            "mapping_mode_manager": NODE_NAME_MAPPING_MODE_MANAGER,
            "frontier_explorer": NODE_NAME_FRONTIER_EXPLORER,
            "pointcloud_monitor": NODE_NAME_POINTCLOUD_MONITOR,
        },
        "topics": {
            "scan": TOPIC_SCAN,
            "odom": TOPIC_ODOM,
            "map": TOPIC_MAP,
            "map_metadata": TOPIC_MAP_METADATA,
            "realsense_points": TOPIC_REALSENSE_POINTS,
            "mapping_ready": TOPIC_MAPPING_READY,
            "mapping_status": TOPIC_MAPPING_STATUS,
            "mapping_mode": TOPIC_MAPPING_MODE,
            "mapping_workflow_phase": TOPIC_MAPPING_WORKFLOW_PHASE,
            "map_quality": TOPIC_MAP_QUALITY,
            "exploration_status": TOPIC_EXPLORATION_STATUS,
            "semantic_status": TOPIC_SEMANTIC_STATUS,
            "mapping_dashboard": TOPIC_MAPPING_DASHBOARD,
            "mapping_dashboard_text": TOPIC_MAPPING_DASHBOARD_TEXT,
            "frontier_explorer_status": TOPIC_FRONTIER_EXPLORER_STATUS,
            "frontier_explorer_goal": TOPIC_FRONTIER_EXPLORER_GOAL,
            "frontier_explorer_goal_pose": TOPIC_FRONTIER_EXPLORER_GOAL_POSE,
            "location_bridge_status": TOPIC_LOCATION_BRIDGE_STATUS,
            "known_locations": TOPIC_KNOWN_LOCATIONS,
            "location_confirmation_command": TOPIC_LOCATION_CONFIRMATION_COMMAND,
            "location_confirmation_status": TOPIC_LOCATION_CONFIRMATION_STATUS,
            "location_confirmation_result": TOPIC_LOCATION_CONFIRMATION_RESULT,
            "apriltag_observation": TOPIC_APRILTAG_OBSERVATION,
            "apriltag_mapper_status": TOPIC_APRILTAG_MAPPER_STATUS,
            "apriltag_mapper_result": TOPIC_APRILTAG_MAPPER_RESULT,
            "location_candidate": TOPIC_LOCATION_CANDIDATE,
            "pointcloud_status": TOPIC_POINTCLOUD_STATUS,
        },
        "has_structured_defaults": DEFAULTS is not None,
    }


__all__ = [
    # version
    "__version__",
    "VERSION",
    "get_version",
    "get_package_version_info",
    # constants/defaults
    "DEFAULTS",
    "MappingDefaults",
    "get_mapping_defaults",
    "PACKAGE_NAME",
    "ROBOT_NAME",
    "NODE_NAME_MAPPING_SUPERVISOR",
    "NODE_NAME_MAPPING_MODE_MANAGER",
    "NODE_NAME_FRONTIER_EXPLORER",
    "NODE_NAME_POINTCLOUD_MONITOR",
    "TOPIC_SCAN",
    "TOPIC_ODOM",
    "TOPIC_MAP",
    "TOPIC_MAP_METADATA",
    "TOPIC_REALSENSE_POINTS",
    "TOPIC_MAPPING_READY",
    "TOPIC_MAPPING_STATUS",
    "TOPIC_MAPPING_MODE",
    "TOPIC_MAPPING_WORKFLOW_PHASE",
    "TOPIC_MAP_QUALITY",
    "TOPIC_EXPLORATION_STATUS",
    "TOPIC_SEMANTIC_STATUS",
    "TOPIC_MAPPING_DASHBOARD",
    "TOPIC_MAPPING_DASHBOARD_TEXT",
    "TOPIC_FRONTIER_EXPLORER_STATUS",
    "TOPIC_FRONTIER_EXPLORER_GOAL",
    "TOPIC_FRONTIER_EXPLORER_GOAL_POSE",
    "TOPIC_LOCATION_BRIDGE_STATUS",
    "TOPIC_KNOWN_LOCATIONS",
    "TOPIC_LOCATION_CONFIRMATION_COMMAND",
    "TOPIC_LOCATION_CONFIRMATION_STATUS",
    "TOPIC_LOCATION_CONFIRMATION_RESULT",
    "TOPIC_APRILTAG_OBSERVATION",
    "TOPIC_APRILTAG_MAPPER_STATUS",
    "TOPIC_APRILTAG_MAPPER_RESULT",
    "TOPIC_LOCATION_CANDIDATE",
    "TOPIC_POINTCLOUD_STATUS",
    # metadata helper
    "get_package_info",
]
