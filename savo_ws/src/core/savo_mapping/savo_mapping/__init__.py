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
        TOPIC_MAP_QUALITY,
        TOPIC_EXPLORATION_STATUS,
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
    TOPIC_MAP_QUALITY = "/savo_mapping/map_quality"
    TOPIC_EXPLORATION_STATUS = "/savo_mapping/exploration_status"
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
            "map_quality": TOPIC_MAP_QUALITY,
            "exploration_status": TOPIC_EXPLORATION_STATUS,
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
    "TOPIC_MAP_QUALITY",
    "TOPIC_EXPLORATION_STATUS",
    "TOPIC_POINTCLOUD_STATUS",
    # metadata helper
    "get_package_info",
]